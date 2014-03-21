/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 *
 *   GPL LICENSE SUMMARY
 *
 *   Copyright(c) 2012 Intel Corporation. All rights reserved.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 *
 *   BSD LICENSE
 *
 *   Copyright(c) 2012 Intel Corporation. All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copy
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Intel PCIe NTB test Linux driver
 *
 * Contact Information:
 * Jon Mason <jon.mason@intel.com>
 */
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/ntb.h>

#define NTB_TEST_VER	"0.1"

MODULE_DESCRIPTION(KBUILD_MODNAME);
MODULE_VERSION(NTB_TEST_VER);
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Intel Corporation");

static bool sink;
module_param(sink, bool, 0644);
MODULE_PARM_DESC(sink, "Do not transmit any data");

static int num_qps = 1;
module_param(num_qps, uint, 0644);
MODULE_PARM_DESC(num_qps, "Number of NTB transport connections");

struct ntb_test_qp {
	struct ntb_transport_qp *qp;
	struct task_struct *kthread;
	u32 mtu;
	bool link;

	/* stats */
	__u64 tx_pkts;
	__u64 tx_bytes;
	__u64 tx_full;
	__u64 tx_nomem;
};

struct ntb_test {
	struct list_head list;
	struct pci_dev *pdev;
	struct ntb_test_qp *test_qp;
};

static LIST_HEAD(dev_list);

static void ntb_test_event_handler(void *data, int status)
{
	struct ntb_test_qp *test_qp = data;

	if (ntb_transport_link_query(test_qp->qp))
		test_qp->link = true;
	else
		test_qp->link = false;
}

static void ntb_test_rx_handler(struct ntb_transport_qp *qp, void *qp_data,
				void *data, int len)
{
	struct ntb_test_qp *test_qp = qp_data;
	int rc;

	rc = ntb_transport_rx_enqueue(qp, data, data, test_qp->mtu);
	if (rc) {
		pr_err("Unable to re-enqueue rx buf\n");
		kfree(data);
	}
}

static void ntb_test_tx_handler(struct ntb_transport_qp *qp, void *qp_data,
				void *data, int len)
{
	if (len > 0) {
		struct ntb_test_qp *test_qp = qp_data;

		test_qp->tx_bytes += len;
		test_qp->tx_pkts++;
	}
}

static int ntb_test_xmit(void *data)
{
	struct ntb_test_qp *test_qp = data;
	unsigned long long secs, mb;
	ktime_t started_at = ktime_get();
	ktime_t stopped_at;
	void *buf;

	buf = kmalloc(test_qp->mtu, GFP_KERNEL);
	if (!buf) {
		test_qp->tx_nomem++;
		goto out;
	}

	while (!kthread_should_stop()) {
		int rc;

		if (!test_qp->link) {
			msleep(100);
			continue;
		}

		rc = ntb_transport_tx_enqueue(test_qp->qp, buf, buf,
					      test_qp->mtu);
		if (rc)
			test_qp->tx_full++;

		schedule();
	}
	kfree(buf);
out:
	stopped_at = ktime_get();

	secs = ktime_to_ms(ktime_sub(stopped_at, started_at));
	do_div(secs, 1000);
	pr_info("%llu bytes sent over %lld secs\n", test_qp->tx_bytes, secs);
	mb = test_qp->tx_bytes;
	do_div(mb, secs);
	do_div(mb, 1000000);
	pr_info("%llu MBps on qp%d\n", mb, ntb_transport_qp_num(test_qp->qp));
	pr_info("%llu Mbps on qp%d\n", mb * 8,
		ntb_transport_qp_num(test_qp->qp));
	pr_info("tx_pkts %llu\ntx_full %llu\ntx_nomem %llu\n", test_qp->tx_pkts,
		test_qp->tx_full, test_qp->tx_nomem);

	return 0;
}

static const struct ntb_queue_handlers ntb_test_handlers = {
	.tx_handler = ntb_test_tx_handler,
	.rx_handler = ntb_test_rx_handler,
	.event_handler = ntb_test_event_handler,
};

static int ntb_test_probe(struct pci_dev *pdev)
{
	struct ntb_test_qp *test_qp;
	struct ntb_test *dev;
	int rc, len, qp_num, node;
	void *buf;

	dev = kmalloc(sizeof(struct ntb_test), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->pdev = pdev;

	dev->test_qp = kcalloc(num_qps, sizeof(struct ntb_test_qp), GFP_KERNEL);
	if (!dev->test_qp) {
		rc = -ENOMEM;
		goto err;
	}

	for (qp_num = 0; qp_num < num_qps; qp_num++) {
		test_qp = &dev->test_qp[qp_num];

		test_qp->qp = ntb_transport_create_queue(test_qp, pdev,
							 &ntb_test_handlers);
		if (!test_qp->qp) {
			rc = -EIO;
			goto err1;
		}

		test_qp->mtu = ntb_transport_max_size(test_qp->qp);

		do {
			buf = kmalloc(test_qp->mtu, GFP_KERNEL);
			if (!buf) {
				rc = -ENOMEM;
				goto err1;
			}

			rc = ntb_transport_rx_enqueue(test_qp->qp, buf, buf,
						      test_qp->mtu);
			if (rc == -EINVAL)
				goto err1;
		} while (rc != -EBUSY);

		ntb_transport_link_up(test_qp->qp);

		if (sink)
			continue;

		node = pcibus_to_node(pdev->bus);

		test_qp->kthread = kthread_create_on_node(ntb_test_xmit,
							  test_qp, node,
							  "ntb_test%d", qp_num);
		rc = IS_ERR(test_qp->kthread);
		if (rc)
			goto err1;

		/* FIXME - The CPU being bound to might not be on this node */
		kthread_bind(test_qp->kthread,
			     ntb_transport_qp_num(test_qp->qp));
		pr_info("Starting NTB test of qp%d on core %d\n",
			ntb_transport_qp_num(test_qp->qp),
			ntb_transport_qp_num(test_qp->qp));
		wake_up_process(test_qp->kthread);
	}

	list_add(&dev->list, &dev_list);
	return 0;

err1:
	for (; qp_num >= 0; qp_num--) {
		test_qp = &dev->test_qp[qp_num];

		if (test_qp->kthread)
			kthread_stop(test_qp->kthread);

		ntb_transport_link_down(test_qp->qp);

		while ((buf = ntb_transport_rx_remove(test_qp->qp, &len)))
			kfree(buf);

		ntb_transport_free_queue(test_qp->qp);
	}
	kfree(dev->test_qp);
err:
	kfree(dev);
	return rc;
}

static void ntb_test_remove(struct pci_dev *pdev)
{
	struct ntb_test_qp *test_qp;
	struct ntb_test *dev;
	int i, len;

	list_for_each_entry(dev, &dev_list, list) {
		if (dev->pdev == pdev)
			break;
	}
	if (dev == NULL)
		return;

	list_del(&dev->list);

	for (i = 0; i < num_qps; i++) {
		void *buf;

		test_qp = &dev->test_qp[i];

		pr_info("Ending NTB test of qp%d on core %d\n",
			ntb_transport_qp_num(test_qp->qp),
			ntb_transport_qp_num(test_qp->qp));
		if (test_qp->kthread)
			kthread_stop(test_qp->kthread);
		ntb_transport_link_down(test_qp->qp);

		while ((buf = ntb_transport_rx_remove(test_qp->qp, &len)))
			kfree(buf);

		ntb_transport_free_queue(test_qp->qp);
	}
	kfree(dev->test_qp);
	kfree(dev);
}

static struct ntb_client ntb_test_client = {
	.driver.name = KBUILD_MODNAME,
	.driver.owner = THIS_MODULE,
	.probe = ntb_test_probe,
	.remove = ntb_test_remove,
};

static int __init ntb_test_init_module(void)
{
	int rc;

	rc = ntb_register_client_dev(KBUILD_MODNAME);
	if (rc)
		return rc;
	return ntb_register_client(&ntb_test_client);
}
module_init(ntb_test_init_module);

static void __exit ntb_test_exit_module(void)
{
	ntb_unregister_client(&ntb_test_client);
	ntb_unregister_client_dev(KBUILD_MODNAME);
}
module_exit(ntb_test_exit_module);
