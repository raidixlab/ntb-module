KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)
EXTRA_CFLAGS= -I$(TOP_DIR)

obj-m += ntb_netdev.o

default:
	cat $(TOP_DIR)/ntb_transport/Module.symvers >> Module.symvers
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) $(ARCH) clean

