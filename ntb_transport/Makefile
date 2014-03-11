KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

obj-m += ntb.o

ntb-objs := ntb_hw.o ntb_transport.o

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) $(ARCH) clean
