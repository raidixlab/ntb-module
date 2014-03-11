TOP_DIR=$(shell pwd)

all: ntb_transport ntb_netdev

ntb_transport:
	cd ntb_transport && make TOP_DIR=$(TOP_DIR)

ntb_netdev: ntb_transport
	cd ntb_netdev && make TOP_DIR=$(TOP_DIR)

clean:
	cd ntb_transport && make clean
	cd ntb_netdev && make clean

.PHONY: ntb_transport ntb_netdev
