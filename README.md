ntb-module
==========

NTB transport driver and net device as separate modules. 
Forked from [Jon Mason's repo](https://github.com/jonmason/ntb), `ntb-2.6.32` branch.

Changes from original:

* Separate Makefiles
* `ntb.h` copied from `include/linux/ntb.h` to `ntb_transport/ntb.h`
* Include paths changed to <ntb_transport/ntb.h>
