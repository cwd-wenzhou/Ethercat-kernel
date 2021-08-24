ifeq ($(KERNELRELEASE),)
KERNEL_DIR :=/lib/modules/$(shell uname -r)/build
PWD :=$(shell pwd)
modules:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules
modules_install:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install
clean:
	rm -rf moudels.* .o *~ core .depend ..cmd .ko *.mod.c .tmp_versions Module
.PHONY:
	modules modules_install clean
else
obj-m := chardevice.o
#生成的目标文件名
#module_name –objs = file1.o file2.o file3.o
endif