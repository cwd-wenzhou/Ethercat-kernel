ifneq ($(KERNELRELEASE),)
obj-m :=hello.o
ccflags-y :=  -std=gnu99 -Wno-declaration-after-statement
else
KERNEL_DIR :=/lib/modules/$(shell uname -r)/build

PWD :=$(shell pwd)
all: modules

modules:
	make -C $(KERNEL_DIR) M=$(PWD) modules
clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean
endif