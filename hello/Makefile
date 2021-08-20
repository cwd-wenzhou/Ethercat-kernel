obj-m := hello.o
KERNEL_DIR := /usr/src/linux-headers-5.4.0-80-generic
PWD :=$(shell pwd)
all: modules

modules:
	make -C $(KERNEL_DIR) M=$(PWD) modules
clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean