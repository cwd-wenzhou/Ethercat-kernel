ifneq ($(KERNELRELEASE),)
obj-m :=demo_chardev.o $(LIBS)
drv-objs := demo_chardev.o 
else
KERNEL_DIR :=/lib/modules/$(shell uname -r)/build

PWD :=$(shell pwd)
all: modules

modules:
	make -C $(KERNEL_DIR) M=$(PWD) modules
clean:
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers *.order
endif