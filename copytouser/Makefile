ifneq ($(KERNELRELEASE),)
obj-m :=copytouser.o $(LIBS)
drv-objs := copytouser.o 
else
KERNEL_DIR :=/lib/modules/$(shell uname -r)/build

PWD :=$(shell pwd)
all: modules

modules:
	make -C $(KERNEL_DIR) M=$(PWD) modules
clean:
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers *.order
endif