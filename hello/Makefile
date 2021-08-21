LIBS = /opt/etherlab/lib/libethercat.a -lrt -lpthread -lm
ifneq ($(KERNELRELEASE),)
obj-m :=hello.o $(LIBS)
drv-objs := hello.o  /opt/etherlab/lib/libethercat.a
else
KERNEL_DIR :=/lib/modules/$(shell uname -r)/build

PWD :=$(shell pwd)
all: modules

modules:
	make -C $(KERNEL_DIR) M=$(PWD) modules
clean:
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers *.order
endif