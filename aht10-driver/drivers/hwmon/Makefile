PWD := $(shell pwd)
obj-m += aht10.o

all:
	make ARCH=arm CROSS_COMPILE=$(CROSS) -C $(KERNEL) SUBDIRS=$(PWD) modules

clean:
	make -C $(KERNEL) SUBDIRS=$(PWD) clean

custom:
	./local.sh

custom-clean:
	./local.sh clean
