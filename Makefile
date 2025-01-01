obj-m += uart_driver.o

CROSS=/home/nambcn/work/beagle_bone_black/kernelbuildscripts/dl/gcc-8.5.0-nolibc/arm-linux-gnueabi/bin/arm-linux-gnueabi-

all:
	make ARCH=arm CROSS_COMPILE=$(CROSS) -C /home/nambcn/work/beagle_bone_black/kernelbuildscripts/KERNEL M=$(PWD) modules

clean:
	make -C /home/nambcn/work/beagle_bone_black/kernelbuildscripts/KERNEL M=$(PWD) clean

