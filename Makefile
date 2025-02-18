#
# Example:
#  KDIR=~/projects/linux
#  ARCH=arm64

KDIR ?= /lib/modules/`uname -r`/build
obj-m := ms7210.o

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

