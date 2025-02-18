#
# Example:
#  KERN_SRC=~/projects/linux
#  ARCH=arm64

obj-m := ms7210.o

all:
	$(MAKE) -C $(KERN_SRC) M=$(PWD) modules
clean:
	$(MAKE) -C $(KERN_SRC) M=$(PWD) clean

install:
	$(MAKE) -C $(KERN_SRC) M=$(PWD) modules_install

