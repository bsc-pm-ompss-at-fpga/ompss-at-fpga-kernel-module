obj-m += ompss_fpga.o
ompss_fpga-objs += main.o hwcounter.o xdma.o bitinfo.o hwruntime.o xdma_mem.o

ifeq ($(KDIR),)
	KDIR := /lib/modules/$(shell uname -r)/build
endif

all: ompss_fpga.ko

ompss_fpga.ko: ompss_fpga.h ompss_fpga_common.h main.c hwcounter.c xdma.c \
bitinfo.c hwruntime.c
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean

install: ompss_fpga.ko
	@Copying ompss_fpga rules to /etc/udev/rules.d
	sudo cp 80-ompss_fpga.rules /etc/udev/rules.d/
	@echo Copying ompss_fpga module to /lib/modules/$$(uname -r)/extra
	if [ ! -d /lib/modules/$$(uname -r)/extra ] ; then \
	   sudo mkdir /lib/modules/$$(uname -r)/extra ; \
	fi
	sudo cp ompss_fpga.ko /lib/modules/$$(uname -r)/extra
	@echo Determining new module dependences
	sudo depmod -a
