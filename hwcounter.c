/*--------------------------------------------------------------------
  OmpSs@FPGA Zynq Kernel Module
  Copyright (C) 2019-2020 Barcelona Supercomputing Center
                          Centro Nacional de Supercomputacion

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along
  with this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
--------------------------------------------------------------------*/

#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/io.h>

#include "ompss_fpga_common.h"

#define HWCOUNTER_MODULE_NAME       "ompss_fpga_hwcounter"
#define HWCOUNTER_DEV_NAME          "hwcounter"
#define HWCOUNTER_PHANDLE_NAME      "hwcounter"

static dev_t hwcounter_devt;
static struct class *hwcounter_cl;
static struct cdev hwcounter_cdev;
static struct device *hwcounter_dev;

static int hwcounter_opens_cnt;             // Opens counter of device
static int hwcounter_major;                 // Device major number
u32 __iomem * hwcounter_io_addr;            // Virt. kernel address of hwcounter BRAM
static unsigned long hwcounter_phy_addr;    // Phy. address of hwcoutner BRAM

static int hwcounter_open(struct inode *i, struct file *f) {
	return generic_open(&hwcounter_opens_cnt, 1 /*max opens*/, HWCOUNTER_DEV_NAME);
}

static int hwcounter_close(struct inode *i, struct file *f) {
	return generic_close(&hwcounter_opens_cnt, HWCOUNTER_DEV_NAME);
}

ssize_t hwcounter_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
	u64 timestamp, lo, hi;
	lo = 0;
	hi = 0;
	if (len < sizeof(u64)) return -EINVAL;
	lo = (u64) readl(hwcounter_io_addr);
	hi = (u64) readl(hwcounter_io_addr + 1) << 32;
	timestamp = lo | hi;

	if (copy_to_user(buf, &timestamp, sizeof(u64)))
		return -EFAULT;

	return sizeof(u64);
}

static long hwcounter_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case HWCOUNTER_GET_ADDR:
		if (copy_to_user((unsigned long*)arg, &hwcounter_phy_addr, sizeof(unsigned long)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;

	}
	return 0;
}

static struct file_operations hwcounter_fops = {
	.owner = THIS_MODULE,
	.open = hwcounter_open,
	.release = hwcounter_close,
	.read = hwcounter_read,
	.unlocked_ioctl = hwcounter_ioctl,
};

static int hwcounter_map_io(struct device_node *hwcounter_node) {

	int status;
	struct device_node *dev_node;
#if TARGET_64_BITS
	u64 dev_mem_space[2];	//address & size
#else
	u32 dev_mem_space[2];
#endif

	dev_node = of_parse_phandle(hwcounter_node, HWCOUNTER_PHANDLE_NAME, 0);
	if (dev_node != NULL) {
		status = read_memspace(dev_node, dev_mem_space);
		of_node_put(dev_node);
		if (status < 0) {
			printk(KERN_WARNING "<%s> Could not read hwcounter address\n", MODULE_NAME);
			goto hwcounter_of_err;
		}
		hwcounter_phy_addr = (unsigned long)dev_mem_space[0];
		//remap register space in virtual kernel space
		hwcounter_io_addr = ioremap((resource_size_t)dev_mem_space[0],
				(size_t)dev_mem_space[1]);
	} else {
		printk(KERN_INFO "<%s> Device hwcounter not available\n", MODULE_NAME);
		hwcounter_phy_addr = 0;
		hwcounter_io_addr = NULL;
	}

	return 0;

hwcounter_of_err:
	return -1;
}

int hwcounter_probe(struct platform_device *pdev)
{
	//Get the addresses from the devicetree
	if (hwcounter_map_io(pdev->dev.of_node) < 0) {
		printk("<%s> Error mapping hwcounter IO\n", MODULE_NAME);
		goto hwcounter_map_io_err;
	}

	hwcounter_opens_cnt = 0;
	if (hwcounter_io_addr == NULL) {
		//hw instrumentation not present in devicetree
		return 0;
	}

	//Create the device class
	if (alloc_chrdev_region(&hwcounter_devt, 0, 1 /*num devices*/, HWCOUNTER_MODULE_NAME) < 0) {
		printk(KERN_ERR "<%s> Could not allocate region for hw instrumentation devices\n",
			MODULE_NAME);
		goto hwcounter_alloc_err;
	}
	hwcounter_major = MAJOR(hwcounter_devt);

	hwcounter_cl = class_create(THIS_MODULE, HWCOUNTER_MODULE_NAME);
	if (hwcounter_cl == NULL) {
		printk(KERN_ERR "<%s> Could not create hw instrumentation device class\n",
			MODULE_NAME);
		goto hwcounter_class_err;
	}

	//Create device
	hwcounter_dev = device_create(hwcounter_cl, NULL, hwcounter_devt, NULL,
			DEV_PREFIX "/" HWCOUNTER_DEV_NAME);
	if (IS_ERR(hwcounter_dev)) {
		printk(KERN_ERR "<%s> Could not create hw instrumentation raw device\n",
			MODULE_NAME);
		goto hwcounter_dev_err;
	}
	cdev_init(&hwcounter_cdev, &hwcounter_fops);
	if (cdev_add(&hwcounter_cdev, hwcounter_devt, 1)<0) {
		goto hwcounter_cdev_err;
	}

	return 0;

hwcounter_cdev_err:
	device_destroy(hwcounter_cl, hwcounter_devt);
hwcounter_dev_err:
	class_destroy(hwcounter_cl);
hwcounter_class_err:
	unregister_chrdev_region(hwcounter_devt, 1 /*num devices*/);
hwcounter_alloc_err:
hwcounter_map_io_err:
	hwcounter_io_addr = NULL;
	return -1;
}

int hwcounter_remove(struct platform_device *pdev)
{
	if (hwcounter_io_addr == NULL) {
		//No device was created nor registered
		return 0;
	}

	if (hwcounter_opens_cnt != 0) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, HWCOUNTER_DEV_NAME);
	}

	cdev_del(&hwcounter_cdev);
	device_destroy(hwcounter_cl, hwcounter_devt);

	class_destroy(hwcounter_cl);
	unregister_chrdev_region(hwcounter_devt, 1 /*num devices*/);
	return 0;
}
