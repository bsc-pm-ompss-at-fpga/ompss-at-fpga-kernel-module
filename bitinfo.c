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

#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <asm/io.h>

#include "ompss_fpga_common.h"

#define BITINFO_MODULE_NAME      "ompss_fpga_bitinfo"
#define BITINFO_NAME             "bitinfo"
#define BITINFO_PHANDLE_NAME     "bitstreaminfo"
#define BITINFO_VERSION 10
#define BITINFO_NUM_DEVICES 1
#define BITINFO_FEATURES_IDX 2
#define BIT_FEATURES_HWCOUNTER_B 1
#define BIT_FEATURES_SPAWN_Q_B 8
#define CMD_IN_BITINFO_ADDR_OFFSET 7
#define CMD_IN_BITINFO_LEN_OFFSET 9
#define CMD_OUT_BITINFO_ADDR_OFFSET 10
#define CMD_OUT_BITINFO_LEN_OFFSET 12
#define SPWN_IN_BITINFO_ADDR_OFFSET 13
#define SPWN_IN_BITINFO_LEN_OFFSET 15
#define SPWN_OUT_BITINFO_ADDR_OFFSET 16
#define SPWN_OUT_BITINFO_LEN_OFFSET 18
#define RST_BITINFO_ADDR_OFFSET 19
#define HWCOUNTER_BITINFO_ADDR_OFFSET 21

#define BITINFO_GET_VERSION 13

#define BITINFO_MAX_WORDS 1024

static dev_t bitinfo_devt;
static struct class *bitinfo_cl;
static struct cdev bitinfo_cdev;

static struct device *bitinfo_dev;

static atomic_t bitinfo_opens_cnt; // Opens counter of device

u32 __iomem * bitinfo_io_addr;

u64 get_cmd_in_addr() {
	return readl(bitinfo_io_addr + CMD_IN_BITINFO_ADDR_OFFSET) |
				((u64)readl(bitinfo_io_addr + CMD_IN_BITINFO_ADDR_OFFSET + 1) << 32);
}
u64 get_cmd_out_addr() {
	return readl(bitinfo_io_addr + CMD_OUT_BITINFO_ADDR_OFFSET) |
				((u64)readl(bitinfo_io_addr + CMD_OUT_BITINFO_ADDR_OFFSET + 1) << 32);
}
u64 get_spawn_in_addr() {
	return readl(bitinfo_io_addr + SPWN_IN_BITINFO_ADDR_OFFSET) |
				((u64)readl(bitinfo_io_addr + SPWN_IN_BITINFO_ADDR_OFFSET + 1) << 32);
}
u64 get_spawn_out_addr() {
	return readl(bitinfo_io_addr + SPWN_OUT_BITINFO_ADDR_OFFSET) |
				((u64)readl(bitinfo_io_addr + SPWN_OUT_BITINFO_ADDR_OFFSET + 1) << 32);
}
u64 get_hwcounter_addr() {
	return readl(bitinfo_io_addr + HWCOUNTER_BITINFO_ADDR_OFFSET) |
				((u64)readl(bitinfo_io_addr + HWCOUNTER_BITINFO_ADDR_OFFSET + 1) << 32);
}
u64 get_managed_rstn_addr() {
	return readl(bitinfo_io_addr + RST_BITINFO_ADDR_OFFSET) |
				((u64)readl(bitinfo_io_addr + RST_BITINFO_ADDR_OFFSET + 1) << 32);
}
int get_spawn_q_feature(void) {
	return (readl(bitinfo_io_addr + BITINFO_FEATURES_IDX) >> BIT_FEATURES_SPAWN_Q_B) & 0x1;
}
int get_hwcounter_feature(void) {
	return (readl(bitinfo_io_addr + BITINFO_FEATURES_IDX) >> BIT_FEATURES_HWCOUNTER_B) & 0x1;
}

static int bitinfo_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_opens_cnt, 1000 /*max_opens*/, BITINFO_NAME);
}

static int bitinfo_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_opens_cnt, BITINFO_NAME);
}

static ssize_t bitinfo_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	u32 * data_ptr;
	ssize_t read_cnt;

	read_cnt = min(length, (size_t)(BITINFO_MAX_WORDS*sizeof(u32) - *offset));
	if (read_cnt <= 0 || read_cnt > BITINFO_MAX_WORDS*sizeof(u32)) return 0;

	data_ptr = (u32 *)(bitinfo_io_addr) + (*offset)/sizeof(u32);
	if (copy_to_user(buffer, data_ptr, read_cnt)) return -EFAULT;

	*offset += read_cnt;
	return read_cnt;
}

static ssize_t bitinfo_write(struct file *filp, const char __user *buffer, size_t length, loff_t *offset) {
	u32 * data_ptr;
	ssize_t write_cnt; //< Number of bytes to move

	write_cnt = min(length, (size_t)(BITINFO_MAX_WORDS*sizeof(u32) - *offset));
	if (write_cnt < 0 || write_cnt > BITINFO_MAX_WORDS*sizeof(u32)) return 0;

	data_ptr = (u32 *)(bitinfo_io_addr) + (*offset)/sizeof(u32);
	if (copy_from_user(data_ptr, buffer, write_cnt)) return -EFAULT;

	*offset += write_cnt;
	return write_cnt;
}

static long bitinfo_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned long supported_version = BITINFO_VERSION;
	switch (cmd) {
	case BITINFO_GET_VERSION:
		if (copy_to_user((unsigned long*)arg, &supported_version, sizeof(unsigned long)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct file_operations bitinfo_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_open,
	.release = bitinfo_close,
	.read = bitinfo_read,
	.write = bitinfo_write,
	.unlocked_ioctl = bitinfo_ioctl
};

static int bitinfo_map_io(struct device_node *bitinfo_node) {
	int status;
	struct device_node *dev_node;
#if TARGET_64_BITS
	u64 dev_mem_space[2]; //address & size
#else
	u32 dev_mem_space[2];
#endif

	dev_node = of_parse_phandle(bitinfo_node, BITINFO_PHANDLE_NAME, 0);
	if (dev_node != NULL) {
		status = read_memspace(dev_node, dev_mem_space);
		of_node_put(dev_node);
		if (status < 0) {
			pr_err("<%s> Could not read bitstream information BRAM address\n", MODULE_NAME);
			goto bitinfo_phandle_of_err;
		}
		//register space in virtual kernel space
		bitinfo_io_addr = (u32* __iomem) ioremap((resource_size_t)dev_mem_space[0],
				(size_t)dev_mem_space[1]);
	} else {
		pr_err("<%s> Bitstream information BRAM not available\n", MODULE_NAME);
		bitinfo_io_addr = NULL;
	}

	return 0;

bitinfo_phandle_of_err:
	return -1;
}

int bitinfo_probe(struct platform_device *pdev)
{
	//Get the addresses from the devicetree
	if (bitinfo_map_io(pdev->dev.of_node) < 0) {
		pr_err("<%s> Error mapping bitstream info BRAM IO\n", MODULE_NAME);
		goto bitinfo_map_io_err;
	}

	if (bitinfo_io_addr == NULL) {
		//Bitinfo BRAM not available
		bitinfo_cl = NULL;
		return 0;
	}

	//Create the device class
	if (alloc_chrdev_region(&bitinfo_devt, 0, BITINFO_NUM_DEVICES, BITINFO_MODULE_NAME) < 0) {
		pr_err("<%s> Could not allocate region for bitstream info devices\n",
			MODULE_NAME);
		goto bitinfo_alloc_chrdev;
	}

	bitinfo_cl = class_create(THIS_MODULE, BITINFO_MODULE_NAME);
	if (bitinfo_cl == NULL) {
		pr_err("<%s> Could not create bitstream info device class\n",
			MODULE_NAME);
		goto bitinfo_class_err;
	}

	bitinfo_dev = device_create(bitinfo_cl, NULL, bitinfo_devt, NULL,
			DEV_PREFIX "/" BITINFO_NAME);
	if (IS_ERR(bitinfo_dev)) {
		pr_err("<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_NAME);
		goto bitinfo_dev_err;
	}
	cdev_init(&bitinfo_cdev, &bitinfo_fops);
	if (cdev_add(&bitinfo_cdev, bitinfo_devt, 1)<0) {
		pr_err("<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_NAME);
		goto bitinfo_cdev_err;
	}
	atomic_set(&bitinfo_opens_cnt, 0);

	return 0;

bitinfo_cdev_err:
	device_destroy(bitinfo_cl, bitinfo_devt);
bitinfo_dev_err:
	class_destroy(bitinfo_cl);
bitinfo_class_err:
	unregister_chrdev_region(bitinfo_devt, BITINFO_NUM_DEVICES);
bitinfo_alloc_chrdev:
bitinfo_map_io_err:
	bitinfo_cl = NULL;
	return -1;
}

int bitinfo_remove(struct platform_device *pdev)
{
	//The device was not created
	if (bitinfo_cl == NULL) return 0;

	device_destroy(bitinfo_cl, bitinfo_devt);
	class_destroy(bitinfo_cl);
	unregister_chrdev_region(bitinfo_devt, BITINFO_NUM_DEVICES);
	return 0;
}
