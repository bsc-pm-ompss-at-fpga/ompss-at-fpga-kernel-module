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
#include <asm/io.h>

#include "ompss_fpga_common.h"

#define TM_MODULE_NAME      "ompss_fpga_hwr"
#define TM_DEV_DIR          "hwruntime"
#define CTRL_DEV_NAME       "ctrl"
#define CMD_IN_DEV_NAME     "cmd_in_queue"
#define CMD_OUT_DEV_NAME    "cmd_out_queue"
#define SPWN_OUT_DEV_NAME   "spawn_out_queue"
#define SPWN_IN_DEV_NAME    "spawn_in_queue"

#define CTRL_PHANDLE_NAME     "hwruntime-rst"
#define CMD_IN_PHANDLE_NAME   "hwruntime-cmdinqueue"
#define CMD_OUT_PHANDLE_NAME  "hwruntime-cmdoutqueue"
#define SPWN_OUT_PHANDLE_NAME "hwruntime-spawnoutqueue"
#define SPWN_IN_PHANDLE_NAME  "hwruntime-spawninqueue"

#define TM_NUM_DEVICES 5 //ctrl + cmd_in queue + cmd_out queue + spawn out queue + spawn in queue
#define CTRL_MINOR     0
#define CMD_IN_MINOR   1
#define CMD_OUT_MINOR  2
#define SPWN_OUT_MINOR 3
#define SPWN_IN_MINOR  4

static dev_t hwruntime_devt;
static struct class *tm_cl;
static struct cdev ctrl_cdev, cmd_in_cdev, cmd_out_cdev, spwn_out_cdev, spwn_in_cdev;
static struct device *ctrl_dev, *cmd_in_dev, *cmd_out_dev, *spwn_out_dev, *spwn_in_dev;

static int ctrl_opens_cnt;         // Opens counter of ctrl device
static int cmd_in_opens_cnt;       // Opens counter of cmd_in device
static int cmd_out_opens_cnt;      // Opens counter of cmd_out device
static int spwn_out_opens_cnt;     // Opens counter of spawn_out device
static int spwn_in_opens_cnt;      // Opens counter of spawn_in device
static int hwruntime_major;

unsigned long ctrl_io_addr;
unsigned long cmd_in_io_addr;
unsigned long cmd_out_io_addr;
unsigned long spwn_out_io_addr;
unsigned long spwn_in_io_addr;


static int hwruntime_ctrl_open(struct inode *i, struct file *f) {
	return generic_open(&ctrl_opens_cnt, 1, CTRL_DEV_NAME);
}

static int hwruntime_ctrl_close(struct inode *i, struct file *f) {
	return generic_close(&ctrl_opens_cnt, CTRL_DEV_NAME);
}

static int hwruntime_ctrl_mmap(struct file *filp, struct vm_area_struct *vma) {
	return generic_mmap(filp, vma, ctrl_io_addr, CTRL_DEV_NAME);
}

static int hwruntime_cmd_in_open(struct inode *i, struct file *f) {
	return generic_open(&cmd_in_opens_cnt, 1, CMD_IN_DEV_NAME);
}

static int hwruntime_cmd_in_close(struct inode *i, struct file *f) {
	return generic_close(&cmd_in_opens_cnt, CMD_IN_DEV_NAME);
}

static int hwruntime_cmd_in_mmap(struct file *filp, struct vm_area_struct *vma) {
	return generic_mmap(filp, vma, cmd_in_io_addr, CMD_IN_DEV_NAME);
}

static int hwruntime_cmd_out_open(struct inode *i, struct file *f) {
	return generic_open(&cmd_out_opens_cnt, 1, CMD_OUT_DEV_NAME);
}

static int hwruntime_cmd_out_close(struct inode *i, struct file *f) {
	return generic_close(&cmd_out_opens_cnt, CMD_OUT_DEV_NAME);
}

static int hwruntime_cmd_out_mmap(struct file *filp, struct vm_area_struct *vma) {
	return generic_mmap(filp, vma, cmd_out_io_addr, CMD_OUT_DEV_NAME);
}

static int hwruntime_spwn_out_open(struct inode *i, struct file *f) {
	return generic_open(&spwn_out_opens_cnt, 1, SPWN_OUT_DEV_NAME);
}

static int hwruntime_spwn_out_close(struct inode *i, struct file *f) {
	return generic_close(&spwn_out_opens_cnt, SPWN_OUT_DEV_NAME);
}

static int hwruntime_spwn_out_mmap(struct file *filp, struct vm_area_struct *vma) {
	return generic_mmap(filp, vma, spwn_out_io_addr, SPWN_OUT_DEV_NAME);
}

static int hwruntime_spwn_in_open(struct inode *i, struct file *f) {
	return generic_open(&spwn_in_opens_cnt, 1, SPWN_IN_DEV_NAME);
}

static int hwruntime_spwn_in_close(struct inode *i, struct file *f) {
	return generic_close(&spwn_in_opens_cnt, SPWN_IN_DEV_NAME);
}

static int hwruntime_spwn_in_mmap(struct file *filp, struct vm_area_struct *vma) {
	return generic_mmap(filp, vma, spwn_in_io_addr, SPWN_IN_DEV_NAME);
}

static struct file_operations hwruntime_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = hwruntime_ctrl_open,
	.release = hwruntime_ctrl_close,
	.mmap = hwruntime_ctrl_mmap,
};
static struct file_operations hwruntime_cmd_in_fops = {
	.owner = THIS_MODULE,
	.open = hwruntime_cmd_in_open,
	.release = hwruntime_cmd_in_close,
	.mmap = hwruntime_cmd_in_mmap,
};
static struct file_operations hwruntime_cmd_out_ops = {
	.owner = THIS_MODULE,
	.open = hwruntime_cmd_out_open,
	.release = hwruntime_cmd_out_close,
	.mmap = hwruntime_cmd_out_mmap,
};
static struct file_operations hwruntime_spwn_out_ops = {
	.owner = THIS_MODULE,
	.open = hwruntime_spwn_out_open,
	.release = hwruntime_spwn_out_close,
	.mmap = hwruntime_spwn_out_mmap,
};
static struct file_operations hwruntime_spwn_in_ops = {
	.owner = THIS_MODULE,
	.open = hwruntime_spwn_in_open,
	.release = hwruntime_spwn_in_close,
	.mmap = hwruntime_spwn_in_mmap,
};

static int hwruntime_map_io(struct device_node *hwruntime_node) {

	int status;
	struct device_node *dev_node;
#if TARGET_64_BITS
	u64 dev_mem_space[2];	//address & size
#else
	u32 dev_mem_space[2];
#endif

	dev_node = of_parse_phandle(hwruntime_node, CTRL_PHANDLE_NAME, 0);
	status = read_memspace(dev_node, dev_mem_space);
	of_node_put(dev_node);
	if (status < 0) {
		printk(KERN_WARNING "<%s> Could not read address of '%s'\n", MODULE_NAME, CTRL_PHANDLE_NAME);
		goto ctrl_of_err;
	}
	ctrl_io_addr = (unsigned long)dev_mem_space[0];

	dev_node = of_parse_phandle(hwruntime_node, CMD_IN_PHANDLE_NAME, 0);
	status = read_memspace(dev_node, dev_mem_space);
	of_node_put(dev_node);
	if (status < 0) {
		printk(KERN_WARNING "<%s> Could not read address of '%s'\n", MODULE_NAME, CMD_IN_PHANDLE_NAME);
		goto cmd_in_of_err;
	}
	cmd_in_io_addr = (unsigned long)dev_mem_space[0];

	dev_node = of_parse_phandle(hwruntime_node, CMD_OUT_PHANDLE_NAME, 0);
	status = read_memspace(dev_node, dev_mem_space);
	of_node_put(dev_node);
	if (status < 0) {
		printk(KERN_WARNING "<%s> Could not read address of '%s'\n", MODULE_NAME, CMD_OUT_PHANDLE_NAME);
		goto cmd_out_of_err;
	}
	cmd_out_io_addr = (unsigned long)dev_mem_space[0];

	dev_node = of_parse_phandle(hwruntime_node, SPWN_OUT_PHANDLE_NAME, 0);
	if (dev_node != NULL) {
		status = read_memspace(dev_node, dev_mem_space);
		of_node_put(dev_node);
		if (status < 0) {
			printk(KERN_WARNING "<%s> Could not read address of '%s'\n",
				MODULE_NAME, SPWN_OUT_PHANDLE_NAME);
			goto spwn_out_of_err;
		}
		spwn_out_io_addr = (unsigned long)dev_mem_space[0];
	} else {
		printk(KERN_INFO "<%s> '%s' not available in this devicetree\n", MODULE_NAME, SPWN_OUT_DEV_NAME);
		spwn_out_io_addr = 0;
	}

	dev_node = of_parse_phandle(hwruntime_node, SPWN_IN_PHANDLE_NAME, 0);
	if (dev_node != NULL) {
		status = read_memspace(dev_node, dev_mem_space);
		of_node_put(dev_node);
		if (status < 0) {
			printk(KERN_WARNING "<%s> Could not read address of '%s'\n",
				MODULE_NAME, SPWN_IN_PHANDLE_NAME);
			goto spwn_in_of_err;
		}
		spwn_in_io_addr = (unsigned long)dev_mem_space[0];
	} else {
		printk(KERN_INFO "<%s> '%s' not available in this devicetree\n", MODULE_NAME, SPWN_IN_DEV_NAME);
		spwn_in_io_addr = 0;
	}

	return 0;

spwn_in_of_err:
spwn_out_of_err:
cmd_out_of_err:
cmd_in_of_err:
ctrl_of_err:
	return -1;
}

int hwruntime_probe(struct platform_device *pdev)
{
	//Get the addresses from the devicetree
	if (hwruntime_map_io(pdev->dev.of_node) < 0) {
		printk("<%s> Error mapping task manager IO\n", MODULE_NAME);
		goto map_io_err;
	}

	//Create the device class
	if (alloc_chrdev_region(&hwruntime_devt, 0, TM_NUM_DEVICES, TM_MODULE_NAME) < 0) {
		printk(KERN_ERR "<%s> Could not allocate region for taskmanager devices\n",
			MODULE_NAME);
		goto hwruntime_alloc_chrdev_err;
	}
	hwruntime_major = MAJOR(hwruntime_devt);

	tm_cl = class_create(THIS_MODULE, TM_MODULE_NAME);
	if (tm_cl == NULL) {
		printk(KERN_ERR "<%s> Could not create hwruntime device class\n",
			MODULE_NAME);
		goto class_err;
	}

	//Create device for the control BRAM
	ctrl_dev = device_create(tm_cl, NULL, hwruntime_devt, NULL,
			DEV_PREFIX "/" TM_DEV_DIR "/" CTRL_DEV_NAME);
	if (IS_ERR(ctrl_dev)) {
		printk(KERN_ERR "<%s> Could not create hwruntime device: '%s'\n",
			MODULE_NAME, CTRL_DEV_NAME);
		goto ctrl_dev_err;
	}
	cdev_init(&ctrl_cdev, &hwruntime_ctrl_fops);
	if (cdev_add(&ctrl_cdev, hwruntime_devt, 1)<0) {
		goto ctrl_cdev_err;
	}
	ctrl_opens_cnt = 0;

	//Create device for the cmd_in queue
	cmd_in_dev = device_create(tm_cl, NULL, MKDEV(hwruntime_major, CMD_IN_MINOR), NULL,
			DEV_PREFIX "/" TM_DEV_DIR "/" CMD_IN_DEV_NAME);
	if (IS_ERR(cmd_in_dev)) {
		printk(KERN_ERR "<%s> Could not create hwruntime device: '%s'\n",
			MODULE_NAME, CMD_IN_DEV_NAME);
		goto cmd_in_dev_err;
	}
	cdev_init(&cmd_in_cdev, &hwruntime_cmd_in_fops);
	if (cdev_add(&cmd_in_cdev, MKDEV(hwruntime_major, CMD_IN_MINOR), 1) < 0) {
		goto cmd_in_cdev_err;
	}
	cmd_in_opens_cnt = 0;

	//Create device for the cmd_out queue
	cmd_out_dev = device_create(tm_cl, NULL, MKDEV(hwruntime_major, CMD_OUT_MINOR), NULL,
			DEV_PREFIX "/" TM_DEV_DIR "/" CMD_OUT_DEV_NAME);
	if (IS_ERR(cmd_out_dev)) {
		printk(KERN_ERR "<%s> Could not create hwruntime device: '%s'\n",
			MODULE_NAME, CMD_OUT_DEV_NAME);
		goto cmd_out_dev_err;
	}
	cdev_init(&cmd_out_cdev, &hwruntime_cmd_out_ops);
	if (cdev_add(&cmd_out_cdev, MKDEV(hwruntime_major, CMD_OUT_MINOR), 1) < 0) {
		goto cmd_out_cdev_err;
	}
	cmd_out_opens_cnt = 0;

	//If available, create device for the new queue
	if (spwn_out_io_addr != 0) {
		spwn_out_dev = device_create(tm_cl, NULL, MKDEV(hwruntime_major, SPWN_OUT_MINOR), NULL,
				DEV_PREFIX "/" TM_DEV_DIR "/" SPWN_OUT_DEV_NAME);
		if (IS_ERR(spwn_out_dev)) {
			printk(KERN_ERR "<%s> Could not create hwruntime device: '%s'\n",
				MODULE_NAME, SPWN_OUT_DEV_NAME);
			goto spwn_out_dev_err;
		}
		cdev_init(&spwn_out_cdev, &hwruntime_spwn_out_ops);
		if (cdev_add(&spwn_out_cdev, MKDEV(hwruntime_major, SPWN_OUT_MINOR), 1) < 0) {
			goto spwn_out_cdev_err;
		}
	}
	spwn_out_opens_cnt = 0;

	//If available, create device for the remote finished queue
	if (spwn_in_io_addr != 0) {
		spwn_in_dev = device_create(tm_cl, NULL, MKDEV(hwruntime_major, SPWN_IN_MINOR), NULL,
				DEV_PREFIX "/" TM_DEV_DIR "/" SPWN_IN_DEV_NAME);
		if (IS_ERR(spwn_in_dev)) {
			printk(KERN_ERR "<%s> Could not create hwruntime device: '%s'\n",
				MODULE_NAME, SPWN_IN_DEV_NAME);
			goto spwn_in_dev_err;
		}
		cdev_init(&spwn_in_cdev, &hwruntime_spwn_in_ops);
		if (cdev_add(&spwn_in_cdev, MKDEV(hwruntime_major, SPWN_IN_MINOR), 1) < 0) {
			goto spwn_in_cdev_err;
		}
	}
	spwn_in_opens_cnt = 0;

	return 0;

spwn_in_cdev_err:
	device_destroy(tm_cl, MKDEV(hwruntime_major, SPWN_IN_MINOR));
spwn_in_dev_err:
	cdev_del(&spwn_out_cdev);
spwn_out_cdev_err:
	device_destroy(tm_cl, MKDEV(hwruntime_major, SPWN_OUT_MINOR));
spwn_out_dev_err:
	cdev_del(&cmd_out_cdev);
cmd_out_cdev_err:
	device_destroy(tm_cl, MKDEV(hwruntime_major, CMD_OUT_MINOR));
cmd_out_dev_err:
	cdev_del(&cmd_in_cdev);
cmd_in_cdev_err:
	device_destroy(tm_cl, MKDEV(hwruntime_major, CMD_IN_MINOR));
cmd_in_dev_err:
	cdev_del(&ctrl_cdev);
ctrl_cdev_err:
	device_destroy(tm_cl, hwruntime_devt);
ctrl_dev_err:
	class_destroy(tm_cl);
class_err:
	unregister_chrdev_region(hwruntime_devt, TM_NUM_DEVICES);
hwruntime_alloc_chrdev_err:
	ctrl_io_addr = 0;
	cmd_in_io_addr = 0;
	cmd_out_io_addr = 0;
	spwn_out_io_addr = 0;
	spwn_in_io_addr = 0;
map_io_err:
	tm_cl = NULL;
	return -1;
}

int hwruntime_remove(struct platform_device *pdev)
{
	if (ctrl_opens_cnt != 0) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, CTRL_DEV_NAME);
	}

	if (ctrl_io_addr != 0) {
		cdev_del(&ctrl_cdev);
		device_destroy(tm_cl, hwruntime_devt);
		ctrl_io_addr = 0;
	}

	if (cmd_in_opens_cnt != 0) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, CMD_IN_DEV_NAME);
	}

	if (cmd_in_io_addr != 0) {
		cdev_del(&cmd_in_cdev);
		device_destroy(tm_cl, MKDEV(hwruntime_major, CMD_IN_MINOR));
		cmd_in_io_addr = 0;
	}

	if (cmd_out_opens_cnt != 0) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, CMD_OUT_DEV_NAME);
	}

	if (cmd_out_io_addr != 0) {
		cdev_del(&cmd_out_cdev);
		device_destroy(tm_cl, MKDEV(hwruntime_major, CMD_OUT_MINOR));
		cmd_out_io_addr = 0;
	}

	if (spwn_out_opens_cnt != 0) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, SPWN_OUT_DEV_NAME);
	}

	if (spwn_out_io_addr != 0) {
		cdev_del(&spwn_out_cdev);
		device_destroy(tm_cl, MKDEV(hwruntime_major, SPWN_OUT_MINOR));
		spwn_out_io_addr = 0;
	}

	if (spwn_in_opens_cnt != 0) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, SPWN_IN_DEV_NAME);
	}

	if (spwn_in_io_addr != 0) {
		cdev_del(&spwn_in_cdev);
		device_destroy(tm_cl, MKDEV(hwruntime_major, SPWN_IN_MINOR));
		spwn_in_io_addr = 0;
	}

	if (tm_cl != NULL) {
		class_destroy(tm_cl);
		unregister_chrdev_region(hwruntime_devt, TM_NUM_DEVICES);
	}
	return 0;
}
