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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>

#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/of_device.h>

#include "ompss_fpga_common.h"

#define XDMAMEM_MODULE_NAME  "ompss_fpga_mem"
#define XDMAMEM_DEV_NAME     "xdma_mem"
#define XDMAMEM_DEV_MEM_NAME "xdma_dev_mem"
#define XDMAMEM_NUM_DEVICES	2
#define XDMAMEM_MEM_MINOR		0
#define XDMAMEM_DEV_MEM_MINOR	1
#define DEV_MEM_PHANDLE_NAME	"devMem"

//#define DEBUG_PRINT 1

#ifdef DEBUG_PRINT
#define PRINT_DBG(...) pr_debug( __VA_ARGS__)
#else
#define PRINT_DBG(...)
#endif

static atomic_t xdmamem_opens_cnt;   // Global counter of device opens
static dev_t dev_num;		// Global variable for the device number
static struct cdev c_dev;	// Global variable for the character device structure
static struct cdev cdev_dev_mem;	// Global variable for dev mem char device struct
static struct class *cl;	// Global variable for the device class
static struct device *dma_dev;
static struct device *dev_mem_dev;
static struct platform_device *xdmamem_pdev;

static int xdmamem_major;

static long dev_mem_io_addr;
static atomic_t devmem_open_cnt;

struct xdmamem_kern_buf {
	void * addr;
	unsigned long dma_addr;
	size_t size;
	struct list_head desc_list;
};

static struct xdmamem_kern_buf *last_dma_handle;
static struct kmem_cache *buf_handle_cache;

static void xdmamem_free_buffers(void);

/* save a list of dma buffers so they can be deleted in case the application
 * does not free them (in case of an abnormal abort)
 */
static struct list_head desc_list;

static int xdmamem_open(struct inode *i, struct file *f)
{
	return generic_open(&xdmamem_opens_cnt, 1 /*max_opens*/, XDMAMEM_MODULE_NAME);
}

static int xdmamem_close(struct inode *i, struct file *f)
{
	xdmamem_free_buffers();
	return generic_close(&xdmamem_opens_cnt, XDMAMEM_MODULE_NAME);
}

static ssize_t xdmamem_read(struct file *f, char __user * buf, size_t
			 len, loff_t * off)
{
	return -ENOSYS;
}

static ssize_t xdmamem_write(struct file *f, const char __user * buf,
			  size_t len, loff_t * off)
{
	return -ENOSYS;
}

static int xdmamem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int result;
	unsigned long requested_size;
	dma_addr_t dma_handle;
	void *buffer_addr;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
#if TARGET_64_BITS
	struct device *dev = dma_dev;
#else //TARGET_64_BITS
	static struct device *dev = NULL;
#endif
#else //(LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
	struct device *dev = &xdmamem_pdev->dev;
#endif

	requested_size = vma->vm_end - vma->vm_start;

	PRINT_DBG(XDMAMEM_MODULE_NAME "Request %lu bytes to kernel\n", requested_size);
#if LINUX_KERNEL_VERSION_4XX || LINUX_KERNEL_VERSION_3XX
	buffer_addr = dma_zalloc_coherent(dev, requested_size, &dma_handle,
			GFP_KERNEL);
#else
	buffer_addr = dma_alloc_coherent(dev, requested_size, &dma_handle,
			GFP_KERNEL | __GFP_ZERO)
#endif

	PRINT_DBG("    dma@: %llx kernel@: %p\n", (u64)dma_handle, buffer_addr);
	if (!buffer_addr) {
		return -ENOMEM;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	//For some reason, physical address is not correctly computed
	//causing the DMA address and the physical address being mapped to
	//be different.
	//This causes the process to write in a different physical location
	//different from the one sent to the HW
	result = remap_pfn_range(vma, vma->vm_start,
			dma_handle >> PAGE_SHIFT,
			//virt_to_pfn(buffer_addr),
			requested_size, vma->vm_page_prot);

	PRINT_DBG("  Mapped usr: %lx kern: %p dma: %llx pfn: %lx\n",
			vma->vm_start, buffer_addr, (u64)dma_handle,
			virt_to_pfn(buffer_addr));
//	PRINT_DBG("  virt_to_phys: %p __pv_phys_pfn_offset: %x\n",
//			virt_to_phys(buffer_addr), __pv_phys_pfn_offset);
	if (result) {
		pr_err("<%s> Error: in calling remap_pfn_range: returned %d\n",
		       XDMAMEM_MODULE_NAME, result);

		return -EAGAIN;
	}

	//last_dma_handle = kmalloc(sizeof(struct xdmameme_kern_buf));
	last_dma_handle = kmem_cache_alloc(buf_handle_cache, GFP_KERNEL);
	last_dma_handle->addr = buffer_addr;
	last_dma_handle->dma_addr = dma_handle;
	last_dma_handle->size = requested_size;

	list_add(&last_dma_handle->desc_list, &desc_list);

	return 0;
}

struct xdmamem_kern_buf* xdmamem_get_last_kern_buff(void)
{
	return last_dma_handle;
}

unsigned long xdmamem_get_dma_address(struct xdmamem_kern_buf *kbuf)
{
	const unsigned long dma_addr = kbuf ? kbuf->dma_addr : 0;
	PRINT_DBG(KERN_DEBUG "DMA addr: %lx\n", dma_addr);
	return dma_addr;
}


//Return the size of the buffer to be reed in order to return to the user for
//unmapping the buffer from user space
static size_t xdmamem_release_kernel_buffer(struct xdmamem_kern_buf *buff_desc)
{
	size_t size = buff_desc->size;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
#if TARGET_64_BITS
	struct device *dev = dma_dev;
#else //TARGET_64_BITS
	static struct device *dev = NULL;
#endif
#else //(LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
	struct device *dev = &xdmamem_pdev->dev;
#endif

	list_del(&buff_desc->desc_list);
	dma_free_coherent(dev, size, buff_desc->addr, buff_desc->dma_addr);
		kmem_cache_free(buf_handle_cache, buff_desc);
	return size;
}

#if LINUX_KERNEL_VERSION_5XX
#define XDMAMEM_ACCESS_OK(type, var, size) access_ok(var, size)
#else
#define XDMAMEM_ACCESS_OK(type, var, size) access_ok(type, var, size)
#endif

static long xdmamem_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	//u32 devices;
	struct xdmamem_kern_buf *kbuff_ptr;
	unsigned long dma_address;

	switch (cmd) {
	case XDMAMEM_GET_LAST_KBUF:
		if (!XDMAMEM_ACCESS_OK(void*, (void*)arg, sizeof(void*))) {
			pr_debug("<%s> Cannot access user variable @0x%lx",
					XDMAMEM_MODULE_NAME, arg);
			return -EFAULT;
		}
		kbuff_ptr = xdmamem_get_last_kern_buff();
		if (!kbuff_ptr)
			ret = -EFAULT;
		put_user((unsigned long)kbuff_ptr, (unsigned long __user *)arg);
		break;
	case XDMAMEM_RELEASE_KBUF:
		PRINT_DBG(KERN_DEBUG "<%s> ioctl: XDMAMEM_RELEASE_KBUFF\n", XDMAMEM_MODULE_NAME);
		if (!XDMAMEM_ACCESS_OK(void*, (void*)arg, sizeof(void*))) {
			pr_debug("<%s> Cannot access user variable @0x%lx",
					XDMAMEM_MODULE_NAME, arg);
			return -EFAULT;
		}
		get_user(kbuff_ptr, (struct xdmamem_kern_buf **)arg);
		ret = xdmamem_release_kernel_buffer(kbuff_ptr);
		break;

	case XDMAMEM_GET_DMA_ADDRESS:
		PRINT_DBG(KERN_DEBUG "<%s> ioctl: XDMAMEM_GET_DMA_ADDRESS\n", XDMAMEM_MODULE_NAME);
		if (!XDMAMEM_ACCESS_OK(void*, (void*)arg, sizeof(void*))) {
			pr_debug("<%s> Cannot access user variable @0x%lx",
					XDMAMEM_MODULE_NAME, arg);
			return -EFAULT;
		}
		get_user(kbuff_ptr, (struct xdmamem_kern_buf **)arg);
		dma_address = xdmamem_get_dma_address(kbuff_ptr);
		if (!dma_address) {
			ret = -EINVAL;
		}
		put_user((unsigned long)dma_address, (unsigned long __user *)arg);
		break;

	default:
		pr_debug("<%s> ioctl: WARNING unknown ioctl command %d\n", XDMAMEM_MODULE_NAME, cmd);
		break;
	}

	return ret;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = xdmamem_open,
	.release = xdmamem_close,
	.read = xdmamem_read,
	.write = xdmamem_write,
	.mmap = xdmamem_mmap,
	.unlocked_ioctl = xdmamem_ioctl,
};

static int xdma_devmem_open(struct inode *i, struct file *f) {
	return generic_open(&devmem_open_cnt, 1, XDMAMEM_DEV_MEM_NAME);
}

static int xdma_devmem_close(struct inode *i, struct file *f) {
	return generic_close(&devmem_open_cnt, XDMAMEM_DEV_MEM_NAME);
}

static int xdma_devmem_mmap(struct file *filp, struct vm_area_struct *vma) {
	return generic_mmap(filp, vma, dev_mem_io_addr, XDMAMEM_DEV_MEM_NAME);
}


static struct file_operations devmem_fops = {
	.owner = THIS_MODULE,
	.open = xdma_devmem_open,
	.release = xdma_devmem_close,
	.mmap = xdma_devmem_mmap,
};


static void xdmamem_free_buffers(void)
{
	struct xdmamem_kern_buf *bdesc;

	//free all allocated dma buffers
	while (!list_empty(&desc_list)) {
		bdesc = list_first_entry(&desc_list, struct xdmamem_kern_buf, desc_list);
		//this frees the buffer and deletes its descriptor from the list
		xdmamem_release_kernel_buffer(bdesc);
	}
}

static int dev_mem_map_io(struct device_node *hwruntime_node) {
	int status;
	struct device_node *dev_node;
#if TARGET_64_BITS
	u64 dev_mem_space[2];	//address & size
#else
	u32 dev_mem_space[2];
#endif

	dev_node = of_parse_phandle(hwruntime_node, DEV_MEM_PHANDLE_NAME, 0);
	status = read_memspace(dev_node, dev_mem_space);
	of_node_put(dev_node);
	dev_mem_io_addr = (unsigned long)dev_mem_space[0];
	if (status < 0) {
		pr_warn("<%s> Could not read address of '%s'\n",
				MODULE_NAME, DEV_MEM_PHANDLE_NAME);
		goto devmem_of_err;
	}
	return 0;

devmem_of_err:
	return -1;
}

int xdmamem_probe(struct platform_device *pdev)
{
	struct device_node *xdmamem_node;
	//num_devices = 0;
	atomic_set(&xdmamem_opens_cnt, 0);

	//Save platform device structure for later use
	xdmamem_pdev = pdev;

	/* device constructor */
	if (alloc_chrdev_region(&dev_num, 0, XDMAMEM_NUM_DEVICES, XDMAMEM_MODULE_NAME) < 0) {
		pr_err("<%s> Could not allocate region for xdma_mem device\n",
			MODULE_NAME);
		goto xdmamem_alloc_chrdev_err;
	}
	xdmamem_major = MAJOR(dev_num);
	if ((cl = class_create(THIS_MODULE, XDMAMEM_MODULE_NAME)) == NULL) {
		pr_err("<%s> Could not create xdma_mem device class\n",
			MODULE_NAME);
		goto xdmamem_class_err;
	}

	dma_dev = device_create(cl, &xdmamem_pdev->dev, dev_num, NULL,
			DEV_PREFIX "/" XDMAMEM_DEV_NAME);
	if (dma_dev == NULL) {
		pr_err("<%s> Could not create xdma_mem device\n",
			MODULE_NAME);
		goto xdmamem_dev_err;
	}
	cdev_init(&c_dev, &fops);
	if (cdev_add(&c_dev, dev_num, 1) == -1) {
		pr_err("<%s> Could not add xdma_mem device\n",
			MODULE_NAME);
		goto xdmamem_cdev_err;
	}

	xdmamem_node = pdev->dev.of_node;
	atomic_set(&devmem_open_cnt, 0);

	//slab cache for the buffer descriptors
	INIT_LIST_HEAD(&desc_list);
	buf_handle_cache = kmem_cache_create("DMA buffer descriptor cache",
			sizeof(struct xdmamem_kern_buf),
			0, 0, NULL);


	//get device memory address
	dev_mem_dev = NULL;
	if (dev_mem_map_io(pdev->dev.of_node) < 0) {
		pr_warn("<%s> Could not map device address space\n", MODULE_NAME);
		//This is not an error as we may be in a shared memory device
		return 0;
	}

	//create device memory device
	dev_mem_dev = device_create(cl, &xdmamem_pdev->dev,
			MKDEV(xdmamem_major, XDMAMEM_DEV_MEM_MINOR), NULL,
			DEV_PREFIX "/" XDMAMEM_DEV_MEM_NAME);
	if (dev_mem_dev == NULL) {
		pr_err("<%s> Error mapping device address space\n", MODULE_NAME);
		goto xdmamem_dev_mem_err;
	}

	cdev_init(&cdev_dev_mem, &devmem_fops);
	if (cdev_add(&cdev_dev_mem, MKDEV(xdmamem_major, XDMAMEM_DEV_MEM_MINOR), 1) < 0) {
		pr_err("<%s> Could not add xdma_dev_mem device\n", MODULE_NAME);
		goto xdmamem_dev_mem_cdev_err;
	}

	return 0;

xdmamem_dev_mem_cdev_err:
	device_destroy(cl, MKDEV(xdmamem_major, XDMAMEM_DEV_MEM_MINOR));
xdmamem_dev_mem_err:
	dev_mem_dev = NULL;
xdmamem_cdev_err:
	device_destroy(cl, dev_num);
xdmamem_dev_err:
	class_destroy(cl);
xdmamem_class_err:
	unregister_chrdev_region(dev_num, XDMAMEM_NUM_DEVICES);
xdmamem_alloc_chrdev_err:
	cl = NULL;
	return -1;
}

int xdmamem_remove(struct platform_device *pdev)
{
	if (atomic_read(&xdmamem_opens_cnt) != 0) {
		pr_err("<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, XDMAMEM_DEV_NAME);
	}

	if (cl == NULL) {
		//Device not initialized
		return 0;
	}

	xdmamem_free_buffers();
	kmem_cache_destroy(buf_handle_cache);

	// Device memory
	if (dev_mem_dev != NULL) {
		cdev_del(&cdev_dev_mem);
		device_destroy(cl, MKDEV(xdmamem_major, XDMAMEM_DEV_MEM_MINOR));
	}

	cdev_del(&c_dev);
	device_destroy(cl, dev_num);
	class_destroy(cl);
	unregister_chrdev_region(dev_num, XDMAMEM_NUM_DEVICES);

	return 0;
}

#undef PRINT_DBG
#undef DEBUG_PRINT
