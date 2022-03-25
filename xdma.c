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

#define XDMA_MODULE_NAME  "ompss_fpga_xdma"
#define XDMA_DEV_NAME     "xdma"
//#define DEBUG_PRINT 1
#define CHAN_NAME_MAX_LEN 32
#define MAX_DEVICES       5

#ifdef DEBUG_PRINT
#define PRINT_DBG(...) pr_debug( __VA_ARGS__)
#else
#define PRINT_DBG(...)
#endif

static u32 num_devices;
static int xdma_opens_cnt;      // Global counter of device opens
static struct platform_device *ompss_at_fpga_pdev;
static dev_t dev_num;		// Global variable for the device number
static struct cdev c_dev;	// Global variable for the character device structure
static struct class *cl;	// Global variable for the device class
static struct device *dma_dev;

#if LINUX_KERNEL_VERSION_5XX
//Support for dma devices is dropped on kernels 5.xx
//Define everything as stubs returning -ENOSYS

static struct file_operations fops = {
};

#else

#if LINUX_KERNEL_VERSION_4XX
#  include <linux/dma/xilinx_dma.h>
#elif LINUX_KERNEL_VERSION_3XX
#  include <linux/amba/xilinx_dma.h>
#else
#  error The support for your Linux Kernel Version is not tested or \
         we could not determine your kernel version
#endif



static struct dma_chan *cdma_channel;

struct xdma_sg_mem {
	struct sg_table sg_tbl;
	unsigned long npages;
	enum dma_transfer_direction dir;
};

static struct xdma_dev *xdma_dev_info[MAX_DEVICES + 1];
static u8 xdma_initialized;

static void xdma_init(void);
static void xdma_cleanup(void);

static int xdma_open(struct inode *i, struct file *f)
{
	if (!xdma_initialized) xdma_init();
	return generic_open(&xdma_opens_cnt, 1 /*max_opens*/, XDMA_MODULE_NAME);
}

static int xdma_close(struct inode *i, struct file *f)
{
	const int ret = generic_close(&xdma_opens_cnt, XDMA_MODULE_NAME);
	if (xdma_initialized) xdma_cleanup();
	return ret;
}

static ssize_t xdma_read(struct file *f, char __user * buf, size_t
			 len, loff_t * off)
{
	PRINT_DBG("<%s> file: read()\n", XDMA_MODULE_NAME);
	return -ENOSYS;
}

static ssize_t xdma_write(struct file *f, const char __user * buf,
			  size_t len, loff_t * off)
{
	PRINT_DBG("<%s> file: write()\n", XDMA_MODULE_NAME);
	return -ENOSYS;
}

static void xdma_get_dev_info(u32 device_id, struct xdma_dev *dev)
{
	int i;

	for (i = 0; i < MAX_DEVICES; i++) {
		if (xdma_dev_info[i]->device_id == device_id)
			break;
	}
	memcpy(dev, xdma_dev_info[i], sizeof(struct xdma_dev));
}

static enum dma_transfer_direction xdma_to_dma_direction(enum xdma_direction
							 xdma_dir)
{
	enum dma_transfer_direction dma_dir;

	switch (xdma_dir) {
	case XDMA_MEM_TO_DEV:
		dma_dir = DMA_MEM_TO_DEV;
		break;
	case XDMA_DEV_TO_MEM:
		dma_dir = DMA_DEV_TO_MEM;
		break;
	default:
		dma_dir = DMA_TRANS_NONE;
		break;
	}

	return dma_dir;
}

static void xdma_sync_callback(void *completion)
{
	PRINT_DBG("Completion callback for %p\n", completion);
	complete(completion);
}

static void xdma_device_control(struct xdma_chan_cfg *chan_cfg)
{
#if LINUX_KERNEL_VERSION_3XX
	struct dma_chan *chan;
	struct dma_device *chan_dev;
	struct xilinx_dma_config config;

	config.direction = xdma_to_dma_direction(chan_cfg->dir);
	config.coalesc = chan_cfg->coalesc;
	config.delay = chan_cfg->delay;
	config.reset = chan_cfg->reset;

	chan = (struct dma_chan *)chan_cfg->chan;

	if (chan) {
		chan_dev = chan->device;
		chan_dev->device_control(chan, DMA_SLAVE_CONFIG,
			(unsigned long)&config);
	}
#else
	//NOTE: No action needed in the new drivers
#endif
}


#ifdef DEBUG_PRINT
static void print_sg_list(struct scatterlist *sg_list, int len) {
	struct scatterlist *cur_sg;
	int i;

	for_each_sg(sg_list, cur_sg, len, i) {
		pr_debug("<%s> pg link = %lx, off = %x, len = %d, dma_@ = %llx, dma_len = %d\n",
			MODULE_NAME,
				cur_sg->page_link,
				cur_sg->offset,
				cur_sg->length,
				cur_sg->dma_address,
				cur_sg->dma_length);
	}
}
#endif


static int prepare_sg_list(struct sg_table **sg_tab,
		unsigned long start_addr, unsigned long len) {

	unsigned int pg_offset, nr_pages, n_pg;
	struct sg_table *sg_tbl;
	int status, i;
	int ret;
	struct scatterlist *sg_start, *sg;
	int fp_offset, pg_len, first_page;
	unsigned long offset, pg_left, cur_base;
	struct page **page_list;
	struct dma_device *cdma_dev = cdma_channel->device;

	sg_tbl = kmalloc(sizeof(struct sg_table), GFP_KERNEL);

	pg_offset = start_addr & ~PAGE_MASK;
	nr_pages = ((((start_addr + len -1) & PAGE_MASK) - (start_addr & PAGE_MASK))
			>> PAGE_SHIFT) + 1;

	page_list = (struct page **) __get_free_page(GFP_KERNEL);
	status = sg_alloc_table(sg_tbl, nr_pages, GFP_KERNEL);
	if (status) {
		pr_warn("Could not allocate SGtable\n");
	}

	offset = start_addr & ~PAGE_MASK;
	sg_start = sg_tbl->sgl;
	cur_base = start_addr;
	pg_left = nr_pages;
	first_page = 1;

	while (pg_left) {
		n_pg = min_t(unsigned long, pg_left,
				PAGE_SIZE / sizeof(struct page *));
		//FIXME write parameter (1) should be set to 1 only if writing these pages
		ret = get_user_pages_fast(cur_base, n_pg, 1, page_list);
		if (ret < 0) {
			//FIXME: free resources in case of error
			pr_err("Error getting user pages from %lu\n", cur_base);
			return ret;
		}
		cur_base += ret*PAGE_SIZE;
		pg_left -= ret;

		for_each_sg(sg_start, sg, ret, i) {
			fp_offset = 0;
			pg_len = PAGE_SIZE;
			//Set offset for first page
			if (first_page) {
				fp_offset = offset;
				pg_len -= offset;
				first_page = 0;
			}
			//Set size for last page
			if (pg_left == 0 && i == ret-1) {
				if ((len + offset) % PAGE_SIZE == 0) {
					//Handle case where the end of the transfer is aligned to the end of the page
					pg_len = PAGE_SIZE - ((len + offset) % PAGE_SIZE);
				} else {
					pg_len -= PAGE_SIZE - ((len + offset) % PAGE_SIZE);
				}
				//printk("  setting last page %x %x %x %x\n", fp_offset, offset, pg_len, len);
			}
			sg_set_page(sg, page_list[i], pg_len, fp_offset);
			//sg->dma_length = pg_len;	//Also set dma length
		}
		sg_start = sg;
	}
	//dma map pages and set SG dma addr
	//FIXME set proper direction
	//FIXME unmap sg list when done
	status = dma_map_sg(cdma_dev->dev, sg_tbl->sgl, nr_pages, DMA_BIDIRECTIONAL);

	sg_tbl->nents = nr_pages;
	*sg_tab = sg_tbl;
	return 0;	//TODO: error management
}

static int xdma_prep_memcpy(struct xdma_memcpy_info *memcpy_info) {

	struct dma_async_tx_descriptor *tx = NULL;
	enum dma_ctrl_flags flags;
	struct completion *cmp;
	//FIXME: Assuming single cdma acc
	//struct dma_device *dev = memcpy_info->chan->device;
	int ret = 0;
	struct sg_table *usr_sg_tbl;
	struct scatterlist *dev_sg_ls;
	struct dma_device *cdma_dev = cdma_channel->device;
	dma_cookie_t cookie;
	struct xdma_sg_mem *sg_mem;

	flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	cmp = kmalloc(sizeof(struct completion), GFP_KERNEL);
	dev_sg_ls = kmalloc(sizeof(struct scatterlist), GFP_KERNEL);
	sg_mem = kmalloc(sizeof(struct xdma_sg_mem), GFP_KERNEL);

	if (!cmp) {
		pr_err("Unable to allocate CDMA completion\n");
	}
	init_completion(cmp);
	PRINT_DBG("memcpy 0x%lx (+ 0x%x) -> 0x%lx (+ 0x%x)\n",
			memcpy_info->src_address + memcpy_info->src_offset,
			memcpy_info->src_offset,
			memcpy_info->dst_address + memcpy_info->dst_offset,
			memcpy_info->dst_offset);

	//FIXME: revisar offsets
	//prepare SG list depending on which is the user space buffer
	if (memcpy_info->direction == XDMA_MEM_TO_DEV) {
		//prepare user sg list
		prepare_sg_list(&usr_sg_tbl,
				memcpy_info->src_address + memcpy_info->src_offset,
				memcpy_info->size
				);
		//prepare device sg entry
		//set a single entry as the buffer is physically contiguous
		sg_set_buf(dev_sg_ls, (void*)memcpy_info->dst_address + memcpy_info->dst_offset,
				memcpy_info->size);
		sg_dma_len(dev_sg_ls) = memcpy_info->size;
		//Looks like xilinx_dma wants us to apply offset
		dev_sg_ls->dma_address = memcpy_info->dst_address + memcpy_info->dst_offset;
		PRINT_DBG("dev sg dma addr 0x%llx\n", dev_sg_ls->dma_address);
		sg_mark_end(dev_sg_ls);
		//looks like flags are not used
		//pr_debug("%p -> %p, %p, 1, %p,   %d, %x", cdma_dev, cdma_channel, dev_sg_ls,
		//		usr_sg_tbl->sgl, usr_sg_tbl->nents, flags);
#ifdef DEBUG_PRINT
		pr_debug("usr sg list\n");
		print_sg_list(usr_sg_tbl->sgl, usr_sg_tbl->nents);
		pr_debug("dev sg list\n");
		print_sg_list(dev_sg_ls, 1);
#endif
		tx = cdma_dev->device_prep_dma_sg(cdma_channel, dev_sg_ls, 1,
				usr_sg_tbl->sgl, usr_sg_tbl->nents, flags);
	} else {	//dev to mem
		prepare_sg_list(&usr_sg_tbl,
				memcpy_info->dst_address + memcpy_info->dst_offset,
				memcpy_info->size
				);
		sg_set_buf(dev_sg_ls,
				(void*)(memcpy_info->src_address + memcpy_info->src_offset),
				memcpy_info->size);
		sg_dma_len(dev_sg_ls) = memcpy_info->size;
		dev_sg_ls->dma_address = memcpy_info->src_address + memcpy_info->src_offset;
		PRINT_DBG("dev sg dma addr 0x%llx\n", dev_sg_ls->dma_address);
		sg_mark_end(dev_sg_ls);
#ifdef DEBUG_PRINT
		pr_debug("usr sg list\n");
		print_sg_list(usr_sg_tbl->sgl, usr_sg_tbl->nents);
		pr_debug("dev sg list\n");
		print_sg_list(dev_sg_ls, 1);
#endif

		tx = cdma_dev->device_prep_dma_sg(cdma_channel,
				usr_sg_tbl->sgl, usr_sg_tbl->nents,
				dev_sg_ls, 1, flags);

	}

//	tx = dev->device_prep_dma_memcpy(
//			cdma_channel,
//			memcpy_info->dst_address + memcpy_info->dst_offset,
//			memcpy_info->src_address + memcpy_info->src_offset,
//            memcpy_info->size,
//			flags);

	if (!tx) {
		pr_err("cdma error device_prep_dma_memcpy");
		ret = -1;
		memcpy_info->cookie = -EBUSY;
	} else {
		tx->callback = xdma_sync_callback;
		tx->callback_param = cmp;
		cookie = dmaengine_submit(tx);
		if (dma_submit_error(cookie)) {
			pr_err("cdma error: tx submit error\n");
			ret = -1;
		}

		PRINT_DBG("Buffer prepared cmp=%p ck=%d\n", cmp, cookie);
		//sg_mem->sg_tbl = usr_sg_tbl;
		//FIXME this memcpy should be avoided
		memcpy(&sg_mem->sg_tbl, usr_sg_tbl, sizeof(*usr_sg_tbl));
		sg_mem->npages = usr_sg_tbl->nents;
		sg_mem->dir = xdma_to_dma_direction(memcpy_info->direction);
		memcpy_info->cookie = cookie;
		memcpy_info->completion = cmp;
		memcpy_info->sg_transfer = (void*)sg_mem;
		//FIXME: assuming single cdma, copy channel to user in order to
		//submit and sync transfer
		memcpy_info->chan = cdma_channel;
	}
	return ret;
}

static int xdma_prep_user_buffer(struct xdma_buf_info * buf_info)
{
	int ret, i;
	unsigned int nr_pages, len, n_pg;
	unsigned long start;
	struct page **page_list;
	struct scatterlist *sg, *sg_start;
	struct dma_chan *chan;
	struct dma_async_tx_descriptor *tx_desc;
	struct completion *cmp;
	enum dma_transfer_direction dir;
	enum dma_ctrl_flags flags;
	dma_cookie_t cookie;
	struct xdma_sg_mem *mem;
	unsigned long cur_base;
	unsigned long offset;
	unsigned long pg_left;
	int fp_offset, pg_len;
	int first_page;

	//TODO: Free resources in case of error in order to prevent memory leaks

	mem = kzalloc(sizeof(struct xdma_sg_mem), GFP_KERNEL);
	cmp = kmalloc(sizeof(dma_cookie_t), GFP_KERNEL);
	//reuse buffer info offset as address
	start = buf_info->address;
	len = buf_info->buf_size;
	chan = (struct dma_chan *)buf_info->chan;
	dir = xdma_to_dma_direction(buf_info->dir);
	flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;

	PRINT_DBG("Pinning buffer @%lx;%u\n", start, len);

	//Check that the address is valid
	if (!access_ok(void*, (void*)start, len)) {
		pr_debug("<%s> Cannot access buffer @%lx:%u\n",
				XDMA_MODULE_NAME, start, len);
		return -EFAULT;
	}
	if (len == 0) {
		pr_debug("<%s> Trying to transfer buffer with length 0 @%lx\n",
				XDMA_MODULE_NAME, start);
		return -EINVAL;
	}

	page_list = (struct page **) __get_free_page(GFP_KERNEL);
	if (!page_list) {
		kfree(mem);
		kfree(cmp);
		pr_warn("<%s> Unable to allocate page list for buffer %lx\n",
				XDMA_MODULE_NAME, start);
		return -ENOMEM;
	}
	offset = start & ~PAGE_MASK;
	nr_pages = ((((start + len -1) & PAGE_MASK) - (start & PAGE_MASK)) >> PAGE_SHIFT) + 1;
	PRINT_DBG("Pinning %u pages @%lx+%lu\n", nr_pages, start, offset);

	ret = sg_alloc_table(&mem->sg_tbl, nr_pages, GFP_KERNEL);
	if (ret) {
		pr_warn("<%s> Coud not allocate SG table for buffer %lx\n",
				XDMA_MODULE_NAME, start);
		return -ENOMEM;
	}

	sg_start = mem->sg_tbl.sgl;
	cur_base = start;
	pg_left = nr_pages;
	first_page = 1;
	while (pg_left) {
		n_pg = min_t(unsigned long, pg_left,
				PAGE_SIZE / sizeof(struct page *));
		ret = get_user_pages_fast(cur_base, n_pg, 1, page_list);
		PRINT_DBG("%d\n", ret);
		if (ret < 0) {
			//FIXME: free resources in case of error
			pr_err("Error getting user pages from %lu\n", cur_base);
			return ret;
		}

		cur_base += ret*PAGE_SIZE;
		pg_left -= ret;

		for_each_sg(sg_start, sg, ret, i) {
			fp_offset = 0;
			pg_len = PAGE_SIZE;
			//Set offset for first page
			if (first_page) {
				fp_offset = offset;
				pg_len -= offset;
				first_page = 0;
			}
			//Set size for last page
			if (pg_left == 0 && i == ret-1) {
				pg_len -= PAGE_SIZE - ((len + offset) % PAGE_SIZE);
			}
			sg_set_page(sg, page_list[i], pg_len, fp_offset);
		}
		sg_start = sg;
	}
	PRINT_DBG("Mapping %u pages and preparing transfer\n", nr_pages);
	ret = dma_map_sg(dma_dev, mem->sg_tbl.sgl, nr_pages, dir);
	if (ret <= 0) {
		pr_err("Error mapping the transfer pages\n");
		return -1;
	}
	tx_desc = dmaengine_prep_slave_sg(chan, mem->sg_tbl.sgl, nr_pages, dir, flags);

	free_page((unsigned long)page_list);

	//submit transfer
	init_completion(cmp);
	tx_desc->callback = xdma_sync_callback;
	tx_desc->callback_param = cmp;
	cookie = dmaengine_submit(tx_desc);
	if (dma_submit_error(cookie)) {
		pr_err("<%s> Error: tx_submit error\n",
				XDMA_MODULE_NAME);
		ret = -1;
	}
	buf_info->cookie = cookie;
	buf_info->completion = cmp;
	buf_info->sg_transfer = mem;

	mem->npages = nr_pages;
	mem->dir = dir;

	PRINT_DBG("Buffer prepared cmp=%p ck=%d\n", cmp, cookie);
	PRINT_DBG("buffer: %p:%d submitted\n", (void*)start, len);

	return 0;
}

static int xdma_user_buffer_release(struct xdma_sg_mem *mem)
{
	struct scatterlist *sg;
	struct page *page;
	int i;
	struct dma_device *cdma_dev = cdma_channel->device;

	//TODO: Error checking

#ifdef DEBUG_PRINT
	pr_debug("unmap sg\n");
	pr_debug("is_coherent: %d\n", is_device_dma_coherent(cdma_dev->dev));
	print_sg_list(mem->sg_tbl.sgl, mem->npages);
#endif
	dma_unmap_sg(cdma_dev->dev, mem->sg_tbl.sgl, mem->npages, mem->dir);

	for_each_sg(mem->sg_tbl.sgl, sg, mem->npages, i) {
		page = sg_page(sg);
		put_page(page);
	}
	sg_free_table(&mem->sg_tbl);
	kfree(mem);
	return 0;

}

static int xdma_prep_buffer(struct xdma_buf_info *buf_info)
{
	int ret = 0;
	struct dma_chan *chan;
	dma_addr_t buf;
	size_t len;
	enum dma_transfer_direction dir;
	enum dma_ctrl_flags flags;
	struct dma_async_tx_descriptor *chan_desc;
	struct completion *cmp;
	dma_cookie_t cookie;

	chan = (struct dma_chan *)buf_info->chan;
	//cmp = (struct completion *)buf_info->completion;
	//Create a new completion for every operation
	//TODO reuse completions when possible
	//  Use a slab cache
	//Completion must be created here
	//XXX: Check if also has to be initialized here
	cmp = kmalloc(sizeof(struct completion), GFP_KERNEL);

	if (!cmp) {
		pr_err("Unable to allocate XDMA completion\n");
	}
	init_completion(cmp);
	buf_info->completion = cmp;
	buf_info->sg_transfer = NULL;

	//init_completion(cmp);
	//Init completion when submitting the transfer

	//TODO: Check that the buffer (or sub-buffer) does not overrun
	//the original buffer
	buf = buf_info->address + buf_info->buf_offset;
	len = buf_info->buf_size;
	dir = xdma_to_dma_direction(buf_info->dir);

	flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;

	chan_desc = dmaengine_prep_slave_single(chan, buf, len, dir, flags);

	if (!chan_desc) {
		pr_err("<%s> Error: dmaengine_prep_slave_single error\n",
		       XDMA_MODULE_NAME);
		ret = -1;
		buf_info->cookie = -EBUSY;
	} else {
		chan_desc->callback = xdma_sync_callback;
		chan_desc->callback_param = cmp;

		// set the prepared descriptor to be executed by the engine
		//cookie = chan_desc->tx_submit(chan_desc);
		cookie = dmaengine_submit(chan_desc);
		if (dma_submit_error(cookie)) {
			pr_err("<%s> Error: tx_submit error\n",
			       XDMA_MODULE_NAME);
			ret = -1;
		}

		buf_info->cookie = cookie;
	}
	PRINT_DBG("Buffer prepared cmp=%p\n", cmp);
	PRINT_DBG("buffer: %p:%zu, %x\n", (void*)buf, len, (int)buf_info->address);

	return ret;
}

static int xdma_start_transfer(struct xdma_transfer *trans)
{
	int ret = 0;
	unsigned long tmo = msecs_to_jiffies(3000);
	enum dma_status status;
	struct dma_chan *chan;
	struct completion *cmp;
	dma_cookie_t cookie;

	chan = (struct dma_chan *)trans->chan;
	cmp = (struct completion *)trans->completion;
	cookie = trans->cookie;

	//init_completion(cmp);
	dma_async_issue_pending(chan);
	PRINT_DBG("Submit transfer %p-%p-%d (ch-cmp-ck)", (void*)trans->chan, (void*)trans->completion, trans->cookie);

	if (trans->wait) {
		PRINT_DBG(" Sync transfer, waiting\n");
		tmo = wait_for_completion_timeout(cmp, tmo);
		status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);
		if (0 == tmo) {
			pr_err("<%s> Error: transfer timed out\n",
			       XDMA_MODULE_NAME);
			ret = -1;
		} else if (status != DMA_COMPLETE) {
			pr_debug("<%s> transfer: returned completion callback status of: \'%s\'\n",
			       XDMA_MODULE_NAME,
			       status == DMA_ERROR ? "error" : "in progress");
			ret = -1;
		}
	}
	return ret;
}

static int xdma_finish_transfer(struct xdma_transfer *trans)
{

	int ret = 0;
	unsigned long tmo = msecs_to_jiffies(3000);
	enum dma_status status;
	struct dma_chan *chan;
	struct completion *cmp;
	dma_cookie_t cookie;

	chan = (struct dma_chan *)trans->chan;
	//get the completion initialized while preparing the buffer
	cmp = (struct completion *)trans->completion;

	cookie = trans->cookie;
	PRINT_DBG("Finish transfer: Cmp/cookie: %p/%d -> done: %d\n", cmp, cookie, cmp->done);

	status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);
	if (status == DMA_COMPLETE) {
		ret = XDMA_DMA_TRANSFER_FINISHED;
		//delete completion if transfer has been completed
		PRINT_DBG(" Transfer finished, deleting completion\n");
		kfree(cmp);
	} else {
		ret = XDMA_DMA_TRANSFER_PENDING;
	}

	if (trans->wait && status != DMA_COMPLETE) {
		PRINT_DBG(" Waiting for completion... %p(%d)\n", cmp, cmp->done);
		tmo = wait_for_completion_timeout(cmp, tmo);
		status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);
		PRINT_DBG("  Finished t left: %lu completed: %d\n", tmo, status == DMA_COMPLETE);
		if (0 == tmo ) {
			pr_err("<%s> Error: transfer timed out\n",
				XDMA_MODULE_NAME);
			ret = -1;
		} else if (status != DMA_COMPLETE) {
			pr_debug("<%s> transfer: returned completion callback status of: \'%s\'\n",
				XDMA_MODULE_NAME,
				status == DMA_ERROR ? "error" : "in progress");
			ret = -1;
			//We may distinguish between error or in progress
		} else {
			//may need to check if something went wrong before timeout
			ret = XDMA_DMA_TRANSFER_FINISHED;
		}
		//if wait is blocking, delete the completion
		kfree(cmp);
	}
	return ret;
}

static void xdma_stop_transfer(struct dma_chan *chan)
{
	if (chan) {
#if LINUX_KERNEL_VERSION_4XX
		dmaengine_terminate_all(chan);
#else
		chan->device->device_control(chan, DMA_TERMINATE_ALL,
			(unsigned long)NULL);
#endif
	}
}

static long xdma_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct xdma_dev xdma_dev;
	struct xdma_chan_cfg chan_cfg;
	struct xdma_buf_info buf_info;
	struct xdma_transfer trans;
	struct xdma_memcpy_info memcpy_info;
	u32 devices;
	struct dma_chan *chan;

	switch (cmd) {
	case XDMA_GET_NUM_DEVICES:
		PRINT_DBG("<%s> ioctl: XDMA_GET_NUM_DEVICES\n",
		       XDMA_MODULE_NAME);

		devices = num_devices;
		if (copy_to_user((u32 *) arg, &devices, sizeof(u32)))
			return -EFAULT;

		break;
	case XDMA_GET_DEV_INFO:
		PRINT_DBG("<%s> ioctl: XDMA_GET_DEV_INFO\n",
		       XDMA_MODULE_NAME);

		if (copy_from_user((void *)&xdma_dev,
				   (const void __user *)arg,
				   sizeof(struct xdma_dev)))
			return -EFAULT;

		xdma_get_dev_info(xdma_dev.device_id, &xdma_dev);

		if (copy_to_user((struct xdma_dev *)arg,
				 &xdma_dev, sizeof(struct xdma_dev)))
			return -EFAULT;

		break;
	case XDMA_DEVICE_CONTROL:
		PRINT_DBG("<%s> ioctl: XDMA_DEVICE_CONTROL\n",
		       XDMA_MODULE_NAME);

		if (copy_from_user((void *)&chan_cfg,
				   (const void __user *)arg,
				   sizeof(struct xdma_chan_cfg)))
			return -EFAULT;

		xdma_device_control(&chan_cfg);
		break;
	case XDMA_PREP_BUF:
		PRINT_DBG("<%s> ioctl: XDMA_PREP_BUF\n", XDMA_MODULE_NAME);

		if (copy_from_user((void *)&buf_info,
				   (const void __user *)arg,
				   sizeof(struct xdma_buf_info)))
			return -EFAULT;

		ret = (long)xdma_prep_buffer(&buf_info);

		if (copy_to_user((struct xdma_buf_info *)arg,
				 &buf_info, sizeof(struct xdma_buf_info)))
			return -EFAULT;

		break;
	case XDMA_START_TRANSFER:
		PRINT_DBG("<%s> ioctl: XDMA_START_TRANSFER\n",
		       XDMA_MODULE_NAME);

		if (copy_from_user((void *)&trans,
				   (const void __user *)arg,
				   sizeof(struct xdma_transfer)))
			return -EFAULT;

		ret = (long)xdma_start_transfer(&trans);
		break;
	case XDMA_STOP_TRANSFER:
		PRINT_DBG("<%s> ioctl: XDMA_STOP_TRANSFER\n",
		       XDMA_MODULE_NAME);

		if (copy_from_user((void *)&chan,
				   (const void __user *)arg, sizeof(u32)))
			return -EFAULT;

		xdma_stop_transfer((struct dma_chan *)chan);
		break;
	case XDMA_FINISH_TRANSFER:
		PRINT_DBG("<%s> ioctl: XDMA_FINISHED_TRANSFER\n",
		        XDMA_MODULE_NAME);
		if (copy_from_user((void *)&trans,
				   (const void __user *)arg,
				   sizeof(struct xdma_transfer)))
			return -EFAULT;
		ret = xdma_finish_transfer(&trans);
		break;
	case XDMA_PREP_USR_BUF:
		PRINT_DBG("<%s> ioctl; XDMA_PREP_USR_BUFFER\n", XDMA_MODULE_NAME);
		if (copy_from_user((void *)&buf_info,
					(const void __user *)arg,
					sizeof(struct xdma_buf_info)))
			return -EFAULT;

		ret = xdma_prep_user_buffer(&buf_info);

		if (copy_to_user((struct xdma_buf_info *)arg,
					&buf_info, sizeof(struct xdma_buf_info)))
			return -EFAULT;
		break;
	case XDMA_RELEASE_USR_BUF:
		PRINT_DBG("<%s> ioctl: XDMA_RELEASE_USR_BUFFER\n", XDMA_MODULE_NAME);
		// The user parameter is already a pointer to the xdma_sg_mem structure
		ret = xdma_user_buffer_release((struct xdma_sg_mem *)arg);

		break;
	case XDMA_PREP_MEMCPY:
		if (copy_from_user(&memcpy_info,
					(const void __user *)arg,
					sizeof(struct xdma_memcpy_info)))
			return -EFAULT;
		ret = xdma_prep_memcpy(&memcpy_info);
		if (copy_to_user((struct xdma_memcpy_info*)arg,
				&memcpy_info, sizeof(struct xdma_memcpy_info)))
			return -EFAULT;

		break;

	default:
		pr_warn("<%s> ioctl: WARNING unknown ioctl command %d\n", XDMA_MODULE_NAME, cmd);
		break;
	}

	return ret;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = xdma_open,
	.release = xdma_close,
	.read = xdma_read,
	.write = xdma_write,
	.unlocked_ioctl = xdma_ioctl,
};

static void xdma_add_dev_info(struct dma_chan *tx_chan,
	struct dma_chan *rx_chan)
{
	struct completion *tx_cmp, *rx_cmp;

	tx_cmp = (struct completion *)
		kzalloc(sizeof(struct completion), GFP_KERNEL);

	rx_cmp = (struct completion *)
		kzalloc(sizeof(struct completion), GFP_KERNEL);

	xdma_dev_info[num_devices] = (struct xdma_dev *)
		kzalloc(sizeof(struct xdma_dev), GFP_KERNEL);

	xdma_dev_info[num_devices]->tx_chan = tx_chan;
	xdma_dev_info[num_devices]->tx_cmp = tx_cmp;

	xdma_dev_info[num_devices]->rx_chan = rx_chan;
	xdma_dev_info[num_devices]->rx_cmp = rx_cmp;

	xdma_dev_info[num_devices]->device_id = num_devices;
	num_devices++;
}

#if LINUX_KERNEL_VERSION_4XX
static void xdma_init(void)
{
	struct dma_chan *tx_chan, *rx_chan;
	int i;
	char chan_to_name[CHAN_NAME_MAX_LEN];
	char chan_from_name[CHAN_NAME_MAX_LEN];
	int has_dma, num_acc;

	if (xdma_initialized) return;
	xdma_initialized = 1;

	//has_dma = bitinfo_dma_enabled();
	//num_acc = bitinfo_get_num_acc();
	has_dma = 0;
	num_acc = 0; //does not matter as has_dma == 0


	//init cdma engine
	cdma_channel = dma_request_slave_channel(&ompss_at_fpga_pdev->dev, "cdma0");
	if (!cdma_channel) {
		pr_warn("<" XDMA_MODULE_NAME "> No cdma devices found\n");
	}

	if (!has_dma) {
		pr_err("<%s> No dma engines in current bitstream\n",
				XDMA_MODULE_NAME);
		return;
	} else if (num_acc < 0) {
		pr_err("<%s> The number of accelerators with dma engines cannot be determined. "
				"Try reloading the kernel module or regenerate the bitstream with a newer Accelerator Integration Tool version\n",
				XDMA_MODULE_NAME);
		return;
	}
	for (i=0;i<num_acc;i++) {

		sprintf(chan_to_name, "acc%d_to_dev", i);
		sprintf(chan_from_name, "acc%d_from_dev", i);

		tx_chan = dma_request_slave_channel(&ompss_at_fpga_pdev->dev, chan_to_name);
		rx_chan = dma_request_slave_channel(&ompss_at_fpga_pdev->dev, chan_from_name);

		if (!tx_chan && !rx_chan) {
			pr_warn("<%s> probe: %d devices found of %d expected\n",
			       XDMA_MODULE_NAME, num_devices, num_acc);
			break;
		} else {
			PRINT_DBG("got channels tx: %p rx: %p\n", tx_chan, rx_chan);
			xdma_add_dev_info(tx_chan, rx_chan);
		}
	}
}
#else
static bool xdma_filter(struct dma_chan *chan, void *param)
{
	if (*((int *)chan->private) == *(int *)param)
		return true;

	return false;
}

static void xdma_init(void)
{
	dma_cap_mask_t mask;
	u32 match_tx, match_rx;
	struct dma_chan *tx_chan, *rx_chan;

	if (xdma_initialized) return;
	xdma_initialized = 1;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE | DMA_PRIVATE, mask);

	for (;;) {
		match_tx = (DMA_MEM_TO_DEV & 0xFF) | XILINX_DMA_IP_DMA |
			(num_devices << XILINX_DMA_DEVICE_ID_SHIFT);

		tx_chan = dma_request_channel(mask, xdma_filter,
			(void *)&match_tx);

		match_rx = (DMA_DEV_TO_MEM & 0xFF) | XILINX_DMA_IP_DMA |
			(num_devices << XILINX_DMA_DEVICE_ID_SHIFT);

		rx_chan = dma_request_channel(mask, xdma_filter,
			(void *)&match_rx);

		if (!tx_chan && !rx_chan) {
			pr_debug("<%s> probe: number of devices found: %d\n",
			       XDMA_MODULE_NAME, num_devices);
			break;
		} else {
			xdma_add_dev_info(tx_chan, rx_chan);
		}
	}
}
#endif

static void xdma_cleanup(void)
{
	int i;

	for (i = 0; i < num_devices; i++) {
		if (xdma_dev_info[i]) {
			if (xdma_dev_info[i]->tx_chan)
				dma_release_channel((struct dma_chan *)
					xdma_dev_info[i]->tx_chan);

			if (xdma_dev_info[i]->tx_cmp)
				kfree((struct completion *)
				      xdma_dev_info[i]->tx_cmp);

			if (xdma_dev_info[i]->rx_chan)
				dma_release_channel((struct dma_chan *)
				                    xdma_dev_info[i]->rx_chan);

			if (xdma_dev_info[i]->rx_cmp)
				kfree((struct completion *)
				      xdma_dev_info[i]->rx_cmp);
		}
	}

	if (cdma_channel) {
		dma_release_channel(cdma_channel);
	}

	num_devices = 0;
	xdma_initialized = 0;
}
#endif //else LINUX_KERNEL_VERSION_5XX

int xdma_probe(struct platform_device *pdev)
{
	struct device_node *xdma_node;
	num_devices = 0;
	xdma_opens_cnt = 0;

	//Save platform device structure for later use
	ompss_at_fpga_pdev = pdev;

	/* device constructor */
	if (alloc_chrdev_region(&dev_num, 0, 1 /*num_devices*/, XDMA_MODULE_NAME) < 0) {
		pr_err("<%s> Could not allocate region for xdma device\n",
			MODULE_NAME);
		goto xdma_alloc_chrdev_err;
	}
	if ((cl = class_create(THIS_MODULE, XDMA_MODULE_NAME)) == NULL) {
		pr_err("<%s> Could not create xdma device class\n",
			MODULE_NAME);
		goto xdma_class_err;
	}

	dma_dev = device_create(cl, &ompss_at_fpga_pdev->dev, dev_num, NULL,
			DEV_PREFIX "/" XDMA_DEV_NAME);
	if (dma_dev == NULL) {
		pr_err("<%s> Could not create xdma device\n",
			MODULE_NAME);
		goto xdma_dev_err;
	}
	cdev_init(&c_dev, &fops);
	if (cdev_add(&c_dev, dev_num, 1) == -1) {
		pr_err("<%s> Could not add xdma device\n",
			MODULE_NAME);
		goto xdma_cdev_err;
	}

	xdma_node = pdev->dev.of_node;

	return 0;

xdma_cdev_err:
	device_destroy(cl, dev_num);
xdma_dev_err:
	class_destroy(cl);
xdma_class_err:
	unregister_chrdev_region(dev_num, 1 /*num_devices*/);
xdma_alloc_chrdev_err:
	cl = NULL;
	return -1;
}

int xdma_remove(struct platform_device *pdev)
{
	if (xdma_opens_cnt != 0) {
		pr_info("<%s> remove: Opens counter is not zero\n", XDMA_MODULE_NAME);
	}

	if (cl == NULL) {
		//Device not initialized
		return 0;
	}

	/* device destructor */
	cdev_del(&c_dev);
	device_destroy(cl, dev_num);
	class_destroy(cl);
	unregister_chrdev_region(dev_num, 1 /*num_devices*/);

	return 0;
}
#undef PRINT_DBG
#undef DEBUG_PRINT
