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

#ifndef __OMPSS_FPGA_H__
#define __OMPSS_FPGA_H__

//#include <linux/dmaengine.h>
#include <linux/ioctl.h>
//#include <linux/platform_device.h>
#include <linux/types.h>

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#define MODULE_NAME    "ompss_fpga"

#define XDMA_CH_CFG_COALESC_DEF 1
#define XDMA_CH_CFG_DELAY_DEF   0
#define XDMA_CH_CFG_RESET_DEF   0

#define XDMA_IOCTL_BASE	'W'
#define XDMA_GET_NUM_DEVICES	_IO(XDMA_IOCTL_BASE, 0)
#define XDMA_GET_DEV_INFO	_IO(XDMA_IOCTL_BASE, 1)
#define XDMA_DEVICE_CONTROL	_IO(XDMA_IOCTL_BASE, 2)
#define XDMA_PREP_BUF		_IO(XDMA_IOCTL_BASE, 3)
#define XDMA_START_TRANSFER	_IO(XDMA_IOCTL_BASE, 4)
#define XDMA_STOP_TRANSFER	_IO(XDMA_IOCTL_BASE, 5)
/*#define XDMA_TEST_TRANSFER	_IO(XDMA_IOCTL_BASE, 6) Unused */
#define XDMA_FINISH_TRANSFER	_IO(XDMA_IOCTL_BASE, 7)
#define XDMA_PREP_USR_BUF	_IO(XDMA_IOCTL_BASE, 8)
#define XDMA_RELEASE_USR_BUF	_IO(XDMA_IOCTL_BASE, 9)
#define XDMAMEM_GET_LAST_KBUF	_IO(XDMA_IOCTL_BASE, 10)
#define XDMAMEM_RELEASE_KBUF	_IO(XDMA_IOCTL_BASE, 11)
#define XDMAMEM_GET_DMA_ADDRESS	_IO(XDMA_IOCTL_BASE, 12)
#define XDMA_PREP_MEMCPY        _IO(XDMA_IOCTL_BASE, 12)

#define HWCOUNTER_IOCTL_BASE	'I'
#define HWCOUNTER_GET_ADDR	_IOR(HWCOUNTER_IOCTL_BASE, 0, unsigned long)

//NOTE: The below defines are a hack that enables the use of kernel data types
//      without having to included standard kernel headers
//#include <linux/dmaengine.h>
#define u32 uint32_t
#define dma_cookie_t int32_t

	enum xdma_direction {
		XDMA_MEM_TO_DEV,
		XDMA_DEV_TO_MEM,
		XDMA_TRANS_NONE,
	};

	enum xdma_transfer_status {
		XDMA_DMA_TRANSFER_FINISHED = 0,
		XDMA_DMA_TRANSFER_PENDING,
	};

	struct xdma_dev {
		struct dma_chan *tx_chan;	/* (struct dma_chan *) */
		struct completion *tx_cmp;	/* (struct completion *) callback_param */
		struct dma_chan *rx_chan;	/* (struct dma_chan *) */
		struct completion *rx_cmp;	/* (struct completion *) callback_param */
		u32 device_id;
	};

	struct xdma_chan_cfg {
		struct dma_chan *chan;	/* (struct dma_chan *) */

		enum xdma_direction dir;	/* Channel direction */
		int coalesc;	/* Interrupt coalescing threshold */
		int delay;	/* Delay counter */
		int reset;	/* Reset Channel */
	};

	struct xdma_buf_info {
		struct dma_chan *chan;	/* (struct dma_chan *) */
		struct completion *completion;	/* (struct completion *) callback_param */

		dma_cookie_t cookie;
		unsigned long address;
		u32 buf_offset;
		u32 buf_size;
		enum xdma_direction dir;
		void *sg_transfer;	//internal type
	};

	struct xdma_transfer {
		struct dma_chan *chan;	/* (struct dma_chan *) */
		struct completion *completion;	/* (struct completion *) callback_param */

		dma_cookie_t cookie;
		u32 wait;	/* true/false */
		void *sg_transfer; /* pointer to internal SG structure */
	};

	struct xdma_memcpy_info {
		struct dma_chan *chan;
		struct completion *completion;
		dma_cookie_t cookie;
		unsigned long src_address;
		u32 src_offset;
		unsigned long dst_address;
		u32 dst_offset;
		u32 size;
		enum xdma_direction direction;
		void *sg_transfer; /* pointer to internal SG structure */
	};


#ifdef __cplusplus
}
#endif //__cplusplus
#endif //__OMPSS_FPGA_H__
