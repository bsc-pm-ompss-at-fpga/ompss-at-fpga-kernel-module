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

#ifndef __OMPSS_FPGA_COMMON_H__
#define __OMPSS_FPGA_COMMON_H__

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "ompss_fpga.h"

#define DEV_PREFIX "ompss_fpga"

#define LINUX_KERNEL_VERSION_4XX (LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0) \
	&& LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))
#define LINUX_KERNEL_VERSION_3XX (LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0) \
	&& LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0))
#define TARGET_64_BITS (defined(CONFIG_64BIT))

int hwcounter_probe(struct platform_device *pdev);
int hwcounter_remove(struct platform_device *pdev);

int xdma_probe(struct platform_device *pdev);
int xdma_remove(struct platform_device *pdev);

int xdmamem_probe(struct platform_device *pdev);
int xdmamem_remove(struct platform_device *pdev);

int bitinfo_probe(struct platform_device *pdev);
int bitinfo_remove(struct platform_device *pdev);

int hwruntime_probe(struct platform_device *pdev);
int hwruntime_remove(struct platform_device *pdev);

int generic_open(int *opens_cnt, const int max_opens, const char *name);
int generic_close(int *opens_cnt, const char *name);

int generic_mmap(struct file *filp, struct vm_area_struct *vma,
		unsigned long io_addr, const char *name);

int bitinfo_get_rev(void);
int bitinfo_get_num_acc(void);
int bitinfo_dma_enabled(void);
u32 * get_n_field_ptr(const u8 n);
int read_hwruntime_addr_from_bitinfo(int rev, const char* phandle_name, int extended_queue, int bitinfo_addr_offset, unsigned long* dev_mem_space);
int read_hwcounter_addr_from_bitinfo(int rev, unsigned long* hwcounter_phy_addr);

#if TARGET_64_BITS
int read_memspace(struct device_node *node, u64 *mem_space);
#else
int read_memspace(struct device_node *node, u32 *mem_space);
#endif

#define BITINFO_MIN_DYNAMIC_REV       7

#define CMD_IN_BITINFO_ADDR_OFFSET    0
#define CMD_IN_BITINFO_LEN_OFFSET     2
#define CMD_OUT_BITINFO_ADDR_OFFSET   3
#define CMD_OUT_BITINFO_LEN_OFFSET    5
#define SPWN_IN_BITINFO_ADDR_OFFSET   6
#define SPWN_IN_BITINFO_LEN_OFFSET    8
#define SPWN_OUT_BITINFO_ADDR_OFFSET  9
#define SPWN_OUT_BITINFO_LEN_OFFSET   11
#define RST_BITINFO_ADDR_OFFSET       12
#define HWCOUNTER_BITINFO_ADDR_OFFSET 14

#endif //__OMPSS_FPGA_COMMON_H__
