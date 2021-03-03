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
#include <linux/mm.h>

#include "ompss_fpga_common.h"

int generic_open(int *opens_cnt, const int max_opens, const char *name) {
	//Allow only one process using the device at the same time
	//NOTE: Not optimizing with initial check as f&a will almost always succed
	if (__sync_fetch_and_add(opens_cnt, 1) >= max_opens) {
		__sync_sub_and_fetch(opens_cnt, 1);
		pr_debug("<%s> open: Device '%s' opened more times than allowed (%d)\n",
			MODULE_NAME, name, max_opens);
		return -EBUSY;
	}
	pr_debug("<%s> Open '%s'\n", MODULE_NAME, name);
	return 0;
}

int generic_close(int *opens_cnt, const char *name) {
	int cnt = __sync_sub_and_fetch(opens_cnt, 1);
	if (cnt < 0) {
		pr_warn("<%s> close: "
			"Device '%s' has been closed more times than opened\n",
			MODULE_NAME, name);
		return -1;
	}
	pr_debug("<%s> Close '%s'\n", MODULE_NAME, name);
	return 0;
}

int generic_mmap(struct file *filp, struct vm_area_struct *vma, unsigned long io_addr, const char *name) {
	int res;
	unsigned long size;

	size = vma->vm_end - vma->vm_start;
	pr_debug("<%s> mmap '%s': addr 0x%lx, size %lu\n", MODULE_NAME, name, io_addr, size);

	vma->vm_page_prot = phys_mem_access_prot(
			filp, io_addr >> PAGE_SHIFT, size, vma->vm_page_prot);
	res = remap_pfn_range(vma, vma->vm_start,
			io_addr >> PAGE_SHIFT, size, vma->vm_page_prot);
	return res;
}

#if TARGET_64_BITS
int read_memspace(struct device_node *node, u64 *mem_space) {
	return of_property_read_u64_array(node, "reg", mem_space, 2);
}
#else
int read_memspace(struct device_node *node, u32 *mem_space) {
	return of_property_read_u32_array(node, "reg", mem_space, 2);
}
#endif

static int ompss_at_fpga_driver_probe(struct platform_device *pdev) {
	int ret_code = 0;
	pr_info("<%s> Module probe\n", MODULE_NAME);
	ret_code |= hwcounter_probe(pdev);
	ret_code |= xdma_probe(pdev);
	ret_code |= xdmamem_probe(pdev);
	ret_code |= bitinfo_probe(pdev);
	ret_code |= hwruntime_probe(pdev);
	return ret_code;
}

static int ompss_at_fpga_driver_remove(struct platform_device *pdev) {
	int ret_code = 0;
	pr_info("<%s> Module remove\n", MODULE_NAME);
	ret_code |= hwruntime_remove(pdev);
	ret_code |= bitinfo_remove(pdev);
	ret_code |= xdmamem_remove(pdev);
	ret_code |= xdma_remove(pdev);
	ret_code |= hwcounter_remove(pdev);
	return ret_code;
}

static const struct of_device_id ompss_at_fpga_of_ids[] = {
	{ .compatible = "ompss-at-fpga" },
	{}
};

static struct platform_driver ompss_at_fpga_platform_driver = {
	.driver = {
		.name = "OmpSs@FPGA device driver",
		.owner = THIS_MODULE,
		.of_match_table = ompss_at_fpga_of_ids,
	},
	.probe = ompss_at_fpga_driver_probe,
	.remove = ompss_at_fpga_driver_remove,
};

static int __init driver_probe(void)
{
	platform_driver_register(&ompss_at_fpga_platform_driver);
	return 0;
}

static void __exit driver_exit(void)
{
	platform_driver_unregister(&ompss_at_fpga_platform_driver);
}

module_init(driver_probe);
module_exit(driver_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("OmpSs@FPGA Team <ompss-fpga-support@bsc.es>");
MODULE_DESCRIPTION("OmpSs@FPGA Linux device driver");
