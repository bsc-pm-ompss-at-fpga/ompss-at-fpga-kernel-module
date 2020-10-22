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

#define BITINFO_MODULE_NAME        "ompss_fpga_bitinfo"
#define BITINFO_DEV_DIR            "bit_info"
#define BITINFO_PHANDLE_NAME       "bitstreaminfo"

#define BITINFO_REV_NAME           "rev"
#define BITINFO_NUMACCS_NAME       "num_accs"
#define BITINFO_XTASKS_NAME        "xtasks"
#define BITINFO_F_RAW_NAME         "features/raw"
#define BITINFO_F_INS_NAME         "features/hwcounter"
#define BITINFO_F_DMA_NAME         "features/dma"
#define BITINFO_F_INOPT_NAME       "features/intercon_opt"
#define BITINFO_F_INLEV_NAME       "features/intercon_level"
#define BITINFO_F_HWR_NAME         "features/hwruntime"
#define BITINFO_F_EHWR_NAME        "features/hwruntime_ext"
#define BITINFO_F_SOM_NAME         "features/hwruntime_som"
#define BITINFO_F_POM_NAME         "features/hwruntime_pom"
#define BITINFO_CALL_NAME          "ait_call"
#define BITINFO_AV_NAME            "ait_version"
#define BITINFO_WRAPPER_NAME       "wrapper_version"
#define BITINFO_HWR_VLNV_NAME      "hwruntime_vlnv"
#define BITINFO_BASE_FREQ_NAME     "base_freq"
#define BITINFO_RAW_NAME           "raw"
#define BITINFO_FIELD_SEP          0xFFFFFFFF
#define BITINFO_MAX_WORDS          512

#define BITINFO_NUM_DEVICES        18
#define BITINFO_VER_MINOR          0
#define BITINFO_NUMACCS_MINOR      1
#define BITINFO_XTASKS_MINOR       2
#define BITINFO_F_RAW_MINOR        5
#define BITINFO_F_INS_MINOR        6
#define BITINFO_F_DMA_MINOR        9
#define BITINFO_F_INOPT_MINOR      11
#define BITINFO_F_INLEV_MINOR      10
#define BITINFO_F_HWR_MINOR        7
#define BITINFO_F_EHWR_MINOR       8
#define BITINFO_F_SOM_MINOR        15
#define BITINFO_F_POM_MINOR        14
#define BITINFO_CALL_MINOR         3
#define BITINFO_AV_MINOR           4
#define BITINFO_WRAPPER_MINOR      13
#define BITINFO_HWR_VLNV_MINOR     16
#define BITINFO_BASE_FREQ_MINOR    17
#define BITINFO_RAW_MINOR          12

#define BITINFO_REV_IDX            0
#define BITINFO_MIN_REV            4
#define BITINFO_MAX_REV            6
#define BITINFO_F_DMA_IDX          1

//NOTE: The value '0' means that the information is not available in those revision
//NOTE: Revision '0' is not valid and it is used as a control revision
const u8 BITINFO_NUMACCS_IDX[]     = {0, 0, 1, 1, 1, 1, 1};
const u8 BITINFO_XTASKS_IDX[]      = {0, 1, 2, 2, 2, 2, 2};
const u8 BITINFO_F_RAW_IDX[]       = {0, 2, 3, 3, 3, 3, 3};
const u8 BITINFO_CALL_IDX[]        = {0, 3, 4, 4, 4, 4, 4};
const u8 BITINFO_AV_IDX[]          = {0, 0, 0, 0, 5, 5, 5};
const u8 BITINFO_WRAPPER_IDX[]     = {0, 0, 0, 5, 6, 6, 6};
const u8 BITINFO_HWR_VLNV_IDX[]    = {0, 0, 0, 0, 0, 7, 7};
const u8 BITINFO_BASE_FREQ_IDX[]   = {0, 0, 0, 0, 0, 0, 8};

static dev_t bitinfo_devt;
static struct class *bitinfo_cl;
static struct cdev bitinfo_rev_cdev, bitinfo_numaccs_cdev, bitinfo_xtasks_cdev, bitinfo_f_raw_cdev,
	bitinfo_f_ins_cdev, bitinfo_f_dma_cdev, bitinfo_f_inopt_cdev, bitinfo_f_inlev_cdev,
	bitinfo_f_hwr_cdev, bitinfo_f_ehwr_cdev, bitinfo_f_som_cdev, bitinfo_f_pom_cdev,
	bitinfo_call_cdev, bitinfo_ait_cdev, bitinfo_wrapper_cdev, bitinfo_hwr_vlnv_cdev,
        bitinfo_base_freq_cdev, bitinfo_raw_cdev;

static struct device *bitinfo_rev_dev, *bitinfo_numaccs_dev, *bitinfo_xtasks_dev, *bitinfo_f_raw_dev,
	*bitinfo_f_ins_dev, *bitinfo_f_dma_dev, *bitinfo_f_inopt_dev, *bitinfo_f_inlev_dev,
	*bitinfo_f_hwr_dev, *bitinfo_f_ehwr_dev, *bitinfo_f_som_dev, *bitinfo_f_pom_dev,
	*bitinfo_call_dev, *bitinfo_ait_dev, *bitinfo_wrapper_dev, *bitinfo_hwr_vlnv_dev,
        *bitinfo_base_freq_dev, *bitinfo_raw_dev;

static int bitinfo_rev_opens_cnt;         // Opens counter of revision device
static int bitinfo_numaccs_opens_cnt;     // Opens counter of num_accs device
static int bitinfo_xtasks_opens_cnt;      // Opens counter of xtasks device
static int bitinfo_f_raw_opens_cnt;       // Opens counter of features/raw device
static int bitinfo_f_ins_opens_cnt;       // Opens counter of features/hwcounter device
static int bitinfo_f_dma_opens_cnt;       // Opens counter of features/dma device
static int bitinfo_f_inopt_opens_cnt;     // Opens counter of features/intercon_opt device
static int bitinfo_f_inlev_opens_cnt;     // Opens counter of features/intercon_level device
static int bitinfo_f_hwr_opens_cnt;       // Opens counter of features/hwruntime device
static int bitinfo_f_ehwr_opens_cnt;      // Opens counter of features/hwruntime_ext device
static int bitinfo_f_som_opens_cnt;       // Opens counter of features/hwruntime_som device
static int bitinfo_f_pom_opens_cnt;     // Opens counter of features/hwruntime_pom device
static int bitinfo_call_opens_cnt;        // Opens counter of ait_call device
static int bitinfo_ait_opens_cnt;         // Opens counter of ait_version device
static int bitinfo_wrapper_opens_cnt;     // Opens counter of wrapper_version device
static int bitinfo_hwr_vlnv_opens_cnt;    // Opens counter of hwruntime_vlnv device
static int bitinfo_base_freq_opens_cnt;   // Opens counter of base_freq device
static int bitinfo_raw_opens_cnt;         // Opens counter of raw device
static int bitinfo_major;

void __iomem * bitinfo_io_addr;

static u32 * get_n_field_ptr(const u8 n) {
	u8 num_sep;
	u32 * ptr, * max_ptr;

	if (n == 0) {
		//0th field: starts at the begining
		ptr = (u32 *)bitinfo_io_addr;
	} else if (n == 1) {
		//1st field: starts at the 2nd word
		ptr = (u32 *)(bitinfo_io_addr) + 2;
	} else {
		num_sep = 1;
		ptr = (u32 *)(bitinfo_io_addr) + 3 /*start after 1st field*/;
		max_ptr = (u32 *)(bitinfo_io_addr) + BITINFO_MAX_WORDS; //< Put some limit to avoid spin forever over wrong data

		while (num_sep < n && ptr < max_ptr) {
			if (readl(ptr++) == BITINFO_FIELD_SEP) num_sep++;
		}
		if (ptr >= max_ptr) {
			ptr = (u32 *)0;
			printk(KERN_ERR "<%s> Uncontrolled path in get_n_field_ptr. n=%u,num_sep=%u\n",
				MODULE_NAME, (unsigned int)n, (unsigned int)num_sep);
		}
	}

	return ptr;
}

int bitinfo_get_rev() {
	//NOTE: If the read value seems invalid return a 0
	const int rev = readl(get_n_field_ptr(BITINFO_REV_IDX));
	if (rev < BITINFO_MIN_REV || rev > BITINFO_MAX_REV) {
		printk(KERN_ERR "<%s> Unsupported bitinfo revision %d. Supported versions are [%d, %d]\n",
			MODULE_NAME, rev, BITINFO_MIN_REV, BITINFO_MAX_REV);
		return 0;
	} else {
		return rev;
	}
}

int bitinfo_get_num_acc() {
	const u8 field_idx = BITINFO_NUMACCS_IDX[bitinfo_get_rev()];
	//NOTE: If field_idx is 0, the number of accelerators field is not available
	return field_idx != 0 ? readl(get_n_field_ptr(field_idx)) : -1;
}

int bitinfo_dma_enabled() {
	u8 field_idx;
	u32 * field_ptr;

	field_idx = BITINFO_F_RAW_IDX[bitinfo_get_rev()];
	if (field_idx == 0) return -1;
	field_ptr = get_n_field_ptr(field_idx);
	return field_ptr ? ((readl(field_ptr)>>BITINFO_F_DMA_IDX)&1 /*Select DMA capability bit*/) : 0;
}

static ssize_t copy_field_data(u32 *data_ptr, char __user *buffer, size_t length, loff_t *offset) {
	u32 data;
	u32 * max_ptr;
	u8 * char_data_ptr;
	size_t data_cnt, write_cnt;

	//Ensure that the field has been found
	if (!data_ptr || data_ptr == bitinfo_io_addr) return 0;

	write_cnt = 0;
	data_ptr += (*offset)/sizeof(u32);

	//Align the offset to 32b words
	data = readl(data_ptr++);
	data_cnt = (*offset)%sizeof(u32);
	if (data_cnt != 0 && data != BITINFO_FIELD_SEP) {
		char_data_ptr = (u8*)(&data) + (sizeof(u32) - data_cnt);
		if (copy_to_user(buffer, char_data_ptr, data_cnt)) return -1;
		buffer += data_cnt;
		length -= data_cnt;
		write_cnt += data_cnt;
	}

	//Read as much 32b words as possible
	max_ptr = (u32 *)(bitinfo_io_addr) + BITINFO_MAX_WORDS;
	while (length >= sizeof(u32) && data != BITINFO_FIELD_SEP && data_ptr < max_ptr) {
		if (copy_to_user(buffer, &data, sizeof(u32))) return -1;
		buffer += sizeof(u32);
		length -= sizeof(u32);
		write_cnt += sizeof(u32);

		//Read next line
		data = readl(data_ptr++);
	}

	//Put the string terminator if reached the end of field data and not wrote before
	//NOTE: Once the string terminator has been wrote, the offset will not be aligned anymore
	if (data == BITINFO_FIELD_SEP && length > 0 && data_cnt == 0) {
		char_data_ptr = (u8 *)&data;
		*char_data_ptr = '\0';
		if (copy_to_user(buffer, char_data_ptr, sizeof(char))) return -1;
		buffer++;
		length--;
		write_cnt++;
	}

	*offset += write_cnt;
	return write_cnt;
}

static ssize_t copy_field_int32(int value, char __user *buffer, size_t length, loff_t *offset) {
	size_t data_cnt, write_cnt;
	//NOTE: u32 can be represented with 10 chars + 1 endl + 1 string terminator
	char data_buffer[12];
	int num_accs;

	//NOTE: Using negative values as unsupported
	if (value < 0) {
		data_cnt = sprintf(data_buffer, "?\n") + 1 /*string terminator*/;
	} else {
		data_cnt = sprintf(data_buffer, "%d\n", value) + 1 /*string terminator*/;
	}

	//Copy the string to user buffer
	if (*offset > data_cnt) return 0;
	write_cnt =  min(length, (size_t)(data_cnt - *offset));
	write_cnt -= copy_to_user(buffer, &data_buffer[*offset], write_cnt);
	*offset += write_cnt;

	return write_cnt;
}

static ssize_t bitinfo_f_n_read(const size_t first_bit_idx, const size_t num_bits, char __user *buffer, size_t length, loff_t *offset) {
	u32 data;
	u32 * field_ptr;
	size_t i, write_cnt;
	//NOTE: num_bits of bitmask + 1 endl + 1 string terminator
	char data_buffer[num_bits + 2];
	const u8 field_idx = BITINFO_F_RAW_IDX[bitinfo_get_rev()];

	if (*offset > (num_bits + 2) || field_idx == 0) return 0;

	field_ptr = get_n_field_ptr(field_idx);
	if (!field_ptr) return 0;

	data = readl(field_ptr);
	for (i = 0; i < num_bits; i++) {
		data_buffer[i] = '0' + ((data&BIT(first_bit_idx + num_bits - i - 1)) != 0);
	}
	data_buffer[num_bits] = '\n';
	data_buffer[num_bits + 1] = '\0';

	//Copy the string to user buffer
	write_cnt  = min(length, (size_t)(num_bits + 2 - *offset));
	write_cnt -= copy_to_user(buffer, &data_buffer[*offset], write_cnt);
	*offset += write_cnt;

	return write_cnt;
}

static int bitinfo_rev_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_rev_opens_cnt, 1000 /*max_opens*/, BITINFO_REV_NAME);
}

static int bitinfo_rev_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_rev_opens_cnt, BITINFO_REV_NAME);
}

static ssize_t bitinfo_rev_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	return copy_field_int32(bitinfo_get_rev(), buffer, length, offset);
}

static int bitinfo_numaccs_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_numaccs_opens_cnt, 1000 /*max_opens*/, BITINFO_NUMACCS_NAME);
}

static int bitinfo_numaccs_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_numaccs_opens_cnt, BITINFO_NUMACCS_NAME);
}

static ssize_t bitinfo_numaccs_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	return copy_field_int32(bitinfo_get_num_acc(), buffer, length, offset);
}

static int bitinfo_xtasks_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_xtasks_opens_cnt, 1000 /*max_opens*/, BITINFO_XTASKS_NAME);
}

static int bitinfo_xtasks_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_xtasks_opens_cnt, BITINFO_XTASKS_NAME);
}

static ssize_t bitinfo_xtasks_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	const u8 field_idx = BITINFO_XTASKS_IDX[bitinfo_get_rev()];
	return copy_field_data(get_n_field_ptr(field_idx), buffer, length, offset);
}

static int bitinfo_call_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_call_opens_cnt, 1000 /*max_opens*/, BITINFO_CALL_NAME);
}

static int bitinfo_call_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_call_opens_cnt, BITINFO_CALL_NAME);
}

static ssize_t bitinfo_call_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	const u8 field_idx = BITINFO_CALL_IDX[bitinfo_get_rev()];
	return copy_field_data(get_n_field_ptr(field_idx), buffer, length, offset);
}

static int bitinfo_ait_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_ait_opens_cnt, 1000 /*max_opens*/, BITINFO_AV_NAME);
}

static int bitinfo_ait_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_ait_opens_cnt, BITINFO_AV_NAME);
}

static ssize_t bitinfo_ait_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	size_t data_cnt, write_cnt;
	//NOTE: u16 can be represented with 5 chars (x2) + 1 dot + 1 endl + 1 string terminator
	char data_buffer[13];
	u32 major, minor;
	u32 * field_ptr;
	u8 field_idx;

	field_idx = BITINFO_AV_IDX[bitinfo_get_rev()];
	field_ptr = get_n_field_ptr(field_idx);
	if (field_idx == 0 || !field_ptr) {
		//NOTE: The field is not available in the currently loaded bitstream
		data_cnt = sprintf(data_buffer, "?\n") + 1 /*string terminator*/;
	} else {
		minor = *field_ptr;
		major = (minor >> 16)&0xFFFF;
		minor = minor&0xFFFF;
		data_cnt = sprintf(data_buffer, "%u.%u\n", major, minor) + 1 /*string terminator*/;
	}

	//Copy the string to user buffer
	if (*offset > data_cnt) return 0;
	write_cnt =  min(length, (size_t)(data_cnt - *offset));
	write_cnt -= copy_to_user(buffer, &data_buffer[*offset], write_cnt);
	*offset += write_cnt;

	return write_cnt;
}

static int bitinfo_hwr_vlnv_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_hwr_vlnv_opens_cnt, 1000 /*max_opens*/, BITINFO_HWR_VLNV_NAME);
}

static int bitinfo_hwr_vlnv_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_hwr_vlnv_opens_cnt, BITINFO_HWR_VLNV_NAME);
}

static ssize_t bitinfo_hwr_vlnv_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	const u8 field_idx = BITINFO_HWR_VLNV_IDX[bitinfo_get_rev()];
	return copy_field_data(get_n_field_ptr(field_idx), buffer, length, offset);
}

static int bitinfo_base_freq_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_base_freq_opens_cnt, 1000 /*max_opens*/, BITINFO_BASE_FREQ_NAME);
}

static int bitinfo_base_freq_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_base_freq_opens_cnt, BITINFO_BASE_FREQ_NAME);
}

static ssize_t bitinfo_base_freq_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	const u8 field_idx = BITINFO_BASE_FREQ_IDX[bitinfo_get_rev()];
	int base_freq = field_idx != 0 ? readl(get_n_field_ptr(field_idx)) : -1;
	return copy_field_int32(base_freq, buffer, length, offset);
}

static int bitinfo_f_raw_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_f_raw_opens_cnt, 1000 /*max_opens*/, BITINFO_F_RAW_NAME);
}

static int bitinfo_f_raw_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_f_raw_opens_cnt, BITINFO_F_RAW_NAME);
}

static ssize_t bitinfo_f_raw_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	u32 * data_ptr;
	size_t data_cnt, write_cnt;
	//NOTE: "0x" + 16 bits of bitmask + 1 endl + 1 string terminator
	char data_buffer[20];
	const u8 field_idx = BITINFO_F_RAW_IDX[bitinfo_get_rev()];

	if (field_idx == 0) return 0;
	data_ptr = get_n_field_ptr(field_idx);
	if (!data_ptr) return 0;

	data_cnt = sprintf(data_buffer, "0x%016x\n", readl(data_ptr)) + 1 /*string terminator*/;
	if (*offset > data_cnt) return 0;

	//Copy the string to user buffer
	write_cnt =  min(length, (size_t)(data_cnt - *offset));
	write_cnt -= copy_to_user(buffer, &data_buffer[*offset], write_cnt);
	*offset += write_cnt;

	return write_cnt;
}

static int bitinfo_f_ins_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_f_ins_opens_cnt, 1000 /*max_opens*/, BITINFO_F_INS_NAME);
}

static int bitinfo_f_ins_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_f_ins_opens_cnt, BITINFO_F_INS_NAME);
}

static ssize_t bitinfo_f_ins_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	return bitinfo_f_n_read(0 /*bit_idx for hwcounter feature*/, 1, buffer, length, offset);
}

static int bitinfo_f_dma_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_f_dma_opens_cnt, 1000 /*max_opens*/, BITINFO_F_DMA_NAME);
}

static int bitinfo_f_dma_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_f_dma_opens_cnt, BITINFO_F_DMA_NAME);
}

static ssize_t bitinfo_f_dma_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	return bitinfo_f_n_read(1 /*bit_idx for dma feature*/, 1, buffer, length, offset);
}

static int bitinfo_f_inopt_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_f_inopt_opens_cnt, 1000 /*max_opens*/, BITINFO_F_INOPT_NAME);
}

static int bitinfo_f_inopt_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_f_inopt_opens_cnt, BITINFO_F_INOPT_NAME);
}

static ssize_t bitinfo_f_inopt_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	return bitinfo_f_n_read(2 /*bit_idx for intercon_opt feature*/, 1, buffer, length, offset);
}

static int bitinfo_f_inlev_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_f_inlev_opens_cnt, 1000 /*max_opens*/, BITINFO_F_INLEV_NAME);
}

static int bitinfo_f_inlev_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_f_inlev_opens_cnt, BITINFO_F_INLEV_NAME);
}

static ssize_t bitinfo_f_inlev_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	return bitinfo_f_n_read(3 /*bit_idx for intercon_level feature*/, 2, buffer, length, offset);
}

static int bitinfo_f_hwr_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_f_hwr_opens_cnt, 1000 /*max_opens*/, BITINFO_F_HWR_NAME);
}

static int bitinfo_f_hwr_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_f_hwr_opens_cnt, BITINFO_F_HWR_NAME);
}

static ssize_t bitinfo_f_hwr_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	return bitinfo_f_n_read(6 /*bit_idx for hwruntime feature*/, 1, buffer, length, offset);
}

static int bitinfo_f_ehwr_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_f_ehwr_opens_cnt, 1000 /*max_opens*/, BITINFO_F_EHWR_NAME);
}

static int bitinfo_f_ehwr_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_f_ehwr_opens_cnt, BITINFO_F_EHWR_NAME);
}

static ssize_t bitinfo_f_ehwr_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	return bitinfo_f_n_read(7 /*bit_idx for extended hwruntime feature*/, 1, buffer, length, offset);
}

static int bitinfo_f_som_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_f_som_opens_cnt, 1000 /*max_opens*/, BITINFO_F_POM_NAME);
}

static int bitinfo_f_som_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_f_som_opens_cnt, BITINFO_F_POM_NAME);
}

static ssize_t bitinfo_f_som_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	return bitinfo_f_n_read(8 /*bit_idx for som feature*/, 1, buffer, length, offset);
}

static int bitinfo_f_pom_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_f_pom_opens_cnt, 1000 /*max_opens*/, BITINFO_F_POM_NAME);
}

static int bitinfo_f_pom_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_f_pom_opens_cnt, BITINFO_F_POM_NAME);
}

static ssize_t bitinfo_f_pom_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	return bitinfo_f_n_read(9 /*bit_idx for pom feature*/, 1, buffer, length, offset);
}

static int bitinfo_raw_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_raw_opens_cnt, 1000 /*max_opens*/, BITINFO_RAW_NAME);
}

static int bitinfo_raw_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_raw_opens_cnt, BITINFO_RAW_NAME);
}

static ssize_t bitinfo_raw_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	u32 data;
	u32 * data_ptr, * max_ptr;
	size_t write_cnt; //< Number of words write
	u8 sep_cnt;       //< Number of consecutive separators found

	//NOTE: The offset variable contains the real offset and the number of separators found
	//      at the end of last read. This supports the case when the 2 last separators fall into
	//      different reads.
	//      offset format: [ num_separators (8b) | (8b) | offset (16b) ]
	write_cnt = (*offset)&0xFFFF;
	data_ptr = (u32 *)(bitinfo_io_addr) + (write_cnt)/sizeof(u32);
	sep_cnt = ((*offset) >> 24)&0xFF;
	write_cnt = 0;

	//Read as much 32b words as possible
	max_ptr = (u32 *)(bitinfo_io_addr) + BITINFO_MAX_WORDS; //< Put some limit to avoid spin forever over wrong data
	while (length >= sizeof(u32) && sep_cnt < 2 && data_ptr < max_ptr) {
		data = readl(data_ptr++);
		if (data == BITINFO_FIELD_SEP) {
			char sep[] = {'\n', '=', '=', '\n'};
			if (sep_cnt >= 1) {
				sep[3] = '\0';
			}
			if (copy_to_user(buffer, sep, sizeof(u32))) return -1;
			sep_cnt++;
		} else {
			if (copy_to_user(buffer, &data, sizeof(u32))) return -1;
			sep_cnt = 0;
		}
		buffer += sizeof(u32);
		length -= sizeof(u32);
		write_cnt += sizeof(u32);
	}

	//*offset += write_cnt;
	data = sep_cnt;
	*offset = (data << 24) | ((*offset + write_cnt)&0xFFFF);
	return write_cnt;
}

static ssize_t bitinfo_raw_write(struct file *filp, const char __user *buffer, size_t length, loff_t *offset) {
	u32 * data_ptr;
	ssize_t write_cnt; //< Number of bytes to move

	write_cnt = min(length, (size_t)(BITINFO_MAX_WORDS*sizeof(u32) - *offset));
	if (write_cnt < 0 || write_cnt > BITINFO_MAX_WORDS*sizeof(u32)) return 0;

	data_ptr = (u32 *)(bitinfo_io_addr) + (*offset)/sizeof(u32);
	if (copy_from_user(data_ptr, buffer, write_cnt)) return -EFAULT;

	*offset += write_cnt;
	return write_cnt;
}

static int bitinfo_wrapper_open(struct inode *i, struct file *f) {
	return generic_open(&bitinfo_wrapper_opens_cnt, 1000 /*max_opens*/, BITINFO_WRAPPER_NAME);
}

static int bitinfo_wrapper_close(struct inode *i, struct file *f) {
	return generic_close(&bitinfo_wrapper_opens_cnt, BITINFO_WRAPPER_NAME);
}

static ssize_t bitinfo_wrapper_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
	size_t data_cnt, write_cnt;
	//NOTE: u32 can be represented with 10 chars + 1 endl + 1 string terminator
	char data_buffer[12];
	u8 field_idx;
	u32 * field_ptr;

	field_idx = BITINFO_WRAPPER_IDX[bitinfo_get_rev()];
	field_ptr = get_n_field_ptr(field_idx);
	if (field_idx == 0 || !field_ptr) {
		//NOTE: The wrapper version information is not available in the currently loaded bitstream
		data_cnt = sprintf(data_buffer, "?\n") + 1 /*string terminator*/;
	} else {
		data_cnt = sprintf(data_buffer, "%u\n", *field_ptr) + 1 /*string terminator*/;
	}

	//Copy the string to user buffer
	if (*offset > data_cnt) return 0;
	write_cnt =  min(length, (size_t)(data_cnt - *offset));
	write_cnt -= copy_to_user(buffer, &data_buffer[*offset], write_cnt);
	*offset += write_cnt;

	return write_cnt;
}

static struct file_operations bitinfo_rev_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_rev_open,
	.release = bitinfo_rev_close,
	.read = bitinfo_rev_read,
};
static struct file_operations bitinfo_numaccs_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_numaccs_open,
	.release = bitinfo_numaccs_close,
	.read = bitinfo_numaccs_read,
};
static struct file_operations bitinfo_xtasks_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_xtasks_open,
	.release = bitinfo_xtasks_close,
	.read = bitinfo_xtasks_read,
};
static struct file_operations bitinfo_call_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_call_open,
	.release = bitinfo_call_close,
	.read = bitinfo_call_read,
};
static struct file_operations bitinfo_ait_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_ait_open,
	.release = bitinfo_ait_close,
	.read = bitinfo_ait_read,
};
static struct file_operations bitinfo_f_raw_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_f_raw_open,
	.release = bitinfo_f_raw_close,
	.read = bitinfo_f_raw_read,
};
static struct file_operations bitinfo_f_ins_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_f_ins_open,
	.release = bitinfo_f_ins_close,
	.read = bitinfo_f_ins_read,
};
static struct file_operations bitinfo_f_dma_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_f_dma_open,
	.release = bitinfo_f_dma_close,
	.read = bitinfo_f_dma_read,
};
static struct file_operations bitinfo_f_inopt_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_f_inopt_open,
	.release = bitinfo_f_inopt_close,
	.read = bitinfo_f_inopt_read,
};
static struct file_operations bitinfo_f_inlev_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_f_inlev_open,
	.release = bitinfo_f_inlev_close,
	.read = bitinfo_f_inlev_read,
};
static struct file_operations bitinfo_f_hwr_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_f_hwr_open,
	.release = bitinfo_f_hwr_close,
	.read = bitinfo_f_hwr_read,
};
static struct file_operations bitinfo_f_ehwr_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_f_ehwr_open,
	.release = bitinfo_f_ehwr_close,
	.read = bitinfo_f_ehwr_read,
};
static struct file_operations bitinfo_f_som_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_f_som_open,
	.release = bitinfo_f_som_close,
	.read = bitinfo_f_som_read,
};
static struct file_operations bitinfo_f_pom_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_f_pom_open,
	.release = bitinfo_f_pom_close,
	.read = bitinfo_f_pom_read,
};
static struct file_operations bitinfo_hwr_vlnv_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_hwr_vlnv_open,
	.release = bitinfo_hwr_vlnv_close,
	.read = bitinfo_hwr_vlnv_read,
};
static struct file_operations bitinfo_base_freq_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_base_freq_open,
	.release = bitinfo_base_freq_close,
	.read = bitinfo_base_freq_read,
};
static struct file_operations bitinfo_raw_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_raw_open,
	.release = bitinfo_raw_close,
	.read = bitinfo_raw_read,
	.write = bitinfo_raw_write,
};
static struct file_operations bitinfo_wrapper_fops = {
	.owner = THIS_MODULE,
	.open = bitinfo_wrapper_open,
	.release = bitinfo_wrapper_close,
	.read = bitinfo_wrapper_read,
};

static int bitinfo_map_io(struct device_node *bitinfo_node) {

	int status;
	struct device_node *dev_node;
#if TARGET_64_BITS
	u64 dev_mem_space[2];	//address & size
#else
	u32 dev_mem_space[2];
#endif

	dev_node = of_parse_phandle(bitinfo_node, BITINFO_PHANDLE_NAME, 0);
	if (dev_node != NULL) {
		status = read_memspace(dev_node, dev_mem_space);
		of_node_put(dev_node);
		if (status < 0) {
			printk(KERN_WARNING "<%s> Could not read bitstream information BRAM address\n", MODULE_NAME);
			goto bitinfo_phandle_of_err;
		}
		//register space in virtual kernel space
		bitinfo_io_addr = ioremap((resource_size_t)dev_mem_space[0],
				(size_t)dev_mem_space[1]);
	} else {
		printk(KERN_INFO "<%s> Bitstream information BRAM not available\n", MODULE_NAME);
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
		printk("<%s> Error mapping bitstream info BRAM IO\n", MODULE_NAME);
		goto bitinfo_map_io_err;
	}

	if (bitinfo_io_addr == NULL) {
		//Bitinfo BRAM not available
		bitinfo_cl = NULL;
		return 0;
	}

	//Create the device class
	if (alloc_chrdev_region(&bitinfo_devt, 0, BITINFO_NUM_DEVICES, BITINFO_MODULE_NAME) < 0) {
		printk(KERN_ERR "<%s> Could not allocate region for bitstream info devices\n",
			MODULE_NAME);
		goto bitinfo_alloc_chrdev;
	}
	bitinfo_major = MAJOR(bitinfo_devt);

	bitinfo_cl = class_create(THIS_MODULE, BITINFO_MODULE_NAME);
	if (bitinfo_cl == NULL) {
		printk(KERN_ERR "<%s> Could not create bitstream info device class\n",
			MODULE_NAME);
		goto bitinfo_class_err;
	}

	//Create device for the rev information
	bitinfo_rev_dev = device_create(bitinfo_cl, NULL, bitinfo_devt, NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_REV_NAME);
	if (IS_ERR(bitinfo_rev_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_REV_NAME);
		goto bitinfo_rev_dev_err;
	}
	cdev_init(&bitinfo_rev_cdev, &bitinfo_rev_fops);
	if (cdev_add(&bitinfo_rev_cdev, bitinfo_devt, 1)<0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_REV_NAME);
		goto bitinfo_rev_cdev_err;
	}
	bitinfo_rev_opens_cnt = 0;

	//Create device for the num_accs information
	//NOTE: This device is only available in versions >= 2. Creating it anyway, it returns "?" if not supported
	bitinfo_numaccs_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_NUMACCS_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_NUMACCS_NAME);
	if (IS_ERR(bitinfo_numaccs_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_NUMACCS_NAME);
		goto bitinfo_numaccs_dev_err;
	}
	cdev_init(&bitinfo_numaccs_cdev, &bitinfo_numaccs_fops);
	if (cdev_add(&bitinfo_numaccs_cdev, MKDEV(bitinfo_major, BITINFO_NUMACCS_MINOR), 1)<0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_NUMACCS_NAME);
		goto bitinfo_numaccs_cdev_err;
	}
	bitinfo_numaccs_opens_cnt = 0;

	//Create device for the xtasks config information
	bitinfo_xtasks_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_XTASKS_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_XTASKS_NAME);
	if (IS_ERR(bitinfo_xtasks_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_XTASKS_NAME);
		goto bitinfo_xtasks_dev_err;
	}
	cdev_init(&bitinfo_xtasks_cdev, &bitinfo_xtasks_fops);
	if (cdev_add(&bitinfo_xtasks_cdev, MKDEV(bitinfo_major, BITINFO_XTASKS_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_XTASKS_NAME);
		goto bitinfo_xtasks_cdev_err;
	}
	bitinfo_xtasks_opens_cnt = 0;

	//Create device for the ait call information
	bitinfo_call_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_CALL_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_CALL_NAME);
	if (IS_ERR(bitinfo_call_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_CALL_NAME);
		goto bitinfo_call_dev_err;
	}
	cdev_init(&bitinfo_call_cdev, &bitinfo_call_fops);
	if (cdev_add(&bitinfo_call_cdev, MKDEV(bitinfo_major, BITINFO_CALL_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_CALL_NAME);
		goto bitinfo_call_cdev_err;
	}
	bitinfo_call_opens_cnt = 0;

	//Create device for the ait version information
	bitinfo_ait_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_AV_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_AV_NAME);
	if (IS_ERR(bitinfo_ait_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_AV_NAME);
		goto bitinfo_ait_dev_err;
	}
	cdev_init(&bitinfo_ait_cdev, &bitinfo_ait_fops);
	if (cdev_add(&bitinfo_ait_cdev, MKDEV(bitinfo_major, BITINFO_AV_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_AV_NAME);
		goto bitinfo_ait_cdev_err;
	}
	bitinfo_ait_opens_cnt = 0;

	//Create device for the features/raw information
	bitinfo_f_raw_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_F_RAW_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_F_RAW_NAME);
	if (IS_ERR(bitinfo_f_raw_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_RAW_NAME);
		goto bitinfo_f_raw_dev_err;
	}
	cdev_init(&bitinfo_f_raw_cdev, &bitinfo_f_raw_fops);
	if (cdev_add(&bitinfo_f_raw_cdev, MKDEV(bitinfo_major, BITINFO_F_RAW_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_RAW_NAME);
		goto bitinfo_f_raw_cdev_err;
	}
	bitinfo_f_raw_opens_cnt = 0;

	//Create device for the features/hwcounter information
	bitinfo_f_ins_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_F_INS_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_F_INS_NAME);
	if (IS_ERR(bitinfo_f_ins_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_INS_NAME);
		goto bitinfo_f_ins_dev_err;
	}
	cdev_init(&bitinfo_f_ins_cdev, &bitinfo_f_ins_fops);
	if (cdev_add(&bitinfo_f_ins_cdev, MKDEV(bitinfo_major, BITINFO_F_INS_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_INS_NAME);
		goto bitinfo_f_ins_cdev_err;
	}
	bitinfo_f_ins_opens_cnt = 0;

	//Create device for the features/hwruntime information
	bitinfo_f_hwr_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_F_HWR_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_F_HWR_NAME);
	if (IS_ERR(bitinfo_f_hwr_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_HWR_NAME);
		goto bitinfo_f_hwr_dev_err;
	}
	cdev_init(&bitinfo_f_hwr_cdev, &bitinfo_f_hwr_fops);
	if (cdev_add(&bitinfo_f_hwr_cdev, MKDEV(bitinfo_major, BITINFO_F_HWR_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_HWR_NAME);
		goto bitinfo_f_hwr_cdev_err;
	}
	bitinfo_f_hwr_opens_cnt = 0;

	//Create device for the features/hwruntime_ext information
	bitinfo_f_ehwr_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_F_EHWR_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_F_EHWR_NAME);
	if (IS_ERR(bitinfo_f_ehwr_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_EHWR_NAME);
		goto bitinfo_f_ehwr_dev_err;
	}
	cdev_init(&bitinfo_f_ehwr_cdev, &bitinfo_f_ehwr_fops);
	if (cdev_add(&bitinfo_f_ehwr_cdev, MKDEV(bitinfo_major, BITINFO_F_EHWR_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_EHWR_NAME);
		goto bitinfo_f_ehwr_cdev_err;
	}
	bitinfo_f_ehwr_opens_cnt = 0;

	//Create device for the features/dma information
	bitinfo_f_dma_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_F_DMA_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_F_DMA_NAME);
	if (IS_ERR(bitinfo_f_dma_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_DMA_NAME);
		goto bitinfo_f_dma_dev_err;
	}
	cdev_init(&bitinfo_f_dma_cdev, &bitinfo_f_dma_fops);
	if (cdev_add(&bitinfo_f_dma_cdev, MKDEV(bitinfo_major, BITINFO_F_DMA_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_DMA_NAME);
		goto bitinfo_f_dma_cdev_err;
	}
	bitinfo_f_dma_opens_cnt = 0;

	//Create device for the features/intercon_lev information
	bitinfo_f_inlev_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_F_INLEV_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_F_INLEV_NAME);
	if (IS_ERR(bitinfo_f_inlev_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_INLEV_NAME);
		goto bitinfo_f_inlev_dev_err;
	}
	cdev_init(&bitinfo_f_inlev_cdev, &bitinfo_f_inlev_fops);
	if (cdev_add(&bitinfo_f_inlev_cdev, MKDEV(bitinfo_major, BITINFO_F_INLEV_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_INLEV_NAME);
		goto bitinfo_f_inlev_cdev_err;
	}
	bitinfo_f_inlev_opens_cnt = 0;

	//Create device for the features/intercon_opt information
	bitinfo_f_inopt_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_F_INOPT_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_F_INOPT_NAME);
	if (IS_ERR(bitinfo_f_inopt_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_INOPT_NAME);
		goto bitinfo_f_inopt_dev_err;
	}
	cdev_init(&bitinfo_f_inopt_cdev, &bitinfo_f_inopt_fops);
	if (cdev_add(&bitinfo_f_inopt_cdev, MKDEV(bitinfo_major, BITINFO_F_INOPT_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_INOPT_NAME);
		goto bitinfo_f_inopt_cdev_err;
	}
	bitinfo_f_inopt_opens_cnt = 0;

	//Create device for the features/hwruntime_som information
	bitinfo_f_som_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_F_SOM_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_F_SOM_NAME);
	if (IS_ERR(bitinfo_f_som_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_SOM_NAME);
		goto bitinfo_f_som_dev_err;
	}
	cdev_init(&bitinfo_f_som_cdev, &bitinfo_f_som_fops);
	if (cdev_add(&bitinfo_f_som_cdev, MKDEV(bitinfo_major, BITINFO_F_SOM_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_SOM_NAME);
		goto bitinfo_f_som_cdev_err;
	}
	bitinfo_f_som_opens_cnt = 0;

	//Create device for the features/hwruntime_pom information
	bitinfo_f_pom_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_F_POM_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_F_POM_NAME);
	if (IS_ERR(bitinfo_f_pom_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_POM_NAME);
		goto bitinfo_f_pom_dev_err;
	}
	cdev_init(&bitinfo_f_pom_cdev, &bitinfo_f_pom_fops);
	if (cdev_add(&bitinfo_f_pom_cdev, MKDEV(bitinfo_major, BITINFO_F_POM_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_F_POM_NAME);
		goto bitinfo_f_pom_cdev_err;
	}
	bitinfo_f_pom_opens_cnt = 0;

	//Create device for the hardware runtime VLNV information
	bitinfo_hwr_vlnv_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_HWR_VLNV_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_HWR_VLNV_NAME);
	if (IS_ERR(bitinfo_hwr_vlnv_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_HWR_VLNV_NAME);
		goto bitinfo_hwr_vlnv_dev_err;
	}
	cdev_init(&bitinfo_hwr_vlnv_cdev, &bitinfo_hwr_vlnv_fops);
	if (cdev_add(&bitinfo_hwr_vlnv_cdev, MKDEV(bitinfo_major, BITINFO_HWR_VLNV_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_HWR_VLNV_NAME);
		goto bitinfo_hwr_vlnv_cdev_err;
	}
	bitinfo_hwr_vlnv_opens_cnt = 0;

	//Create device for the hardware base frequency information
	bitinfo_base_freq_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_BASE_FREQ_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_BASE_FREQ_NAME);
	if (IS_ERR(bitinfo_base_freq_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_BASE_FREQ_NAME);
		goto bitinfo_base_freq_dev_err;
	}
	cdev_init(&bitinfo_base_freq_cdev, &bitinfo_base_freq_fops);
	if (cdev_add(&bitinfo_base_freq_cdev, MKDEV(bitinfo_major, BITINFO_BASE_FREQ_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_BASE_FREQ_NAME);
		goto bitinfo_base_freq_cdev_err;
	}
	bitinfo_base_freq_opens_cnt = 0;

	//Create device for the raw information
	bitinfo_raw_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_RAW_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_RAW_NAME);
	if (IS_ERR(bitinfo_raw_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_RAW_NAME);
		goto bitinfo_raw_dev_err;
	}
	cdev_init(&bitinfo_raw_cdev, &bitinfo_raw_fops);
	if (cdev_add(&bitinfo_raw_cdev, MKDEV(bitinfo_major, BITINFO_RAW_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_RAW_NAME);
		goto bitinfo_raw_cdev_err;
	}
	bitinfo_raw_opens_cnt = 0;

	//Create device for the wrapper version information
	//NOTE: This device is only available in versions >= 3. Creating it anyway, it returns "?" if not supported
	bitinfo_wrapper_dev = device_create(bitinfo_cl, NULL, MKDEV(bitinfo_major, BITINFO_WRAPPER_MINOR), NULL,
			DEV_PREFIX "/" BITINFO_DEV_DIR "/" BITINFO_WRAPPER_NAME);
	if (IS_ERR(bitinfo_wrapper_dev)) {
		printk(KERN_ERR "<%s> Could not create bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_WRAPPER_NAME);
		goto bitinfo_wrapper_dev_err;
	}
	cdev_init(&bitinfo_wrapper_cdev, &bitinfo_wrapper_fops);
	if (cdev_add(&bitinfo_wrapper_cdev, MKDEV(bitinfo_major, BITINFO_WRAPPER_MINOR), 1) < 0) {
		printk(KERN_ERR "<%s> Could not add bitstream info device: '%s'\n",
			MODULE_NAME, BITINFO_WRAPPER_NAME);
		goto bitinfo_wrapper_cdev_err;
	}
	bitinfo_wrapper_opens_cnt = 0;

	return 0;

	cdev_del(&bitinfo_wrapper_cdev);
bitinfo_wrapper_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_WRAPPER_MINOR));
bitinfo_wrapper_dev_err:
	cdev_del(&bitinfo_raw_cdev);
bitinfo_raw_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_RAW_MINOR));
bitinfo_raw_dev_err:
	cdev_del(&bitinfo_base_freq_cdev);
bitinfo_base_freq_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_BASE_FREQ_MINOR));
bitinfo_base_freq_dev_err:
	cdev_del(&bitinfo_hwr_vlnv_cdev);
bitinfo_hwr_vlnv_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_HWR_VLNV_MINOR));
bitinfo_hwr_vlnv_dev_err:
	cdev_del(&bitinfo_f_pom_cdev);
bitinfo_f_pom_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_POM_MINOR));
bitinfo_f_pom_dev_err:
	cdev_del(&bitinfo_f_som_cdev);
bitinfo_f_som_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_SOM_MINOR));
bitinfo_f_som_dev_err:
	cdev_del(&bitinfo_f_inopt_cdev);
bitinfo_f_inopt_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_INOPT_MINOR));
bitinfo_f_inopt_dev_err:
	cdev_del(&bitinfo_f_inlev_cdev);
bitinfo_f_inlev_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_INLEV_MINOR));
bitinfo_f_inlev_dev_err:
	cdev_del(&bitinfo_f_dma_cdev);
bitinfo_f_dma_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_DMA_MINOR));
bitinfo_f_dma_dev_err:
	cdev_del(&bitinfo_f_ehwr_cdev);
bitinfo_f_ehwr_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_EHWR_MINOR));
bitinfo_f_ehwr_dev_err:
	cdev_del(&bitinfo_f_hwr_cdev);
bitinfo_f_hwr_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_HWR_MINOR));
bitinfo_f_hwr_dev_err:
	cdev_del(&bitinfo_f_ins_cdev);
bitinfo_f_ins_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_INS_MINOR));
bitinfo_f_ins_dev_err:
	cdev_del(&bitinfo_f_raw_cdev);
bitinfo_f_raw_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_RAW_MINOR));
bitinfo_f_raw_dev_err:
	cdev_del(&bitinfo_ait_cdev);
bitinfo_ait_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_AV_MINOR));
bitinfo_ait_dev_err:
	cdev_del(&bitinfo_call_cdev);
bitinfo_call_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_CALL_MINOR));
bitinfo_call_dev_err:
	cdev_del(&bitinfo_xtasks_cdev);
bitinfo_xtasks_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_XTASKS_MINOR));
bitinfo_xtasks_dev_err:
	cdev_del(&bitinfo_numaccs_cdev);
bitinfo_numaccs_cdev_err:
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_NUMACCS_MINOR));
bitinfo_numaccs_dev_err:
	cdev_del(&bitinfo_rev_cdev);
bitinfo_rev_cdev_err:
	device_destroy(bitinfo_cl, bitinfo_devt);
bitinfo_rev_dev_err:
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

	if (bitinfo_rev_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_REV_NAME);
	}
	cdev_del(&bitinfo_rev_cdev);
	device_destroy(bitinfo_cl, bitinfo_devt);

	if (bitinfo_numaccs_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_NUMACCS_NAME);
	}
	cdev_del(&bitinfo_numaccs_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_NUMACCS_MINOR));

	if (bitinfo_xtasks_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_XTASKS_NAME);
	}
	cdev_del(&bitinfo_xtasks_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_XTASKS_MINOR));

	if (bitinfo_call_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_CALL_NAME);
	}
	cdev_del(&bitinfo_call_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_CALL_MINOR));

	if (bitinfo_ait_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_AV_NAME);
	}
	cdev_del(&bitinfo_ait_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_AV_MINOR));

	if (bitinfo_f_raw_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_F_RAW_NAME);
	}
	cdev_del(&bitinfo_f_raw_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_RAW_MINOR));

	if (bitinfo_f_ins_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_F_INS_NAME);
	}
	cdev_del(&bitinfo_f_ins_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_INS_MINOR));

	if (bitinfo_f_hwr_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_F_HWR_NAME);
	}
	cdev_del(&bitinfo_f_hwr_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_HWR_MINOR));

	if (bitinfo_f_ehwr_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_F_EHWR_NAME);
	}
	cdev_del(&bitinfo_f_ehwr_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_EHWR_MINOR));

	if (bitinfo_f_dma_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_F_DMA_NAME);
	}
	cdev_del(&bitinfo_f_dma_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_DMA_MINOR));

	if (bitinfo_f_inlev_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_F_INLEV_NAME);
	}
	cdev_del(&bitinfo_f_inlev_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_INLEV_MINOR));

	if (bitinfo_f_inopt_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_F_INOPT_NAME);
	}
	cdev_del(&bitinfo_f_inopt_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_INOPT_MINOR));

	if (bitinfo_f_som_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_F_SOM_NAME);
	}
	cdev_del(&bitinfo_f_som_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_SOM_MINOR));

	if (bitinfo_f_pom_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_F_POM_NAME);
	}
	cdev_del(&bitinfo_f_pom_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_F_POM_MINOR));

	if (bitinfo_hwr_vlnv_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_HWR_VLNV_NAME);
	}
	cdev_del(&bitinfo_hwr_vlnv_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_HWR_VLNV_MINOR));

	if (bitinfo_base_freq_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_BASE_FREQ_NAME);
	}
	cdev_del(&bitinfo_base_freq_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_BASE_FREQ_MINOR));

	if (bitinfo_raw_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_RAW_NAME);
	}
	cdev_del(&bitinfo_raw_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_RAW_MINOR));

	if (bitinfo_wrapper_opens_cnt) {
		printk(KERN_INFO "<%s> exit: Device '%s' opens counter is not zero\n",
			MODULE_NAME, BITINFO_WRAPPER_NAME);
	}
	cdev_del(&bitinfo_wrapper_cdev);
	device_destroy(bitinfo_cl, MKDEV(bitinfo_major, BITINFO_WRAPPER_MINOR));

	class_destroy(bitinfo_cl);
	unregister_chrdev_region(bitinfo_devt, BITINFO_NUM_DEVICES);
	return 0;
}
