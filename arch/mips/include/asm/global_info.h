/*
 * Xburst u-boot global infomation structure.
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __GLOBAL_INFO_H__
#define __GLOBAL_INFO_H__

#include <ddr/ddr_params.h>

struct global_info {
	uint32_t extal;
	uint32_t cpufreq;
	uint32_t ddrfreq;
	uint32_t ddr_div;
	uint32_t uart_idx;
	uint32_t baud_rate;

	struct {
		uint32_t ddr_cfg;
		uint32_t ddr_mmap0;
		uint32_t ddr_mmap1;
		uint32_t ddr_timing4;
		uint32_t ddr_autosr;
		uint32_t ddr_remap_array[5];
	} ddr_change_param;

};

#endif /* __GLOBAL_INFO_H__ */
