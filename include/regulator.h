/*
 * SoC Regulator driver support
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: yqfu <yqfu@ingenic.cn>
 * Based on: u-boot-ak47/include/regulator.h
 *           Written by Kage Shen <kkshen@ingenic.cn>
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

#ifndef __REGULATOR_H__
#define __REGULATOR_H__

enum regulator_outnum { REGULATOR_CORE = 1, REGULATOR_MEM, REGULATOR_IO };

int regulator_init(void);
int regulator_set_voltage(enum regulator_outnum outnum, int vol_mv);

#endif
