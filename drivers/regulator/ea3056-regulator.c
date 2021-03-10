/*
 * drivers/regulator/ricoh619-regulator.c
 *
 * Regulator driver for RICOH R5T619 power management chip.
 *
 * Copyright (C) 2012-2014 RICOH COMPANY,LTD
 *
 * Based on code
 *	Copyright (C) 2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <config.h>
#include <common.h>
#include <linux/err.h>
#include <linux/list.h>
#include <regulator.h>
#include <asm/arch/jzsoc.h>

#define EA3056_I2C_ADDR    0x35

#define EA3056_DC1  0x01
#define EA3056_DC2  0x02
#define EA3056_DC3  0x03
#define	EA3056_LDO1 0x04

static int ea3056_write_byte(unsigned int scl, unsigned int sda, unsigned char data)
{
	int j;
	int nack;

	for(j = 0; j < 8; j++) {
		gpio_direction_output(scl, 0);
		udelay(5);

		if(data & 0x80)
			gpio_direction_input(sda);
		else
			gpio_direction_output(sda, data & 0x80);

		udelay(5);
		gpio_direction_output(scl, 1);
		udelay(5);
		udelay(5);

		data <<= 1;
	}

	gpio_direction_output(scl, 0);
	udelay(5);
	gpio_direction_input(sda);
	udelay(5);
	gpio_direction_output(scl, 1);
	udelay(5);
	udelay(5);
	nack = gpio_get_value(sda);
	gpio_direction_output(scl, 0);
	udelay(5);

	return(nack);	/* not a nack is an ack */
}

int  ea3056_i2c_write(unsigned int scl, unsigned int sda, unsigned char chip,
					unsigned int addr, int alen, unsigned char *buffer, int len)
{
	int shift, failures = 0;

	udelay(5);
	gpio_direction_input(sda);
	udelay(5);
	gpio_direction_output(scl,1);
	udelay(5);
	gpio_direction_output(sda,0);
	udelay(5);

	if(ea3056_write_byte(scl, sda, chip << 1)) {	/* write cycle */
		gpio_direction_output(scl,0);
		udelay(5);
		gpio_direction_output(sda,0);
		udelay(5);
		gpio_direction_output(scl,1);
		udelay(5);
		gpio_direction_input(sda);
		udelay(5);
		return -EINVAL;
	}
	shift = (alen-1) * 8;
	while(alen-- > 0) {
		if(ea3056_write_byte(scl, sda, addr >> shift))
			return -EINVAL;
		shift -= 8;
	}

	while(len-- > 0) {
		if(ea3056_write_byte(scl, sda, *buffer++))
			failures++;
	}
	gpio_direction_output(scl,0);
	udelay(5);
	gpio_direction_output(sda,0);
	udelay(5);
	gpio_direction_output(scl,1);
	udelay(5);
	gpio_direction_input(sda);
	udelay(5);

	return(failures);
}

void ea3056_i2c_init(unsigned int scl, unsigned int sda)
{
	gpio_direction_output(sda,1);
	gpio_direction_output(scl,1);

	for(int j = 0; j < 9; j++) {
		gpio_direction_output(scl,0);
		udelay(5);
		udelay(5);
		gpio_direction_output(scl,1);
		udelay(5);
		udelay(5);
	}

	gpio_direction_output(scl,0);
	udelay(5);
	gpio_direction_output(sda,0);
	udelay(5);
	gpio_direction_output(scl,1);
	udelay(5);
	gpio_direction_input(sda);
	udelay(5);
}

static int ea3056_write_reg(u8 reg, u8 *val)
{
	unsigned int  ret;

	ret = ea3056_i2c_write(CONFIG_EA3056_I2C_SCL, CONFIG_EA3056_I2C_SDA,
							EA3056_I2C_ADDR, reg, 1, val, 1);

	if(ret) {
		debug("EA3056 write register error\n");
		return -EIO;
	}

	return 0;
}

int ea3056_set_voltage(enum regulator_outnum outnum, int vol_mv)
{
	char reg;
	u8 vid;
	u8 regvalue;

	switch(outnum) {
		case REGULATOR_CORE:
			reg = EA3056_DC3;
			if ((vol_mv < 1000) || (vol_mv >1400)) {
				debug("voltage for core is out of range\n");
				return -EINVAL;
			}
			break;
		case REGULATOR_MEM:
			reg = EA3056_DC2;
			if ((vol_mv < 1700) || (vol_mv >1900)) {
				debug("voltage for mem is out of range\n");
				return -EINVAL;
			}
			break;
		case REGULATOR_IO:
			reg = EA3056_DC1;
			if ((vol_mv < 3000) || (vol_mv >3400)) {
				debug("voltage for mem is out of range\n");
				return -EINVAL;
			}
			break;
		default:return -EINVAL;
	}

	vid = (vol_mv * CONFIG_SPL_CORE_VOLTAGE_RATIO - 580 + 2) / 5;

	if ((vid < 0) || (vid > 45)) {
		debug("unsupported voltage\n");
		return -EINVAL;
	} else {
		regvalue = vid + 0x40;
	}

	return ea3056_write_reg(reg, &regvalue);
}

int regulator_init(void)
{
	ea3056_i2c_init(CONFIG_EA3056_I2C_SCL, CONFIG_EA3056_I2C_SDA);

	return 0;
}

int regulator_set_voltage(enum regulator_outnum outnum, int vol_mv)
{
	return ea3056_set_voltage(outnum, vol_mv);
}
