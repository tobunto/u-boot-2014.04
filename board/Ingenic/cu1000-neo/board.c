/*
 * Ingenic CU1000-Neo setup code
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: Zoro <ykli@ingenic.cn>
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

#include <common.h>
#include <net.h>
#include <netdev.h>
#include <regulator.h>
#include <sha1.h>
#include <asm/io.h>
#include <asm/arch/jzsoc.h>

#define REG32(addr) *(volatile unsigned int *)(addr)
#define EFUSE_CTRL	    0xb3540000
#define EFUSE_CFG	    0xb3540004
#define EFUSE_STATE	    0xb3540008
#define EFUSE_DATA0	    0xb354000C
#define EFUSE_DATA1	    0xb3540010
#define EFUSE_DATA2	    0xb3540014
#define EFUSE_DATA3	    0xb3540018

extern int jz_net_initialize(bd_t *bis);
extern void jz_mmc_init(int clock_div);

int board_early_init_f(void)
{
	regulator_init();
	regulator_set_voltage(REGULATOR_CORE, CONFIG_SPL_CORE_VOLTAGE);

	cpm_writel(CONFIG_SYS_APLL_MNOD | (1 << 8) | 0x20, CPM_CPAPCR);
	while(!(cpm_readl(CPM_CPAPCR) & (1 << 10)));

	/* disable pull for all pins */
	writel(0xffffffff, GPIO_PXPEN(0));
	writel(0xffffffff, GPIO_PXPEN(1));
	writel(0xffffffff, GPIO_PXPEN(2));
	writel(0xffffffff, GPIO_PXPEN(3));

	/* setup UART2 pins */
	writel(0x30, GPIO_PXINTC(3));
	writel(0x30, GPIO_PXMASKC(3));
	writel(0x30, GPIO_PXPAT1C(3));
	writel(0x30, GPIO_PXPAT0C(3));
	writel(0x30, GPIO_PXPENC(3));

	return 0;
}

int board_early_init_r(void)
{
	return 0;
}

static void read_efuse_segment(unsigned int addr, unsigned int length, unsigned int *buf)
{
	unsigned int val;

	/* clear read done staus */
	REG32(EFUSE_STATE) = 0;
	val = addr << 21 | length << 16 | 1;
	REG32(EFUSE_CTRL) = val;
	/* wait read done status */
	while(!(REG32(EFUSE_STATE) & 1));

	buf[0] = REG32(EFUSE_DATA0);
	buf[1] = REG32(EFUSE_DATA1);
	buf[2] = REG32(EFUSE_DATA2);
	buf[3] = REG32(EFUSE_DATA3);

	/* clear read done staus */
	REG32(EFUSE_STATE) = 0;
}

int misc_init_r(void)
{
	uint32_t chipid[4] = { 0 };
	uint8_t macaddr[20] = { 0 };

	writel(readl(0xb0000020) & 0xfffffffd, 0xb0000020);
	read_efuse_segment(0x00, 15, chipid);
	writel(readl(0xb0000020) | 0x00000002, 0xb0000020);

	sha1_csum((uint8_t *)chipid, sizeof(chipid), macaddr);

	macaddr[0] &= 0xfe;

	/* set MAC address */
	eth_setenv_enetaddr("ethaddr", macaddr);

	return 0;
}

#ifdef CONFIG_MMC
int board_mmc_init(bd_t *bd)
{
	uint32_t msc_cdr;

	/* setup MSC0 clock */
	msc_cdr = CONFIG_SYS_MPLL_FREQ / 50000000 / 2 - 1;
	writel(readl(0xb0000020) & 0xffffffef, 0xb0000020);
	writel(msc_cdr | CPM_MSCCDR_MPCS_MPLL | CPM_MSCCDR_CE, CPM_BASE + CPM_MSC0CDR);
	while (readl(CPM_BASE + CPM_MSC0CDR) & CPM_MSCCDR_MSC_BUSY);

	/* setup MSC0 pins */
	writel(0x3ff0000, GPIO_PXINTC(0));
	writel(0x3ff0000, GPIO_PXMASKC(0));
	writel(0x3ff0000, GPIO_PXPAT1C(0));
	writel(0x3ff0000, GPIO_PXPAT0S(0));

	jz_mmc_init((msc_cdr + 1) * 2);
	return 0;
}
#endif

#ifdef CONFIG_SYS_NAND_SELF_INIT
void board_nand_init(void)
{
	return 0;
}
#endif

int board_eth_init(bd_t *bis)
{
	int rv;
#ifndef  CONFIG_USB_ETHER
	/* reset grus DM9000 */
#ifdef CONFIG_NET_X1000
	uint32_t mac_cdr;

	/* setup MAC clock */
	mac_cdr = CONFIG_SYS_MPLL_FREQ / 50000000 - 1;
	writel(readl(0xb0000020) & 0xfdffffff, 0xb0000020);
	writel(mac_cdr | CPM_MACCDR_MACPCS_MPLL | CPM_MACCDR_CE, CPM_BASE + CPM_MACCDR);
	while (readl(CPM_BASE + CPM_MACCDR) & CPM_MACCDR_MAC_BUSY);

	/* setup MAC pins */
	/*writel(0x000ffc0, GPIO_PXINTC(1));
	writel(0x000ffc0, GPIO_PXMASKC(1));
	writel(0x000ffc0, GPIO_PXPAT1C(1));
	writel(0x000ffc0, GPIO_PXPAT0S(1));*/
	
	writel(0x0, GPIO_PXINTS(1));
	writel(0x000ffc0, GPIO_PXMASKS(1));
	writel(0x0, GPIO_PXPAT1S(1));
	writel(0x0, GPIO_PXPAT0S(1));
	
	writel(0x000ffc0, GPIO_PXINTC(1));
	writel(0x0, GPIO_PXMASKC(1));
	writel(0x000ffc0, GPIO_PXPAT1C(1));
	writel(0x000ffc0, GPIO_PXPAT0C(1));

	gpio_direction_output(CONFIG_GPIO_LAN8720A_RESET, CONFIG_GPIO_LAN8720A_RESET_ENLEVEL);
	udelay(10000);
	gpio_direction_output(CONFIG_GPIO_LAN8720A_RESET, !CONFIG_GPIO_LAN8720A_RESET_ENLEVEL);
	udelay(10000);
	
	writel(0x0, GPIO_PXINTS(1));
	writel(0x0, GPIO_PXMASKS(1));
	writel(0x0, GPIO_PXPAT1S(1));
	writel(0x000ffc0, GPIO_PXPAT0S(1));
	
	writel(0x000ffc0, GPIO_PXINTC(1));
	writel(0x000ffc0, GPIO_PXMASKC(1));
	writel(0x000ffc0, GPIO_PXPAT1C(1));
	writel(0x0, GPIO_PXPAT0C(1));

	rv = jz_net_initialize(bis);
#endif
#else
	rv = usb_eth_initialize(bis);
#endif
	return rv;
}

#ifdef CONFIG_SPL_NOR_SUPPORT
int spl_start_uboot(void)
{
	return 1;
}
#endif

/* U-Boot common routines */
int checkboard(void)
{
	puts("Board: CU1000-Neo (Ingenic XBurst X1000E SoC)\n");
	(*(volatile unsigned int *)0xB0010718) = 1 << 10;
	(*(volatile unsigned int *)0xB0010724) = 1 << 10;
	(*(volatile unsigned int *)0xB0010738) = 1 << 10;
	(*(volatile unsigned int *)0xB0010744) = 1 << 10;
	(*(volatile unsigned int *)0xB00107F0) = 0x0;
	puts("Force Pull-up PA10 for eMMC RSTN Signal\n");
	return 0;
}

#ifdef CONFIG_SPL_BUILD
void spl_board_init(void)
{
}
#endif /* CONFIG_SPL_BUILD */
