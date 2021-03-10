/*
 * Ingenic CU1830-Neo setup code
 *
 * Copyright (c) 2017 Ingenic Semiconductor Co.,Ltd
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

extern int jz_net_initialize(bd_t *bis);
extern void jz_mmc_init(int clock_div);

int board_early_init_f(void)
{
	regulator_init();
	regulator_set_voltage(REGULATOR_CORE, CONFIG_SPL_CORE_VOLTAGE);

	/* set CPU frequency to the target value */
	writel(CONFIG_SYS_APLL_MNOD | (1 << 0), CPM_CPAPCR);
	while(!(readl(CPM_CPAPCR) & (1 << 3)));

	/* disable pull for all pins */
	writel(0x00000000, GPIO_PXPEL(0));
	writel(0x00000000, GPIO_PXPEH(0));
	writel(0x00000000, GPIO_PXPEL(1));
	writel(0x00000000, GPIO_PXPEH(1));
	writel(0x00000000, GPIO_PXPEL(2));
	writel(0x00000000, GPIO_PXPEH(2));
	writel(0x00000000, GPIO_PXPEL(3));
	writel(0x00000000, GPIO_PXPEH(3));

	/* setup UART1 pins */
	writel(0x01800000, GPIO_PXINTC(1));
	writel(0x01800000, GPIO_PXMASKC(1));
	writel(0x01800000, GPIO_PXPAT1C(1));
	writel(0x01800000, GPIO_PXPAT0C(1));
	writel(0x00014000, GPIO_PXPEHS(1));

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

	/* clear read done staus */
	REG32(EFUSE_STATE) = 0;
	val = (addr + 1) << 21 | length << 16 | 1;
	REG32(EFUSE_CTRL) = val;
	/* wait read done status */
	while(!(REG32(EFUSE_STATE) & 1));

	buf[1] = REG32(EFUSE_DATA0);

	/* clear read done staus */
	REG32(EFUSE_STATE) = 0;
	val = (addr + 2) << 21 | length << 16 | 1;
	REG32(EFUSE_CTRL) = val;
	/* wait read done status */
	while(!(REG32(EFUSE_STATE) & 1));

	buf[2] = REG32(EFUSE_DATA0);

	/* clear read done staus */
	REG32(EFUSE_STATE) = 0;
}

int misc_init_r(void)
{
	uint32_t chipid[3] = { 0 };
	uint8_t macaddr[20] = { 0 };

	writel(readl(0xb0000020) & 0xfffffffd, 0xb0000020);
	read_efuse_segment(0x00, 11, chipid);
	writel(readl(0xb0000020) | 0x00000002, 0xb0000020);

	sha1_csum((uint8_t *)chipid, sizeof(chipid), macaddr);

	macaddr[0] &= 0xfe;

	/* set MAC address */
	eth_setenv_enetaddr("ethaddr", macaddr);

	return 0;
}

#ifdef CONFIG_SYS_NAND_SELF_INIT
void board_nand_init(void)
{
	return 0;
}
#endif

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
	writel(0x3f, GPIO_PXINTC(1));
	writel(0x3f, GPIO_PXMASKC(1));
	writel(0x3f, GPIO_PXPAT1C(1));
	writel(0x3f, GPIO_PXPAT0C(1));

	jz_mmc_init((msc_cdr + 1) * 2);

	return 0;
}
#endif

int board_eth_init(bd_t *bis)
{
	int ret = 0;
	uint32_t mac_cdr;

	/* setup MAC clock */
	mac_cdr = CONFIG_SYS_MPLL_FREQ / 50000000 - 1;
	writel(readl(0xb0000028) & 0xffffffef, 0xb0000028);
	writel(mac_cdr | CPM_MACCDR_MACPCS_MPLL | CPM_MACCDR_CE, CPM_BASE + CPM_MACCDR);
	while (readl(CPM_BASE + CPM_MACCDR) & CPM_MACCDR_MAC_BUSY);

	/* setup MAC pins */
	writel(0x001efc0, GPIO_PXINTC(1));
	writel(0x001efc0, GPIO_PXMASKC(1));
	writel(0x001efc0, GPIO_PXPAT1C(1));
	writel(0x001efc0, GPIO_PXPAT0C(1));

	gpio_direction_output(CONFIG_GPIO_IP101G_RESET, CONFIG_GPIO_IP101G_RESET_ENLEVEL);
	udelay(10000);
	gpio_direction_output(CONFIG_GPIO_IP101G_RESET, !CONFIG_GPIO_IP101G_RESET_ENLEVEL);
	udelay(10000);

	ret = jz_net_initialize(bis);
	return ret;
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
	puts("Board: CU1830-Neo (Ingenic XBurst X1830 SoC)\n");

	return 0;
}

#ifdef CONFIG_SPL_BUILD
void spl_board_init(void)
{
}
#endif /* CONFIG_SPL_BUILD */
