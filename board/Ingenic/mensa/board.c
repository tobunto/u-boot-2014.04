/*
 * Ingenic mensa setup code
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
#include <nand.h>
#include <net.h>
#include <netdev.h>
#include <asm/arch/jz4775.h>
#include <asm/io.h>
#include <asm/jz_mmc.h>

#define FUNC_ENTER()
#define FUNC_EXIT()

#define dump_ddrc_register()
#define dump_ddrp_register()

extern int act8600_regulator_init(void);
extern int jz_net_initialize(bd_t *bis);
#ifdef CONFIG_BOOT_ANDROID
extern void boot_mode_select(void);
#endif

#if defined(CONFIG_CMD_BATTERYDET) && defined(CONFIG_BATTERY_INIT_GPIO)
static void battery_init_gpio(void)
{
}
#endif

int board_early_init_f(void)
{
	/* disable pull for all pins */
	writel(0xffffffff, GPIO_PXPEN(0));
	writel(0xffffffff, GPIO_PXPEN(1));
	writel(0xffffffff, GPIO_PXPEN(2));
	writel(0xffffffff, GPIO_PXPEN(3));
	writel(0xffffffff, GPIO_PXPEN(4));
	writel(0xffffffff, GPIO_PXPEN(5));
	writel(0xffffffff, GPIO_PXPEN(6));

	/* setup UART3 pins */
	writel(0x80000000, GPIO_PXINTC(0));
	writel(0x80000000, GPIO_PXMASKC(0));
	writel(0x80000000, GPIO_PXPAT1C(0));
	writel(0x80000000, GPIO_PXPAT0S(0));

	/* Power on TF-card */
	gpio_direction_output(GPIO_PB(3), 1);
//	act8600_regulator_init();

	return 0;
}

#ifdef CONFIG_USB_GADGET
int jz_udc_probe(void);
void board_usb_init(void)
{
	printf("USB_udc_probe\n");
	jz_udc_probe();
}
#endif /* CONFIG_USB_GADGET */

int misc_init_r(void)
{
#if 0 /* TO DO */
	uint8_t mac[6] = { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc };

	/* set MAC address */
	eth_setenv_enetaddr("ethaddr", mac);
#endif
#ifdef CONFIG_BOOT_ANDROID
	boot_mode_select();
#endif

#if defined(CONFIG_CMD_BATTERYDET) && defined(CONFIG_BATTERY_INIT_GPIO)
	battery_init_gpio();
#endif
	return 0;
}

#ifdef CONFIG_MTD_NAND_JZ
#ifdef CONFIG_SYS_NAND_SELF_INIT
extern void mtd_nand_probe(void);
void board_nand_init(void)
{
	mtd_nand_probe();
}
#else
extern int mtd_nand_init(struct nand_chip *nand);
int board_nand_init(struct nand_chip *nand)
{
	mtd_nand_init(nand);
	return 0;
}
#endif
#else
int board_nand_init(struct nand_chip *nand)
{
	return 0;
}
#endif


#ifdef CONFIG_MMC
int board_mmc_init(bd_t *bd)
{
	uint32_t msc_cdr;

	/* setup MSC0 clock */
	msc_cdr = CONFIG_SYS_APLL_FREQ / 50000000 / 2 - 1;
	writel(msc_cdr | CPM_MSCCDR_MPCS_SCLKA | CPM_MSCCDR_CE, CPM_BASE + CPM_MSC0CDR);
	while (readl(CPM_BASE + CPM_MSC0CDR) & CPM_MSCCDR_MSC_BUSY);

	/* setup MSC0 pins */
	writel(0x00fc00f0, GPIO_PXINTC(0));
	writel(0x00fc00f0, GPIO_PXMASKC(0));
	writel(0x00fc00f0, GPIO_PXPAT1C(0));
	writel(0x00fc00f0, GPIO_PXPAT0S(0));

	jz_mmc_init((msc_cdr + 1) * 2);
	return 0;
}
#endif

int board_eth_init(bd_t *bis)
{
	int rv;
#ifndef  CONFIG_USB_ETHER
	/*  initialize jz4775 gpio */
//	gpio_set_func(GPIO_PORT_B, GPIO_FUNC_1, 0x0003fc10);
//	gpio_set_func(GPIO_PORT_D, GPIO_FUNC_1, 0x3c000000);
//	gpio_set_func(GPIO_PORT_F, GPIO_FUNC_0, 0x0000fff0);
//	udelay(100000);

	/*  reset DM9161 */
//	gpio_direction_output(CONFIG_GPIO_DM9161_RESET, CONFIG_GPIO_DM9161_RESET_ENLEVEL);
//	mdelay(10);
//	gpio_set_value(CONFIG_GPIO_DM9161_RESET, !CONFIG_GPIO_DM9161_RESET_ENLEVEL);
//	mdelay(10);

	rv = jz_net_initialize(bis);
#else
	rv = usb_eth_initialize(bis);
#endif
	return rv;
}

/* U-Boot common routines */
int checkboard(void)
{
	puts("Board: mensa (Ingenic XBurst JZ4775 SoC)\n");

	return 0;
}

#ifdef CONFIG_SPL_BUILD

void spl_board_init(void)
{
}

#endif /* CONFIG_SPL_BUILD */
