/*
 * Ingenic x2000 setup code
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: cxtan <chenxi.tan@ingenic.cn>
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
#include <asm/io.h>
#include <asm/arch/x2000.h>
#include <asm/arch/mmc.h>

extern int jz_net_initialize(bd_t *bis);
extern void jz_mmc_init(void);

int board_early_init_f(void)
{
	/* disable pull for all pins */
	writel(0x00000000, GPIO_PXPU(0));
	writel(0x00000000, GPIO_PXPD(0));
	writel(0x00000000, GPIO_PXPU(1));
	writel(0x00000000, GPIO_PXPD(1));
	writel(0x00000000, GPIO_PXPU(2));
	writel(0x00000000, GPIO_PXPD(2));
	writel(0x0000000f, GPIO_PXPU(3));
	writel(0x00000000, GPIO_PXPD(3));
	writel(0x00000000, GPIO_PXPU(4));
	writel(0x00000000, GPIO_PXPD(4));

	/* setup UART3 pins */
	writel(0x06000000, GPIO_PXINTC(2));
	writel(0x06000000, GPIO_PXMASKC(2));
	writel(0x06000000, GPIO_PXPAT1C(2));
	writel(0x06000000, GPIO_PXPAT0C(2));
	writel(0x06000000, GPIO_PXPUS(2));

	return 0;
}

int board_early_init_r(void)
{

#ifdef CONFIG_REGULATOR
	regulator_init();
#endif
	return 0;

}

#ifdef CONFIG_REGULATOR
int regulator_init(void)
{
	int ret;
#ifdef CONFIG_PMU_RICOH6x
	ret = ricoh61x_regulator_init();
#endif
	return ret;
}
#endif /* CONFIG_REGULATOR */


int misc_init_r(void)
{
#ifdef CONFIG_BOOT_ANDROID
	boot_mode_select();
#endif
	return 0;
}



#ifdef CONFIG_MMC
int board_mmc_init(bd_t *bd)
{
	uint32_t msc_cdr;

//	writel(readl(CPM_BASE + 0x24) | (1 << 2), CPM_BASE + 0x24);

	/* setup MSC2 clock */
	msc_cdr = CONFIG_SYS_MPLL_FREQ / 50000000 / 4 - 1;
	writel(readl(0xb0000028) & 0xfdffffff, 0xb0000028);
	writel(msc_cdr | CPM_MSCCDR_MPCS_MPLL | CPM_MSCCDR_CE, CPM_BASE + CPM_MSC2CDR);
	while (readl(CPM_BASE + CPM_MSC2CDR) & CPM_MSCCDR_MSC_BUSY);

	/* setup MSC2 pins */
	writel(0x0080003f, GPIO_PXINTC(4));
	writel(0x0080003f, GPIO_PXMASKC(4));
	writel(0x0080003f, GPIO_PXPAT1C(4));
	writel(0x0080003f, GPIO_PXPAT0C(4));

	jz_mmc_init();

	return 0;
}
#endif

#ifdef CONFIG_SYS_NAND_SELF_INIT
void board_nand_init(void)
{
	return;
}
#endif

int board_eth_init(bd_t *bis)
{
	int rv = 0;
#ifndef  CONFIG_USB_ETHER
	uint32_t mac_cdr;
	/*writel(0xbe401, CPM_BASE + 0xe8);*/
	/* setup MAC1 clock */
	mac_cdr = CONFIG_SYS_MPLL_FREQ / 50000000 - 1;
	writel(readl(0xb0000028) & 0xfeffffff, 0xb0000028);
	writel(mac_cdr | CPM_MACCDR_MACPCS_MPLL | CPM_MACCDR_CE, CPM_BASE + CPM_MACPHYCDR);
	while (readl(CPM_BASE + CPM_MACPHYCDR) & CPM_MACCDR_MAC_BUSY);

	mac_cdr = CONFIG_SYS_MPLL_FREQ / 125000000 - 1;
	writel(mac_cdr | CPM_MACCDR_MACPCS_MPLL | CPM_MACCDR_CE, CPM_BASE + CPM_MACTXCDR1);
	while (readl(CPM_BASE + CPM_MACTXCDR1) & CPM_MACCDR_MAC_BUSY);

	/* setup MAC1 pins */
	/*writel(0x00feff00, GPIO_PXINTC(1));
	writel(0x00feff00, GPIO_PXMASKC(1));
	writel(0x00feff00, GPIO_PXPAT1C(1));
	writel(0x00feff00, GPIO_PXPAT0C(1));	*/
	
	writel(0x0, GPIO_PXINTS(1));
	writel(0x00feff00, GPIO_PXMASKS(1));
	writel(0x00feff00, GPIO_PXPAT1S(1));
	writel(0x0, GPIO_PXPAT0S(1));
	
	writel(0x00feff00, GPIO_PXINTC(1));
	writel(0x0, GPIO_PXMASKC(1));
	writel(0x0, GPIO_PXPAT1C(1));
	writel(0x00feff00, GPIO_PXPAT0C(1));
	
	writel(0x0, GPIO_PXPUC(1));
	writel(0x00feff00, GPIO_PXPUS(1));

	gpio_direction_output(CONFIG_GMAC_PHY_RESET, CONFIG_GMAC_PHY_RESET_ENLEVEL);
	udelay(10000);
	gpio_direction_output(CONFIG_GMAC_PHY_RESET, !CONFIG_GMAC_PHY_RESET_ENLEVEL);
	udelay(10000);
	
	writel(0x0, GPIO_PXINTS(1));
	writel(0x0, GPIO_PXMASKS(1));
	writel(0x00feff00, GPIO_PXPAT1S(1));
	writel(0x00feff00, GPIO_PXPAT0S(1));
	
	writel(0x00feff00, GPIO_PXINTC(1));
	writel(0x00feff00, GPIO_PXMASKC(1));
	writel(0x0, GPIO_PXPAT1C(1));
	writel(0x0, GPIO_PXPAT0C(1));
	
	writel(0x0, GPIO_PXPUC(1));
	writel(0x00feff00, GPIO_PXPUS(1));

	/* reset grus DM9000 */
	rv = jz_net_initialize(bis);
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
	puts("Board: Halley5 (Ingenic XBurst2 X2000 SoC)\n");
	return 0;
}

#ifdef CONFIG_SPL_BUILD

void spl_board_init(void)
{
}

#endif /* CONFIG_SPL_BUILD */
