
/*
 * DDR driver for inno DDR PHY.
 * Used by x1xxx
 *
 * Copyright (C) 2017 Ingenic Semiconductor Co.,Ltd
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

/* #define DEBUG */
#include <config.h>
#include <common.h>
#include <ddr/ddr_common.h>
#include <asm/arch/ddr_reg_values.h>
#include <asm/arch/jzsoc.h>
#include "ddr_innophy.h"

#ifdef CONFIG_DWC_DEBUG
#define dwc_debug(fmt, args...)			\
	do {					\
		printf(fmt, ##args);		\
	} while (0)
#else
#define dwc_debug(fmt, args...)			\
	do {					\
	} while (0)
#endif

extern unsigned int sdram_size(int cs, struct ddr_params *p);

struct ddr_params *ddr_params_p = NULL;

#define BYPASS_ENABLE       1
#define BYPASS_DISABLE      0
#define IS_BYPASS_MODE(x)     (((x) & 1) == BYPASS_ENABLE)
#define DDR_TYPE_MODE(x)     (((x) >> 1) & 0xf)

static void reset_controller(void)
{
	ddr_writel(0xf << 20, DDRC_CTRL);
	udelay(5000);
	ddr_writel(0x8 << 20, DDRC_CTRL);
	udelay(5000);
}

static void ddrc_post_init(void)
{
	ddr_writel(DDRC_REFCNT_VALUE, DDRC_REFCNT);
	debug("DDRC_STATUS: %x\n",ddr_readl(DDRC_STATUS));
	ddr_writel(DDRC_CTRL_VALUE, DDRC_CTRL);
}

static void ddrc_prev_init(void)
{
	dwc_debug("DDR Controller init\n");

	ddr_writel(DDRC_TIMING1_VALUE, DDRC_TIMING(1));
	ddr_writel(DDRC_TIMING2_VALUE, DDRC_TIMING(2));
	ddr_writel(DDRC_TIMING3_VALUE, DDRC_TIMING(3));
	ddr_writel(DDRC_TIMING4_VALUE, DDRC_TIMING(4));
	ddr_writel(DDRC_TIMING5_VALUE, DDRC_TIMING(5));
	ddr_writel(DDRC_TIMING6_VALUE, DDRC_TIMING(6));

	/* DDRC memory map configure*/
	ddr_writel(DDRC_MMAP0_VALUE, DDRC_MMAP0);
	ddr_writel(DDRC_MMAP1_VALUE, DDRC_MMAP1);
	ddr_writel(DDRC_CTRL_VALUE & 0xffff8fff, DDRC_CTRL);
}

void ddr_inno_phy_init(void)
{
	u32 reg = 0;
	/*
	 * ddr phy pll initialization
	 */
	phy_writel(0x14, INNO_PLL_FBDIV);
	phy_writel(0x5, INNO_PLL_PDIV);
	phy_writel(0x1a, INNO_PLL_CTRL);
	phy_writel(0x18, INNO_PLL_CTRL);
	debug("ddrp pll lock 0x%x\n", phy_readl(INNO_PLL_LOCK));
	while(!(readl(DDR_APB_PHY_INIT) & (1<<2))); //polling pll lock

	/*
	 * ddr phy register cfg
	 */
	phy_writel(0x3, INNO_DQ_WIDTH);

	phy_writel(0x11,INNO_MEM_CFG);  // MEMSEL  =  DDR2  ,    BURSEL = burst8
	phy_writel(0x0d,INNO_CHANNEL_EN);
	phy_writel(((DDR_MR0_VALUE&0xf0)>>4)-1, INNO_CWL);
	reg = ((DDR_MR0_VALUE&0xf0)>>4);
	phy_writel(reg, INNO_CL);

	phy_writel(0x0, INNO_AL);

	debug("CWL = 0x%x\n", phy_readl(INNO_CWL));
	debug("CL = 0x%x\n", phy_readl(INNO_CL));
	debug("AL = 0x%x\n", phy_readl(INNO_AL));
}

void ddrc_dfi_init(void)
{
	u32 reg = 0;

	writel(1, DDR_APB_PHY_INIT); //start high
	writel(0, DDR_APB_PHY_INIT); //start low
	while(!(readl(DDR_APB_PHY_INIT) & (1<<1))); //polling dfi init comp

	ddr_writel(0, DDRC_CTRL);
	ddr_writel(DDRC_CFG_VALUE, DDRC_CFG);
	ddr_writel(0x2, DDRC_CTRL);

	/*DDR2*/
	ddr_writel(0x211,DDRC_LMR);
	debug("DDRC_LMR: %x\n",ddr_readl(DDRC_LMR));
	ddr_writel(0,DDRC_LMR);

	ddr_writel(0x311,DDRC_LMR);
	debug("DDRC_LMR: %x\n", ddr_readl(DDRC_LMR));
	ddr_writel(0,DDRC_LMR);

	ddr_writel(0x111,DDRC_LMR);
	debug("DDRC_LMR: %x\n", ddr_readl(DDRC_LMR));
	ddr_writel(0,DDRC_LMR);

	reg = ((DDR_MR0_VALUE)<<12)|0x011;
	ddr_writel(reg, DDRC_LMR);
	debug("DDRC_LMR, MR0: %x\n", reg);
	ddr_writel(0,DDRC_LMR);
}

void ddrp_wl_training(void)
{
	phy_writel(0x51, INNO_MEM_CFG);

	writel(0x24,0xb3011028);
}

/*
 * Name     : phy_calibration()
 * Function : control the RX DQS window delay to the DQS
 * */
void phy_calibration(void)
{
	int m = phy_readl(INNO_TRAINING_CTRL);
	debug("INNO_TRAINING_CTRL 1: %x\n", phy_readl(INNO_TRAINING_CTRL));
	m = 0xa1;
	phy_writel(m,INNO_TRAINING_CTRL);
	debug("INNO_TRAINING_CTRL 2: %x\n", phy_readl(INNO_TRAINING_CTRL));
	while (0x3 != phy_readl(INNO_CALIB_DONE));
	debug("calib done: %x\n", phy_readl(INNO_CALIB_DONE));
	phy_writel(0xa0,INNO_TRAINING_CTRL);
	debug("INNO_TRAINING_CTRL 3: %x\n", phy_readl(INNO_TRAINING_CTRL));
}

/* DDR sdram init */
void sdram_init(void)
{
	uint32_t ddr_cdr;

	/* setup DDR clock */
	ddr_cdr = CONFIG_SYS_MPLL_FREQ / CONFIG_SYS_MEM_FREQ - 1;
	writel(ddr_cdr | CPM_DDRCDR_DCS_MPLL | CPM_DDRCDR_CE, CPM_BASE + CPM_DDRCDR);
	while (readl(CPM_BASE + CPM_DDRCDR) & CPM_DDRCDR_DDR_BUSY);

	/*
	 * WARNING: 2015-01-08
	 * DDR CLK GATE(CPM_DRCG 0xB00000D0), BIT6 must set to 1 (or 0x40).
	 * If clear BIT6, chip memory will not stable, gpu hang occur.
	 */
	writel(0x73 | (1 << 6), CPM_BASE + 0xd0);
	udelay(1000);
	writel(0x71 | (1 << 6), CPM_BASE + 0xd0);
	udelay(1000);

	reset_controller();

	ddr_inno_phy_init();
	ddrc_dfi_init();

	ddrp_wl_training();
        /* DDR Controller init*/
	ddrc_prev_init();
	phy_calibration();
	dwc_debug("DDR PHY init OK\n");
	ddrc_post_init();

	ddr_writel(ddr_readl(DDRC_STATUS) & ~DDRC_DSTATUS_MISS, DDRC_STATUS);

	ddr_writel(0 , DDRC_DLP);

	dwc_debug("sdram init finished\n");
}

phys_size_t initdram(int board_type)
{
#ifndef EMC_LOW_SDRAM_SPACE_SIZE
#define EMC_LOW_SDRAM_SPACE_SIZE 0x10000000 /* 256M */
#endif /* EMC_LOW_SDRAM_SPACE_SIZE */
	unsigned int ram_size;

	ram_size = (unsigned int)(DDR_CHIP_0_SIZE) + (unsigned int)(DDR_CHIP_1_SIZE);
	if (ram_size > EMC_LOW_SDRAM_SPACE_SIZE)
		ram_size = EMC_LOW_SDRAM_SPACE_SIZE;

	return ram_size;
}
