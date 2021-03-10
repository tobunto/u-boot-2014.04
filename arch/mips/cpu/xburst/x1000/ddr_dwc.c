/*
 * DDR driver for Synopsys DWC DDR PHY.
 * Used by Jz4775, JZ4780...
 *
 * Copyright (C) 2013 Ingenic Semiconductor Co.,Ltd
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
#include <asm/io.h>
#include <asm/arch/jzsoc.h>

#include <asm/arch/ddr_reg_values.h>

void ddr_controller_init(int bypass)
{
	ddr_writel(0, DDRC_CTRL);
	/* DDRC CFG init*/
	ddr_writel(DDRC_CFG_VALUE, DDRC_CFG);
	/* DDRC timing init*/
	ddr_writel(DDRC_TIMING1_VALUE, DDRC_TIMING(1));
	ddr_writel(DDRC_TIMING2_VALUE, DDRC_TIMING(2));
	ddr_writel(DDRC_TIMING3_VALUE, DDRC_TIMING(3));
	ddr_writel(DDRC_TIMING4_VALUE, DDRC_TIMING(4));
	ddr_writel(DDRC_TIMING5_VALUE, DDRC_TIMING(5));
	ddr_writel(DDRC_TIMING6_VALUE, DDRC_TIMING(6));

	/* DDRC memory map configure*/
	ddr_writel(DDRC_MMAP0_VALUE, DDRC_MMAP0);
	ddr_writel(DDRC_MMAP1_VALUE, DDRC_MMAP1);
	ddr_writel(DDRC_CTRL_CKE | DDRC_CTRL_ALH, DDRC_CTRL);
	ddr_writel(DDRC_REFCNT_VALUE, DDRC_REFCNT);
	ddr_writel(DDRC_CTRL_VALUE, DDRC_CTRL);

	/* memory remap */
	ddr_writel(0x03020d0c,DDRC_REMAP(1));
	ddr_writel(0x07060504,DDRC_REMAP(2));
	ddr_writel(0x0b0a0908,DDRC_REMAP(3));
	ddr_writel(0x0f0e0100,DDRC_REMAP(4));
	ddr_writel(0x13121110,DDRC_REMAP(5));

	ddr_writel(ddr_readl(DDRC_STATUS) & ~DDRC_DSTATUS_MISS, DDRC_STATUS);

	if(DDRC_AUTOSR_EN_VALUE){
		if(!bypass) {
			ddr_writel(0, DDRC_DLP);
		} else {
			ddr_writel((9 << 28) | 0xf,DDRC_CLKSTP_CFG);
		}
	}
	ddr_writel(DDRC_AUTOSR_EN_VALUE, DDRC_AUTOSR_EN);
}

static int ddr_training_hardware(int bypass)
{
	int result = 0;
	unsigned int wait_val = 0;
	unsigned int pir_val = DDRP_PIR_INIT | DDRP_PIR_QSTRN;

	wait_val = DDRP_PGSR_IDONE | DDRP_PGSR_DTDONE;
	if(bypass) {
		pir_val |= DDRP_PIR_DLLBYP | DDRP_PIR_LOCKBYP;
	}
	ddr_writel(pir_val, DDRP_PIR);
	while((ddr_readl(DDRP_PGSR) & wait_val) != wait_val);
	result = ddr_readl(DDRP_PGSR);
	if (!(result & (DDRP_PGSR_DTERR | DDRP_PGSR_DTIERR)))
		result = 0;

	return result;
}

static void ddr_phy_init_dram(int bypass)
{
	unsigned int pir_val;
	unsigned int val;
	unsigned int wait_val = 0;

	pir_val = DDRP_PIR_INIT | DDRP_PIR_DLLSRST | DDRP_PIR_ITMSRST | DDRP_PIR_DRAMINT | DDRP_PIR_DLLLOCK | DDRP_PIR_ZCAL;
	wait_val = DDRP_PGSR_IDONE | DDRP_PGSR_ZCDONE | DDRP_PGSR_DIDONE | DDRP_PGSR_DLDONE;

	pir_val &= ~(DDRP_PIR_ZCAL);
	wait_val &= ~(DDRP_PGSR_ZCDONE);

	if(bypass) {
		pir_val &= ~DDRP_PIR_DLLLOCK;
		pir_val &= ~DDRP_PIR_DLLSRST;
		pir_val |= DDRP_PIR_DLLBYP | DDRP_PIR_LOCKBYP;
		// DLL Disable: only bypassmode
		ddr_writel(0x1 << 31, DDRP_ACDLLCR);
		// 200M bypass.
		ddr_writel(ddr_readl(DDRP_DLLGCR) | (1 << 23), DDRP_DLLGCR);

		/*  LPDLLPD:  only for ddr bypass mode
		 * Low Power DLL Power Down: Specifies if set that the PHY should respond to the *
		 * DFI low power opportunity request and power down the DLL of the PHY if the *
		 * wakeup time request satisfies the DLL lock time */
	 	ddr_writel(ddr_readl(DDRP_DSGCR) & ~(1 << 4),DDRP_DSGCR);
		wait_val &= ~DDRP_PGSR_DLDONE;
	}

	val = DDRP_PGSR_IDONE | DDRP_PGSR_DLDONE | DDRP_PGSR_ZCDONE;
	while((ddr_readl(DDRP_PGSR) & val) != val);
	ddr_writel(pir_val, DDRP_PIR);
	while((ddr_readl(DDRP_PGSR) & wait_val) != wait_val);
}

static void force_no_slefresh(void)
{
	/* force CKE1 CS1 HIGH */
	ddr_writel(DDRC_CFG_VALUE | DDRC_CFG_CS1EN |  DDRC_CFG_CS0EN, DDRC_CFG);
	ddr_writel((1 << 1), DDRC_CTRL);
}
void ddr_phy_init(int bypass)
{
	writel(0x150000, DDRC_BASE + DDRP_DTAR);

	/* DDR training address set*/
	writel(DDRP_DCR_TYPE_MDDR | DDRP_DCR_DDR8BNK_DIS, DDRP_DCR);

	writel(DDRP_MR0_VALUE, DDRC_BASE + DDRP_MR0);
	writel(DDRP_MR2_VALUE, DDRC_BASE + DDRP_MR2);

	writel(DDRP_PTR0_VALUE, DDRC_BASE + DDRP_PTR0);
	writel(DDRP_PTR1_VALUE, DDRC_BASE + DDRP_PTR1);
	writel(DDRP_PTR2_VALUE, DDRC_BASE + DDRP_PTR2);

	writel(DDRP_DTPR0_VALUE, DDRC_BASE + DDRP_DTPR0);
	writel(DDRP_DTPR1_VALUE, DDRC_BASE + DDRP_DTPR1);
	writel(DDRP_DTPR2_VALUE, DDRC_BASE + DDRP_DTPR2);

	writel(DDRP_PGCR_VALUE, DDRC_BASE + DDRP_PGCR);

	writel(DDRP_DX0GCR_VALUE,DDRC_BASE + DDRP_DXGCR(0));
	writel(DDRP_DX1GCR_VALUE,DDRC_BASE + DDRP_DXGCR(1));
	writel(DDRP_DX2GCR_VALUE,DDRC_BASE + DDRP_DXGCR(2));
	writel(DDRP_DX3GCR_VALUE,DDRC_BASE + DDRP_DXGCR(3));

	writel(0x30c00813, DDRC_BASE + DDRP_ACIOCR);
	writel(0x4912, DDRC_BASE + DDRP_DXCCR);

	ddr_phy_init_dram(bypass);

	/* reset DDR ctrl */
	ddr_writel(0x2 << 21, DDRC_PHYRST_CFG);
	udelay(1000);
	ddr_writel(0, DDRC_PHYRST_CFG);
	udelay(1000);

	force_no_slefresh();
	ddr_training_hardware(bypass);
}

void sdram_init(void)
{
	unsigned int bypass = 0;
	uint32_t ddr_cdr;

	/* setup DDR clock */
	ddr_cdr = CONFIG_SYS_MPLL_FREQ / CONFIG_SYS_MEM_FREQ - 1;
	writel(ddr_cdr | CPM_DDRCDR_DCS_MPLL | CPM_DDRCDR_CE, CPM_BASE + CPM_DDRCDR);
	while (readl(CPM_BASE + CPM_DDRCDR) & CPM_DDRCDR_DDR_BUSY);

	if(CONFIG_SYS_MEM_FREQ < 300000000)
		bypass = 1;

	/*
	 * WARNING: 2015-01-08
	 * DDR CLK GATE(CPM_DRCG 0xB00000D0), BIT6 must set to 1 (or 0x40).
	 * If clear BIT6, chip memory will not stable, gpu hang occur.
	 */
	/* auto DDR clk gating */
	writel(0x73 | (1 << 6), CPM_BASE + 0xd0);
	udelay(1000);
	writel(0x71 | (1 << 6), CPM_BASE + 0xd0);
	udelay(1000);

	/* reset phy dll dfi and cfg */
	ddr_writel(0xd << 21 | 1, DDRC_PHYRST_CFG);
	udelay(1000);
	ddr_writel(0, DDRC_PHYRST_CFG);
	udelay(1000);

	/* DDR PHY init*/
	ddr_phy_init(bypass);
	/* DDR Controller init*/
	ddr_controller_init(bypass);

	if(bypass)
		cpm_outl(cpm_inl(CPM_DDRCDR) | CPM_DDRCDR_GATE_EN, CPM_DDRCDR);
}

phys_size_t initdram(int board_type)
{
	/* SDRAM size was calculated when compiling. */
#ifndef EMC_LOW_SDRAM_SPACE_SIZE
#define EMC_LOW_SDRAM_SPACE_SIZE 0x10000000 /* 256M */
#endif /* EMC_LOW_SDRAM_SPACE_SIZE */

	unsigned int ram_size;
	ddrc_cfg_t ddrc_cfg = {.d32 = ddr_readl(DDRC_CFG)};
	uint16_t col0 = ddrc_cfg.b.COL0 + 8, row0 = ddrc_cfg.b.ROW0 + 12;
	uint8_t dw = ddrc_cfg.b.DW ? 4 : 2, ba0 = ddrc_cfg.b.BA0 ? 8 : 4;
	uint32_t chip_0_szie = (uint32_t)(1 << (col0 + row0)) * dw * ba0;
	ram_size = chip_0_szie + (unsigned int)(DDR_CHIP_1_SIZE);
	if (ram_size > EMC_LOW_SDRAM_SPACE_SIZE)
		ram_size = EMC_LOW_SDRAM_SPACE_SIZE;

	return ram_size;
}
