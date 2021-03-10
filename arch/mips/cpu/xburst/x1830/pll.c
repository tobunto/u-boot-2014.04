/*
 * X1830 pll configuration
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

#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/x1830.h>

/**
 * Board CPCCR configure.
 * CONFIG_SYS_CPCCR_SEL should be define in [board].h
 */
#define CPCCR_CFG CONFIG_SYS_CPCCR_SEL

/*********set CPXPCR register************/
static void pll_set(void)
{
	/* Init APLL */
	writel(0x9508821, CPM_CPAPCR);
	while(!(readl(CPM_CPAPCR) & (1 << 3)));

	/* Init MPLL */
	writel(CONFIG_SYS_MPLL_MNOD | (1 << 0), CPM_CPMPCR);
	while(!(readl(CPM_CPMPCR) & (1 << 3)));

#ifdef CONFIG_SYS_EPLL_MNOD
	/* Init EPLL */
	writel(CONFIG_SYS_EPLL_MNOD | (1 << 0), CPM_CPEPCR);
	while(!(readl(CPM_CPEPCR) & (1 << 3)));
#endif

#ifdef CONFIG_SYS_VPLL_MNOD
	/* Init VPLL */
	writel(CONFIG_SYS_VPLL_MNOD | (1 << 0), CPM_CPVPCR);
	while(!(readl(CPM_CPVPCR) & (1 << 3)));
#endif
}

/*
 *bit 20 :22  使能分频值的写功能
 *
 * */
static void cpccr_init(void)
{
	unsigned int cpccr;

	/* change div 改变低24位 改变 分频值 */
	cpccr = (readl(CPM_CPCCR) & (0xff << 24))
		| (CPCCR_CFG & ~(0xff << 24))
		| (7 << 20);
	writel(cpccr,CPM_CPCCR);
	while(readl(CPM_CPCSR) & 0x7);

	/* change sel 改变高8位 选择时钟源 */
	cpccr = (CPCCR_CFG & (0xff << 24)) | (readl(CPM_CPCCR) & ~(0xff << 24));
	writel(cpccr,CPM_CPCCR);
}

int pll_init(void)
{
	pll_set();
	cpccr_init();
	return 0;
}
