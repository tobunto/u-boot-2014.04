/*
 * Timer for JZ4775, JZ4780
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <config.h>
#include <common.h>
#include <div64.h>
#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/arch/ost.h>

#define USEC_IN_1SEC 1000000

static inline uint32_t ost_readl(uint32_t off)
{
	return readl((void __iomem *)OST_BASE + off);
}

static inline void ost_writel(uint32_t val, uint32_t off)
{
	writel(val, (void __iomem *)OST_BASE + off);
}

int timer_init(void)
{
	ost_writel(OST2ENS, OSTECR);
	ost_writel(OSTCSR_PRESCALE(OST_DIV_16, OSTCSR_PRESCALE2), OSTCCR);
	ost_writel(OST2CLR, OSTCR);
	ost_writel(OST2ENS, OSTESR);
	return 0;
}

static uint64_t get_timer64(void)
{
	uint32_t low = ost_readl(OST2CNTL);
	uint32_t high = ost_readl(OSTCNT2HBUF);
	return ((uint64_t)high << 32) | low;
}

void __udelay(unsigned long usec)
{
	uint32_t tmo;
	uint64_t endtime;

	/* OST count increments at 1.5MHz */
	tmo = CONFIG_SYS_HZ_CLOCK / 1000;
	tmo *= usec;
	tmo /= 1000;

	endtime = get_timer64() + tmo;

	while (get_timer64() < endtime);
}

ulong get_timer(ulong base)
{
	return lldiv(get_timer64(), CONFIG_SYS_HZ_CLOCK / CONFIG_SYS_HZ) - base;
}

unsigned long long get_ticks(void)
{
	return get_timer64();
}

ulong get_tbclk(void)
{
	return CONFIG_SYS_HZ;
}
