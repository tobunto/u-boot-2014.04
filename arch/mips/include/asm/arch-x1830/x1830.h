/*
 * X1830  cpm definitions
 *
 * Copyright (c) 2017 Ingenic Semiconductor Co.,Ltd
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

#ifndef __CPM_H__
#define __CPM_H__

#include <asm/arch/base.h>

/*************************************************************************
 * CPM (Clock reset and Power control Management)
 *************************************************************************/
#define CPM_CPCCR		(CPM_BASE+0x00) /* Clock control register	*/
#define CPM_CPCSR		(CPM_BASE+0xd4) /* Clock Status register	*/
#define CPM_CPPCR		(CPM_BASE+0x0c) /* PLL control register 	*/
#define CPM_CPAPCR		(CPM_BASE+0x10) /* APLL control Register	*/
#define CPM_CPMPCR		(CPM_BASE+0x14) /* MPLL control Register	*/
#define CPM_CPEPCR		(CPM_BASE+0x58) /* EPLL control Register	*/
#define CPM_CPVPCR		(CPM_BASE+0xe0) /* VPLL control Register	*/

#define CPM_DDRCDR	(0x2c)
//#define CPM_VPUCDR	(0x30)
#define CPM_HELIXCDR (0x30)
#define CPM_MACCDR	(0x54)

#define CPM_I2SCDR	(0x60)
#define CPM_I2SCDR1 (0x70)
#define CPM_LPCDR	(0x64)
#define CPM_MSC0CDR	(0x68)
#define CPM_MSC1CDR	(0xa4)
#define CPM_SSICDR	(0x74)
#define CPM_CIMCDR	(0x7c)
#define CPM_ISPCDR	(0x80)
#define CPM_GMACPHYC (0xE8)
#define CPM_DRCG	(0xd0)

#define CPM_INTR	(0xb0)
#define CPM_INTRE	(0xb4)
#define CPM_CPSPPR	(0x38)
#define CPM_CPSPR 	(0x34)

#define CPM_USBPCR	(0x3c)
#define CPM_USBRDT	(0x40)
#define CPM_USBVBFIL	(0x44)
#define CPM_USBPCR1	(0x48)

#define CPM_EXCLK_DS (0x8C)

/************power/reset management*********/
#define CPM_LCR		(0x04)
#define CPM_CLKGR0  (0x20)
#define CPM_CLKGR1  (0x28)

#define CPM_SRBC0   (0xc4)
#define CPM_SRBC1   (0xAC)
#define CPM_MESTSEL (0xEC)
#define CPM_ERNG    (0xD8)
#define CPM_RNG     (0xDC)
#define CPM_OPCR    (0x24)

#define MEMSCR0     (0xF0)
#define MEMSCR1     (0xF4)
#define MEMPDR0     (0xF8)
/******************************************/

#define CPM_RSR     (0x08)

#define LCR_LPM_MASK		(0x3)
#define LCR_LPM_SLEEP		(0x1)

#define CPM_LCR_PD_P0		(0x1<<31)
#define CPM_LCR_PD_P1		(0x1<<30)
#define CPM_LCR_PD_VPU		(0x1<<29)
#define CPM_LCR_PD_GPU		(0x1<<28)
#define CPM_LCR_PD_ISP		(0x1<<27)
#define CPM_LCR_PD_H2D		(0x1<<26)
#define CPM_LCR_P0S		(0x1<<25)
#define CPM_LCR_P1S		(0x1<<24)
#define CPM_LCR_VPUS 		(0x1<<23)
#define CPM_LCR_GPUS		(0x1<<22)
#define CPM_LCR_ISPS		(0x1<<21)
#define CPM_LCR_H2DS		(0x1<<20)
#define CPM_LCR_PD_DMIC		(0x1<<7)
#define CPM_LCR_DMIC_S		(0x1<<6)
#define CPM_LCR_IDLE_DS		(0x1<<3)
#define CPM_LCR_SLEEP_DS	(0x1<<2)

/**********CLKGR0 0x20**************/
#define CPM_CLKGR_DDR		(1 << 31)
#define CPM_CLKGR_TCU		(1 << 30)
#define CPM_CLKGR_RTC		(1 << 29)
#define CPM_CLKGR_DES		(1 << 28)
#define CPM_CLKGR_ISP		(1 << 23)
#define CPM_CLKGR_PDMA		(1 << 21)
#define CPM_CLKGR_SFC		(1 << 20)
#define CPM_CLKGR_UART1		(1 << 15)
#define CPM_CLKGR_UART0		(1 << 14)
#define CPM_CLKGR_SADC		(1 << 13)
#define CPM_CLKGR_DMIC		(1 << 12)
#define CPM_CLKGR_AIC		(1 << 11)
#define CPM_CLKGR_I2C1		(1 << 8)
#define CPM_CLKGR_I2C0		(1 << 7)
#define CPM_CLKGR_SSI0		(1 << 6)
#define CPM_CLKGR_MSC1		(1 << 5)
#define CPM_CLKGR_MSC0		(1 << 4)
#define CPM_CLKGR_OTG		(1 << 3)
#define CPM_CLKGR_EFUSE		(1 << 1)

/*********CLKGR1 0x28***************/
#define CPM_CLKGR1_CPU		(1 << 15)
#define CPM_CLKGR1_APB0		(1 << 14)
#define CPM_CLKGR1_RADIX    (1 << 12)
#define CPM_CLKGR1_SYS_OST	(1 << 11)
#define CPM_CLKGR1_AHB0		(1 << 10)
#define CPM_CLKGR1_LDC      (1 << 9)
#define CPM_CLKGR1_AHB1     (1 << 6)
#define CPM_CLKGR1_AES		(1 << 5)
#define CPM_CLKGR1_GMAC		(1 << 4)
#define CPM_CLKGR1_NCU      (1 << 3)
#define CPM_CLKGR1_IPU		(1 << 2)
#define CPM_CLKGR1_HELIX	(1 << 0)

#define CPM_RSR_HR		(1 << 3)
#define CPM_RSR_WR		(1 << 1)
#define CPM_RSR_PR		(1 << 0)

#define OPCR_ERCS		(0x1<<2)
#define OPCR_PD			(0x1<<3)
#define OPCR_IDLE		(0x1<<31)

/* DDR clock divider register */
#define CPM_DDRCDR_DCS_BIT		30
#define CPM_DDRCDR_DCS_MASK		(3 << CPM_DDRCDR_DCS_BIT)
#define CPM_DDRCDR_DCS_STOP		(0x0 << CPM_DDRCDR_DCS_BIT)
#define CPM_DDRCDR_DCS_SCLKA	(0x1 << CPM_DDRCDR_DCS_BIT)
#define CPM_DDRCDR_DCS_MPLL		(0x2 << CPM_DDRCDR_DCS_BIT)
#define CPM_DDRCDR_CE			(1 << 29)
#define CPM_DDRCDR_DDR_BUSY		(1 << 28)
#define CPM_DDRCDR_DDR_STOP		(1 << 27)
#define CPM_DDRCDR_GATE_EN		(1 << 26)
#define CPM_DDRCDR_DDRDIV_BIT	0
#define CPM_DDRCDR_DDRDIV_MASK	(0xf << CPM_DDRCDR_DDRDIV_BIT)

/* MAC clock divider register */
#define CPM_MACCDR_MACPCS_BIT	30
#define CPM_MACCDR_MACPCS_MASK	(3 << CPM_MACCDR_MACPCS_BIT)
#define CPM_MACCDR_MACPCS_SCLKA	(0x0 << CPM_MACCDR_MACPCS_BIT)
#define CPM_MACCDR_MACPCS_MPLL	(0x1 << CPM_MACCDR_MACPCS_BIT)
#define CPM_MACCDR_MACPCS_VPLL	(0x2 << CPM_MACCDR_MACPCS_BIT)
#define CPM_MACCDR_MACPCS_EPLL	(0x3 << CPM_MACCDR_MACPCS_BIT)
#define CPM_MACCDR_CE			(1 << 29)
#define CPM_MACCDR_MAC_BUSY		(1 << 28)
#define CPM_MACCDR_MAC_STOP		(1 << 27)
#define CPM_MACCDR_MACDIV_BIT	0
#define CPM_MACCDR_MACDIV_MASK	(0xff << CPM_MACCDR_MACDIV_BIT)

/* MSC clock divider register */
#define CPM_MSCCDR_MPCS_BIT		30
#define CPM_MSCCDR_MPCS_MASK	(3 << CPM_MSCCDR_MPCS_BIT)
#define CPM_MSCCDR_MPCS_SCLKA	(0x0 << CPM_MSCCDR_MPCS_BIT)
#define CPM_MSCCDR_MPCS_MPLL	(0x1 << CPM_MSCCDR_MPCS_BIT)
#define CPM_MSCCDR_MPCS_VPLL	(0x2 << CPM_MSCCDR_MPCS_BIT)
#define CPM_MSCCDR_MPCS_EPLL	(0x3 << CPM_MSCCDR_MPCS_BIT)
#define CPM_MSCCDR_CE			(1 << 29)
#define CPM_MSCCDR_MSC_BUSY		(1 << 28)
#define CPM_MSCCDR_MSC_STOP		(1 << 27)
#define CPM_MSCCDR_MSC_CLK0_SEL	(1 << 15)
#define CPM_MSCCDR_MSCDIV_BIT	0
#define CPM_MSCCDR_MSCDIV_MASK	(0xff << CPM_MSCCDR_MSCDIV_BIT)

/*USBCDR*/
#define USBCDR_UCS_PLL		(1 << 31)
#define USBCDR_UPCS_MPLL	(1 << 30)
#define USBCDR_CE_USB		(1 << 29)
#define USBCDR_USB_BUSY		(1 << 28)
#define USBCDR_USB_STOP		(1 << 27)
#define USBCDR_USB_DIS		(1 << 26)
#define USBCDR_MIPI_CS		(1 << 25)
#define USBCDR_USBCDR_MSK	(0xff)

/*USBPCR*/
#define USBPCR_USB_MODE_ORG	(1 << 31)
#define USBPCR_VBUSVLDEXT	(1 << 24)
#define USBPCR_VBUSVLDEXTSEL	(1 << 23)
#define USBPCR_POR		(1 << 22)
#define USBPCR_OTG_DISABLE	(1 << 20)

/*USBPCR1*/
#define USBPCR1_REFCLKSEL_BIT	(26)
#define USBPCR1_REFCLKSEL_MSK	(0x3 << USBPCR1_REFCLKSEL_BIT)
#define USBPCR1_REFCLKSEL_CORE	(0x2 << USBPCR1_REFCLKSEL_BIT)
#define USBPCR1_REFCLKSEL_EXT	(0x1 << USBPCR1_REFCLKSEL_BIT)
#define USBPCR1_REFCLKSEL_CSL	(0x0 << USBPCR1_REFCLKSEL_BIT)
#define USBPCR1_REFCLKDIV_BIT	(24)
#define USBPCR1_REFCLKDIV_MSK	(0X3 << USBPCR1_REFCLKDIV_BIT)
#define USBPCR1_REFCLKDIV_19_2M	(0x3 << USBPCR1_REFCLKDIV_BIT)
#define USBPCR1_REFCLKDIV_48M	(0x2 << USBPCR1_REFCLKDIV_BIT)
#define USBPCR1_REFCLKDIV_24M	(0x1 << USBPCR1_REFCLKDIV_BIT)
#define USBPCR1_REFCLKDIV_12M	(0x0 << USBPCR1_REFCLKDIV_BIT)
#define USBPCR1_WORD_IF0_16_30	(1 << 19)

/* SSI clock divider register */
#define CPM_SSICDR_SSIPCS_BIT		30
#define CPM_SSICDR_SSIPCS_MASK		(3 << CPM_SSICDR_SSIPCS_BIT)
#define CPM_SSICDR_SSIPCS_SCLKA		(0x0 << CPM_SSICDR_SSIPCS_BIT)
#define CPM_SSICDR_SSIPCS_MPLL		(0x1 << CPM_SSICDR_SSIPCS_BIT)
#define CPM_SSICDR_SSIPCS_VPLL		(0x2 << CPM_SSICDR_SSIPCS_BIT)
#define CPM_SSICDR_SSIPCS_EPLL		(0x3 << CPM_SSICDR_SSIPCS_BIT)
#define CPM_SSICDR_SSISCS_BIT		29
#define CPM_SSICDR_SSISCS_MASK		(1 << CPM_SSICDR_SSISCS_BIT)
#define CPM_SSICDR_SSISCS_EXCLK		(0x0 << CPM_SSICDR_SSIPCS_BIT)
#define CPM_SSICDR_SSISCS_PLL_DIV2	(0x1 << CPM_SSICDR_SSIPCS_BIT)
#define CPM_SSICDR_CE				(1 << 28)
#define CPM_SSICDR_SSI_BUSY			(1 << 27)
#define CPM_SSICDR_SSI_STOP			(1 << 26)
#define CPM_SSICDR_SSIDIV_BIT		0
#define CPM_SSICDR_SSIDIV_MASK		(0xff << CPM_SSICDR_SSIDIV_BIT)

/*OPCR*/
#define OPCR_SPENDN0		(1 << 7)

/* CPM scratch pad protected register(CPSPPR) */
#define CPSPPR_CPSPR_WRITABLE   (0x00005a5a)
#define RECOVERY_SIGNATURE      (0x1a1a)        /* means "RECY" */
#define RECOVERY_SIGNATURE_SEC  0x800           /* means "RECY" */
#define FASTBOOT_SIGNATURE      (0x0666)        /* means "FASTBOOT" */

#define GPIO_PA(n) 	(0*32 + n)
#define GPIO_PB(n) 	(1*32 + n)
#define GPIO_PC(n) 	(2*32 + n)
#define GPIO_PD(n) 	(3*32 + n)

#define GPIO_PXPIN(n)	(GPIO_BASE + (0x000 + (n)*0x1000)) /* PIN Level Register */
#define GPIO_PXINT(n)	(GPIO_BASE + (0x010 + (n)*0x1000)) /* Port Interrupt Register */
#define GPIO_PXINTS(n)	(GPIO_BASE + (0x014 + (n)*0x1000)) /* Port Interrupt Set Register */
#define GPIO_PXINTC(n)	(GPIO_BASE + (0x018 + (n)*0x1000)) /* Port Interrupt Clear Register */
#define GPIO_PXMASK(n)	(GPIO_BASE + (0x020 + (n)*0x1000)) /* Port Interrupt Mask Register */
#define GPIO_PXMASKS(n)	(GPIO_BASE + (0x024 + (n)*0x1000)) /* Port Interrupt Mask Set Reg */
#define GPIO_PXMASKC(n)	(GPIO_BASE + (0x028 + (n)*0x1000)) /* Port Interrupt Mask Clear Reg */
#define GPIO_PXPAT1(n)	(GPIO_BASE + (0x030 + (n)*0x1000)) /* Port Pattern 1 Register */
#define GPIO_PXPAT1S(n)	(GPIO_BASE + (0x034 + (n)*0x1000)) /* Port Pattern 1 Set Reg. */
#define GPIO_PXPAT1C(n)	(GPIO_BASE + (0x038 + (n)*0x1000)) /* Port Pattern 1 Clear Reg. */
#define GPIO_PXPAT0(n)	(GPIO_BASE + (0x040 + (n)*0x1000)) /* Port Pattern 0 Register */
#define GPIO_PXPAT0S(n)	(GPIO_BASE + (0x044 + (n)*0x1000)) /* Port Pattern 0 Set Register */
#define GPIO_PXPAT0C(n)	(GPIO_BASE + (0x048 + (n)*0x1000)) /* Port Pattern 0 Clear Register */
#define GPIO_PXFLG(n)	(GPIO_BASE + (0x050 + (n)*0x1000)) /* Port Flag Register */
#define GPIO_PXFLGC(n)	(GPIO_BASE + (0x058 + (n)*0x1000)) /* Port Flag Clear Register */
#define GPIO_PXPEL(n)	(GPIO_BASE + (0x110 + (n)*0x1000)) /* Port Pull Register */
#define GPIO_PXPELS(n)	(GPIO_BASE + (0x114 + (n)*0x1000)) /* Port Pull Set Register */
#define GPIO_PXPELC(n)	(GPIO_BASE + (0x118 + (n)*0x1000)) /* Port Pull Clear Register */
#define GPIO_PXPEH(n)	(GPIO_BASE + (0x120 + (n)*0x1000)) /* Port Pull Register */
#define GPIO_PXPEHS(n)	(GPIO_BASE + (0x124 + (n)*0x1000)) /* Port Pull Set Register */
#define GPIO_PXPEHC(n)	(GPIO_BASE + (0x128 + (n)*0x1000)) /* Port Pull Clear Register */

#ifndef __ASSEMBLY__

#include <asm/io.h>

static inline int gpio_get_value(unsigned gpio)
{
	unsigned port = gpio / 32;
	unsigned pin = gpio % 32;

	return !!(readl(GPIO_PXPIN(port)) & (1 << pin));
}

static inline void gpio_port_set(int port, int pin, int value)
{
	if (value)
		writel(1 << pin, GPIO_PXPAT0S(port));
	else
		writel(1 << pin, GPIO_PXPAT0C(port));
}

static inline void gpio_port_direction_input(int port, int pin)
{
	writel(1 << pin, GPIO_PXINTC(port));
	writel(1 << pin, GPIO_PXMASKS(port));
	writel(1 << pin, GPIO_PXPAT1S(port));
}

static inline void gpio_port_direction_output(int port, int pin, int value)
{
	writel(1 << pin, GPIO_PXINTC(port));
	writel(1 << pin, GPIO_PXMASKS(port));
	writel(1 << pin, GPIO_PXPAT1C(port));

	gpio_port_set(port, pin, value);
}

static inline void gpio_direction_input(int gpio)
{
	int port = gpio / 32;
	int pin = gpio % 32;
	gpio_port_direction_input(port, pin);
}

static inline void gpio_direction_output(int gpio, int value)
{
	int port = gpio / 32;
	int pin = gpio % 32;
	gpio_port_direction_output(port, pin, value);
}

#endif /* __ASSEMBLY__ */

#endif /* __CPM_H__ */
