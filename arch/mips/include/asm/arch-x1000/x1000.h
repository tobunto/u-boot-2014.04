/*
 * X1000 cpm definitions
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
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

/* AHB0 BUS Devices */
#define	DDRC_BASEN	0xb3010000
#define DDRC_BASE	0xb34f0000

#define CPM_CPCCR	(0x00)
#define CPM_CPCSR	(0xd4)

#define CPM_DDRCDR	(0x2c)
#define CPM_VPUCDR	(0x30)
#define CPM_CPSPR	(0x34)
#define CPM_CPSPPR	(0x38)
#define CPM_USBPCR	(0x3c)
#define CPM_USBRDT	(0x40)
#define CPM_USBVBFIL	(0x44)
#define CPM_USBPCR1	(0x48)
#define CPM_USBCDR	(0x50)
#define CPM_MACCDR	(0x54)
#define CPM_I2SCDR	(0x60)
#define CPM_LPCDR	(0x64)
#define CPM_MSC0CDR	(0x68)
#define CPM_MSC1CDR	(0xa4)
#define CPM_MSC2CDR	(0xa8)
#define CPM_UHCCDR	(0x6c)
#define CPM_SSICDR	(0x74)
#define CPM_CIMCDR	(0x7c)
#define CPM_CIM1CDR	(0x80)
#define CPM_PCMCDR	(0x84)
#define CPM_BCHCDR	(0xac)

#define CPM_MPHYCR	(0xe8)
#define CPM_DRCG	(0xd0)

#define CPM_CPAPCR	(0x10)
#define CPM_CPMPCR	(0x14)

#define CPM_INTR	(0xb0)
#define CPM_INTRE	(0xb4)
#define CPM_CPSPPR	(0x38)
#define CPM_CPPSR	(0x34)

#define CPM_USBPCR	(0x3c)
#define CPM_USBRDT	(0x40)
#define CPM_USBVBFIL	(0x44)
#define CPM_USBPCR1	(0x48)

#define CPM_LCR		(0x04)
#define CPM_PSWC0ST     (0x90)
#define CPM_PSWC1ST     (0x94)
#define CPM_PSWC2ST     (0x98)
#define CPM_PSWC3ST     (0x9c)
#define CPM_CLKGR	(0x20)
#define CPM_SRBC	(0xc4)
#define CPM_SLBC	(0xc8)
#define CPM_SLPC	(0xcc)
#define CPM_OPCR	(0x24)

#define CPM_RSR		(0x08)

#define LCR_LPM_MASK		(0x3)
#define LCR_LPM_SLEEP		(0x1)

#define CPM_LCR_PD_X2D		(0x1<<31)
#define CPM_LCR_PD_VPU		(0x1<<30)
#define CPM_LCR_PD_IPU		(0x1<<29)
#define CPM_LCR_PD_EPD		(0x1<<28)
#define CPM_LCR_PD_MASK		(0x7<<28)
#define CPM_LCR_X2DS 		(0x1<<27)
#define CPM_LCR_VPUS		(0x1<<26)
#define CPM_LCR_IPUS		(0x1<<25)
#define CPM_LCR_EPDS 		(0x1<<24)
#define CPM_LCR_STATUS_MASK 	(0xf<<24)

#define CPM_CLKGR_DDR		(1 << 31)
#define CPM_CLKGR_CPU		(1 << 30)
#define CPM_CLKGR_AHB0		(1 << 29)
#define CPM_CLKGR_APB0		(1 << 28)
#define CPM_CLKGR_RTC		(1 << 27)
#define CPM_CLKGR_PCM		(1 << 26)
#define CPM_CLKGR_MAC		(1 << 25)
#define CPM_CLKGR_AES		(1 << 24)
#define CPM_CLKGR_LCD		(1 << 23)
#define CPM_CLKGR_CIM		(1 << 22)
#define CPM_CLKGR_PDMA		(1 << 21)
#define CPM_CLKGR_OST		(1 << 20)
#define CPM_CLKGR_SSI		(1 << 19)
#define CPM_CLKGR_TCU		(1 << 18)
#define CPM_CLKGR_DMIC		(1 << 17)
#define CPM_CLKGR_UART2		(1 << 16)
#define CPM_CLKGR_UART1		(1 << 15)
#define CPM_CLKGR_UART0		(1 << 14)
#define CPM_CLKGR_SADC		(1 << 13)
#define CPM_CLKGR_JPEG		(1 << 12)
#define CPM_CLKGR_AIC		(1 << 11)
#define CPM_CLKGR_I2C3	        (1 << 10)
#define CPM_CLKGR_I2C2		(1 << 9)
#define CPM_CLKGR_I2C1		(1 << 8)
#define CPM_CLKGR_I2C0		(1 << 7)
#define CPM_CLKGR_SCC		(1 << 6)
#define CPM_CLKGR_MSC1		(1 << 5)
#define CPM_CLKGR_MSC0		(1 << 4)
#define CPM_CLKGR_OTG		(1 << 3)
#define CPM_CLKGR_SFC		(1 << 2)
#define CPM_CLKGR_EFUSE		(1 << 1)
#define CPM_CLKGR_NEMC		(1 << 0)

#define CPM_RSR_HR		(1 << 3)
#define CPM_RSR_P0R		(1 << 2)
#define CPM_RSR_WR		(1 << 1)
#define CPM_RSR_PR		(1 << 0)

#define OPCR_ERCS		(0x1<<2)
#define OPCR_PD			(0x1<<3)
#define OPCR_IDLE		(0x1<<31)

#define CLKGR_VPU              (0x1<<19)

#define cpm_readl(off)          readl(CPM_BASE + (off))
#define cpm_writel(val,off)     writel(val, CPM_BASE + (off))
#define cpm_inl(off)		readl(CPM_BASE + (off))
#define cpm_outl(val,off)	writel(val, CPM_BASE + (off))

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
#define CPM_MACCDR_MACPCS_BIT	31
#define CPM_MACCDR_MACPCS_MASK	(1 << CPM_MACCDR_MACPCS_BIT)
#define CPM_MACCDR_MACPCS_SCLKA	(0x0 << CPM_MACCDR_MACPCS_BIT)
#define CPM_MACCDR_MACPCS_MPLL	(0x1 << CPM_MACCDR_MACPCS_BIT)
#define CPM_MACCDR_CE			(1 << 29)
#define CPM_MACCDR_MAC_BUSY		(1 << 28)
#define CPM_MACCDR_MAC_STOP		(1 << 27)
#define CPM_MACCDR_MACDIV_BIT	0
#define CPM_MACCDR_MACDIV_MASK	(0xff << CPM_MACCDR_MACDIV_BIT)

/* MSC clock divider register */
#define CPM_MSCCDR_MPCS_BIT		31
#define CPM_MSCCDR_MPCS_MASK	(1 << CPM_MSCCDR_MPCS_BIT)
#define CPM_MSCCDR_MPCS_SCLKA	(0x0 << CPM_MSCCDR_MPCS_BIT)
#define CPM_MSCCDR_MPCS_MPLL	(0x1 << CPM_MSCCDR_MPCS_BIT)
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
#define USBPCR1_WORD_IF_16_30	(1 << 19)

/* SSI clock divider register */
#define CPM_SSICDR_SSIPCS_BIT		31
#define CPM_SSICDR_SSIPCS_MASK		(1 << CPM_SSICDR_SSIPCS_BIT)
#define CPM_SSICDR_SSIPCS_SCLKA		(0x0 << CPM_SSICDR_SSIPCS_BIT)
#define CPM_SSICDR_SSIPCS_MPLL		(0x1 << CPM_SSICDR_SSIPCS_BIT)
#define CPM_SSICDR_SSISCS_BIT		30
#define CPM_SSICDR_SSISCS_MASK		(1 << CPM_SSICDR_SSISCS_BIT)
#define CPM_SSICDR_SSISCS_EXCLK		(0x0 << CPM_SSICDR_SSIPCS_BIT)
#define CPM_SSICDR_SSISCS_PLL_DIV2	(0x1 << CPM_SSICDR_SSIPCS_BIT)
#define CPM_SSICDR_CE				(1 << 29)
#define CPM_SSICDR_SSI_BUSY			(1 << 28)
#define CPM_SSICDR_SSI_STOP			(1 << 27)
#define CPM_SSICDR_SSIDIV_BIT		0
#define CPM_SSICDR_SSIDIV_MASK		(0xff << CPM_SSICDR_SSIDIV_BIT)

/*OPCR*/
#define OPCR_SPENDN		(1 << 7)

/* CPM scratch pad protected register(CPSPPR) */
#define CPSPPR_CPSPR_WRITABLE   (0x00005a5a)
#define RECOVERY_SIGNATURE      (0x1a1a)        /* means "RECY" */
#define RECOVERY_SIGNATURE_SEC  0x800           /* means "RECY" */
#define FASTBOOT_SIGNATURE      (0x0666)        /* means "FASTBOOT" */

#define GPIO_PXPIN(n)	(GPIO_BASE + (0x00 + (n)*0x100)) /* PIN Level Register */
#define GPIO_PXINT(n)	(GPIO_BASE + (0x10 + (n)*0x100)) /* Port Interrupt Register */
#define GPIO_PXINTS(n)	(GPIO_BASE + (0x14 + (n)*0x100)) /* Port Interrupt Set Register */
#define GPIO_PXINTC(n)	(GPIO_BASE + (0x18 + (n)*0x100)) /* Port Interrupt Clear Register */
#define GPIO_PXMASK(n)	(GPIO_BASE + (0x20 + (n)*0x100)) /* Port Interrupt Mask Register */
#define GPIO_PXMASKS(n)	(GPIO_BASE + (0x24 + (n)*0x100)) /* Port Interrupt Mask Set Reg */
#define GPIO_PXMASKC(n)	(GPIO_BASE + (0x28 + (n)*0x100)) /* Port Interrupt Mask Clear Reg */
#define GPIO_PXPAT1(n)	(GPIO_BASE + (0x30 + (n)*0x100)) /* Port Pattern 1 Register */
#define GPIO_PXPAT1S(n)	(GPIO_BASE + (0x34 + (n)*0x100)) /* Port Pattern 1 Set Reg. */
#define GPIO_PXPAT1C(n)	(GPIO_BASE + (0x38 + (n)*0x100)) /* Port Pattern 1 Clear Reg. */
#define GPIO_PXPAT0(n)	(GPIO_BASE + (0x40 + (n)*0x100)) /* Port Pattern 0 Register */
#define GPIO_PXPAT0S(n)	(GPIO_BASE + (0x44 + (n)*0x100)) /* Port Pattern 0 Set Register */
#define GPIO_PXPAT0C(n)	(GPIO_BASE + (0x48 + (n)*0x100)) /* Port Pattern 0 Clear Register */
#define GPIO_PXFLG(n)	(GPIO_BASE + (0x50 + (n)*0x100)) /* Port Flag Register */
#define GPIO_PXFLGC(n)	(GPIO_BASE + (0x58 + (n)*0x100)) /* Port Flag clear Register */
#define GPIO_PXPEN(n)	(GPIO_BASE + (0x70 + (n)*0x100)) /* Port Pull Disable Register */
#define GPIO_PXPENS(n)	(GPIO_BASE + (0x74 + (n)*0x100)) /* Port Pull Disable Set Register */
#define GPIO_PXPENC(n)	(GPIO_BASE + (0x78 + (n)*0x100)) /* Port Pull Disable Clear Register */

#define GPIO_PA(n) 	(0*32 + n)
#define GPIO_PB(n) 	(1*32 + n)
#define GPIO_PC(n) 	(2*32 + n)
#define GPIO_PD(n) 	(3*32 + n)


#define DDR_PHY_OFFSETN	0x1000
#define DDR_PHY_OFFSET	(-0x4e0000 + 0x1000)

#define DDRP_DCR	(DDRC_BASEN + DDR_PHY_OFFSETN + 0x30) /* DRAM Configuration Register*/

/* DDRP DRAM Configuration Register */
#define DDRP_DCR_TYPE_BIT	0
#define DDRP_DCR_TYPE_MASK	(0x7 << DDRP_DCR_TYPE_BIT)
#define DDRP_DCR_TYPE_MDDR	(0 << DDRP_DCR_TYPE_BIT)
#define DDRP_DCR_TYPE_DDR	(1 << DDRP_DCR_TYPE_BIT)
#define DDRP_DCR_TYPE_DDR2	(2 << DDRP_DCR_TYPE_BIT)
#define DDRP_DCR_TYPE_DDR3	(3 << DDRP_DCR_TYPE_BIT)
#define DDRP_DCR_TYPE_LPDDR2	(4 << DDRP_DCR_TYPE_BIT)
#define DDRP_DCR_DDR8BNK_BIT	3
#define DDRP_DCR_DDR8BNK_MASK	(1 << DDRP_DCR_DDR8BNK_BIT)
#define DDRP_DCR_DDR8BNK	(1 << DDRP_DCR_DDR8BNK_BIT)
#define DDRP_DCR_DDR8BNK_DIS	(0 << DDRP_DCR_DDR8BNK_BIT)





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
