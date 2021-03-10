/*
 * X2000 cpm definitions
 *
 * Copyright (c) 2016 Ingenic Semiconductor Co.,Ltd
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

#define CPM_CPCCR		(0x00)
#define CPM_CPCSR		(0xD4)
#define CPM_DDRCDR		(0x2C)
#define CPM_MACPHYCDR		(0x54)
#define CPM_MACTXCDR0		(0x58)
#define CPM_MACTXCDR1		(0xDC)
#define CPM_MACPTPCDR		(0x4c)
#define CPM_I2S0CDR		(0x60)
#define CPM_I2S0CDR1		(0x70)
#define CPM_I2S1CDR		(0x7C)
#define CPM_I2S1CDR1		(0x80)
#define CPM_I2S2CDR		(0x84)
#define CPM_I2S2CDR1		(0x88)
#define CPM_I2S3CDR		(0x8C)
#define CPM_I2S3CDR1		(0xA0)
#define CPM_AUDIOCR		(0xAC)
#define CPM_LPCDR		(0x64)
#define CPM_MSC0CDR		(0x68)
#define CPM_MSC1CDR		(0xA4)
#define CPM_MSC2CDR		(0xA8)
#define CPM_SFCCDR		(0x74)
#define CPM_SSICDR		(0x5C)
#define CPM_CIMCDR		(0x78)
#define CPM_PWMCDR		(0x6C)
#define CPM_ISPCDR		(0x30)
#define CPM_RSACDR		(0x50)
#define CPM_MACPHYC0		(0xE4)
#define CPM_MACPHYC1		(0xE8)
#define CPM_INTR		(0xB0)
#define CPM_INTRE		(0xB4)
#define CPM_SFTINT		(0xBC)
#define CPM_DRCG		(0xD0)
#define CPM_CPSPR		(0x34)
#define CPM_CPSPPR		(0x38)
#define CPM_USBPCR		(0x3C)
#define CPM_USBRDT		(0x40)
#define CPM_USBVBFIL		(0x44)
#define CPM_USBPCR1		(0x48)
#define CPM_CPPCR		(0x0C)
#define CPM_CPAPCR		(0x10)
#define CPM_CPMPCR		(0x14)
#define CPM_CPEPCR		(0x18)

#define CPM_LCR			(0x04)
#define CPM_PSWC0ST		(0x90)
#define CPM_PSWC1ST		(0x94)
#define CPM_PSWC2ST		(0x98)
#define CPM_PSWC3ST		(0x9C)
#define CPM_SRBC		(0xC4)
#define CPM_SLBC		(0xC8)
#define CPM_SLPC		(0xCC)
#define CPM_CLKGR0		(0x20)
#define CPM_CLKGR1		(0x28)
#define CPM_MEMPD0		(0xF8)
#define CPM_MEMPD1		(0xFC)
#define CPM_OPCR		(0x24)
#define CPM_MESTSEL		(0xEC)
#define CPM_EXCLK_DS		(0xE0)


#define LCR_LPM_MASK		(0x3)
#define LCR_LPM_SLEEP		(0x1)

#define CPM_CLKGR_DDR		(1 << 31)
#define CPM_CLKGR_IPU		(1 << 30)
#define CPM_CLKGR_AHB0		(1 << 29)
#define CPM_CLKGR_APB0		(1 << 28)
#define CPM_CLKGR_RTC		(1 << 27)
#define CPM_CLKGR_SSI1		(1 << 26)
#define CPM_CLKGR_RSA		(1 << 25)
#define CPM_CLKGR_AES		(1 << 24)
#define CPM_CLKGR_LCD		(1 << 23)
#define CPM_CLKGR_CIM		(1 << 22)
#define CPM_CLKGR_PDMA		(1 << 21)
#define CPM_CLKGR_OST		(1 << 20)
#define CPM_CLKGR_SSI0		(1 << 19)
#define CPM_CLKGR_TCU		(1 << 18)
#define CPM_CLKGR_DTRNG		(1 << 17)
#define CPM_CLKGR_UART2		(1 << 16)
#define CPM_CLKGR_UART1		(1 << 15)
#define CPM_CLKGR_UART0		(1 << 14)
#define CPM_CLKGR_SADC		(1 << 13)
#define CPM_CLKGR_HELIX		(1 << 12)
#define CPM_CLKGR_AUDIO		(1 << 11)
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

#define CPM_CLKGR_MIPI_DSI	(1 << 29)
#define CPM_CLKGR_MIPI_CSI	(1 << 28)
#define CPM_CLKGR_INTC		(1 << 26)
#define CPM_CLKGR_MSC2		(1 << 25)
#define CPM_CLKGR_GMAC1		(1 << 24)
#define CPM_CLKGR_GMAC0		(1 << 23)
#define CPM_CLKGR_UART9		(1 << 22)
#define CPM_CLKGR_UART8		(1 << 21)
#define CPM_CLKGR_UART7		(1 << 20)
#define CPM_CLKGR_UART6		(1 << 19)
#define CPM_CLKGR_UART5		(1 << 18)
#define CPM_CLKGR_UART4		(1 << 17)
#define CPM_CLKGR_UART3		(1 << 16)
#define CPM_CLKGR_SPDIF		(1 << 14)
#define CPM_CLKGR_DMIC		(1 << 13)
#define CPM_CLKGR_PCM		(1 << 12)
#define CPM_CLKGR_I2S3		(1 << 11)
#define CPM_CLKGR_I2S2		(1 << 10)
#define CPM_CLKGR_I2S1		(1 << 9)
#define CPM_CLKGR_I2S0		(1 << 8)
#define CPM_CLKGR_ROT		(1 << 7)
#define CPM_CLKGR_HASH		(1 << 6)
#define CPM_CLKGR_PWM		(1 << 5)
#define CPM_CLKGR_FELIX		(1 << 4)
#define CPM_CLKGR_ISP1		(1 << 3)
#define CPM_CLKGR_ISP0		(1 << 2)
#define CPM_CLKGR_I2C5		(1 << 1)
#define CPM_CLKGR_I2C4		(1 << 0)

/* DDR clock divider register */
#define CPM_DDRCDR_DCS_BIT		30
#define CPM_DDRCDR_DCS_MASK		(3 << CPM_DDRCDR_DCS_BIT)
#define CPM_DDRCDR_DCS_STOP		(0x0 << CPM_DDRCDR_DCS_BIT)
#define CPM_DDRCDR_DCS_SCLKA	(0x1 << CPM_DDRCDR_DCS_BIT)
#define CPM_DDRCDR_DCS_MPLL		(0x2 << CPM_DDRCDR_DCS_BIT)
#define CPM_DDRCDR_CE			(1 << 29)
#define CPM_DDRCDR_DDR_BUSY		(1 << 28)
#define CPM_DDRCDR_DDR_STOP		(1 << 27)
#define CPM_DDRCDR_DDRDIV_BIT	0
#define CPM_DDRCDR_DDRDIV_MASK	(0xf << CPM_DDRCDR_DDRDIV_BIT)

/* MAC clock divider register */
#define CPM_MACCDR_MACPCS_BIT	30
#define CPM_MACCDR_MACPCS_MASK	(3 << CPM_MACCDR_MACPCS_BIT)
#define CPM_MACCDR_MACPCS_SCLKA	(0x0 << CPM_MACCDR_MACPCS_BIT)
#define CPM_MACCDR_MACPCS_MPLL	(0x1 << CPM_MACCDR_MACPCS_BIT)
#define CPM_MACCDR_MACPCS_EPLL	(0x2 << CPM_MACCDR_MACPCS_BIT)
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
#define CPM_MSCCDR_MPCS_EXTCLK	(0x2 << CPM_MSCCDR_MPCS_BIT)
#define CPM_MSCCDR_CE			(1 << 29)
#define CPM_MSCCDR_MSC_BUSY		(1 << 28)
#define CPM_MSCCDR_MSC_STOP		(1 << 27)
#define CPM_MSCCDR_MSCDIV_BIT	0
#define CPM_MSCCDR_MSCDIV_MASK	(0xff << CPM_MSCCDR_MSCDIV_BIT)

#define cpm_inl(off)		readl(CPM_BASE + (off))
#define cpm_outl(val,off)	writel(val,CPM_BASE + (off))
#define cpm_clear_bit(val,off)	do{cpm_outl((cpm_inl(off) & ~(1<<(val))),off);}while(0)
#define cpm_set_bit(val,off)	do{cpm_outl((cpm_inl(off) |  (1<<val)),off);}while(0)
#define cpm_test_bit(val,off)	(cpm_inl(off) & (0x1<<val))

/* CPM scratch pad protected register(CPSPPR) */
#define CPSPPR_CPSPR_WRITABLE   (0x00005a5a)
#define RECOVERY_SIGNATURE      (0x1a1a)        /* means "RECY" */
#define RECOVERY_SIGNATURE_SEC  0x800           /* means "RECY" */
#define FASTBOOT_SIGNATURE      (0x0666)        /* means "FASTBOOT" */

#define cpm_get_scrpad()        readl(CPM_BASE + CPM_CPSPR)
#define cpm_set_scrpad(data)                    \
do {                                            \
	volatile int i = 0x3fff;                \
	writel(0x00005a5a,CPM_BASE + CPM_CPSPPR);		\
	while(i--);				\
	writel(data,CPM_BASE + CPM_CPSPR);			\
	writel(0x0000a5a5,CPM_BASE + CPM_CPSPPR);      	\
} while (0)


#define GPIO_PA(n) 	(0*32 + n)
#define GPIO_PB(n) 	(1*32 + n)
#define GPIO_PC(n) 	(2*32 + n)
#define GPIO_PD(n) 	(3*32 + n)
#define GPIO_PE(n) 	(4*32 + n)
#define GPIO_PF(n) 	(4*32 + n)

/*************************************************************************
 * GPIO (General-Purpose I/O Ports)
 *************************************************************************/
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
#define GPIO_PXPU(n)	(GPIO_BASE + (0x80 + (n)*0x100)) /* Port Pull Up Register */
#define GPIO_PXPUS(n)	(GPIO_BASE + (0x84 + (n)*0x100)) /* Port Pull Up Set Register */
#define GPIO_PXPUC(n)	(GPIO_BASE + (0x88 + (n)*0x100)) /* Port Pull Up Clear Register */
#define GPIO_PXPD(n)	(GPIO_BASE + (0x90 + (n)*0x100)) /* Port Pull Down Register */
#define GPIO_PXPDS(n)	(GPIO_BASE + (0x94 + (n)*0x100)) /* Port Pull Down Set Register */
#define GPIO_PXPDC(n)	(GPIO_BASE + (0x98 + (n)*0x100)) /* Port Pull Down Clear Register */

#ifndef __ASSEMBLY__

#include <asm/io.h>

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
