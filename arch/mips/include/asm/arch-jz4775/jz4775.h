#ifndef __JZ4775_H__
#define __JZ4775_H__

#include <asm/arch/base.h>

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

#define CPM_MPHYCR	(0xe0)
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
#define CPM_CLKGR_EPDE		(1 << 27)
#define CPM_CLKGR_EPDC		(1 << 26)
#define CPM_CLKGR_LCD		(1 << 25)
#define CPM_CLKGR_CIM1		(1 << 24)
#define CPM_CLKGR_CIM0		(1 << 23)
#define CPM_CLKGR_UHC		(1 << 22)
#define CPM_CLKGR_GMAC		(1 << 21)
#define CPM_CLKGR_PDMA		(1 << 20)
#define CPM_CLKGR_VPU		(1 << 19)
#define CPM_CLKGR_UART3		(1 << 18)
#define CPM_CLKGR_UART2		(1 << 17)
#define CPM_CLKGR_UART1		(1 << 16)
#define CPM_CLKGR_UART0		(1 << 15)
#define CPM_CLKGR_SADC		(1 << 14)
#define CPM_CLKGR_PCM		(1 << 13)
#define CPM_CLKGR_MSC2		(1 << 12)
#define CPM_CLKGR_MSC1		(1 << 11)
#define CPM_CLKGR_AHB_MON	(1 << 10)
#define CPM_CLKGR_X2D		(1 << 9)
#define CPM_CLKGR_AIC		(1 << 8)
#define CPM_CLKGR_I2C2		(1 << 7)
#define CPM_CLKGR_I2C1		(1 << 6)
#define CPM_CLKGR_I2C0		(1 << 5)
#define CPM_CLKGR_SSI0		(1 << 4)
#define CPM_CLKGR_MSC0		(1 << 3)
#define CPM_CLKGR_OTG		(1 << 2)
#define CPM_CLKGR_BCH		(1 << 1)
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
#define cpm_test_bit(bit,off)	(cpm_inl(off) & 0x1<<(bit))
#define cpm_set_bit(bit,off)	(cpm_outl((cpm_inl(off) | 0x1<<(bit)),off))
#define cpm_clear_bit(bit,off)	(cpm_outl(cpm_inl(off) & ~(0x1 << bit), off))

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

/* MSC clock divider register */
#define CPM_MSCCDR_MPCS_BIT		30
#define CPM_MSCCDR_MPCS_MASK	(3 << CPM_MSCCDR_MPCS_BIT)
#define CPM_MSCCDR_MPCS_STOP	(0x0 << CPM_MSCCDR_MPCS_BIT)
#define CPM_MSCCDR_MPCS_SCLKA	(0x1 << CPM_MSCCDR_MPCS_BIT)
#define CPM_MSCCDR_MPCS_MPLL	(0x2 << CPM_MSCCDR_MPCS_BIT)
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

/*OPCR*/
#define OPCR_SPENDN		(1 << 7)

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
#define GPIO_PF(n) 	(5*32 + n)
#define GPIO_PG(n) 	(6*32 + n)

/*************************************************************************
 * GPIO (General-Purpose I/O Ports)
 *************************************************************************/
#define PXPIN		0x00   /* PIN Level Register */
#define PXINT		0x10   /* Port Interrupt Register */
#define PXINTS		0x14   /* Port Interrupt Set Register */
#define PXINTC		0x18   /* Port Interrupt Clear Register */
#define PXMSK		0x20   /* Port Interrupt Mask Reg */
#define PXMSKS		0x24   /* Port Interrupt Mask Set Reg */
#define PXMSKC		0x28   /* Port Interrupt Mask Clear Reg */
#define PXPAT1		0x30   /* Port Pattern 1 Set Reg. */
#define PXPAT1S		0x34   /* Port Pattern 1 Set Reg. */
#define PXPAT1C		0x38   /* Port Pattern 1 Clear Reg. */
#define PXPAT0		0x40   /* Port Pattern 0 Register */
#define PXPAT0S		0x44   /* Port Pattern 0 Set Register */
#define PXPAT0C		0x48   /* Port Pattern 0 Clear Register */
#define PXFLG		0x50   /* Port Flag Register */
#define PXFLGC		0x58   /* Port Flag clear Register */
#define PXPE		0x70   /* Port Pull Disable Register */
#define PXPES		0x74   /* Port Pull Disable Set Register */
#define PXPEC		0x78   /* Port Pull Disable Clear Register */

#define GPIO_PXPIN(n)	(GPIO_BASE + (PXPIN + (n)*0x100)) /* PIN Level Register */
#define GPIO_PXINT(n)	(GPIO_BASE + (PXINT + (n)*0x100)) /* Port Interrupt Register */
#define GPIO_PXINTS(n)	(GPIO_BASE + (PXINTS + (n)*0x100)) /* Port Interrupt Set Register */
#define GPIO_PXINTC(n)	(GPIO_BASE + (PXINTC + (n)*0x100)) /* Port Interrupt Clear Register */
#define GPIO_PXMASK(n)	(GPIO_BASE + (PXMSK + (n)*0x100)) /* Port Interrupt Mask Register */
#define GPIO_PXMASKS(n)	(GPIO_BASE + (PXMSKS + (n)*0x100)) /* Port Interrupt Mask Set Reg */
#define GPIO_PXMASKC(n)	(GPIO_BASE + (PXMSKC + (n)*0x100)) /* Port Interrupt Mask Clear Reg */
#define GPIO_PXPAT1(n)	(GPIO_BASE + (PXPAT1 + (n)*0x100)) /* Port Pattern 1 Register */
#define GPIO_PXPAT1S(n)	(GPIO_BASE + (PXPAT1S + (n)*0x100)) /* Port Pattern 1 Set Reg. */
#define GPIO_PXPAT1C(n)	(GPIO_BASE + (PXPAT1C + (n)*0x100)) /* Port Pattern 1 Clear Reg. */
#define GPIO_PXPAT0(n)	(GPIO_BASE + (PXPAT0 + (n)*0x100)) /* Port Pattern 0 Register */
#define GPIO_PXPAT0S(n)	(GPIO_BASE + (PXPAT0S + (n)*0x100)) /* Port Pattern 0 Set Register */
#define GPIO_PXPAT0C(n)	(GPIO_BASE + (PXPAT0C + (n)*0x100)) /* Port Pattern 0 Clear Register */
#define GPIO_PXFLG(n)	(GPIO_BASE + (PXFLG + (n)*0x100)) /* Port Flag Register */
#define GPIO_PXFLGC(n)	(GPIO_BASE + (PXFLGC + (n)*0x100)) /* Port Flag clear Register */
#define GPIO_PXPEN(n)	(GPIO_BASE + (PXPE + (n)*0x100)) /* Port Pull Disable Register */
#define GPIO_PXPENS(n)	(GPIO_BASE + (PXPES + (n)*0x100)) /* Port Pull Disable Set Register */
#define GPIO_PXPENC(n)	(GPIO_BASE + (PXPEC + (n)*0x100)) /* Port Pull Disable Clear Register */



#define DDRP_DCR	(DDR_PHY_OFFSET + 0x30) /* DRAM Configuration Register*/

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

enum gpio_function {
	GPIO_FUNC_0     = 0x00,  //0000, GPIO as function 0 / device 0
	GPIO_FUNC_1     = 0x01,  //0001, GPIO as function 1 / device 1
	GPIO_FUNC_2     = 0x02,  //0010, GPIO as function 2 / device 2
	GPIO_FUNC_3     = 0x03,  //0011, GPIO as function 3 / device 3
        GPIO_OUTPUT0    = 0x04,  //0100, GPIO output low  level
        GPIO_OUTPUT1    = 0x05,  //0101, GPIO output high level
	GPIO_INPUT	= 0x06,	 //0110, GPIO as input
	GPIO_RISE_EDGE  = 0x0b,	//1011, GPIO as rise edge interrupt 
};

enum gpio_port {
	GPIO_PORT_A,
	GPIO_PORT_B,
	GPIO_PORT_C,
	GPIO_PORT_D,
	GPIO_PORT_E,
	GPIO_PORT_F,
	GPIO_PORT_G,
	/* this must be last */
	GPIO_NR_PORTS,
};

struct jz_gpio_func_def {
	int port;
	int func;
	unsigned long pins;
};

void gpio_set_func(enum gpio_port n, enum gpio_function func, unsigned int pins);
void gpio_port_set_value(int port, int pin, int value);
void gpio_port_direction_input(int port, int pin);
void gpio_init(void);
void gpio_enable_pull(unsigned gpio);
void gpio_disable_pull(unsigned gpio);
void gpio_as_irq_high_level(unsigned gpio);
void gpio_as_irq_low_level(unsigned gpio);
void gpio_as_irq_rise_edge(unsigned gpio);
void gpio_as_irq_fall_edge(unsigned gpio);
void gpio_ack_irq(unsigned gpio);
int gpio_clear_flag(unsigned gpio);
int gpio_get_flag(unsigned int gpio);



static inline void gpio_port_set(int port, int pin, int value)
{
	if (value)
		writel(1 << pin, GPIO_PXPAT0S(port));
	else
		writel(1 << pin, GPIO_PXPAT0C(port));
}

static inline void gpio_port_direction_output(int port, int pin, int value)
{
	writel(1 << pin, GPIO_PXINTC(port));
	writel(1 << pin, GPIO_PXMASKS(port));
	writel(1 << pin, GPIO_PXPAT1C(port));

	gpio_port_set(port, pin, value);
}

static inline void gpio_direction_output(int gpio, int value)
{
	int port = gpio / 32;
	int pin = gpio % 32;
	gpio_port_direction_output(port, pin, value);
}

#endif /* __ASSEMBLY__ */

#endif	/* __JZ4780_H__ */
