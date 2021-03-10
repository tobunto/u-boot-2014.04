 /*
 * Copyright (c) 2016 Ingenic Semiconductor Co.,Ltd
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
#ifndef __ZEBRA__
#define	__ZEBRA__

/**
 * Basic configuration(SOC, Cache, UART, DDR).
 */
#define CONFIG_MIPS32		/* MIPS32 CPU core */
#define CONFIG_CPU_XBURST2
#define CONFIG_SYS_LITTLE_ENDIAN
#define CONFIG_X2000	/* x2000 SoC */


#define CONFIG_SYS_APLL_FREQ		1200000000	/*If APLL not use mast be set 0*/
#define CONFIG_SYS_MPLL_FREQ		1500000000	/*If MPLL not use mast be set 0*/
#define CONFIG_SYS_EPLL_FREQ		300000000	/*If MPLL not use mast be set 0*/
#define CONFIG_CPU_SEL_PLL		APLL
#define CONFIG_DDR_SEL_PLL		MPLL
#define CONFIG_SYS_CPU_FREQ		1200000000
#define CONFIG_SYS_MEM_FREQ		750000000

/* CLK CGU */
#define  CGU_CLK_SRC {				\
		{LCD, MPLL},			\
		{MSC0, MPLL},			\
		{MSC2, MPLL},			\
		{SFC, MPLL},			\
		{CIM, MPLL},			\
		{SRC_EOF,SRC_EOF}		\
	}

#define CONFIG_SYS_EXTAL		24000000	/* EXTAL freq: 24 MHz */
#define CONFIG_SYS_HZ			1000		/* incrementer freq */

#define CONFIG_SYS_BOOTM_LEN	(128 << 20)

/**
 *  Cache Configs:
 *  	Must implement DCACHE/ICACHE SCACHE according to xburst spec.
 * */
#define CONFIG_SYS_DCACHE_SIZE		(32 * 1024)
#define CONFIG_SYS_DCACHELINE_SIZE	(32)
#define CONFIG_SYS_DCACHE_WAYS		(8)
#define CONFIG_SYS_ICACHE_SIZE		(32 * 1024)
#define CONFIG_SYS_ICACHELINE_SIZE	(32)
#define CONFIG_SYS_ICACHE_WAYS		(8)
#define CONFIG_SYS_CACHELINE_SIZE	(32)
/* A switch to configure whether cpu has a 2nd level cache */
#define CONFIG_BOARD_SCACHE
#define CONFIG_SYS_SCACHE_SIZE		(512 * 1024)
#define CONFIG_SYS_SCACHELINE_SIZE	(64)
#define CONFIG_SYS_SCACHE_WAYS		(8)


#define CONFIG_SYS_UART_INDEX		3
#define CONFIG_BAUDRATE			115200

#define CONFIG_JZSOC
/* NS16550-ish UARTs, uart[0134] are accessible */
#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_SYS_NS16550_CLK		CONFIG_SYS_EXTAL
#define CONFIG_SYS_NS16550_COM1		0xb0030000 /* uart0 */
#define CONFIG_SYS_NS16550_COM2		0xb0031000 /* uart1 */
#define CONFIG_SYS_NS16550_COM3		0xb0032000 /* uart2 */
#define CONFIG_SYS_NS16550_COM4		0xb0033000 /* uart3 */
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_CONSOLE_MUX
#define CONFIG_CONS_INDEX		4
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE \
	{ 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 115200, 230400 }

#define CONFIG_DDR_INNOPHY
#define CONFIG_DDR_DLL_OFF
#define CONFIG_DDR_PARAMS_CREATOR
#define CONFIG_DDR_HOST_CC
/* #define CONFIG_DDR_TYPE_DDR3 */
#define CONFIG_DDR_TYPE_LPDDR3
/*#define CONFIG_DDR_TYPE_LPDDR2*/
#define CONFIG_DDR_CS0			1	/* 1-connected, 0-disconnected */
#define CONFIG_DDR_CS1			0	/* 1-connected, 0-disconnected */
#define CONFIG_DDR_DW32			0	/* 1-32bit-width, 0-16bit-width */
/*#define CONFIG_DDR3_TSD34096M1333C9_E*/

#ifdef CONFIG_DDR_TYPE_LPDDR2
#define CONFIG_LPDDR2_FMT4D32UAB_25LI_FPGA
	/* #define CONFIG_LPDDR2_AD210032F_AB_FPGA */
#endif

#ifdef CONFIG_DDR_TYPE_LPDDR3
	/* #define CONFIG_LPDDR3_MT52L256M32D1PF_FPGA*/
	/* #define CONFIG_LPDDR3_AD310032C_AB_FPGA */
	#define CONFIG_LPDDR3_W63AH6NKB_BI
#endif

#define CONFIG_DDR_PHY_IMPEDANCE 40
#define CONFIG_DDR_PHY_ODT_IMPEDANCE 120
/* #define CONFIG_FPGA_TEST */
/*#define CONFIG_DDR_AUTO_REFRESH_TEST*/

#define CONFIG_DDR_AUTO_SELF_REFRESH
#define CONFIG_DDR_AUTO_SELF_REFRESH_CNT 257

/*pmu slp pin*/
/*#define CONFIG_REGULATOR*/
#ifdef  CONFIG_REGULATOR
#define CONFIG_JZ_PMU_SLP_OUTPUT1
#define CONFIG_INGENIC_SOFT_I2C
#define CONFIG_PMU_RICOH6x
#define CONFIG_RICOH61X_I2C_SCL	GPIO_PC(25)
#define CONFIG_RICOH61X_I2C_SDA	GPIO_PC(26)
#define CONFIG_SOFT_I2C_READ_REPEATED_START
#endif


#if defined(CONFIG_SPL_SFC_NOR) || defined(CONFIG_SPL_SFC_NAND)
#define CONFIG_SPL_SFC_SUPPORT
#define CONFIG_SPL_VERSION	1
#endif

/**
 * Boot arguments definitions.
 */

/* #define BOOTARGS_COMMON "console=ttyS3,115200 mem=96M@0x0 rmem=32M@0x6000000"*/
#define BOOTARGS_COMMON "console=ttyS3,115200 mem=128M@0x0"

#ifdef CONFIG_BOOT_ANDROID
  #if defined(CONFIG_SPL_NOR_SUPPORT)
    #define CONFIG_BOOTARGS BOOTARGS_COMMON " ip=192.168.10.211:192.168.10.1:192.168.10.1:255.255.255.0 nfsroot=192.168.4.13:/home/nfsroot/fpga/user/wqshao/android/root rw noinitrd init=/init"
  #else
    #define CONFIG_BOOTARGS BOOTARGS_COMMON " ip=off root=/dev/ram0 rw rdinit=/init androidboot.hardware=dorado androidboot.selinux=permissive"
  #endif
#else
  #if defined(CONFIG_SPL_JZMMC_SUPPORT) || defined(CONFIG_SPL_MMC_SUPPORT)
	#define CONFIG_BOOTARGS BOOTARGS_COMMON " rootfstype=ext4 root=/dev/mmcblk0p2 rootdelay=3 rw"
  #elif defined(CONFIG_SPL_NOR_SUPPORT)
  /*#define CONFIG_BOOTARGS  BOOTARGS_COMMON " ip=192.168.10.210:192.168.10.1:192.168.10.1:255.255.255.0 nfsroot=192.168.4.13:/home/nfsroot/fpga/user/bliu/root_ok rw" */
    /*#define CONFIG_BOOTARGS  BOOTARGS_COMMON " ip=off root=/dev/ram0 rw rdinit=/linuxrc"*/
    #define CONFIG_BOOTARGS BOOTARGS_COMMON " ip=192.168.10.207:192.168.10.1:192.168.10.1:255.255.255.0 nfsroot=192.168.4.13:/home/nfsroot/fpga/user/pzqi/rootfs-tst rw"
  #elif defined(CONFIG_SPL_SFC_NOR)
	#define CONFIG_BOOTARGS BOOTARGS_COMMON "ip=off init=/linuxrc rootfstype=jffs2 root=/dev/mtdblock2 rw"
  #elif defined(CONFIG_SPL_SFC_NAND)
	#define CONFIG_BOOTARGS BOOTARGS_COMMON "ip=off init=/linuxrc ubi.mtd=2 root=ubi0:rootfs ubi.mtd=3 rootfstype=ubifs rw"
  #else
    #define CONFIG_BOOTARGS BOOTARGS_COMMON " ubi.mtd=1 root=ubi0:root rootfstype=ubifs rw"
  #endif
#endif



/**
 * Boot command definitions.
 */
#define CONFIG_BOOTDELAY 1

#if defined(CONFIG_SPL_NOR_SUPPORT)
#define CONFIG_BOOTCOMMAND "tftpboot 0x80600000 user/pzqi/uImage; bootm 0x80600000"
/*#define CONFIG_BOOTCOMMAND "loady 0x80600000; bootm 0x80600000"*/
#elif defined(CONFIG_SPL_JZMMC_SUPPORT) || defined(CONFIG_SPL_MMC_SUPPORT)
#define CONFIG_BOOTCOMMAND "mmc dev 0; mmc read 0x80600000 0x1800 0x3000; bootm 0x80600000"
#elif defined(CONFIG_SPL_SFC_NOR)
#define CONFIG_BOOTCOMMAND "sfcnor read 0x40000 0x300000 0x80800000 ;bootm 0x80800000"
#elif defined(CONFIG_SPL_SFC_NAND)
#define CONFIG_BOOTCOMMAND "sfcnand read 0x100000 0x400000 0x80600000 ;bootm 0x80600000"
#else
#define CONFIG_BOOTCOMMAND						\
"mtdparts default; ubi part system; ubifsmount ubi:boot; "	\
"ubifsload 0x80f00000 vmlinux.ub; bootm 0x80f00000"
#endif

#define PARTITION_NUM 10


/**
 * Drivers configuration.
 */

/* MMC */
#ifdef CONFIG_JZ_MMC_MSC0
#define CONFIG_GENERIC_MMC
#define CONFIG_MMC
#define CONFIG_SDHCI
#define CONFIG_JZ_SDHCI
/*#define CONFIG_MMC_SDMA*/

/* MSC Command configuration */
#define CONFIG_CMD_MMC

#define CONFIG_JZ_MMC_MSC0_PD   //set gpio
/*#define CONFIG_MMC_TRACE		// only for DEBUG*/
/*#define CONFIG_SDHCI_TRACE	// only for DEBUG*/
#endif

#ifdef CONFIG_JZ_MMC_MSC1
#define CONFIG_GENERIC_MMC
#define CONFIG_MMC
#define CONFIG_SDHCI
#define CONFIG_JZ_SDHCI
/*#define CONFIG_MMC_SDMA*/

/* MSC Command configuration */
#define CONFIG_CMD_MMC

#define CONFIG_JZ_MMC_MSC1_PE   //set gpio
/*#define CONFIG_MMC_TRACE		// only for DEBUG*/
/*#define CONFIG_SDHCI_TRACE	// only for DEBUG*/
#endif

#ifdef CONFIG_JZ_MMC_MSC2
#define CONFIG_GENERIC_MMC
#define CONFIG_MMC
#define CONFIG_SDHCI
#define CONFIG_JZ_SDHCI
/*#define CONFIG_MMC_SDMA*/

/* MSC Command configuration */
#define CONFIG_CMD_MMC

#define CONFIG_JZ_MMC_MSC2_PE   //set gpio
/*#define CONFIG_MMC_TRACE		// only for DEBUG*/
/*#define CONFIG_SDHCI_TRACE	// only for DEBUG*/
#endif


/* SFC */
#define CONFIG_SFC_V20

/* sfc gpio */
/* #define CONFIG_JZ_SFC_PD_4BIT */
/* #define CONFIG_JZ_SFC_PD_8BIT */
/* #define CONFIG_JZ_SFC_PD_8BIT_PULL */
#define CONFIG_JZ_SFC_PE

/* sfc ota config */
#ifdef CONFIG_OTA_VERSION30
#define CONFIG_KUNPENG_OTA_VERSION20
#endif

/* sfc nor config */
#ifdef CONFIG_SPL_SFC_NOR
#define CONFIG_JZ_SFC
#define CONFIG_CMD_SFC_NOR
#define CONFIG_JZ_SFC_NOR
#define CONFIG_SPI_SPL_CHECK
#define CONFIG_SFC_NOR_RATE	296000000	/* value <= 296000000(sfc 74Mhz)*/
#define CONFIG_SFC_QUAD
#define CONFIG_SPIFLASH_PART_OFFSET		0x5800
#define CONFIG_SPI_NORFLASH_PART_OFFSET		0x5874
#define CONFIG_NOR_MAJOR_VERSION_NUMBER		1
#define CONFIG_NOR_MINOR_VERSION_NUMBER		0
#define CONFIG_NOR_REVERSION_NUMBER		0
#define CONFIG_NOR_VERSION     (CONFIG_NOR_MAJOR_VERSION_NUMBER | (CONFIG_NOR_MINOR_VERSION_NUMBER << 8) | (CONFIG_NOR_REVERSION_NUMBER <<16))
#endif

/* sfc nand config */
#ifdef  CONFIG_SPL_SFC_NAND
#define CONFIG_SFC_NAND_RATE    296000000	/* value <= 296000000(sfc 74Mhz)*/
#define CONFIG_SFC_QUAD
#define CONFIG_SPI_SPL_CHECK
#define CONFIG_SPIFLASH_PART_OFFSET		0x5800
#define CONFIG_SPI_NAND_BPP                     (2048 +64)      /*Bytes Per Page*/
#define CONFIG_SPI_NAND_PPB                     (64)            /*Page Per Block*/
#define CONFIG_JZ_SFC
#define CONFIG_CMD_SFCNAND
#define CONFIG_CMD_NAND
#define CONFIG_SYS_MAX_NAND_DEVICE		1
#define CONFIG_SYS_NAND_BASE			0xb3441000
#define CONFIG_SYS_MAXARGS			16

/* sfc nand env config */
#define CONFIG_MTD_DEVICE
#define CONFIG_CMD_SAVEENV		/* saveenv */
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_PARTITIONS
#define MTDIDS_DEFAULT                  "nand0:nand"
#define MTDPARTS_DEFAULT                "mtdparts=nand:1M(boot),8M(kernel),40M(rootfs),-(data)"
#define CONFIG_SYS_NAND_BLOCK_SIZE	(128 * 1024)
#endif

#define CONFIG_SYS_NAND_SELF_INIT

/* end of sfc */

/* Ethernet: gmac*/

/* DEBUG ETHERNET */
#define CONFIG_SERVERIP	 192.168.5.94
#define CONFIG_IPADDR		 192.168.5.33
#define CONFIG_GATEWAYIP        192.168.5.1
#define CONFIG_NETMASK          255.255.255.0
#define CONFIG_ETHADDR          00:11:22:33:44:55


#define GMAC_PHY_MII    0
#define GMAC_PHY_RMII   4
#define GMAC_PHY_GMII   0
#define GMAC_PHY_RGMII  1
#define CONFIG_SYS_RX_ETH_BUFFER 64

#define PHY_TYPE_DM9161	1
#define PHY_TYPE_88E1111	2
#define PHY_TYPE_8710A		3
#define PHY_TYPE_IP101G	4
#define PHY_TYPE_YT8511	5
#define CONFIG_NET_PHY_TYPE	PHY_TYPE_YT8511

#define CONFIG_SET_ETHADDR
#define CONFIG_NET_X2000

#ifdef CONFIG_NET_X2000

#define CONFIG_MAC_AXI_BUS

/* Select GMAC Controller */
/*#define CONFIG_GMAC0*/
#define CONFIG_GMAC1

/* Select GMAC Interface mode */
#define CONFIG_RGMII

#ifdef CONFIG_RGMII
#define CONFIG_NET_GMAC_PHY_MODE GMAC_PHY_RGMII
#else
#define CONFIG_NET_GMAC_PHY_MODE GMAC_PHY_RMII
#endif

#ifdef CONFIG_GMAC1
#define CONFIG_GAMAC_MODE_CTRL_ADDR 0xb00000e8
#define JZ_GMAC_BASE GMAC1_BASE
#define CONFIG_GMAC_CRLT_PORT 0x01/*GPIO_PORT_B*/
#define CONFIG_GMAC_CRLT_PORT_PINS (0xffff << 8)
#define CONFIG_GMAC_CRTL_PORT_INIT_FUNC 0x03/*GPIO_FUNC_3*/
#define CONFIG_GMAC_PHY_RESET	GPIO_PB(16)
#define CONFIG_GMAC_TX_CLK_DELAY 0x3f
#define CONFIG_GMAC_RX_CLK_DELAY 0
#endif

#define CONFIG_GMAC_CRTL_PORT_SET_FUNC 0x06/*GPIO_INPUT*/
#define CONFIG_GMAC_PHY_RESET_ENLEVEL	0
#endif /* CONFIG_NET_X2000 */
/* end of gmac */

/* GPIO */
#define CONFIG_JZ_GPIO

/**
 * Command configuration.
 */
#define CONFIG_CMD_BOOTD	/* bootd			*/
#define CONFIG_CMD_CONSOLE	/* coninfo			*/
#define CONFIG_CMD_DHCP 	/* DHCP support			*/
#define CONFIG_CMD_ECHO		/* echo arguments		*/
#define CONFIG_CMD_EXT4 	/* ext4 support			*/
#define CONFIG_CMD_FAT		/* FAT support			*/
/*#define CONFIG_USE_XYZMODEM*/	/* xyzModem 			*/
/*#define CONFIG_CMD_LOAD*/		/* serial load support 		*/
/*#define CONFIG_CMD_LOADB*/	/* loadb			*/
/*#define CONFIG_CMD_LOADS*/	/* loads			*/
#define CONFIG_CMD_MEMORY	/* md mm nm mw cp cmp crc base loop mtest */
#define CONFIG_CMD_MISC		/* Misc functions like sleep etc*/
#define CONFIG_CMD_NET		/* networking support			*/
#define CONFIG_CMD_PING
#define CONFIG_CMD_RUN		/* run command in env variable	*/
#define CONFIG_CMD_SETGETDCR	/* DCR support on 4xx		*/
#define CONFIG_CMD_SOURCE	/* "source" command support	*/
#define CONFIG_CMD_GETTIME
#define CONFIG_CMD_UNZIP        /* unzip from memory to memory  */
/*#define CONFIG_CMD_GPIO*/

#define CONFIG_EFI_PARTITION
#define CONFIG_EXT4_WRITE
#define CONFIG_PARTITION_UUIDS

#define CONFIG_SOFT_BURNER
#define CONFIG_CMD_MII
/*#define CONFIG_PHYLIB*/

#define CONFIG_CMD_DDR_TEST	/* DDR Test Command */

/**
 * Serial download configuration
 */
#define CONFIG_LOADS_ECHO	1	/* echo on for serial download */

/**
 * Miscellaneous configurable options
 */
/*#define CONFIG_DOS_PARTITION*/

#define CONFIG_LZO
#define CONFIG_RBTREE

#define CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_FLASH_BASE	0 /* init flash_base as 0 */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_MISC_INIT_R 1

#define CONFIG_BOOTP_MASK	(CONFIG_BOOTP_DEFAUL)

#define CONFIG_SYS_MAXARGS 16
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT CONFIG_SYS_BOARD "# "
#define CONFIG_SYS_CBSIZE 1024 /* Console I/O Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_SYS_MONITOR_LEN		(384 * 1024)
#define CONFIG_SYS_MALLOC_LEN		(64 * 1024 * 1024)
#define CONFIG_SYS_BOOTPARAMS_LEN	(128 * 1024)

#define CONFIG_SYS_SDRAM_BASE		0x80000000 /* cached (KSEG0) address */
#define CONFIG_SYS_SDRAM_MAX_TOP	0x90000000 /* don't run into IO space */
#define CONFIG_SYS_INIT_SP_OFFSET	0x400000
#define CONFIG_SYS_LOAD_ADDR		0x88000000
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		0x88000000

#define CONFIG_SYS_TEXT_BASE		0x80100000
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_TEXT_BASE

#define CONFIG_UBOOT_OFFSET             0x6000

/**
 * Environment
 */
#ifdef CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		0
#define CONFIG_ENV_SIZE			(32 << 10)
#define CONFIG_ENV_OFFSET		(CONFIG_SYS_MONITOR_LEN + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)
#elif defined(CONFIG_ENV_IS_IN_SFC)
#define CONFIG_CMD_SFC_NOR
#define CONFIG_ENV_SIZE                 (4 << 10)
#define CONFIG_ENV_OFFSET               0x3f000 /*write nor flash 252k address*/
#define CONFIG_CMD_SAVEENV
#endif

/**
 * SPL configuration
 */
#define CONFIG_SPL
#define CONFIG_SPL_FRAMEWORK

#define CONFIG_SPL_NO_CPU_SUPPORT_CODE
/*#define CONFIG_SPL_START_S_PATH		"$(CPUDIR)/$(SOC)"*/
#ifdef CONFIG_SPL_NOR_SUPPORT
#define CONFIG_SPL_LDSCRIPT		"$(CPUDIR)/$(SOC)/u-boot-nor-spl.lds"
#else
#define CONFIG_SPL_LDSCRIPT		"$(CPUDIR)/$(SOC)/u-boot-spl.lds"
#endif
#define CONFIG_SPL_PAD_TO		24576 /* equal to spl max size */

#define CONFIG_SPL_BOARD_INIT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_GPIO_SUPPORT
/*#define CONFIG_SPL_SERIAL_SUPPORT*/
/* #define CONFIG_SPL_I2C_SUPPORT */
/* #define CONFIG_SPL_REGULATOR_SUPPORT */
/* #define CONFIG_SPL_CORE_VOLTAGE		1300 */
#ifdef CONFIG_SPL_NOR_SUPPORT
#define CONFIG_SPL_TEXT_BASE		0xba000000
#else
#define CONFIG_SPL_TEXT_BASE		0xb2401000
#endif	/*CONFIG_SPL_NOR_SUPPORT*/
#define CONFIG_SPL_MAX_SIZE		(18 * 1024)

#define CONFIG_CMD_BOOTMENU
#define CONFIG_MENU
#define AUTOBOOT_KEYED
#define CONFIG_MENU_SHOW

/* Environment information */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"menucmd=bootmenu\0" \
	"bootmenu_0=Boot kernel from /dev/mmcblk0p2=mmc dev 0; mmc read 0x80600000 0x1800 0x3000; bootm 0x80600000\0" \
	"bootmenu_1=Boot kernel from /boot=run sdboot\0" \
	"bootmenu_2=U-Boot boot order=boot\0" \
	"bootmenu_delay=30\0" \
	""


#ifdef CONFIG_SPL_NOR_SUPPORT
#define CONFIG_SYS_UBOOT_BASE		(CONFIG_SPL_TEXT_BASE + CONFIG_SPL_PAD_TO - 0x40)	//0x40 = sizeof (image_header)
#define CONFIG_SYS_OS_BASE		0
#define CONFIG_SYS_SPL_ARGS_ADDR	0
#define CONFIG_SYS_FDT_BASE		0
#endif

/* MMC  spl stage */
#if defined(CONFIG_SPL_MMC_SUPPORT) || defined(CONFIG_SPL_JZMMC_SUPPORT)
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	82 /* 17k + 24k (17KB GPT offset and spl size CONFIG_SPL_PAD_TO) */
#define CONFIG_CMD_SAVEENV  /* saveenv */
/*#define CONFIG_SPL_JZ_MSC_BUS_8BIT	//only for emmc*/
  #ifdef CONFIG_SPL_JZMMC_SUPPORT
	#define CONFIG_SPL_JZSDHCI
  #endif
  #ifdef CONFIG_SPL_MMC_SUPPORT
	#define CONFIG_JZ_MMC_SPLMSC		//Configuration SPL stage msc controller use jz_sdhci driver
  #endif
#endif /* CONFIG_SPL_MMC_SUPPORT || CONFIG_SPL_JZMMC_SUPPORT */

/**
 * GPT configuration
 */
#ifdef CONFIG_GPT_CREATOR
#define CONFIG_GPT_TABLE_PATH	"board/$(BOARDDIR)"
#else
/* USE MBR + zero-GPT-table instead if no gpt table defined*/
#define CONFIG_MBR_P0_OFF	64mb
#define CONFIG_MBR_P0_END	556mb
#define CONFIG_MBR_P0_TYPE 	linux

#define CONFIG_MBR_P1_OFF	580mb
#define CONFIG_MBR_P1_END 	1604mb
#define CONFIG_MBR_P1_TYPE 	linux

#define CONFIG_MBR_P2_OFF	28mb
#define CONFIG_MBR_P2_END	58mb
#define CONFIG_MBR_P2_TYPE 	linux

#define CONFIG_MBR_P3_OFF	1609mb
#define CONFIG_MBR_P3_END	7800mb
#define CONFIG_MBR_P3_TYPE 	fat
#endif


/* Wrong keys. */
#define CONFIG_GPIO_RECOVERY           GPIO_PB(11)
#define CONFIG_GPIO_RECOVERY_ENLEVEL   0

#endif/*END OF __ZEBRA__*/
