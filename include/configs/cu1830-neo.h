/*
 * Ingenic CU1830-Neo configuration
 *
 * Copyright (c) 2017  Ingenic Semiconductor Co.,Ltd
 * Author: Zoro <ykli@ingenic.cn>
 * Based on: include/configs/urboard.h
 *           Written by Paul Burton <paul.burton@imgtec.com>
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

#ifndef __CONFIG_CU1830_NEO_H__
#define __CONFIG_CU1830_NEO_H__

/**
 * Basic configuration(SOC, Cache, UART, DDR).
 */
#define CONFIG_MIPS32		/* MIPS32 CPU core */
#define CONFIG_CPU_XBURST
#define CONFIG_SYS_LITTLE_ENDIAN
#define CONFIG_X1830
#define CONFIG_SYS_EXTAL			24000000	/* EXTAL freq: 24 MHz */
#define CONFIG_SYS_HZ_CLOCK			1500000		/* incrementer freq */

/**
 * PLL
 **/
#define CONFIG_SYS_APLL_FREQ		1500000000
#define CONFIG_SYS_APLL_MNOD		((249 << 20) | (3 << 14) | (1 << 11) | (1 << 5))
#define CONFIG_SYS_MPLL_FREQ		1200000000
#define CONFIG_SYS_MPLL_MNOD		((149 << 20) | (2 << 14) | (1 << 11) | (1<<5))
/*#define CONFIG_SYS_MPLL_FREQ		600000000
#define CONFIG_SYS_MPLL_MNOD		((99 << 20) | (1 << 14) | (2 << 11) | (2 << 5))*/
#define CONFIG_SYS_VPLL_FREQ		0			/*If VPLL not use mast be set 0*/
#define CONFIG_SYS_EPLL_FREQ		0			/*If EPLL not use mast be set 0*/

/**
 * CACHE
 **/
#define CONFIG_SYS_DCACHE_SIZE		32768
#define CONFIG_SYS_ICACHE_SIZE		32768
#define CONFIG_SYS_CACHELINE_SIZE	32

/**
 * DEBUG
 **/
#define CONFIG_SYS_UART_INDEX		1
#define CONFIG_BAUDRATE				115200

/**
 * CPU & DDR
 **/
#define CONFIG_SYS_CPU_FREQ		CONFIG_SYS_APLL_FREQ
#define CONFIG_SYS_MEM_FREQ		600000000
#define CONFIG_SYS_CPCCR_SEL	((2 << 30) | (1 << 28) | (2 << 26) | (2 << 24) \
						| ((12 - 1) << 16) | ((6 - 1) << 12) | ((6 - 1) << 8) \
						| ((2 - 1) << 4) | ((1 - 1) << 0))
#define CONFIG_CPU_SEL_PLL		APLL
#define CONFIG_DDR_SEL_PLL		MPLL


#define CONFIG_DDR_INNOPHY
#define CONFIG_X1XXX_INNOPHY
#define CONFIG_DDR_TYPE_DDR2
#define CONFIG_DDR_PARAMS_CREATOR
#define CONFIG_DDR_HOST_CC
#define CONFIG_DDR_CS0			1	/* 1-connected, 0-disconnected */
#define CONFIG_DDR_CS1			0	/* 1-connected, 0-disconnected */
#define CONFIG_DDR_DW32			0	/* 1-32bit-width, 0-16bit-width */
#define CONFIG_DDRC_CTRL_PDT DDRC_CTRL_PDT_128
#define CONFIG_DDR2_M14D1G1664A
#define DDR2_CHIP_DRIVER_OUT_STRENGTH 0
#define CONFIG_DDR_PHY_IMPEDANCE 40000
#define CONFIG_DDR_PHY_ODT_IMPEDANCE 50000

/**
 * Command configuration.
 */
#define CONFIG_CMD_BOOTD		/* bootd			*/
#define CONFIG_CMD_CONSOLE		/* coninfo			*/
#define CONFIG_CMD_ECHO			/* echo arguments		*/
#define CONFIG_CMD_EXT4 		/* ext4 support			*/
#define CONFIG_CMD_FAT			/* FAT support			*/
#define CONFIG_CMD_MEMORY		/* md mm nm mw cp cmp crc base loop mtest */
#define CONFIG_CMD_MISC			/* Misc functions like sleep etc*/
#define CONFIG_CMD_RUN			/* run command in env variable	*/
#define CONFIG_CMD_SAVEENV		/* saveenv			*/
#define CONFIG_CMD_SETGETDCR	/* DCR support on 4xx		*/
#define CONFIG_CMD_SOURCE		/* "source" command support	*/
#define CONFIG_CMD_GETTIME
#define CONFIG_CMD_UNZIP        /* unzip from memory to memory  */
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_NET			/* networking support*/
#define CONFIG_CMD_PING

#define CONFIG_JZSOC
/* NS16550-ish UARTs, uart[0134] are accessible */
#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_SYS_NS16550_CLK		CONFIG_SYS_EXTAL
#define CONFIG_SYS_NS16550_COM1		0xb0030000 /* uart0 */
#define CONFIG_SYS_NS16550_COM2		0xb0031000 /* uart1 */
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_CONSOLE_MUX
#define CONFIG_CONS_INDEX		2
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE \
	{ 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 115200, 230400 }


/*#define CONFIG_SPL_MMC_SUPPORT*/

/**
 * Boot arguments & command definitions.
 */
#define BOOTARGS_COMMON "console=ttyS1,115200n8 mem=128M@0x0 rootwait "
#define CONFIG_BOOTDELAY 1
#ifdef CONFIG_SPL_MMC_SUPPORT
	#define CONFIG_BOOTARGS BOOTARGS_COMMON "root=/dev/mmcblk0p2 rootfstype=ext4 rw"
	/*#define CONFIG_BOOTCOMMAND "mmc read 0x80600000 0x1800 0x3000; bootm 0x80600000"*/
	#define CONFIG_BOOTCOMMAND "mmc dev 0;ext4load mmc 0:2 0x80600000 /boot/uImage; bootm 0x80600000"
#elif defined(CONFIG_MTD_SFCNOR)
	#define CONFIG_BOOTARGS BOOTARGS_COMMON "ip=off init=/linuxrc root=/dev/mtdblock2 rootfstype=jffs2 rw"
	#define CONFIG_BOOTCOMMAND "sfcnor read 0x40000 0x500000 0x80800000 ;bootm 0x80800000"
#endif /* CONFIG_SFC_NOR */

/**
 * Drivers configuration.
 */
/* MMC */
#ifdef CONFIG_JZ_MMC_MSC0
	#define CONFIG_CMD_MMC			/* MMC/SD support*/
	#define CONFIG_GENERIC_MMC		1
	#define CONFIG_MMC				1
	#define CONFIG_JZ_MMC			1
	#define CONFIG_JZ_MMC_SPLMSC	0
	#define CONFIG_JZ_MMC_MSC0_PB	1
	#define CONFIG_MSC_DATA_WIDTH_4BIT
#endif  /* JZ_MMC_MSC0 */

/* SFC */
#ifdef CONFIG_MTD_SFCNOR
	#define CONFIG_CMD_SFC_NOR
	#define CONFIG_JZ_SFC
	#define CONFIG_JZ_SFC_NOR
	#define CONFIG_SFC_QUAD
	#define CONFIG_SFC_NOR_RATE					150000000
	#define CONFIG_SPIFLASH_PART_OFFSET         (CONFIG_SPL_MAX_SIZE)
	#define CONFIG_SPI_NORFLASH_PART_OFFSET     (CONFIG_SPIFLASH_PART_OFFSET + ((size_t)&((struct burner_params*)0)->norflash_partitions))
	#define CONFIG_NOR_MAJOR_VERSION_NUMBER     1
	#define CONFIG_NOR_MINOR_VERSION_NUMBER     0
	#define CONFIG_NOR_REVERSION_NUMBER         0
	#define CONFIG_NOR_VERSION     (CONFIG_NOR_MAJOR_VERSION_NUMBER | (CONFIG_NOR_MINOR_VERSION_NUMBER << 8) | (CONFIG_NOR_REVERSION_NUMBER <<16))
	#define CONFIG_ENV_IS_IN_SFC_NOR
#endif /*MTD_SFCNOR*/

/* GMAC */
#define GMAC_PHY_MII	1
#define GMAC_PHY_RMII	2
#define GMAC_PHY_GMII	3
#define GMAC_PHY_RGMII	4
#define CONFIG_NET_GMAC_PHY_MODE GMAC_PHY_RMII

#define PHY_TYPE_DM9161		1
#define PHY_TYPE_88E1111	2
#define PHY_TYPE_8710A		3
#define PHY_TYPE_IP101G		4

#define CONFIG_NET_PHY_TYPE   PHY_TYPE_IP101G

#define CONFIG_NET_X1830
#define CONFIG_SET_ETHADDR

#define CONFIG_GPIO_IP101G_RESET		GPIO_PB(28)
#define CONFIG_GPIO_IP101G_RESET_ENLEVEL	0

/* DEBUG ETHERNET */
#define CONFIG_SERVERIP     192.168.5.94
#define CONFIG_IPADDR       192.168.5.33
#define CONFIG_GATEWAYIP    192.168.5.1
#define CONFIG_NETMASK      255.255.255.0
#define CONFIG_ETHADDR      00:11:22:33:44:55

/* GPIO */
#define CONFIG_JZ_GPIO

/*pmu slp pin*/
#define CONFIG_REGULATOR
#ifdef  CONFIG_REGULATOR
	#define CONFIG_JZ_PMU_SLP_OUTPUT1
	#define CONFIG_INGENIC_SOFT_I2C
	#define CONFIG_PMU_EA3056
	#define CONFIG_EA3056_I2C_SCL	GPIO_PC(28)
	#define CONFIG_EA3056_I2C_SDA	GPIO_PC(27)
	#define CONFIG_SOFT_I2C_READ_REPEATED_START
#endif

/**
 * Miscellaneous configurable options
 */
#define CONFIG_EFI_PARTITION
#define CONFIG_EXT4_WRITE
#define CONFIG_PARTITION_UUIDS

#define CONFIG_SHA1
#define CONFIG_LZO
#define CONFIG_BZIP2
#define CONFIG_RBTREE
#define CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_FLASH_BASE	0 /* init flash_base as 0 */
#define CONFIG_SYS_BOOTM_LEN	(128 << 20)
#define CONFIG_ENV_OVERWRITE
#define CONFIG_MISC_INIT_R	1
#define CONFIG_BOOTP_MASK	(CONFIG_BOOTP_DEFAUL)
#define CONFIG_SYS_MAXARGS 	16
#define CONFIG_SYS_LONGHELP

#if defined(CONFIG_SPL_SFC_NOR)
	#define CONFIG_SPL_SFC_SUPPORT
#endif
#if defined(CONFIG_SPL_MMC_SUPPORT)
	#if defined(CONFIG_JZ_MMC_MSC0)
		#define CONFIG_SYS_PROMPT CONFIG_SYS_BOARD "-msc0# "
	#endif
#elif defined(CONFIG_SPL_SFC_SUPPORT)
	#if defined(CONFIG_SPL_SFC_NOR)
		#define CONFIG_SYS_PROMPT CONFIG_SYS_BOARD "-sfcnor# "
	#endif
#endif

#define CONFIG_SYS_CBSIZE 1024 /* Console I/O Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_SYS_MONITOR_LEN		((256 * 1024) - CONFIG_SPL_PAD_TO)
#define CONFIG_SYS_MALLOC_LEN		(32 * 1024 * 1024)
#define CONFIG_SYS_BOOTPARAMS_LEN	(128 * 1024)

#define CONFIG_SYS_SDRAM_BASE		0x80000000 /* cached (KSEG0) address */
#define CONFIG_SYS_SDRAM_MAX_TOP	0x90000000 /* don't run into IO space */
#define CONFIG_SYS_INIT_SP_OFFSET	0x400000
#define CONFIG_SYS_LOAD_ADDR		0x88000000
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		0x84000000

#define CONFIG_SYS_TEXT_BASE		0x80100000
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_TEXT_BASE

/**
 * Environment
 */
#if defined(CONFIG_ENV_IS_IN_MMC)
	#define CONFIG_CMD_SAVEENV
	#define CONFIG_SYS_MMC_ENV_DEV		0
	#define CONFIG_ENV_SIZE			(16 * 1024)
	#undef CONFIG_SYS_MONITOR_LEN
	#define CONFIG_SYS_MONITOR_LEN          ((512 * 1024) - (CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512) - CONFIG_ENV_SIZE)
	#define CONFIG_ENV_OFFSET		(CONFIG_SYS_MONITOR_LEN + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)
#elif defined(CONFIG_ENV_IS_IN_SFC_NOR)
	#define CONFIG_CMD_SAVEENV
	#define CONFIG_ENV_SIZE			(16 * 1024)
	#undef CONFIG_SYS_MONITOR_LEN
	#define CONFIG_SYS_MONITOR_LEN		((256 * 1024) - CONFIG_UBOOT_OFFSET - CONFIG_ENV_SIZE)
	#define CONFIG_ENV_OFFSET		(CONFIG_SYS_MONITOR_LEN + CONFIG_UBOOT_OFFSET)
#else
	#define CONFIG_ENV_IS_NOWHERE
	#define CONFIG_ENV_SIZE			(512)
#endif

/**
 * SPL configuration
 */
#define CONFIG_SPL
#define CONFIG_SPL_FRAMEWORK

#define CONFIG_SPL_NO_CPU_SUPPORT_CODE
#define CONFIG_SPL_I2C_SUPPORT
#define CONFIG_SPL_REGULATOR_SUPPORT
#define CONFIG_SPL_CORE_VOLTAGE			1275
#define CONFIG_SPL_CORE_VOLTAGE_RATIO	22/37
#ifdef CONFIG_SPL_NOR_SUPPORT
  #define CONFIG_SPL_LDSCRIPT           "$(CPUDIR)/$(SOC)/u-boot-nor-spl.lds"
#else
  #define CONFIG_SPL_LDSCRIPT		"$(CPUDIR)/x1830/u-boot-spl.lds"
#endif
#define CONFIG_SPL_GPIO_SUPPORT
#define CONFIG_SPL_BOARD_INIT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#ifdef CONFIG_SPL_NOR_SUPPORT
  #define CONFIG_SPL_TEXT_BASE		0xba000000
  #define CONFIG_SYS_OS_BASE		0
  #define CONFIG_SYS_SPL_ARGS_ADDR	0
  #define CONFIG_SYS_FDT_BASE		0
  #define CONFIG_SPL_PAD_TO		32768
  #define CONFIG_SPL_MAX_SIZE		CONFIG_SPL_PAD_TO
  #define CONFIG_SYS_UBOOT_BASE		(CONFIG_SPL_TEXT_BASE + CONFIG_SPL_PAD_TO - 0x40)  /* 0x40 = sizeof (image_header) */
#elif defined(CONFIG_SPL_MMC_SUPPORT)
  #define CONFIG_SPL_TEXT_BASE		0x80001000
  #define CONFIG_SPL_MAX_SIZE		26624
  #define CONFIG_SPL_PAD_TO	        CONFIG_SPL_MAX_SIZE
  #define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	 ((CONFIG_SPL_PAD_TO + (17 * 1024)) >> 9) /* 28KB + 17K offset */
#elif defined(CONFIG_SPL_SFC_SUPPORT)
  #define CONFIG_SPI_SPL_CHECK
  #define CONFIG_SPL_TEXT_BASE		0x80001000
  #define CONFIG_SPL_PAD_TO		27648
  #define CONFIG_SPL_MAX_SIZE		(26 * 1024)
  #define CONFIG_UBOOT_OFFSET           CONFIG_SPL_PAD_TO
  #define CONFIG_SPL_VERSION            1
#endif	/*CONFIG_SPL_NOR_SUPPORT*/

/**
 * MBR & GPT configuration
 */
#ifdef CONFIG_MBR_CREATOR
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
#else
#define CONFIG_GPT_TABLE_PATH	"board/$(BOARDDIR)"
#endif

#endif /*__CONFIG_CU1830_NEO_H__*/
