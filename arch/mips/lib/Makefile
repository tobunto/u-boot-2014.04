#
# (C) Copyright 2003-2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# SPDX-License-Identifier:	GPL-2.0+
#

ifndef CONFIG_SPL_BUILD
obj-y	+= board.o
obj-$(CONFIG_CMD_BOOTM) += bootm.o
obj-$(CONFIG_USE_ARCH_MEMSET) += memset.o
obj-$(CONFIG_USE_ARCH_MEMCPY) += memcpy.o
else
obj-$(CONFIG_SPL_FRAMEWORK) += spl.o
endif

lib-$(CONFIG_USE_PRIVATE_LIBGCC) += ashldi3.o ashrdi3.o lshrdi3.o
