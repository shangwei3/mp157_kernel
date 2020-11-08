##############################################################################
#
#    Copyright (C) 2005 - 2017 by Vivante Corp.
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the license, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not write to the Free Software
#    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
###############################################################################

AQROOT?=$(CURDIR)
AQARCH=$(AQROOT)/arch/XAQ2

include $(AQROOT)/config


export AQROOT AQARCH


#ARCH_TYPE         ?= arm
#KERNEL_DIR        ?= /home/alientek2004/stm32mp157/linux-5.4.31
#CROSS_COMPILE     ?= /home/software/Linux/arm-2007q1/bin/arm-none-linux-gnueabi-
SDK_DIR           ?= $(AQROOT)/build/sdk

export KERNEL_DIR CROSS_COMPILE SDK_DIR

.PHONY: all clean install

all:
	@make -f Kbuild

clean:
	@make -f Kbuild ARCH=$(ARCH_TYPE)  clean
	cd hal/os/linux/kernel/ ; rm -fr */*/*.mod.c */*/*.mod */*/*.o */*/.*.cmd */*/*.ko
	cd hal/os/linux/kernel/allocator/default/ ; rm -fr */*/*.mod.c */*/*.mod */*/*.o */*/.*.cmd */*/*.ko
	rm -fr Module.symvers ; rm -fr Module.markers ; rm -fr modules.order
	rm -fr *.mod.c *.mod *.o .*.cmd *.ko *~
	rm -fr .tmp_versions


install:
	@make -f Kbuild install

