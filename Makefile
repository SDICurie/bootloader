# Copyright (c) 2015, Intel Corporation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#### config rules

ifdef T
BOOTLOADER_ROOT ?= $(T)/bsp/bootable/bootloader
else
BOOTLOADER_ROOT ?= ./
endif

fsbl_menuconfig: fsbl_defconfig
	$(AT)mkdir -p $(OUT)/$*
	$(AT)+$(MAKE) -f $(BOOTLOADER_ROOT)/build/config.mk menuconfig \
		T=$(T) \
		OUT=$(OUT)/$*/ \
		DEFCONFIG=$(FSBL_DEFCONFIG) \
		KCONFIG_ROOT=$(T)/bsp/bootable/bootloader/Kconfig

ssbl_menuconfig: ssbl_defconfig
	$(AT)mkdir -p $(OUT)/$*
	$(AT)+$(MAKE) -f $(BOOTLOADER_ROOT)/build/config.mk menuconfig \
		T=$(T) \
		OUT=$(OUT)/$*/ \
		DEFCONFIG=$(SSBL_DEFCONFIG) \
		KCONFIG_ROOT=$(T)/bsp/bootable/bootloader/Kconfig

fsbl_defconfig:
	$(AT)mkdir -p $(OUT)/$*
	$(AT)+$(MAKE) -f $(BOOTLOADER_ROOT)/build/config.mk defconfig \
		T=$(T) \
		OUT=$(OUT)/$*/ \
		KCONFIG_ROOT=$(BOOTLOADER_ROOT)/Kconfig \
		DEFCONFIG=$(FSBL_DEFCONFIG)

ssbl_defconfig:
	$(AT)mkdir -p $(OUT)/$*
	$(AT)+$(MAKE) -f $(BOOTLOADER_ROOT)/build/config.mk defconfig \
		T=$(T) \
		OUT=$(OUT)/$*/ \
		KCONFIG_ROOT=$(BOOTLOADER_ROOT)/Kconfig \
		DEFCONFIG=$(SSBL_DEFCONFIG)

%_savedefconfig:
	$(AT)mkdir -p $(OUT)/$*
	$(AT)+$(MAKE) -f $(BOOTLOADER_ROOT)/build/config.mk savedefconfig \
		T=$(T) \
		OUT=$(OUT)/$*/ \
		KCONFIG_ROOT=$(BOOTLOADER_ROOT)/Kconfig \
		DEFCONFIG=$(DEFCONFIG)

$(OUT)/ssbl/.config: $(SSBL_DEFCONFIG)
	$(AT)mkdir -p $(OUT)/ssbl
	$(AT)+$(MAKE) -f $(BOOTLOADER_ROOT)/build/config.mk defconfig \
		T=$(T) \
		OUT=$(OUT)/ssbl \
		KCONFIG_ROOT=$(BOOTLOADER_ROOT)/Kconfig \
		DEFCONFIG=$(SSBL_DEFCONFIG)

ssbl: $(OUT)/ssbl/.config
	$(AT)mkdir -p $(OUT)/$@
	$(AT)+$(MAKE) all -f $(BOOTLOADER_ROOT)/build/Makefile.ssbl \
		PROJECT_INCLUDES=$(PROJECT_INCLUDES) \
		BOOTLOADER_ROOT=$(BOOTLOADER_ROOT) \
		T=$(T) \
		BOARD=$(BOARD) \
		USE_BOARD_DIR=y \
		USE_CHIP_DIR=y \
		BUILDVARIANT=$(BUILDVARIANT) \
		DEFCONFIG=$(SSBL_DEFCONFIG) \
		OUT=$(OUT)/ssbl \
		COS_BLOB=$(COS_BLOB)

$(OUT)/fsbl/.config: $(FSBL_DEFCONFIG)
	$(AT)mkdir -p $(OUT)/fsbl
	$(AT)+$(MAKE) -f $(BOOTLOADER_ROOT)/build/config.mk defconfig \
		T=$(T) \
		OUT=$(OUT)/fsbl \
		KCONFIG_ROOT=$(BOOTLOADER_ROOT)/Kconfig \
		DEFCONFIG=$(FSBL_DEFCONFIG)

fsbl: $(OUT)/fsbl/.config
	$(AT)mkdir -p $(OUT)/$@
	$(AT)+$(MAKE) all -f $(BOOTLOADER_ROOT)/build/Makefile.fsbl \
		PROJECT_INCLUDES=$(PROJECT_INCLUDES) \
		BOOTLOADER_ROOT=$(BOOTLOADER_ROOT) \
		T=$(T) \
		BOARD=$(BOARD) \
		USE_BOARD_DIR=n \
		USE_CHIP_DIR=y \
		BUILDVARIANT=$(BUILDVARIANT) \
		DEFCONFIG=$(FSBL_DEFCONFIG) \
		OUT=$(OUT)/$@

BOARD_DEFCONFIGS := $(wildcard $(BOOTLOADER_ROOT)/board/*/*_defconfig $(OPTION_PRIVATE_DIR)/bootloader/board/*/*_defconfig)
CHIP_DEFCONFIGS := $(wildcard $(BOOTLOADER_ROOT)/chip/*/*/*_defconfig)

.PHONY: sanity
sanity:
	$(AT)$(T)/external/kconfig/utils/kconfig_lint -G -w --path=$(BOOTLOADER_ROOT)

.PHONY: all
all: sanity fsbl ssbl
	$(AT)echo Building bootloader

clean:
	$(AT)rm -rf $(OUT)/fsbl
	$(AT)rm -rf $(OUT)/ssbl
