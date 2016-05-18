subdir-cflags-$(CONFIG_CRYPTO_ECDSA) += -DuECC_SQUARE_FUNC=0 -DuECC_CURVE=uECC_secp256r1
subdir-cflags-$(CONFIG_CRYPTO_ECDSA) += -I$(T)/external/micro-ecc/
subdir-cflags-$(CONFIG_COMP_LZG) += -I$(T)/external/liblzg/src/include/
subdir-cflags-$(CONFIG_COMP_LZG) += -I$(T)/external/liblzg/src/lib
subdir-cflags-$(CONFIG_CRYPTO_SHA256) += -I$(T)/external/sha256/

ifeq (${OPTION_PUBLIC_BOARD},n)
obj-$(USE_BOARD_DIR) += $(OPTION_PRIVATE_DIR)/bootloader/board/
else
obj-$(USE_BOARD_DIR) += board/
endif

ifeq (${OPTION_PUBLIC_CHIP},n)
obj-$(USE_CHIP_DIR) += $(OPTION_PRIVATE_DIR)/bootloader/chip/
else
obj-$(USE_CHIP_DIR) += chip/
endif

obj-y += common/

ifeq (${OPTION_PUBLIC_CORE},n)
obj-y += $(OPTION_PRIVATE_DIR)/bootloader/core/
else
obj-y += core/
endif

obj-y += drivers/

obj-$(CONFIG_COMP_LZG) += $(T)/external/liblzg/src/lib/

ifeq (${CONFIG_SHARE_CRYPTO},y)
obj-$(CONFIG_CRYPTO_ECDSA) += $(T)/external/micro-ecc/
endif

ifeq (${CONFIG_SHARE_CRYPTO},y)
obj-$(CONFIG_CRYPTO_SHA256) += $(T)/external/sha256/
endif
