obj-y += main.o
obj-y += version.o
obj-y += bootlogic.o
obj-$(CONFIG_CRC16_CCITT) += crc16_ccitt.o
obj-$(CONFIG_CRC32) += crc32.o
obj-y += utils.o

obj-$(CONFIG_FACTORY_BOOT) +=  factory.o
obj-$(CONFIG_BALLOC) += balloc.o
obj-$(CONFIG_PRINTK) += printk.o
obj-$(CONFIG_CRYPTO) += crypto.o
obj-$(CONFIG_PANIC_DUMP) += panic_dump.o
obj-$(CONFIG_OTA) += ota.o
obj-$(CONFIG_SECURE_BOOT) += sign.o
obj-$(CONFIG_PROFILING) += profiling.o
cflags-$(CONFIG_PROFILING) += -finstrument-functions -finstrument-functions-exclude-file-list=balloc.c
