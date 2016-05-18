CFLAGS_dfu_desc.o +=  -Wno-missing-braces

obj-y +=  boot_x86.o
obj-y +=  quark_se.o
obj-$(CONFIG_USB_DEVICE) +=  usb_setup.o
obj-y +=  soc_flash.o
obj-y += soc_rom.o
obj-$(CONFIG_PANIC_DUMP) += panic_boot.o
obj-$(CONFIG_HARDWARE_CHARGING) += cos.o hardware_charging.o
obj-$(CONFIG_OTA) += boot_ota.o ota_partition.o lzgdecode.o
obj-$(CONFIG_USB_DFU) += dfu_desc.o dfu_spi_flash.o
