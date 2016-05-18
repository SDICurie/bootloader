cflags-$(CONFIG_USB) 	+= -Wno-error=format
cflags-$(CONFIG_USB) 	+= -Wno-error -w

cflags-$(CONFIG_USB_DEBUG) 	+= -DUSB_DEBUG
cflags-$(CONFIG_USB_DEBUG_EP0) 	+= -DDEBUG_EP0

cflags-$(CONFIG_USB_DEVICE)	+= -DDWC_DEVICE_ONLY

cflags-$(CONFIG_USB)	+= -I$(BOOTLOADER_ROOT)/drivers/usb/dwc_3.10b/dwc_common_port/
cflags-$(CONFIG_USB)	+= -I$(BOOTLOADER_ROOT)/include/usb/

obj-$(CONFIG_USB)	+= dwc_otg_cil.o dwc_otg_cil_intr.o
obj-$(CONFIG_USB)	+= dwc_otg_pcd.o dwc_otg_pcd_intr.o
obj-$(CONFIG_USB)	+= dwc_usb_device_driver.o
obj-$(CONFIG_USB)	+= dwc_otg_adp_stub.o
