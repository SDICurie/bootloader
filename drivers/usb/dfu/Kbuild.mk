cflags-$(CONFIG_USB_DEBUG) += -DDEBUG
cflags-$(CONFIG_USB_DEBUG_EP0) += -DDEBUG_EP0

obj-y	+= dfu.o
