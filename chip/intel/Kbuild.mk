obj-y += $(subst ",,$(CONFIG_CHIP_NAME))/
obj-$(CONFIG_GPIO) += gpio.o
obj-$(CONFIG_I2C) += i2c.o
obj-$(CONFIG_SPI) += spi.o
obj-$(CONFIG_UART) += uart.o
