/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <usb/usb.h>
#include <dfu.h>

usb_device_descriptor_t dfu_device_desc = {
	.bLength = USB_DEVICE_DESCRIPTOR_SIZE,
	.bDescriptorType = 0x01,
	.bcdUSB[0] = UD_USB_2_0 & 0xff,
	.bcdUSB[1] = (UD_USB_2_0 >> 8) & 0xff,
	.bDeviceClass = 0x00,
	.bDeviceSubClass = 0x00,
	.bDeviceProtocol = 0x00,
	.bMaxPacketSize = 64,
	.idVendor[0] = (CONFIG_USB_DFU_VID & 0xff),
	.idVendor[1] = ((CONFIG_USB_DFU_VID >> 8) & 0xff),
	.idProduct[0] = (CONFIG_USB_DFU_PID & 0xff),
	.idProduct[1] = ((CONFIG_USB_DFU_PID >> 8) & 0xff),
	.bcdDevice[0] = 0x87,
	.bcdDevice[1] = 0x80,
	/* STRING_MANUFACTURER */
	.iManufacturer = 1,
	/* STRING_PRODUCT */
	.iProduct = 2,
	/* STRING_SERIAL */
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

struct dfu_usb_descriptor dfu_config_desc = {
	.config_descriptor = {
		.bLength = USB_CONFIG_DESCRIPTOR_SIZE,
		.bDescriptorType = 0x2,
		.wTotalLength[0] = DFU_CONF_SIZE & 0xff,
		.wTotalLength[1] = (DFU_CONF_SIZE >> 8) & 0xff,
		.bNumInterface = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = UC_BUS_POWERED | UC_SELF_POWERED,
		.bMaxPower = 50,        /* max current in 2mA units */
	},

#define ALTERNATE(alt, desc) \
	{ \
		.bLength = USB_INTERFACE_DESCRIPTOR_SIZE, \
		.bDescriptorType = 0x4,	\
		.bInterfaceNumber = 0, \
		.bAlternateSetting = alt ## _ALT, \
		.bNumEndpoints = 0, \
		.bInterfaceClass = 0xfe, \
		.bInterfaceSubClass = 1, \
		.bInterfaceProtocol = 2, \
		.iInterface = alt ## _STRING_ID, \
	},
	.interface_descriptor = {
		LIST_OF_ALTERNATES
	},
#undef ALTERNATE
	.dfu_functional_descriptor = {
		.bLength = DFU_FUNCTIONAL_DESCRIPTOR_SIZE,
		.bDescriptorType = 0x21,
		/* SHOULD BE DFU_DOWNLOAD_CAPABLE */
		.bmAttributes =
			DFU_UPLOAD_CAPABLE | DFU_DOWNLOAD_CAPABLE |
			DFU_MANIFESTATION_TOLERANT,
		.wDetachTimeout[0] = 0,
		.wDetachTimeout[1] = 0,
		.wTransferSize[0] = 0,
		.wTransferSize[1] = 8,
		.bcdDFUVersion[0] = 0x10,
		.bcdDFUVersion[1] = 0x01,
	}
};

struct run_time_dfu_usb_descriptor run_time_dfu_config_desc = {
	.config_descriptor = {
		.bLength = USB_CONFIG_DESCRIPTOR_SIZE,
		.bDescriptorType = 0x2,
		.wTotalLength[0] = RUN_TIME_DFU_CONF_SIZE & 0xff,
		.wTotalLength[1] = (RUN_TIME_DFU_CONF_SIZE >> 8) & 0xff,
		.bNumInterface = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = UC_BUS_POWERED | UC_SELF_POWERED,
		.bMaxPower = 50,        /* max current in 2mA units */
	},
	.interface_descriptor = {
		.bLength = USB_INTERFACE_DESCRIPTOR_SIZE,
		.bDescriptorType = 0x4,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 0,
		.bInterfaceClass = 0xfe,
		.bInterfaceSubClass = 1,
		.bInterfaceProtocol = 1,
		.iInterface = 1,
	},
	.dfu_functional_descriptor = {
		.bLength = DFU_FUNCTIONAL_DESCRIPTOR_SIZE,
		.bDescriptorType = 0x21,
		/* SHOULD BE DFU_DOWNLOAD_CAPABLE */
		.bmAttributes =
			DFU_UPLOAD_CAPABLE | DFU_DOWNLOAD_CAPABLE |
			DFU_MANIFESTATION_TOLERANT,
		.wDetachTimeout[0] = 0,
		.wDetachTimeout[1] = 4,
		.wTransferSize[0] = 0,
		.wTransferSize[1] = 8, /* 2048 bytes */
		.bcdDFUVersion[0] = 0x10,
		.bcdDFUVersion[1] = 0x01,
	}
};

#define ALTERNATE(alt, desc) \
	{ \
		.bLength = sizeof(# desc) * 2, \
		.bDescriptorType = UDESC_STRING, \
		.bString = desc ## _ ## u16 \
	},
usb_string_descriptor_t dfu_strings_desc[] = {
	{
		/*String descriptor language, only one, so min size 4 bytes */
		/* 0x0409 English(US) language code used */
		.bLength = 4,
		.bDescriptorType = UDESC_STRING,
		.bString = { 0x09, 0x04 }
	},
	{
		.bLength = sizeof("Intel") * 2,
		.bDescriptorType = UDESC_STRING,
		.bString = { 'I', 0, 'n', 0, 't', 0, 'e', 0, 'l', 0 }
	},
	{
		.bLength = sizeof("CURIECRB") * 2,
		.bDescriptorType = UDESC_STRING,
		.bString = { 'C', 0, 'U', 0, 'R', 0, 'I', 0, 'E', 0,
			     'C', 0, 'R', 0, 'B', 0 }
	},
	{
		.bLength = sizeof("00.01") * 2,
		.bDescriptorType = UDESC_STRING,
		.bString = { '0', 0, '0', 0, '.', 0, '0', 0, '1', 0 }
	},
	LIST_OF_ALTERNATES
};
#undef ALTERNATE
