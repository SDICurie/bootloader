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

#ifndef _DFU_H_
#define _DFU_H_

#include "usb/usb.h"
#include "stdint.h"
#include <stdbool.h>

#include "dfu_alternates.h"

/***** STATUS CODE ****/
#define statusOK            0x00
#define errTARGET           0x01
#define errFILE             0x02
#define errWRITE            0x03
#define errERASE            0x04
#define errCHECK_ERASED     0x05
#define errPROG             0x06
#define errVERIFY           0x07
#define errADDRESS          0x08
#define errNOTDONE          0x09
#define errFIRMWARE         0x0a
#define errVENDOR           0x0b
#define errUSB              0x0c
#define errPOR              0x0d
#define errUNKNOWN          0x0e
#define errSTALLEDPKT       0x0f

/**** STATES ****/
#define appIDLE             0x00
#define appDETACH           0x01
#define dfuIDLE             0x02
#define dfuDNLOAD_SYNC      0x03
#define dfuDNBUSY           0x04
#define dfuDNLOAD_IDLE      0x05
#define dfuMANIFEST_SYNC    0x06
#define dfuMANIFEST         0x07
#define dfuMANIFEST_WAIT_RST    0x08
#define dfuUPLOAD_IDLE      0x09
#define dfuERROR            0x0a

#define USB_REQ_RECIPIENT_INTERFACE 0x01

/* DFU specific functional descriptor */
typedef struct usb_dfu_functional_descriptor {
	uByte bLength;
	uByte bDescriptorType;
	uByte bmAttributes;
#define DFU_WILL_DETACH (1 << 3)        /* will detach */
#define DFU_MANIFESTATION_TOLERANT (1 << 2)
#define DFU_UPLOAD_CAPABLE (1 << 1)         /* upload capable */
#define DFU_DOWNLOAD_CAPABLE (1 << 0)   /* download capable */
	uWord wDetachTimeout;
	uWord wTransferSize;
	uWord bcdDFUVersion;
} UPACKED usb_dfu_functional_descriptor_t;
#define DFU_FUNCTIONAL_DESCRIPTOR_SIZE sizeof(usb_dfu_functional_descriptor_t) /* 0x09 expected */


#define NBR_STRING_DESC (ALTERNATE_COUNT + 4)

struct run_time_dfu_usb_descriptor {
	usb_config_descriptor_t config_descriptor;
	usb_interface_descriptor_t interface_descriptor;
	usb_dfu_functional_descriptor_t dfu_functional_descriptor;
} UPACKED;

struct dfu_usb_descriptor {
	usb_config_descriptor_t config_descriptor;
	usb_interface_descriptor_t interface_descriptor[ALTERNATE_COUNT];
	usb_dfu_functional_descriptor_t dfu_functional_descriptor;
} UPACKED;

#define RUN_TIME_DFU_CONF_SIZE (USB_CONFIG_DESCRIPTOR_SIZE \
				+ (USB_INTERFACE_DESCRIPTOR_SIZE) \
				+ DFU_FUNCTIONAL_DESCRIPTOR_SIZE)

#define DFU_CONF_SIZE (USB_CONFIG_DESCRIPTOR_SIZE \
		       + (USB_INTERFACE_DESCRIPTOR_SIZE * ALTERNATE_COUNT) \
		       + DFU_FUNCTIONAL_DESCRIPTOR_SIZE)

extern usb_device_descriptor_t dfu_device_desc;
extern struct run_time_dfu_usb_descriptor run_time_dfu_config_desc;
extern struct dfu_usb_descriptor dfu_config_desc;

extern usb_string_descriptor_t dfu_strings_desc[NBR_STRING_DESC];
extern const uint32_t dfu_strings_desc_len;

void dfu_class_init();

struct dfu_ops {
	int alternate;
	int state;
	int status;
	uint32_t len;
	void *data;
	usb_device_request_t *device_request;

	void (*erase)(struct dfu_ops *ops);
	void (*write)(struct dfu_ops *ops);
	uint32_t (*read)(struct dfu_ops *ops);
	uint32_t block_count;
	uint32_t partition_offset;
};

/* start usb enumeration for dfu mode */
void switch_to_dfu_mode(void);
/* start usb enumeration for run time mode */
void switch_to_run_time_mode(void);
/* check if dfu mode or run time mode */
bool dfu_busy(void);

#endif /* _DFU_H_ */
