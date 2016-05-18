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

#include <dfu.h>
#include <usb/usb_driver_interface.h>
#include <printk.h>
#include <bootlogic.h>
#include <reboot.h>
#include <utils.h>

#ifndef NULL
#define NULL 0
#endif

/*Class specific request*/
#define DFU_DETACH          0x00
#define DFU_DNLOAD          0x01
#define DFU_UPLOAD          0x02
#define DFU_GETSTATUS       0x03
#define DFU_CLRSTATUS       0x04
#define DFU_GETSTATE        0x05
#define DFU_ABORT           0x06

#define DFU_USB_RESET       0x10
#define DFU_TIMEOUT         0x11

#define DFU_ENTER_STATE     0xFF

typedef void (*dfu_state_func)(uint32_t event, uint32_t *len);

static struct dfu_ops dfu_ops = {
	.alternate = -1,
	.state = appIDLE,
	.status = statusOK,
	.data = NULL,
	.len = 0,
	.device_request = NULL,
	.erase = NULL,
	.write = NULL,
	.read = NULL
};

struct usb_interface_init_data dfu_init_data;

static void state_app_idle(uint32_t event, uint32_t *len);
static void state_app_detach(uint32_t event, uint32_t *len);
static void state_dfu_idle(uint32_t event, uint32_t *len);
static void state_dfu_dnload_sync(uint32_t event, uint32_t *len);
static void state_dfu_dnload_busy(uint32_t event, uint32_t *len);
static void state_dfu_dnload_idle(uint32_t event, uint32_t *len);
static void state_dfu_manifest_sync(uint32_t event, uint32_t *len);
static void state_dfu_manifest(uint32_t event, uint32_t *len);
static void state_dfu_manifest_wait_reset(uint32_t event, uint32_t *len);
static void state_dfu_upload_idle(uint32_t event, uint32_t *len);
static void state_dfu_error(uint32_t event, uint32_t *len);

static dfu_state_func dfu_state[] = {
	state_app_idle,                                 /* appIDLE */
	state_app_detach,                               /* appDETACH */
	state_dfu_idle,                                 /* dfuIDLE */
	state_dfu_dnload_sync,                  /* dfuDNLOAD_SYNC */
	state_dfu_dnload_busy,                  /* dfuDNBUSY */
	state_dfu_dnload_idle,                  /* dfuDNLOAD_IDLE */
	state_dfu_manifest_sync,                /* dfuMANIFEST_SYNC */
	state_dfu_manifest,                             /* dfuMANIFEST */
	state_dfu_manifest_wait_reset,  /* dfuMANIFEST_WAIT_RST */
	state_dfu_upload_idle,                  /* dfuUPLOAD_IDLE */
	state_dfu_error                                 /* dfuERROR */
};

static char __maybe_unused(*req_string[]) =
{
	"DFU_DETACH",
	"DFU_DNLOAD",
	"DFU_UPLOAD",
	"DFU_GETSTATUS",
	"DFU_CLRSTATUS",
	"DFU_GETSTATE",
	"DFU_ABORT"
};

static char __maybe_unused(*state_string[]) =
{
	"appIDLE",
	"appDETACH",
	"dfuIDLE",
	"dfuDNLOAD_SYNC",
	"dfuDNBUSY",
	"dfuDNLOAD_IDLE",
	"dfuMANIFEST_SYNC",
	"dfuMANIFEST",
	"dfuMANIFEST_WAIT_RST",
	"dfuUPLOAD_IDLE",
	"dfuERROR"
};

static bool busy = false;

bool dfu_busy(void)
{
	return busy;
}

void switch_to_dfu_mode(void)
{
	dfu_init_data.conf_desc = (usb_config_descriptor_t *)&dfu_config_desc;
	dfu_init_data.conf_desc_size = sizeof(dfu_config_desc);

	dfu_class_init();
	dfu_ops.state = dfuIDLE;
}

void switch_to_run_time_mode(void)
{
	dfu_init_data.conf_desc =
		(usb_config_descriptor_t *)&run_time_dfu_config_desc;
	dfu_init_data.conf_desc_size = sizeof(run_time_dfu_config_desc);

	dfu_class_init();
	dfu_ops.state = appIDLE;
}

static void dfu_error(uint32_t *len)
{
	*len = 0;

	if (dfu_ops.status == statusOK)
		dfu_ops.status = errUNKNOWN;

	dfu_ops.state = dfuERROR;
}

__weak void dfu_set_alternate(struct dfu_ops __unused(*ops))
{
}

void do_dfu_download(uint32_t *len)
{
	pr_debug("%s %d\n", __func__, *len);

	usb_device_request_t *setup_packet = dfu_ops.device_request;
	unsigned int address = UGETW(setup_packet->wValue);

	dfu_ops.len = *len;
	dfu_set_alternate(&dfu_ops);
	if (address > dfu_ops.block_count - 1) {
		dfu_ops.status = errADDRESS;
	} else if (dfu_ops.write) {
		dfu_ops.write(&dfu_ops);
	} else {
		dfu_ops.status = errTARGET;
	}
	*len = 0;
}

void do_dfu_upload(uint32_t *len)
{
	pr_debug("%s %d\n", __func__, *len);

	usb_device_request_t *setup_packet = dfu_ops.device_request;
	unsigned int address = UGETW(setup_packet->wValue);

	dfu_ops.len = *len;
	dfu_set_alternate(&dfu_ops);
	if (address >= dfu_ops.block_count) {
		*len = 0;
	} else if (dfu_ops.read) {
		*len = dfu_ops.read(&dfu_ops);
	} else {
		*len = 0;
		dfu_ops.status = errTARGET;
	}
}

void do_dfu_usb_reset(uint32_t *len)
{
	pr_debug("%s\n", __func__);

	*len = 0;

	if (dfu_ops.status == errFIRMWARE) {
		dfu_ops.state = dfuERROR;
	} else {
		dfu_ops.state = appIDLE;
		reboot(TARGET_MAIN);
	}
}

static void do_getstatus(uint32_t *len)
{
	pr_debug("%s %x\n", __func__, dfu_ops.status);

	unsigned char *dfu_buffer = dfu_ops.data;
	dfu_buffer[0] = dfu_ops.status;
	dfu_buffer[1] = 1; /* 1ms wait before sending subsequent
	                    * dfu_getstatus request */
	dfu_buffer[2] = 0;
	dfu_buffer[3] = 0;
	dfu_buffer[4] = dfu_ops.state;
	dfu_buffer[5] = 0; /* no string description */
	*len = 6;
}

static void do_getstate(uint32_t *len)
{
	pr_debug("%s\n", __func__);

	unsigned char *dfu_buffer = dfu_ops.data;

	dfu_buffer[0] = dfu_ops.state;
	*len = 1;
}

static void do_download_start(uint32_t *len)
{
	pr_debug("%s\n", __func__);

	*len = UGETW(dfu_ops.device_request->wLength);

	if ((dfu_config_desc.dfu_functional_descriptor.bmAttributes
	     & DFU_DOWNLOAD_CAPABLE) == 0) {
		dfu_ops.status = errTARGET;
	} else if (*len > 0) {
		state_dfu_dnload_busy(DFU_ENTER_STATE, len);
		dfu_ops.state = dfuDNLOAD_SYNC;
	} else {
		dfu_ops.status = errNOTDONE;
	}

	if (dfu_ops.status != statusOK)
		dfu_error(len);
}

static void do_download_continue(uint32_t *len)
{
	pr_debug("%s\n", __func__);

	*len = UGETW(dfu_ops.device_request->wLength);

	if (*len > 0) {
		state_dfu_dnload_busy(DFU_ENTER_STATE, len);
		dfu_ops.state = dfuDNLOAD_SYNC;
	} else {
		dfu_ops.state = dfuMANIFEST_SYNC;
		state_dfu_manifest_sync(DFU_ENTER_STATE, len);
	}

	if (dfu_ops.status != statusOK)
		dfu_error(len);
}

static void do_upload_start(uint32_t *len)
{
	pr_debug("%s\n", __func__);

	uint32_t wLength = UGETW(dfu_ops.device_request->wLength);
	uint32_t wTransferSize =
		UGETW(dfu_config_desc.dfu_functional_descriptor.wTransferSize);

	if ((dfu_config_desc.dfu_functional_descriptor.bmAttributes
	     & DFU_UPLOAD_CAPABLE)
	    && wLength <= wTransferSize) {
		do_dfu_upload(len);

		if (*len < wLength) /* short frame */
			dfu_ops.state = dfuIDLE;
		else
			dfu_ops.state = dfuUPLOAD_IDLE;
	} else {
		dfu_ops.status = errSTALLEDPKT;
	}

	if (dfu_ops.status != statusOK)
		dfu_error(len);
}

static void do_upload_continue(uint32_t *len)
{
	pr_debug("%s\n", __func__);

	uint32_t wLength = UGETW(dfu_ops.device_request->wLength);
	uint32_t wTransferSize =
		UGETW(dfu_config_desc.dfu_functional_descriptor.wTransferSize);

	if (wLength <= wTransferSize) {
		do_dfu_upload(len);

		if (*len < wLength) /* short frame */
			dfu_ops.state = dfuIDLE;
	} else {
		dfu_ops.status = errSTALLEDPKT;
	}

	if (dfu_ops.status != statusOK)
		dfu_error(len);
}

static void state_app_idle(uint32_t event, uint32_t *len)
{
	pr_debug("%s\n", __func__);

	switch (event) {
	case DFU_DETACH:
		dfu_ops.state = appDETACH;
		break;

	case DFU_GETSTATUS:
		do_getstatus(len);
		break;

	case DFU_GETSTATE:
		do_getstate(len);
		break;

	default:
		break;
	}
}

static void state_app_detach(uint32_t event, uint32_t *len)
{
	pr_debug("%s\n", __func__);

	switch (event) {
	case DFU_GETSTATUS:
		do_getstatus(len);
		break;

	case DFU_GETSTATE:
		do_getstate(len);
		break;

	case DFU_DETACH:
	case DFU_USB_RESET:
		switch_to_dfu_mode();
		break;

	case DFU_TIMEOUT:
	default:
		dfu_ops.state = appIDLE;
		break;
	}
}

static void state_dfu_idle(uint32_t event, uint32_t *len)
{
	pr_debug("%s\n", __func__);
	busy = true;

	switch (event) {
	case DFU_DNLOAD:
		do_download_start(len);
		break;

	case DFU_UPLOAD:
		do_upload_start(len);
		break;

	case DFU_ABORT:
		*len = 0;
		break;

	case DFU_GETSTATUS:
		do_getstatus(len);
		break;

	case DFU_GETSTATE:
		do_getstate(len);
		break;

	case DFU_DETACH:
	case DFU_USB_RESET:
		do_dfu_usb_reset(len);
		break;

	default:
		dfu_ops.status = errSTALLEDPKT;
		dfu_error(len);
		break;
	}
}

static void state_dfu_dnload_sync(uint32_t event, uint32_t *len)
{
	pr_debug("%s\n", __func__);

	switch (event) {
	case DFU_GETSTATUS:
		do_getstatus(len);
		dfu_ops.state = dfuDNLOAD_IDLE;
		break;

	case DFU_GETSTATE:
		do_getstate(len);
		break;

	case DFU_DETACH:
	case DFU_USB_RESET:
		do_dfu_usb_reset(len);
		break;

	default:
		dfu_ops.status = errSTALLEDPKT;
		dfu_error(len);
		break;
	}
}

static void state_dfu_dnload_busy(uint32_t event, uint32_t *len)
{
	switch (event) {
	case DFU_ENTER_STATE:
		do_dfu_download(len);
		break;

	case DFU_DETACH:
	case DFU_USB_RESET:
		do_dfu_usb_reset(len);
		break;

	case DFU_TIMEOUT:
	default:
		dfu_ops.status = errSTALLEDPKT;
		dfu_error(len);
		break;
	}
}

static void state_dfu_dnload_idle(uint32_t event, uint32_t *len)
{
	pr_debug("%s\n", __func__);

	switch (event) {
	case DFU_DNLOAD:
		do_download_continue(len);
		break;

	case DFU_ABORT:
		*len = 0;
		dfu_ops.state = dfuIDLE;
		break;

	case DFU_GETSTATUS:
		do_getstatus(len);
		break;

	case DFU_GETSTATE:
		do_getstate(len);
		break;

	case DFU_DETACH:
	case DFU_USB_RESET:
		do_dfu_usb_reset(len);
		break;

	default:
		dfu_ops.status = errSTALLEDPKT;
		dfu_error(len);
		break;
	}
}

static void state_dfu_manifest_sync(uint32_t event, uint32_t *len)
{
	pr_debug("%s\n", __func__);

	static enum {
		MANIFEST_NOT_STARTED,
		MANIFEST_DONE
	} manifestation_phase;

	switch (event) {
	case DFU_ENTER_STATE:
		manifestation_phase = MANIFEST_NOT_STARTED;
		break;

	case DFU_GETSTATUS:
		if (manifestation_phase == MANIFEST_NOT_STARTED) {
			dfu_ops.state = dfuMANIFEST;
			state_dfu_manifest(DFU_ENTER_STATE, len);
			manifestation_phase = MANIFEST_DONE;
		} else if (manifestation_phase == MANIFEST_DONE) {
			dfu_ops.state = dfuIDLE;
		}

		do_getstatus(len);
		break;

	case DFU_GETSTATE:
		do_getstate(len);
		break;

	case DFU_DETACH:
	case DFU_USB_RESET:
		do_dfu_usb_reset(len);
		break;

	default:
		dfu_ops.status = errSTALLEDPKT;
		dfu_error(len);
		break;
	}
}

__weak int dfu_download_verify(struct dfu_ops __unused(*ops))
{
	return 0;
}

static void state_dfu_manifest(uint32_t event, uint32_t *len)
{
	pr_debug("%s\n", __func__);

	switch (event) {
	case DFU_ENTER_STATE:
		if (!dfu_download_verify(&dfu_ops)) {
			dfu_ops.status = errFIRMWARE;
		}
	/* no break */

	case DFU_TIMEOUT:
		if (dfu_config_desc.dfu_functional_descriptor.bmAttributes
		    & DFU_MANIFESTATION_TOLERANT) {
			dfu_ops.state = dfuMANIFEST_SYNC;
		} else {
			dfu_ops.state = dfuMANIFEST_WAIT_RST;
		}
		break;

	case DFU_DETACH:
	case DFU_USB_RESET:
		do_dfu_usb_reset(len);
		break;

	default:
		dfu_ops.status = errSTALLEDPKT;
		dfu_error(len);
		break;
	}
}

static void state_dfu_manifest_wait_reset(uint32_t event, uint32_t *len)
{
	switch (event) {
	case DFU_DETACH:
	case DFU_USB_RESET:
		do_dfu_usb_reset(len);
		break;

	default:
		break;
	}
}

static void state_dfu_upload_idle(uint32_t event, uint32_t *len)
{
	pr_debug("%s\n", __func__);

	switch (event) {
	case DFU_UPLOAD:
		do_upload_continue(len);
		break;

	case DFU_ABORT:
		*len = 0;
		dfu_ops.state = dfuIDLE;
		break;

	case DFU_GETSTATUS:
		do_getstatus(len);
		break;

	case DFU_GETSTATE:
		do_getstate(len);
		break;

	case DFU_DETACH:
	case DFU_USB_RESET:
		do_dfu_usb_reset(len);
		break;

	default:
		dfu_ops.status = errSTALLEDPKT;
		dfu_error(len);
		break;
	}
}

static void state_dfu_error(uint32_t event, uint32_t *len)
{
	pr_debug("%s\n", __func__);

	switch (event) {
	case DFU_GETSTATUS:
		do_getstatus(len);
		break;

	case DFU_GETSTATE:
		do_getstate(len);
		break;

	case DFU_CLRSTATUS:
		*len = 0;
		dfu_ops.status = statusOK;
		dfu_ops.state = dfuIDLE;
		break;

	case DFU_DETACH:
	case DFU_USB_RESET:
		do_dfu_usb_reset(len);
		break;

	default:
		dfu_error(len);
		break;
	}
}

int class_handle_req(usb_device_request_t *setup_packet, uint32_t *len,
		     uint8_t **data)
{
	if (setup_packet->bmRequestType == USB_REQ_RECIPIENT_INTERFACE) {
		if (setup_packet->bRequest == UR_SET_INTERFACE)
			dfu_ops.alternate = UGETW(setup_packet->wValue);
		return 0;
	}

	pr_debug("  %s req %s\n", state_string[dfu_ops.state],
		 req_string[setup_packet->bRequest]);

	dfu_ops.data = *data;
	dfu_ops.len = *len;
	dfu_ops.device_request = setup_packet;

	if (dfu_state[dfu_ops.state]) {
		dfu_state[dfu_ops.state] (setup_packet->bRequest, len);
		dfu_ops.len = *len;
		pr_debug("next -> %s\n", state_string[dfu_ops.state]);
		return 0;
	}
	return -1;
}

void usb_event_cb(struct usb_event *evt)
{
	pr_debug("event %x\n", evt->event);

	static enum event_type last_event = -1;

	if (evt->event != last_event &&
	    evt->event == USB_EVENT_RESET &&
	    busy == true) {
		if (dfu_state[dfu_ops.state]) {
			uint32_t len;
			dfu_state[dfu_ops.state] (DFU_USB_RESET, &len);
			pr_debug("next -> %s\n", state_string[dfu_ops.state]);
		}
	}
	last_event = evt->event;
}
#define EP0_BUFFER_SIZE 2048
uint8_t ep0_buffer[EP0_BUFFER_SIZE];

struct usb_interface_init_data dfu_init_data = {
	.ep_complete = NULL,
	.class_handler = class_handle_req,
	.usb_evt_cb = usb_event_cb,
	.ep0_buffer = &ep0_buffer[0],
	.ep0_buffer_size = EP0_BUFFER_SIZE,
	.dev_desc = &dfu_device_desc,
	.conf_desc = (usb_config_descriptor_t *)&run_time_dfu_config_desc,
	.conf_desc_size = sizeof(run_time_dfu_config_desc),
	.strings_desc = dfu_strings_desc,
	.num_strings = sizeof(dfu_strings_desc) /
		       sizeof(usb_string_descriptor_t),
	.eps = NULL,
	.num_eps = 0,
};

void dfu_class_init()
{
	usb_interface_init(&dfu_init_data);
}
