/*
 * Copyright (c) 2016, Intel Corporation. All rights reserved.
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

#include <stdbool.h>

#include "sys_events/sys_events.h"

/* SPI flash management */
#include "cir_storage.h"
#include "cir_storage_flash_spi/cir_storage_flash_spi.h"

/* Memory management */
#include <mem.h>
#include "partition.h"

#include "version.h"
#include "utils.h"
#include "printk.h"
#include "time.h"

/* TODO: Use the commonly defined constant instead */
#define PANIC_NVM_BASE (DEBUGPANIC_START_BLOCK * EMBEDDED_FLASH_BLOCK_SIZE)
/* Align address on 32bits (add 3 then clear LSBs) */
#define PANIC_ALIGN_32(x) (((uint32_t)(x) + 3) & ~(3))

#ifndef NULL
#define NULL 0
#endif


static cir_storage_t *storage = NULL;

void system_events_init()
{
	BUILD_BUG_ON(sizeof(struct system_event) > SYSTEM_EVENT_SIZE);

	storage = cir_storage_flash_spi_init(SYSTEM_EVENT_SIZE,
					     SPI_SYSTEM_EVENT_START_BLOCK,
					     SPI_SYSTEM_EVENT_NB_BLOCKS);
}

void __attribute__((weak)) on_system_event_generated(struct system_event
						     __unused(*evt))
{
}

void system_event_push(struct system_event *event)
{
	if (storage) {
		event->h.hash[0] = version_header.hash[0];
		event->h.hash[1] = version_header.hash[1];
		event->h.hash[2] = version_header.hash[2];
		event->h.hash[3] = version_header.hash[3];
		cir_storage_push(storage, (uint8_t *)event);
		on_system_event_generated(event);
	}
}

struct system_event *system_event_pop()
{
	if (storage) {
		struct system_event *evt = balloc(SYSTEM_EVENT_SIZE);
		cir_storage_err_t err = cir_storage_pop(storage, (uint8_t *)evt);
		if (err == CBUFFER_STORAGE_SUCCESS)
			return evt;
		else {
			bfree(evt);
			return NULL;
		}
	} else {
		return NULL;
	}
}

/**
 * Fill the header part of a system event
 *
 * @param event the system event header to fill
 * @param id the system event id to set in the header.
 */
static void system_event_fill_header(struct system_event *e, int type)
{
	e->h.type = type;
	e->h.size = SYSTEM_EVENT_SIZE;
	e->h.version = SYSTEM_EVENT_VERSION;
	e->h.timestamp = time();
}

void system_event_push_boot_event(enum reset_reasons reset_reason)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_BOOT);
	evt.event_data.boot.reason = reset_reason;
	evt.event_data.boot.uptime = 0;
	system_event_push(&evt);
}

void system_event_push_shutdown(int type, int reason)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_SHUTDOWN);
	evt.event_data.shutdown.type = type;
	evt.event_data.shutdown.reason = reason;
	system_event_push(&evt);
}

void system_event_push_battery(uint8_t type, uint8_t data)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_BATTERY);
	evt.event_data.battery.type = type;
	evt.event_data.battery.u.data = data;
	system_event_push(&evt);
}

void system_event_push_uptime(void)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_UPTIME);
	evt.event_data.uptime.time = get_uptime_ms() / 1000;

	system_event_push(&evt);
}

void system_event_push_ota(uint8_t state, uint8_t error_code)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_OTA);
	evt.event_data.ota.state = state;
	evt.event_data.ota.error_code = error_code;
	system_event_push(&evt);
}

void system_event_push_hw_charging(uint16_t level_mv)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_HW_CHARGING);
	evt.event_data.hw_charging.level_mv = level_mv;
	system_event_push(&evt);
}

void system_event_push_flash(uint8_t state, uint8_t error_code)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_FLASH);
	evt.event_data.flash.state = state;
	evt.event_data.flash.error_code = error_code;
	system_event_push(&evt);
}
