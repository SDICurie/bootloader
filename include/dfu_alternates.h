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

#ifndef _DFU_ALTERNATES_H_
#define _DFU_ALTERNATES_H_

#include <partition.h>

#if defined(CONFIG_SPI_FLASH)
#define ALTERNATE_CONFIG_SPI_FLASH ALTERNATE
#else
#define ALTERNATE_CONFIG_SPI_FLASH(...)
#endif

#if defined(CONFIG_CACHE_PARTITION)
#define ALTERNATE_CONFIG_CACHE_PARTITION ALTERNATE
#else
#define ALTERNATE_CONFIG_CACHE_PARTITION(...)
#endif

#if defined(CONFIG_LOG_PARTITION)
#define ALTERNATE_CONFIG_LOG_PARTITION ALTERNATE
#else
#define ALTERNATE_CONFIG_LOG_PARTITION(...)
#endif

#if defined(CONFIG_SPI_FLASH_APP_DATA_PARTITION)
#define ALTERNATE_CONFIG_SPI_FLASH_APP_DATA_PARTITION ALTERNATE
#else
#define ALTERNATE_CONFIG_SPI_FLASH_APP_DATA_PARTITION(...)
#endif

#if defined(CONFIG_SPI_FLASH_SYS_EVENT_PARTITION)
#define ALTERNATE_CONFIG_SPI_FLASH_SYS_EVENT_PARTITION ALTERNATE
#else
#define ALTERNATE_CONFIG_SPI_FLASH_SYS_EVENT_PARTITION(...)
#endif

#if defined(CONFIG_SPI_FLASH_USER_DATA_PARTITION) && \
	defined(SPI_USER_DATA_NB_BLOCKS)
#define ALTERNATE_CONFIG_SPI_FLASH_USER_DATA_PARTITION ALTERNATE
#else
#define ALTERNATE_CONFIG_SPI_FLASH_USER_DATA_PARTITION(...)
#endif

#define  x86_rom_u16 { 'x', 0, '8', 0, '6', 0, '_', 0, 'r', 0, 'o', 0, 'm', 0 }
#define  x86_boot_u16 { 'x', 0, '8', 0, '6', 0, '_', 0, 'b', 0, 'o', 0, 'o', 0,	\
			't', 0 }
#define  x86_app_u16 { 'x', 0, '8', 0, '6', 0, '_', 0, 'a', 0, 'p', 0, 'p', 0 }
#define  factory1_u16 { 'f', 0, 'a', 0, 'c', 0, 't', 0, 'o', 0, 'r', 0, 'y', 0,	\
			'1', 0 }
#define  panic_u16 { 'p', 0, 'a', 0, 'n', 0, 'i', 0, 'c', 0 }
#define  factory2_u16 { 'f', 0, 'a', 0, 'c', 0, 't', 0, 'o', 0, 'r', 0, 'y', 0,	\
			'2', 0 }
#define  data_u16 { 'd', 0, 'a', 0, 't', 0, 'a', 0 }
#define  sensor_core_u16 { 's', 0, 'e', 0, 'n', 0, 's', 0, 'o', 0, 'r', 0, '_',	\
			   0, 'c', 0, 'o', 0, 'r', 0, 'e', 0 }
#define  ble_core_u16 { 'b', 0, 'l', 0, 'e', 0, '_', 0, 'c', 0, 'o', 0, 'r', 0,	\
			'e', 0 }
#define  snor_u16 { 's', 0, 'n', 0, 'o', 0, 'r', 0 }
#define  cache_u16 { 'c', 0, 'a', 0, 'c', 0, 'h', 0, 'e', 0 }
#define  log_u16 { 'l', 0, 'o', 0, 'g', 0 }
#define  app_data_u16 { 'a', 0, 'p', 0, 'p', 0, '_', 0, 'd', 0, 'a', 0, 't', 0,	\
			'a', 0 }
#define  sys_event_u16 { 's', 0, 'y', 0, 's', 0, '_', 0, 'e', 0, 'v', 0, 'e', 0, \
			 'n', 0, 't', 0 }
#define  user_data_u16 { 'u', 0, 's', 0, 'e', 0, 'r', 0, '_', 0, 'd', 0, 'a', 0, \
			 't', 0, 'a', 0 }
#define  customer_rom_u16 { 'c', 0, 'u', 0, 's', 0, 't', 0, 'o', 0, 'm', 0, 'e', \
			    0, 'r', 0, '_', 0, 'r', 0, 'o', 0, 'm', 0 }
#define  fsbl_u16 { 'f', 0, 's', 0, 'b', 0, 'l', 0 }

/* add new alternate here by adding:
 * ALTERNATE(MY_ALTERNATE, my_name) \
 *
 * and add unicode encoded string:
 * #define  my_name_u16 { 'm', 0, 'y', 0, '_', 0, 'n', 0, 'a', 0, 'm', 0, 'e', 0 }
 */
#define LIST_OF_ALTERNATES \
	ALTERNATE(X86_ROM, x86_rom) \
	ALTERNATE(X86_BOOT, x86_boot) \
	ALTERNATE(X86_APP, x86_app) \
	ALTERNATE(FACT_NONPERSISTENT, factory1)	\
	ALTERNATE(PANIC, panic)	\
	ALTERNATE(FACT_PERSISTENT, factory2) \
	ALTERNATE(DATA, data) \
	ALTERNATE(SENSOR_CORE, sensor_core) \
	ALTERNATE(BLE_CORE, ble_core) \
	ALTERNATE_CONFIG_SPI_FLASH(SNOR, snor) \
	ALTERNATE_CONFIG_CACHE_PARTITION(CACHE, cache) \
	ALTERNATE_CONFIG_LOG_PARTITION(LOG, log) \
	ALTERNATE_CONFIG_SPI_FLASH_APP_DATA_PARTITION(APP_DATA, app_data) \
	ALTERNATE_CONFIG_SPI_FLASH_SYS_EVENT_PARTITION(SYS_EVENT, sys_event) \
	ALTERNATE_CONFIG_SPI_FLASH_USER_DATA_PARTITION(USER_DATA, user_data) \
	ALTERNATE(CUSTOMER_ROM, customer_rom) \
	ALTERNATE(FSBL, fsbl)

#define ALTERNATE(alt, desc) alt ## _ALT,
enum dfu_alternate {
	LIST_OF_ALTERNATES
	ALTERNATE_COUNT
};
#undef ALTERNATE


#define ALTERNATE(alt, desc) alt ## _STRING_ID,
enum dfu_string_description {
	STRING_OFFSET=3,
	LIST_OF_ALTERNATES
	STRING_ID_COUNT
};
#undef ALTERNATE

#endif /* _DFU_ALTERNATES_H_ */
