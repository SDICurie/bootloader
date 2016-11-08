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

#include "api/part.h"
#include "api/platform-desc.h"
#include "api/bootloader-os-comm.h"
#include "api/part-helpers.h"

declare_device(INTERNAL_FLASH_0) =
{
	/* Header */
	.major = 1,
	.minor = 0,

	/* Global ID of the storage device in the system */
	.id = INTERNAL_FLASH_0,

	/* link to the driver */
	.technology = INTERNAL_FLASH,

	/* Sequence number per technology/controller (or controller group if regions
	 * in memory space are contiguous) */
	/*uint8_t tech_index;*/

	/* Driver can use it or not. */
	.block_size = 2048,
};

declare_device(SPI_FLASH_0) =
{
	/* Header */
	.major = 1,
	.minor = 0,

	/* Global ID of the storage device in the system */
	.id = SPI_FLASH_0,

	/* link to the driver */
	.technology = SPI_FLASH,

	/* Sequence number per technology/controller (or controller group if regions
	 * in memory space are contiguous) */
	/*uint8_t tech_index;*/

	/* Driver can use it or not. */
	.block_size = 4096,
};

#include "project_mapping.h"

/* The actual list of partitions across the system (excl. devices that have
 * their own partition table) */
struct partition partitions[] __section(".table_partition") =
{
	/* INTERNAL_FLASH */
	define_partition(PART_FACTORY_INTEL,
					"factory intel",
					bind_to_device(INTERNAL_FLASH_0),
					OFFSET_BEGINNING,
					512,
					ATTR_OTP)
	define_partition(PART_FACTORY_OEM,
					"factory oem",
					bind_to_device(INTERNAL_FLASH_0),
					OFFSET_AUTO,
					512,
					ATTR_OTP)
	define_partition(PART_RESET_VECTOR,
					"reset vector",
					bind_to_device(INTERNAL_FLASH_0),
					OFFSET_AUTO,
					7 * KiB,
					ATTR_OTP)
	define_partition(PART_BOOTLOADER,
					"bootloader",
					bind_to_device(INTERNAL_FLASH_0),
					OFFSET_AUTO,
					4 * KiB,
					ATTR_NONE)
	define_partition(PART_RESERVED,
					"reserved",
					bind_to_device(INTERNAL_FLASH_0),
					OFFSET_AUTO,
					48 * KiB,
					ATTR_RESERVED)
	define_partition(PART_PANIC,
					"panic",
					bind_to_device(INTERNAL_FLASH_0),
					OFFSET_AUTO,
					4 * KiB,
					ATTR_CLEAR_FACTORY_RESET)
	define_partition(PART_MAIN,
					"main",
					bind_to_device(INTERNAL_FLASH_0),
					OFFSET_AUTO,
					144 * KiB,
					ATTR_NONE)
	define_partition(PART_SENSOR,
					"sensor",
					bind_to_device(INTERNAL_FLASH_0),
					OFFSET_AUTO,
					152 * KiB,
					ATTR_NONE)
	define_partition(PART_FACTORY_NPERSIST,
					"factory npersist",
					bind_to_device(INTERNAL_FLASH_0),
					OFFSET_AUTO,
					6 * KiB,
					ATTR_CLEAR_FACTORY_RESET)
	define_partition(PART_FACTORY_PERSIST,
					"factory persist",
					bind_to_device(INTERNAL_FLASH_0),
					OFFSET_AUTO,
					6 * KiB,
					ATTR_NONE)
	define_partition(PART_APP_DATA,
					"app data",
					bind_to_device(INTERNAL_FLASH_0),
					OFFSET_AUTO,
					8 * KiB,
					ATTR_CLEAR_FACTORY_RESET)
	define_partition(PART_FACTORY,
					"factory",
					bind_to_device(INTERNAL_FLASH_0),
					OFFSET_AUTO,
					4 * KiB,
					ATTR_NONE)

	/* SPI_FLASH */
	define_partition(PART_OTA_CACHE,
					"ota cache",
					bind_to_device(SPI_FLASH_0),
					OFFSET_BEGINNING,
					1 * MiB,
					ATTR_CLEAR_FACTORY_RESET)
	define_partition(PART_APP_LOGS,
					"app logs",
					bind_to_device(SPI_FLASH_0),
					OFFSET_AUTO,
					256 * KiB,
					ATTR_CLEAR_FACTORY_RESET)
	define_partition(PART_ACTIVITY_DATA,
					"activity data",
					bind_to_device(SPI_FLASH_0),
					OFFSET_AUTO,
					756 * KiB,
					ATTR_CLEAR_FACTORY_RESET)
	define_partition(PART_SYSTEM_EVENTS,
					"system events",
					bind_to_device(SPI_FLASH_0),
					OFFSET_AUTO,
					12 * KiB,
					ATTR_CLEAR_FACTORY_RESET)
#if defined(EXTRA_PARTITIONS)
	EXTRA_PARTITIONS
#endif
};

int partition_count = sizeof(partitions) / sizeof(struct partition);

struct partition partition_resizes[] =
{
#if defined(RESIZE_PARTITIONS)
	RESIZE_PARTITIONS
#endif
};

int partition_resize_count =
	sizeof(partition_resizes) / sizeof(struct partition);

struct platform_device platform_devices[]  __section(".table_partition") =
{
	{
		/* Header */
		.major = 1,
		.minor = 0,

		.type = STORAGE_DEVICE,
		.device = bind_to_device(INTERNAL_FLASH_0),
	},
	{
		/* Header */
		.major = 1,
		.minor = 0,

		.type = STORAGE_DEVICE,
		.device = bind_to_device(SPI_FLASH_0)
	}
};

struct bootloader_os_commbuff os_commbuff  __section(".table_partition") =
{
	.magic = { 'B', 'L', 'O', 'S' },
	.major = 1,
	.minor = 0,
	.plat_desc =
	{
		.magic = { 'T', 'R', 'E', 'E' },
		.major = 1,
		.minor = 0,
		.devices_count = sizeof(platform_devices) /
				 sizeof(struct platform_device),
		.devices = platform_devices,
	},
	.partitions_count = sizeof(partitions) / sizeof(struct partition),
	.partitions = partitions
};
