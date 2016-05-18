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

#include <bootlogic.h>
#include <utils.h>
#include <partition.h>
#include <sign.h>
#include "factory.h"

#if defined(CONFIG_SYSTEM_EVENTS)

#include "sys_events/sys_events.h"
#endif
void weak_blank(void)

{
}

void cpu_init(void) __attribute__((weak, alias("weak_blank")));

void soc_init(void) __attribute__((weak, alias("weak_blank")));

void dnx_serial(void) __attribute__((weak, alias("weak_blank")));

void dnx_spi(void) __attribute__((weak, alias("weak_blank")));

void dnx_usb(void) __attribute__((weak, alias("weak_blank")));

void weak_dnx(void)
{
	dnx_serial();
	dnx_spi();
	dnx_usb();
}
void dnx(void) __attribute__((weak, alias("weak_dnx")));

void hardware_charging_init(void) __attribute__((weak, alias("weak_blank")));

void hardware_charging(void) __attribute__((weak, alias("weak_blank")));


void early_fixup(void) __attribute__((weak, alias("weak_blank")));

void weak_early_sign_of_life(void)
{
#if defined(CONFIG_LED)
	led_init();
#endif
#if defined(CONFIG_PLAY_BOOT_PATTERN)
	notify_boot();
#endif
}
void early_sign_of_life(void)
__attribute__((weak, alias("weak_early_sign_of_life")));

void board_early_init(void) __attribute__((weak, alias("weak_blank")));

void timer_init(void) __attribute__((weak, alias("weak_blank")));

void rtc_init(void) __attribute__((weak, alias("weak_blank")));

void serial_init(void) __attribute__((weak, alias("weak_blank")));

void gpio_init(void) __attribute__((weak, alias("weak_blank")));

void watchdog_init(void) __attribute__((weak, alias("weak_blank")));

void spi_init(void) __attribute__((weak, alias("weak_blank")));

void spi_early_init(void) __attribute__((weak, alias("weak_blank")));

void i2c_init(void) __attribute__((weak, alias("weak_blank")));

void i2c_early_init(void) __attribute__((weak, alias("weak_blank")));

void board_misc_init(void) __attribute__((weak, alias("weak_blank")));

void interrupt_init(void) __attribute__((weak, alias("weak_blank")));

void init_flash(void) __attribute__((weak, alias("weak_blank")));

void late_fixup(void) __attribute__((weak, alias("weak_blank")));

void late_sign_of_life(void) __attribute__((weak, alias("weak_blank")));

void board_late_init(void) __attribute__((weak, alias("weak_blank")));


/*
 * Boot logic
 */
void weak_boot_logic(void)
{
	enum wake_sources wake_source;
	enum reset_reasons reset_reason;
	enum boot_targets boot_target;

	override_boot_logic();
	wake_source = get_wake_source();
	reset_reason = get_reset_reason();
#if defined(CONFIG_FACTORY_BOOT)
	if (is_factory()) {
		boot_target = TARGET_FACTORY;
		set_boot_target(boot_target);
	} else
#endif
	boot_target = get_boot_target();

	charger_connected(&boot_target);
	override_boot_target(wake_source, reset_reason, &boot_target);
	boot(wake_source, reset_reason, boot_target);
}
void boot_logic(void)  __attribute__((weak, alias("weak_boot_logic")));

void override_boot_logic(void) __attribute__((weak, alias("weak_blank")));

enum wake_sources weak_get_wake_source(void)
{
	return WAKE_UNKNOWN;
}
enum wake_sources get_wake_source(void)
__attribute__((weak, alias("weak_get_wake_source")));

enum reset_reasons weak_get_reset_reason(void)
{
	return RESET_UNKNOWN;
}
enum reset_reasons get_reset_reason(void)
__attribute__((weak, alias("weak_get_reset_reason")));

enum boot_targets weak_get_boot_target(void)
{
	return TARGET_MAIN;
}
enum boot_targets get_boot_target(void)
__attribute__((weak, alias("weak_get_boot_target")));

void weak_set_boot_target(enum boot_targets __unused(boot_target))
{
}

void set_boot_target(enum boot_targets boot_target)
__attribute__((weak, alias("weak_set_boot_target")));

void weak_override_boot_target(enum wake_sources	__unused(wake_source),
			       enum reset_reasons	__unused(reset_reason),
			       enum boot_targets	__unused(*boot_target))
{
}

void override_boot_target(enum wake_sources	wake_source,
			  enum reset_reasons	reset_reason,
			  enum boot_targets *	boot_target)
__attribute__((weak, alias("weak_override_boot_target")));

enum boot_flags get_boot_flags(void)
__attribute__((weak, alias("weak_get_boot_flags")));

enum boot_flags weak_get_boot_flags(void)
{
	return BOOT_NORMAL;
}

__weak void set_boot_flags(enum boot_flags __unused(boot_flags))
{
}

__weak void mpu_init(enum boot_targets __unused(boot_target))
{
}


void weak_boot(enum wake_sources wake_source, enum reset_reasons reset_reason,
	       enum boot_targets boot_target)
{
	void (*do_boot[])(enum wake_sources, enum reset_reasons,
			  enum boot_targets) = {
		[TARGET_MAIN] = boot_main,
		[TARGET_CHARGING] = boot_charging,
		[TARGET_WIRELESS_CHARGING] = boot_charging,
		[TARGET_RECOVERY] = boot_recovery,
		[TARGET_FLASHING] = boot_flashing,
		[TARGET_FACTORY] = boot_main,
		[TARGET_OTA] = boot_ota,
		[TARGET_DTM] = boot_dtm,
		[TARGET_CERTIFICATION] = boot_default,
		[TARGET_RESTORE_SETTINGS] = boot_restore_settings,
		[TARGET_APP_1] = boot_default,
		[TARGET_APP_2] = boot_default,
		[TARGET_RESERVED_1] = boot_default,
		[TARGET_RESERVED_2] = boot_default,
		[TARGET_RESERVED_3] = boot_default,
		[TARGET_RESERVED_4] = boot_default
	};

#if defined(CONFIG_MPU)
	mpu_init();
#endif

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_WATCHDOG_TIMEOUT)
	watchdog_start(CONFIG_WATCHDOG_TIMEOUT);
	watchdog_reset();
#endif

#if defined(CONFIG_SYSTEM_EVENTS)
	system_event_push_boot_event(reset_reason);
#endif

	if (boot_target < ARRAY_SIZE(do_boot))
		do_boot[boot_target] (wake_source, reset_reason, boot_target);
	boot_default(wake_source, reset_reason, boot_target);
}
void boot(enum wake_sources wake_source, enum reset_reasons reset_reason,
	  enum boot_targets boot_target)
__attribute__((weak, alias("weak_boot")));

void weak_blank_with_parameter(uint32_t __unused(p))
{
}

void weak_blank_with_parameter_16(uint16_t __unused(p))
{
}

void watchdog_start(uint16_t period_sec)
__attribute__((weak, alias("weak_blank_with_parameter_16")));

void watchdog_reset(void)
__attribute__((weak, alias("weak_blank")));

void load_binary(uint32_t address)
__attribute__((weak, alias("weak_blank_with_parameter")));

void start_binary(uint32_t address)
__attribute__((weak, alias("weak_blank_with_parameter")));

void weak_charger_connected(enum boot_targets *boot_target)
{
	if (is_conductive_charger_connected(boot_target))
		set_boot_target(*boot_target);
	else if (is_wireless_charger_connected(boot_target))
		set_boot_target(*boot_target);
}
void charger_connected(enum boot_targets *boot_target)
__attribute__((weak, alias("weak_charger_connected")));


uint8_t weak_device_charger_connected(enum boot_targets __unused(*boot_target))
{
	return 0;
}

uint8_t is_wireless_charger_connected(enum boot_targets *boot_target)
__attribute__((weak, alias("weak_device_charger_connected")));

uint8_t is_conductive_charger_connected(enum boot_targets *boot_target)
__attribute__((weak, alias("weak_device_charger_connected")));


void weak_boot_default(enum wake_sources	__unused(wake_source),
		       enum reset_reasons	__unused(reset_reason),
		       enum boot_targets	__unused(boot_target))
{
#if defined(CONFIG_SECURE_BOOT)
	if (secure_boot (QUARK_FLASH_START_ADDR))
#endif
	((void (*)(void))(QUARK_FLASH_START_ADDR + VERSION_HEADER_SIZE +
			  CONFIG_SIGNATURE_HEADER_SIZE))();
}

void boot_main(enum wake_sources wake_source, enum reset_reasons reset_reason,
	       enum boot_targets boot_target)
__attribute__((weak, alias("weak_boot_default")));


void boot_default(enum wake_sources	wake_source,
		  enum reset_reasons	reset_reason,
		  enum boot_targets	boot_target)
__attribute__((weak, alias("weak_boot_default")));

void boot_charging(enum wake_sources	wake_source,
		   enum reset_reasons	reset_reason,
		   enum boot_targets	boot_target)
__attribute__((weak, alias("weak_boot_default")));

void boot_recovery(enum wake_sources	wake_source,
		   enum reset_reasons	reset_reason,
		   enum boot_targets	boot_target)
__attribute__((weak, alias("weak_boot_default")));

void boot_flashing(enum wake_sources	wake_source,
		   enum reset_reasons	reset_reason,
		   enum boot_targets	boot_target)
__attribute__((weak, alias("weak_boot_default")));

void boot_factory(enum wake_sources	wake_source,
		  enum reset_reasons	reset_reason,
		  enum boot_targets	boot_target)
__attribute__((weak, alias("weak_boot_default")));

void boot_restore_settings(enum wake_sources	wake_source,
			   enum reset_reasons	reset_reason,
			   enum boot_targets	boot_target)
__attribute__((weak, alias("weak_boot_default")));

void boot_ota(enum wake_sources wake_source, enum reset_reasons reset_reason,
	      enum boot_targets boot_target)
__attribute__((weak, alias("weak_boot_default")));


void boot_dtm(enum wake_sources wake_source, enum reset_reasons reset_reason,
	      enum boot_targets boot_target)
__attribute__((weak, alias("weak_boot_default")));


void features_init(void)
__attribute__((weak, alias("weak_blank")));
