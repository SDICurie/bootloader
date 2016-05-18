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

#include "machine/soc/intel/quark_se/scss_registers.h"

#if defined(CONFIG_USB)
#include <usb/usb_api.h>
#include <usb/usb.h>
#include <usb/usb_driver_interface.h>
#include <usb/usb_shared_interface.h>
#include "usb_setup.h"
#include <utils.h>
#endif

#if defined(CONFIG_DNX)
#include <bootlogic.h>
#endif

#if defined(CONFIG_XMODEM)
#include <serial/xmodem.h>
#endif

#if defined(CONFIG_USB_DFU)
#include <dfu.h>
#include <dfu_spi_flash.h>
#endif

#if defined(CONFIG_SWD)
#include <swd/swd.h>
#endif

#if defined(CONFIG_SYSTEM_EVENTS)
#include <sys_events/sys_events.h>
#endif

#if defined(CONFIG_LP5562)
#include <lp5562/lp5562.h>
#endif


#include <printk.h>
#include <partition.h>
#include <gpio.h>
#include <uart.h>
#include <i2c.h>
#include <soc_flash.h>
#include <rom/soc_rom.h>

#include <mem.h>
#include <bootlogic.h>
#include <factory_data.h>
#include <features.h>
#include <features_soc.h>
#include "time.h"

#if defined(CONFIG_DEBUG_RAMDUMP_SERIAL)
#include "panic.h"
#include "quark_se/boot_config.h"
#endif
#include <sign.h>

#ifndef NULL
#define NULL 0
#endif

#define SOC_UART1_BASE  0xB0002400

#if defined(CONFIG_USB) && defined(CONFIG_USB_DFU) && defined(CONFIG_DNX)
uint8_t usb_gpio;
#endif

#define OSC_INTERNAL 1
#define OSC_EXTERNAL 0
#define INTERNAL_OSC_TRIM 0x240

void x_modem(void);

/* In Quark SE, Factory data is in the first page of the ROM. */
#define FACTORY_DATA_ADDR ROM_PAGE_START_ADDR
const struct factory_data *global_factory_data =
	(struct factory_data *)FACTORY_DATA_ADDR;

/* This structure has to be allocated in a RAM region that needs to be shared
 * between the bootloader code and the application code.
 */
#if defined(CONFIG_ADVANCED_BOOT)
static volatile uint32_t __board_features __section(".board_features");
volatile uint32_t *board_features;
#endif

#if defined(CONFIG_DEBUG_RAMDUMP_SERIAL)
static void uart_print(uint8_t *s)
{
	while (*s)
		uart_out(*s++);
}

static void print_itoa(uint8_t c)
{
	uart_out(((c >> 4) > 9) ? ((c >> 4) - 10) + 'A' : (c >> 4) + '0');
	uart_out(((c & 0xF) > 9) ? ((c & 0xF) - 10) + 'A' : (c & 0xF) + '0');
}

static void print_word(uint32_t w)
{
	print_itoa((uint8_t)(w >> 24));
	print_itoa((uint8_t)(w >> 16));
	print_itoa((uint8_t)(w >> 8));
	print_itoa((uint8_t)(w));
}
#endif

void set_oscillator(int internal)
{
	if (internal) {
		/* Start internal oscillator (with trim) */
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_CFG1) |=
			INTERNAL_OSC_TRIM << OSC0_CFG1_INTERNAL_OSC_TRIM_BIT |
			OSC0_CFG1_INTERNAL_OSC_EN_MASK;
		/* Wait internal oscillator ready */
		while (!((MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
						 SCSS_OSC0_STAT1)
			  & OSC0_STAT1_LOCK_INTERNAL))) ;
		/* Trim internal oscillator */
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_CFG1) =
			INTERNAL_OSC_TRIM << OSC0_CFG1_INTERNAL_OSC_TRIM_BIT |
			OSC0_CFG1_INTERNAL_OSC_EN_MASK;
	} else {
		/* Set clk to 32MHz, external oscillator, 5.5pF load */
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_CFG1) |=
			OSC0_CFG1_XTAL_OSC_TRIM_5_55_PF |
			OSC0_CFG1_XTAL_OSC_EN_MASK;
		/* Wait internal regulator ready */
		while (!((MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
						 SCSS_OSC0_STAT1)
			  & OSC0_STAT1_LOCK_XTAL))) ;
		/* Switch to external oscillator */
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_CFG1) =
			OSC0_CFG1_XTAL_OSC_TRIM_5_55_PF |
			(OSC0_CFG1_XTAL_OSC_EN_MASK |
			 OSC0_CFG1_XTAL_OSC_OUT_MASK);
	}
}

/* Soc Specific initialization */
void soc_init(void)
{
#if defined(CONFIG_ADVANCED_BOOT)
	/* Reset AON counter */
	uint32_t time = get_uptime_32k();
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_AONC_CFG) =
		AONC_CNT_DIS;
	while (time <= get_uptime_32k()) {
		__asm__ volatile ("nop");
	}
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_AONC_CFG) =
		AONC_CNT_EN;
#endif
#if defined(CONFIG_QUARK_SE_SWITCH_INTERNAL_OSCILLATOR)
	set_oscillator(OSC_INTERNAL);
#else
	set_oscillator(OSC_EXTERNAL);
#endif

#if defined(CONFIG_SERIAL)
	/* UART1 pins output in quark for curie */
	SET_PIN_MODE(16, QRK_PMUX_SEL_MODEC);
	SET_PIN_MODE(17, QRK_PMUX_SEL_MODEC);
	uart_init(SOC_UART1_BASE, 115200);
	pr_info("BOOTLOADER INIT\n\r");
#endif

#if defined(CONFIG_DEBUG_RAMDUMP_SERIAL)
/* Check if a panic occurred on Quark or Arc core*/
	if ((*((uint32_t *)(QUARK_PANIC_RAM_ADDR)-1) == PANIC_DATA_MAGIC) ||
	    (*((uint32_t *)(ARC_PANIC_RAM_ADDR)-1) == PANIC_DATA_MAGIC)) {
		uart_print("\n--------Start of RAM dump--------\n");
		/* Then, dump the 80 kb of RAM */
		for (uint32_t *p = BOOT_RAM_START_ADDR;
		     p < (BOOT_RAM_START_ADDR + BOOT_RAM_SIZE + 1024);
		     p++) {
			if ((uint32_t)p % 32 == 0) {
				uart_out('\n');
				print_word((uint32_t)p);
				uart_print(" : ");
			}

			print_word(*p);
			uart_out(' ');
		}
		uart_print("\n--------End of RAM dump--------\n");
	}
#endif

#if defined(CONFIG_USB) && defined(CONFIG_USB_DFU) && defined(CONFIG_DNX)
	usb_driver_os_dep->printk = printk;
	usb_driver_os_dep->alloc = usb_balloc;
	usb_driver_os_dep->free = bfree;

	SET_PIN_MODE(7, QRK_PMUX_SEL_MODEA);

	gpio_cfg_data_t pin_cfg = {
		.gpio_type = GPIO_INPUT,
		.int_type = LEVEL,
		.int_polarity = ACTIVE_LOW,
		.int_debounce = DEBOUNCE_OFF,
		.int_ls_sync = LS_SYNC_OFF,
		.gpio_cb = NULL
	};

	if ((gpio_set_config(0, 7, &pin_cfg))
	    != DRV_RC_OK)
		pr_info("error configure gpio\n");
	usb_gpio = gpio_read(0, 7);

	pr_info("usb status: %d\n", usb_gpio);
#endif

#if defined(CONFIG_SWD)
	// Force BLE Core reset even if SWD debug has been entered
	swd_init();
	swd_debug_mode_reset_to_normal();
#endif

#if defined(CONFIG_ADVANCED_BOOT)
	/* Setup empty features set by default */
	__board_features = 0x0;
	board_features = &__board_features;

	/* Check the revision of the SoC */
	/* Rev 1 silicon has bit 2 of SLP_CFG register tied to 1 */
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SLP_CFG_BASE) &= ~(0x4);
	if ((MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
				    SLP_CFG_BASE) & 0x4) == 0)
		board_feature_enable(HW_IDLE_QUIRK);
#endif
}

#if defined(CONFIG_XMODEM) && defined(CONFIG_DNX)

int input_byte(int pTimeout)
{
	unsigned char c;
	uint32_t timeout = get_uptime_32k() + (pTimeout * 32);

	while (get_uptime_32k() < timeout) {
		if (uart_in(&c) == 0) return (int)c;
	}
	return -1;
}

void output_byte(int pChar)
{
	uart_out((char)pChar);
}

void x_modem(void)
{
	unsigned char data[PAGE_SIZE];
	unsigned int current_block = BOOT_PAGE_START;
	const unsigned int end_block = BOOT_PAGE_START +
				       (PAGE_SIZE * BOOT_PAGE_NR);
	int receive;
	unsigned int retlen;
	unsigned int address;

	while ((receive =
			xmodem_receive(data, sizeof(data), input_byte,
				       output_byte)) > 0) {
		soc_flash_block_erase(current_block, 1);
		address = PAGE_SIZE * current_block;
		receive = receive | 0x3;
		soc_flash_write(address, receive / 4, &retlen, (uint32_t *)data);
		current_block++;
		if (current_block == end_block) {
			break;
		}
	}
}

void dnx_serial(void)
{
	x_modem();
}
#endif

#if defined(CONFIG_USB) && defined(CONFIG_USB_DFU) && defined(CONFIG_DNX)
/*
 * Support DnX on DFU over USB
 * FIXME: move to a more common place once proper USB api available
 * FIXME: add usb vbus detection in loop
 */
extern void usb_shared_interface_init();
static bool target_flash_mode = false;
void usb_init(void)
{
	usb_shared_interface_init();
	if (usb_gpio == 0) {
		return;
	}
	set_oscillator(OSC_EXTERNAL);
	platform_usb_init();
	usb_driver_init(SOC_USB_BASE_ADDR);
	switch_to_dfu_mode();
	if (get_boot_target() == TARGET_FLASHING)
		target_flash_mode = true;
}

void dnx_usb(void)
{
	uint32_t saved_date, current_date;
	uint32_t timeout_ticks;

	saved_date = get_uptime_32k();
	current_date = get_uptime_32k();
	if (usb_gpio == 0) {
		return;
	}
	timeout_ticks = CONFIG_DNX_TIMEOUT_S * 32000;

	while (
#if defined(CONFIG_USB_DFU)
		dfu_busy() == true || target_flash_mode == true ||
#endif
		(saved_date + timeout_ticks) > current_date) {
#if defined(CONFIG_WATCHDOG)
		watchdog_reset();
#endif
		poll_usb();
		current_date = get_uptime_32k();
	}
	platform_usb_release();

#if defined(CONFIG_QUARK_SE_SWITCH_INTERNAL_OSCILLATOR)
	set_oscillator(OSC_INTERNAL);
#endif
}

struct flash_block {
	uint32_t block_start;
	uint32_t block_count;
};

static const struct flash_block flash_blocks[] = {
	/* rom */
	{ ROM_PAGE_START, ROM_PAGE_NR },
	/* x86_boot */
	{ BOOT_PAGE_START, BOOT_PAGE_NR },
	/* x86_app */
	{ QUARK_START_PAGE, QUARK_NB_PAGE },
	/* Factory reset non persistent data */
	{ FACTORY_RESET_NON_PERSISTENT_START_BLOCK,
	  FACTORY_RESET_NON_PERSISTENT_NB_BLOCKS },
	/* panic */
	{ DEBUGPANIC_START_BLOCK, DEBUGPANIC_NB_BLOCKS },
	/* Factory reset persistent data */
	{ FACTORY_RESET_PERSISTENT_START_BLOCK,
	  FACTORY_RESET_PERSISTENT_NB_BLOCKS },
	/* data */
	{ APPLICATION_DATA_START_BLOCK, APPLICATION_DATA_NB_BLOCKS },
	/* sensor_core */
	{ ARC_START_PAGE, ARC_NB_PAGE },
};

int dfu_download_verify(struct dfu_ops *ops)
{
	uint32_t address;

	switch (ops->alternate) {
	case X86_APP_ALT: /* x86_app or bootupdater */
		address = flash_blocks[ops->alternate].block_start *
			  PAGE_SIZE + 0x40000000;

		if (!secure_boot(address)) {
			ops->status = errVERIFY;
		}
		break;
	default:
		break;
	}

	if (ops->status != statusOK) {
		soc_flash_block_erase(flash_blocks[ops->alternate].block_start,
				      flash_blocks[ops->alternate].block_count);
		return 0;
	}
	return 1;
}

/* For now prohibit write on some alternates.
 * A DFU_CLRSTATUS request clears the status/state and allows
 * further (valid) transfers */
uint32_t dfu_forbidden_transfer_r(struct dfu_ops *ops)
{
	ops->status = errTARGET;
	return 0;
}

void dfu_forbidden_transfer_we(struct dfu_ops *ops)
{
	ops->status = errTARGET;
}

void dfu_flash_write(struct dfu_ops *ops)
{
	usb_device_request_t *setup_packet = ops->device_request;
	unsigned int retlen;
	int ret;
	unsigned int address = (flash_blocks[ops->alternate].block_start
				+ UGETW(setup_packet->wValue)) * PAGE_SIZE;
	unsigned int start_blk = flash_blocks[ops->alternate].block_start;

	ret = soc_flash_block_erase(start_blk + UGETW(setup_packet->wValue), 1);
	if (ret != DRV_RC_OK) {
		ops->status = errERASE;
		return;
	}
	ret = soc_flash_write(address, ops->len / 4, &retlen,
			      (uint32_t *)(ops->data));
	if (ret != DRV_RC_OK) {
		ops->status = errWRITE;
	}
}

void dfu_customer_otp_write(struct dfu_ops *ops)
{
	usb_device_request_t *setup_packet = ops->device_request;
	unsigned int retlen;
	int ret;
	unsigned int i;
	unsigned int address = UGETW(setup_packet->wValue) * PAGE_SIZE;

	if (address != 0 || ops->len > ROM_CUSTOMER_PROVISIONNING_SIZE) {
		ops->state = dfuERROR;
		ops->status = errADDRESS;
		return;
	}

	unsigned char data[PAGE_SIZE];
	/* copy intel provisionning data*/
	for (i = 0; i < PAGE_SIZE / 4; i++) {
		*(((int *)data) + i) =
			MMIO_REG_VAL_FROM_BASE(ROM_PAGE_START_ADDR,
					       ROM_PAGE_START + 4 * i);
	}

	for (i = 0; i < ROM_CUSTOMER_PROVISIONNING_SIZE; i++) {
		if (i < ops->len)
			data[ROM_CUSTOMER_PROVISIONNING_OFFSET +
			     i] = *(((unsigned char *)ops->data) + i);
		else
			data[ROM_CUSTOMER_PROVISIONNING_OFFSET + i] = 0xff;
	}

	ret = soc_rom_block_erase(0, 1);
	ret |=
		soc_rom_write(address + 4, (PAGE_SIZE - 4) / 4, &retlen,
			      (uint32_t *)data + 4);
	if (ret == DRV_RC_OK) {
		ops->state = dfuDNLOAD_IDLE;
	} else {
		ops->state = dfuERROR;
		ops->status = errWRITE;
	}
}


uint32_t dfu_customer_otp_read(struct dfu_ops *ops)
{
	usb_device_request_t *setup_packet = ops->device_request;
	unsigned int i;
	unsigned int address = UGETW(setup_packet->wValue) * PAGE_SIZE;

	if (address != 0) {
		ops->state = dfuIDLE;
		ops->len = 0;
		return ops->len;
	}

	ops->len = ROM_CUSTOMER_PROVISIONNING_SIZE;
	// short frame
	ops->state = dfuIDLE;

	for (i = 0; i < ops->len / 4; i++) {
		*(((int *)ops->data) + i) =
			MMIO_REG_VAL_FROM_BASE(ROM_PAGE_START_ADDR,
					       (ROM_PAGE_START +
						ROM_CUSTOMER_PROVISIONNING_OFFSET
						+ 4 * i));
	}
	return ops->len;
}

void dfu_fsbl_otp_write(struct dfu_ops *ops)
{
	usb_device_request_t *setup_packet = ops->device_request;
	unsigned int retlen;
	int ret = 0;
	int i;
	unsigned char data[PAGE_SIZE];
	unsigned int address = UGETW(setup_packet->wValue);

	if (address == 0) {
		/* copy intel provisionning data*/
		for (i = 0; i < ROM_FSBL_OFFSET / 4; i++) {
			*(((int *)data) + i) =
				MMIO_REG_VAL_FROM_BASE(ROM_PAGE_START_ADDR,
						       ROM_PAGE_START + 4 * i);
		}
		/* erase 2k block and rewrite 1k data partition*/
		ret = soc_rom_block_erase(0, 1);
		ret |=
			soc_rom_write(address + 4, (ROM_FSBL_OFFSET - 4) / 4,
				      &retlen,
				      (uint32_t *)data + 1);
	}

	if (address < ROM_PAGE_NR - 1)
		/* erase next overlapping block */
		ret |= soc_rom_block_erase(address + 1, 1);

	/* write data */
	address = (address * PAGE_SIZE) + ROM_FSBL_OFFSET;
	ret |= soc_rom_write(address, ops->len / 4, &retlen,
			     (uint32_t *)(ops->data));

	if (ret == DRV_RC_OK) {
		ops->state = dfuDNLOAD_IDLE;
	} else {
		ops->state = dfuERROR;
		ops->status = errWRITE;
	}
}

uint32_t dfu_fsbl_otp_read(struct dfu_ops *ops)
{
	usb_device_request_t *setup_packet = ops->device_request;
	unsigned int i;
	unsigned int address = UGETW(setup_packet->wValue) * PAGE_SIZE;


	ops->len = ROM_FSBL_SIZE - address;
	if (ops->len >= PAGE_SIZE) {
		ops->len = PAGE_SIZE;
	}

	address += ROM_FSBL_OFFSET;
	for (i = 0; i < ops->len / 4; i++) {
		*(((int *)ops->data) + i) =
			MMIO_REG_VAL_FROM_BASE(ROM_PAGE_START_ADDR,
					       (address + 4 * i));
	}
	return ops->len;
}

uint32_t dfu_soc_flash_read(struct dfu_ops *ops, unsigned int base_address)
{
	usb_device_request_t *setup_packet = ops->device_request;
	unsigned int i;
	unsigned int address = UGETW(setup_packet->wValue) * PAGE_SIZE;

	ops->len = flash_blocks[ops->alternate].block_count * PAGE_SIZE -
		   address;
	if (ops->len >= PAGE_SIZE) {
		ops->len = PAGE_SIZE;
	}
	address += flash_blocks[ops->alternate].block_start * PAGE_SIZE;

	for (i = 0; i < ops->len / 4; i++) {
		*(((int *)ops->data) + i) =
			MMIO_REG_VAL_FROM_BASE(base_address, address + 4 * i);
	}
	return ops->len;
}

uint32_t dfu_flash_read(struct dfu_ops *ops)
{
	return dfu_soc_flash_read(ops, 0x40000000);
}

uint32_t dfu_otp_read(struct dfu_ops *ops)
{
	return dfu_soc_flash_read(ops, ROM_PAGE_START_ADDR);
}

uint32_t dfu_swd_read(struct dfu_ops *ops)
{
#if defined(CONFIG_SWD)
	usb_device_request_t *setup_packet = ops->device_request;
	int ret = SWD_ERROR_FAULT;
	unsigned int address = UGETW(setup_packet->wValue) * PAGE_SIZE;

	if (address == 0)
		swd_init();

	ops->len = BLE_CORE_FLASH_SIZE - address;
	if (ops->len >= PAGE_SIZE) {
		ops->len = PAGE_SIZE;
	}

	ret = swd_dump_image(address, (uint32_t *)(ops->data), ops->len);

	if (ret != SWD_ERROR_OK) {
		ops->status = errUNKNOWN;
		ops->len = 0;
	}
	return ops->len;
#endif
}

void dfu_swd_write(struct dfu_ops *ops)
{
#if defined(CONFIG_SWD)
	usb_device_request_t *setup_packet = ops->device_request;
	int __unused(ret) = SWD_ERROR_FAULT;
	unsigned int address = UGETW(setup_packet->wValue) * PAGE_SIZE;

	if (address == 0) {
		swd_init();
		ret = swd_erase_all();
		if (ret != DRV_RC_OK) {
			ops->status = errERASE;
			return;
		}
	}

	ret = swd_load_image(address, (uint32_t *)(ops->data), ops->len);
	if (ret != SWD_ERROR_OK) {
		ops->status = errWRITE;
		return;
	}
	ret = swd_verify_image(address, (uint32_t *)(ops->data),
			       ops->len);
	if (ret != SWD_ERROR_OK) {
		ops->status = errVERIFY;
	}
#endif
}

void dfu_set_alternate(struct dfu_ops *ops)
{
#if defined(CONFIG_SPI_FLASH)
	spi_flash_info_t *spi_flash;
#endif
	switch (ops->alternate) {
	case X86_ROM_ALT: /* rom */
		ops->write = dfu_forbidden_transfer_we;
		ops->read = dfu_otp_read;
		ops->block_count = flash_blocks[ops->alternate].block_count;
		break;
	case CUSTOMER_ROM_ALT: /* customer rom */
		ops->write = dfu_customer_otp_write;
		ops->read = dfu_customer_otp_read;
		ops->block_count = 1;
		break;
	case FSBL_ALT: /* fsbl rom */
		ops->write = dfu_fsbl_otp_write;
		ops->read = dfu_fsbl_otp_read;
		ops->block_count = 4;
		break;
	case X86_BOOT_ALT: /* bootloader */
		ops->write = dfu_forbidden_transfer_we;
		ops->read = dfu_flash_read;
		ops->block_count = flash_blocks[ops->alternate].block_count;
		break;
	case X86_APP_ALT: /* quark.bin */
	case FACT_NONPERSISTENT_ALT:
	case PANIC_ALT:
	case FACT_PERSISTENT_ALT:
	case DATA_ALT:
	case SENSOR_CORE_ALT: /* arc.bin */
		ops->write = dfu_flash_write;
		ops->read = dfu_flash_read;
		ops->block_count = flash_blocks[ops->alternate].block_count;
		break;
	case BLE_CORE_ALT:
		/* ble_core */
		ops->write = dfu_swd_write;
		ops->read = dfu_swd_read;
		ops->block_count = (BLE_CORE_FLASH_SIZE / PAGE_SIZE);
		break;
#if defined(CONFIG_SPI_FLASH)
	case SNOR_ALT:  /* snor */
		spi_flash = get_spi_flash();
		ops->write = dfu_spi_flash_write;
		ops->read = dfu_spi_flash_read;
		ops->block_count = (spi_flash->mem_size / PAGE_SIZE);
		ops->partition_offset = 0;
		break;
#if defined(CONFIG_CACHE_PARTITION)
	case CACHE_ALT: /* cache */
		ops->write = dfu_spi_flash_write;
		ops->read = dfu_spi_flash_read;
		ops->block_count =
			(SPI_FOTA_NB_BLOCKS *
			 SERIAL_FLASH_BLOCK_SIZE) / PAGE_SIZE;
		ops->partition_offset = SPI_FOTA_START_BLOCK *
					SERIAL_FLASH_BLOCK_SIZE;
		break;
#endif
#if defined(CONFIG_LOG_PARTITION)
	case LOG_ALT:   /* log */
		ops->write = dfu_spi_flash_write;
		ops->read = dfu_spi_flash_read;
		ops->block_count =
			(SPI_LOG_NB_BLOCKS *
			 SERIAL_FLASH_BLOCK_SIZE) / PAGE_SIZE;
		ops->partition_offset = SPI_LOG_START_BLOCK *
					SERIAL_FLASH_BLOCK_SIZE;
		break;
#endif
#if defined(CONFIG_SPI_FLASH_APP_DATA_PARTITION)
	case APP_DATA_ALT:
		ops->write = dfu_spi_flash_write;
		ops->read = dfu_spi_flash_read;
		ops->block_count =
			(SPI_APPLICATION_DATA_NB_BLOCKS *
			 SERIAL_FLASH_BLOCK_SIZE) / PAGE_SIZE;
		ops->partition_offset = SPI_APPLICATION_DATA_START_BLOCK *
					SERIAL_FLASH_BLOCK_SIZE;
		break;
#endif
#if defined(CONFIG_SPI_FLASH_SYS_EVENT_PARTITION)
	case SYS_EVENT_ALT:
		ops->write = dfu_spi_flash_write;
		ops->read = dfu_spi_flash_read;
		ops->block_count =
			(SPI_SYSTEM_EVENT_NB_BLOCKS *
			 SERIAL_FLASH_BLOCK_SIZE) / PAGE_SIZE;
		ops->partition_offset = SPI_SYSTEM_EVENT_START_BLOCK *
					SERIAL_FLASH_BLOCK_SIZE;
		break;
#endif
#if defined(CONFIG_SPI_FLASH_USER_DATA_PARTITION) && \
		defined(SPI_USER_DATA_NB_BLOCKS)
	case USER_DATA_ALT:
		ops->write = dfu_spi_flash_write;
		ops->read = dfu_spi_flash_read;
		ops->block_count =
			(SPI_USER_DATA_NB_BLOCKS *
			 SERIAL_FLASH_BLOCK_SIZE) / PAGE_SIZE;
		ops->partition_offset = SPI_USER_DATA_START_BLOCK *
					SERIAL_FLASH_BLOCK_SIZE;
		break;
#endif
#endif
	default:
		ops->write = dfu_forbidden_transfer_we;
		ops->read = dfu_forbidden_transfer_r;
		ops->block_count = 0;
		ops->partition_offset = 0;
		break;
	}
}
#endif // CONFIG_USB && CONFIG_USB_DFU && CONFIG_DNX

#if defined(CONFIG_I2C)
void  i2c_init(void)
{
	MMIO_REG_VAL(PERIPH_CLK_GATE_CTRL) |= I2C0_CLK_GATE_MASK;

	MMIO_REG_VAL(PERIPH_CLK_GATE_CTRL) |= I2C1_CLK_GATE_MASK;

	i2c_cfg_data_t i2c_cfg;

	i2c_cfg.speed = I2C_SLOW;
	i2c_cfg.addressing_mode = I2C_7_Bit;
	i2c_cfg.mode_type = I2C_MASTER;

	i2c_set_config(SOC_I2C_0_BASE, &i2c_cfg);
}
#endif

#if defined(CONFIG_SYSTEM_EVENTS)
void board_misc_init(void)
{
	system_events_init();
}
#endif

#if defined(CONFIG_LP5562)
__weak void notify_boot()
{
	lp5562_notify_boot();
}
#endif

uint32_t get_uptime_32k(void)
{
	return SCSS_REG_VAL(SCSS_AONC_CNT);
}

uint32_t get_uptime_ms(void)
{
	return ((uint64_t)SCSS_REG_VAL(SCSS_AONC_CNT) * 1000) / 32768;
}

static uint32_t epoch_time_ref = 0;
static uint32_t uptime_ms_ref = 0;

void set_time(uint32_t epoch_time)
{
	uptime_ms_ref = get_uptime_ms();
	epoch_time_ref = epoch_time;
}

uint32_t uptime_to_epoch(uint32_t uptime_ms)
{
	return epoch_time_ref + ((int32_t)(uptime_ms - uptime_ms_ref) / 1000);
}

uint32_t time(void)
{
	return uptime_to_epoch(get_uptime_ms());
}

void boot_restore_settings(enum wake_sources	wake_source,
			   enum reset_reasons	reset_reason,
			   enum boot_targets	__unused(boot_target))
{
	/* Non persistent factory data */
	soc_flash_block_erase(FACTORY_RESET_NON_PERSISTENT_START_BLOCK,
			      FACTORY_RESET_PERSISTENT_NB_BLOCKS);
	/* Application data */
	soc_flash_block_erase(APPLICATION_DATA_START_BLOCK,
			      APPLICATION_DATA_NB_BLOCKS);
#if defined(CONFIG_SWD)
	/* ble data */
	swd_init();
	swd_page_erase(BLE_PART_DEVICE_MANAGER_START);
#endif
#if defined(CONFIG_SPI_FLASH)
	/* erase spi flash partitions */
	spi_flash_info_t *spi_flash = get_spi_flash();
	/* Fota partition */
	spi_flash_sector_erase(spi_flash, SPI_FOTA_START_BLOCK,
			       SPI_FOTA_NB_BLOCKS);
	/* Activity data partition */
	spi_flash_sector_erase(spi_flash, SPI_APPLICATION_DATA_START_BLOCK,
			       SPI_APPLICATION_DATA_NB_BLOCKS);
	/* System Event partition */
	spi_flash_sector_erase(spi_flash, SPI_SYSTEM_EVENT_START_BLOCK,
			       SPI_SYSTEM_EVENT_NB_BLOCKS);
#endif

#if defined(CONFIG_SWD)
	swd_debug_mode_reset_to_normal();
#endif
	boot_main(wake_source, reset_reason, TARGET_RESTORE_SETTINGS);
}
