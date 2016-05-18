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

#include <scss_registers.h>

void qrk_cxxxx_wdt_clk_enable(void)
{
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
			       QRK_SCSS_PERIPH_CFG0_OFFSET) |=
		QRK_SCSS_PERIPH_CFG0_WDT_ENABLE;
}

void qrk_cxxxx_wdt_gating_enable()
{
	MMIO_REG_VAL(QRK_CLKGATE_CTRL) |= QRK_CLKGATE_CTRL_WDT_ENABLE;
}

void watchdog_reset(void)
{
	/* This register is used to restart the WDT
	 *  counter. as a safetly feature to pervent
	 *  accidenctal restarts the value 0x76 must be
	 *  written. A restart also clears the WDT
	 *  interrupt.*/
	MMIO_REG_VAL_FROM_BASE(QRK_WDT_BASE_ADDR,
			       QRK_WDT_CRR) = QRK_WDT_CRR_VAL;
}

void watchdog_init(void)
{
	qrk_cxxxx_wdt_gating_enable();
}

void watchdog_start(uint16_t period_sec)
{
	/* Enables the clock for the peripheral watchdog */
	qrk_cxxxx_wdt_clk_enable();

	/*  Set timeout value
	 *  [7:4] TOP_INIT - the initial timeout value is hardcoded in silicon,
	 *  only bits [3:0] TOP are relevant.
	 *  Once tickled TOP is loaded at the next expiration.
	 */
	uint32_t i;
	uint32_t ref = (1 << 16) / (CONFIG_CLOCK_SPEED / 1000); /* 2^16/FREQ_CPU */
	uint32_t timeout = period_sec * 1000;
	for (i = 0; i < 16; i++) {
		if (timeout <= ref) break;
		ref = ref << 1;
	}
	if (i > 15) {
		i = 15;
	}
	MMIO_REG_VAL_FROM_BASE(QRK_WDT_BASE_ADDR, QRK_WDT_TORR) = i;

	/* Response mode: RESET  */
	MMIO_REG_VAL_FROM_BASE(QRK_WDT_BASE_ADDR,
			       QRK_WDT_CR) &= ~QRK_WDT_CR_INT_ENABLE;

	/* Enable WDT, cannot be disabled until soc reset */
	MMIO_REG_VAL_FROM_BASE(QRK_WDT_BASE_ADDR,
			       QRK_WDT_CR) |= QRK_WDT_CR_ENABLE;

	watchdog_reset();
}
