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

#include <sign.h>
#include <bootlogic.h>
#include <partition.h>

#define SSBL_BASE_ADDRESS 0x40000000

void boot(enum wake_sources	__unused(wake_source),
	  enum reset_reasons	__unused(reset_reason),
	  enum boot_targets	__unused(boot_target))
{
}

#if !defined(CONFIG_BOOT_SSBL_DISABLED)
__noreturn void boot_logic(void)
{
	uint32_t addr = SSBL_BASE_ADDRESS;
	uint8_t *sig = (uint8_t *)addr;
	int secure = 0;


#if defined(CONFIG_SECURE_BOOT)
	if (sig[0] == '$' && sig[1] == 'S' && sig[2] == 'I' && sig[3] == 'G') {
#if !defined(CONFIG_SECURE_BOOT_SSBL)
		addr += CONFIG_SIGNATURE_HEADER_SIZE + VERSION_HEADER_SIZE;
#endif
	} else {
		addr += VERSION_HEADER_SIZE;
	}

#if defined(CONFIG_SECURE_BOOT_SSBL)
	secure = secure_boot(SSBL_BASE_ADDRESS);
#else
	secure = 1;
#endif
#endif

	if (secure == 1)
		asm volatile ("jmp *%0" : : "r" (addr));

#if defined(CONFIG_DNX)
	dnx();
#endif

	__builtin_unreachable();
}
#else

__noreturn void boot_logic(void)
{
	die();
}
#endif
