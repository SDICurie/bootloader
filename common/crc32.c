/*
 * Copyright (c) 2014, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer
 *      in the documentation and/or other materials provided with the
 *      distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "crc.h"

#define CRC32_POLYNOMIAL 0xedb88320UL

uint32_t calc_crc32(uint32_t crcinit, const void *buf, size_t size,
		    uint32_t (*read_byte)(uint32_t, uint8_t *))
{
	int i;
	uint32_t crc, mask;
	const uint8_t *p;
	uint32_t byte = 0;

	p = buf;

	crc = crcinit ^ ~0U;
	while (size--) {
		/* if read_byte function is not given, read from memory */
		if (!read_byte)
			byte = *p;
		else
		if (0 != read_byte((uint32_t)p, (uint8_t *)&byte))
			return -1;
		p++;
		crc = crc ^ byte;
		for (i = 8; i > 0; i--) {
			mask = -(crc & 1);
			crc = (crc >> 1) ^ (CRC32_POLYNOMIAL & mask);
		}
	}
	return crc ^ ~0U;
}
