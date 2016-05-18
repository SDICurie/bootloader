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

#include <serial/xmodem.h>
#include <utils.h>
#include "crc.h"

static void xmodem_abort(int __unused((*p_fgetc))(int), void (*p_fputc)(int))
{
	int i;

	for (i = 0; i < 3; i++) {
		p_fputc(XMODEM_CAN);
	}
}

static bool xmodem_check(const unsigned char *p_buffer, int p_size)
{
	unsigned short crc16_calc = calc_crc16_ccitt(p_buffer, p_size);
	unsigned short crc16_read =
		(p_buffer[p_size] << 8) + p_buffer[p_size + 1];

	if (crc16_calc == crc16_read)
		return true;
	return false;
}

static struct xmodem_ops xmodem_ops = {
	.sync_char = 'C',
	.sequence = 1
};

int xmodem_receive(unsigned char *p_dest, int p_dest_size, int (*p_fgetc)(int),
		   void (*p_fputc)(int))
{
	int char_recv = 0;
	int len_recv = 0;
	char error_count = 0;
	int size_buffer = 0;
	const int packet_size = CONFIG_XMODEM_PACKET_SIZE;      /* 128 or 1024 */
	/* soh/stx + seq + ~seq + (1024 or 128) bytes data + 2 bytes crc */
	unsigned char buffer[CONFIG_XMODEM_PACKET_SIZE + 5];
	int retrans = 25;

	int i;

	if (xmodem_ops.sync_char == 0)
		p_fputc(XMODEM_ACK);

	while (1) {
		while (error_count < XMODEM_NBRETRY) {
			if (xmodem_ops.sync_char)
				p_fputc(xmodem_ops.sync_char);

			char_recv = p_fgetc(300);
			switch (char_recv) {
#if CONFIG_XMODEM_PACKET_SIZE == 1024
			case XMODEM_STX:        //xmodem 1024bytes
				goto recv_begin;
				break;
#elif CONFIG_XMODEM_PACKET_SIZE == 128
			case XMODEM_SOH:        //xmodem 128bytes
				goto recv_begin;
				break;
#else
#error "set CONFIG_XMODEM_PACKET_SIZE = 128 or 1024"
#endif
			case XMODEM_EOT:
				p_fputc(XMODEM_ACK);
				return len_recv;
				break;

			case XMODEM_CAN:
				if (p_fgetc(1000) == XMODEM_CAN)
					return -1;
				break;

			default:
				break;
			}

			error_count++;
			if (error_count >= XMODEM_NBRETRY) {
				xmodem_abort(p_fgetc, p_fputc);
				return -2;
			}
		}

recv_begin:
		xmodem_ops.sync_char = 0;
		size_buffer = 3 + packet_size + 1 + 1;
		buffer[0] = char_recv;
		for (i = 1; i < size_buffer; i++) {
			char_recv = p_fgetc(1000);
			if (char_recv < 0)
				goto fail;
			buffer[i] = char_recv;
		}

		if (buffer[1] == (unsigned char)(~buffer[2]) &&
		    (buffer[1] == xmodem_ops.sequence
		     || buffer[1] == (unsigned char)xmodem_ops.sequence - 1)
		    && xmodem_check(&buffer[3], packet_size)) {
			if (buffer[1] == xmodem_ops.sequence) {
				register int count = p_dest_size - len_recv;
				if (count > packet_size)
					count = packet_size;

				for (i = 0; i < count; i++) {
					p_dest[len_recv + i] = buffer[3 + i];
				}
				len_recv += count;

				++xmodem_ops.sequence;
				retrans = 25 + 1;
				if (p_dest_size == len_recv)
					return len_recv;
			}
			if (--retrans <= 0) {
				while (p_fgetc(100) > 0) ;
				xmodem_abort(p_fgetc, p_fputc);
				return -3;      /* too many retry error */
			}
			p_fputc(XMODEM_ACK);
			continue;
		}

fail:
		while (p_fgetc(100) > 0) ;
		p_fputc(XMODEM_NAK);
	}
}
