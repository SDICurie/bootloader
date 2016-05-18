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

#include <api/security.h>

#include <utils.h>

#ifdef CONFIG_SHARE_CRYPTO
static volatile struct security_interface static_security_interface_section
__section(".security_shared_if") =
{
#if defined(CONFIG_CRYPTO_ECDSA)
	uECC_verify,
#else
	0,
#endif

#if defined(CONFIG_CRYPTO_SHA256)
	sha256, sha256_init, sha256_update, sha256_final
#else
	0, 0, 0, 0
#endif
};

#else

int uECC_verify(const uint8_t	public_key[uECC_BYTES * 2],
		const uint8_t	hash[uECC_BYTES],
		const uint8_t	signature[uECC_BYTES * 2])
{
	return static_security_interface->uECC_verify_f(public_key, hash,
							signature);
}

unsigned char *sha256(const unsigned char *d, size_t n, unsigned char *md)
{
	return static_security_interface->sha256_f(d, n, md);
}

void sha256_init(sha256_context_t *ctx)
{
	static_security_interface->sha256_init_f(ctx);
}

void sha256_update(sha256_context_t *ctx, const void *in, size_t len)
{
	static_security_interface->sha256_update_f(ctx, in, len);
}

void sha256_final(unsigned char digest[32], sha256_context_t *ctx)
{
	static_security_interface->sha256_final(digest, ctx);
}
#endif
