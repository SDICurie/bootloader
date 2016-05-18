#ifndef __SECURITY_H__
#define __SECURITY_H__

#include <partition.h>
#include <stdint.h>

#include <uECC.h>
#include <sha256.h>

struct security_interface {
	int (*uECC_verify_f)(const uint8_t	public_key[uECC_BYTES * 2],
			     const uint8_t	hash[uECC_BYTES],
			     const uint8_t	signature[uECC_BYTES * 2]);
	unsigned char * (*sha256_f)(const unsigned char *d, size_t n,
				    unsigned char *md);
	void (*sha256_init_f)(sha256_context_t *ctx);
	void (*sha256_update_f)(sha256_context_t *ctx, const void *in,
				size_t len);
	void (*sha256_final)(unsigned char digest[32], sha256_context_t *ctx);
};

#define static_security_interface ((struct security_interface *) \
				   BOOTLOADER_SHARED_SECURITY)

#endif /* __SECURITY_H__ */
