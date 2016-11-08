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

#ifndef __PART_HELPERS_H_
#define __PART_HELPERS_H_

/* Some helpers to define the list of partitions */
#define OFFSET_BEGINNING 0
#define OFFSET_AUTO 0xdeadbeefdeadbeef
#define KiB 1024
#define MiB 1048576
#define GiB 1073741824
#define define_partition(partition, pname, pstorage, pstart, psize, pattributes) \
	{ \
		.partition_id = partition, \
		.name = pname, \
		.storage = pstorage, \
		.start = pstart, \
		.size = psize, \
		.attributes = pattributes \
	},

#define partition_new_size(partition, psize) \
	{ \
		.partition_id = partition, \
		.storage = 0, \
		.start = 0, \
		.size = psize, \
		.attributes = 0 \
	},

#define declare_device(id) \
	struct storage_device id ## _device __section(".table_partition")

#define bind_to_device(id) &id ## _device

#endif /* __PART_HELPERS_H_ */
