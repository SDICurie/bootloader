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

#include "api/part-helpers.h"

int partitions_init(void)
{
	int i;
	int j;

	/* Override partition sizes */
	for (i = 0; i < partition_resize_count; i++) {
		for (j = 1; j < partition_count; j++) {
			if (partition_resizes[i].partition_id ==
			    partitions[j].partition_id) {
				partitions[j].size =
					partition_resizes[i].size;
				break;
			}
		}
	}

	for (i = 1; i < partition_count; i++) {
		if (partitions[i].start == OFFSET_AUTO) {
			/* Check this is not the first partition of the storage */
			if (partitions[i - 1].storage !=
			    partitions[i].storage)
				return 1;
			partitions[i].start =
				partitions[i - 1].start + partitions[i - 1].size;
		}
	}

	return 0;
}

struct partition *get_partition(uint8_t id)
{
	int i;

	for (i = 1; i < partition_count; i++) {
		if (partitions[i].partition_id == id) {
			return &partitions[i];
		}
	}
	return 0;
}
