/*
 * cirbuf.c
 *
 *  Created on: Jul 24, 2024
 *      Author: NeilF
 */

#include "cirbuf.h"

float ArrayIdxToCirBufIdx(
		uint8_t start_idx,
		int8_t array_idx,
		uint8_t cirbuf_len)
{
	// Allow Negative indexes (with some overhead)
	while (array_idx < 0){
		array_idx += cirbuf_len;
	}
	return (start_idx + array_idx) % cirbuf_len;
}
