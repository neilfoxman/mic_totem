/*
 * cirbuf.h
 *
 *  Created on: Jul 24, 2024
 *      Author: NeilF
 */

#ifndef INC_CIRBUF_H_
#define INC_CIRBUF_H_

#include <stdint.h>

float ArrayIdxToCirBufIdx(
		uint8_t start_idx,
		int8_t array_idx,
		uint8_t cirbuf_len);

float ApplyFirstDifferenceLPF(float alpha, float one_minus_alpha, float x_0, float y_minus_1);

float ApplyFirstDifferenceEnvelopeLPF(float alpha, float one_minus_alpha, float x_0, float y_minus_1);

float ApplyFirstDifferenceHPF(float one_minus_alpha, float x_0, float x_minus_1, float y_minus_1);

#endif /* INC_CIRBUF_H_ */
