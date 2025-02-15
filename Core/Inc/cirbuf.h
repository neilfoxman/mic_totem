/*
 * cirbuf.h
 *
 *  Created on: Jul 24, 2024
 *      Author: NeilF
 */

#ifndef INC_CIRBUF_H_
#define INC_CIRBUF_H_

#include <stdint.h>

uint8_t ArrayIdxToCirBufIdx(
		uint8_t start_idx,
		int8_t array_idx,
		uint8_t cirbuf_len);

int32_t ApplyFirstDifferenceLPF(int32_t y_minus_1, int32_t x_0, int32_t tau_over_T, int32_t gain);

int32_t ApplyFIRLPF(int32_t* x_cirbuf, int32_t cirbuf_idx_start, uint32_t filt_len, uint32_t cirbuf_size, int32_t* p_accum);

int32_t ApplyFirstDifferenceEnvelopeLPF(int32_t y_minus_1, int32_t x_0, int32_t tau_over_T, int32_t gain);

int32_t ApplyFirstDifferenceEnvelopeLinear(int32_t y_minus_1, int32_t x_0, int32_t gain);

float ApplyFirstDifferenceLPF_float(float y_minus_1, float x_0, float tau_over_T);

float ApplyFirstDifferenceEnvelopeLPF_float(float y_minus_1, float x_0, float tau_over_T);

int32_t ApplyFirstDifferenceHPF(int32_t y_minus_1, int32_t x_0, int32_t x_minus_1, int32_t tau_over_T, int32_t gain);

float ApplyFirstDifferenceHPF_float(float y_minus_1, float x_0, float x_minus_1, float tau_over_T);

int32_t CalcTauOverTFromFloat(float f_n, float f_s);

#endif /* INC_CIRBUF_H_ */
