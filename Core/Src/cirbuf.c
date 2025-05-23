/*
 * cirbuf.c
 *
 *  Created on: Jul 24, 2024
 *      Author: NeilF
 */

#include "cirbuf.h"

float pi = 3.1415;


uint8_t ArrayIdxToCirBufIdx(
		uint8_t start_idx,
		int8_t array_idx,
		uint8_t cirbuf_len)
{
	// Allow Negative indexes (with some overhead)
	if (array_idx < 0){
		array_idx += cirbuf_len;
	}
	return (start_idx + array_idx) % cirbuf_len;
}

int32_t ApplyFirstDifferenceLPF(int32_t y_minus_1, int32_t x_0, int32_t tau_over_T, int32_t gain)
{
	/*
	 * NOTE: Returns
	 */
	// Reference https://neilfoxman.com/?page_id=1714#First_Difference_Lowpass_Filter
	return (tau_over_T * y_minus_1 + gain * x_0)/(tau_over_T + 1);
}

int32_t ApplyFIRLPF(int32_t* x_cirbuf, int32_t cirbuf_idx_start, uint32_t filt_len, uint32_t cirbuf_size, int32_t* p_accum)
{
	// Get value of accumulator
	int32_t v_accum = (int32_t)*p_accum;

	// Add most recent sample to accumulator
	v_accum += x_cirbuf[cirbuf_idx_start];

	// Subtract last (now outdated) sample from the accumulator.
	int32_t offset_idx =  cirbuf_idx_start - filt_len;
	if (offset_idx < 0){
		offset_idx += cirbuf_size;
	}
	v_accum -= x_cirbuf[offset_idx];

	// Assign new value to accumulator
	*p_accum = v_accum;
	return v_accum / filt_len;
}

int32_t ApplyFirstDifferenceEnvelopeLPF(int32_t y_minus_1, int32_t x_0, int32_t tau_over_T, int32_t gain)
{
	int32_t x_0_gain = x_0 * gain;
	int32_t y_lpf = ApplyFirstDifferenceLPF(y_minus_1, x_0, tau_over_T, gain);
	if (x_0_gain > y_lpf)
	{
		return x_0_gain;
	}
	else
	{
		return y_lpf;
	}
}

int32_t ApplyFirstDifferenceEnvelopeLinear(int32_t y_minus_1, int32_t x_0, int32_t gain)
{
	int32_t x_0_gain = x_0 * gain;
	int32_t y_lin = y_minus_1 - 1;
	if ( x_0_gain > y_lin)
	{
		return x_0_gain;
	}
	else
	{
		return y_lin;
	}
}

float ApplyFirstDifferenceLPF_float(float y_minus_1, float x_0, float tau_over_T)
{
	// Reference https://neilfoxman.com/?page_id=1714#First_Difference_Lowpass_Filter
	return (tau_over_T * y_minus_1 + x_0)/(tau_over_T + 1.0);
}

float ApplyFirstDifferenceEnvelopeLPF_float(float y_minus_1, float x_0, float tau_over_T)
{
	float y_lpf = ApplyFirstDifferenceLPF_float(y_minus_1, x_0, tau_over_T);
	float x_0_float = (float)x_0;
	if (x_0_float > y_lpf)
	{
		return x_0_float;
	}
	else
	{
		return y_lpf;
	}
}

int32_t ApplyFirstDifferenceHPF(int32_t y_minus_1, int32_t x_0, int32_t x_minus_1, int32_t tau_over_T, int32_t gain)
{
	// Reference https://neilfoxman.com/?page_id=1714#First_Difference_Highpass_Filter
	return (tau_over_T * (y_minus_1 + gain * (x_0 - x_minus_1) ) ) / (tau_over_T + 1);
}

float ApplyFirstDifferenceHPF_float(float y_minus_1, float x_0, float x_minus_1, float tau_over_T)
{
	// Reference https://neilfoxman.com/?page_id=1714#First_Difference_Highpass_Filter
	return (float)(tau_over_T*(y_minus_1 + x_0 - x_minus_1)) / (tau_over_T + 1);
}

int32_t CalcTauOverTFromFloat(float f_n, float f_s)
{
	return (int32_t)( f_s / (2 * pi * f_n) );
}
