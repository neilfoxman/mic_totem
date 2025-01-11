/*
 * cirbuf.c
 *
 *  Created on: Jul 24, 2024
 *      Author: NeilF
 */

#include "cirbuf.h"

float pi = 3.1415;


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

int32_t ApplyFirstDifferenceLPF(int32_t y_minus_1, int32_t x_0, int32_t tau_over_T, int32_t gain)
{
	/*
	 * NOTE: Returns
	 */
	// Reference https://neilfoxman.com/?page_id=1714#First_Difference_Lowpass_Filter
	return (tau_over_T * y_minus_1 + gain * x_0)/(tau_over_T + 1);
}

int32_t ApplyFirstDifferenceEnvelopeLPF(int32_t y_minus_1, int32_t x_0, int32_t tau_over_T, int32_t gain)
{
	int32_t y_lpf = ApplyFirstDifferenceLPF(y_minus_1, x_0, tau_over_T, gain);
	if (x_0 > y_lpf)
	{
		return x_0;
	}
	else
	{
		return y_lpf;
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
