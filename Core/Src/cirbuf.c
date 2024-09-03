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

float ApplyFirstDifferenceLPF(float alpha, float one_minus_alpha, float x_0, float y_minus_1)
{
	// Reference https://neilfoxman.com/?page_id=1714#First_Difference_Lowpass_Filter
	return one_minus_alpha * y_minus_1 + alpha * x_0;
}

float ApplyFirstDifferenceEnvelopeLPF(float alpha, float one_minus_alpha, float x_0, float y_minus_1)
{
	float y_lpf = ApplyFirstDifferenceLPF(alpha, one_minus_alpha, x_0, y_minus_1);
	if (x_0 > y_lpf)
	{
		return x_0;
	}
	else
	{
		return y_lpf;
	}
}

float ApplyFirstDifferenceHPF(float one_minus_alpha, float x_0, float x_minus_1, float y_minus_1)
{
	// Reference https://neilfoxman.com/?page_id=1714#First_Difference_Highpass_Filter
	return one_minus_alpha * (y_minus_1 + x_0 - x_minus_1);
}
