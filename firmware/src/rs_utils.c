/*
 * Copyright 2003 Phil Karn, KA9Q
 * May be used under the terms of the GNU Lesser General Public License (LGPL)
 */

/*  AX5043 OS-independent driver
 *
 *  Copyright (C) 2019 Libre Space Foundation (https://libre.space)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rs_utils.h"

uint8_t pad = 0, u, q, tmp, num1, num2, den, temp_r;
uint8_t l_arr[PRT_LENGTH + 1], s[PRT_LENGTH], *eras_pos = 0, no_eras = 0;
uint8_t b[PRT_LENGTH + 1], t[PRT_LENGTH + 1], evaluator[PRT_LENGTH + 1];
uint8_t root[PRT_LENGTH], reg[PRT_LENGTH + 1], loc[PRT_LENGTH];
int16_t l_val, conf_2, eval_val, i, j, conf_1, k;
int16_t syndrome_flag, corrected_bits;

static int
mod255(int16_t x)
{
	return (x % 255);
}

void
clear_values()
{
	*eras_pos = 0;
	no_eras = 0;
	pad = 0;
	u = 0;
	q = 0;
	tmp = 0;
	num1 = 0;
	num2 = 0;
	den = 0;
	temp_r = 0;
	l_val = 0;
	conf_2 = 0;
	eval_val = 0;
	i = 0;
	j = 0;
	conf_1 = 0;
	k = 0;
	syndrome_flag = 0;
	corrected_bits = 0;
}

static bool
pad_valid(uint8_t pad)
{
	if (pad < 0 || pad > 222) {
		return false;
	} else {
		return true;
	}
}

void
err_root_finder()
{
	memcpy(&reg[1], &l_arr[1], PRT_LENGTH * sizeof(reg[0]));
	corrected_bits = 0;
	for (i = 1, k = IPRIM - 1; i <= NN; i++, k = MODFF(k + IPRIM)) {
		q = 1;
		for (j = l_val; j > 0; j--)
			if (reg[j] != FF) {
				reg[j] = MODFF(reg[j] + j);
				q ^= alpha_to[reg[j]];
			}

		if (q != 0) {
			continue;
		}
		root[corrected_bits] = i;
		loc[corrected_bits] = k;
		if (++corrected_bits == l_val) {
			break;
		}
	}
}

void
generate_syndromes(uint8_t *data)
{
	for (i = 0; i < PRT_LENGTH; i++) {
		s[i] = data[0];
	}
	for (j = 1; j < NN - pad; j++)
		for (i = 0; i < PRT_LENGTH; i++) {
			if (s[i] == 0) {
				s[i] = data[j];
			} else {
				s[i] = data[j] ^ alpha_to[MODFF(index_of[s[i]] + (FCR + i) * PRIM)];
			}
		}
}

void
transform_syndromes()
{
	for (i = 0; i < PRT_LENGTH; i++) {
		syndrome_flag |= s[i];
		s[i] = index_of[s[i]];
	}
}

void
l_arr_transform()
{
	l_val = 0;
	for (i = 0; i < PRT_LENGTH + 1; i++) {
		l_arr[i] = index_of[l_arr[i]];
		if (l_arr[i] != FF) {
			l_val = i;
		}
	}
}

void
err_locator()
{
	conf_1 = conf_2 = 0;
	while (++conf_1 <= PRT_LENGTH) {
		temp_r = 0;
		for (i = 0; i < conf_1; i++)
			if ((l_arr[i] != 0) && (s[conf_1 - i - 1] != FF)) {
				temp_r ^= alpha_to[MODFF(index_of[l_arr[i]] + s[conf_1 - i - 1])];
			}
		temp_r = index_of[temp_r];
		if (temp_r == FF) {
			memmove(&b[1], b, PRT_LENGTH * sizeof(b[0]));
			b[0] = FF;
		} else {
			t[0] = l_arr[0];
			for (i = 0; i < PRT_LENGTH; i++) {
				if (b[i] != FF) {
					t[i + 1] = l_arr[i + 1] ^ alpha_to[MODFF(temp_r + b[i])];
				} else {
					t[i + 1] = l_arr[i + 1];
				}
			}
			if (2 * conf_2 <= conf_1 + no_eras - 1) {
				conf_2 = conf_1 + no_eras - conf_2;

				for (i = 0; i <= PRT_LENGTH; i++) {
					b[i] = (l_arr[i] == 0) ? FF : MODFF(index_of[l_arr[i]] - temp_r + NN);
				}
			} else {
				memmove(&b[1], b, PRT_LENGTH * sizeof(b[0]));
				b[0] = FF;
			}
			memcpy(l_arr, t, (PRT_LENGTH + 1) * sizeof(t[0]));
		}
	}
}

void
err_evaluator()
{
	eval_val = l_val - 1;
	for (i = 0; i <= eval_val; i++) {
		tmp = 0;
		for (j = i; j >= 0; j--) {
			if ((s[i - j] != FF) && (l_arr[j] != FF)) {
				tmp ^= alpha_to[MODFF(s[i - j] + l_arr[j])];
			}
		}
		evaluator[i] = index_of[tmp];
	}
}

void
err_val_calculator(uint8_t *data)
{

	for (j = corrected_bits - 1; j >= 0; j--) {
		num1 = 0;
		for (i = eval_val; i >= 0; i--) {
			if (evaluator[i] != FF) {
				num1 ^= alpha_to[MODFF(evaluator[i] + i * root[j])];
			}
		}
		num2 = alpha_to[MODFF(root[j] * (FCR - 1) + NN)];
		den = 0;

		for (i = min(l_val, PRT_LENGTH - 1) & ~1; i >= 0; i -= 2) {
			if (l_arr[i + 1] != FF) {
				den ^= alpha_to[MODFF(l_arr[i + 1] + i * root[j])];
			}
		}

		if (num1 != 0 && loc[j] >= pad) {
			data[loc[j] - pad] ^= alpha_to[MODFF(index_of[num1] + index_of[num2] + NN -
			                                     index_of[den])];
		}
	}
}

void
rs_utils_encode(uint8_t *parity, uint8_t *data, size_t len)
{
	memset(parity, 0, sizeof(uint8_t) * PRT_LENGTH);

	uint8_t pad = 223 - len, feedback;

	for (i = 0; i < NN - PRT_LENGTH - pad; i++) {
		feedback = index_of[data[i] ^ parity[0]];
		if (feedback != FF) {
			for (j = 1; j < PRT_LENGTH; j++) {
				parity[j] ^= alpha_to[mod255(feedback + genpoly[PRT_LENGTH - j])];
			}
		}
		memmove(&parity[0], &parity[1], sizeof(uint8_t) * (PRT_LENGTH - 1));
		if (feedback != FF) {
			parity[PRT_LENGTH - 1] = alpha_to[mod255(feedback + genpoly[0])];
		} else {
			parity[PRT_LENGTH - 1] = 0;
		}
	}
}

int
rs_utils_decode(uint8_t *data, size_t len)
{
	syndrome_flag = 0;
	pad = 223 - len;

	if (!pad_valid(pad)) {
		return (-1);
	}

	generate_syndromes(&data[0]);

	transform_syndromes();

	if (syndrome_flag == 0) {
		return 0;
	}

	memset(&l_arr[1], 0, PRT_LENGTH * sizeof(l_arr[0]));

	l_arr[0] = 1;
	for (i = 0; i < PRT_LENGTH + 1; i++) {
		b[i] = index_of[l_arr[i]];
	}

	err_locator();

	l_arr_transform();

	err_root_finder();

	if (l_val != corrected_bits) {
		return 0;
	}

	err_evaluator();

	err_val_calculator(&data[0]);

	return corrected_bits;
}
