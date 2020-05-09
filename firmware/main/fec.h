/*
 *  AX5043 OS-independent driver
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



#include <stdint.h>
#include "rs_utils.h"
#include <stddef.h>

#define CCSDS_CONV_CODE_RATE (1/2)
#define CCSDS_CONV_CONSTRAINT_LENGTH_BITS (7)
#define DATA_ARRAY_LEN (256)

#ifdef __cplusplus
extern "C" {
#endif 
void
rs_encode(uint8_t *parity, const uint8_t *data, size_t len);

int
rs_decode(const uint8_t *data, size_t len);

void
conv_encoder_1_2_7(uint8_t *out, const uint8_t *data, size_t len);

void
conv_encoder_2_3_7(uint8_t *out, const uint8_t *data, size_t len);

void
conv_encoder_3_4_7(uint8_t *out, const uint8_t *data, size_t len);

void
conv_encoder_5_6_7(uint8_t *out, const uint8_t *data, size_t len);

void
conv_encoder_7_8_7(uint8_t *out, const uint8_t *data, size_t len);

#ifdef __cplusplus
  }
#endif 

