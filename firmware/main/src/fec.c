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

#include "fec.h"

uint8_t _data[DATA_ARRAY_LEN];

/**
 * Creates 32 byte parity of the Reed Solomon (255, 223)
 * @param parity a 32 byte buffer to hold the calculated  parity
 * @param data the input data
 * @param len the size of the input data. Normally this should be 223 but
 * in case of the available data are less, proper zero padding will be
 * considered
 */
void
rs_encode(uint8_t *parity, const uint8_t *data, size_t len)
{
	memset(&_data[0], 0, 256 * sizeof(uint8_t));
	memcpy(&_data[0], &data[0], len * sizeof(uint8_t));
	rs_utils_encode(&_data[223], &_data[0], 223);
	memcpy(&parity[0], &_data[223], 32 * sizeof(uint8_t));
}
/**
 * @param data RS encoded input data. The last 32 bytes should be the parity
 * bytes. Decoding is performed in place.
 * @param len the size of the input data (including the 32 parity bytes).
 * Normally this should be 255. If not, padding should be considered.
 * @return the number of corrected bits or -1 in case of unrecoverable decoding
 * error
 */
int
rs_decode(const uint8_t *data, size_t len)
{
	memset(&_data[0], 0, 256 * sizeof(uint8_t));
	memcpy(&_data[0], &data[0], (len - 32) * sizeof(uint8_t));
	memcpy(&_data[223], &data[len - 32], 32 * sizeof(uint8_t));
	int result = rs_utils_decode(&_data[0], 223);
	memcpy(&data[0], &_data[0], (len - 32) * sizeof(uint8_t));
	memcpy(&data[len - 32], &_data[223], 32 * sizeof(uint8_t));
	return result;
}

/* CCSDS polynomial functions */
static const uint8_t CCSDS_CONV_G1[7] = {1, 1, 1, 1, 0, 0, 1};
static const uint8_t CCSDS_CONV_G2[7] = {1, 0, 1, 1, 0, 1, 1};

/**
 * Static helper function to write punctured output to buffer. It is used for
 * all puncured codes except 2/3 that unrolling the code is feasible.
 *
 * @param out the out buffer that holds the result
 * @param data the input data
 * @param len size of input data in bytes
 * @param puncturing_pattern_c1 puncturing pattern for c1 polynomial function
 * @param puncturing_pattern_c2 puncturing pattern for c2 polynomial function
 * @param puncturing_pattern_length the length of the polynomials
 */
static void
calculate_punctured_output(uint8_t *out, const uint8_t *data, const size_t len,
                           const uint8_t *puncturing_pattern_c1,
                           const uint8_t *puncturing_pattern_c2,
                           const uint8_t puncturing_pattern_length);

/**
 * Static helper function to read specific bit from 1 byte length word.
 * Which bit is read is defined by the user.
 *
 * @param data the 1 byte length word to be read
 * @param input_bit the bit position to be read (starting from 0 for MS)
 *
 * @returns the bit value in an 1 byte word
 */
static inline uint8_t
read_bit(const uint8_t data, const uint8_t input_bit);

/**
 * Static helper function to calculate the output bit for given polynomial.
 *
 * @param shift_reg the shift register holding the bits (1 byte per bit)
 * @param polynomial the polynomial used for the calculation (1 byte per bit)
 *
 * @returns the out bit that was calculated
 */
static inline uint8_t
calculate_generator_output(const uint8_t *shift_reg, const uint8_t *polynomial);

/**
 * Virtual shift registers function used for conv_encoder_27.
 * @param reg array of registers
 * @param len size of array
 */
static inline void
bit_shift(uint8_t *reg, uint8_t len);

/**
 * CCSDS compliant K=7 R=1/2 convolutional encoder. Each call to this function
 * is completely independent from each other and there is no state information.
 * Initially the K-1 stages are set to 0. It is responsibility of the caller to
 * add tailbits or proper padding. With these assumptions, the size of the
 * encoded data will be always \f$2 \times len\f$
 *
 * @param out the out buffer to hold the result. It is responsibility of the
 * caller to provide enough space for the result
 *
 * @param data the input data
 * @param len the number of input bytes in the \a data buffer
 */
void
conv_encoder_1_2_7(uint8_t *out, const uint8_t *data, size_t len)
{
	uint8_t shift_reg_7[CCSDS_CONV_CONSTRAINT_LENGTH_BITS] = {0, 0, 0, 0, 0, 0, 0};
	uint8_t bit = 0;
	uint8_t out_g1 = 0;
	uint8_t out_g2 = 0;

	/* Clean output buffer from possible garbage value */
	*out = 0;

	for (int i = 0; i < len; i++) {
		/* Read 1st bit */
		bit = read_bit(data[i], 0);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 1st and 2nd bits */
		*out |= (((out_g1 & 1) << 7));
		*out |= ((!(out_g2 & 1) << 6));

		/* Read 2nd bit */
		bit = read_bit(data[i], 1);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 3rd and 4th bits */
		*out |= (((out_g1 & 1) << 5));
		*out |= ((!(out_g2 & 1) << 4));

		/* Read 3rd bit */
		bit = read_bit(data[i], 2);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 5th and 6th bits */
		*out |= (((out_g1 & 1) << 3));
		*out |= ((!(out_g2 & 1) << 2));

		/* Read 4th bit */
		bit = read_bit(data[i], 3);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 7th and 8th bits */
		*out |= (((out_g1 & 1) << 1));
		*out |= ((!(out_g2 & 1)));

		/* 8bits written to output move to next word */
		out++;
		*out = 0;

		/* Read 5th bit */
		bit = read_bit(data[i], 4);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 1st and 2nd bits */
		*out |= (((out_g1 & 1) << 7));
		*out |= ((!(out_g2 & 1) << 6));

		/* Read 6th bit */
		bit = read_bit(data[i], 5);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 3rd and 4th bits */
		*out |= (((out_g1 & 1) << 5));
		*out |= ((!(out_g2 & 1) << 4));

		/* Read 7th bit */
		bit = read_bit(data[i], 6);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 5th and 6th bits */
		*out |= (((out_g1 & 1) << 3));
		*out |= ((!(out_g2 & 1) << 2));

		/* Read 8th bit */
		bit = read_bit(data[i], 7);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 7th and 8th bits */
		*out |= (((out_g1 & 1) << 1));
		*out |= ((!(out_g2 & 1)));

		/* 8 bits written to output move to next word */
		out++;
		*out = 0;
	}
}

/**
 * CCSDS compliant K=7 R=2/3 convolutional encoder. Each call to this function
 * is completely independent from each other and there is no state information.
 * Initially the K-1 stages are set to 0. It is responsibility of the caller to
 * add tailbits or proper padding. With these assumptions, the size of the
 * encoded data will be always \f$2/3 \times len\f$
 *
 * @param out the out buffer to hold the result. It is responsibility of the
 * caller to provide enough space for the result
 *
 * @param data the input data
 * @param len the number of input bytes in the \a data buffer
 */
void
conv_encoder_2_3_7(uint8_t *out, const uint8_t *data, size_t len)
{
	/* Pancuring Pattern for 2/3 */
	/* C1: [1, 0] */
	/* C2: [1, 1] */
	uint8_t shift_reg_7[CCSDS_CONV_CONSTRAINT_LENGTH_BITS] = {0, 0, 0, 0, 0, 0, 0};
	uint8_t bit = 0;
	uint8_t out_g1 = 0;
	uint8_t out_g2 = 0;

	/* Clean output buffer from possible garbage value */
	*out = 0;

	for (int i = 0; i < len; i++) {
		/* Read 1st bit */
		bit = read_bit(data[i], 0);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 1st and 2nd bits */
		*out |= (((out_g1 & 1) << 7));
		*out |= (((out_g2 & 1) << 6));

		/* Read 2nd bit */
		bit = read_bit(data[i], 1);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G2 */
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 3rd bit */
		*out |= (((out_g2 & 1) << 5));

		/* Read 3rd bit */
		bit = read_bit(data[i], 2);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 4th and 5th bits */
		*out |= (((out_g1 & 1) << 4));
		*out |= (((out_g2 & 1) << 3));

		/* Read 4th bit */
		bit = read_bit(data[i], 3);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G2 */
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 6th bit */
		*out |= (((out_g2 & 1) << 2));

		/* Read 5th bit */
		bit = read_bit(data[i], 4);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 7th and 8th bits */
		*out |= ((out_g1 & 1) << 1);
		*out |= ((out_g2 & 1));

		/* 8bits written to output move to next word */
		out++;
		*out = 0;

		/* Read 6th bit */
		bit = read_bit(data[i], 5);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G2 */
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 1st bit */
		*out |= ((out_g2 & 1) << 7);

		/* Read 7th bit */
		bit = read_bit(data[i], 6);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 2nd and 3rd bits */
		*out |= (((out_g1 & 1) << 6));
		*out |= (((out_g2 & 1) << 5));

		/* Read 8th bit */
		bit = read_bit(data[i], 7);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G2 */
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 4th bit */
		*out |= (((out_g2 & 1) << 4));

		/* Read 8 bits from input move to next word */
		i++;

		/* Read 1st bit */
		bit = read_bit(data[i], 0);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 5th and 6th bits */
		*out |= (((out_g1 & 1) << 3));
		*out |= (((out_g2 & 1) << 2));

		/* Read 2nd bit */
		bit = read_bit(data[i], 1);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G2 */
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 7th bit */
		*out |= (((out_g2 & 1) << 1));

		/* Read 3rd bit */
		bit = read_bit(data[i], 2);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 8th bit */
		*out |= ((out_g1 & 1));

		/* 8 bits written to output move to next word */
		out++;
		*out = 0;

		/* Write convolution output 1st bit */
		*out |= (((out_g2 & 1) << 7));

		/* Read 4th bit */
		bit = read_bit(data[i], 3);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G2 */
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 2nd bit */
		*out |= (((out_g2 & 1) << 6));

		/* Read 5th bit */
		bit = read_bit(data[i], 4);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 3rd and 4th bit */
		*out |= (((out_g1 & 1) << 5));
		*out |= (((out_g2 & 1) << 4));

		/* Read 6th bit */
		bit = read_bit(data[i], 5);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G2 */
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 5th bit */
		*out |= (((out_g2 & 1) << 3));

		/* Read 7th bit */
		bit = read_bit(data[i], 6);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 6th and 7th bits */
		*out |= (((out_g1 & 1) << 2));
		*out |= (((out_g2 & 1) << 1));

		/* Read 8th bit */
		bit = read_bit(data[i], 7);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G2 */
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output 8th bit */
		*out |= (((out_g2 & 1)));

		/* 8 bits written to output move to next word */
		out++;
		*out = 0;
	}
}

/**
 * CCSDS compliant K=7 R=3/4 convolutional encoder. Each call to this function
 * is completely independent from each other and there is no state information.
 * Initially the K-1 stages are set to 0. It is responsibility of the caller to
 * add tailbits or proper padding. With these assumptions, the size of the
 * encoded data will be always \f$3/4 \times len\f$
 *
 * @param out the out buffer to hold the result. It is responsibility of the
 * caller to provide enough space for the result
 *
 * @param data the input data
 * @param len the number of input bytes in the \a data buffer
 */
void
conv_encoder_3_4_7(uint8_t *out, const uint8_t *data, size_t len)
{
	/* Pancuring Pattern for 3/4 */
	const uint8_t C1[3] = {1, 0, 1};
	const uint8_t C2[3] = {1, 1, 0};

	calculate_punctured_output(out, data, len, C1, C2, 3);
}

/**
 * CCSDS compliant K=7 R=5/6 convolutional encoder. Each call to this function
 * is completely independent from each other and there is no state information.
 * Initially the K-1 stages are set to 0. It is responsibility of the caller to
 * add tailbits or proper padding. With these assumptions, the size of the
 * encoded data will be always \f$5/6 \times len\f$
 *
 * @param out the out buffer to hold the result. It is responsibility of the
 * caller to provide enough space for the result
 *
 * @param data the input data
 * @param len the number of input bytes in the \a data buffer
 */
void
conv_encoder_5_6_7(uint8_t *out, const uint8_t *data, size_t len)
{
	/* Pancturing Pattern for 5/6 */
	const uint8_t C1[5] = {1, 0, 1, 0, 1};
	const uint8_t C2[5] = {1, 1, 0, 1, 0};

	calculate_punctured_output(out, data, len, C1, C2, 5);
}

/**
 * CCSDS compliant K=7 R=7/8 convolutional encoder. Each call to this function
 * is completely independent from each other and there is no state information.
 * Initially the K-1 stages are set to 0. It is responsibility of the caller to
 * add tailbits or proper padding. With these assumptions, the size of the
 * encoded data will be always \f$7/8 \times len\f$
 *
 * @param out the out buffer to hold the result. It is responsibility of the
 * caller to provide enough space for the result
 *
 * @param data the input data
 * @param len the number of input bytes in the \a data buffer
 */
void
conv_encoder_7_8_7(uint8_t *out, const uint8_t *data, size_t len)
{
	/* Pancturing Pattern for 7/8 */
	const uint8_t C1[7] = {1, 0, 0, 0, 1, 0, 1};
	const uint8_t C2[7] = {1, 1, 1, 1, 0, 1, 0};

	calculate_punctured_output(out, data, len, C1, C2, 7);
}

static void
calculate_punctured_output(uint8_t *out, const uint8_t *data, const size_t len,
                           const uint8_t *puncturing_pattern_c1,
                           const uint8_t *puncturing_pattern_c2,
                           const uint8_t puncturing_pattern_length)
{
	uint8_t shift_reg_7[CCSDS_CONV_CONSTRAINT_LENGTH_BITS] = {0, 0, 0, 0, 0, 0, 0};
	uint8_t bit = 0;
	size_t processed = 0;
	uint8_t input_bit = 0;
	uint8_t out_bit = 0;
	uint8_t out_g1 = 0;
	uint8_t out_g2 = 0;
	uint8_t puncturing_pattern_index = 0;

	/* Clean output buffer from possible garbage value */
	*out = 0;

	while (processed < len * 8) {
		/* Read each bit */
		bit = read_bit(*data, input_bit++);
		/* bit shift virtual register */
		bit_shift(shift_reg_7, bit);
		/* Calculate output for G1 and G2 */
		out_g1 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G1);
		out_g2 = calculate_generator_output(shift_reg_7, CCSDS_CONV_G2);

		/* Write convolution output to out */
		*out |= (((out_g1 & puncturing_pattern_c1[puncturing_pattern_index])
		          << (7 - out_bit)));
		out_bit += puncturing_pattern_c1[puncturing_pattern_index];
		*out |= (((out_g2 & puncturing_pattern_c2[puncturing_pattern_index])
		          << (7 - out_bit)));
		out_bit += puncturing_pattern_c2[puncturing_pattern_index];

		/* Move pancture index to next value and reset if needed */
		puncturing_pattern_index = (puncturing_pattern_index + 1) %
		                           puncturing_pattern_length;

		/* Increase processed bits */
		processed++;
		/* Check if processed 8 bits */
		if (out_bit == 8) {
			out++;
			*out = 0;
			out_bit = 0;
		}
		/* Check if out byte full */
		if (input_bit == 8) {
			data++;
			input_bit = 0;
		}
	}
}

static inline uint8_t
read_bit(const uint8_t data, const uint8_t input_bit)
{
	return (data >> (7 - input_bit)) & 1;
}

static inline uint8_t
calculate_generator_output(const uint8_t *shift_reg, const uint8_t *polynomial)
{
	uint8_t out = 0;

	for (int i = 0; i < CCSDS_CONV_CONSTRAINT_LENGTH_BITS; i++) {
		out ^= shift_reg[i] * polynomial[i];
	}
	return out;
}

static inline void
bit_shift(uint8_t *reg, uint8_t d_in)
{
	for (int i = 6; i > 0; i--) {
		reg[i] = reg[i - 1];
	}
	reg[0] = d_in;
}
