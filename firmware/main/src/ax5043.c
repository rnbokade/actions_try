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

#include "ax5043.h"
#include "ax25.h"
#include <string.h>
#include <stdlib.h>

static uint8_t __tx_buf[MAX_FRAME_LEN];
static size_t __tx_buf_idx = 0;
static uint8_t __tx_fifo_chunk[AX5043_FIFO_MAX_SIZE];
static uint32_t __tx_remaining = 0;
static uint8_t __spi_tx[SPI_MAX_TRASNFER_SIZE];
static uint8_t __spi_rx[SPI_MAX_TRASNFER_SIZE];

/**
 * FIFO command for the preamble. The third byte corresponds the length of
 * the preamble and is set by the TX routine for every frame
 */
static uint8_t __preamble_cmd[4] = {
	AX5043_FIFO_REPEATDATA_CMD,
	AX5043_FIFO_PKTSTART | AX5043_FIFO_RAW | AX5043_FIFO_NOCRC,
	0,
	AX25_SYNC_FLAG
};

/**
 * FIFO command for the postable. The third byte corresponds the length of
 * the postable and is set by the TX routine for every frame
 */
static uint8_t __postamble_cmd[4] = {
	AX5043_FIFO_REPEATDATA_CMD,
	AX5043_FIFO_PKTSTART | AX5043_FIFO_PKTEND | AX5043_FIFO_RAW
	| AX5043_FIFO_NOCRC, 0, AX25_SYNC_FLAG
};


/**
 * Indicates if a TX is currently active
 */
static volatile uint8_t __tx_active = 0;

static struct ax5043_conf *__ax5043_conf = NULL;

static inline int
set_tx_black_magic_regs();

static int
ax5043_spi_read(struct ax5043_conf *conf, uint8_t *out, uint16_t reg,
                uint32_t len);

static int
ax5043_spi_write(struct ax5043_conf *conf, uint16_t reg, const uint8_t *in,
                 uint32_t len);

static int
ax5043_spi_read_8(struct ax5043_conf *conf, uint8_t *out,
                  uint16_t reg);

static int
ax5043_spi_read_16(struct ax5043_conf *conf, uint16_t *out, uint16_t reg);

static int
ax5043_spi_read_32(struct ax5043_conf *conf, uint32_t *out, uint16_t reg);


static int
ax5043_spi_write_8(struct ax5043_conf *conf, uint16_t reg,
                   uint8_t in);

static int
ax5043_spi_write_16(struct ax5043_conf *conf, uint16_t reg,
                    uint16_t in);

static int
ax5043_spi_write_24(struct ax5043_conf *conf, uint16_t reg,
                    uint32_t in);

static int
ax5043_spi_write_32(struct ax5043_conf *conf, uint16_t reg,
                    uint32_t in);

static int
ax5043_set_pll_params(struct ax5043_conf *conf);

static int
ax5043_autoranging(struct ax5043_conf *conf, freq_mode_t mode);

static int
ax5043_set_tx_synth(struct ax5043_conf *conf, freq_mode_t mode);

/**
 * Checks if the AX5043 handler is valid
 * @param conf the AX5043 configuration handler pointer
 * @return true if it is valid, false otherwise
 */
static bool
ax5043_conf_valid(struct ax5043_conf *conf)
{
	if (!conf || !conf->spi_select || !conf->read || !conf->write
	    || !conf->f_xtal) {
		return false;
	}
	return true;
}


static inline int
tx_fifo_irq_enable(struct ax5043_conf *conf, uint8_t enable)
{
	int ret;
	uint16_t val =
	        enable ?
	        AX5043_IRQMFIFOTHRFREE | AX5043_IRQMRADIOCTRL : AX5043_IRQMRADIOCTRL;
	/* Enable FIFO IRQs */
	ret = ax5043_spi_write_16(conf, AX5043_REG_IRQMASK1, val);
	if (ret) {
		return ret;
	}
	return AX5043_OK;
}

/**
 * Checks if the AX5043 IC is ready for use. This includes proper values
 * on the \ref conf structure and succesfull completion of the \ref init()
 * function
 * @param conf the AX5043 configuration handler
 * @return true if the IC can be used, false otherwise
 */
bool
ax5043_ready(struct ax5043_conf *conf)
{
	return ax5043_conf_valid(conf) && conf->priv.ready;
}

/**
 * Resets the AX5043 IC and set it to \a POWEDOWN mode.
 * Any RF sythnesizer setup is invalidated after the call of this
 * function
 * @param conf the AX5043 configuration handler
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_reset(struct ax5043_conf *conf)
{
	int ret;
	uint8_t val;

	if (!ax5043_conf_valid(conf)) {
		return -AX5043_INVALID_PARAM;
	}

	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}

	struct ax5043_conf_priv *conf_priv = &conf->priv;
	conf_priv->rf_init = 0;
	conf->spi_select(0);
	conf->delay_us(100);
	conf->spi_select(1);
	conf->delay_us(100);
	conf->spi_select(0);
	conf->delay_us(100);

	/* Reset the chip using the appropriate register */
	val = BIT(7);
	ret = ax5043_spi_write_8(conf, AX5043_REG_PWRMODE, val);
	if (ret) {
		return ret;
	}
	conf->delay_us(100);
	/* Clear the reset bit, but keep REFEN and XOEN */
	ret = ax5043_spi_read_8(conf, &val, AX5043_REG_PWRMODE);
	if (ret) {
		return ret;
	}
	val &= (BIT(6) | BIT(5));
	ret = ax5043_spi_write_8(conf, AX5043_REG_PWRMODE, val);
	if (ret) {
		return ret;
	}
	conf->delay_us(100);

	ret = ax5043_set_power_mode(conf, POWERDOWN);
	if (ret) {
		return ret;
	}
	return AX5043_OK;
}

/**
 * Helper function for the initialization of the ax5043_conf_t structure
 * @param conf the AX5043 configuration handler
 * @param fxtal the frequency of the reference crystal
 * @param vco the type of the VCO used
 * @param spi_sel function pointer for the SPI CS signal
 * @param spi_read SPI read function pointer
 * @param spi_write SPI write function pointer
 * @param delay_us delay function pointer with microsecond (us) accuracy
 * @param millis function pointer returning the MCU time in milliseconds
 * @param tx_complete_callback function pointer to a callback, activated after
 * the end of each transmission. Can be used to enable/disable external PAs,
 * RF switches etc. Can be NULL if it is not needed
 * @return 0 on success or negative error code
 */
int
ax5043_prepare(struct ax5043_conf *conf, uint32_t fxtal, vco_mode_t vco,
               int (*spi_sel)(bool enable),
               int (*spi_read)(uint8_t *rx, uint8_t *tx, uint32_t len),
               int (*spi_write)(uint8_t *rx, uint8_t *tx, uint32_t len),
               void (*delay_us)(uint32_t us),
               uint32_t (*millis)(),
               void (*tx_complete_callback)()
              )
{
	if (!conf || !spi_sel || !spi_read || !spi_write || !delay_us || !millis) {
		return -AX5043_INVALID_PARAM;
	}
	memset(conf, 0, sizeof(struct ax5043_conf));
	conf->f_xtal = fxtal;
	conf->vco = vco;
	conf->spi_select = spi_sel;
	conf->read = spi_read;
	conf->write = spi_write;
	conf->delay_us = delay_us;
	conf->millis = millis;
	conf->tx_complete_clbk = tx_complete_callback;
	return AX5043_OK;
}

/**
 * Initialization routine for the AX5043 IC
 * @param conf the AX5043 configuration handler
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_init(struct ax5043_conf *conf)
{
	int ret;
	uint8_t revision;
	uint8_t val;

	if (!ax5043_conf_valid(conf)) {
		return -AX5043_INVALID_PARAM;
	}

	struct ax5043_conf_priv *conf_priv = &conf->priv;
	if (conf->f_xtal < MIN_FXTAL || conf->f_xtal > MAX_FXTAL) {
		return -AX5043_INVALID_PARAM;
	}

	switch (conf->vco) {
		case VCO_INTERNAL:
		case VCO_EXTERNAL:
			break;
		default:
			return -AX5043_INVALID_PARAM;
	}

	conf_priv->rf_init = 0;
	if (conf->f_xtal > 24800000) {
		conf_priv->f_xtaldiv = 2;
	} else {
		conf_priv->f_xtaldiv = 1;
	}

	/* Try first to read the revision register of the AX5043 */
	ret = ax5043_spi_read_8(conf, &revision, AX5043_REG_REV);
	if (ret) {
		return ret;
	}

	if (revision != AX5043_REV) {
		return -AX5043_NOT_FOUND;
	}

	/* To ensure communication try to write and read the scratch register */
	val = AX5043_SCRATCH_TEST;
	ret = ax5043_spi_write_8(conf, AX5043_REG_SCRATCH, val);
	if (ret) {
		return ret;
	}

	val = 0x0;
	ret = ax5043_spi_read_8(conf, &val, AX5043_REG_SCRATCH);
	if (ret) {
		return ret;
	}

	if (val != AX5043_SCRATCH_TEST) {
		return -AX5043_NOT_FOUND;
	}

	/* From now on the IC is considered ready */
	conf_priv->ready = 1;
	conf_priv->freqa = 0;
	conf_priv->freqb = 0;
	conf_priv->freqa_req = 0;
	conf_priv->freqb_req = 0;
	conf_priv->auto_rng_freqa = 0;
	conf_priv->auto_rng_freqb = 0;
	ret = ax5043_reset(conf);
	if (ret) {
		return ret;
	}

	ret = ax5043_set_pll_params(conf);
	if (ret) {
		return ret;
	}

	/* Write the performance register F35 based on the XTAL frequency */
	if (conf_priv->f_xtaldiv == 1) {
		ret = ax5043_spi_write_8(conf, 0xF35, 0x10);
	} else {
		ret = ax5043_spi_write_8(conf, 0xF35, 0x11);
	}
	if (ret) {
		return ret;
	}

	/* FIFO maximum chunk */
	ret = ax5043_spi_write_8(conf, AX5043_REG_PKTCHUNKSIZE,
	                         AX5043_PKTCHUNKSIZE_240);
	if (ret) {
		return ret;
	}

	/* Set an internal copy for the IRQ handler */
	__ax5043_conf = conf;

	/*
	 * Set the FIFO IRQ threshold. During TX, when the free space is larger
	 * than this threshold, an IRQ is raised
	 */
	ret = ax5043_spi_write_16(conf, AX5043_REG_FIFOTHRESH1, AX5043_FIFO_FREE_THR);
	ret = ax5043_spi_write_16(conf, AX5043_REG_IRQMASK1, AX5043_IRQMRADIOCTRL);
	ret = ax5043_spi_write_8(conf, AX5043_REG_RADIOEVENTMASK0, AX5043_REVMDONE);
	if (ret) {
		return ret;
	}
	return AX5043_OK;
}

/**
 * Sets the power mode of the AX5043
 * @param conf the AX5043 configuration handler
 * @param mode the power mode
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_set_power_mode(struct ax5043_conf *conf, power_mode_t mode)
{
	int ret;
	uint8_t val;

	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}

	/* Read the contents of the register */
	ret = ax5043_spi_read_8(conf, &val, AX5043_REG_PWRMODE);
	if (ret) {
		return ret;
	}

	/* Keep REFEN and XOEN values */
	val &= (BIT(6) | BIT(5));

	switch (mode) {
		case POWERDOWN:
			val |= AX5043_POWERDOWN;
			break;
		case DEEPSLEEP:
			val |= AX5043_DEEPSLEEP;
			break;
		case STANDBY:
			val |= AX5043_STANDBY;
			break;
		case FIFO_ENABLED:
			val |= AX5043_FIFO_ENABLED;
			break;
		case RECEIVE_MODE:
			val |= AX5043_RECEIVE_MODE;
			break;
		case RECEIVER_RUNNING:
			val |= AX5043_RECEIVER_RUNNING;
			break;
		case RECEIVER_WOR:
			val |= AX5043_RECEIVER_WOR;
			break;
		case TRANSMIT_MODE:
			val |= AX5043_TRANSMIT_MODE;
			break;
		case FULLTX:
			val |= AX5043_FULLTX;
			break;
		default:
			return -AX5043_INVALID_PARAM;
	}
	return ax5043_spi_write_8(conf, AX5043_REG_PWRMODE, val);
}

/**
 * Set the TX deviation
 * @param conf the AX5043 configuration handler
 * @param mod_index the modulation index
 * @param baud the baud rate
 * @return 0 on success or negative error code
 */
int
ax5043_set_fsk_deviation(struct ax5043_conf *conf, float mod_index,
                         uint32_t baud)
{
	int ret = AX5043_OK;
	uint32_t val;

	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}

	float f_deviation = ((mod_index / 2.0) * (float) baud);
	val = (uint32_t)((((float)f_deviation) / (float) conf->f_xtal) *
	                 (1 << 24)) | 0x1;

	ret = ax5043_spi_write_24(conf, AX5043_REG_FSKDEV2, val);
	if (ret) {
		return ret;
	}
	return AX5043_OK;
}

int
ax5043_conf_tx(struct ax5043_conf *conf, const struct tx_params *params)
{
	int ret = AX5043_OK;
	uint32_t val32;
	uint8_t val8 = 0;

	if (!conf || !params) {
		return -AX25_INVALID_PARAM;
	}
	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}

	switch (params->rf_out_mode) {
		case TXDIFF:
			val8 = BIT(0);
			break;
		case TXSE:
			val8 = BIT(1);
			break;
		default:
			return -AX25_INVALID_PARAM;
	}

	switch (params->shaping) {
		case UNSHAPED:
		case RC:
			val8 |= (params->shaping << 2);
			break;
		default:
			return -AX25_INVALID_PARAM;
	}

	/* Enable explicitly PTTLCK and BROWN gates */
	/* FIXME! */
	/* val8 |= BIT(6) | BIT(7); */
	ret = ax5043_spi_write_8(conf, AX5043_REG_MODCFGA, val8);
	if (ret) {
		return ret;
	}

	switch (params->mod) {
		case ASK:
		case ASK_COHERENT:
		case PSK:
		case OQPSK:
		case MSK:
		case FSK:
		case FSK_4:
		case AFSK:
		case FM:
			ret = ax5043_spi_write_8(conf, AX5043_REG_MODULATION, params->mod);
			if (ret) {
				return ret;
			}
			break;
		default:
			return -AX25_INVALID_PARAM;
	}

	switch (params->framing) {
		case RAW:
		case RAW_SOFT:
		case HDLC:
		case RAW_PATTERN_MATCH:
		case M_BUS:
		case MBUS_4_6:
			val8 = params->framing << 1;
			break;
		default:
			return -AX5043_INVALID_PARAM;
	}
	/* FIXME: Depending on the framing, append CRC if any */
	ret = ax5043_spi_write_8(conf, AX5043_REG_FRAMING, val8);
	if (ret) {
		return ret;
	}

	/* Apply framing specific encoding schemes */
	switch (params->framing) {
		case RAW:
		case RAW_SOFT:
			val8 = params->raw.en_inv & 0x1;
			val8 |= (params->raw.en_diff & 0x1) << 1;
			val8 |= (params->raw.en_scrambler & 0x1) << 2;
			val8 |= (params->raw.en_manch & 0x1) << 3;
			val8 |= (params->raw.en_nosync & 0x1) << 4;
			ret = ax5043_spi_write_8(conf, AX5043_REG_ENCODING, val8);
			if (ret) {
				return ret;
			}
			break;
		case HDLC:
			/* TODO! */
			break;
	}

	/* Set TX power, it can re-adjusted later */
	ret = ax5043_set_tx_power(conf, params->pout_dBm);
	if (ret) {
		return ret;
	}

	/* Set baudrate */
	ret = ax5043_set_tx_baud(conf, params->baudrate);
	if (ret) {
		return ret;
	}

	/* In case of FSK/MSK , set the frequency deviation and the shaping filter */
	if (params->mod == FSK || params->mod == MSK) {
		freq_shaping_t freq_shape;
		float mod_index = 0;
		if (params->mod == FSK) {
			freq_shape = params->fsk.freq_shaping;
			mod_index = params->fsk.mod_index;
		} else if (params->mod == MSK) {
			freq_shape = params->msk.freq_shaping;
			mod_index = 0.5;
		}
		switch (freq_shape) {
			case EXTERNAL_FILTER:
			case GAUSIAN_BT_0_3:
			case GAUSIAN_BT_0_5:
				val8 = freq_shape;
				ret = ax5043_spi_write_8(conf, AX5043_REG_MODCFGF, val8);
				if (ret) {
					return ret;
				}
				break;
			case INVALID:
			default:
				return -AX5043_INVALID_PARAM;
		}

		ret = ax5043_set_fsk_deviation(conf, mod_index, params->baudrate);
	}
	return ret;
}


/**
 * Reconfigures the TX power. TX power settings are clamped in the [0-16] dBm
 * range automatically.
 *
 * @param conf the AX5043 configuration handler
 * @param pout the desired output power in dBm. Automatically quantized to 0.5
 * dBm steps
 * @return 0 on success or negative error code
 */
int
ax5043_set_tx_power(struct ax5043_conf *conf, float pout)
{
	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}

	/* Clamp power value */
	pout = min(pout, 16.0f);
	pout = max(pout, 0.0f);

	const float txpwrb = (pout * ((1 << 12) - 1)) / 16.0f;
	const uint16_t val = txpwrb;
	return ax5043_spi_write_16(conf, AX5043_REG_TXPWRCOEFFB1, val);
}


/**
 * Set the TX baudrate
 * @param conf the AX5043 configuration handler
 * @param baud the baudrate
 * @return 0 on success or negative error code
 */
int
ax5043_set_tx_baud(struct ax5043_conf *conf, uint32_t baud)
{
	int ret = AX5043_OK;
	uint32_t val;

	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}

	/*FIXME: Sanity check? */
	val = (uint32_t)((((float) baud) / (float) conf->f_xtal) * (1 << 24)) | 0x1;
	ret = ax5043_spi_write_24(conf, AX5043_REG_TXRATE2, val);
	if (ret) {
		return ret;
	}
	return AX5043_OK;
}

int
ax5043_tune(struct ax5043_conf *conf, freq_mode_t mode)
{
	int ret = AX5043_OK;
	uint32_t freq = 0;
	uint8_t rfdiv = 0;
	uint8_t pllcodediv = 0;
	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}

	struct ax5043_conf_priv *conf_priv = &conf->priv;
	switch (mode) {
		case FREQA_MODE:
			freq = conf_priv->freqa_req;
			break;
		case FREQB_MODE:
			freq = conf_priv->freqb_req;
			break;
		default:
			return -AX25_INVALID_PARAM;
	}

	ret = ax5043_set_tx_synth(conf, mode);
	if (ret) {
		return ret;
	}

	/*
	 * Check the frequency range. The actual range depends on the VCO used.
	 * \ref ax5043_config_freq() checks for a valid frequency, but there is
	 * always the case someone accidentally to alter the private members
	 * of the configuration structure
	 */
	switch (conf->vco) {
		case VCO_INTERNAL:
			if (freq >= MIN_RF_FREQ_INT_VCO_RFDIV0
			    && freq <= MAX_RF_FREQ_INT_VCO_RFDIV0) {
				rfdiv = AX5043_RFDIV0;
			} else if (freq >= MIN_RF_FREQ_INT_VCO_RFDIV1
			           && freq <= MAX_RF_FREQ_INT_VCO_RFDIV1) {
				rfdiv = AX5043_RFDIV1;
			} else {
				return -AX5043_INVALID_PARAM;
			}
			break;
		case VCO_EXTERNAL:
			if (freq >= MIN_RF_FREQ_EXT_VCO_RFDIV0
			    && freq <= MAX_RF_FREQ_EXT_VCO_RFDIV0) {
				rfdiv = AX5043_RFDIV0;
			} else if (freq >= MIN_RF_FREQ_EXT_VCO_RFDIV1
			           && freq <= MAX_RF_FREQ_EXT_VCO_RFDIV1) {
				rfdiv = AX5043_RFDIV1;
			} else {
				return -AX5043_INVALID_PARAM;
			}
			break;
		default:
			return -AX5043_INVALID_PARAM;
	}
	pllcodediv = rfdiv | (conf->vco << 4);
	ret = ax5043_spi_write_8(conf, AX5043_REG_PLLVCODIV, pllcodediv);
	if (ret) {
		return ret;
	}

	/* Write properly the F34 performance register based on the RFDIV*/
	if (rfdiv == AX5043_RFDIV1) {
		ret = ax5043_spi_write_8(conf, 0xF34, 0x28);
	} else {
		ret = ax5043_spi_write_8(conf, 0xF34, 0x08);
	}
	if (ret) {
		return ret;
	}

	/*
	 * Set the RF frequency
	 * Frequency should be avoided to be a multiple integer of the crystal
	 * frequency, so we always set to 1 the LSB
	 */
	uint32_t reg_val = ((uint32_t)(((float) freq / (float) conf->f_xtal) *
	                               (1 << 24))
	                    | 0x1);
	if (mode == FREQA_MODE) {
		ret = ax5043_spi_write_32(conf, AX5043_REG_FREQA3, reg_val);
		if (ret == AX5043_OK) {
			conf_priv->freqa = freq;
		}
	} else {
		ret = ax5043_spi_write_32(conf, AX5043_REG_FREQB3, reg_val);
		if (ret == AX5043_OK) {
			conf_priv->freqb = freq;
		}
	}
	if (ret) {
		return ret;
	}
	/* Perform autoranging if it is necessary */
	return ax5043_autoranging(conf, mode);
}

/**
 * The AX5043 IC has two different set of registers for setting the RF frequency.
 * This allows very fast tuning settling times between different frequency bands.
 * It also makes possible for altering frequency, without affecting in any way
 * the currently active frequency setup.
 *
 * @note: This function sets only the desired frequency at the corresponding
 * registers (A or B). It does not perform any actual RF tuning. Users should
 * call the \ref ax5043_tune() function for the hardware to tune on the desired
 * frequency
 *
 * @param conf the AX5043 configuration handler
 * @param mode the frequency mode (A or B) to set the frequency
 * @param freq the frequency in Hz
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_config_freq(struct ax5043_conf *conf, freq_mode_t mode, uint32_t freq)
{
	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}
	if (mode != FREQA_MODE && mode != FREQB_MODE) {
		return -AX5043_INVALID_PARAM;
	}

	struct ax5043_conf_priv *conf_priv = &conf->priv;

	/* Check the frequency range. The actual range depends on the VCO used */
	switch (conf->vco) {
		case VCO_INTERNAL:
			if (!((freq >= MIN_RF_FREQ_INT_VCO_RFDIV0
			       && freq <= MAX_RF_FREQ_INT_VCO_RFDIV0)
			      || (freq >= MIN_RF_FREQ_INT_VCO_RFDIV1
			          && freq <= MAX_RF_FREQ_INT_VCO_RFDIV1))) {
				return -AX5043_INVALID_PARAM;
			}
			break;
		case VCO_EXTERNAL:
			if (!((freq >= MIN_RF_FREQ_EXT_VCO_RFDIV0
			       && freq <= MAX_RF_FREQ_EXT_VCO_RFDIV0)
			      || (freq >= MIN_RF_FREQ_EXT_VCO_RFDIV1
			          && freq <= MAX_RF_FREQ_EXT_VCO_RFDIV1))) {
				return -AX5043_INVALID_PARAM;
			}
			break;
		default:
			return -AX5043_INVALID_PARAM;
	}

	switch (mode) {
		case FREQA_MODE:
			conf_priv->freqa_req = freq;
			break;
		case FREQB_MODE:
			conf_priv->freqb_req = freq;
			break;
		default:
			return -AX5043_INVALID_PARAM;
	}
	return AX5043_OK;
}

/**
 * Sets the TX frequency synthesizer related configuration registers.
 * @param conf the AX5043 configuration handler
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_set_tx_synth(struct ax5043_conf *conf, freq_mode_t mode)
{
	int ret;
	uint8_t val;

	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}

	switch (mode) {
		case FREQA_MODE:
			val = 0x0;
			break;
		case FREQB_MODE:
			val = 1 << 7;
			break;
		default:
			return -AX5043_INVALID_PARAM;
	}

	/* Bypass external filter and use 100 kHZ loop bandwidth */
	val |= BIT(3) | BIT(0);
	ret = ax5043_spi_write_8(conf, AX5043_REG_PLLLOOP, val);
	if (ret) {
		return ret;
	}

	/*
	 * Set the charge pump current based on the loop bandwidth
	 * 68 uA @ 100 kHZ
	 */
	ret = ax5043_spi_write_8(conf, AX5043_REG_PLLCPI, (uint8_t)(68 / 8.5));
	if (ret) {
		return ret;
	}
	ret = ax5043_spi_write_8(conf, AX5043_REG_XTALCAP, 0);
	return ret;
}

/**
 * Sets the PLL related configuration registers.
 * @param conf the AX5043 configuration handler
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_set_pll_params(struct ax5043_conf *conf)
{
	int ret;
	uint8_t i = 8;

	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}

	/* Set VCO to manual */
	ret = ax5043_spi_write_8(conf, AX5043_REG_PLLVCOI,
	                         AX5043_PLLVCOI_MANUAL | (1250 / 50));
	if (ret) {
		return ret;
	}

	/*
	 * According to the manual PLL ranging clock should be less than 1/10
	 * of the PLL loop bandwidth. The smallest PLL bandwidth configuration
	 * is 100 kHz.
	 */
	while (conf->f_xtal / (1 << i) > 10000) {
		i++;
	}
	i = i > 15 ? 15 : i;
	ret = ax5043_spi_write_8(conf, AX5043_REG_PLLRNGCLK, i - 8);
	return ret;
}

/**
 * Performs auto-ranging using the frequency registers configured by
 * ax5043_freqsel().
 *
 * @param conf the AX5043 configuration handler
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_autoranging(struct ax5043_conf *conf, freq_mode_t mode)
{
	int ret = AX5043_OK;
	uint16_t pllranging_reg;
	uint8_t val = 0;
	uint32_t new_freq = 0;

	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}

	struct ax5043_conf_priv *conf_priv = &conf->priv;
	switch (mode) {
		case FREQA_MODE:
			pllranging_reg = AX5043_REG_PLLRANGINGA;
			new_freq = conf_priv->freqa;
			break;
		case FREQB_MODE:
			pllranging_reg = AX5043_REG_PLLRANGINGB;
			new_freq = conf_priv->freqb;
			break;
		default:
			return -AX5043_INVALID_PARAM;
	}

	ret = ax5043_set_power_mode(conf, STANDBY);
	if (ret) {
		return ret;
	}
	ret = ax5043_wait_xtal(conf, 100);
	if (ret) {
		return ret;
	}

	/* Check if autoranging is necessary */
	if (mode == FREQA_MODE) {
		if (abs(new_freq - conf_priv->auto_rng_freqa) < 2500000) {
			return AX5043_OK;
		}
	} else {
		if (abs(new_freq - conf_priv->auto_rng_freqb) < 2500000) {
			return AX5043_OK;
		}
	}

	/* Write the initial VCO setting and start autoranging */
	val = BIT(4) | AX5043_VCOR_INIT;
	ret = ax5043_spi_write_8(conf, pllranging_reg, val);
	if (ret) {
		return ret;
	}

	conf->delay_us(10);
	val = 0;
	/* Wait until the autoranging is complete */
	while (val & BIT(4)) {
		ret = ax5043_spi_read_8(conf, &val, pllranging_reg);
		if (ret) {
			return ret;
		}
	}

	if (val & BIT(5)) {
		return -AX5043_AUTORANGING_ERROR;
	}
	/* Mark the last succesfull autorange frequency */
	if (mode == FREQA_MODE) {
		conf_priv->auto_rng_freqa = new_freq;
	} else {
		conf_priv->auto_rng_freqb = new_freq;
	}
	return AX5043_OK;
}


/**
 *
 * @param conf the AX5043 configuration handler
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_aprs_framing_setup(struct ax5043_conf *conf)
{
	int ret = AX5043_OK;
	uint8_t val = 0;

	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}

	/* Set modulation */
	val = AX5043_MODULATION_FSK;
	ret = ax5043_spi_write_8(conf, AX5043_REG_MODULATION, val);
	if (ret) {
		return ret;
	}

	/*
	 * As we do not use any external filter, try to filter from
	 * the AX5043 the signal
	 */
	ret = ax5043_spi_write_8(conf, AX5043_REG_MODCFGF,
	                         AX5043_FREQSHAPE_GAUSSIAN_BT_05);
	if (ret) {
		return ret;
	}

	/* Set HDLC encoding: Differential = 1, Inverse = 1, Scrambling = 1 */
	ax5043_spi_write_8(conf, AX5043_REG_ENCODING,
	                   AX5043_ENC_DIFF | AX5043_ENC_INV | AX5043_ENC_SCRAM);

	/* HDLC framing */
	ax5043_spi_write_8(conf, AX5043_REG_FRAMING,
	                   AX5043_HDLC_FRAMING | AX5043_CRC16_CCITT);
	return ret;
}


static int
__tx_frame_end(struct ax5043_conf *conf)
{
	int ret;

	tx_fifo_irq_enable(conf, 0);

	/* Set AX5043 to power down mode */
	ret = ax5043_set_power_mode(conf, POWERDOWN);
	if (conf->tx_complete_clbk) {
		conf->tx_complete_clbk();
	}
	__tx_active = 0;
	return ret;
}

static int
__tx_frame(struct ax5043_conf *conf, const uint8_t *in, uint32_t len,
           uint8_t preamble_len,
           uint8_t postamble_len,
           uint32_t timeout_ms)
{
	int ret = AX5043_OK;
	uint8_t single_fifo_access = 0;
	uint8_t data_cmd[3] = { AX5043_FIFO_VARIABLE_DATA_CMD, 0, 0};
	size_t chunk_size = 0;
	size_t avail;
	uint8_t val;
	uint32_t start = conf->millis();

	/*
	 * Apply preamble and postamble repetition length. Rest of the fields should
	 * remain unaltered
	 */
	__preamble_cmd[2] = preamble_len;
	__postamble_cmd[2] = postamble_len;

	memcpy(__tx_fifo_chunk, __preamble_cmd, sizeof(__preamble_cmd));
	chunk_size = sizeof(__preamble_cmd);
	__tx_buf_idx = 0;

	/*
	 * Always leave some space for the postamble. This greatly reduces the
	 * complexity of dealing with some corner cases
	 */
	avail = min(AX5043_FIFO_MAX_SIZE - sizeof(__preamble_cmd) - sizeof(data_cmd)
	            - sizeof(__postamble_cmd), len);
	if (len == avail) {
		data_cmd[1] = len + 1;
		data_cmd[2] = AX5043_FIFO_PKTEND;
		__tx_remaining = 0;
		memcpy(__tx_fifo_chunk + chunk_size, data_cmd, sizeof(data_cmd));
		chunk_size += sizeof(data_cmd);
		memcpy(__tx_fifo_chunk + chunk_size, in, len);
		chunk_size += len;
		/*
		 * At this point we are sure that the whole frame + postamble can fit in
		 * the FIFO chunk
		 */
		memcpy(__tx_fifo_chunk + chunk_size, __postamble_cmd,
		       sizeof(__postamble_cmd));
		chunk_size += sizeof(__postamble_cmd);
		single_fifo_access = 1;
	} else {
		data_cmd[1] = avail + 1;
		data_cmd[2] = 0;
		memcpy(__tx_fifo_chunk + chunk_size, data_cmd, sizeof(data_cmd));
		chunk_size += sizeof(data_cmd);
		memcpy(__tx_fifo_chunk + chunk_size, in, avail);
		chunk_size += avail;

		memcpy(__tx_buf, in + avail, len - avail);
		__tx_remaining = len - avail;
		single_fifo_access = 0;
	}

	/* Set AX5043 to FULLTX mode */
	ret = ax5043_set_power_mode(conf, FULLTX);
	if (ret) {
		return ret;
	}

	ax5043_wait_xtal(conf, 100);

	/* Wait for the FIFO to become ready */
	val = 0;
	while (!val) {
		ax5043_spi_read_8(conf, &val, AX5043_REG_POWSTAT);
		/* Select only the modem power state */
		val &= AX5043_SVMODEM;
		if (conf->millis() - start > timeout_ms) {
			ret = -AX5043_TIMEOUT;
			break;
		}
	}

	/* Fire-up the first data to the FIFO */
	ret = ax5043_spi_write(conf, AX5043_REG_FIFODATA, __tx_fifo_chunk, chunk_size);
	if (ret) {
		return ret;
	}
	__tx_active = 1;
	/* Commit to FIFO ! */
	ret = ax5043_spi_write_8(conf, AX5043_REG_FIFOSTAT, AX5043_FIFO_COMMIT_CMD);
	/*
	 * Enable FIFO free IRQ, only if it is needed.
	 * From now on, the IRQ handler will deal with filling the FIFO properly if
	 * the frame size exceeds the maximum FIFO packet size
	 */
	if (!single_fifo_access) {
		tx_fifo_irq_enable(conf, 1);
	} else {
		tx_fifo_irq_enable(conf, 0);
	}
	return ret;
}

int
ax5043_tx_frame(struct ax5043_conf *conf, const uint8_t *in, uint32_t len,
                uint8_t preamble_len,
                uint8_t postamble_len,
                uint32_t timeout_ms)
{
	int ret = AX5043_OK;

	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}

	/* Wait for the previous frame to be transmitted */
	while (__tx_active) {
		ret++;
	}

	ret = __tx_frame(conf, in, len, preamble_len, postamble_len, timeout_ms);
	return ret;
}

/**
 * Wait the crystal to become ready
 * @param conf the AX5043 configuration handler
 * @param timeout_ms the timeout in milliseconds
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_wait_xtal(struct ax5043_conf *conf, uint32_t timeout_ms)
{
	int ret;
	uint8_t val = 0x0;

	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}

	uint32_t start = conf->millis();
	while (!val) {
		ret = ax5043_spi_read_8(conf,  &val, AX5043_REG_XTALSTATUS);
		if (ret) {
			return ret;
		}
		if ((conf->millis() - start) > timeout_ms) {
			return -AX5043_TIMEOUT;
		}
	}
	return AX5043_OK;
}

int
ax5043_spi_read(struct ax5043_conf *conf, uint8_t *out, uint16_t reg,
                uint32_t len)
{
	int ret = AX5043_OK;
	reg &= 0xFFF;
	if (!ax5043_conf_valid(conf) || len - 2 > SPI_MAX_TRASNFER_SIZE) {
		return -AX5043_INVALID_PARAM;
	}
	const uint8_t mask = BIT(6) | BIT(5) | BIT(4);
	memset(__spi_tx, 0, len + 2);

	/* Prepare the long access header AND9347 p.5 */
	__spi_tx[0] =  mask | (~mask & (reg >> 8));
	__spi_tx[1] = reg & 0xFF;
	conf->spi_select(1);
	ret = conf->read(__spi_rx, __spi_tx, len + 2);
	conf->spi_select(0);
	if (ret) {
		return ret;
	}
	/* Skip the status response */
	memcpy(out, __spi_rx + 2, len);
	return AX5043_OK;
}

int
ax5043_spi_read_8(struct ax5043_conf *conf, uint8_t *out, uint16_t reg)
{
	int ret;
	reg &= 0xFFF;
	conf->spi_select(1);
	if (reg > 0xFF) {
		/* Long register access based on AND9347 p.5 */
		const uint8_t mask = BIT(6) | BIT(5) | BIT(4);
		uint8_t req[3] = {mask | (~mask & (reg >> 8)), reg & 0xFF, 0x0};
		uint8_t res[3] = {0x0, 0x0, 0x0};
		ret = conf->read(res, req, 3);
		*out = res[2];
	} else {
		/* Short register access based on AND9347 p.5 */
		uint8_t req[2] = {0x7F & reg, 0x0};
		uint8_t res[2] = {0x0, 0x0};
		ret = conf->read(res, req, 2);
		*out = res[1];
	}
	conf->spi_select(0);
	return ret;
}

int
ax5043_spi_read_16(struct ax5043_conf *conf, uint16_t *out, uint16_t reg)
{
	int ret;
	reg &= 0xFFF;
	const uint8_t mask = BIT(6) | BIT(5) | BIT(4);
	/* Prepare the long access header AND9347 p.5 */
	uint8_t req[4] = {mask | (~mask & (reg >> 8)), reg & 0xFF, 0x0, 0x0};
	uint8_t res[4];
	conf->spi_select(1);
	ret = conf->read(res, req, 4);
	conf->spi_select(0);
	uint16_t tmp = res[2];
	tmp = (tmp << 8) | res[3];
	*out = tmp;
	return ret;
}

int
ax5043_spi_read_32(struct ax5043_conf *conf, uint32_t *out, uint16_t reg)
{
	int ret;
	reg &= 0xFFF;
	const uint8_t mask = BIT(6) | BIT(5) | BIT(4);
	/* Prepare the long access header AND9347 p.5 */
	uint8_t req[6] = {mask | (~mask & (reg >> 8)), reg & 0xFF, 0x0, 0x0, 0x0, 0x0};
	uint8_t res[6];
	conf->spi_select(1);
	ret = conf->read(res, req, 6);
	conf->spi_select(0);
	uint32_t tmp = res[2];
	tmp = (tmp << 8) | res[3];
	tmp = (tmp << 8) | res[4];
	tmp = (tmp << 8) | res[5];
	*out = tmp;
	return ret;
}

int
ax5043_spi_write(struct ax5043_conf *conf, uint16_t reg, const uint8_t *in,
                 uint32_t len)
{
	int ret = AX5043_OK;
	reg &= 0xFFF;
	if (!ax5043_conf_valid(conf) || len - 2 > SPI_MAX_TRASNFER_SIZE) {
		return -AX5043_INVALID_PARAM;
	}
	const uint8_t mask = BIT(7) | BIT(6) | BIT(5) | BIT(4);

	memset(__spi_tx, 0, len + 2);

	/* Prepare the long access header AND9347 p.5 */
	__spi_tx[0] =  mask | (~mask & (reg >> 8));
	__spi_tx[1] = reg & 0xFF;

	memcpy(__spi_tx + 2, in, len);
	conf->spi_select(1);
	ret = conf->write(__spi_rx, __spi_tx, len + 2);
	conf->spi_select(0);
	return ret;
}

int
ax5043_spi_write_8(struct ax5043_conf *conf, uint16_t reg,
                   uint8_t in)
{
	int ret;
	reg &= 0xFFF;
	conf->spi_select(1);
	if (reg > 0xFF) {
		const uint8_t mask = BIT(7) | BIT(6) | BIT(5) | BIT(4);
		/* Long register access based on AND9347 p.5 */
		uint8_t req[3] = {mask | (~mask & (reg >> 8)), reg & 0xFF, in};
		uint8_t res[3] = {0x0, 0x0, 0x0};
		ret = conf->write(res, req, 3);
	} else {
		/* Short register access based on AND9347 p.5 */
		uint8_t req[2] = {BIT(7) | (0x7F & reg), in};
		uint8_t res[2] = {0x0, 0x0};
		ret = conf->write(res, req, 2);
	}
	conf->spi_select(0);
	return ret;
}

int
ax5043_spi_write_16(struct ax5043_conf *conf, uint16_t reg,
                    uint16_t in)
{
	reg &= 0xFFF;
	const uint8_t mask = BIT(7) | BIT(6) | BIT(5) | BIT(4);
	/* Prepare the long access header AND9347 p.5 */
	uint8_t req[4] = {mask | (~mask & (reg >> 8)), reg & 0xFF, (in >> 8), in};
	uint8_t res[4];
	conf->spi_select(1);
	int ret = conf->write(res, req, 4);
	conf->spi_select(0);
	return ret;
}

int
ax5043_spi_write_24(struct ax5043_conf *conf, uint16_t reg,
                    uint32_t in)
{
	reg &= 0xFFF;
	const uint8_t mask = BIT(7) | BIT(6) | BIT(5) | BIT(4);
	/* Prepare the long access header AND9347 p.5 */
	uint8_t req[5] = {mask | (~mask & (reg >> 8)), reg & 0xFF, (in >> 16),
	                  (in >> 8), in
	                 };
	uint8_t res[5];
	conf->spi_select(1);
	int ret = conf->write(res, req, 5);
	conf->spi_select(0);
	return ret;
}

int
ax5043_spi_write_32(struct ax5043_conf *conf, uint16_t reg,
                    uint32_t in)
{
	reg &= 0xFFF;
	const uint8_t mask = BIT(7) | BIT(6) | BIT(5) | BIT(4);
	/* Prepare the long access header AND9347 p.5 */
	uint8_t req[6] = {mask | (~mask & (reg >> 8)), reg & 0xFF, (in >> 24),
	                  (in >> 16), (in >> 8), in
	                 };
	uint8_t res[6];
	conf->spi_select(1);
	int ret = conf->write(res, req, 6);
	conf->spi_select(0);
	return ret;
}

/**
 * Sets properly some undocumented TX registers
 * @param conf  the AX5043 configuration handler
 * @return 0 on success or appropriate negative error code
 */
static inline int
set_tx_black_magic_regs(struct ax5043_conf *conf)
{
	int ret;
	ret = ax5043_spi_write_8(conf, 0xF00, 0x0F);
	if (ret) {
		return ret;
	}

	ret = ax5043_spi_write_8(conf, 0xF0C, 0x0);
	if (ret) {
		return ret;
	}

	/*FIXME: This assumes that we always use a TCXO */
	ret = ax5043_spi_write_8(conf, 0xF11, 0x0);
	if (ret) {
		return ret;
	}

	ret = ax5043_spi_write_8(conf, 0xF1C, 0x07);
	if (ret) {
		return ret;
	}

	ret = ax5043_spi_write_8(conf, 0xF44, 0x24);
	if (ret) {
		return ret;
	}

	/* Dafuq? Got it from RadioLab */
	ret = ax5043_spi_write_8(conf, 0xF18, 0x06);
	return ret;
}

/**
 * Enables/Disables the power amplifier pin
 * @param conf the AX5043 configuration handler
 * @param enable 1 to enable 0 to disable
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_enable_pwramp(struct ax5043_conf *conf, uint8_t enable)
{
	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}
	return ax5043_spi_write_8(conf, AX5043_REG_PWRAMP, enable & 0x1);
}

/**
 * Controls the ANTSEL pin
 * @param conf the AX5043 configuration handler
 * @param weak_pullup set to 1 to enable the weak pullup.
 * @param invert set to 1 to revert the ANTSEL logic
 * @param pfantsel use the pfantsel_t enumeration to set properly the behavior
 * of the ANTSEL pin
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_set_antsel(struct ax5043_conf *conf, uint8_t weak_pullup, uint8_t invert,
                  pfantsel_t pfantsel)
{
	uint8_t b = 0x0;
	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}
	switch (pfantsel) {
		case ANTSEL_OUTPUT_0:
		case ANTSEL_OUTPUT_1:
		case ANTSEL_OUTPUT_BB_TUBE_CLK:
		case ANTSEL_OUTPUT_DAC:
		case ANTSEL_OUTPUT_DIVERSITY:
		case ANTSEL_OUTPUT_EXT_TCXO_EN:
		case ANTSEL_OUTPUT_TEST_OBS:
		case ANTSEL_OUTPUT_Z:
			b = pfantsel;
			b |= (invert & 0x1) << 6;
			b |= (weak_pullup & 0x1) << 7;
			break;
		default:
			return -AX5043_INVALID_PARAM;
	}
	return ax5043_spi_write_8(conf, AX5043_REG_PINFUNCANTSEL, b);
}

/**
 * Enables/Disables the power amplifier pin
 * @param conf the AX5043 configuration handler
 * @param enable 1 to enable 0 to disable
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_pwramp_control(struct ax5043_conf *conf, uint8_t enable)
{
	int ret;
	ret = ax5043_spi_write_8(conf, AX5043_REG_PWRAMP, enable & 0x1);
	return ret;
}

/**
 * Configures the IC for CW transmission
 * @param conf the AX5043 configuration handler
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_conf_cw(struct ax5043_conf *conf)
{
	struct tx_params p;
	/* Set modulation to ASK to emulate CW */
	p.mod = ASK_COHERENT;
	p.shaping = UNSHAPED;
	p.rf_out_mode = TXSE;
	p.pout_dBm = 16.0f;
	p.framing = RAW;
	p.raw.en_inv = 0;
	p.raw.en_diff = 0;
	p.raw.en_scrambler = 0;
	p.raw.en_manch = 0;
	p.raw.en_nosync = 0;
	p.baudrate = 19200;
	return ax5043_conf_tx(conf, &p);
}

int
ax5043_tx_cw(struct ax5043_conf *conf, uint32_t duration_ms)
{
	int ret = AX5043_OK;

	uint8_t data_cmd[4] =  { AX5043_FIFO_REPEATDATA_CMD,
	                         AX5043_FIFO_PKTSTART | AX5043_FIFO_RAW | AX5043_FIFO_NOCRC, 255, 0xFF
	                       };
	uint8_t val;
	uint32_t start = conf->millis();

	/* Wait for the previous frame to be transmitted */
	while (__tx_active) {
		ret++;
	}

	/* Set AX5043 to FULLTX mode */
	ret = ax5043_set_power_mode(conf, FULLTX);
	if (ret) {
		return ret;
	}

	ax5043_wait_xtal(conf, 100);

	/* Wait for the FIFO to become ready */
	val = 0;
	while (!val) {
		ax5043_spi_read_8(conf, &val, AX5043_REG_POWSTAT);
		/* Select only the modem power state */
		val &= AX5043_SVMODEM;
		if (conf->millis() - start > duration_ms) {
			ret = -AX5043_TIMEOUT;
			break;
		}
	}

	__tx_active = 1;
	while (conf->millis() - start < duration_ms) {
		uint16_t ffree = 0;
		while (ffree < 8) {
			ax5043_spi_read_16(conf, &ffree, AX5043_REG_FIFOFREE1);
		}
		ret = ax5043_spi_write(conf, AX5043_REG_FIFODATA, data_cmd, 4);
		ret = ax5043_spi_write_8(conf, AX5043_REG_FIFOSTAT, AX5043_FIFO_COMMIT_CMD);
	}
	__tx_frame_end(conf);
	return AX5043_OK;
}

/**
 * Returns the RSSI and the background RSSI
 * @param conf the AX5043 configuration handler
 * @param rssi pointer to store the RSSI
 * @param bgnrssi pointer to store the background RSSI
 * @return 0 on success or appropriate negative error code
 */
int
ax5043_rssi(struct ax5043_conf *conf, float *rssi, float *bgnrssi)
{
	int ret = AX5043_OK;
	if (!ax5043_ready(conf)) {
		return -AX5043_NOT_READY;
	}
	uint16_t val = 0;
	ret = ax5043_spi_read_16(conf, &val, AX5043_REG_RSSI);
	if (ret) {
		return ret;
	}
	*rssi = val >> 8;
	*bgnrssi = val & 0xFF;
	return AX5043_OK;
}

/**
 * The IRQ handler for the AX5043
 * @return 0 on success, or appropriate negative error code
 */
int
ax5043_irq_callback()
{
	int ret;
	uint8_t data_cmd[3] = { AX5043_FIFO_VARIABLE_DATA_CMD, 0, 0};
	size_t avail;
	size_t chunk_size;
	uint32_t tmp;
	uint16_t reg0 = 0x0;
	uint16_t reg1 = 0x0;

	if (!__ax5043_conf) {
		return AX5043_OK;
	}

	/* Read both IRQREQUEST1 & RADIOEVENTREQ1 at once, to save some cycles */
	ret = ax5043_spi_read_32(__ax5043_conf, &tmp, AX5043_REG_IRQREQUEST1);
	if (ret) {
		return ret;
	}
	reg0 = tmp >> 16;
	reg1 = tmp & 0xFFFF;


	/* TX is done! */
	if (reg1 & AX5043_REVMDONE) {
		__tx_frame_end(__ax5043_conf);
		return AX5043_OK;
	}

	/* If FIFO has free space fill in data */
	if (reg0 & AX5043_IRQRFIFOTHRFREE) {
		/* Always left some space for the postamble for a simplified logic */
		avail = min(
		                AX5043_FIFO_FREE_THR - sizeof(data_cmd) - sizeof(__postamble_cmd),
		                __tx_remaining);
		data_cmd[1] = avail + 1;
		chunk_size = sizeof(data_cmd) + avail;
		memcpy(__tx_fifo_chunk + sizeof(data_cmd), __tx_buf + __tx_buf_idx, avail);

		if (avail == __tx_remaining) {
			data_cmd[2] = AX5043_FIFO_PKTEND;
			memcpy(__tx_fifo_chunk + chunk_size,
			       __postamble_cmd, sizeof(__postamble_cmd));
			chunk_size += sizeof(__postamble_cmd);
			/* Mask the FIFO free IRQ as it is not needed anymore */
			tx_fifo_irq_enable(__ax5043_conf, 0);
		}
		memcpy(__tx_fifo_chunk, data_cmd, sizeof(data_cmd));
		ax5043_spi_write(__ax5043_conf, AX5043_REG_FIFODATA, __tx_fifo_chunk,
		                 chunk_size);
		/* Commit to FIFO ! */
		ret = ax5043_spi_write_8(__ax5043_conf, AX5043_REG_FIFOSTAT,
		                         AX5043_FIFO_COMMIT_CMD);

		__tx_remaining -= avail;
		__tx_buf_idx += avail;
	}
	return AX5043_OK;
}

