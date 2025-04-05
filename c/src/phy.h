/**
 * @file
 * @brief   Physical layer code for modem
 *
 * This file handles transmission/reception of data frames over the air. It uses fsk.c to
 * perform baseband IQ modulation and demodulation, and libbladeRF to transmit/receive
 * samples using the bladeRF device. A different modulator could be used by swapping
 * fsk.c with a file that implements a different modulator. On the receive side the file
 * uses fir_filter.c to low-pass filter the received signal, pnorm.c to power normalize
 * the input signal, and correlator.c to correlate the received signal with a preamble.
 * waveform.
 *
 * The structure of a physical layer transmission is as follows:
 * ```
 *    / ramp up | training sequence | preamble | link layer frame | ramp down \
 * ```
 * The ramps at the beginning/end are SAMP_PER_SYMB samples long
 *
 * This file is part of the bladeRF-fsk project
 *
 * Copyright (C) 2016 Nuand LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef PHY_H
#define PHY_H

#include <stdbool.h>
#include <libbladeRF.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>

#include "common.h"
#include "utils.h"
#include "link.h" //for link layer frame type definitions

//Training sequence which goes at the start of every frame
#define TRAINING_SEQ        {0xAA, 0xAA, 0xAA, 0xAA}
//Length of training sequence in bytes
#define TRAINING_SEQ_LENGTH 4
//preamble which goes after the training sequence
#define PREAMBLE            {0x2E, 0x69, 0x2C, 0xF0}
//Length of preamble in bytes
#define PREAMBLE_LENGTH     4
//Seed for pseudorandom number sequence generator
#define PRNG_SEED           0x0109BBA53CFFD081
//Length (in samples) of ramp up/ramp down
#define RAMP_LENGTH         SAMP_PER_SYMB
//Number of samples to receive at a time from bladeRF
#define NUM_SAMPLES_RX      SYNC_BUFFER_SIZE
//Correlator countdown size
#define CORR_COUNTDOWN      SAMP_PER_SYMB
//power normalization alpha coefficient - how slowly it responds to changes in power
#define PNORM_ALPHA         0.95f
#define PNORM_MIN_GAIN      0.1f
#define PNORM_MAX_GAIN      50.0f
//--FSK waveform properties
//Set for phase modulation index = pi/2 (1/4 turn per symbol)
//NOTE: If you change these two macros, you'll need to change the FIR channel filter as
//well if you want the modem to work properly
#define SAMP_PER_SYMB       8    //TX samples per symbol
#define SYMB_PER_REV        4    //Symbols per revolution around IQ unit circle
                                 //=2pi/modulation_index. Our modulation_index = pi/2
//RX decimation factor after FIR filter. SAMP_PER_SYMB must be a multiple of this.
#define RX_DEC_FACT         2

//DEBUG: Define this to write all RX samples out to binary file rx_samples_[serial].bin
// #define LOG_RX_SAMPLES
//DEBUG: Define this to log filtered/normalized samples instead of raw samples when
//       LOG_RX_SAMPLES is defined
// #define LOG_RX_SAMPLES_USE_PNORM
//DEBUG: Define this to write all TX samples out to binary file tx_samples_[serial].bin
// #define LOG_TX_SAMPLES
//DEBUG: Define this to replace TX frame transmissions with a DC tone
// #define TX_DC_TONE
//DEBUG: Define this to use a binary file for RX input samples, in_rx_samples_[serial].bin
//instead of actual samples returned by bladerf_sync_rx()
// #define RX_SAMPLES_FROM_FILE


struct phy_handle;

//----------------------Transmitter functions---------------------------
/**
 * Start the PHY transmitter thread
 * 
 * @param[in]   phy     pointer to phy_handle struct
 *
 * @return      0 on success, -1 on failure
 */
int phy_start_transmitter(struct phy_handle *phy);

/**
 * Stop the PHY transmitter thread
 * 
 * @param[in]   phy     pointer to phy_handle struct
 *
 * @return      0 on success, -1 on failure
 */
int phy_stop_transmitter(struct phy_handle *phy);

/**
 * Fill the tx buffer so its data will be transmitted using phy_transmit_frames()
 *
 * @param[in]   phy         pointer to phy handle structure
 * @param[in]   data_buf    bytes to transmit
 * @param[in]   length      length of data buf
 *
 * @return      0 on success, -1 on failure
 */
int phy_fill_tx_buf(struct phy_handle *phy, uint8_t *data_buf, unsigned int length);

//------------------------Receiver functions---------------------------
/**
 * Start the PHY receiver  thread
 * 
 * @param[in]   phy     pointer to phy_handle struct
 *
 * @return      0 on success, -1 on failure
 */
int phy_start_receiver(struct phy_handle *phy);

/**
 * Stop the PHY receiver thread
 * 
 * @param[in]   phy     pointer to phy_handle struct
 *
 * @return      0 on success,-1 on failure,
 *              1 if RX overruns were experienced (but otherwise successful)
 */
int phy_stop_receiver(struct phy_handle *phy);

/**
 * Request a received frame from phy_receive_frames(). Caller should call
 * phy_release_rx_buf() when done with the received frame so that 
 * phy_receive_frames() does not drop any frames.
 *
 * @param[in]   phy             pointer to phy_handle struct
 * @param[in]   timeout_ms      amount of time to wait for a buffer from the PHY
 *
 * @return      pointer to filled buffer with received frame inside on success,
 *              NULL on failure/timeout
 */
uint8_t *phy_request_rx_buf(struct phy_handle *phy, unsigned int timeout_ms);

/**
 * Release RX buffer so that phy_receive_frames() can copy new frames into the buffer
 *
 * @param[in]   phy     pointer to phy_handle struct
 */
void phy_release_rx_buf(struct phy_handle *phy);

//-----------------------Init/Deinit functions-------------------------
/**
 * Open/Initialize a phy_handle
 * 
 * @param[in]   dev              pointer to opened bladeRF device handle
 * @param[in]   params           pointer to radio parameters 
 * @param[in]   max_frame_size   maximum data frame size to transmit/receive
 *
 * @return      allocated phy_handle on success, NULL on failure
 */
struct phy_handle *phy_init(struct bladerf *dev, struct radio_params *params,
                            unsigned int max_frame_size);

/**
 * Close a phy handle. Does nothing if handle is NULL
 *
 * @param[in]   phy_handle to close
 */
void phy_close(struct phy_handle *phy);

#endif
