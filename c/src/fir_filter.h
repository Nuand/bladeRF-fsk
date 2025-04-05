/**
 * @file
 * @brief   Filters the input signal with the given FIR filter coefficients, applying
 *          optional downsampling when dec_factor>1. Only computes the outputs that will
 *          not be thrown out by downsampling.
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
#ifndef FIR_FILTER_H_
#define FIR_FILTER_H_

#include "common.h"

struct fir_filter;

/**
 * Construct a FIR filter with provided taps and decimation factor
 *
 * @param[in]   taps        Filter taps.
 * @param[in]   num_tamps   Number of taps contained within `taps`
 * @param[in]   dec_factor  Decimation factor to apply to filtered output signal
 *
 * @return      `fir_filter` handle on success,
 *              or NULL on failure or invalid parameter
 */
struct fir_filter * fir_init(const float *taps, size_t num_taps, unsigned int dec_factor);

/**
 * Deinitialize and deallocate the provided filter
 *
 * @param   filt        Filter to deinitialize
 */
void fir_deinit(struct fir_filter *filt);

/**
 * Perform filter operation over the provided samples
 *
 * @param[in]   filt        Filter handle to use
 *
 * @param[in]   input       Input SC16Q11 samples, I before Q in each pair of int16_t's
 * @param[out]  output      Output SC16Q11 samples
 * @param[in]   count       Number samples to process
 *
 * @return      number of output samples produced (<count if dec_factor>1)
 */
unsigned int fir_process(struct fir_filter *filt, int16_t *input,
                         struct complex_sample *output, size_t count);

#endif
