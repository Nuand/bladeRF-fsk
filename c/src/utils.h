/**
 * @file
 * @brief   Utility functions: loading/saving samples from/to a file,
 *          conversions, etc.
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

#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#if BLADERF_OS_WINDOWS || BLADERF_OS_OSX
    #include "clock_gettime.h"
#else
    #include <time.h>
#endif

#include "common.h"

/**
 * Write samples to csv file
 *
 * @param[in]   filename        name of csv file to write
 * @param[in]   samples         buffer containing IQ samples to write
 * @param[in]   num_samples     number of IQ samples to write to the file
 *
 * @return      0 on success, -1 on failure
 */
int write_samples_to_csv_file(char *filename, int16_t *samples, int num_samples);

/**
 * Load samples from csv file into heap memory buffer. Caller is responsible for
 * freeing the the buffer when done using it.
 *
 * @param[in]   filename        file name to load samples from
 * @param[in]   pad_zeros       If true, the function will pad zeros onto the end until
 *                              the samples fill to a multiple of 'buffer_size' samples
 * @param[in]   buffer_size     buffer size to fill multiples of, if pad_zeros is true.
 *                              Parameter ignored if pad_zeros is false.
 * @param[out]  samples         pointer to the loaded sample buffer. Buffer set to NULL
 *                              on failure. If successful, the buffer must be freed when
 *                              done using it.
 * @return      number of samples loaded on success; -1 on failure
 */
int load_samples_from_csv_file(char *filename, bool pad_zeros, int buffer_size,
                               int16_t **samples);

/**
 * Write 'struct complex_sample' samples to csv file
 *
 * @param[in]   filename        name of csv file to write
 * @param[in]   samples         buffer containing IQ samples to write
 * @param[in]   num_samples     number of IQ samples to write to the file
 *
 * @return      0 on success, -1 on failure
 */
int write_struct_samples_to_csv_file(char *filename, struct complex_sample *samples,
                                     int num_samples);

/**
 * Load 'struct complex_sample' samples from csv file into heap memory buffer.
 * Caller is responsible for freeing the the buffer when done using it.
 *
 * @param[in]   filename        file name to load samples from
 * @param[in]   pad_zeros       If true, the function will pad zeros onto the end until
 *                              the samples fill to a multiple of 'buffer_size' samples
 * @param[in]   buffer_size     buffer size to fill multiples of, if pad_zeros is true.
 *                              Parameter ignored if pad_zeros is false.
 * @param[out]  samples         pointer to the loaded sample buffer. Buffer set to NULL
 *                              on failure. If successful, the buffer must be freed when
 *                              done using it.
 * @return      number of samples loaded on success; -1 on failure
 */
int load_struct_samples_from_csv_file(char *filename, bool pad_zeros, int buffer_size,
                                      struct complex_sample **samples);

void print_chars(uint8_t *data, int num_bytes);
void print_hex_contents(uint8_t *data, int num_bytes);

/**
 * Converts a relative timeout to an absolute time (i.e. 5000 ms to 07/02/2016 20:32:10)
 *
 * @param[in]   timeout_ms      the timeout to create in ms
 * @param[out]  timeout_abs     pointer to timespec struct to fill
 *
 * @return      0 on success, -1 on failure
 */
int create_timeout_abs(unsigned int timeout_ms, struct timespec *timeout_abs);


struct numeric_suffix {
    const char *suffix;
    uint64_t multiplier;
};

/**
 * Convert a string to an LNA gain value
 * @param str String to convert
 * @param gain Pointer to store the converted gain value
 * @return 0 on success, -1 on failure
 */
int str2lnagain(const char *str, bladerf_lna_gain *gain);

/**
 * Convert a string to an integer value
 * @param str String to convert
 * @param min Minimum allowed value
 * @param max Maximum allowed value
 * @param valid Pointer to store validity status
 * @return Converted value
 */
int str2int(const char *str, int min, int max, bool *valid);

/**
 * Convert a string to an unsigned integer value
 * @param str String to convert
 * @param min Minimum allowed value
 * @param max Maximum allowed value
 * @param valid Pointer to store validity status
 * @return Converted value
 */
unsigned int str2uint(const char *str, unsigned int min, unsigned int max,
                    bool *valid);
/**
 * Convert a string with optional suffix to a uint64_t value
 * @param str String to convert
 * @param min Minimum allowed value
 * @param max Maximum allowed value
 * @param suffixes Array of valid suffixes and their multipliers
 * @param num_suffixes Number of elements in suffixes array
 * @param valid Set to true if conversion was successful
 * @return Converted value, or 0 if conversion failed
 */
uint64_t str2uint64_suffix(const char *str, uint64_t min, uint64_t max,
                                const struct numeric_suffix *suffixes,
                                size_t num_suffixes, bool *valid);
#endif
