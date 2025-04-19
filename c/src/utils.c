/**
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

#include <string.h>
#include <stdbool.h>
#include <limits.h>
#include <getopt.h>
#include <sys/stat.h>
#include <errno.h>
#include <ctype.h>
#include "utils.h"

#ifdef _WIN32
#include <windows.h>
#elif defined(__APPLE__)
#include <mach/mach_time.h>
#include <mach/clock.h>
#include <mach/mach.h>
#endif

int load_samples_from_csv_file(char *filename, bool pad_zeros, int buffer_size,
                                int16_t **samples)
{
    int i, result, num_samples;
    int samples_buf_size;        //Max number of samples in buffer
    int buf_inc_size;            //Number of additional samples to add space for on a realloc()
    bool reachedEOF;
    int16_t temp_sample1, temp_sample2;
    int16_t *tmp_ptr;

    //Open csv file
    FILE *fp = fopen(filename, "r");
    if (fp == NULL){
        perror("fopen");
        *samples = NULL;
        return -1;
    }

    if (pad_zeros){
        buf_inc_size = buffer_size;
    }else{
        buf_inc_size = 1024;
    }

    //Allocate initial memory for tx_samples
    *samples = (int16_t *) malloc(buf_inc_size * 2 * sizeof(int16_t));
    if (*samples == NULL){
        perror("malloc");
        fclose(fp);
        return -1;
    }

    //Initial size of allocated memory
    samples_buf_size = buf_inc_size;

    //Read in samples, and reallocate more memory if needed
    i = 0;
    num_samples = 0;
    reachedEOF = false;
    while(true){
        //Check to see if the samples buffer is full
        if (num_samples == samples_buf_size){
            //Before allocating more memory, check to see if there are no more samples
            result = fscanf(fp, "%hi,%hi\n", &temp_sample1, &temp_sample2 );
            if(result == EOF || reachedEOF){
                break;
            }
            //allocate more memory. Add space for another 'buf_inc_size' samples
            samples_buf_size += buf_inc_size;
            tmp_ptr = (int16_t *) realloc(*samples, samples_buf_size * 2 * sizeof(int16_t));
            if (tmp_ptr == NULL){
                perror("realloc");
                free(*samples);
                *samples = NULL;
                fclose(fp);
                return -1;
            }
            *samples = tmp_ptr;
            (*samples)[i] = temp_sample1;
            (*samples)[i+1] = temp_sample2;
            num_samples++;
            i += 2;
            continue;
        }
        if (!reachedEOF){
            result = fscanf(fp, "%hi,%hi\n", &(*samples)[i], &(*samples)[i+1] );
            if (result == EOF){
                reachedEOF = true;
            }
        }
        if (reachedEOF){
            if (pad_zeros){
                //pad with zeros to fill the rest of the buffer
                (*samples)[i] = 0;
                (*samples)[i+1] = 0;
            }else{
                break;
            }
        }
        num_samples++;
        i += 2;        //increment index
    }
    fclose(fp);

    return num_samples;
}

int load_struct_samples_from_csv_file(char *filename, bool pad_zeros, int buffer_size,
                                        struct complex_sample **samples)
{
    int i, result;
    int samples_buf_size;        //Max number of samples in buffer
    int buf_inc_size;            //Number of additional samples to add space for on a realloc()
    bool reachedEOF;
    int16_t temp_sample1, temp_sample2;
    struct complex_sample *tmp_ptr;

    //Open csv file
    FILE *fp = fopen(filename, "r");
    if (fp == NULL){
        perror("fopen");
        *samples = NULL;
        return -1;
    }

    if (pad_zeros){
        buf_inc_size = buffer_size;
    }else{
        buf_inc_size = 1024;
    }

    //Allocate initial memory for tx_samples
    *samples = (struct complex_sample *) malloc(buf_inc_size * sizeof(struct complex_sample));
    if (*samples == NULL){
        perror("malloc");
        fclose(fp);
        return -1;
    }

    //Initial size of allocated memory
    samples_buf_size = buf_inc_size;

    //Read in samples, and reallocate more memory if needed
    i = 0;
    reachedEOF = false;
    while(true){
        //Check to see if the samples buffer is full
        if (i == samples_buf_size){
            //Before allocating more memory, check to see if there are no more samples
            result = fscanf(fp, "%hi,%hi\n", &temp_sample1, &temp_sample2 );
            if(result == EOF || reachedEOF){
                break;
            }
            //allocate more memory. Add space for another 'buf_inc_size' samples
            samples_buf_size += buf_inc_size;
            tmp_ptr = (struct complex_sample *) realloc(*samples,
                                       samples_buf_size * sizeof(struct complex_sample));
            if (tmp_ptr == NULL){
                perror("realloc");
                free(*samples);
                *samples = NULL;
                fclose(fp);
                return -1;
            }
            *samples = tmp_ptr;
            (*samples)[i].i = temp_sample1;
            (*samples)[i].q = temp_sample2;
            i++;
            continue;
        }
        if (!reachedEOF){
            result = fscanf(fp, "%hi,%hi\n", &(*samples)[i].i, &(*samples)[i].q);
            if (result == EOF){
                reachedEOF = true;
            }
        }
        if (reachedEOF){
            if (pad_zeros){
                //pad with zeros to fill the rest of the buffer
                (*samples)[i].i = 0;
                (*samples)[i].q = 0;
            }else{
                break;
            }
        }
        i++;        //increment index
    }
    fclose(fp);

    return i;
}

int write_samples_to_csv_file(char *filename, int16_t *samples, int num_samples)
{
    int i;
    //Open file
    FILE *fp = fopen(filename, "w");
    if (fp == NULL){
        perror("fopen");
        return -1;
    }
    //Write file
    //Loop through all samples
    //First int is the I sample. Second int is the Q sample
    for (i = 0; i < num_samples*2; i+= 2){
        fprintf(fp, "%hi,%hi\n", samples[i], samples[i+1]);
    }
    fclose(fp);

    return 0;
}

int write_struct_samples_to_csv_file(char *filename, struct complex_sample *samples,
                                        int num_samples)
{
    int i;
    //Open file
    FILE *fp = fopen(filename, "w");
    if (fp == NULL){
        perror("fopen");
        return -1;
    }
    //Write file
    //Loop through all samples
    //First int is the I sample. Second int is the Q sample
    for (i = 0; i < num_samples; i++){
        fprintf(fp, "%hi,%hi\n", samples[i].i, samples[i].q);
    }
    fclose(fp);

    return 0;
}

void print_chars(uint8_t *data, int num_bytes)
{
    int i;

    printf("'");
    for (i = 0; i < num_bytes; i++){
        printf("%c", data[i]);
    }
    printf("'\n");
}

void print_hex_contents(uint8_t *data, int num_bytes)
{
    int i;

    for (i = 0; i < num_bytes; i++){
        printf("%.2X ", data[i]);
    }
    printf("\n");
}

int create_timeout_abs(unsigned int timeout_ms, struct timespec *timeout_abs)
{
    int status;

#ifdef _WIN32
    // Windows implementation
    FILETIME ft;
    ULARGE_INTEGER uli;
    uint64_t nanoseconds;

    // Get current time in 100-nanosecond intervals since January 1, 1601
    GetSystemTimeAsFileTime(&ft);
    uli.LowPart = ft.dwLowDateTime;
    uli.HighPart = ft.dwHighDateTime;

    // Convert to nanoseconds since Unix epoch (January 1, 1970)
    // Windows epoch is 116444736000000000 100-ns intervals before Unix epoch
    nanoseconds = (uli.QuadPart - 116444736000000000ULL) * 100;

    // Convert to seconds and nanoseconds
    timeout_abs->tv_sec = (time_t)(nanoseconds / 1000000000ULL);
    timeout_abs->tv_nsec = (long)(nanoseconds % 1000000000ULL);
#elif defined(__APPLE__)
    // macOS implementation
    clock_serv_t cclock;
    mach_timespec_t mts;

    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);

    timeout_abs->tv_sec = mts.tv_sec;
    timeout_abs->tv_nsec = mts.tv_nsec;
#else
    // Linux implementation
    status = clock_gettime(CLOCK_REALTIME, timeout_abs);
    if(status != 0){
        perror("clock_gettime");
        return -1;
    }
#endif

    //Add the timeout onto the current time
    timeout_abs->tv_sec  += timeout_ms/1000;
    timeout_abs->tv_nsec += (timeout_ms % 1000) * 1000000;
    //Check for overflow in nsec
    if (timeout_abs->tv_nsec >= 1000000000){
        timeout_abs->tv_sec  += timeout_abs->tv_nsec / 1000000000;
        timeout_abs->tv_nsec %= 1000000000;
    }

    return 0;
}

int str2lnagain(const char *str, bladerf_lna_gain *gain)
{
    if (strcasecmp(str, "bypass") == 0) {
        *gain = BLADERF_LNA_GAIN_BYPASS;
    } else if (strcasecmp(str, "mid") == 0) {
        *gain = BLADERF_LNA_GAIN_MID;
    } else if (strcasecmp(str, "max") == 0) {
        *gain = BLADERF_LNA_GAIN_MAX;
    } else {
        return -1;
    }
    return 0;
}

int str2int(const char *str, int min, int max, bool *valid)
{
    char *endptr;
    long value;

    *valid = false;

    errno = 0;
    value = strtol(str, &endptr, 0);
    if (errno != 0 || endptr == str || *endptr != '\0') {
        return 0;
    }

    if (value < min || value > max) {
        return 0;
    }

    *valid = true;
    return (int)value;
}

unsigned int str2uint(const char *str, unsigned int min, unsigned int max,
                    bool *valid)
{
    char *endptr;
    unsigned long value;

    *valid = false;

    errno = 0;
    value = strtoul(str, &endptr, 0);
    if (errno != 0 || endptr == str || *endptr != '\0') {
        return 0;
    }

    if (value < min || value > max) {
        return 0;
    }

    *valid = true;
    return (unsigned int)value;
}

uint64_t str2uint64_suffix(const char *str, uint64_t min, uint64_t max,
                                const struct numeric_suffix *suffixes,
                                size_t num_suffixes, bool *valid)
{
    const char *p = str;
    uint64_t int_part = 0;
    uint64_t frac_part = 0;
    int frac_digits = 0;
    bool has_decimal = false;
    uint64_t multiplier = 1;
    uint64_t value = 0;
    bool negative = false; // Although target is uint64_t, check for invalid input
    size_t i;

    *valid = false;

    // Skip leading whitespace
    while (isspace((unsigned char)*p)) {
        p++;
    }

    // Check for sign (though result must be positive)
    if (*p == '-') {
        negative = true;
        p++;
    } else if (*p == '+') {
        p++;
    }

    // Parse integer part
    while (isdigit((unsigned char)*p)) {
        if (int_part > (UINT64_MAX - (*p - '0')) / 10) return 0; // Overflow check
        int_part = int_part * 10 + (*p - '0');
        p++;
    }

    // Parse fractional part
    if (*p == '.') {
        has_decimal = true;
        p++;
        uint64_t frac_scale = 1;
        while (isdigit((unsigned char)*p)) {
            if (frac_digits < 18) { // Limit fractional digits to avoid overflow later
                 if (frac_part > (UINT64_MAX - (*p - '0')) / 10) return 0; // Overflow
                 if (frac_scale > UINT64_MAX / 10) return 0; // Overflow

                frac_part = frac_part * 10 + (*p - '0');
                frac_scale *= 10;
                frac_digits++;
            } else {
                // Ignore excessive fractional digits
            }
            p++;
        }
    }

    // Skip trailing whitespace before suffix
    while (isspace((unsigned char)*p)) {
        p++;
    }

    // Check for suffix
    if (*p != '\0') {
        bool suffix_found = false;
        for (i = 0; i < num_suffixes; i++) {
            if (strcasecmp(p, suffixes[i].suffix) == 0) {
                multiplier = suffixes[i].multiplier;
                suffix_found = true;
                break;
            }
        }
        if (!suffix_found) {
            fprintf(stderr, "Error: Invalid suffix: %s\n", p);
            return 0; // Invalid suffix
        }
    }

    // Combine parts and apply multiplier
    if (multiplier > UINT64_MAX / int_part) return 0; // Overflow check
    value = int_part * multiplier;

    if (has_decimal && frac_digits > 0) {
        uint64_t scaled_frac_multiplier = multiplier;
        uint64_t divisor = 1;
        // Calculate divisor = 10^frac_digits safely
        for (int k = 0; k < frac_digits; ++k) {
            if (divisor > UINT64_MAX / 10) { // Overflow calculating divisor
                 fprintf(stderr, "Error: Too many fractional digits for multiplier.\n");
                 return 0;
            }
            divisor *= 10;
        }

        // Check if multiplier is divisible by divisor
        if (multiplier % divisor != 0) {
             fprintf(stderr, "Warning: Suffix multiplier not divisible by 10^%d, truncating fractional part.\n", frac_digits);
             // Proceed with integer division, effectively truncating
        }
        scaled_frac_multiplier /= divisor;


        if (scaled_frac_multiplier > 0 && frac_part > UINT64_MAX / scaled_frac_multiplier) return 0; // Overflow
        uint64_t frac_contribution = frac_part * scaled_frac_multiplier;

        if (value > UINT64_MAX - frac_contribution) return 0; // Overflow
        value += frac_contribution;
    }

    // Final checks
    if (negative) {
        fprintf(stderr, "Error: Negative value provided for unsigned type.\n");
        return 0; // Negative value invalid
    }

    if (value < min || value > max) {
        fprintf(stderr, "Error: Value %llu is out of range [%llu, %llu]\n", (unsigned long long)value, (unsigned long long)min, (unsigned long long)max);
        return 0;
    }

    *valid = true;
    return value;
}
