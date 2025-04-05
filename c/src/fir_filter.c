/**
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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "fir_filter.h"

#ifdef DEBUG_MODE
#   define DEBUG_MSG(...) fprintf(stderr, "[FIR] " __VA_ARGS__)
#else
#   define DEBUG_MSG(...)
#endif

struct fir_filter {

    float *taps;        /* Filter taps */
    size_t length;      /* Length of the filter, in taps */

    /* Insertion points into "shift register" implementation that utilizes
     * two copies of data to achieve a contiguious shifting window. */
    size_t ins1;
    size_t ins2;

    /* Filter state buffers */
    int32_t *i;
    int32_t *q;

    //Decimation factor. After filtering, the output signal is downsampled by this factor
    //to produce decimation. Filter computations intelligently avoid computing outputs
    //that will be thrown out by downsampling.
    unsigned int dec_factor;

    //Filter output decimation phase: 0 to dec_factor-1
    //only phase 0 outputs are computed, the rest are dropped by downsampling
    unsigned int dec_phase;
};

static void fir_reset(struct fir_filter *filt)
{
    size_t n;

    for (n = 0; n < (2 * filt->length); n++) {
        filt->i[n] = 0;
        filt->q[n] = 0;
    }

    /* Remember, state is 2 copies: array len is 2 * filt->length elements */
    filt->ins1      = 0;
    filt->ins2      = filt->length;

    filt->dec_phase = 0;
}

void fir_deinit(struct fir_filter *filt)
{
    if (filt) {
        free(filt->q);
        free(filt->i);
        free(filt->taps);
        free(filt);
    }
}

struct fir_filter * fir_init(const float *taps, size_t length, unsigned int dec_factor)
{
    struct fir_filter *filt;

    filt = calloc(1, sizeof(filt[0]));
    if (!filt) {
        perror("calloc");
        return NULL;
    }

    /* Filter state is a sliding window implemented with 2 copies of data */
    filt->i = calloc(2 * length, sizeof(filt->i[0]));
    if (!filt->i) {
        perror("calloc");
        fir_deinit(filt);
        return NULL;
    }

    filt->q = calloc(2 * length, sizeof(filt->q[0]));
    if (!filt->i) {
        perror("calloc");
        fir_deinit(filt);
        return NULL;
    }

    filt->taps = calloc(length, sizeof(filt->taps[0]));
    if (!filt->taps) {
        perror("calloc");
        fir_deinit(filt);
        return NULL;
    }

    memcpy(filt->taps, taps, length * sizeof(filt->taps[0]));

    filt->length     = length;
    filt->dec_factor = dec_factor;

    fir_reset(filt);
    return filt;
}

unsigned int fir_process(struct fir_filter *f, int16_t *input,
                         struct complex_sample *output, size_t count)
{
    /* Index into input/output buffers */
    size_t in_idx;

    /* Index into f->taps array */
    size_t t;

    /* Index into f->i and f->q state arrays */
    size_t s;

    //index into output
    size_t out_idx = 0;

    for (in_idx = 0; in_idx < (2*count); in_idx += 2) {
        int32_t i        = input[in_idx];
        int32_t q        = input[in_idx+1];
        float   result_i = 0;
        float   result_q = 0;

        /* Insert samples */
        f->i[f->ins1] = f->i[f->ins2] = i;
        f->q[f->ins1] = f->q[f->ins2] = q;


        if (f->dec_phase == 0){
            /* Convolve */
            for (t = 0, s = f->ins2; t < f->length; t++, s--) {
                float tmp_i, tmp_q;

                tmp_i = f->taps[t] * f->i[s];
                tmp_q = f->taps[t] * f->q[s];

                result_i += tmp_i;
                result_q += tmp_q;
            }

            /* Update output sample */
            output[out_idx].i = (int16_t) roundf(result_i);
            output[out_idx].q = (int16_t) roundf(result_q);

            out_idx++;
        }

        /* Advance insertion points */
        f->ins2++;
        if (f->ins2 == (2 * f->length)) {
            f->ins1 = 0;
            f->ins2 = f->length;
        } else {
            f->ins1++;
        }

        //update decimation phase
        if (f->dec_phase == f->dec_factor-1){
            f->dec_phase = 0;
        }else{
            f->dec_phase++;
        }
    }
    return out_idx;
}

#ifdef FIR_FILTER_TEST
#include <stdbool.h>
#include "rx_ch_filter.h"
#include "utils.h"

int main(int argc, char *argv[])
{
    int status              = EXIT_FAILURE;
    FILE *infile            = NULL;
    FILE *outfile           = NULL;

    int16_t *inbuf = NULL;
    struct complex_sample *outbuf = NULL;

    struct fir_filter *filt = NULL;

    static const unsigned int max_chunk_size = 1024 * 1024 * 1024;

    unsigned int chunk_size = 4096;
    size_t n_read, n_written;
    bool done = false;

    if (argc < 3 || argc > 4) {
        fprintf(stderr,
                "Filter sc16q11 samples from <infile> and write"
                "them to <outfile>.\n\n");

        fprintf(stderr,
                "Usage: %s <infile> <outfile> [# chunk size(samples)]\n",
                argv[0]);

        return EXIT_FAILURE;
    }

    if (argc == 4) {
        bool valid;
        chunk_size = str2uint(argv[3], 1, max_chunk_size, &valid);
        if (!valid) {
            fprintf(stderr, "Invalid chunk size: %s samples\n", argv[3]);
            return EXIT_FAILURE;
        }
    }

    filt = fir_init(rx_ch_filter, rx_ch_filter_len, 1);
    if (!filt) {
        fprintf(stderr, "Failed to allocate filter.\n");
        return EXIT_FAILURE;
    }

    inbuf = calloc(chunk_size, 2*sizeof(int16_t));
    if (!inbuf) {
        perror("calloc");
        goto out;
    }

    outbuf = calloc(chunk_size, sizeof(struct complex_sample));
    if (!outbuf) {
        perror("calloc");
        goto out;
    }

    infile = fopen(argv[1], "rb");
    if (!infile) {
        perror("Failed to open input file");
        goto out;
    }

    outfile = fopen(argv[2], "wb");
    if (!outfile) {
        perror("Failed to open output file");
        goto out;
    }

    while (!done) {
        n_read = fread(inbuf, 2*sizeof(int16_t), chunk_size, infile);
        done = n_read != chunk_size;

        fir_process(filt, inbuf, outbuf, n_read);

        n_written = fwrite((int16_t *)outbuf, 2*sizeof(int16_t), n_read, outfile);
        if (n_written != n_read) {
            fprintf(stderr, "Short write encountered. Exiting.\n");
            status = -1;
            goto out;
        }
    }

    status = 0;

out:
    if (infile) {
        fclose(infile);
    }

    if (outfile) {
        fclose(outfile);
    }
    free(inbuf);
    free(outbuf);
    fir_deinit(filt);

    return status;
}
#endif
