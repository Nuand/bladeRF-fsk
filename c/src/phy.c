/**
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
 * / ramp up | training sequence | preamble | link layer frame | ramp down \
 *
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
#include <assert.h>
#include "phy.h"
#include "fir_filter.h"
#include "rx_ch_filter.h"
#include "pnorm.h"
#include "prng.h"
#include "correlator.h"
#include "fsk.h"            //modulator/demodulator
#include "radio_config.h"    //bladeRF configuration

#ifdef DEBUG_MODE
    #define DEBUG_MSG(...) fprintf(stderr, "[PHY] " __VA_ARGS__)
    #ifndef ENABLE_NOTES
        #define ENABLE_NOTES
    #endif
#else
    #define DEBUG_MSG(...)
#endif

#ifdef ENABLE_NOTES
    #define NOTE(...) fprintf(stderr, "[PHY] " __VA_ARGS__)
#else
    #define NOTE(...)
#endif

#define ERROR(...) fprintf(stderr, "[PHY] " __VA_ARGS__)

//Internal structs
struct rx {
    int16_t               *in_samples;      //Raw input samples from device
    struct fir_filter     *ch_filt;         //Channel filter
    struct pnorm_state_t  *pnorm;           //Power normalizer
    struct correlator     *corr;            //Correlator
    struct complex_sample *filt_samples;    //Filtered input samples
    struct complex_sample *pnorm_samples[2];//power normalized samples
                                            //2 buffers - ping/pong to handle edge cases
                                            //in phy_receive_frames where past samples are
                                            //needed
    float                 *est_power;       //power estimate samples vector (from pnorm)
    uint8_t               *data_buf;        //received data output buffer (no training/preamble)
    bool                   buf_filled;      //is the rx data buffer filled
    bool                   stop;            //control variable to stop the receiver
    pthread_t              thread;          //pthread for the receiver
    pthread_cond_t         buf_filled_cond; //condition variable for buf_filled
    pthread_mutex_t        buf_status_lock; //mutex variable for accessing buf_filled
    bool                   overrun;         //True if receiver experienced RX sample overruns
    #ifdef LOG_RX_SAMPLES
        FILE              *samples_file;
    #endif
};
struct tx {
    uint8_t               *data_buf;      //input data to transmit (including training/preamble)
    unsigned int           data_length;   //length of data to transmit (not including preamble)
    bool                   buf_filled;
    bool                   stop;
    pthread_t              thread;
    pthread_cond_t         buf_filled_cond;
    pthread_mutex_t        buf_status_lock;
    unsigned int           max_num_samples; //Maximum number of tx samples to transmit
    struct complex_sample *samples;         //output samples to transmit
    #ifdef LOG_TX_SAMPLES
        FILE              *samples_file;
    #endif
};

struct phy_handle {
    struct bladerf    *dev;                 //bladeRF device handle
    struct fsk_handle *fsk;                 //fsk handle
    struct tx         *tx;                  //tx data structure
    struct rx         *rx;                  //rx data structure
    uint8_t           *scrambling_sequence;
    unsigned int       max_frame_size;      //maximum data frame size (not including preamble)
};

//Internal functions
void *phy_receive_frames(void *arg);
void *phy_transmit_frames(void *arg);
#ifndef BYPASS_PHY_SCRAMBLING
    static void scramble_frame(uint8_t *frame, int frame_length, uint8_t *scrambling_sequence);
    static void unscramble_frame(uint8_t *frame, int frame_length, uint8_t *scrambling_sequence);
#endif
static void create_ramps(unsigned int ramp_length, struct complex_sample ramp_down_init,
                    struct complex_sample *ramp_up, struct complex_sample *ramp_down);

/****************************************
 *                                      *
 *        INIT/DEINIT  FUNCTIONS        *
 *                                      *
 ****************************************/

struct phy_handle *phy_init(struct bladerf *dev, struct radio_params *params,
                            unsigned int max_frame_size)
{
    int                status;
    struct phy_handle *phy;
    uint64_t           prng_seed;
    uint8_t            preamble[PREAMBLE_LENGTH] = PREAMBLE;
    #if defined(LOG_RX_SAMPLES) || defined(LOG_TX_SAMPLES)
        char filename[15+BLADERF_SERIAL_LENGTH];
        struct bladerf_serial sn;
    #endif

    //Conversions between complex_sample and int16_t assume packed struct
    if (sizeof(struct complex_sample) != 2*sizeof(int16_t)){
        ERROR("Unexpected, unsupported padding in complex_sample structure");
        return NULL;
    }

    //------------Allocate memory for phy handle struct--------------
    //Calloc so all pointers are initialized to NULL
    phy = calloc(1, sizeof(struct phy_handle));
    if (phy == NULL){
        perror("malloc");
        return NULL;
    }

    DEBUG_MSG("Initializing...\n");

    if (dev == NULL){
        ERROR("%s: BladeRF device uninitialized", __FUNCTION__);
    }
    phy->dev = dev;

    //--------Initialize and configure bladeRF device-------------
    status = radio_init_and_configure(phy->dev, params);
    if (status != 0){
        ERROR("%s: Couldn't configure bladeRF: %s\n", __FUNCTION__,
                        bladerf_strerror(status));
        goto error;
    }
    DEBUG_MSG("BladeRF initialized and configured successfully\n");

    //-------------------Open fsk handle------------------------
    phy->fsk = fsk_init(SAMP_PER_SYMB*SYMB_PER_REV);
    if(phy->fsk == NULL){
        ERROR("%s: Couldn't open fsk handle\n", __FUNCTION__);
        goto error;
    }
    DEBUG_MSG("FSK Initialized\n");

    //------------------Initialize TX struct--------------------
    phy->tx = calloc(1, sizeof(struct tx));
    if (phy->tx == NULL){
        perror("[PHY] malloc");
        goto error;
    }
    //Allocate memory for tx data buffer
    phy->tx->data_buf = malloc(TRAINING_SEQ_LENGTH + PREAMBLE_LENGTH +
                               max_frame_size);
    if (phy->tx->data_buf == NULL){
        perror("[PHY] malloc");
        goto error;
    }
    //Allocate memory for tx samples buffer
    //2*RAMP_LENGTH for the ramp up/ramp down
    phy->tx->max_num_samples = 2*RAMP_LENGTH + (TRAINING_SEQ_LENGTH + PREAMBLE_LENGTH +
                               max_frame_size) * 8 * SAMP_PER_SYMB;
    #ifdef SYNC_NO_METADATA
        //Not using metadata mode; round up to multiple of SYNC_BUFFER_SIZE
        int rem = phy->tx->max_num_samples % SYNC_BUFFER_SIZE;
        if (rem != 0){
            phy->tx->max_num_samples = phy->tx->max_num_samples + (SYNC_BUFFER_SIZE - rem);
        }
    #endif

    phy->tx->samples = malloc(phy->tx->max_num_samples * sizeof(struct complex_sample));
    if (phy->tx->samples == NULL){
        perror("[PHY] malloc");
        goto error;
    }
    //Initialize control variables
    phy->tx->data_length = 0;
    phy->tx->buf_filled  = false;
    phy->tx->stop        = false;
    //Initialize pthread condition variable for buf_filled
    status = pthread_cond_init(&(phy->tx->buf_filled_cond), NULL);
    if (status != 0){
        ERROR("%s: Error initializing pthread_cond\n", __FUNCTION__);
        goto error;
    }
    //Initialize pthread mutex variable for buf_filled
    status = pthread_mutex_init(&(phy->tx->buf_status_lock), NULL);
    if (status != 0){
        ERROR("%s: Error initializing pthread_mutex\n", __FUNCTION__);
        goto error;
    }

    #ifdef LOG_TX_SAMPLES
        //Open TX samples file
        status = bladerf_get_serial_struct(dev, &sn);
        if (status != 0){
            ERROR("Failed to get serial number: %s\n", bladerf_strerror(status));
            goto error;
        }
        snprintf(filename, sizeof(filename), "tx_samples_%s.bin", sn.serial);

        phy->tx->samples_file = fopen(filename, "wb");
        if (phy->tx->samples_file == NULL){
            ERROR("Failed to open TX samples file for writing: %s\n", strerror(errno));
            goto error;
        }
        NOTE("Writing TX samples to %s\n", filename);
    #endif

    //------------------Initialize RX struct------------------
    phy->rx = calloc(1, sizeof(struct rx));
    if (phy->rx == NULL){
        perror("[PHY] malloc");
        goto error;
    }
    //Allocate memory for rx data buffer
    phy->rx->data_buf = malloc(max_frame_size);
    if (phy->rx->data_buf == NULL){
        perror("[PHY] malloc");
        goto error;
    }
    //Allocate memory for rx raw samples buffer
    phy->rx->in_samples = malloc(NUM_SAMPLES_RX * 2 * sizeof(phy->rx->in_samples[0]));
    if (phy->rx->in_samples == NULL){
        perror("[PHY] malloc");
        goto error;
    }

    //Allocate memory for filtered RX samples
    phy->rx->filt_samples = malloc(NUM_SAMPLES_RX/RX_DEC_FACT *
                                   sizeof(struct complex_sample));
    if (phy->rx->filt_samples == NULL){
        perror("[PHY] malloc");
        goto error;
    }

    //Allocate memory for power normalized samples ping pong buffer
    phy->rx->pnorm_samples[0] = malloc(NUM_SAMPLES_RX/RX_DEC_FACT *
                                       sizeof(struct complex_sample));
    if (phy->rx->pnorm_samples[0] == NULL){
        perror("[PHY] malloc");
        goto error;
    }
    phy->rx->pnorm_samples[1] = malloc(NUM_SAMPLES_RX/RX_DEC_FACT *
                                       sizeof(struct complex_sample));
    if (phy->rx->pnorm_samples[1] == NULL){
        perror("[PHY] malloc");
        goto error;
    }

    //Allocate memory for power estimate vector
    phy->rx->est_power = malloc(NUM_SAMPLES_RX/RX_DEC_FACT * sizeof(phy->rx->est_power[0]));
    if (phy->rx->est_power == NULL){
        perror("[PHY] malloc");
        goto error;
    }

    // Create RX Channel Filter
    phy->rx->ch_filt = fir_init(rx_ch_filter, rx_ch_filter_len, RX_DEC_FACT);
    if (phy->rx->ch_filt == NULL) {
        ERROR("%s: Failed to create channel filter.\n", __FUNCTION__);
        goto error;
    }

    // Create power normalizer
    phy->rx->pnorm = pnorm_init(PNORM_ALPHA, PNORM_MIN_GAIN, PNORM_MAX_GAIN);
    if (phy->rx->pnorm == NULL){
        ERROR("%s: Couldn't initialize power normalizer\n", __FUNCTION__);
        goto error;
    }

    //Create RX correlator
    phy->rx->corr = corr_init(preamble, 8*PREAMBLE_LENGTH, SAMP_PER_SYMB/RX_DEC_FACT);
    if (phy->rx->corr == NULL){
        ERROR("%s: Couldn't initialize correlator\n", __FUNCTION__);
        goto error;
    }

    //Initialize control variables
    phy->rx->buf_filled = false;
    phy->rx->stop       = false;
    //Initialize pthread condition variable for buf_filled
    status = pthread_cond_init(&(phy->rx->buf_filled_cond), NULL);
    if (status != 0){
        ERROR("%s: Error initializing pthread_cond\n", __FUNCTION__);
        goto error;
    }
    //Initialize pthread mutex variable for buf_filled
    status = pthread_mutex_init(&(phy->rx->buf_status_lock), NULL);
    if (status != 0){
        ERROR("%s: Error initializing pthread_mutex\n", __FUNCTION__);
        goto error;
    }

    #ifdef LOG_RX_SAMPLES
        //Open RX samples file
        status = bladerf_get_serial_struct(dev, &sn);
        if (status != 0){
            ERROR("Failed to get serial number: %s\n", bladerf_strerror(status));
            goto error;
        }
        snprintf(filename, sizeof(filename), "rx_samples_%s.bin", sn.serial);

        phy->rx->samples_file = fopen(filename, "wb");
        if (phy->rx->samples_file == NULL){
            ERROR("Failed to open RX samples file for writing: %s\n", strerror(errno));
            goto error;
        }
        NOTE("Writing RX samples to %s\n", filename);
    #endif

    //-----------------Load scrambling sequence--------------------
    prng_seed                = PRNG_SEED;
    phy->scrambling_sequence = prng_fill(&prng_seed, max_frame_size);
    if (phy->scrambling_sequence == NULL){
        ERROR("%s: Couldn't load scrambling sequence\n", __FUNCTION__);
        goto error;
    }

    phy->max_frame_size = max_frame_size;

    #ifdef BYPASS_RX_CHANNEL_FILTER
        NOTE("Info: Bypassing rx channel filter\n");
    #endif
    #ifdef BYPASS_RX_PNORM
        NOTE("Info: Bypassing rx power normalization\n");
    #endif
    #ifdef BYPASS_PHY_SCRAMBLING
        NOTE("Info: Bypassing scrambling\n");
    #endif

    DEBUG_MSG("Initialization done\n");
    return phy;

    error:
        phy_close(phy);
        return NULL;
}

void phy_close(struct phy_handle *phy)
{
    int status;

    DEBUG_MSG("Closing\n");
    if (phy != NULL){
        //close fsk handle
        fsk_close(phy->fsk);
        //Stop bladeRF (handle closed elsewhere)
        radio_stop(phy->dev);
        //free scrambling sequence buffer
        free(phy->scrambling_sequence);
        //free TX struct and its buffers
        if (phy->tx != NULL){
            #ifdef LOG_TX_SAMPLES
                if (phy->tx->samples_file != NULL) {
                    fclose(phy->tx->samples_file);
                }
            #endif
            free(phy->tx->data_buf);
            free(phy->tx->samples);
            status = pthread_mutex_destroy(&(phy->tx->buf_status_lock));
            if (status != 0){
                ERROR("%s: Error destroying pthread_mutex\n", __FUNCTION__);
            }
            status = pthread_cond_destroy(&(phy->tx->buf_filled_cond));
            if (status != 0){
                ERROR("%s: Error destroying pthread_cond\n", __FUNCTION__);
            }
        }
        free(phy->tx);
        //free RX struct and its buffers
        if (phy->rx != NULL){
            #ifdef LOG_RX_SAMPLES
                if (phy->rx->samples_file != NULL) {
                    fclose(phy->rx->samples_file);
                }
            #endif
            free(phy->rx->data_buf);
            fir_deinit(phy->rx->ch_filt);
            corr_deinit(phy->rx->corr);
            pnorm_deinit(phy->rx->pnorm);
            free(phy->rx->in_samples);
            free(phy->rx->filt_samples);
            free(phy->rx->pnorm_samples[0]);
            free(phy->rx->pnorm_samples[1]);
            free(phy->rx->est_power);
            status = pthread_mutex_destroy(&(phy->rx->buf_status_lock));
            if (status != 0){
                ERROR("%s: Error destroying pthread_mutex\n", __FUNCTION__);
            }
            status = pthread_cond_destroy(&(phy->rx->buf_filled_cond));
            if (status != 0){
                ERROR("%s: Error destroying pthread_cond\n", __FUNCTION__);
            }
        }
        free(phy->rx);
    }
    //free phy struct
    free(phy);
    phy = NULL;
}

/****************************************
 *                                      *
 *          TRANSMITTER FUNCTIONS       *
 *                                      *
 ****************************************/

int phy_start_transmitter(struct phy_handle *phy)
{
    int status;

    //turn off stop signal
    phy->tx->stop = false;
    //Kick off frame transmitter thread
    status = pthread_create(&(phy->tx->thread), NULL, phy_transmit_frames, phy);
    if (status != 0){
        ERROR("%s: Error creating tx thread: %s\n", __FUNCTION__, strerror(status));
        return -1;
    }
    return 0;
}

int phy_stop_transmitter(struct phy_handle *phy)
{
    int status;

    DEBUG_MSG("TX: Stopping transmitter...\n");
    //signal stop
    phy->tx->stop = true;
    //Signal the buffer filled condition so that the thread will stop
    //waiting for a filled buffer
    status = pthread_mutex_lock(&(phy->tx->buf_status_lock));
    if (status != 0){
        ERROR("%s: Error locking pthread_mutex\n", __FUNCTION__);
    }
    status = pthread_cond_signal(&(phy->tx->buf_filled_cond));
    if (status != 0){
        ERROR("%s: Error signaling pthread_cond\n", __FUNCTION__);
    }
    status = pthread_mutex_unlock(&(phy->tx->buf_status_lock));
    if (status != 0){
        ERROR("%s: Error unlocking pthread_mutex\n", __FUNCTION__);
    }
    //Wait for tx thread to finish
    status = pthread_join(phy->tx->thread, NULL);
    if (status != 0){
        ERROR("%s: Error joining tx thread: %s\n", __FUNCTION__, strerror(status));
        return -1;
    }
    DEBUG_MSG("TX: Transmitter stopped\n");
    return 0;
}

int phy_fill_tx_buf(struct phy_handle *phy, uint8_t *data_buf, unsigned int length)
{
    int status;

    //Check for null
    if (data_buf == NULL){
        ERROR("%s: The supplied data buf is null\n", __FUNCTION__);
        return -1;
    }
    //Check for length outside of bounds
    if (length > phy->max_frame_size){
        ERROR("%s: Data length of %u is greater than the maximum allowed length (%u)\n",
              __FUNCTION__, length, phy->max_frame_size);
        return -1;
    }
    //Wait for the buffer to be empty (and therefore ready for the next frame)
    //Not doing a pthread_cond_wait() because this shouldn't take long
    while(phy->tx->buf_filled){
        usleep(50);
    }

    //Copy the tx data into the phy's tx data buf, just after where the preamble goes
    memcpy(&(phy->tx->data_buf[TRAINING_SEQ_LENGTH+PREAMBLE_LENGTH]), data_buf, length);
    //Set the data length
    phy->tx->data_length = length;

    //Mark buffer filled and signal the buffer filled condition
    status = pthread_mutex_lock(&(phy->tx->buf_status_lock));
    if (status != 0){
        ERROR("%s: Error locking pthread_mutex\n", __FUNCTION__);
        return -1;
    }
    phy->tx->buf_filled = true;
    status = pthread_cond_signal(&(phy->tx->buf_filled_cond));
    if (status != 0){
        ERROR("%s: Error signaling pthread_cond\n", __FUNCTION__);
        return -1;
    }
    status = pthread_mutex_unlock(&(phy->tx->buf_status_lock));
    if (status != 0){
        ERROR("%s: Error unlocking pthread_mutex\n", __FUNCTION__);
        return -1;
    }
    return 0;
}

/**
 * Thread function which transmits data frames
 *
 * @param[in]   arg     pointer to phy handle struct
 */
void *phy_transmit_frames(void *arg)
{
    int                     status;
    //Cast arg
    struct phy_handle      *phy = (struct phy_handle *) arg;
    uint8_t                 preamble[PREAMBLE_LENGTH]         = PREAMBLE;
    uint8_t                 training_seq[TRAINING_SEQ_LENGTH] = TRAINING_SEQ;
    int                     ramp_down_index;
    int                     num_mod_samples, num_samples;
    bool                    failed = false;
    struct bladerf_metadata metadata;
    int16_t                *out_samples_raw;
    #ifdef LOG_TX_SAMPLES
        size_t nwritten;
    #endif

    //Set field(s) in bladerf metadata struct
    memset(&metadata, 0, sizeof(metadata));
    metadata.flags =    BLADERF_META_FLAG_TX_BURST_START |
                        BLADERF_META_FLAG_TX_NOW |
                        BLADERF_META_FLAG_TX_BURST_END;

    while (!phy->tx->stop){
        //--------Wait for buffer to be filled---------
        //Lock mutex
        status = pthread_mutex_lock(&(phy->tx->buf_status_lock));
        if (status != 0){
            ERROR("%s: Mutex lock failed: %s\n", __FUNCTION__, strerror(status));
            goto out;
        }
        //Wait for condition signal - meaning buffer is full
        while (!phy->tx->buf_filled && !phy->tx->stop){
            DEBUG_MSG("TX: Waiting for buffer to be filled\n");
            status = pthread_cond_wait(&(phy->tx->buf_filled_cond), &(phy->tx->buf_status_lock));
            if (status != 0){
                ERROR("%s: Condition wait failed: %s\n", __FUNCTION__, strerror(status));
                failed = true;
                break;
            }
        }
        //Stop thread if stop variable is true, or something with pthreads went wrong
        if (phy->tx->stop || failed){
            phy->tx->buf_filled = false;
            //Unlock mutex
            status = pthread_mutex_unlock(&(phy->tx->buf_status_lock));
            if (status != 0){
                ERROR("%s: Mutex unlock failed: %s\n", __FUNCTION__, strerror(status));
                failed = true;
            }
            goto out;
        }
        //------------Transmit the frame-------------
        DEBUG_MSG("TX: Buffer filled. Transmitting.\n");
        //Calculate the number of samples to transmit.
        num_samples = 2*RAMP_LENGTH + (TRAINING_SEQ_LENGTH + PREAMBLE_LENGTH +
                      phy->tx->data_length) * 8 * SAMP_PER_SYMB;
        #ifdef SYNC_NO_METADATA
            //Not using metadata mode; round up to multiple of SYNC_BUFFER_SIZE
            int rem = num_samples % SYNC_BUFFER_SIZE;
            if (rem != 0){
                num_samples = num_samples + (SYNC_BUFFER_SIZE - rem);
            }
        #endif

        //Add training sequence to tx data buffer
        memcpy(phy->tx->data_buf, &training_seq, TRAINING_SEQ_LENGTH);
        //Add preamble to tx data buffer
        memcpy(&(phy->tx->data_buf[TRAINING_SEQ_LENGTH]), &preamble, PREAMBLE_LENGTH);
        #ifndef BYPASS_PHY_SCRAMBLING
            //Scramble the frame data (not including the training sequence or preamble)
            scramble_frame(&(phy->tx->data_buf[TRAINING_SEQ_LENGTH + PREAMBLE_LENGTH]),
                           phy->tx->data_length, phy->scrambling_sequence);
        #endif
        //zero the tx samples buffer
        memset(phy->tx->samples, 0, sizeof(int16_t) * 2 * num_samples);
        //modulate samples - leave space for ramp up/ramp down in the samples buffer
        num_mod_samples = fsk_mod(phy->fsk, phy->tx->data_buf,
                            TRAINING_SEQ_LENGTH + PREAMBLE_LENGTH + phy->tx->data_length,
                            SAMP_PER_SYMB, &(phy->tx->samples[RAMP_LENGTH]));
        //Mark the buffer empty
        phy->tx->buf_filled = false;

        //Unlock mutex now that we have marked the buffer empty
        status = pthread_mutex_unlock(&(phy->tx->buf_status_lock));
        if (status != 0){
            ERROR("%s: Mutex unlock failed: %s\n", __FUNCTION__, strerror(status));
            failed = true;
        }

        #ifdef TX_DC_TONE
            for (int i = 0; i < num_samples; i++){
                phy->tx->samples[i].i = 2047;
                phy->tx->samples[i].q = 0;
            }
        #endif

        //Add the ramp up/ ramp down of samples
        ramp_down_index = RAMP_LENGTH+num_mod_samples;
        create_ramps(RAMP_LENGTH, phy->tx->samples[ramp_down_index-1], phy->tx->samples,
                     &(phy->tx->samples[ramp_down_index]));
        //Convert struct complex_sample to int16_t
        //This simple cast works fine because the struct is packed: int16_t I, int16_t Q
        out_samples_raw = (int16_t *) phy->tx->samples;

        #ifdef LOG_TX_SAMPLES
            //Write samples out to binary file
            nwritten = fwrite(out_samples_raw, sizeof(int16_t), num_samples*2,
                              phy->tx->samples_file);
            if (nwritten != (size_t)(num_samples*2)){
                ERROR("Failed to write all samples to TX debug file: %s\n",
                      strerror(errno));
                goto out;
            }
        #endif

        //transmit all samples. TX_NOW
        status = bladerf_sync_tx(phy->dev, out_samples_raw, num_samples, &metadata, 5000);
        if (status != 0){
            ERROR("%s: Couldn't transmit samples with bladeRF: %s\n",
                  __FUNCTION__, bladerf_strerror(status));
            goto out;
        }
    }

out:
    return NULL;
}

/**
 * Creates a ramp up and ramp down of samples in the following format: Ramp up will use
 * 'ramp_length' samples to ramp up the I samples from 0 to 2048, and set the Q samples to 0.
 * Ramp down will use 'ramp_length' samples to ramp down the I samples from
 * ramp_down_init.i to 0, and ramp down the Q samples from ramp_down_init.q to 0.
 *
 * Ex: If ramp_length is 4, ramp_down_i_init is -2048, and ramp_down_q_init is 0,
 * ramp_up buffer will be:
 * [.25+0j  .5+0j  .75+0j  1+0j] scaled by 2048
 * and ramp_down will be
 * [-.75+0j  -.5+0j  -.25+0j  0+0j] scaled by 2048
 */
static void create_ramps(unsigned int ramp_length, struct complex_sample ramp_down_init,
                         struct complex_sample *ramp_up, struct complex_sample *ramp_down)
{
    unsigned int samp;
    double       ramp_up_step = 2048.0/ramp_length;
    double       ramp_down_step_i;
    double       ramp_down_step_q;

    //Determine ramp down steps
    if (ramp_down_init.i == 0){
        ramp_down_step_i = 0;
    }else{
        ramp_down_step_i = (double)ramp_down_init.i/(double)ramp_length;
    }
    if (ramp_down_init.q == 0){
        ramp_down_step_q = 0;
    }else{
        ramp_down_step_q = (double)ramp_down_init.q/(double)ramp_length;
    }
    //Create the ramps
    for (samp = 0; samp < ramp_length-1; samp++){
        //ramp up
        ramp_up[samp].i = (int16_t) round(ramp_up_step*(samp+1));    //I
        ramp_up[samp].q = 0;                            //Q

        //ramp down
		ramp_down[samp].i = (int16_t) round(ramp_down_init.i - ramp_down_step_i*(samp + 1));
		ramp_down[samp].q = (int16_t) round(ramp_down_init.q - ramp_down_step_q*(samp + 1));
    }

    //Set last ramp up/ ramp down sample (will always be the same)
    ramp_up[ramp_length-1].i = 2047;    //I
    ramp_up[ramp_length-1].q = 0;        //Q

    ramp_down[ramp_length-1].i = 0;        //I
    ramp_down[ramp_length-1].q = 0;        //Q
}

#ifndef BYPASS_PHY_SCRAMBLING
    /**
    * Scrambles the given data with the given scrambling sequence. The size of the
    * scrambling sequence array must be at least as large as the frame.
    */
    static void scramble_frame(uint8_t *frame, int frame_length,
                               uint8_t *scrambling_sequence)
    {
        int i;
        for (i = 0; i < frame_length; i++){
            //XOR byte with byte from scrambling sequence
            frame[i] ^= scrambling_sequence[i];
        }
    }
#endif

/****************************************
 *                                      *
 *          RECEIVER FUNCTIONS          *
 *                                      *
 ****************************************/

int phy_start_receiver(struct phy_handle *phy)
{
    int status;

    //turn off stop signal
    phy->rx->stop    = false;
    //reset debug status
    phy->rx->overrun = false;
    //Kick off frame receiver thread
    status = pthread_create(&(phy->rx->thread), NULL, phy_receive_frames, phy);
    if (status != 0){
        ERROR("%s: Error creating rx thread: %s\n", __FUNCTION__, strerror(status));
        return -1;
    }
    return 0;
}

int phy_stop_receiver(struct phy_handle *phy)
{
    int status;

    DEBUG_MSG("RX: Stopping receiver...\n");
    //signal stop
    phy->rx->stop = true;
    //Wait for rx thread to finish
    status = pthread_join(phy->rx->thread, NULL);
    if (status != 0){
        ERROR("%s: Error joining rx thread: %s\n", __FUNCTION__, strerror(status));
        return -1;
    }
    DEBUG_MSG("RX: Receiver stopped\n");
    if (phy->rx->overrun){
        return 1;   //warning: RX overruns were detected
    }else{
        return 0;
    }
}

uint8_t *phy_request_rx_buf(struct phy_handle *phy, unsigned int timeout_ms)
{
    int             status;
    struct timespec timeout_abs;
    bool            failed = false;

    //Create absolute time format timeout
    status = create_timeout_abs(timeout_ms, &timeout_abs);
    if (status != 0){
        ERROR("%s: Error creating timeout\n", __FUNCTION__);
        return NULL;
    }

    //Lock mutex
    status = pthread_mutex_lock(&(phy->rx->buf_status_lock));
    if (status != 0){
        ERROR("%s: Error locking mutex: %s\n", __FUNCTION__, strerror(status));
        return NULL;
    }
    //Wait for condition signal - meaning buffer is full
    while (!phy->rx->buf_filled){
        status = pthread_cond_timedwait(&(phy->rx->buf_filled_cond),
                                        &(phy->rx->buf_status_lock), &timeout_abs);
        if (status != 0){
            if (status == ETIMEDOUT){
                //DEBUG_MSG("phy_request_rx_buf(): Condition wait timed out\n");
            }else{
                ERROR("%s: Condition wait failed: %s\n", __FUNCTION__, strerror(status));
            }
            failed = true;
            break;
        }
    }
    //Unlock mutex
    status = pthread_mutex_unlock(&(phy->rx->buf_status_lock));
    if (status != 0){
        ERROR("%s: Mutex unlock failed: %s\n", __FUNCTION__, strerror(status));
        failed = true;
    }

    if (failed){
        return NULL;
    }
    return phy->rx->data_buf;
}

void phy_release_rx_buf(struct phy_handle *phy)
{
    phy->rx->buf_filled = false;
}

/**
 * Thread function which listens for and receives frames. The steps are:
 * 1) Receive samples with libbladeRF
 * 2) Low pass filter the samples
 * 3) Power normalize the samples
 * 4) Correlate the samples with the preamble waveform
 * 5) If a match is found, demodulate the samples into data bytes
 * 6) Unscramble the data
 * 7) Estimate SNR
 * 8) Copy the frame to a buffer which can be acquired with phy_request_rx_buf(), or
 *    drop the frame if the buffer is still in use
 *
 * @param    arg        pointer to phy_handle struct
 */
void *phy_receive_frames(void *arg)
{
    struct phy_handle      *phy = (struct phy_handle *) arg;
    int                     status;
    bool                    samp_buf_sel;   //Buffer select for phy->rx->pnorm_samples
    unsigned int            num_bytes_rx;
    unsigned int            data_idx;   //Current index of rx_buffer to receive new bytes
                                        //on. doubles as the current received frame length
    uint64_t                samp_buf_idx;   //Current index of pnorm samples buffer to
                                            //correlate/demod samples from
    uint64_t                samp_idx;       //Total running samples index/timestamp, after
                                            //decimation. Marks the 1st sample in current
                                            //buffer
    uint64_t                corr_samp_idx;          //correlator output index/timestamp
    bool                    preamble_detected;
    bool                    new_frame;
    bool                    checked_frame_type;     //have we checked frame type yet?
    int                     frame_length;           //link layer frame length
    uint8_t                *rx_buffer = NULL;       //local rx data buffer
    uint8_t                 frame_type;
    struct bladerf_metadata metadata;               //bladerf metadata for sync_rx()
    unsigned int            num_bytes_to_demod;
    //actual number of samples RX'd from bladerf_sync_rx()
    unsigned int            num_samples_rx_act;
    //actual number of samples output from FIR filter post decimation into each buffer
    unsigned int            num_samples_rx_dec[2];
    uint64_t                noise_est_pwr_idx = 0;
    float                   signoise_est_pwr, noise_est_pwr, sig_est_pwr;
    float                   snr_est, snr_est_db, snr_est_avg;
    unsigned int            num_snr_ests = 0;
    bool                    waiting_on_snr_est;
    int                     pnorm_settle_time;
    unsigned long           frame_cnt;
    int                     num_samples_proc;  //num samps processed by fsk_demod())
    bool                    already_rxd_next_buf;

    #ifndef SYNC_NO_METADATA
        uint64_t            timestamp = UINT64_MAX;
    #endif
    #ifdef LOG_RX_SAMPLES
        size_t              nwritten;
    #endif
    #ifdef BYPASS_RX_CHANNEL_FILTER
        unsigned int        dec_phase = 0;  //decimation phase tracker
    #endif

    enum states {RECEIVE, PREAMBLE_CORRELATE, DEMOD, CHECK_FRAME_TYPE, DECODE,
                 ESTIMATE_SNR, COPY};
    enum states state;          //current state variable
    enum states next_state;     //stored next state, only needed for ESTIMATE_SNR

    //corr_process() takes a size_t count. Ensure a cast from uint64_t to size_t is valid
    assert(NUM_SAMPLES_RX < SIZE_MAX);

    #ifdef RX_SAMPLES_FROM_FILE
        //Open RX input samples file
        FILE                 *in_samples_file = NULL;
        char                  filename[18+BLADERF_SERIAL_LENGTH];
        struct bladerf_serial sn;
        size_t                nread;
        size_t                nsamp;

        status = bladerf_get_serial_struct(phy->dev, &sn);
        if (status != 0){
            ERROR("Failed to get serial number: %s\n", bladerf_strerror(status));
            goto out;
        }
        snprintf(filename, sizeof(filename), "in_rx_samples_%s.bin", sn.serial);

        in_samples_file = fopen(filename, "rb");
        if (in_samples_file == NULL){
            ERROR("Failed to open RX input samples file '%s' for reading: %s\n",
                   filename, strerror(errno));
            goto out;
        }
        NOTE("Reading RX input samples from %s instead of using bladerf_sync_rx()\n",
             filename);
    #endif

    //Allocate memory for buffer
    rx_buffer = malloc(phy->max_frame_size);
    if (rx_buffer == NULL){
        perror("[PHY] malloc");
        goto out;
    }

    //Set bladeRF metadata
    memset(&metadata, 0, sizeof(metadata));
    metadata.flags = BLADERF_META_FLAG_RX_NOW;

    //number of samples for power estimate to settle 99.9% of the way after the input
    //power changes
    pnorm_settle_time = (int) ceil( log2(1 - .999f) / log2(PNORM_ALPHA) );
    DEBUG_MSG("RX: Pnorm settle time = %d samples\n", pnorm_settle_time);

    preamble_detected    = false;
    data_idx             = 0;
    samp_idx             = 0;
    samp_buf_sel         = 0;
    frame_cnt            = 0;
    snr_est_avg          = 0;
    waiting_on_snr_est   = false;
    already_rxd_next_buf = false;
    state                = RECEIVE;
    //Loop until stop signal detected
    while(!phy->rx->stop){
        switch(state){
            case RECEIVE:
                //--Receive samples, filter, and power normalize
                // DEBUG_MSG("RX: State = RECEIVE, samp_idx = %lu\n", samp_idx);
                samp_buf_idx = 0;
                if (!already_rxd_next_buf){
                    //Did not already receive/filter the next buffer of samples
                    //Receive more samples
                    #ifdef RX_SAMPLES_FROM_FILE
                        //Read input samples from file
                        nread = fread(phy->rx->in_samples, sizeof(phy->rx->in_samples[0]),
                                      NUM_SAMPLES_RX*2, in_samples_file);
                        if (nread == 0){
                            //EOF
                            NOTE("Reached end of RX input samples file %s. Stopping receiver\n",
                                 filename);
                            goto out;
                        }
                        if (nread < NUM_SAMPLES_RX*2){
                            NOTE("Only read %lu samples from file instead of %u\n", nread/2,
                                 NUM_SAMPLES_RX);
                        }
                        num_samples_rx_act = nread/2;
                    #else
                        status = bladerf_sync_rx(phy->dev, phy->rx->in_samples,
                                                 NUM_SAMPLES_RX, &metadata, 5000);
                        if (status != 0){
                            ERROR("RX: %s: Couldn't receive samples from bladeRF: %s\n",
                                  __FUNCTION__, bladerf_strerror(status));
                            goto out;
                        }
                        #ifdef SYNC_NO_METADATA
                            num_samples_rx_act = NUM_SAMPLES_RX;
                        #else
                            //Check metadata
                            if (metadata.status & BLADERF_META_STATUS_OVERRUN){
                                NOTE("RX: %s: Got an overrun. Expected count = %u; actual count = %u.\n",
                                     __FUNCTION__, NUM_SAMPLES_RX, metadata.actual_count);
                                phy->rx->overrun = true;
                            }
                            num_samples_rx_act = metadata.actual_count;
                            if (timestamp != UINT64_MAX && metadata.timestamp != timestamp+NUM_SAMPLES_RX){
                                NOTE("RX: %s: Unexpected timestamp. Expected %lu, got %lu.\n",
                                     __FUNCTION__, timestamp+NUM_SAMPLES_RX, metadata.timestamp);
                            }
                            timestamp = metadata.timestamp;
                        #endif
                    #endif

                    #ifdef BYPASS_RX_CHANNEL_FILTER
                        //Downsample by RX_DEC_FACT without filtering (not ideal)
                        //Also convert from raw int16_t to struct complex_sample
                        int j = 0;  //output index
                        for (unsigned int i = 0; i < num_samples_rx_act*2; i += 2){
                            if (dec_phase == 0){
                                phy->rx->filt_samples[j].i = phy->rx->in_samples[i];
                                phy->rx->filt_samples[j].q = phy->rx->in_samples[i+1];
                                j++;
                            }
                            if (dec_phase == RX_DEC_FACT){
                                dec_phase = 0;
                            }else{
                                dec_phase++;
                            }
                        }
                        num_samples_rx_dec[samp_buf_sel] = j;
                    #else
                        //Apply channel filter within built-in decimation by RX_DEC_FACT
                        num_samples_rx_dec[samp_buf_sel] =
                            fir_process(phy->rx->ch_filt, phy->rx->in_samples,
                                        phy->rx->filt_samples, num_samples_rx_act);
                    #endif

                    #ifdef BYPASS_RX_PNORM
                        memcpy(phy->rx->pnorm_samples[samp_buf_sel], phy->rx->filt_samples,
                               num_samples_rx_dec[samp_buf_sel] * sizeof(struct complex_sample));
                    #else
                        //Power normalize
                        pnorm(phy->rx->pnorm, num_samples_rx_dec[samp_buf_sel],
                              phy->rx->filt_samples, phy->rx->pnorm_samples[samp_buf_sel],
                              phy->rx->est_power, NULL);
                    #endif

                    #ifdef LOG_RX_SAMPLES
                        #ifdef LOG_RX_SAMPLES_USE_PNORM
                            //Write filtered+power normalized samples out to binary file
                            nwritten = fwrite((int16_t *)phy->rx->pnorm_samples[samp_buf_sel],
                                              sizeof(int16_t), num_samples_rx_dec[samp_buf_sel]*2,
                                              phy->rx->samples_file);
                            if (nwritten != (size_t)(num_samples_rx_dec[samp_buf_sel]*2)){
                                ERROR("Failed to write all samples to RX debug file: %s\n",
                                      strerror(errno));
                                goto out;
                            }
                        #else
                            //Write raw samples out to binary file
                            nwritten = fwrite(phy->rx->in_samples, sizeof(int16_t),
                                              num_samples_rx_act*2, phy->rx->samples_file);
                            if (nwritten != (size_t)(num_samples_rx_act*2)){
                                ERROR("Failed to write all samples to RX debug file: %s\n",
                                      strerror(errno));
                                goto out;
                            }
                        #endif
                    #endif
                }
                already_rxd_next_buf = false;

                if (preamble_detected){
                    next_state = DEMOD;
                }else{
                    next_state = PREAMBLE_CORRELATE;
                }

                if (waiting_on_snr_est){
                    //We were waiting for more samples to collect stable noise power
                    //estimate after previous frame ended. We might have them now.
                    //Hop sideways to estimate SNR before going to next state
                    state = ESTIMATE_SNR;
                }else{
                    state = next_state;
                }
                break;
            case PREAMBLE_CORRELATE:
                //--Cross correlate received samples with preamble to find start
                //--of the data frame
                // DEBUG_MSG("RX: State = PREAMBLE_CORRELATE\n");
                corr_samp_idx =
                    corr_process(phy->rx->corr,
                                 &(phy->rx->pnorm_samples[samp_buf_sel][samp_buf_idx]),
                                 (size_t) (num_samples_rx_dec[samp_buf_sel]-samp_buf_idx),
                                 samp_idx);
                if (corr_samp_idx != CORRELATOR_NO_RESULT){
                    //Detected a frame
                    if (corr_samp_idx < samp_idx){
                        //Reported peak is in the previous buffer - corner case where we
                        //ran out of samples before the correlator's countdown finished.
                        //Switch to previous buffer, backtrack samp_idx, and indicate that
                        //we've already RX'd the samples for the subsequent buffer
                        DEBUG_MSG("RX: Correlator reported match is in previous buffer. "
                                  "Switching ping/pong buffer back to previous buffer.\n");
                        samp_buf_sel         = !samp_buf_sel;
                        samp_idx            -= num_samples_rx_dec[samp_buf_sel];
                        already_rxd_next_buf = true;
                    }
                    samp_buf_idx = corr_samp_idx - samp_idx;

                    DEBUG_MSG("RX: Preamble matched @ samp idx %lu (buffer samp idx = %lu)\n",
                              corr_samp_idx, samp_buf_idx);
                    preamble_detected  = true;
                    new_frame          = true;
                    checked_frame_type = false;
                    frame_cnt++;

                    //SNR estimator bookkeeping
                    if (waiting_on_snr_est){
                        DEBUG_MSG("RX: Detected another frame before we had time to get a "
                             "stable noise power estimate. Skipping previous SNR estimate.\n");
                        waiting_on_snr_est = false;
                    }
                    //store the current power estimate (at start of data) as S+N power
                    //at this point the power estimate has settled from training+preamble
                    signoise_est_pwr   = phy->rx->est_power[samp_buf_idx];

                    //First we only demod the first byte to determine frame type
                    num_bytes_to_demod = 1;
                    data_idx           = 0;
                    state              = DEMOD;
                }else{
                    //No preamble match. Receive more samples
                    state = RECEIVE;
                }
                break;
            case DEMOD:
                //--Demod samples
                DEBUG_MSG("RX: State = DEMOD\n");
                num_bytes_rx = fsk_demod(phy->fsk,
                                         &(phy->rx->pnorm_samples[samp_buf_sel][samp_buf_idx]),
                                         num_samples_rx_dec[samp_buf_sel]-(int)samp_buf_idx,
                                         new_frame, num_bytes_to_demod, SAMP_PER_SYMB/RX_DEC_FACT,
                                         &rx_buffer[data_idx], &num_samples_proc);
                if (num_bytes_rx < num_bytes_to_demod){
                    //Receive more samples
                    num_bytes_to_demod -= num_bytes_rx;
                    state               = RECEIVE;
                }else{
                    if (!checked_frame_type){
                        //On the 1st byte; need to check frame type to determine length
                        state = CHECK_FRAME_TYPE;
                    }else{
                        //We demodulated all samples in the frame
                        state = DECODE;
                    }
                }
                data_idx     += num_bytes_rx;
                samp_buf_idx += num_samples_proc;
                new_frame     = false;
                break;
            case CHECK_FRAME_TYPE:
                //--Check the frame type byte
                DEBUG_MSG("RX: State = CHECK_FRAME_TYPE\n");
                #ifndef BYPASS_PHY_SCRAMBLING
                    //unscramble
                    frame_type = rx_buffer[0] ^ phy->scrambling_sequence[0];
                #else
                    frame_type = rx_buffer[0];
                #endif
                //Set frame length according to what type of frame it is
                if (frame_type == ACK_FRAME_CODE){
                    DEBUG_MSG("RX: Getting an ACK frame...\n");
                    frame_length = ACK_FRAME_LENGTH;
                }else if(frame_type == DATA_FRAME_CODE){
                    DEBUG_MSG("RX: Getting a data frame...\n");
                    frame_length = phy->max_frame_size;
                }else{
                    NOTE("RX: Received unknown frame type 0x%02X. Dropping this frame.\n",
                         frame_type);
                    data_idx          = 0;
                    preamble_detected = false;
                    state             = PREAMBLE_CORRELATE;
                    break;
                }
                checked_frame_type = true;
                //Demod the rest of the bytes
                num_bytes_to_demod = frame_length-1;
                state              = DEMOD;
                break;
            case DECODE:
                //--Remove any phy encoding on the received frame
                DEBUG_MSG("RX: State = DECODE\n");
                #ifndef BYPASS_PHY_SCRAMBLING
                    //Unscramble the frame
                    unscramble_frame(rx_buffer, frame_length, phy->scrambling_sequence);
                #endif
                next_state = COPY;

                //Done processing frame. Prepare SNR estimate before going to next state
                state = ESTIMATE_SNR;
                break;
            case ESTIMATE_SNR:
                //--Estimate SNR using the power estimate just after the preamble as
                //--(signal+noise), and the power estimate just after the frame as noise.
                //--signal power estimate = (signal+noise) - noise
                DEBUG_MSG("RX: State = ESTIMATE_SNR\n");
                //signoise_est_pwr has been previously stored away at beginning of frame

                if (!waiting_on_snr_est){
                    //Prepare new SNR estimate after frame ended
                    //samp_buf_idx is now at the end of frame, at start of ramp down. Add
                    //time for ramp down and power estimate settling
                    //ideally would add extra settling time (+7000) to account for
                    //bladeRF2 DC offset IIR filter decay, but AGC can change very quickly
                    //post-frame samples buffer index to use for noise estimate:
                    noise_est_pwr_idx  = samp_buf_idx + RAMP_LENGTH +
                                         pnorm_settle_time;
                }

                if (noise_est_pwr_idx >= num_samples_rx_dec[samp_buf_sel]){
                    //need to receive more samples before we can obtain our noise estimate
                    noise_est_pwr_idx -= num_samples_rx_dec[samp_buf_sel];
                    waiting_on_snr_est = true;
                }else{
                    //Have everything needed; compute SNR estimate
                    noise_est_pwr = phy->rx->est_power[noise_est_pwr_idx];

                    sig_est_pwr   = signoise_est_pwr - noise_est_pwr;
                    snr_est       = sig_est_pwr/noise_est_pwr;
                    snr_est_avg  += snr_est; //add to sum in preparation for taking average
                    if (snr_est <= 0){
                        snr_est_db = -INFINITY;
                    }else{
                        snr_est_db = 10*log10(snr_est);
                    }
                    waiting_on_snr_est = false;
                    num_snr_ests++;
                    NOTE("RX: Post-filter SNR estimate = %.1f dB (frame %lu)\n",
                         snr_est_db, frame_cnt);
                }

                state = next_state;     //Go to previously stored next state
                break;
            case COPY:
                //--Copy frame into buffer which can be accessed by the link layer
                DEBUG_MSG("RX: State = COPY\n");
                //Is the link layer still working with the previous frame?
                if (phy->rx->buf_filled){
                    //Instead of disrupting the link layer, drop this frame
                    NOTE("RX: Frame dropped! Had frame ready but byte buffer was full.\n");
                }else{
                    //Copy frame into rx_data_buf
                    memcpy(phy->rx->data_buf, rx_buffer, frame_length);
                    //Signal that the buffer is filled
                    status = pthread_mutex_lock(&(phy->rx->buf_status_lock));
                    if (status != 0){
                        ERROR("%s: Error locking pthread_mutex\n", __FUNCTION__);
                        goto out;
                    }
                    phy->rx->buf_filled = true;
                    status = pthread_cond_signal(&(phy->rx->buf_filled_cond));
                    if (status != 0){
                        ERROR("%s: Error signaling pthread_cond\n", __FUNCTION__);
                        goto out;
                    }
                    status = pthread_mutex_unlock(&(phy->rx->buf_status_lock));
                    if (status != 0){
                        ERROR("%s: Error unlocking pthread_mutex\n", __FUNCTION__);
                        goto out;
                    }
                    DEBUG_MSG("RX: Frame ready\n");
                }
                preamble_detected = false;

                state = PREAMBLE_CORRELATE;
                break;
            default:
                ERROR("%s: Invalid state\n", __FUNCTION__);
                goto out;
        }
        //Do some things based on the next state, before the next state machine cycle
        if (state == RECEIVE){
            samp_idx    += num_samples_rx_dec[samp_buf_sel];
            samp_buf_sel = !samp_buf_sel;
        }
    }
    out:
        if (num_snr_ests > 0){
            //Print final SNR estimate, average from all frames
            snr_est_avg /= num_snr_ests;
            if (snr_est_avg < 0){
                snr_est_db = -INFINITY;
            }else{
                snr_est_db = 10*log10(snr_est_avg);
            }
            fprintf(stderr, "[PHY] RX: Average SNR = %.1f dB\n", snr_est_db);
        }
        #ifdef RX_SAMPLES_FROM_FILE
            if (in_samples_file != NULL){
                fclose(in_samples_file);
            }
        #endif
        if (rx_buffer != NULL){
            free(rx_buffer);
        }
        return NULL;
}

#ifndef BYPASS_PHY_SCRAMBLING
    /**
    * Unscrambles the given data with the given scrambling sequence. The size of the
    * scrambling sequence array must be at least as large as the frame.
    */
    static void unscramble_frame(uint8_t *frame, int frame_length,
                                 uint8_t *scrambling_sequence)
    {
        int i;
        for (i = 0; i < frame_length; i++){
            //XOR byte with byte from scrambling sequence
            frame[i] ^= scrambling_sequence[i];
        }
    }
#endif
