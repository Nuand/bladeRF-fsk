/**
 * @brief   BladeRF radio configuration - setting of frequencies, gains, etc.
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

#include "radio_config.h"
#include <string.h>
#include <inttypes.h>

#if defined(DEBUG_MODE) || defined(VERBOSE_MODE)
    #define DEBUG_MSG(...) fprintf(stderr, "[RADIO] " __VA_ARGS__)
#else
    #define DEBUG_MSG(...)
#endif

//private structs
struct module_config{
    bladerf_channel     module;
    bladerf_frequency   frequency;
    bladerf_bandwidth   bandwidth;
    bladerf_sample_rate samplerate;
    /* Gains */
    bool                use_unified;
    bool                use_agc;
    bladerf_gain        unified_gain;
    bladerf_lna_gain    rx_lna;
    int                 vga1;
    int                 vga2;
    /* Biastee control */
    bool                biastee;
};
//internal functinos
static int radio_configure_module(struct bladerf *dev, struct module_config *c);
static int radio_init_sync(struct bladerf *dev);

/**
 * Configure RX/TX module
 */
static int radio_configure_module(struct bladerf *dev, struct module_config *c)
{
    int status;
    const struct bladerf_range *freq_range;
    const struct bladerf_range *gain_range;

    //Check to see if frequency is within bounds of device
    status = bladerf_get_frequency_range(dev, c->module, &freq_range);
    if (status != 0){
        fprintf(stderr, "Failed to get frequency range: %s\n",
                bladerf_strerror(status));
        return status;
    }
    if ((int64_t)c->frequency < freq_range->min || (int64_t)c->frequency > freq_range->max){
        fprintf(stderr, "Requested frequency %" PRIu64 " Hz is outside supported range [%" PRId64 ", %" PRId64 "] Hz\n",
                c->frequency, freq_range->min, freq_range->max);
        return BLADERF_ERR_RANGE;
    }

    status = bladerf_set_frequency(dev, c->module, c->frequency);
    if (status != 0){
        fprintf(stderr, "Failed to set frequency = %" PRIu64 ": %s\n",
                c->frequency, bladerf_strerror(status));
        return status;
    }
    status = bladerf_set_sample_rate(dev, c->module, c->samplerate, NULL);
    if (status != 0){
        fprintf(stderr, "Failed to set samplerate = %u: %s\n",
                c->samplerate, bladerf_strerror(status));
        return status;
    }
    status = bladerf_set_bandwidth(dev, c->module, c->bandwidth, NULL);
    if (status != 0){
        fprintf(stderr, "Failed to set bandwidth = %u: %s\n",
                c->bandwidth, bladerf_strerror(status));
        return status;
    }
    if (c->biastee){
        status = bladerf_set_bias_tee(dev, c->module, c->biastee);
        if (status != 0) {
            fprintf(stderr, "Failed to set bias-tee = %d: %s\n",
                    c->biastee, bladerf_strerror(status));
            return status;
        }
    }

    if (BLADERF_CHANNEL_IS_TX(c->module)){
        //TX gains
        if (c->use_unified) {
            //Check to see if gain is within bounds of device
            status = bladerf_get_gain_range(dev, c->module, &gain_range);
            if (status != 0){
                fprintf(stderr, "Failed to get TX gain range: %s\n",
                        bladerf_strerror(status));
                return status;
            }
            if (c->unified_gain < gain_range->min || c->unified_gain > gain_range->max){
                fprintf(stderr, "Requested TX gain %d is outside supported range [%" PRId64 ", %" PRId64 "]\n",
                        c->unified_gain, gain_range->min, gain_range->max);
                return BLADERF_ERR_RANGE;
            }

            status = bladerf_set_gain(dev, c->module, c->unified_gain);
            if (status != 0){
                fprintf(stderr, "Failed to set unified TX gain: %s\n",
                        bladerf_strerror(status));
                return status;
            }
        }else{
            //Configure the TX VGA1 and TX VGA2 gains using gain stages
            status = bladerf_set_gain_stage(dev, c->module, "txvga1", c->vga1);
            if (status != 0){
                fprintf(stderr, "Failed to set TX VGA1 gain: %s\n",
                        bladerf_strerror(status));
                return status;
            }
            status = bladerf_set_gain_stage(dev, c->module, "txvga2", c->vga2);
            if (status != 0){
                fprintf(stderr, "Failed to set TX VGA2 gain: %s\n",
                        bladerf_strerror(status));
                return status;
            }
        }

    }else{
        //RX gains
        if (c->use_agc) {
            const char       *board_name = bladerf_get_board_name(dev);
            bladerf_gain_mode agc_mode;

            if (strcmp(board_name, "bladerf2") == 0){
                //Found that fastattack works best
                agc_mode = BLADERF_GAIN_FASTATTACK_AGC; //bladeRF2 AGC
            }else{
                agc_mode = BLADERF_GAIN_DEFAULT;        //bladeRF1 AGC
            }

            status = bladerf_set_gain_mode(dev, c->module, agc_mode);
            if (status != 0) {
                fprintf(stderr, "Failed enable RX AGC: %s\n",
                        bladerf_strerror(status));
                return status;
            }
        }else{
            status = bladerf_set_gain_mode(dev, c->module, BLADERF_GAIN_MGC);
            if (status != 0){
                fprintf(stderr, "Failed to disable RX AGC: %s\n",
                        bladerf_strerror(status));
                return status;
            }
            if (c->use_unified){
                //Check to see if gain is within bounds of device
                status = bladerf_get_gain_range(dev, c->module, &gain_range);
                if (status != 0){
                    fprintf(stderr, "Failed to get RX gain range: %s\n",
                            bladerf_strerror(status));
                    return status;
                }
                if (c->unified_gain < gain_range->min || c->unified_gain > gain_range->max){
                    fprintf(stderr, "Requested RX gain %d is outside supported range [%" PRId64 ", %" PRId64 "]\n",
                            c->unified_gain, gain_range->min, gain_range->max);
                    return BLADERF_ERR_RANGE;
                }

                status = bladerf_set_gain(dev, c->module, c->unified_gain);
                if (status != 0){
                    fprintf(stderr, "Failed to set unified RX gain: %s\n",
                            bladerf_strerror(status));
                    return status;
                }
            }else{
                //Configure the gains using gain stages
                status = bladerf_set_gain_stage(dev, c->module, "lna", c->rx_lna);
                if (status != 0){
                    fprintf(stderr, "Failed to set RX LNA gain: %s\n",
                            bladerf_strerror(status));
                    return status;
                }
                status = bladerf_set_gain_stage(dev, c->module, "rxvga1", c->vga1);
                if (status != 0){
                    fprintf(stderr, "Failed to set RX VGA1 gain: %s\n",
                            bladerf_strerror(status));
                    return status;
                }
                status = bladerf_set_gain_stage(dev, c->module, "rxvga2", c->vga2);
                if (status != 0){
                    fprintf(stderr, "Failed to set RX VGA2 gain: %s\n",
                            bladerf_strerror(status));
                    return status;
                }
            }
        }
    }

    // //DEBUG: get gains and print them out
    // bladerf_gain_mode mode;
    // bladerf_gain      gain;
    // bladerf_lna_gain  lna_gain;
    // int               vga_gain;

    // //TX
    // status = bladerf_get_gain_mode(dev, BLADERF_MODULE_TX, &mode);
    // if (status != 0) {
    //     fprintf(stderr, "Failed to get TX gain mode: %s\n",
    //             bladerf_strerror(status));
    //     return status;
    // }
    // printf("TX gain mode = %d\n", mode);

    // status = bladerf_get_gain(dev, BLADERF_MODULE_TX, &gain);
    // if (status != 0) {
    //     fprintf(stderr, "Failed to get TX gain: %s\n",
    //             bladerf_strerror(status));
    //     return status;
    // }
    // printf("TX unified gain = %d\n", gain);

    // status = bladerf_get_txvga1(dev, &vga_gain);
    // if (status != 0) {
    //     fprintf(stderr, "Failed to get TX vga1 gain: %s\n",
    //             bladerf_strerror(status));
    //     return status;
    // }
    // printf("TX vga1 gain = %d\n", vga_gain);

    // status = bladerf_get_txvga2(dev, &vga_gain);
    // if (status != 0) {
    //     fprintf(stderr, "Failed to get TX vga2 gain: %s\n",
    //             bladerf_strerror(status));
    //     return status;
    // }
    // printf("TX vga2 gain = %d\n", vga_gain);

    // //RX
    // status = bladerf_get_gain_mode(dev, BLADERF_MODULE_RX, &mode);
    // if (status != 0) {
    //     fprintf(stderr, "Failed to get RX gain mode: %s\n",
    //             bladerf_strerror(status));
    //     return status;
    // }
    // printf("RX gain mode = %d\n", mode);

    // status = bladerf_get_gain(dev, BLADERF_MODULE_RX, &gain);
    // if (status != 0) {
    //     fprintf(stderr, "Failed to get RX gain: %s\n",
    //             bladerf_strerror(status));
    //     return status;
    // }
    // printf("RX unified gain = %d\n", gain);

    // status = bladerf_get_lna_gain(dev, &lna_gain);
    // if (status != 0) {
    //     fprintf(stderr, "Failed to get RX lna gain: %s\n",
    //             bladerf_strerror(status));
    //     return status;
    // }
    // printf("RX lna gain = %d\n", lna_gain);

    // status = bladerf_get_rxvga1(dev, &vga_gain);
    // if (status != 0) {
    //     fprintf(stderr, "Failed to get RX vga1 gain: %s\n",
    //             bladerf_strerror(status));
    //     return status;
    // }
    // printf("RX vga1 gain = %d\n", vga_gain);

    // status = bladerf_get_rxvga2(dev, &vga_gain);
    // if (status != 0) {
    //     fprintf(stderr, "Failed to get RX vga2 gain: %s\n",
    //             bladerf_strerror(status));
    //     return status;
    // }
    // printf("RX vga2 gain = %d\n", vga_gain);

    return status;
}

/**
 * Initialize synchronous interface
 */
static int radio_init_sync(struct bladerf *dev)
{
    int status;
    bladerf_format format            = BLADERF_FORMAT_SC16_Q11_META;
    const unsigned int num_buffers   = 64;
    const unsigned int buffer_size   = SYNC_BUFFER_SIZE;  /* Must be a multiple of 1024 */
    const unsigned int num_transfers = 16;
    const unsigned int timeout_ms    = 3500;
    #ifdef SYNC_NO_METADATA
        DEBUG_MSG("Configuring synchronous interface WITHOUT metadata\n");
        format = BLADERF_FORMAT_SC16_Q11;
    #endif

    //Configure both the device's RX and TX modules for use with the synchronous interface
    status = bladerf_sync_config(dev,
                                 BLADERF_RX_X1,
                                 format,
                                 num_buffers,
                                 buffer_size,
                                 num_transfers,
                                 timeout_ms);
    if (status != 0) {
        fprintf(stderr, "Failed to configure RX sync interface: %s\n",
                bladerf_strerror(status));
        return status;
    }
    status = bladerf_sync_config(dev,
                                 BLADERF_TX_X1,
                                 format,
                                 num_buffers,
                                 buffer_size,
                                 num_transfers,
                                 timeout_ms);
    if (status != 0) {
        fprintf(stderr, "Failed to configure TX sync interface: %s\n",
                bladerf_strerror(status));
    }
    return status;
}

int radio_init_and_configure(struct bladerf *dev, struct radio_params *params)
{
    struct module_config config;
    int status;

    #ifdef VERBOSE_MODE
        bladerf_log_set_verbosity(BLADERF_LOG_LEVEL_VERBOSE);
    #elif defined(DEBUG_MODE)
        bladerf_log_set_verbosity(BLADERF_LOG_LEVEL_DEBUG);
    #endif

    // //DEBUG Set SMD mode
    // status = bladerf_set_smb_mode(dev, BLADERF_SMB_MODE_DISABLED);
    // // status = bladerf_set_smb_mode(dev, BLADERF_SMB_MODE_INPUT);
    // // status = bladerf_set_smb_mode(dev, BLADERF_SMB_MODE_OUTPUT);
    // if (status != 0){
    //     fprintf(stderr, "Failed to set SMD mode: %s\n",
    //             bladerf_strerror(status));
    // }
    // bladerf_smb_mode mode;
    // status = bladerf_get_smb_mode(dev, &mode);
    // if (status != 0){
    //     fprintf(stderr, "Failed to get SMD mode: %s\n",
    //             bladerf_strerror(status));
    // }
    // DEBUG_MSG("smb_mode = %d\n", mode);
    // // DEBUG_MSG("Sleeping for 15 seconds...\n");
    // // sleep(15);

    //Check for invalid channel index outside of supported range
    if (params->tx_chan >= (int)bladerf_get_channel_count(dev, BLADERF_TX)){
        fprintf(stderr, "Requested TX channel (%d) exceeds supported range [0, %d]\n",
                         params->tx_chan, (int)bladerf_get_channel_count(dev, BLADERF_TX) - 1);
        return BLADERF_ERR_RANGE;
    }
    if (params->rx_chan >= (int)bladerf_get_channel_count(dev, BLADERF_RX)){
        fprintf(stderr, "Requested RX channel (%d) exceeds supported range [0, %d]\n",
                         params->rx_chan, (int)bladerf_get_channel_count(dev, BLADERF_RX) - 1);
        return BLADERF_ERR_RANGE;
    }

    //Configure TX parameters
    config.module       = BLADERF_CHANNEL_TX(params->tx_chan);
    config.frequency    = params->tx_freq;
    config.bandwidth    = BLADERF_BANDWIDTH;
    config.samplerate   = BLADERF_SAMPLE_RATE;
    config.vga1         = params->tx_vga1_gain;
    config.vga2         = params->tx_vga2_gain;
    config.use_unified  = params->tx_use_unified;
    config.unified_gain = params->tx_unified_gain;
    config.biastee      = params->tx_biastee;
    status = radio_configure_module(dev, &config);
    if (status != 0){
        fprintf(stderr, "Couldn't configure TX module: %s\n", bladerf_strerror(status));
        return status;
    }

    //Configure RX parameters
    config.module       = BLADERF_CHANNEL_RX(params->rx_chan);
    config.frequency    = params->rx_freq;
    config.bandwidth    = BLADERF_BANDWIDTH;
    config.samplerate   = BLADERF_SAMPLE_RATE;
    config.rx_lna       = params->rx_lna_gain;
    config.vga1         = params->rx_vga1_gain;
    config.vga2         = params->rx_vga2_gain;
    config.use_unified  = params->rx_use_unified;
    config.use_agc      = params->rx_agc;
    config.unified_gain = params->rx_unified_gain;
    config.biastee      = params->rx_biastee;
    status = radio_configure_module(dev, &config);
    if (status != 0){
        fprintf(stderr, "Couldn't configure RX module: %s\n", bladerf_strerror(status));
        return status;
    }

    //Unset loopback
    status = bladerf_set_loopback(dev, BLADERF_LB_NONE);
    if (status != 0){
        fprintf(stderr, "Couldn't set loopback: %s", bladerf_strerror(status));
        return status;
    }

    //Initialize synchronous interface
    status = radio_init_sync(dev);
    if (status != 0){
        fprintf(stderr, "Couldn't initialize synchronous interface: %s\n",
                bladerf_strerror(status));
        return status;
    }

    //Enable tx module
    status = bladerf_enable_module(dev, BLADERF_CHANNEL_TX(params->tx_chan), true);
    if (status != 0){
        fprintf(stderr, "Couldn't enable TX module: %s\n", bladerf_strerror(status));
        return status;
    }
    //Enable rx module
    status = bladerf_enable_module(dev, BLADERF_CHANNEL_RX(params->rx_chan), true);
    if (status != 0){
        fprintf(stderr, "Couldn't enable RX module: %s\n", bladerf_strerror(status));
        return status;
    }

    return 0;
}

void radio_stop(struct bladerf *dev)
{
    int status;

    if (dev == NULL){
        return;
    }
    //Disable tx module
    status = bladerf_enable_module(dev, BLADERF_MODULE_TX, false);
    if (status != 0){
        fprintf(stderr, "Couldn't disable TX module: %s\n", bladerf_strerror(status));
    }
    //Disable rx module
    status = bladerf_enable_module(dev, BLADERF_MODULE_RX, false);
    if (status != 0){
        fprintf(stderr, "Couldn't disable RX module: %s\n", bladerf_strerror(status));
    }
}
