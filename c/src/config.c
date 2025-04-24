/**
 * @brief   Gets configuration options from command line arguments
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

#include "config.h"
#include "utils.h"

#ifdef DEBUG_MODE
#   define DEBUG_MSG(...) fprintf(stderr, "[CONFIG] " __VA_ARGS__)
#else
#   define DEBUG_MSG(...)
#endif

#define OPTIONS "hd:p:r:o:t:i:q"

#define OPTION_HELP        'h'
#define OPTION_DEVICE      'd'
#define OPTION_PAYLOAD_LEN 'p'
#define OPTION_QUIET       'q'


#define OPTION_RXFREQ   'r'
#define OPTION_INPUT    'i'
#define OPTION_RXLNA    0x80
#define OPTION_RXVGA1   0x81
#define OPTION_RXVGA2   0x82
#define OPTION_RXGAIN   0x83
#define OPTION_RXBIAST  0x84
#define OPTION_RXAGC    0x85
#define OPTION_RXCHAN   0x86

#define OPTION_TXFREQ   't'
#define OPTION_OUTPUT   'o'
#define OPTION_TXVGA1   0x90
#define OPTION_TXVGA2   0x91
#define OPTION_TXGAIN   0x92
#define OPTION_TXBIAST  0x93
#define OPTION_TXCHAN   0x94

//--Frequency definitions for argument checking
#define BLADERF1_FREQ_MIN BLADERF_FREQUENCY_MIN
#define BLADERF1_FREQ_MAX BLADERF_FREQUENCY_MAX
#define BLADERF2_FREQ_MIN 47000000
#define BLADERF2_FREQ_MAX 6000000000
//Worst case max and min between both devices, based on the above numbers
#define FREQ_MIN ((BLADERF1_FREQ_MIN < BLADERF2_FREQ_MIN) ? BLADERF1_FREQ_MIN \
                                                          : BLADERF2_FREQ_MIN)
#define FREQ_MAX ((BLADERF1_FREQ_MAX > BLADERF2_FREQ_MAX) ? BLADERF1_FREQ_MAX \
                                                          : BLADERF2_FREQ_MAX)

//--Gain definitions for argument checking and help message
#define BLADERF1_TX_GAIN_MAX 73
#define BLADERF1_TX_GAIN_MIN \
    BLADERF1_TX_GAIN_MAX - \
    (BLADERF_TXVGA1_GAIN_MAX - BLADERF_TXVGA1_GAIN_MIN) - \
    (BLADERF_TXVGA2_GAIN_MAX - BLADERF_TXVGA2_GAIN_MIN)
#define BLADERF1_RX_GAIN_MAX 60
#define BLADERF1_RX_GAIN_MIN \
    BLADERF1_RX_GAIN_MAX - \
    BLADERF_LNA_GAIN_MAX_DB - \
    (BLADERF_RXVGA1_GAIN_MAX - BLADERF_RXVGA1_GAIN_MIN) - \
    (BLADERF_RXVGA2_GAIN_MAX - BLADERF_RXVGA2_GAIN_MIN)
//Hardcoded bladeRF 2 max/min values. No existing macros for these.
#define BLADERF2_TX_GAIN_MAX 66
#define BLADERF2_TX_GAIN_MIN BLADERF2_TX_GAIN_MAX - 90
#define BLADERF2_RX_GAIN_MAX 60
#define BLADERF2_RX_GAIN_MIN BLADERF2_RX_GAIN_MAX - 76  //sometimes higher than this depending on freq

//Worst case max and min between both devices, based on the above numbers
#define TX_GAIN_MIN ((BLADERF1_TX_GAIN_MIN < BLADERF2_TX_GAIN_MIN) ? BLADERF1_TX_GAIN_MIN \
                                                                   : BLADERF2_TX_GAIN_MIN)
#define TX_GAIN_MAX ((BLADERF1_TX_GAIN_MAX > BLADERF2_TX_GAIN_MAX) ? BLADERF1_TX_GAIN_MAX \
                                                                   : BLADERF2_TX_GAIN_MAX)
#define RX_GAIN_MIN ((BLADERF1_RX_GAIN_MIN < BLADERF2_RX_GAIN_MIN) ? BLADERF1_RX_GAIN_MIN \
                                                                   : BLADERF2_RX_GAIN_MIN)
#define RX_GAIN_MAX ((BLADERF1_RX_GAIN_MAX > BLADERF2_RX_GAIN_MAX) ? BLADERF1_RX_GAIN_MAX \
                                                                   : BLADERF2_RX_GAIN_MAX)

#define CHAN_MIN 0
#define CHAN_MAX 1

//--Defaults
#define RX_FREQ_DEFAULT 904000000
#define RX_LNA_DEFAULT  BLADERF_LNA_GAIN_MAX
#define RX_VGA1_DEFAULT BLADERF_RXVGA1_GAIN_MAX
#define RX_VGA2_DEFAULT BLADERF_RXVGA2_GAIN_MIN

#define TX_FREQ_DEFAULT 924000000
#define TX_VGA1_DEFAULT BLADERF_TXVGA1_GAIN_MAX
#define TX_VGA2_DEFAULT BLADERF_TXVGA2_GAIN_MIN

#define CHAN_DEFAULT CHAN_MIN
#define TX_GAIN_DEFAULT 50
#define RX_GAIN_DEFAULT 30

#define PAYLOAD_LEN_MIN     1
#define PAYLOAD_LEN_MAX     1000000
#define PAYLOAD_LEN_DEFAULT 1000

static struct option long_options[] = {
    { "help",        no_argument,        NULL,   OPTION_HELP        },
    { "device",      required_argument,  NULL,   OPTION_DEVICE      },
    { "quiet",       no_argument,        NULL,   OPTION_QUIET       },
    { "payload_len", required_argument,  NULL,   OPTION_PAYLOAD_LEN },

    { "output",      required_argument,  NULL,   OPTION_OUTPUT      },
    { "rx-lna",      required_argument,  NULL,   OPTION_RXLNA       },
    { "rx-vga1",     required_argument,  NULL,   OPTION_RXVGA1      },
    { "rx-vga2",     required_argument,  NULL,   OPTION_RXVGA2      },
    { "rx-freq",     required_argument,  NULL,   OPTION_RXFREQ      },
    { "rx-gain",     required_argument,  NULL,   OPTION_RXGAIN      },
    { "rx-biast",    no_argument,        NULL,   OPTION_RXBIAST     },
    { "rx-agc",      no_argument,        NULL,   OPTION_RXAGC       },
    { "rx-chan",     required_argument,  NULL,   OPTION_RXCHAN      },

    { "input",       required_argument,  NULL,   OPTION_INPUT       },
    { "tx-vga1",     required_argument,  NULL,   OPTION_TXVGA1      },
    { "tx-vga2",     required_argument,  NULL,   OPTION_TXVGA2      },
    { "tx-freq",     required_argument,  NULL,   OPTION_TXFREQ      },
    { "tx-gain",     required_argument,  NULL,   OPTION_TXGAIN      },
    { "tx-biast",    no_argument,        NULL,   OPTION_TXBIAST     },
    { "tx-chan",     required_argument,  NULL,   OPTION_TXCHAN      },

    { NULL,          0,                  NULL,   0                  },
};

const struct numeric_suffix freq_suffixes[] = {
    { "k",      1000 },
    { "KHz",    1000 },

    { "M",      1000 * 1000 },
    { "MHz",    1000 * 1000 },

    { "G",      1000 * 1000 * 1000 },
    { "GHz",    1000 * 1000 * 1000 },
};

static const size_t num_freq_suffixes =
    sizeof(freq_suffixes) / sizeof(freq_suffixes[0]);

#if defined(DEBUG_MODE) || defined(CONFIG_TEST)
    void print_config(const struct config *config)
    {
        printf("  bladeRF handle:     %p\n",  config->bladerf_dev);
        printf("  payload length:     %u\n",  config->payload_length);
        printf("  quiet:              %b\n",  config->quiet);
        printf("\n");
        printf("  RX Parameters:\n");
        printf("    Output handle:    %p\n",  config->rx_output);
        printf("    Frequency [Hz]:   %lu\n", config->params.rx_freq);
        printf("    Channel:          %d\n",  config->params.rx_chan);
        printf("    Biastee:          %d\n",  config->params.rx_biastee);
        printf("    AGC enabled:      %d\n",  config->params.rx_agc);
        printf("    Use unified gain: %d\n",  config->params.rx_use_unified);
        printf("    Unified gain:     %d\n",  config->params.rx_unified_gain);
        printf("    LNA gain:         %d\n",  config->params.rx_lna_gain);
        printf("    VGA1 gain:        %d\n",  config->params.rx_vga1_gain);
        printf("    VGA2 gain:        %d\n",  config->params.rx_vga2_gain);
        printf("\n");
        printf("  TX Parameters:\n");
        printf("    Input handle:     %p\n",  config->tx_input);
        printf("    File size:        %lu\n", config->tx_filesize);
        printf("    Frequency [Hz]:   %lu\n", config->params.tx_freq);
        printf("    Channel:          %d\n",  config->params.tx_chan);
        printf("    Biastee:          %d\n",  config->params.tx_biastee);
        printf("    Use unified gain: %d\n",  config->params.tx_use_unified);
        printf("    Unified gain:     %d\n",  config->params.tx_unified_gain);
        printf("    VGA1 gain:        %d\n",  config->params.tx_vga1_gain);
        printf("    VGA2 gain:        %d\n",  config->params.tx_vga2_gain);
        printf("\n");
    }
#endif

static struct config *alloc_config_with_defaults()
{
    struct config *config;

    //calloc so everything is initialized to 0/false/NULL
    config = calloc(1, sizeof(*config));
    if (!config) {
        perror("calloc");
        return NULL;
    }

    /* RX defaults */
    config->rx_output              = NULL;
    config->params.rx_freq         = RX_FREQ_DEFAULT;
    config->params.rx_chan         = CHAN_DEFAULT;
    config->params.rx_lna_gain     = RX_LNA_DEFAULT;
    config->params.rx_vga1_gain    = RX_VGA1_DEFAULT;
    config->params.rx_vga2_gain    = RX_VGA2_DEFAULT;
    config->params.rx_agc          = 1;
    config->params.rx_use_unified  = 1;
    config->params.rx_unified_gain = RX_GAIN_DEFAULT;
    config->params.rx_biastee      = 0;

    /* TX defaults */
    config->tx_input               = NULL;
    config->params.tx_freq         = TX_FREQ_DEFAULT;
    config->params.tx_chan         = CHAN_DEFAULT;
    config->params.tx_vga1_gain    = TX_VGA1_DEFAULT;
    config->params.tx_vga2_gain    = TX_VGA2_DEFAULT;
    config->params.tx_use_unified  = 1;
    config->params.tx_unified_gain = TX_GAIN_DEFAULT;
    config->params.tx_biastee      = 0;

    config->payload_length         = PAYLOAD_LEN_DEFAULT;

    return config;
}

/* Initialize configuration items not provided by user */
static int init_remaining_params(struct config *config)
{
    int status;

    if (config->bladerf_dev == NULL) {
        status = bladerf_open(&config->bladerf_dev, NULL);
        if (status != 0) {
            fprintf(stderr, "Failed to open any available bladeRF: %s\n",
                    bladerf_strerror(status));
            return status;
        }
    }

    if (config->rx_output == NULL) {
        config->rx_output = stdout;
    }

    if (config->tx_input == NULL) {
        config->tx_input = stdin;
        config->tx_filesize = -1;
    }

    return 0;
}

int config_init_from_cmdline(int argc, char * const argv[], struct config **config_out)
{
    int status = 0;
    struct config *config = NULL;
    int c, idx;
    bool valid;
    struct stat file_info;

    config = alloc_config_with_defaults();
    if (config == NULL) {
        DEBUG_MSG("%s: Failed to alloc config.\n", __FUNCTION__);
        return -2;
    }

    while ((c = getopt_long(argc, argv, OPTIONS, long_options, &idx)) != -1) {
        switch (c) {
            case OPTION_HELP:
                status = 1;
                goto out;

            case OPTION_DEVICE:
                if (config->bladerf_dev == NULL) {
                    status = bladerf_open(&config->bladerf_dev, optarg);
                    if (status < 0) {
                        fprintf(stderr, "Failed to open bladeRF=%s: %s\n",
                                optarg, bladerf_strerror(status));
                        goto out;
                    }
                }
                break;

            case OPTION_OUTPUT:
                if (config->rx_output == NULL) {
                    if (!strcasecmp(optarg, "stdout")) {
                        config->rx_output = stdout;
                    } else {
                        config->rx_output = fopen(optarg, "wb");
                        if (config->rx_output == NULL) {
                            status = -errno;
                            fprintf(stderr, "Failed to open %s for writing: %s\n",
                                    optarg, strerror(-status));
                            goto out;
                        }
                    }
                }
                break;

            case OPTION_RXLNA:
                config->params.rx_agc         = 0;
                config->params.rx_use_unified = 0;
                status = str2lnagain(optarg, &config->params.rx_lna_gain);
                if (status != 0) {
                    fprintf(stderr, "Invalid RX LNA gain: %s\n", optarg);
                    goto out;
                }
                break;

            case OPTION_RXVGA1:
                config->params.rx_agc         = 0;
                config->params.rx_use_unified = 0;
                config->params.rx_vga1_gain   =
                    str2int(optarg,
                            BLADERF_RXVGA1_GAIN_MIN, BLADERF_RXVGA1_GAIN_MAX,
                            &valid);

                if (!valid) {
                    status = -1;
                    fprintf(stderr, "Invalid RX VGA1 gain: %s\n", optarg);
                    goto out;
                }
                break;

            case OPTION_RXVGA2:
                config->params.rx_agc         = 0;
                config->params.rx_use_unified = 0;
                config->params.rx_vga2_gain   =
                    str2int(optarg,
                            BLADERF_RXVGA2_GAIN_MIN, BLADERF_RXVGA2_GAIN_MAX,
                            &valid);

                if (!valid) {
                    status = -1;
                    fprintf(stderr, "Invalid RX VGA2 gain: %s\n", optarg);
                    goto out;
                }
                break;

            case OPTION_RXFREQ:
                config->params.rx_freq =
                    str2uint64_suffix(optarg,
                                      FREQ_MIN, FREQ_MAX,
                                      freq_suffixes, num_freq_suffixes,
                                      &valid);

                if (!valid) {
                    status = -1;
                    fprintf(stderr, "Invalid RX frequency: %s\n", optarg);
                    goto out;
                }
                break;

            case OPTION_RXGAIN:
                config->params.rx_agc          = 0;
                config->params.rx_use_unified  = 1;
                config->params.rx_unified_gain = str2int(optarg,
                                                         RX_GAIN_MIN, RX_GAIN_MAX,
                                                         &valid);

                if (!valid) {
                    status = -1;
                    fprintf(stderr, "Invalid unified RX gain: %s\n", optarg);
                    goto out;
                }
                break;

            case OPTION_RXAGC:
                config->params.rx_agc = 1;
                break;

            case OPTION_RXBIAST:
                config->params.rx_biastee = 1;
                break;

            case OPTION_RXCHAN:
                config->params.rx_chan = str2int(optarg, CHAN_MIN, CHAN_MAX, &valid);
                if (!valid) {
                    status = -1;
                    fprintf(stderr, "Invalid RX Channel: %s\n", optarg);
                    goto out;
                }
                break;

            case OPTION_INPUT:
                if (config->tx_input == NULL) {
                    if (!strcasecmp(optarg, "stdin")) {
                        config->tx_input = stdin;
                        config->tx_filesize = -1;
                    } else {
                        config->tx_input = fopen(optarg, "rb");
                        if (config->tx_input == NULL) {
                            status = -errno;
                            fprintf(stderr, "Failed to open %s for reading: %s\n",
                                    optarg, strerror(-status));
                            goto out;
                        }
                        //Get file size
                        status = stat(optarg, &file_info);
                        if (status != 0){
                            fprintf(stderr, "Couldn't get filesize of %s: %s\n",
                                    optarg, strerror(status));
                        }
                        config->tx_filesize = file_info.st_size;
                    }
                }
                break;

            case OPTION_TXVGA1:
                config->params.tx_use_unified = 0;
                config->params.tx_vga1_gain   =
                    str2int(optarg,
                            BLADERF_TXVGA1_GAIN_MIN, BLADERF_TXVGA2_GAIN_MAX,
                            &valid);

                if (!valid) {
                    status = -1;
                    fprintf(stderr, "Invalid TX VGA1 gain: %s\n", optarg);
                    goto out;
                }
                break;

            case OPTION_TXVGA2:
                config->params.tx_use_unified = 0;
                config->params.tx_vga2_gain   =
                    str2int(optarg,
                            BLADERF_TXVGA2_GAIN_MIN, BLADERF_TXVGA2_GAIN_MAX,
                            &valid);

                if (!valid) {
                    status = -1;
                    fprintf(stderr, "Invalid TX VGA2 gain: %s\n", optarg);
                    goto out;
                }
                break;

            case OPTION_TXGAIN:
                config->params.tx_use_unified  = 1;
                config->params.tx_unified_gain = str2int(optarg,
                                                         TX_GAIN_MIN, TX_GAIN_MAX,
                                                         &valid);

                if (!valid) {
                    status = -1;
                    fprintf(stderr, "Invalid unified TX gain: %s\n", optarg);
                    goto out;
                }
                break;

            case OPTION_TXBIAST:
                config->params.tx_biastee = 1;
                break;

            case OPTION_TXFREQ:
                config->params.tx_freq =
                    str2uint64_suffix(optarg,
                                      FREQ_MIN, FREQ_MAX,
                                      freq_suffixes, num_freq_suffixes,
                                      &valid);
                if (!valid) {
                    status = -1;
                    fprintf(stderr, "Invalid TX frequency: %s\n", optarg);
                    goto out;
                }
                break;

            case OPTION_TXCHAN:
                config->params.tx_chan = str2int(optarg, CHAN_MIN, CHAN_MAX, &valid);
                if (!valid) {
                    status = -1;
                    fprintf(stderr, "Invalid TX Channel: %s\n", optarg);
                    goto out;
                }
                break;

            case OPTION_QUIET:
                config->quiet = true;
                break;

            case OPTION_PAYLOAD_LEN:
                config->payload_length = str2uint(optarg,
                                                  PAYLOAD_LEN_MIN, PAYLOAD_LEN_MAX,
                                                  &valid);
                if (!valid) {
                    status = -1;
                    fprintf(stderr, "Invalid payload length: %s. Range: %u to %u\n",
                            optarg, PAYLOAD_LEN_MIN, PAYLOAD_LEN_MAX);
                    goto out;
                }
                break;
        }
    }

    /* Initialize any properties not covered by user input */
    status = init_remaining_params(config);

out:
    if (status != 0) {
        config_deinit(config);
        *config_out = NULL;
    } else {
        #ifdef DEBUG_MODE
            DEBUG_MSG("Program configuration:\n");
            print_config(config);
        #endif
        *config_out = config;
    }

    return status;
}

void config_deinit(struct config *config)
{
    if (config == NULL) {
        DEBUG_MSG("%s: NULL ptr\n", __FUNCTION__);
        return;
    }

    DEBUG_MSG("Deinitializing...\n");

    if (config->bladerf_dev != NULL) {
        DEBUG_MSG("   Closing bladerf.\n");
        bladerf_close(config->bladerf_dev);
        config->bladerf_dev = NULL;
    }
    if (config->rx_output != NULL) {
        fclose(config->rx_output);
        config->rx_output = NULL;
    }

    if (config->tx_input != NULL) {
        fclose(config->tx_input);
        config->tx_input = NULL;
    }

    free(config);
}

void config_print_options()
{
    printf(

"   -h, --help              Show this help text.\n"
"   -d, --device <str>      Open the specified bladeRF device. Default: any available device.\n"
"   -p, --packet_size <val> Payload data length per link layer frame [bytes]. Default: %u\n"
"                             Both ends of the link must use the same size.\n"
"   -q, --quiet             Suppress printing of banner/exit messages.\n"
"\n"
"   -r, --rx-freq <freq>    RX frequency [Hz]. May use shorthand (ex: 904M). Default: %d\n"
"   -o, --output  <file>    RX data output. Default: stdout.\n"
"\n"
"   -t, --tx-freq <freq>    TX frequency [Hz]. May use shorthand (ex: 924M). Default: %d\n"
"   -i, --input   <file>    TX data input. Default: stdin.\n"
"\n\n"
"   --rx-chan <value>       RX channel. Range: %d to %d. Default: %d.            [bladeRF 2 only]\n"
"   --rx-biast              Enable bias-tee amplifier accessory on RX channel [bladeRF 2 only]\n"
"\n"
"   --tx-chan <value>       TX channel. Range: %d to %d. Default: %d.            [bladeRF 2 only]\n"
"   --tx-biast              Enable bias-tee amplifier accessory on TX channel [bladeRF 2 only]\n"
"\n"
"   RX gains:\n"
"   The default RX gain mode is to enable the AGC. To use manual unified gains, use --rx-gain.\n"
"     Alternatively, to set specific gain stages on the bladeRF 1 use --rx-lna, --rx-vga1, --rx-vga2\n"
"\n"
"   --rx-agc                Enables slow moving RX AGC (enabled by default)\n"
"   --rx-gain <value>       RX unified gain [dB]\n"
"                             Range: %d to %d (bladeRF 1), %d to %d (bladeRF 2). Default: %d.\n"
"   --rx-lna  <value>       RX LNA gain. Values: bypass, mid, max;\n"
"                             representing %d to %d dB. Default: max          [bladeRF 1 only]\n"
"   --rx-vga1 <value>       RX VGA1 gain [dB]. Range: %d to %d. Default: %d. [bladeRF 1 only]\n"
"   --rx-vga2 <value>       RX VGA2 gain [dB]. Range: %d to %d. Default: %d.  [bladeRF 1 only]\n"
"\n"
"   TX gains:\n"
"   The default TX gain mode is to use unified gains, which can be set with --tx-gain.\n"
"     Alternatively, to set specific gain stages on the bladeRF 1 use --tx-vga1, --tx-vga2\n"
"\n"
"   --tx-gain <value>       TX unified gain [dB]\n"
"                             Range: %d to %d (bladeRF 1), %d to %d (bladeRF 2). Default: %d.\n"
"   --tx-vga1 <value>       TX VGA1 gain [dB]. Range: %d to %d. Default: %d. [bladeRF 1 only]\n"
"   --tx-vga2 <value>       TX VGA2 gain [dB]. Range: %d to %d. Default: %d.    [bladeRF 1 only]\n",

    PAYLOAD_LEN_DEFAULT,

    RX_FREQ_DEFAULT, TX_FREQ_DEFAULT,

    CHAN_MIN, CHAN_MAX, CHAN_DEFAULT,
    CHAN_MIN, CHAN_MAX, CHAN_DEFAULT,

    BLADERF1_RX_GAIN_MIN, BLADERF1_RX_GAIN_MAX, BLADERF2_RX_GAIN_MIN, BLADERF2_RX_GAIN_MAX, RX_GAIN_DEFAULT,
    0, BLADERF_LNA_GAIN_MAX_DB,
    BLADERF_RXVGA1_GAIN_MIN, BLADERF_RXVGA1_GAIN_MAX, RX_VGA1_DEFAULT,
    BLADERF_RXVGA2_GAIN_MIN, BLADERF_RXVGA2_GAIN_MAX, RX_VGA2_DEFAULT,

    BLADERF1_TX_GAIN_MIN, BLADERF1_TX_GAIN_MAX, BLADERF2_TX_GAIN_MIN, BLADERF2_TX_GAIN_MAX, TX_GAIN_DEFAULT,
    BLADERF_TXVGA1_GAIN_MIN, BLADERF_TXVGA1_GAIN_MAX, TX_VGA1_DEFAULT,
    BLADERF_TXVGA2_GAIN_MIN, BLADERF_TXVGA2_GAIN_MAX, TX_VGA2_DEFAULT

    );
}

#ifdef CONFIG_TEST
void print_test_help(const char *argv0)
{
    printf("Config structure test\n");
    printf("Usage: %s [options]\n\n", argv0);
    printf("Options:\n");
    config_print_options();
    printf("\n");
}

int main(int argc, char *argv[])
{
    int status;
    struct config *config = NULL;

    status = config_init_from_cmdline(argc, argv, &config);
    if (status < 0) {
        return status;
    } else if (status == 1) {
        print_test_help(argv[0]);
        return 0;
    } else if (status > 0) {
        fprintf(stderr, "Unexpected return value: %d\n", status);
        return status;
    }

    print_config(config);
    config_deinit(config);
    return EXIT_SUCCESS;
}
#endif
