/**
 * @file
 * @brief   Various modem tests for phy.c, link.c, and fsk.c
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
#include <stdio.h>
#include <stdint.h>
#include <libbladeRF.h>
#include <stdlib.h>
//Utility files
#include "utils.h"
#include "prng.h"
//Units under test
#include "phy.h"
#include "link.h"
#include "fsk.h"

#ifdef DEBUG_MODE
    #define DEBUG_MSG(...) fprintf(stderr, "[TEST] " __VA_ARGS__)
#else
    #define DEBUG_MSG(...)
#endif

#define PAYLOAD_LENGTH    1000                  //link layer payload length
#define DATA_FRAME_LENGTH (PAYLOAD_LENGTH+9)    //link layer total frame length

/**
 * Test link layer code with data transfer between two devices
 */
int link_test(char *dev_id1, char *dev_id2, bladerf_frequency tx_freq1, bladerf_frequency tx_freq2, 
              bladerf_gain tx_gain, bladerf_gain rx_gain)
{
    struct link_handle *link1 = NULL;
    struct link_handle *link2 = NULL;
    int                 status = 0;
    int                 bytes_received;
    uint8_t             tx_data[PAYLOAD_LENGTH];
    uint8_t             tx_data2[PAYLOAD_LENGTH] = "Another message";
    uint8_t             rx_data[PAYLOAD_LENGTH];
    struct radio_params params;
    struct bladerf     *dev1 = NULL, *dev2 = NULL;
    char               *result;
    bool                passed = 1;

    printf("------------BEGINNING LINK TEST-----------\n");
    //Open bladeRFs
    status = bladerf_open(&dev1, dev_id1);
    if (status != 0){
        fprintf(stderr, "Couldn't open bladeRF device #1: %s\n", bladerf_strerror(status));
        goto out;
    }
    status = bladerf_open(&dev2, dev_id2);
    if (status != 0){
        fprintf(stderr, "Couldn't open bladeRF device #2: %s\n", bladerf_strerror(status));
        goto out;
    }

    //link1 -> link2
    printf("Sending a message from link1->link2...\n");
    //Zero the tx_data buffer
    memset((void *)tx_data, 0, sizeof(tx_data));
    //Prompt for a string
    printf("Enter a string to tx using link1: ");
    result = fgets((char *) tx_data, sizeof(tx_data), stdin);
    if (result == NULL){
        status = -1;
        goto out;
    }
    //Remove newline character
    tx_data[strlen((char *)tx_data) - 1] = '\0';

    //Init link1
    params.tx_freq         = tx_freq1;
    params.tx_chan         = 0;
    params.tx_vga1_gain    = -4;
    params.tx_vga2_gain    = 0;
    params.tx_use_unified  = true;
    params.tx_unified_gain = tx_gain;
    params.tx_biastee      = false;

    params.rx_freq         = tx_freq2;
    params.rx_chan         = 0;
    params.rx_lna_gain     = BLADERF_LNA_GAIN_MAX;
    params.rx_vga1_gain    = 23;
    params.rx_vga2_gain    = 0;
    params.rx_use_unified  = true;
    params.rx_unified_gain = rx_gain;
    params.rx_biastee      = false;
    params.rx_agc          = false;

    link1 = link_init(dev1, &params, PAYLOAD_LENGTH);
    if (link1 == NULL){
        fprintf(stderr, "Couldn't initialize link1\n");
        status = -1;
        goto out;
    }
    //Init link2
    params.tx_freq = tx_freq2;
    params.rx_freq = tx_freq1;
    link2 = link_init(dev2, &params, PAYLOAD_LENGTH);
    if (link2 == NULL){
        fprintf(stderr, "Couldn't initialize link2\n");
        status = -1;
        goto out;
    }

    //Transmit with link1
    status = link_send_data(link1, tx_data, (unsigned int) strlen((char *)tx_data) + 1);
    if (status != 0){
        fprintf(stderr, "Couldn't send data with link1\n");
        goto out;
    }

    //Receive with link2
    bytes_received = link_receive_data(link2, (int)strlen((char *)tx_data) + 1, 5, rx_data);
    if (bytes_received < 0){
        fprintf(stderr, "Couldn't receive data with link2\n");
        status = -1;
        goto out;
    }else if (bytes_received != (int)(strlen((char *) tx_data) + 1)){
        fprintf(stderr, "link2 receive timed out\n");
        status = -1;
        goto out;
    }
    printf("Received on link2: '%s'\n", rx_data);
    if (strncmp((char *)rx_data, (char *)tx_data, sizeof(tx_data)) != 0){
        fprintf(stderr, "   ERROR: RX data did not match TX data");
        passed = 0;
    }else{
        printf("   RX data matched TX data\n");
    }

    // link1 -> link2
    //Transmit with link1
    printf("\nSending another message from link1->link2...\n");
    status = link_send_data(link1, tx_data2, sizeof(tx_data2));
    if (status != 0){
        fprintf(stderr, "Couldn't send data with link1\n");
        goto out;
    }

    //Receive with link2
    bytes_received = link_receive_data(link2, sizeof(tx_data2), 5, rx_data);
    if (bytes_received < 0){
        fprintf(stderr, "Couldn't receive data with link2\n");
        status = -1;
        goto out;
    }else if (bytes_received != sizeof(tx_data2)){
        fprintf(stderr, "link2 receive timed out\n");
        status = -1;
        goto out;
    }
    printf("Received on link2: '%s'\n", rx_data);
    if (memcmp(rx_data, tx_data2, sizeof(tx_data2)) != 0){
        fprintf(stderr, "   ERROR: RX data did not match TX data");
        passed = 0;
    }else{
        printf("   RX data matched TX data\n");
    }

    out:
        DEBUG_MSG("Closing link 1\n");
        link_close(link1);
        DEBUG_MSG("Closing link 2\n");
        link_close(link2);
        DEBUG_MSG("Closing bladeRFs\n");
        bladerf_close(dev1);
        bladerf_close(dev2);
        if (!passed || status != 0){
            status = -1;
            printf("\nTEST FAILED\n");
        }else{
            status = 0;
            printf("\nTEST PASSED\n");
        }
        printf("------------ENDING LINK TEST--------------\n");
        return status;
}

/**
 * Test phy layer code with data transfer between two devices
 */
int phy_test(char *dev_id1, char *dev_id2, bladerf_frequency tx_freq1, bladerf_frequency tx_freq2,
             bladerf_gain tx_gain, bladerf_gain rx_gain)
{
    struct phy_handle  *phy1 = NULL;
    struct phy_handle  *phy2 = NULL;
    uint8_t             tx_data[DATA_FRAME_LENGTH];
    uint8_t            *tx_data2 = NULL;
    uint8_t            *rx_data;
    uint64_t            prng_seed = 29398283513841632;
    int                 status = 0, ret;
    bool                rx_on = false, tx_on = false;
    struct radio_params params;
    struct bladerf     *dev1 = NULL, *dev2 = NULL;
    char               *result;
    bool                passed = 1;

    printf("------------BEGINNING PHY TEST------------\n");
    //phy1 -> phy2
    printf("Transmitting a message from phy1->phy2...\n");
    //Zero the tx_data buffer
    memset(tx_data, 0, sizeof(tx_data));
    //Set first byte to data frame code
    tx_data[0] = DATA_FRAME_CODE;
    //Prompt for a string
    printf("Enter a string to tx using phy1: ");
    result = fgets((char *) &tx_data[1], sizeof(tx_data)-1, stdin);
    if (result == NULL){
        status = -1;
        goto out;
    }
    //Remove newline character
    tx_data[strlen((char *)&tx_data[1])] = '\0';

    //Init phy1
    status = bladerf_open(&dev1, dev_id1);
    if (status != 0){
        fprintf(stderr, "Couldn't open bladeRF device #1: %s\n", bladerf_strerror(status));
        goto out;
    }
    params.tx_freq         = tx_freq1;
    params.tx_chan         = 0;
    params.tx_vga1_gain    = -4;
    params.tx_vga2_gain    = 0;
    params.tx_use_unified  = true;
    params.tx_unified_gain = tx_gain;
    params.tx_biastee      = false;

    params.rx_freq         = tx_freq2;
    params.rx_chan         = 0;
    params.rx_lna_gain     = BLADERF_LNA_GAIN_MAX;
    params.rx_vga1_gain    = 23;
    params.rx_vga2_gain    = 0;
    params.rx_use_unified  = true;
    params.rx_unified_gain = rx_gain;
    params.rx_biastee      = false;
    params.rx_agc          = false;

    phy1 = phy_init(dev1, &params, DATA_FRAME_LENGTH);
    if (phy1 == NULL){
        fprintf(stderr, "Couldn't initialize phy1\n");
        status = -1;
        goto out;
    }

    //Init phy2
    status = bladerf_open(&dev2, dev_id2);
    if (status != 0){
        fprintf(stderr, "Couldn't open bladeRF device #2: %s\n", bladerf_strerror(status));
        goto out;
    }
    params.tx_freq = tx_freq2;
    params.rx_freq = tx_freq1;
    phy2 = phy_init(dev2, &params, DATA_FRAME_LENGTH);
    if (phy2 == NULL){
        fprintf(stderr, "Couldn't initialize phy2\n");
        status = -1;
        goto out;
    }

    //Start receiving on phy2
    DEBUG_MSG("Starting phy2 receiver\n");
    status = phy_start_receiver(phy2);
    if (status != 0){
        fprintf(stderr, "Couldn't start phy2 receiver\n");
        goto out;
    }
    rx_on = true;

    //Transmit on phy1
    DEBUG_MSG("Starting phy1 transmitter\n");
    status = phy_start_transmitter(phy1);
    if (status != 0){
        fprintf(stderr, "Couldn't start phy1 transmitter\n");
        goto out;
    }
    tx_on = true;

    DEBUG_MSG("Transmitting on phy1\n");
    status = phy_fill_tx_buf(phy1, tx_data, sizeof(tx_data));
    if (status != 0){
        fprintf(stderr, "Couldn't fill tx buffer\n");
        goto out;
    }

    //Get the received frame from phy2
    DEBUG_MSG("Requesting RX buf on phy2\n");
    rx_data = phy_request_rx_buf(phy2, 6000);
    if (rx_data == NULL){
        fprintf(stderr, "ERROR: Request buffer failed\n");
        status = -1;
        goto out;
    }
    printf("Received from phy2: ");
    //print what is in the buffer
    print_chars(&rx_data[1], DATA_FRAME_LENGTH-1);
    //Release the rx buffer
    phy_release_rx_buf(phy2);
    //Compare with transmitted data
    if (memcmp(rx_data, tx_data, DATA_FRAME_LENGTH) != 0){
        fprintf(stderr, "   ERROR: RX data did not match TX data");
        passed = 0;
    }else{
        printf("   RX data matched TX data\n");
    }

    //phy1 -> phy2
    //Transmit a pseudo random sequence with phy1
    printf("\nTransmitting pseudo random sequence from phy1->phy2...\n");
    tx_data2    = prng_fill(&prng_seed, DATA_FRAME_LENGTH);
    tx_data2[0] = DATA_FRAME_CODE;        //Set frame type to data frame
    status = phy_fill_tx_buf(phy1, tx_data2, DATA_FRAME_LENGTH);
    if (status != 0){
        fprintf(stderr, "Couldn't transmit frame\n");
        goto out;
    }

    //Receive
    rx_data = phy_request_rx_buf(phy2, 2000);
    if (rx_data == NULL){
        fprintf(stderr, "ERROR: Request buffer failed\n");
        status = -1;
        goto out;
    }
    phy_release_rx_buf(phy2);
    //Compare with transmitted data
    if (memcmp(rx_data, tx_data2, DATA_FRAME_LENGTH) != 0){
        fprintf(stderr, "   ERROR: RX data did not match TX data");
        passed = 0;
    }else{
        printf("   RX data matched TX data\n");
    }

    out:
        ret = status;
        //Stop transmitters/receivers
        if (tx_on){
            status = phy_stop_transmitter(phy1);
            if (status != 0){
                fprintf(stderr, "Couldn't stop phy1 transmitter\n");
            }
        }
        if (rx_on){
            status = phy_stop_receiver(phy2);
            if (status != 0){
                if (status == 1){
                    fprintf(stderr, "WARNING: RX overruns were detected on PHY receiver\n");
                }else{
                    fprintf(stderr, "Couldn't stop phy2 receiver\n");
                }
            }
        }
        free(tx_data2);
        DEBUG_MSG("\tClosing phy1 and phy2\n");
        phy_close(phy1);
        phy_close(phy2);
        DEBUG_MSG("\tClosing bladeRFs\n");
        bladerf_close(dev1);
        bladerf_close(dev2);
        if (!passed || ret != 0 || status != 0){
            ret = -1;
            printf("\nTEST FAILED\n");
        }else{
            ret  = 0;
            printf("\nTEST PASSED\n");
        }
        printf("------------ENDING PHY TEST---------------\n");
        return ret;
}

/**
 * Test phy receiver thread to see if it's using too much CPU. If ENABLE_NOTES is
 * defined, overrun messages will be seen if the PHY receiver is at ~100% CPU.
 */
int phy_receive_test(char *dev_id)
{
    struct phy_handle  *phy = NULL;
    uint8_t            *rx_data;
    int                 status = 0, ret;
    bool                rx_on = false;
    struct radio_params params;
    struct bladerf     *dev = NULL;

    printf("------------BEGINNING PHY RECEIVER TEST------------\n");
    //Init phy
    status = bladerf_open(&dev, dev_id);
    if (status != 0){
        fprintf(stderr, "Couldn't open bladeRF device: %s\n", bladerf_strerror(status));
        goto out;
    }
    #ifdef DEBUG_MODE
        //print serial number
        struct bladerf_serial sn;
        status = bladerf_get_serial_struct(dev, &sn);
        if (status != 0){
            fprintf(stderr, "Couldn't get device serial number: %s\n", bladerf_strerror(status));
            goto out;
        }
        DEBUG_MSG("bladeRF serial#: %s\n", sn.serial);
    #endif

    params.tx_freq         = 904000000;
    params.tx_chan         = 0;
    params.tx_vga1_gain    = -4;
    params.tx_vga2_gain    = 0;
    params.tx_use_unified  = true;
    params.tx_unified_gain = 20;
    params.tx_biastee      = false;

    params.rx_freq         = 924000000;
    params.rx_chan         = 0;
    params.rx_lna_gain     = BLADERF_LNA_GAIN_MAX;
    params.rx_vga1_gain    = 23;
    params.rx_vga2_gain    = 0;
    params.rx_use_unified  = true;
    params.rx_unified_gain = 30;
    params.rx_biastee      = false;
    params.rx_agc          = false;

    phy = phy_init(dev, &params, DATA_FRAME_LENGTH);
    if (phy == NULL){
        fprintf(stderr, "Couldn't initialize phy\n");
        status = -1;
        goto out;
    }

    //Start receiving
    DEBUG_MSG("Starting phy receiver\n");
    status = phy_start_receiver(phy);
    if (status != 0){
        fprintf(stderr, "Couldn't start phy receiver\n");
        goto out;
    }
    rx_on = true;

    //Request data
    rx_data = phy_request_rx_buf(phy, 10000);
    if (rx_data == NULL){
        fprintf(stderr, "Request buffer failed (expected from this test, because there is no transmission)\n");
        goto out;
    }
    phy_release_rx_buf(phy);

    out:
        ret = status;
        //Stop transmitters/receivers
        if (rx_on){
            status = phy_stop_receiver(phy);
            if (status != 0){
                if (status == 1){
                    fprintf(stderr, "ERROR: RX overruns were detected on PHY receiver\n");
                    ret = status;
                }else{
                    fprintf(stderr, "Couldn't stop phy receiver\n");
                }
            }
        }
        DEBUG_MSG("\tClosing phy\n");
        phy_close(phy);
        DEBUG_MSG("\tClosing bladeRF\n");
        bladerf_close(dev);
        if (ret != 0){
            printf("\nTEST FAILED\n");
        }else{
            printf("\nTEST PASSED\n");
        }
        printf("------------ENDING PHY RECEIVE TEST---------------\n");
        return ret;
}

/**
 * FSK mod/demod test. 
 * Tests the case where part of a byte is received from first set of samples,
 * and the rest is received from the next set of samples
 */
int fsk_test1(void)
{
    //tx
    struct complex_sample *tx_samples = NULL;
    uint8_t                tx_data[] = "=Hello-there=";
    unsigned int           num_samples_tx;
    //rx
    unsigned int           num_bytes_rx1, num_bytes_rx2;
    struct fsk_handle     *fsk = NULL;
    unsigned int           num_samples_rx1;
    unsigned int           num_samples_rx2;
    int                    status = 0;
    uint8_t                rx_data[512];
    bool                   passed = 1;
    int                    num_samples_processed;

    printf("------------BEGINNING FSK TEST 1-------------\n");
    //Open fsk handle
    fsk = fsk_init(SAMP_PER_SYMB*SYMB_PER_REV);
    if (fsk == NULL){
        fprintf(stderr, "Couldn't initialize fsk\n");
        status = -1;
        goto out;
    }

    num_samples_tx = sizeof(tx_data)*8*SAMP_PER_SYMB+1;

    //Allocate tx samples
    tx_samples = (struct complex_sample *) malloc(num_samples_tx * sizeof(struct complex_sample));
    if (tx_samples == NULL){
        perror("malloc");
        status = -1;
        goto out;
    }
    //Initial sample
    tx_samples[0].i = 2047;
    tx_samples[0].q = 0;
    //Modulate
    fsk_mod(fsk, tx_data, sizeof(tx_data), SAMP_PER_SYMB, &tx_samples[1]);
    printf("Modulating: '%s'\n", tx_data);
    printf("%u tx samples\n", num_samples_tx);

    num_samples_rx1 = 304;
    num_samples_rx2 = num_samples_tx - num_samples_rx1;

    printf("1st demod will receive %u samples\n", num_samples_rx1);
    printf("2nd demod will receive %u samples\n", num_samples_rx2);

    //Demod part 1
    num_bytes_rx1 = fsk_demod(fsk, tx_samples, num_samples_rx1, true, -1, SAMP_PER_SYMB,
                              rx_data, &num_samples_processed);
    printf("Demodulated %u bytes\n", num_bytes_rx1);
    print_chars(rx_data, num_bytes_rx1);
    printf("   Num samples processed = %d\n", num_samples_processed);

    //Demod part 2
    num_bytes_rx2 = fsk_demod(fsk, &tx_samples[num_samples_rx1], num_samples_rx2, false,
                              sizeof(tx_data)-num_bytes_rx1, SAMP_PER_SYMB,
                              &rx_data[num_bytes_rx1], &num_samples_processed);
    printf("Demodulated %u bytes\n", num_bytes_rx2);
    print_chars(&rx_data[num_bytes_rx1], num_bytes_rx2);
    printf("   Num samples processed = %d\n", num_samples_processed);

    printf("Received %u bytes total: ", num_bytes_rx1+num_bytes_rx2);
    print_chars(rx_data, num_bytes_rx1+num_bytes_rx2);

    //Check
    if (strcmp((char *)rx_data, (char *)tx_data) != 0){
        fprintf(stderr, "   ERROR: RX data did not match TX data\n");
        passed = 0;
    }else{
        printf("   RX data matched TX data\n");
    }

    out:
        free(tx_samples);
        fsk_close(fsk);
        if (!passed || status != 0){
            status = -1;
            printf("\nTEST FAILED\n");
        }else{
            status = 0;
            printf("\nTEST PASSED\n");
        }
        printf("------------ENDING FSK TEST 1----------------\n");
        return status;
}

/**
 * Run all tests
 */
int main(int argc, char *argv[])
{
    char        *dev_id1;
    char        *dev_id2;
    bladerf_gain tx_gain;
    bladerf_gain rx_gain;
    int          status;
    bool         passed = 1;

    //Parse arguments
    if (argc < 5){
        fprintf(stderr, "Usage: %s [bladeRF device ID 1] [bladeRF device ID 2] [TX unified gain] [RX unified gain]\n"
                        "   Ex: %s *:serial=00 *:serial=8a 45 20\n",
                argv[0], argv[0]);
        return 0;
    }

    dev_id1 = argv[1];
    dev_id2 = argv[2];
    tx_gain = (bladerf_gain)atoi(argv[3]);
    rx_gain = (bladerf_gain)atoi(argv[4]);


    // FSK mod/demod
    status = fsk_test1();
    if (status != 0){
        passed = 0;
    }

    //PHY receive with device 1
    status = phy_receive_test(dev_id1);
    if (status != 0){
        passed = 0;
    }
    //PHY receive with device 2
    status = phy_receive_test(dev_id2);
    if (status != 0){
        passed = 0;
    }

    //PHY transmission from device 1 -> device 2
    status = phy_test(dev_id1, dev_id2, 904000000, 924000000, tx_gain, rx_gain);
    if (status != 0){
        passed = 0;
    }
    //PHY transmission from device 2 -> device 1
    status = phy_test(dev_id2, dev_id1, 904000000, 924000000, tx_gain, rx_gain);
    if (status != 0){
        passed = 0;
    }

    //LINK transmission w/ ACKs from device 1 -> device 2
    status = link_test(dev_id1, dev_id2, 904000000, 924000000, tx_gain, rx_gain);
    if (status != 0){
        passed = 0;
    }

    if (!passed){
        printf("\nTESTS FAILED\n");
    }else{
        printf("\nTESTS PASSED\n");
    }

    return 0;
}
