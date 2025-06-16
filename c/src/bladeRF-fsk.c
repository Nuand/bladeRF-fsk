/**
 * @file
 * @brief   Top-level program that enables data transfer between two devices, using
 *          link.c to send/receive data
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
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "thread.h"
#include <math.h>
#include <libbladeRF.h>

#include "link.h"
#include "config.h"

struct bladerf_fsk_handle {
    struct link_handle *link;
    unsigned int        payload_length; //payload length per link layer frame
    struct {
        FILE     *in;
        long int  filesize;
        THREAD    thread;
        bool      stop;
        bool      on;
    } tx;
    struct {
        FILE     *out;
        THREAD    thread;
        bool      stop;
        bool      on;
    } rx;
};

void *sender(void *arg);
void *receiver(void *arg);
struct bladerf_fsk_handle *start(struct config *config);
void stop(struct bladerf_fsk_handle *handle);

/**
 * Thread function which gets bytes from the user via either stdin or a file and sends
 * them with link_send_data(). Thread will return when it encounters EOF. Setting
 * handle->tx.stop to true will not necessarily stop the thread since it may be stuck
 * on fgets().
 *
 * @param[in]   arg     pointer to bladerf_fsk_handle
 */
void *sender(void *arg)
{
    int status;
    struct bladerf_fsk_handle *handle = (struct bladerf_fsk_handle *) arg;
    uint8_t *tx_data;   //data buffer
    char    *result;
    size_t   num_bytes;
    size_t   bytes_sent;
    size_t   tx_data_size;

    //--allocate memory for data buffer
    //For efficient file transmission, do a multiple of payload length, large enough
    //(~2000 bytes) for efficient reads when sending a file in many chunks
    tx_data_size = (size_t)ceil(2000.0/handle->payload_length) * handle->payload_length;
    tx_data      = malloc(tx_data_size);
    if (tx_data == NULL){
        perror("malloc");
        return NULL;
    }

    bytes_sent = 0;
    while(!handle->tx.stop){
        //Get input data
        if (handle->tx.in == stdin){
            result = fgets((char *) tx_data, tx_data_size, stdin);
            if (result == NULL){
                break;
            }
            num_bytes = strlen((char *) tx_data) + 1;   //+1 includes null terminator
        }else{
            //Print progress
            fprintf(stderr, "\rSent: %d%%   ",
                    (int)roundf((float)bytes_sent/handle->tx.filesize * 100));
            //Read next chunk from file
            num_bytes = fread(tx_data, 1, tx_data_size, handle->tx.in);
            if (num_bytes == 0){
                fprintf(stderr, "\n");
                break;    //EOF
            }

        }
        //Transmit the message
        status = link_send_data(handle->link, tx_data, (unsigned int)num_bytes);
        if (status == -2){
            fprintf(stderr, "ERROR: Transmission failed (no connection)\n");
            break;
        }else if (status != 0){
            fprintf(stderr, "ERROR: Transmission failed (unexpected error)\n");
            break;
        }
        bytes_sent += num_bytes;
    }

    if (tx_data != NULL){
        free(tx_data);
    }
    return NULL;
}

/**
 * Thread function which receives bytes with link_receive_data() and writes them
 * to the output FILE pointer. Thread will return within 0.25 seconds if
 * handle->rx_stop is true.
 *
 * @param[in]   arg     pointer to bladerf_fsk_handle
 */
void *receiver(void *arg)
{
    struct bladerf_fsk_handle *handle = (struct bladerf_fsk_handle *) arg;
    uint8_t *rx_data;
    int      bytes_received;
    size_t   nwritten;
    size_t   rx_data_size;
    int      max_timeouts;

    //--allocate memory for data buffer
    //Do a multiple of payload length, large enough (~2000 bytes) for efficient file writes
    rx_data_size = (size_t)ceil(2000.0/handle->payload_length) * handle->payload_length;
    rx_data      = malloc(rx_data_size);
    if (rx_data == NULL){
        perror("malloc");
        return NULL;
    }

    if (handle->rx.out == stdout){
        //Set up to wait forever then return immediately after receiving first data chunk,
        //without waiting longer for more potential data. This means there is no extra
        //latency and the terminal will be snappy with received text messages.
        max_timeouts = -1;
    }else{
        max_timeouts = 1;
    }

    while(!handle->rx.stop){
        //Receive data into buffer
        bytes_received = link_receive_data(handle->link, rx_data_size, max_timeouts, rx_data);
        if (bytes_received == 0){
            continue;
        }else if (bytes_received < 0){
            fprintf(stderr, "ERROR: Receive data failed (unexpected error)\n");
            continue;
        }
        //Write the received bytes
        nwritten = fwrite(rx_data, 1, bytes_received, handle->rx.out);
        if ((int)nwritten != bytes_received){
            fprintf(stderr, "ERROR: Couldn't write out received data: ");
            perror("");
        }
        if (handle->rx.out == stdout){
            fflush(stdout);
        }
    }

    if (rx_data != NULL){
        free(rx_data);
    }
    return NULL;
}

/*
 * Initializes the link and starts the sender/receiver threads
 * @param[in]   config      pointer to config struct specifying configuration info
 *
 * @return      pointer to bladerf_fsk_handle on success, NULL on failure
 */
struct bladerf_fsk_handle *start(struct config *config)
{
    int status;
    struct bladerf_fsk_handle *handle;

    //Check to see if FPGA is loaded
    status = bladerf_is_fpga_configured(config->bladerf_dev);
    if (status < 0){
        fprintf(stderr, "Couldn't query FPGA configuration: %s\n",
                bladerf_strerror(status));
        return NULL;
    }else if (status == 0){
        fprintf(stderr, "FPGA is not loaded on bladeRF device. "
                        "Load the FPGA or configure autoloading.\n");
        return NULL;
    }

    //Allocate memory for bladerf-fsk handle
    handle = calloc(1, sizeof(handle[0]));
    if (handle == NULL){
        perror("calloc");
        goto error;
    }

    handle->payload_length = config->payload_length;
    //Set input/output files
    handle->tx.in          = config->tx_input;
    handle->tx.filesize    = config->tx_filesize;
    handle->rx.out         = config->rx_output;

    //Init the link
    handle->link = link_init(config->bladerf_dev, &config->params, config->payload_length);
    if (handle->link == NULL){
        goto error;
    }

    //Start the receiver thread
    status = THREAD_CREATE(&(handle->rx.thread), receiver, handle);
    if (status != 0){
        fprintf(stderr, "Couldn't create rx thread: %s\n", strerror(status));
        goto error;
    }
    handle->rx.on = true;

    //Start the sender thread
    status = THREAD_CREATE(&(handle->tx.thread), sender, handle);
    if (status != 0){
        fprintf(stderr, "Couldn't create tx thread: %s\n", strerror(status));
        goto error;
    }
    handle->tx.on = true;

    return handle;

    error:
        stop(handle);
        config_deinit(config);
        return NULL;
}

/**
 * Stop tx/rx threads and close/free all resources
 *
 * @param[in]   handle      pointer to bladerf_fsk_handle to stop
 */
void stop(struct bladerf_fsk_handle *handle)
{
    int status;
    if (handle != NULL){
        //Stop tx thread if on
        if (handle->tx.on){
            handle->tx.stop = true;
            status = THREAD_JOIN(handle->tx.thread, NULL);
            if (status != 0){
                fprintf(stderr, "Error joining tx thread: %s\n", strerror(status));
            }
        }

        //Stop rx thread if on and close link
        if (handle->rx.on){
            handle->rx.stop = true;

            //Close the link
            //This stops potential indefinite waiting inside link.c() receiver functions
            //so that calls to link_receive_data() will return
            link_close(handle->link, NULL);

            status = THREAD_JOIN(handle->rx.thread, NULL);
            if (status != 0){
                fprintf(stderr, "Error joining rx thread: %s\n", strerror(status));
            }
        }else{
            link_close(handle->link, NULL);
        }

    }
    free(handle);
}

/**
 * Run bladeRF-fsk
 */
int main(int argc, char *argv[])
{
    struct config             *config = NULL;
    struct bladerf_fsk_handle *handle = NULL;
    int status;

    //parse arguments
    status = config_init_from_cmdline(argc, argv, &config);
    if (status < 0) {
        return status;
    } else if (status == 1) {
        printf(
            "Usage: %s <options>\n"
            "bladeRF-to-bladeRF text/file transfer program based on a custom FSK software modem\n\n"
            "Options:\n",
            argv[0]
        );
        config_print_options();
        printf(
            "\n"
            "Notes:\n"
            "  -The -d option takes a device specifier string. See the bladerf_open() documentation\n"
            "   for more information about the format of this string.\n"
            "  -Average post-filter SNR estimate prints out when program quits. Expect to experience\n"
            "   bit errors (causing CRC errors requiring retransmission of frames) at <14 dB SNR.\n"
            "\n"
            "Example: Text chat between two devices at 904MHz/924MHz with TX gain adjusted.\n"
            "  Device 1> bladeRF-fsk -d *:serial=4a -r 904M -t 924M --tx-gain 55\n"
            "  Device 2> bladeRF-fsk -d *:serial=f0 -r 924M -t 904M --tx-gain 55\n\n"
            "Example: File transfer between two devices at 2.43GHz/2.45GHz.\n"
            "  Receiver   > bladeRF-fsk -d *:serial=4a -r 2.43G -t 2.45G -o rx.jpg\n"
            "  Transmitter> bladeRF-fsk -d *:serial=f0 -r 2.45G -t 2.43G -i puppy.jpg\n\n"
        );
        return 0;
    } else if (status > 0) {
        fprintf(stderr, "Unexpected return value: %d\n", status);
        return status;
    }

    if (config->quiet == false){
        printf("=============== bladeRF-fsk ================\n");
    }
    //Initialize the bladeRF-fsk handle
    handle = start(config);
    if (handle == NULL){
        fprintf(stderr, "ERROR: Couldn't start bladeRF-fsk\n");
        return 1;
    }
    if (handle->tx.in == stdin && config->quiet == false){
        #if BLADERF_OS_WINDOWS
            printf("----  Press CTRL-Z then ENTER to quit   ----\n");
        #else
            printf("----        Press CTRL-D to quit        ----\n");
        #endif
        printf("\n");
    }

    //Wait for TX thread function to end - meaning tx file ended
    status = THREAD_JOIN(handle->tx.thread, NULL);
    if (status != 0){
        fprintf(stderr, "Error joining tx thread\n");
    }
    handle->tx.on = false;

    if (config->quiet == false){
        printf("\nQuitting...\n");
    }
    //Stop/cleanup everything else
    stop(handle);
    config_deinit(config);
    return 0;
}
