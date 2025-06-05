/**
 * @brief   Link layer code for modem
 *
 * This file interfaces with phy.c through its transmit_data_frames() and
 * receive_frames() functions.
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

#include "link.h"
#include "phy.h"
#include "crc32.h"
#include "utils.h"

#ifdef DEBUG_MODE
    #define DEBUG_MSG(...) fprintf(stderr, "[LINK] " __VA_ARGS__)
    #ifndef ENABLE_NOTES
        #define ENABLE_NOTES
    #endif
#else
    #define DEBUG_MSG(...)
#endif

#ifdef ENABLE_NOTES
    #define NOTE(...) fprintf(stderr, "[LINK] " __VA_ARGS__)
#else
    #define NOTE(...)
#endif

#define ERROR(...) fprintf(stderr, "[LINK] " __VA_ARGS__)

/********************************************
 *                                          *
 *  PRIVATE FUNCTIONS AND DATA STRUCTURES   *
 *                                          *
 ********************************************/

struct data_frame {
    //Total frame length = 1009 bytes (8072 bits)
    uint8_t  type;                  //0x00 = data frame, 0xFF = ack frame
    uint16_t seq_num;               //Sequence number
    uint16_t used_payload_length;   //Length of used payload data in bytes
    uint8_t *payload;               //payload data - dynamically allocated
    uint32_t crc32;                 //32-bit CRC
};

struct ack_frame {
    //Total frame length = 7 bytes (56 bits)
    uint8_t  type;                  //0x00 = data frame, 0xFF = ack frame
    uint16_t ack_num;               //Acknowledgement number
    uint32_t crc32;                 //32-bit CRC
};

struct tx {
    struct data_frame data_frame_buf;       //Input data frame buffer
    struct ack_frame  ack_frame_buf;        //Input ack frame buffer
    bool              stop;                 //Signal to stop tx thread
    bool              data_buf_filled;      //Is the data frame buffer filled
    THREAD            thread;               //Transmitter thread
    COND              data_buf_filled_cond; //condition for data_buf_filled
    MUTEX             data_buf_status_lock; //Mutex for data_buf_filled_cond
    bool              link_on;              //Is the transmitter on
    bool              done;                 //Has the transmitter finished a transmission
    bool              success;              //Was the transmitter transmission successful
};

struct rx {
    struct data_frame data_frame_buf;       //Output data frame buffer
    struct ack_frame  ack_frame_buf;        //Output ack frame buffer
    uint8_t          *extra_bytes;          //Leftover bytes received but not returned to
                                            //the user after a call to link_receive_data()
    unsigned int      num_extra_bytes;      //Number of bytes in 'extra_bytes' buffer
    THREAD            thread;               //Receiver thread
    bool              stop;                 //Signal to stop rx thread
    bool              data_buf_filled;      //Is the data frame buffer filled
    COND              data_buf_filled_cond; //condition for data_buf_filled
    MUTEX             data_buf_status_lock; //mutex for data_buf_filled_cond
    bool              ack_buf_filled;       //Is the ack frame buffer filled
    COND              ack_buf_filled_cond;  //condition for ack_buf_filled
    MUTEX             ack_buf_status_lock;  //mutex for ack_buf_filled_cond
    bool              link_on;              //Is the receiver on
};

struct link_handle {
    struct phy_handle *phy;
    struct tx         *tx;
    struct rx         *rx;
    bool               phy_tx_on;       //Is the phy transmitter on
    bool               phy_rx_on;       //Is the phy receiver on
    unsigned int       payload_length;  //link frame payload length
    unsigned int       frame_length;    //full link frame length including header/footer
};

//internal functions
//tx:
static int start_transmitter(struct link_handle *link);
static int stop_transmitter(struct link_handle *link);
void *transmit_data_frames(void *arg);
static int send_payload(struct link_handle *link, uint8_t *payload,
                        uint16_t used_payload_length);
//rx:
static int start_receiver(struct link_handle *link);
static int stop_receiver(struct link_handle *link);
void *receive_frames(void *arg);
static int receive_ack(struct link_handle *link, uint16_t ack_num,
                       unsigned int timeout_ms);
static int receive_payload(struct link_handle *link, uint8_t *payload, int timeout_ms);
//utility:
static void convert_data_frame_struct_to_buf(struct data_frame *frame, uint8_t *buf, unsigned int payload_length);
static void convert_ack_frame_struct_to_buf(struct ack_frame *frame, uint8_t *buf);
static void convert_buf_to_data_frame_struct(uint8_t *buf, struct data_frame *frame, unsigned int payload_length);
static void convert_buf_to_ack_frame_struct(uint8_t *buf, struct ack_frame *frame);

/****************************************
 *                                      *
 *  INIT/DEINIT AND UTILITY FUNCTIONS   *
 *                                      *
 ****************************************/

struct link_handle *link_init(struct bladerf *dev, struct radio_params *params, unsigned int payload_length)
{
    int                 status;
    struct link_handle *link;

    DEBUG_MSG("Initializing\n");
    //-------------Allocate memory for link handle struct--------------
    //Calloc so all pointers are initialized to NULL, and all bools are initialized false
    link = calloc(1, sizeof(struct link_handle));
    if (link == NULL){
        perror("malloc");
        return NULL;
    }

    link->payload_length = payload_length;
    //Compute full link frame length
    link->frame_length = sizeof(((struct data_frame *)0)->type) +
                         sizeof(((struct data_frame *)0)->seq_num) +
                         sizeof(((struct data_frame *)0)->used_payload_length) +
                         payload_length +
                         sizeof(((struct data_frame *)0)->crc32);

    //---------------Open/Initialize phy handle--------------------------
    link->phy = phy_init(dev, params, link->frame_length);
    if (link->phy == NULL){
        ERROR("Couldn't initialize phy handle\n");
        goto error;
    }
    //Start phy receiver
    status = phy_start_receiver(link->phy, 0);
    if (status != 0){
        ERROR("Couldn't start phy recevier\n");
        goto error;
    }
    link->phy_rx_on = true;
    //Start phy transmitter
    status = phy_start_transmitter(link->phy);
    if (status != 0){
        ERROR("Couldn't start phy transmitter\n");
        goto error;
    }
    link->phy_tx_on = true;

    //------------------Allocate memory for tx struct and initialize-----
    link->tx = malloc(sizeof(struct tx));
    if (link->tx == NULL){
        perror("malloc");
        goto error;
    }
    //Allocate memory for tx data frame buffer payload
    link->tx->data_frame_buf.payload = malloc(payload_length);
    if (link->tx->data_frame_buf.payload == NULL){
        perror("malloc");
        goto error;
    }

    //Initialize control/state variables
    link->tx->stop            = false;
    link->tx->data_buf_filled = false;
    link->tx->done            = false;
    link->tx->success         = false;
    link->tx->link_on         = false;
    //Initialize pthread condition variable
    status = COND_INIT(&(link->tx->data_buf_filled_cond));
    if (status != 0){
        ERROR("Error initializing pthread_cond: %s\n", strerror(status));
        goto error;
    }
    //Initialize pthread mutex variable
    status = MUTEX_INIT(&(link->tx->data_buf_status_lock));
    if (status != 0){
        ERROR("Error initializing pthread_mutex: %s\n", strerror(status));
        goto error;
    }
    //------------------Allocate memory for rx struct and initialize-----
    link->rx = malloc(sizeof(struct rx));
    if (link->rx == NULL){
        perror("malloc");
        goto error;
    }
    //Allocate memory for rx data frame buffer payload
    link->rx->data_frame_buf.payload = malloc(payload_length);
    if (link->rx->data_frame_buf.payload == NULL){
        perror("malloc");
        goto error;
    }
    // Allocate memory for extra_bytes buffer
    link->rx->extra_bytes = malloc(payload_length);
    if (link->rx->extra_bytes == NULL){
        perror("malloc");
        goto error;
    }
    //Initialize pthread condition variable for data
    status = COND_INIT(&(link->rx->data_buf_filled_cond));
    if (status != 0){
        ERROR("Error initializing pthread_cond: %s\n", strerror(status));
        goto error;
    }
    //Initialize pthread mutex variable for data
    status = MUTEX_INIT(&(link->rx->data_buf_status_lock));
    if (status != 0){
        ERROR("Error initializing pthread_mutex: %s\n", strerror(status));
        goto error;
    }
    //Initialize pthread condition variable for ack
    status = COND_INIT(&(link->rx->ack_buf_filled_cond));
    if (status != 0){
        ERROR("Error initializing pthread_cond: %s\n", strerror(status));
        goto error;
    }
    //Initialize pthread mutex variable for ack
    status = MUTEX_INIT(&(link->rx->ack_buf_status_lock));
    if (status != 0){
        ERROR("Error initializing pthread_mutex: %s\n", strerror(status));
        goto error;
    }
    //Initialize control/state variables
    link->rx->stop            = false;
    link->rx->data_buf_filled = false;
    link->rx->ack_buf_filled  = false;
    link->rx->num_extra_bytes = 0;
    link->rx->link_on         = false;

    //---------------------Start the link receiver--------------------------
    status = start_receiver(link);
    if (status != 0){
        ERROR("Couldn't start receiver\n");
        goto error;
    }
    link->rx->link_on = true;
    //-------------------Start the link transmitter-------------------------
    status = start_transmitter(link);
    if (status != 0){
        ERROR("Couldn't start receiver\n");
        goto error;
    }
    link->tx->link_on = true;

    DEBUG_MSG("Initialization done\n");
    return link;

    error:
        link_close(link);
        return NULL;
}

void link_close(struct link_handle *link)
{
    int status;

    DEBUG_MSG("Closing\n");

    //Cleanup all internal resources
    if (link != NULL){
        //Cleanup tx struct
        if (link->tx != NULL){
            //Stop the link transmitter if it is on
            if (link->tx->link_on){
                status = stop_transmitter(link);
                if (status != 0){
                    ERROR("TX: Error stopping link transmitter\n");
                }
            }
            status = MUTEX_DESTROY(&(link->tx->data_buf_status_lock));
            if (status != 0){
                ERROR("TX: Error destroying data buf status pthread_mutex\n");
            }
            status = COND_DESTROY(&(link->tx->data_buf_filled_cond));
            if (status != 0){
                ERROR("TX: Error destroying data buf status pthread_cond\n");
            }
            if (link->tx->data_frame_buf.payload != NULL){
                free(link->tx->data_frame_buf.payload);
            }
        }
        free(link->tx);
        //Cleanup rx struct
        if (link->rx != NULL){
            //Stop the link receiver if it is on
            if (link->rx->link_on){
                status = stop_receiver(link);
                if (status != 0){
                    ERROR("RX: Error stopping link receiver\n");
                }
            }
            //sleep to prevent race condition where we potentially destroy a mutex that
            //was locked by receive_payload()
            usleep(300);
            status = MUTEX_DESTROY(&(link->rx->data_buf_status_lock));
            if (status != 0){
                ERROR("RX: Error destroying data buf status pthread_mutex\n");
            }
            status = COND_DESTROY(&(link->rx->data_buf_filled_cond));
            if (status != 0){
                ERROR("RX: Error destroying data buf status pthread_cond\n");
            }
            status = MUTEX_DESTROY(&(link->rx->ack_buf_status_lock));
            if (status != 0){
                ERROR("RX: Error destroying ack buf status pthread_mutex\n");
            }
            status = COND_DESTROY(&(link->rx->ack_buf_filled_cond));
            if (status != 0){
                ERROR("RX: Error destroying ack buf status pthread_cond\n");
            }
            if (link->rx->data_frame_buf.payload != NULL){
                free(link->rx->data_frame_buf.payload);
            }
            if (link->rx->extra_bytes != NULL){
                free(link->rx->extra_bytes);
            }
        }
        free(link->rx);
        //Close the phy
        if (link->phy != NULL){
            //Stop phy transmitter/receiver if they are on
            if (link->phy_tx_on){
                status = phy_stop_transmitter(link->phy);
                if (status != 0){
                    ERROR("TX: Error stopping phy transmitter\n");
                }
            }
            if (link->phy_rx_on){
                status = phy_stop_receiver(link->phy);
                if (status != 0){
                    if (status == 1){
                        ERROR("Warning: RX overruns were detected\n");
                    }else{
                        ERROR("RX: Error stopping phy receiver\n");
                    }
                }
            }
        }
        //Close phy handle
        phy_close(link->phy);
    }
    free(link);
    link = NULL;
}

/**
 * Convert data frame struct to a buffer of uint8_t
 * @param[in]   frame   pointer to data_frame structure to convert
 * @param[out]  buf     pointer to buffer to place the frame in
 */
static void convert_data_frame_struct_to_buf(struct data_frame *frame, uint8_t *buf, unsigned int payload_length)
{
    int i = 0;

    //Frame type
    memcpy(&buf[i], &(frame->type), sizeof(frame->type));
    i += sizeof(frame->type);
    //Seq num
    memcpy(&buf[i], &(frame->seq_num), sizeof(frame->seq_num));
    i += sizeof(frame->seq_num);
    //payload length
    memcpy(&buf[i], &(frame->used_payload_length), sizeof(frame->used_payload_length));
    i += sizeof(frame->used_payload_length);
    //payload
    memcpy(&buf[i], frame->payload, payload_length);
    i += payload_length;
    //crc
    memcpy(&buf[i], &(frame->crc32), sizeof(frame->crc32));
    i += sizeof(frame->crc32);
}

/**
 * Convert ack frame struct to a buffer of uint8_t
 * @param[in]   frame   pointer to ack_frame structure to convert
 * @param[out]  buf     pointer to buffer to place the frame in
 */
static void convert_ack_frame_struct_to_buf(struct ack_frame *frame, uint8_t *buf)
{
    int i = 0;

    //Frame type
    memcpy(&buf[i], &(frame->type), sizeof(frame->type));
    i += sizeof(frame->type);
    //ack num
    memcpy(&buf[i], &(frame->ack_num), sizeof(frame->ack_num));
    i += sizeof(frame->ack_num);
    //crc
    memcpy(&buf[i], &(frame->crc32), sizeof(frame->crc32));
    i += sizeof(frame->crc32);

    if (i != ACK_FRAME_LENGTH){
        ERROR("%s: ERROR: Actual ACK frame struct length (%d) does not match "
              "defined ACK_FRAME_LENGTH (%d). Fix these in link.c/link.h so they match\n",
              __FUNCTION__, i, ACK_FRAME_LENGTH);
    }
}

/**
 * Convert uint8_t buffer to data frame struct
 *
 * @param[in]   buf             pointer to buffer holding a data frame
 * @param[out]  frame           pointer to data_frame struct to place frame in
 * @param[in]   payload_length  length of payload in bytes
 */
static void convert_buf_to_data_frame_struct(uint8_t *buf, struct data_frame *frame, unsigned int payload_length)
{
    int i = 0;

    //Frame type
    memcpy(&(frame->type), &buf[i], sizeof(frame->type));
    i += sizeof(frame->type);
    //Seq num
    memcpy(&(frame->seq_num), &buf[i], sizeof(frame->seq_num));
    i += sizeof(frame->seq_num);
    //payload length
    memcpy(&(frame->used_payload_length), &buf[i], sizeof(frame->used_payload_length));
    i += sizeof(frame->used_payload_length);
    //payload
    memcpy(frame->payload, &buf[i], payload_length);
    i += payload_length;
    //crc
    memcpy(&(frame->crc32), &buf[i], sizeof(frame->crc32));
    i += sizeof(frame->crc32);
}

/**
 * Convert uint8_t buffer to ack frame struct
 *
 * @param[in]   buf     pointer to buffer holding a ack frame
 * @param[out]  frame   pointer to ack_frame struct to place frame in
 */
static void convert_buf_to_ack_frame_struct(uint8_t *buf, struct ack_frame *frame)
{
    int i = 0;

    //Frame type
    memcpy(&(frame->type), &buf[i], sizeof(frame->type));
    i += sizeof(frame->type);
    //ack num
    memcpy(&(frame->ack_num), &buf[i], sizeof(frame->ack_num));
    i += sizeof(frame->ack_num);
    //crc
    memcpy(&(frame->crc32), &buf[i], sizeof(frame->crc32));
    i += sizeof(frame->crc32);

    if (i != ACK_FRAME_LENGTH){
        ERROR("%s: ERROR: Actual ACK frame struct length (%d) does not match "
              "defined ACK_FRAME_LENGTH (%d). Fix these in link.c/link.h so they match\n",
              __FUNCTION__, i, ACK_FRAME_LENGTH);
    }
}

/****************************************
 *                                      *
 *          TRANSMITTER FUNCTIONS       *
 *                                      *
 ****************************************/

/**
 * Start the transmitter thread
 *
 * @param[in]   link    pointer to link handle
 *
 * @return  0 on success, -1 on failure
 */
static int start_transmitter(struct link_handle *link)
{
    int status;

    link->tx->done    = false;
    link->tx->success = false;
    //be sure stop signal is off
    link->tx->stop    = false;
    //Kick off transmitter thread
    status = THREAD_CREATE(&(link->tx->thread), transmit_data_frames, link);
    if (status != 0){
        ERROR("Error creating tx thread: %s\n", strerror(status));
        return -1;
    }
    return 0;
}

/**
 * Stop the transmitter thread
 *
 * @param[in]   link    pointer to link handle
 *
 * @return  0 on success, -1 on failure
 */
static int stop_transmitter(struct link_handle *link)
{
    int status;

    DEBUG_MSG("TX: Stopping transmitter...\n");
    //signal stop
    link->tx->stop = true;
    //Signal the buffer filled condition so the thread will stop waiting
    //for a filled buffer
    status = MUTEX_LOCK(&(link->tx->data_buf_status_lock));
    if (status != 0){
        ERROR("Error locking pthread_mutex\n");
    }
    status = COND_SIGNAL(&(link->tx->data_buf_filled_cond));
    if (status != 0){
        ERROR("Error signaling pthread_cond\n");
    }
    status = MUTEX_UNLOCK(&(link->tx->data_buf_status_lock));
    if (status != 0){
        ERROR("Error unlocking pthread_mutex\n");
    }
    //Wait for tx thread to finish
    status = THREAD_JOIN(link->tx->thread, NULL);
    if (status != 0){
        ERROR("Error joining tx thread: %s\n", strerror(status));
        return -1;
    }
    DEBUG_MSG("TX: Transmitter stopped\n");
    return 0;
}

int link_send_data(struct link_handle *link, uint8_t *data, unsigned int data_length)
{
    unsigned int num_full_payloads;
    unsigned int i;
    unsigned int last_payload_length;
    int status;

    num_full_payloads = data_length/link->payload_length;

    //Loop through each full payload
    for(i = 0; i < num_full_payloads; i++){
        //Send the frame
        status = send_payload(link, &data[i*link->payload_length], link->payload_length);
        if (status != 0){
            if (status == -2){
                DEBUG_MSG("TX: Send data failed: "
                          "No response for payload #%d\n", i+1);
            }else{
                ERROR("TX: Send data failed: "
                      "Unexpected error sending payload #%d\n", i+1);
            }
            return status;
        }
    }

    //Send the last payload for the remaining bytes
    last_payload_length = data_length % link->payload_length;
    if (last_payload_length != 0){
        status = send_payload(link, &data[i*link->payload_length],
                                (uint16_t) last_payload_length);
        if (status != 0){
            if (status == -2){
                DEBUG_MSG("TX: Send data failed: "
                          "No response for payload #%d\n", i+1);
            }else{
                ERROR("TX: Send data failed: "
                      "Unexpected error sending payload #%d\n", i+1);
            }
            return status;
        }
    }
    return 0;
}

/**
 * Sends a payload. Blocks until either the transmission was successful (with an
 * acknowledgement) or there was no response (exceeded max number of retransmissions)
 *
 * @param[in]   link                    pointer to link handle
 * @param[in]   payload                 buffer of bytes to send
 * @param[in]   used_payload_length     number of bytes to send in 'payload'. If less
 *                                      than link->payload_length, zeros will be padded.
 * @return      0 on success, -1 on error, -2 on timeout/no response
 */
static int send_payload(struct link_handle *link, uint8_t *payload,
                    uint16_t used_payload_length)
{
    int status;

    if (used_payload_length > link->payload_length){
        ERROR("%s: Invalid payload length of %hu, must be <=%u\n", __FUNCTION__,
              used_payload_length, link->payload_length);
        return -1;
    }

    //Wait for tx buf to be empty
    while(link->tx->data_buf_filled){
        usleep(50);
    }
    //Copy payload data into frame buffer
    memcpy(link->tx->data_frame_buf.payload, payload, used_payload_length);
    //Pad zeros to unused portion of the payload
    memset(&(link->tx->data_frame_buf.payload[used_payload_length]), 0,
            link->payload_length - used_payload_length);
    //Set payload length
    link->tx->data_frame_buf.used_payload_length = used_payload_length;
    //Mark tx not done
    link->tx->done = false;
    //Mark buffer filled
    link->tx->data_buf_filled = true;
    //Signal the buffer filled condition
    status = MUTEX_LOCK(&(link->tx->data_buf_status_lock));
    if (status != 0){
        ERROR("Error locking pthread_mutex: %s\n", strerror(status));
        return -1;
    }
    status = COND_SIGNAL(&(link->tx->data_buf_filled_cond));
    if (status != 0){
        ERROR("Error signaling pthread_cond: %s\n", strerror(status));
        return -1;
    }
    status = MUTEX_UNLOCK(&(link->tx->data_buf_status_lock));
    if (status != 0){
        ERROR("Error unlocking pthread_mutex: %s\n", strerror(status));
        return -1;
    }
    //Wait for the transmission to complete
    //TODO: Change this to a pthread condition signal
    while (!link->tx->done){
        usleep(100);
    }
    //Check the status of the transmission
    if (!link->tx->success){
        return -2;
    }

    return 0;
}

/**
 * Thread function that transmits data frames and waits for acks.
 * Does not directly receive acks - the receive_frames() function does this.
 * Does not transmit acks - the receive_frames function does this.
 *
 * @param[in]   arg     pointer to link handle
 */
void *transmit_data_frames(void *arg)
{
    int          status;
    uint16_t     seq_num;
    uint32_t     crc_32;
    unsigned int tries;
    uint8_t     *data_send_buf;
    bool         failed;

    //cast arg
    struct link_handle *link = (struct link_handle *) arg;
    //allocate data send buf
    data_send_buf = malloc(link->frame_length);
    if (data_send_buf == NULL){
        perror("malloc");
        goto out;
    }
    //Set initial sequence number to random value
    srand((unsigned int)time(NULL));
    seq_num = rand() % 65536;

    DEBUG_MSG("TX: Initial seq num = %hu\n", seq_num);
    tries = 1;

    while (!link->tx->stop){
        if (tries > LINK_MAX_TRIES){
            DEBUG_MSG("TX: Exceeded max tries (%u) without an ACK."
                      " Skipping frame\n", tries-1);
            link->tx->success = false;
            link->tx->done    = true;
            tries             = 1;
        }
        if (tries == 1){
            //---------Wait for data buffer to be filled-----------
            //Lock mutex
            status = MUTEX_LOCK(&(link->tx->data_buf_status_lock));
            if (status != 0){
                ERROR("Mutex lock failed: %s\n", strerror(status));
                goto out;
            }
            //Wait for condition signal - meaning buffer is full
            failed = false;
            while (!link->tx->data_buf_filled && !link->tx->stop){
                DEBUG_MSG("TX: Waiting for buffer to be filled\n");
                status = COND_WAIT(&(link->tx->data_buf_filled_cond),
                                            &(link->tx->data_buf_status_lock));
                if (status != 0){
                    ERROR("%s(): Condition wait failed: %s\n", __FUNCTION__,
                          strerror(status));
                    failed = true;
                    break;
                }
            }
            //Unlock mutex
            status = MUTEX_UNLOCK(&(link->tx->data_buf_status_lock));
            if (status != 0){
                ERROR("%s(): Mutex unlock failed: %s\n", __FUNCTION__, strerror(status));
                failed = true;
            }
            //Stop thread if stop variable is true, or something with pthreads went wrong
            if (link->tx->stop || failed){
                link->tx->data_buf_filled = false;
                goto out;
            }
            DEBUG_MSG("TX: Frame buffer filled. Sending...\n");
            //Set frame type
            link->tx->data_frame_buf.type = DATA_FRAME_CODE;
            //Set sequence number
            link->tx->data_frame_buf.seq_num = seq_num;
            //Copy frame into send buf
            convert_data_frame_struct_to_buf(&(link->tx->data_frame_buf), data_send_buf,
                                             link->payload_length);
            //Calculate the CRC
            crc_32 = crc32(data_send_buf, link->frame_length - sizeof(crc_32));
            //Copy this CRC to the send buf
            memcpy(&data_send_buf[link->frame_length - sizeof(crc_32)],
                    &crc_32, sizeof(crc_32));
            //Mark tx data buffer empty
            link->tx->data_buf_filled = false;
        }
        //Transmit the frame
        status = phy_fill_tx_buf(link->phy, data_send_buf, link->frame_length);
        if (status != 0){
            ERROR("TX: Couldn't fill phy tx buffer\n");
            goto out;
        }
        DEBUG_MSG("TX: Frame sent to PHY. Waiting for ACK...\n");
        //Wait for an ack
        status = receive_ack(link, seq_num, ACK_TIMEOUT_MS);
        if (status == -2){
            DEBUG_MSG("TX: Didn't get an ACK (timed out). Resending\n");
            tries++;
            continue;
        }else if (status == -3){
            DEBUG_MSG("TX: Received wrong ACK number. Resending\n");
            tries++;
            continue;
        }else if (status < 0){
            ERROR("TX: Error receiving ACK. Stopping transmitter\n");
            goto out;
        }
        //Success!
        DEBUG_MSG("TX: Got an ACK (ack# = %hu)\n", seq_num);
        link->tx->success = true;
        link->tx->done    = true;
        tries             = 1;
        seq_num++;
    }

    out:
        if (data_send_buf != NULL){
            free(data_send_buf);
        }
        link->tx->done = true;
        return NULL;
}

/****************************************
 *                                      *
 *          RECEIVER FUNCTIONS          *
 *                                      *
 ****************************************/

/**
 * Start the receiver thread
 *
 * @param[in]   link    pointer to link handle
 *
 * @return  0 on success, -1 on failure
 */
static int start_receiver(struct link_handle *link)
{
    int status;

    //be sure stop signal is off
    link->rx->stop = false;
    //Kick off receiver thread
    status = THREAD_CREATE(&(link->rx->thread), receive_frames, link);
    if (status != 0){
        ERROR("Error creating rx thread: %s\n", strerror(status));
        return -1;
    }
    return 0;
}

/**
 * Stop the receiver thread
 *
 * @param[in]   link    pointer to link handle
 *
 * @return  0 on success, -1 on failure
 */
static int stop_receiver(struct link_handle *link)
{
    int status;

    DEBUG_MSG("RX: Stopping receiver...\n");
    //signal stop
    link->rx->stop = true;

    //Wait for rx thread to finish
    status = THREAD_JOIN(link->rx->thread, NULL);
    if (status != 0){
        ERROR("Error joining rx thread: %s\n", strerror(status));
        return -1;
    }
    DEBUG_MSG("RX: Receiver stopped\n");
    return 0;
}

int link_receive_data(struct link_handle *link, int size, int max_timeouts,
                      uint8_t *data_buf)
{
    int      bytes_received;
    int      bytes_remaining;
    uint8_t *temp_buf = NULL;
    int      i = 0;
    int      timeouts;
    int      timeout_ms;

    //Check for invalid input
    if (size < 0){
        ERROR("RX: %s(): parameter is negative\n", __FUNCTION__);
        goto error;
    }

    if (max_timeouts<0){
        //Special case max_timeouts=-1: wait forever and return immediately after
        //receiving 1st payload or 1st piece of leftover data from a previous call
        timeout_ms = -1;
    }else{
        timeout_ms = 500;
    }

    //allocate temporary buffer for last payload
    temp_buf = malloc(link->payload_length);
    if (temp_buf == NULL){
        perror("malloc");
        goto error;
    }

    timeouts = 0;
    //Read any extra bytes left over from the last call to this function
    if (size < (int)(link->rx->num_extra_bytes)){
        //Special case: More extra bytes than the request size.
        memcpy(data_buf, link->rx->extra_bytes, size);
        //Move over the remaining extra_bytes
        memmove(link->rx->extra_bytes, &(link->rx->extra_bytes[size]),
                link->rx->num_extra_bytes - size);
        i = size;
        link->rx->num_extra_bytes -= size;
        goto out;
    }else{
        if (link->rx->num_extra_bytes > 0){
            memcpy(data_buf, link->rx->extra_bytes, link->rx->num_extra_bytes);
            i = link->rx->num_extra_bytes;
            link->rx->num_extra_bytes = 0;
            if (max_timeouts<0){
                //return immediately with leftover data
                goto out;
            }
        }
    }

    while(i < size && (timeouts < max_timeouts || max_timeouts < 0)){
        //We have additional bytes to receive before timeouts have been exhausted
        bytes_remaining = size - i;
        //Make sure to not write more than 'size' bytes. For the last payload(s),
        //Use a temp buffer first
        if (bytes_remaining < (int)link->payload_length){
            bytes_received = receive_payload(link, temp_buf, timeout_ms);
            if (bytes_received == -2){
                timeouts++;     //timeout
                continue;
            }else if (bytes_received == -3){
                goto out;       //receiver shut down
            }else if (bytes_received < 0){
                ERROR("RX: Error receiving payload\n");
                goto error;
            }else if (bytes_received > 0){
                //Copy bytes from temp buf to data_buf
                if (bytes_received < bytes_remaining){
                    memcpy(&data_buf[i], temp_buf, bytes_received);
                    i += bytes_received;
                }else{    //bytes_received >= bytes_remaining
                    memcpy(&data_buf[i], temp_buf, bytes_remaining);
                    i += bytes_remaining;
                    //Copy extra bytes to extra_bytes buffer
                    link->rx->num_extra_bytes = bytes_received - bytes_remaining;
                    memcpy(link->rx->extra_bytes, &temp_buf[bytes_remaining],
                           link->rx->num_extra_bytes);
                }
                if (max_timeouts<0){
                    //return immediately after first payload
                    break;
                }
            }
        //For all other payloads
        }else{
            bytes_received = receive_payload(link, &data_buf[i], timeout_ms);
            //Check for timeout
            if (bytes_received == -2){
                timeouts++;     //timeout
                continue;
            }else if (bytes_received == -3){
                goto out;       //receiver shut down
            }else if (bytes_received < 0){
                ERROR("RX: Error receiving payload\n");
                goto error;
            }else if (bytes_received > 0){
                i += bytes_received;
                if (max_timeouts<0){
                    //return immediately with first payload
                    break;
                }
            }
        }
    }

    out:
        if (temp_buf != NULL){
            free(temp_buf);
        }
        return i;

    error:
        if (temp_buf != NULL){
            free(temp_buf);
        }
        return -1;
}

/**
 * Receives a payload and copies it into the given buffer
 * @param[in]   link            pointer to link handle
 * @param[in]   timeout_ms      Amount of time to wait for a received payload
 *                              Use -1 to wait forever
 * @param[out]  payload         pointer to buffer to place payload in
 *
 * @return      number of bytes received (0 - link->payload_length), -1 on error,
 *              -2 on timeout, -3 if receiver was shut down
 */
static int receive_payload(struct link_handle *link, uint8_t *payload, int timeout_ms)
{
    int used_payload_length = 0;
    struct timespec timeout_abs;
    int status;

    if (link->rx->stop){
        return -3;  //receiver is shut down
    }

    if (timeout_ms >= 0){
        //Create absolute time format timeout
        status = create_timeout_abs(timeout_ms, &timeout_abs);
        if (status != 0){
            ERROR("RX: %s(): Error creating timeout\n", __FUNCTION__);
            return -1;
        }
    }

    //Prepare to wait with COND_TIMED_WAIT()
    status = MUTEX_LOCK(&(link->rx->data_buf_status_lock));
    if (status != 0){
        ERROR("RX: %s(): Error locking mutex: %s\n", __FUNCTION__, strerror(status));
        return -1;
    }
    //Wait for condition signal - meaning rx data buffer is full
    while (!link->rx->data_buf_filled){
        if (link->rx->stop){
            used_payload_length = -3;   //receiver shut down
            break;
        }

        if (timeout_ms >= 0){
            //Wait for timeout
            status = COND_TIMED_WAIT(&(link->rx->data_buf_filled_cond),
                                            &(link->rx->data_buf_status_lock), timeout_ms);
            if (status != 0){
                if (status == ETIMEDOUT){
                    used_payload_length = -2;
                }else{
                    ERROR("RX: %s(): Condition wait failed: %s\n", __FUNCTION__,
                          strerror(status));
                    used_payload_length = -1;
                }
                break;
            }
        }else{
            //Wait forever (or until receiver signals condition during shutdown)
            status = COND_WAIT(&(link->rx->data_buf_filled_cond),
                                       &(link->rx->data_buf_status_lock));
            if (status != 0) {
                ERROR("RX: %s(): Condition wait failed: %s\n", __FUNCTION__,
                      strerror(status));
                used_payload_length = -1;
                break;
            }
        }
    }

    //Waiting is done
    //Did we error, timeout, or was receiver stopped? Return
    if (used_payload_length < 0){
        goto out;
    }

    //Get the length of the used portion of the payload
    used_payload_length = link->rx->data_frame_buf.used_payload_length;
    //Copy the used portion of the payload
    memcpy(payload, link->rx->data_frame_buf.payload, used_payload_length);
    //Mark the rx data buffer empty
    link->rx->data_buf_filled = false;

    out:
        //Unlock mutex on link->rx->data_buf_filled
        status = MUTEX_UNLOCK(&(link->rx->data_buf_status_lock));
        if (status != 0){
            ERROR("RX: %s(): Mutex unlock failed: %s\n", __FUNCTION__, strerror(status));
            used_payload_length = -1;
        }

        return used_payload_length;
}

/**
 * Attempts to receive an ACK with the given ACK number
 * Waits until it either receives an ack or times out.
 *
 * @param[in]   link        pointer to link handle
 * @param[in]   ack_num     acknowledgement number to look for
 * @param[in]   timeout_ms  amount of time to wait for an ACK, in milliseconds
 *
 * @return      0 if successfully received ack, -1 if pthread error or wrong ack number,
 *              -2 if timed out, -3 if wrong ack number
 */
static int receive_ack(struct link_handle *link, uint16_t ack_num, unsigned int timeout_ms)
{
    struct timespec timeout_abs;
    int status, ret = 0;

    //Create absolute time format timeout
    status = create_timeout_abs(timeout_ms, &timeout_abs);
    if (status != 0){
        ERROR("RX: %s(): Error creating timeout\n", __FUNCTION__);
        return -1;
    }

    //Prepare to wait with COND_TIMED_WAIT()
    status = MUTEX_LOCK(&(link->rx->ack_buf_status_lock));
    if (status != 0){
        ERROR("RX: %s(): Error locking mutex: %s\n", __FUNCTION__, strerror(status));
        return -1;
    }
    //Wait for condition signal - meaning buffer is full
    while (!link->rx->ack_buf_filled){
        status = COND_TIMED_WAIT(&(link->rx->ack_buf_filled_cond),
                                        &(link->rx->ack_buf_status_lock), timeout_ms);
        if (status != 0){
            if (status == ETIMEDOUT){
                ret = -2;
            }else{
                ERROR("RX: %s(): Condition wait failed: %s\n", __FUNCTION__, strerror(status));
                ret = -1;
            }
            break;
        }
    }

    //Check the sequence number if nothing failed
    if (ret == 0 && link->rx->ack_frame_buf.ack_num != ack_num){
        ERROR("RX: %s(): Incorrect ack number %hu (expected %hu)\n", __FUNCTION__,
              link->rx->ack_frame_buf.ack_num, ack_num);
        ret = -3;
    }
    //mark the rx ack buffer empty
    link->rx->ack_buf_filled = false;

    //unlock mutex after modifying link->rx->ack_buf_filled
    status = MUTEX_UNLOCK(&(link->rx->ack_buf_status_lock));
    if (status != 0){
        ERROR("RX: %s(): Mutex unlock failed: %s\n", __FUNCTION__, strerror(status));
        ret = -1;
    }

    return ret;
}

/**
 * Thread function which receives data and ACK frames from the PHY, and transmits ACKs.
 * Checks CRC on all received frames before copying them to a buffer which other
 * functions can use. If the CRC is incorrect, it disregards the frame and does not
 * copy it. If a data frame is received, it checks sequence number for duplicate.
 * If not a duplicate, it copies the frame to rx data_frame_buf and marks the buffer
 * full. An acknowledgement is always sent, regardless of whether or not the received
 * data frame is a duplicate. This is the only function that sends acks.
 *
 * @param[in]   arg     pointer to link handle
 */
void *receive_frames(void *arg)
{
    uint8_t     *rx_buf = NULL;
    unsigned int frame_length;
    uint32_t     crc_32, crc_32_rx;
    bool         is_data_frame = false;
    int          status;
    uint8_t      ack_send_buf[ACK_FRAME_LENGTH];
    uint16_t     seq_num = 0;
    bool         first_frame = true;
    bool         duplicate;

    //cast arg
    struct link_handle *link = (struct link_handle *) arg;
    //declare states
    enum states {WAIT, CHECK_CRC, COPY, SEND_ACK};
    //current state variable
    enum states state = WAIT;

    while(!link->rx->stop){
        switch(state){
            case WAIT:
                //--Wait for PHY to receive a frame
                //DEBUG_MSG("RX: State = WAIT\n");
                rx_buf = phy_request_rx_buf(link->phy, 500);
                if (rx_buf == NULL){
                    //timed out (or error). Continue waiting.
                    state = WAIT;
                }else{
                    state = CHECK_CRC;
                }
                break;
            case CHECK_CRC:
                //--We received a frame from the PHY. Check the CRC
                DEBUG_MSG("RX: State = CHECK_CRC\n");
                //First check if it's an ack or data frame to determine length
                if (rx_buf[0] == DATA_FRAME_CODE){
                    DEBUG_MSG("RX: Received a data frame\n");
                    is_data_frame = true;
                    frame_length  = link->frame_length;
                }else{
                    DEBUG_MSG("RX: Received a ack frame\n");
                    is_data_frame = false;
                    frame_length  = ACK_FRAME_LENGTH;
                }
                //Copy the received CRC
                memcpy(&crc_32_rx, &rx_buf[frame_length - sizeof(crc_32)],
                        sizeof(crc_32));
                //Compute expected CRC
                crc_32 = crc32(rx_buf, frame_length - sizeof(crc_32));
                //Compare received CRC vs expected CRC
                if (crc_32_rx != crc_32){
                    #ifdef LINK_IGNORE_CRC_ERRORS
                        NOTE("RX: Frame received with errors. Continuing anyway (DEBUG).\n");
                        state = COPY;
                    #else
                        //Drop the frame since there was an error
                        //First release the buffer for the PHY
                        phy_release_rx_buf(link->phy);
                        NOTE("RX: Frame received with errors. Dropping.\n");
                        state = WAIT;
                    #endif
                }else{
                    DEBUG_MSG("RX: Frame received with no errors\n");
                    state = COPY;
                }
                break;
            case COPY:
                //--CRC passed. Now copy the frame to link layer buffer
                //--if it is not a duplicate frame
                DEBUG_MSG("RX: State = COPY\n");
                if (is_data_frame){
                    //Is the previous data frame still being worked with?
                    if (link->rx->data_buf_filled){
                        //Instead of causing a disruption, drop the current frame
                        ERROR("RX: Data frame dropped!\n");
                        phy_release_rx_buf(link->phy);
                        state = WAIT;
                        break;
                    }
                    //Copy/convert to data frame struct
                    convert_buf_to_data_frame_struct(rx_buf, &(link->rx->data_frame_buf),
                                                     link->payload_length);
                    //Release buffer from the phy
                    phy_release_rx_buf(link->phy);
                    //Is this frame a duplicate of the last frame?
                    if (link->rx->data_frame_buf.seq_num == seq_num && !first_frame){
                        NOTE("RX: Received a duplicate frame. Will send ACK but ignore data\n");
                        duplicate = true;
                    }else{
                        duplicate = false;
                    }
                    seq_num     = link->rx->data_frame_buf.seq_num;
                    first_frame = false;
                    //Mark buffer filled if not a duplicate data frame
                    if (!duplicate){
                        //Mark shared buffer status as filled via mutex lock
                        status = MUTEX_LOCK(&(link->rx->data_buf_status_lock));
                        if (status != 0){
                            ERROR("RX: %s(): Error locking pthread_mutex\n", __FUNCTION__);
                            return NULL;
                        }
                        link->rx->data_buf_filled = true;
                        //Signal that the link data buffer is filled
                        status = COND_SIGNAL(&(link->rx->data_buf_filled_cond));
                        if (status != 0){
                            ERROR("RX: %s(): Error signaling pthread_cond\n", __FUNCTION__);
                            return NULL;
                        }
                        status = MUTEX_UNLOCK(&(link->rx->data_buf_status_lock));
                        if (status != 0){
                            ERROR("RX: %s(): Error unlocking pthread_mutex\n", __FUNCTION__);
                            return NULL;
                        }
                    }
                    //Transition to send acknowledgement
                    state = SEND_ACK;
                }else{
                    //Is the previous ack frame still being worked with?
                    if (link->rx->ack_buf_filled){
                        //Instead of causing a disruption, drop the current frame
                        ERROR("RX: ACK frame dropped!\n");
                        phy_release_rx_buf(link->phy);
                        state = WAIT;
                        break;
                    }
                    //Copy/convert to ack frame struct and mark ack buffer filled
                    convert_buf_to_ack_frame_struct(rx_buf, &(link->rx->ack_frame_buf));
                    //Release buffer from the phy
                    phy_release_rx_buf(link->phy);
                    //Signal that the link ack buffer is filled
                    status = MUTEX_LOCK(&(link->rx->ack_buf_status_lock));
                    if (status != 0){
                        ERROR("RX: %s(): Error locking pthread_mutex\n", __FUNCTION__);
                        return NULL;
                    }
                    link->rx->ack_buf_filled = true;
                    status = COND_SIGNAL(&(link->rx->ack_buf_filled_cond));
                    if (status != 0){
                        ERROR("RX: %s(): Error signaling pthread_cond\n", __FUNCTION__);
                        return NULL;
                    }
                    status = MUTEX_UNLOCK(&(link->rx->ack_buf_status_lock));
                    if (status != 0){
                        ERROR("RX: %s(): Error unlocking pthread_mutex\n", __FUNCTION__);
                        return NULL;
                    }
                    //Done with the frame. Go back to WAIT state
                    state = WAIT;
                }
                break;
            case SEND_ACK:
                //--We received a data frame, now it's time to send the ack
                DEBUG_MSG("RX: State = SEND_ACK (ack# = %hu)\n", seq_num);

                link->tx->ack_frame_buf.type    = ACK_FRAME_CODE;
                link->tx->ack_frame_buf.ack_num = seq_num;

                //Copy frame into send buf
                convert_ack_frame_struct_to_buf(&(link->tx->ack_frame_buf),
                                                ack_send_buf);
                //Calculate the CRC
                crc_32 = crc32(ack_send_buf, ACK_FRAME_LENGTH - sizeof(crc_32));
                //Copy this CRC to the send buf
                memcpy(&ack_send_buf[ACK_FRAME_LENGTH - sizeof(crc_32)], &crc_32,
                        sizeof(crc_32));
                //Transmit with phy
                status = phy_fill_tx_buf(link->phy, ack_send_buf, ACK_FRAME_LENGTH);
                if (status != 0){
                    ERROR("Couldn't fill phy tx buffer\n");
                    return NULL;
                }
                //Done; go back to the WAIT state
                state = WAIT;
                break;
            default:
                ERROR("%s(): invalid state\n", __FUNCTION__);
                return NULL;
        }
    }

    //Stop condition detected, shutting down thread

    //In case receive_payload() is stuck on its condition wait, signal condition that RX
    //data buffer is filled to break it out of the condition wait
    status = MUTEX_LOCK(&(link->rx->data_buf_status_lock));
    if (status != 0){
        ERROR("RX: %s(): Error locking pthread_mutex\n", __FUNCTION__);
        return NULL;
    }
    status = COND_SIGNAL(&(link->rx->data_buf_filled_cond));
    if (status != 0){
        ERROR("RX: %s(): Error signaling pthread_cond\n", __FUNCTION__);
        return NULL;
    }
    status = MUTEX_UNLOCK(&(link->rx->data_buf_status_lock));
    if (status != 0){
        ERROR("RX: %s(): Error unlocking pthread_mutex\n", __FUNCTION__);
        return NULL;
    }

    return NULL;
}
