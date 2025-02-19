#include <libbladeRF.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>

// Comment this out to disable metadata
#define USE_METADATA

#define SAMPLE_RATE    2000000  // 2 MHz
#define BUFFER_SIZE    16384    // Same as FSK modem
#define NUM_BUFFERS    64
#define NUM_TRANSFERS  16
#define TIMEOUT_MS     3500

int main(int argc, char *argv[]) {
   struct bladerf *dev = NULL;
   int status;
   struct bladerf_metadata meta;
   int16_t *tx_samples;
   unsigned int nsamp_out;
   unsigned int requested_samples;

   // Check command line arguments
   if (argc != 2) {
      fprintf(stderr, "Usage: %s <number_of_samples>\n", argv[0]);
      return 1;
   }

   // Parse the number of samples from command line
   requested_samples = atoi(argv[1]);
   if (requested_samples <= 0) {
      fprintf(stderr, "Number of samples must be positive\n");
      return 1;
   }

   #ifdef USE_METADATA
      nsamp_out = requested_samples;
   #else
      //round up to multiple of BUFFER_SIZE
      int rem = requested_samples % BUFFER_SIZE;
      if (rem != 0){
         nsamp_out = requested_samples + (BUFFER_SIZE-rem);
      }else{
         nsamp_out = requested_samples;
      }
   #endif

   // Allocate memory for samples (2 int16_t per complex sample)
   tx_samples = (int16_t *)calloc(nsamp_out * 2, sizeof(int16_t));
   if (tx_samples == NULL) {
      fprintf(stderr, "Failed to allocate TX samples\n");
      return 1;
   }

//    // Set verbosity level to verbose
//    bladerf_log_set_verbosity(BLADERF_LOG_LEVEL_VERBOSE);
   
   // Open device
   status = bladerf_open(&dev, NULL);
   if (status != 0) {
      fprintf(stderr, "Failed to open device: %s\n", bladerf_strerror(status));
      return 1;
   }

   // Print versions
   struct bladerf_version fw_version;
   bladerf_fw_version(dev, &fw_version);
   printf("Firmware version: %s\n", fw_version.describe);
   
   struct bladerf_version fpga_version;
   bladerf_fpga_version(dev, &fpga_version);
   printf("FPGA version: %s\n", fpga_version.describe);

   // Configure device
   status = bladerf_set_frequency(dev, BLADERF_CHANNEL_TX(0), 915000000);
   if (status != 0) goto error;

   status = bladerf_set_sample_rate(dev, BLADERF_CHANNEL_TX(0), SAMPLE_RATE, NULL);
   if (status != 0) goto error;

   status = bladerf_set_bandwidth(dev, BLADERF_CHANNEL_TX(0), 1500000, NULL);
   if (status != 0) goto error;

   // Configure sync interface
   #ifdef USE_METADATA
      bladerf_format format = BLADERF_FORMAT_SC16_Q11_META;
      printf("Using metadata mode\n");
   #else
      bladerf_format format = BLADERF_FORMAT_SC16_Q11;
      printf("Not using metadata mode\n");
   #endif

   printf("Configuring TX...\n");
   status = bladerf_sync_config(dev, BLADERF_TX_X1, format,
                               NUM_BUFFERS, BUFFER_SIZE,
                               NUM_TRANSFERS, TIMEOUT_MS);
   if (status != 0) {
      fprintf(stderr, "Failed to configure TX sync interface: %s\n", 
              bladerf_strerror(status));
      goto error;
   }

   // Optional: Configure and enable RX
   printf("Configuring RX...\n");
   status = bladerf_sync_config(dev, BLADERF_RX_X1, format,
                               NUM_BUFFERS, BUFFER_SIZE,
                               NUM_TRANSFERS, TIMEOUT_MS);
   if (status != 0) {
      fprintf(stderr, "Failed to configure RX sync interface: %s\n", 
              bladerf_strerror(status));
      goto error;
   }

   // Enable RX module (comment out to test without RX enabled)
   status = bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), true);
   if (status != 0) {
      fprintf(stderr, "Failed to enable RX module: %s\n", bladerf_strerror(status));
      goto error;
   }

   // Enable TX module
   status = bladerf_enable_module(dev, BLADERF_CHANNEL_TX(0), true);
   if (status != 0) {
      fprintf(stderr, "Failed to enable TX module: %s\n", bladerf_strerror(status));
      goto error;
   }

   // Initialize metadata
   memset(&meta, 0, sizeof(meta));
   meta.flags = BLADERF_META_FLAG_TX_BURST_START | 
                BLADERF_META_FLAG_TX_BURST_END |
                BLADERF_META_FLAG_TX_NOW;

   printf("Starting transmission...\n");
   status = bladerf_sync_tx(dev, tx_samples, nsamp_out, &meta, TIMEOUT_MS);
   if (status != 0) {
      fprintf(stderr, "Failed to TX samples: %s\n", bladerf_strerror(status));
      goto error;
   }
   printf("Sleeping 6 sec to check for error...\n");
   sleep(6);

cleanup:
   free(tx_samples);
   bladerf_enable_module(dev, BLADERF_CHANNEL_TX(0), false);
   bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), false);
   bladerf_close(dev);
   return status == 0 ? 0 : 1;

error:
   fprintf(stderr, "Error: %s\n", bladerf_strerror(status));
   goto cleanup;
}