# bladeRF-FSK #

## Project Overview ##

The bladeRF-fsk project is a simple frequency shift keying (FSK) based software modem
implemented entirely on the host PC side in C code. The project uses libbladeRF to
transmit/receive samples with a bladeRF device. A USB 3.0 port is not required when using
this modem. The modem supports both bladeRF 1.0 and bladeRF 2.0 devices. The project also
contains a MATLAB/Octave model and simulation of the physical layer (PHY) portion of the
modem.

The top level bladeRF-fsk C program demonstrates the functionality of the modem in a
simple bladeRF-to-bladeRF data transfer program. This program can transmit/receive both
text (like a chat program) and binary files (like a file transfer program) with a raw
link rate of 250 kbps. To properly demonstrate the program, two instances of the program
must be run with two separate bladeRF devices (loopback is not supported).

The modem modulates with continuous-phase frequency shift keying (CPFSK). Baseband I/Q
CPFSK samples are sent to the bladeRF device, inside which they converted from digital
to analog, mixed with quadrature RF carriers, and transmitted through the air.
Received signals are mixed with quadrature RF carriers to downconvert to baseband I/Q,
sampled with an ADC, and sent to the host PC program over the USB connection.

The physical layer code features an FIR low-pass filter with decimation, power
normalization, preamble correlation for signal detection, CPFSK modulation/demodulation,
and scrambling. The link layer code features framing, error detection via CRC32 checksums,
and guaranteed delivery of frames via acknowledgements and retransmissions.

This project is meant to be an experimental example and should not be treated as a
rigorous modem.

## Dependencies ##

- [libbladeRF]
   - Installs as part of the main [bladeRF host software]. Install instructions can be
     found on the [wiki].

[libbladeRF]: https://github.com/Nuand/bladeRF/tree/master/host/libraries/libbladeRF
[bladeRF host software]: https://github.com/Nuand/bladeRF/tree/master/host


## Build Instructions ##

The program builds/installs as part of the main bladeRF host software. Follow the
instructions listed on the [wiki]. Instructions to build from the source can also be found
in the [host README].

The program also builds standalone if libbladeRF has been installed on your system. To
build:
```
cd c
mkdir build && cd build
cmake [options] ..
make
```
The executable will be created in the build folder; you can run it with `./bladeRF-fsk`.
Build files can be cleaned up by removing the `build` directory.

[wiki]: https://github.com/Nuand/bladeRF/wiki#user-content-bladeRF_software_buildinstallation
[host README]: https://github.com/Nuand/bladeRF/blob/master/host/README.md

_NOTE_: Debug builds (without compiler optimization) may cause RX overruns which can cause
the modem to fail.

### Build Variables ###

Below is a list of project-specific CMake options.

| Option                                          | Description                                                               |
| ----------------------------------------------- |:--------------------------------------------------------------------------|
| -DBLADERF-FSK_BYPASS_RX_CHANNEL_FILTER=<ON/OFF> | Bypass the RX low-pass channel filter. Default: OFF                       |
| -DBLADERF-FSK_BYPASS_RX_PNORM=<ON/OFF>          | Bypass RX power normalization. Default: OFF                               |
| -DBLADERF-FSK_BYPASS_PHY_SCRAMBLING=<ON/OFF>    | Bypass scrambling in the PHY layer. Default: OFF                          |
| -DBLADERF-FSK_ENABLE_NOTES_ALL=<ON/OFF>         | Print noteworthy messages from all files. Default: OFF                    |
| -DBLADERF-FSK_ENABLE_NOTES_LINK=<ON/OFF>        | Print noteworthy messages from link.c, including CRC errors. Default: OFF |
| -DBLADERF-FSK_ENABLE_NOTES_PHY=<ON/OFF>         | Print noteworthy messages from phy.c, including RX overruns. Default: OFF |
| -DBLADERF-FSK_ENABLE_NOTES_BLADERF=<ON/OFF>     | Print bladeRF noteworthy messages. Default: OFF                           |
| -DBLADERF-FSK_ENABLE_DEBUG_ALL=<ON/OFF>         | Print debug messages from all files & bladeRF. Default: OFF               |
| -DBLADERF-FSK_ENABLE_DEBUG_TEST_SUITE=<ON/OFF>  | Print debug messages from test_suite.c. Default: OFF                      |
| -DBLADERF-FSK_ENABLE_DEBUG_CONFIG=<ON/OFF>      | Print debug messages from config.c. Default: OFF                          |
| -DBLADERF-FSK_ENABLE_DEBUG_LINK=<ON/OFF>        | Print debug messages from link.c. Default: OFF                            |
| -DBLADERF-FSK_ENABLE_DEBUG_PHY=<ON/OFF>         | Print debug messages from phy.c. Default: OFF                             |
| -DBLADERF-FSK_ENABLE_DEBUG_CORR=<ON/OFF>        | Print debug messages from correlator.c. Default: OFF                      |
| -DBLADERF-FSK_ENABLE_DEBUG_BLADERF=<ON/OFF>     | Print bladeRF debug messages. Default: OFF                                |
| -DBLADERF-FSK_ENABLE_VERBOSE_BLADERF=<ON/OFF>   | Print bladeRF verbose (and debug) messages. Default: OFF                  |
| -DBLADERF-FSK_LOG_TX_SAMPLES=<ON/OFF>           | Log all TX samples to binary file tx_samples_[serial].bin. Default: OFF   |
| -DBLADERF-FSK_LOG_RX_SAMPLES=<ON/OFF>           | Log all RX samples to binary file rx_samples_[serial].bin. Default: OFF   |

Setting -DBLADERF-FSK_ENABLE_NOTES_LINK=ON will show you when a frame is received with
CRC errors. Setting -DBLADERF-FSK_ENABLE_NOTES_PHY=ON will enable SNR estimates to print
out after every received frame, and it will show you when RX overruns occur, meaning the
PHY receiver thread could not process a set of samples fast enough and samples had to be
dropped. Setting -DBLADERF-FSK_BYPASS_RX_PNORM=ON likely will cause acquisition to fail,
since the correlation power threshold is based on the assumption of a full scale signal
output by the power normalizer.

## How to Run ##
To run the top-level bladeRF-fsk program with defaults, type into a terminal:
```
bladeRF-fsk
```
To see a list of configuration options and how to set them, type:
```
bladeRF-fsk -h
```
If you want to run on a specific device rather than the first available device, use the
`-d` option to specify the bladeRF serial number, which can be abbreviated.
Example (where `4e` is the first 2 characters of the serial number):
```
bladeRF-fsk -d *:serial=4e
```
By default the program uses the first available bladeRF device, gets TX input from stdin,
writes RX output to stdout, and uses a default set of transmit/receive frequencies and
gains, with automatic gain control on the RX side. Gains may need to be adjusted for a
good connection with another bladeRF running bladeRF-fsk. To transfer files, use the `-i`
and `-o` options. If using stdin for tx data, the program will transmit data line-by-line.

The program runs until it gets an EOF in its TX input.

### Example: Text Chat Between BladeRFs ###
1) Be sure two bladeRF devices are plugged into your PC (or two separate PCs) with
   TX and RX antennas attached.

2) Run bladeRF-fsk on one of the devices:
```
bladeRF-fsk -r 904M -t 924M
```
3) Run bladeRF-fsk on the other device with opposite frequencies:
```
bladeRF-fsk -r 924M -t 904M
```
4) Type out a message on one device and press ENTER. A packet will be transmitted and the
   message will appear on the other device. Both sides can send/receive messages.
5) Press [CTRL-D] on Linux/OSX or [CTRL-Z then ENTER] on Windows to quit

If the sending device does not get any response from the receiving device, it will quit
the program. Try adjusting the gains and run it again.

### Example: Transferring Files Between BladeRFs ###
1) Be sure two bladeRF devices are plugged into your PC (or two separate PCs) with
   TX and RX antennas attached.

2) Run bladeRF-fsk on one of the devices (receiver), with the output RX file specified:
```
bladeRF-fsk -r 904M -t 924M -o rx.jpg
```
3) Run bladeRF-fsk on the other device (sender), with opposite frequencies and the input
   TX file specified:
```
bladeRF-fsk -r 924M -t 904M -i puppy.jpg
```
4) The file will begin transferring, and progress will be printed in the terminal for the
   sending device.

5) Once the transmission is complete, press [CTRL-D] on Linux/OSX or [CTRL-Z then ENTER]
   on Windows to stop the program on the receiving end.

If the sending device does not get any response from the receiving device, it will quit
the program. Try adjusting the gains and run it again.

## Known Limitations ##
1) The program does not currently support the use of an XB-200 transverter expansion
   board for the bladeRF 1 to transmit/receive at frequencies below 300MHz. In order to
   add XB-200 support, a new configuration option as well as functions from the
   "Expansion boards" section of the libbladeRF API would need to be added to the source
   code.

2) The program is currently unable to properly perform two full file transfers in both directions
   simultaneously. Reason: The program runs until is gets an EOF in its TX input,
   meaning whichever side finishes transmitting its file first will quit and stop
   receiving. An EOF bit would need to be added to the link layer packet format in order
   to stop this behavior.

3) SNR estimates are not perfect and may report lower SNR than in reality, particularly
   when AGC is enabled. SNR estimates are based off the noise power estimate shortly after
   the end of each frame, once the noise power estimate from `pnorm.c` has settled.
   Occassionally when AGC is enabled, the AGC reacts too quickly after the frame ends,
   gaining up the noise and causing the noise power estimate to be too high.

4) The program currently does not support Windows

## Troubleshooting ##
Is the FSK modem not working for you? Here are some troubleshooting steps:

1) Enable debug messages to get a clearer picture of what's going on. Use the cmake option
  `-DBLADERF-FSK_ENABLE_DEBUG_ALL=ON`. For example, you'll be able to see if a signal is
  being acquired but the frame has CRC errors. You can go a step further and also set
  `-DBLADERF-FSK_ENABLE_VERBOSE_BLADERF=ON` to get full verbosity in bladeRF device
  messages from libbladeRF.

2) Adjust the TX and RX gains and/or disable AGC. If TX+RX gains are too high, the receiver may experience
  clipping which can cause bit errors or weaken acquisition. If TX gain is too low, SNR
  may be too low to acquire and/or demodulate without errors. If RX gain is too low, there
  may be too much quantization noise at the receiver.

3) If using bladeRF1, be sure TX and RX frequencies are not multiples of each other, i.e. `-r 2G -t 1G`. Harmonics in the bladeRF1 are prominent and can cause distorted signals, resulting in CRC errors in bladeRF-fsk.

4) Log received samples to file, using cmake option `-DBLADERF-FSK_LOG_RX_SAMPLES=ON`.
  File format is binary 16-bit IQ with each sample having a range of [-2047, 2048], I
  before Q in each IQ sample pair, little endian.
   * Feed the signal into the FSK model by running `matlab/fsk.m` in MATLAB/Octave, which will plot the signal, correlate, and attempt to demodulate it. Set `use_file=1`, `csv=0`, `no_tx=1`, `rx_nbytes` to your payload length + 9 for link layer header/footer, and replace `rxfile=rx_samples.bin` with the path to your samples file.
     * Note that `fsk.m` does not include link layer functionality, so it will not perform a CRC check and it will display link layer header and footer bytes.

5) Move devices very close to each other to maximize signal power. Or, replace antennas
  with SMA cables (TX hooked up to RX) and perform wired testing to rule out any RF
  channel effects or interference. Note: be careful not to set TX gain too high to
  potentially damage your device during wired testing. Start with a lower gain. Suggested
  wired gains below:
   * On the bladeRF 2, `--tx-gain 50`, `--rx-gain -12` will result in a clean signal with no clipping and low noise/distortion.
   * On the bladeRF 1, `--tx-gain 50`, `--rx-gain -1` will result in a similar signal that does not clip at the receiver.

6) Check for frequency offset (could be caused by bad calibration). A high enough frequency
  offset (say 20 kHz) will cause signal acquisition to fail. In `c/src/phy.h`, uncomment
  `#define TX_DC_TONE` then rebuild the program to make the PHY transmitter send a DC tone
  in its bursts instead of an FSK signal. Log the received samples to file, and find the
  frequency of the large DC tone spike on the spectrum. Frequency offsets are expected but
  they should be <500 Hz or so if the bladeRF is properly calibrated.

## Modem Details ##
### Waveform Specifications ###

The listed specifications are based off the default sample rate of 2 Msps and the default
packet payload size of 1000 bytes.

| Field                            | Value                                  |
| -------------------------------- |:---------------------------------------|
| BladeRF Sample rate              | 2 Msps                                 |
| BladeRF Bandwidth                | 1 MHz (bladeRF 2), 1.5 MHz (bladeRF 1) |
| Raw link rate                    | 250 kbps                               |
| Symbol rate                      | 250 ksym/s                             |
| Bits per symbol                  | 1                                      |
| Symbol mapping                   | Positive frequency deviation = 1<br>Negative frequency deviation = 0 |
| Samples per symbol               | 8                                      |
| Phase modulation index<br>(phase deviation per symbol) | π/2 radians  (1/4 revolution)  |
| Frequency deviation              | +/- 62.5 kHz                           |
| Main lobe bandwidth              | 375 kHz                                |
| RX FIR filter passband bandwidth | 250 kHz                                |
| RX FIR filter stopband bandwidth | 666 kHz                                |
| Data frame length                | 8138 symbols (32.55 ms)                |
| Data frame payload length        | 1000 bytes                             |
| Byte ordering                    | Little endian (LSB first)              |
| Bit ordering                     | LSb first                              |

### Framing Details ###
Link layer frame is embedded within the physical layer frame.

Note: Link layer data frame payload length is adjustable on the command line via
`-p`/`--packet-size`. Listed below are the frame sizes for the default payload length of
1000 bytes.

PHY frame contents:
| Field             | Length           |
| ----------------- |:-----------------|
| Ramp up           | 8 samples        |
| Training sequence | 4 bytes          |
| Preamble          | 4 bytes          |
| Link layer frame  | 7 or 1009 bytes  |
| Ramp down         | 8 samples        |
| Total             | 15 or 1017 bytes |

Link layer data frame contents:
| Field                    | Length     |
| ------------------------ |:-----------|
| Frame type (data or ACK) | 1 byte     |
| Sequence number          | 2 bytes    |
| Used payload length      | 2 bytes    |
| Payload                  | 1000 bytes |
| CRC32 checksum           | 4 bytes    |
| Total                    | 1009 bytes |

Link layer acknowledgement (ACK) frame contents:
| Field                    | Length     |
| ------------------------ |:-----------|
| Frame type (data or ACK) | 1 byte     |
| Sequence number          | 2 bytes    |
| CRC32 checksum           | 4 bytes    |
| Total                    | 7 bytes    |

The modulation is performed at baseband by rotating the phase either counter-clockwise
(positive phase change = positive frequency = 1) or clockwise (negative phase change =
negative frequency = 1) around the IQ unit circle. Demodulation is performed by calculating
the phase for each sample based on its IQ angle, and measuring the change in phase over
the length of the symbol (positive change = positive frequency = 1, negative change =
negative frequency = 0).

The training sequence allows time for power normalization and any automatic gain control
to settle.

A preamble is used for synchronization.

## Commit Guidelines ##
Commit messages in this repository use the following guidelines:
* the first line of the message is a concise summary prefixed with the section/function of code that has changed. If multiple portions have changed, use multiple prefixes separated by commas. Use the following prefixes:
  * `link:` changes to link.c/h, crc32.c/h
  * `phy:` changes to  phy.c/h, radio_config.c/h, fsk.c/h, fir_filter.c/h, rx_ch_filter.h, pnorm.c/h, correlator.c/h, prng.c/h
  * `doc:` changes to README.md
  * `test:` changes to test_suite.c
  * `ui:` changes to config.c/h and bladeRF-fsk.c
  * `cmake:` changes to CMakeLists.txt
  * `model:` changes to files in the matlab/ folder
  * `misc:` changes to utils.c/h, or something that doesn't fit well into another category
* each line of the message is <=80 characters wide