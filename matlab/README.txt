This set of MATLAB model files will generate/receive binary CPFSK baseband waveforms. Only
models the physical layer portion of the modem, does not include link layer.
Works on both MATLAB and GNU Octave, however it may not be fully tested in MATLAB.

Run fsk.m to simulate the FSK modem
- Set use_file=1 to write/read IQ samples to/from a file so they can be
  transmitted/received on an RF carrier with a bladeRF (e.g. using bladeRF-cli)
   -set csv=1 to use a CSV file, csv=0 to use a binary file
- If use_file=0, the modem will be simulated only internally in MATLAB/Octave

fsk_mod(): FSK baseband modulator function
- generates CPFSK baseband IQ waveform of the given set of bits

fsk_demod(): FSK baseband demodulator function
- extracts the bit sequence of the given CPFSK baseband waveform

fsk_transmit(): Generates baseband signal for an FSK frame
- Applies scrambling to input data bits
- Adds training sequence and preamble to create full FSK frame
- Calls fsk_mod() to modulate FSK frame into an IQ signal
- Adds a ramp up/ramp down to the beginning/end of the signal

fsk_receive(): Receives an FSK frame from a baseband signal
- Low-pass filters and power normalizes the input signal
- Correlates the input signal with the given preamble waveform to determine
  start of FSK data
- Calls fsk_demod() to extract bits from the FSK data signal
- Descrambles output data bits

fsk.m: Script for simulating the modem
- prompts for an input string (spaces are welcome) or generates random data
- converts ASCII string to a set of bits
- calls fsk_transmit to generate the CPFSK baseband IQ waveform
- if use_file=0, adds attenuation and noise to the signal to simulate a channel
- if use_file=1, writes TX IQ samples to CSV file for transmission with a bladeRF, and waits for user to populate the RX CSV samples file received by a bladeRF (such has with bladeRF-cli)
- calls fsk_receive to process the waveform and extract data bits
- converts bits to ASCII string
- prints received string to command window
- makes plots

