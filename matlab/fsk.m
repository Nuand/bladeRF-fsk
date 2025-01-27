%-------------------------------------------------------------------------
% fsk.m: Simulation/model of PHY portion of CPFSK modem. Generates a transmit signal and
% feeds it into the receiver model, makes plots and displays ASCII output. When compared
% to the C modem this code has the following differences:
%   1) Simpler/less rigorous power normalization
%   2) No scrambling.
%   3) No link layer functions (no link framing with header, CRC check, ACKs, etc)
%
% Set use_file=1 to write TX samples to a CSV file that can be transmitted with a bladeRF,
% and ingest RX samples from a CSV file received from a bladeRF. bladeRF-cli can be used
% to transmit/receive the files. txfile and rxfile specify the filenames.
%
% If use_file=0, change 'noise_pow' for different amount of channel AWGN noise. Change
% 'gain' for different amount of channel gain.
%
% Set rand_input=1 for pseudorandom input data rather a string entered by the user.
% Change 'null_amt' for different amount of flat samples placed at front/back of tx
% waveform.
%
% This file is part of the bladeRF-fsk project
%
% Copyright (C) 2016 Nuand LLC
%
% This program is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 2 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License along
% with this program; if not, write to the Free Software Foundation, Inc.,
% 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
%-------------------------------------------------------------------------
clear; close all;

%%-------------------SETTINGS-----------------------------
use_file       = 0;     %1=Write TX samples to CSV file and read RX samples from CSV file
                        %0=Simulate internally only
rand_input     = 0;     %1=use random input TX data, 0=prompt for string entered by user
   rand_nbytes = 1000;  %number of bytes to transmit if rand_input=1

Fs             = 2e6;   %Sample rate of 2 Msps (500ns sample period)
samps_per_symb = 8;     %Samples per symbol
h              = pi/2;  %Phase modulation index (phase deviation per symbol)
                        %Note: preamble has been optimized for h = pi/2
dec_factor     = 2;     %Amount to decimate by when performing correlation with preamble

if use_file
   addpath('../../../misc/matlab/');   %add save_csv()/load_csv() utility functions
   txfile = 'tx_samples.csv';
   rxfile = 'rx_samples.csv';
end

%32 bit training sequence
%hex: AA, AA, AA, AA
%Note: In order for the preamble waveform not to be messed up, the last
%sample of the modulated training sequence MUST be 1 + 0j
training_seq = ['10101010';
                '10101010';
                '10101010';
                '10101010'];
%32-bit preamble - hex: 2E, 69, 2C, F0
preamble     = ['00101110'; ...
                '01101001'; ...
                '00101100'; ...
                '11110000'];

%--Plot settings
%Turn on grid by default
set(groot,'defaultAxesXGrid','on')
set(groot,'defaultAxesYGrid','on')
%pwelch setting string
if (exist('OCTAVE_VERSION', 'builtin') ~= 0)
   plot_freqrange = 'centerdc'; %octave
else
	plot_freqrange = 'centered'; %matlab
end

%%-------------------TRANSMIT-----------------------------
%Amount of flat samples (0+0j) to add to the beginning/end of tx signal
null_amt = 200;

if rand_input
   %Create random data
   fprintf('Generating %d bytes of random data\n', rand_nbytes);
   rng(0);  %set seed for consistent output
   tx_bits = randi([0, 255], 1, rand_nbytes);
   tx_bits = dec2bin(uint8(tx_bits), 8);
else
   %Prompt for string to transmit
   prompt = 'Enter a string to TX: ';
   str = input(prompt, 's');
   %Convert to bit matrix
   tx_bits = dec2bin(uint8(str), 8);
end

%Make FSK signal
tx_sig = fsk_transmit(training_seq, preamble, tx_bits, samps_per_symb, h);
%Add some flat output to the front and back of the signal
tx_sig = [zeros(1, null_amt), tx_sig, zeros(1, null_amt)];

%time vector: 1/Fs increments
t    = (0:length(tx_sig)-1)/Fs;
nsym = numel(training_seq) + numel(preamble) + numel(tx_bits);

%--TX plots
%TX IQ samples
figure('position', [0 0 960 200]);
plot(t, real(tx_sig)); hold on;
plot(t, imag(tx_sig));
xlabel('time (sec)');
%symbol boundaries
first_sym_start = null_amt + samps_per_symb;   %first symbol start idx
preamble_start  = first_sym_start + numel(training_seq)*samps_per_symb;
data_start      = preamble_start  + numel(preamble)    *samps_per_symb;
last_sym_end    = data_start      + numel(tx_bits)     *samps_per_symb;

plot(repmat(t(first_sym_start), 1, 2), [-1 1], '--');
plot(repmat(t(preamble_start),  1, 2), [-1 1], '--');
plot(repmat(t(data_start),      1, 2), [-1 1], '--');
plot(repmat(t(last_sym_end),    1, 2), [-1 1], '--');
plot(t(first_sym_start:samps_per_symb  :last_sym_end), zeros(1, nsym+1),   '*r');
plot(t(first_sym_start:samps_per_symb*8:last_sym_end), zeros(1, nsym/8+1), '*g');
legend('I', 'Q', 'training start', 'preamble start', 'data start', 'data end', 'sym boundaries', 'byte boundaries');
title('TX IQ samples vs time');

%TX spectrum
figure('position', [0 0 960 200]);
pwelch(tx_sig, [], [], 4096, Fs, plot_freqrange, 'dB');
xlabel('frequency [Hz]');
ylabel('power [dB]');
title('TX spectrum');

%---------------------CHANNEL-------------------------
if use_file
   %Write to csv file
   save_csv(txfile, tx_sig.');
   fprintf('Wrote TX IQ samples to %s.\n', txfile);
else
   %--Add gaussian noise and attenuation to signal
   %Noise power desired in channel (units dBW)
   noise_pow = -25;
   %gain desired in channel (this should be less than 1)
   gain      = 0.6;
   noise     = wgn(1, length(tx_sig), noise_pow) + 1j*wgn(1, length(tx_sig), noise_pow);
   rx_sig    = gain*tx_sig + noise;
end

%%---------------------RECEIVE---------------------------
if use_file
   %Wait for user to transmit/receive these samples with bladeRF
   fprintf('Press any key when RX IQ samples file %s is ready...\n', rxfile);
   pause;
   %Load signal from file
   rx_sig = load_csv(rxfile).';
else
   %clamp signal to [-1.0, 1.0]
   rx_sig_i = real(rx_sig);
   rx_sig_q = imag(rx_sig);
   rx_sig_i(rx_sig_i > 1.0) = 1.0;
   rx_sig_i(rx_sig_i < -1.0) = -1.0;
   rx_sig_q(rx_sig_q > 1.0) = 1.0;
   rx_sig_q(rx_sig_q < -1.0) = -1.0;
   rx_sig = rx_sig_i + 1j*rx_sig_q;
end

%Get the modulated IQ waveform for the preamble
preamble_waveform = fsk_mod(preamble, samps_per_symb, h, 0);
%Filter/Normalize/Detect/Demodulate FSK signal
[rx_bits, rx_info] = fsk_receive(preamble_waveform, rx_sig, dec_factor, ...
                                 samps_per_symb, h, size(tx_bits, 1));

%--RX plots
%time vector: 1/Fs increments
t = (0:length(rx_sig)-1)/Fs;
%RX IQ samples
figure('position', [0 0 960 200]);
plot(t, real(rx_sig)); hold on;
plot(t, imag(rx_sig));
ylim([-1, 1]);
xlabel('time (sec)');
title('RX raw IQ samples vs time');

%RX raw spectrum
figure('position', [0 0 960 200]);
pwelch(rx_sig, [], [], 4096, Fs, plot_freqrange, 'dB');
xlabel('frequency [Hz]');
ylabel('power [dB]');
title('RX raw spectrum');

%RX filtered IQ samples
figure('position', [0 0 960 200]);
plot(real(rx_info.iq_filt_norm));
hold on;
plot(imag(rx_info.iq_filt_norm));
%symbol boundaries
first_sym_start = rx_info.sig_start_idx-(numel(training_seq)+numel(preamble))*samps_per_symb;
preamble_start  = first_sym_start + numel(training_seq)*samps_per_symb;
data_start      = preamble_start  + numel(preamble)    *samps_per_symb;
last_sym_end    = data_start      + numel(tx_bits)     *samps_per_symb;
%byte boundaries

plot(repmat(first_sym_start, 1, 2), [-1 1], '--');
plot(repmat(preamble_start,  1, 2), [-1 1], '--');
plot(repmat(data_start,      1, 2), [-1 1], '--');
plot(repmat(last_sym_end,    1, 2), [-1 1], '--');
plot(first_sym_start:samps_per_symb  :last_sym_end, zeros(1, nsym+1),   '*r');
plot(first_sym_start:samps_per_symb*8:last_sym_end, zeros(1, nsym/8+1), '*g');
legend('I', 'Q', 'training start', 'preamble start', 'data start', 'data end', 'sym boundaries', 'byte boundaries');
xlabel('samp idx');
title('RX filtered & normalized IQ samples');

%RX cross correlation with preamble. Plot power (i^2 + q^2).
figure('position', [0 0 960 200]);
plot(abs(rx_info.preamble_corr).^2);
title('RX cross correlation with preamble (power)');

%RX dphase (data signal only, training/preamble not included)
if (rx_info.dphase ~= -1)
   figure('position', [0 0 960, 200]);
   plot(rx_info.dphase);
   %sym boundaries
   hold on;
   plot(1:samps_per_symb:1+numel(tx_bits)*samps_per_symb, zeros(1, numel(tx_bits)+1), '*r');
   %byte boundaries
   plot(1:samps_per_symb*8:1+numel(tx_bits)*samps_per_symb, zeros(1, numel(tx_bits)/8+1), '*g');
   legend('change in phase', 'sym boundaries', 'byte boundaries');
   title('RX dphase (data signal only)');
end

%RX dphase total per symbol (data signal only, training/preamble not included)
if rx_info.dphase_sym ~= -1
   figure('position', [0 0 960, 200]);
   plot(rx_info.dphase_sym);
   %byte boundaries
   hold on;
   plot(1:8:length(rx_info.dphase_sym), zeros(1, length(rx_info.dphase_sym)/8), '*g');;
   legend('total change in phase', 'byte boundaries (first bit)');
   title('RX total dphase per symbol (data signal only)');
end

%---------------------Print received data-------------------
if (rx_bits(1) ~= -1)
   fprintf('Received: ''%s''\n', bin2dec(rx_bits));
   if isequal(rx_bits, tx_bits)
      fprintf('RX data matched TX data\n');
   else
      fprintf(2, 'RX data did not match TX data\n');
   end
end

