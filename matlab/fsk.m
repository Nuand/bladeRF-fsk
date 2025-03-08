%-------------------------------------------------------------------------
% fsk.m: Simulation/model of PHY portion of CPFSK modem. Generates a transmit signal and
% feeds it into the receiver model, makes plots and displays ASCII output. When compared
% to the C modem this code has the following differences:
%   1) Samples use normalized floating point [-1.0, 1.0]
%   2) No link layer functions (no link framing with header, CRC check, ACKs, etc)
%
% Set use_file=1 to write TX samples to a file that can be transmitted with a bladeRF,
% and ingest RX samples from a file received from a bladeRF. bladeRF-cli can be used
% to transmit/receive the files. txfile and rxfile specify the filenames.
%
% If use_file=0, change 'noise_pow' for different amount of channel AWGN noise. Change
% 'gain' for different amount of channel gain.
%
% Set rand_input=1 for pseudorandom input data rather a string entered by the user.
% Change 'null_amt' for different amount of flat samples placed at front/back of tx
% waveform.
%
% Use 'no_tx' and 'no_rx' to simulate only one half of the modem.
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
use_file       = 0;     %1=Write TX samples to file and read RX samples from file
                        %0=Simulate internally only
   csv         = 1;     %1=CSV file, 0=binary file
rand_input     = 0;     %1=use random input TX data, 0=prompt for string entered by user
   rand_nbytes = 1000;  %number of bytes to transmit if rand_input=1

scrambling     = 1;     %1=do scrambling of data bits, 0=no scrambling
no_tx          = 0;     %1=Don't simulate TX side (must use file for RX signal)
   rx_nbytes   = 1000;  %if no_tx=1, number of bytes to demodulate on the RX side
no_rx          = 0;     %1=Don't simulate RX side

Fs             = 2e6;   %Sample rate of 2 Msps (500ns sample period)
samps_per_symb = 8;     %Samples per symbol
h              = pi/2;  %Phase modulation index (phase deviation per symbol)
                        %Note: preamble has been optimized for h = pi/2
dec_factor     = 2;     %Amount to decimate by when performing correlation with preamble

if use_file
   addpath('../../../misc/matlab/');   %add save_csv()/load_csv() utility functions
   if csv
      txfile = 'tx_samples.csv';
      rxfile = 'rx_samples.csv';
   else
      txfile = 'tx_samples.bin';
      rxfile = 'rx_samples.bin';
   end
end

%32 bit training sequence
%hex: AA, AA, AA, AA
training_seq = ['10101010';
                '10101010';
                '10101010';
                '10101010'];
%32-bit preamble - hex: 2E, 69, 2C, F0
preamble     = ['00101110';
                '01101001';
                '00101100';
                '11110000'];

if scrambling
   scrambling_seed = uint64(74797187195719809);   %=0x0109BBA53CFFD081
else
   scrambling_seed = [];
end

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
if ~no_tx
   %Amount of flat samples (0+0j) to add to the beginning/end of tx signal
   null_amt = 200;

   if rand_input
      %Create random data
      fprintf('Generating %d bytes of random data to TX\n', rand_nbytes);
      %rng(5);  %set seed for consistent output
      tx_bits = randi([0, 255], 1, rand_nbytes);
      tx_bits = dec2bin(uint8(tx_bits), 8);
   else
      %Prompt for string to transmit
      prompt = 'Enter a string to TX: ';
      str = input(prompt, 's');

      %Convert to bit matrix
      tx_bits = dec2bin(uint8(str), 8);
      fprintf('       String to TX: ''%s''\n', bin2dec(tx_bits));
   end

   %%Add header/footer to mimic link layer data frame
   %seq_num   = 56444;
   %frame_len = size(tx_bits, 1);
   %hdr       = dec2bin([0, bitand(seq_num, 255), bitshift(seq_num, -8), bitand(frame_len, 255), bitshift(frame_len, -8)], 8);
   %ftr       = dec2bin([123, 80, 64, 129], 8);  %fake CRC32
   %tx_bits   = [hdr; tx_bits; ftr];

   %%DEBUG
   %%Write tx_bits to binary file
   %fid = fopen('tx.bin', 'w');
   %fwrite(fid, bin2dec(tx_bits));
   %fclose(fid);

   %Make FSK signal
   tx_sig = fsk_transmit(training_seq, preamble, tx_bits, scrambling_seed, samps_per_symb, h);
   %Add some flat output to the front and back of the signal
   tx_sig = [zeros(1, null_amt), tx_sig, zeros(1, null_amt)];

   tx_nsym = numel(training_seq) + numel(preamble) + numel(tx_bits);

   %--TX plots
   %TX IQ samples
   figure('position', [0 0 960 200]);
   plot(real(tx_sig)); hold on;
   plot(imag(tx_sig));
   xlabel('time (sec)');
   %symbol boundaries
   first_sym_start = null_amt + samps_per_symb;   %first symbol start idx
   preamble_start  = first_sym_start + numel(training_seq)*samps_per_symb;
   data_start      = preamble_start  + numel(preamble)    *samps_per_symb;
   last_sym_end    = data_start      + numel(tx_bits)     *samps_per_symb;

   plot(repmat(first_sym_start, 1, 2), [-1 1], '--');
   plot(repmat(preamble_start,  1, 2), [-1 1], '--');
   plot(repmat(data_start,      1, 2), [-1 1], '--');
   plot(repmat(last_sym_end,    1, 2), [-1 1], '--');
   plot(first_sym_start:samps_per_symb  :last_sym_end, zeros(1, tx_nsym+1),   '*r');
   plot(first_sym_start:samps_per_symb*8:last_sym_end, zeros(1, tx_nsym/8+1), '*g');
   legend('I', 'Q', 'training start', 'preamble start', 'data start', 'data end', 'sym boundaries', 'byte boundaries');
   title('TX IQ samples');

   %TX spectrum
   figure('position', [0 0 960 200]);
   pwelch(tx_sig, [], [], 4096, Fs, plot_freqrange, 'dB');
   xlabel('frequency [Hz]');
   ylabel('power [dB]');
   title('TX spectrum');

   %---------------------CHANNEL-------------------------
   if use_file
      %Write signal to file
      if csv
         save_csv(txfile, tx_sig.');         %Write to csv file
      else
         save_sc16q11(txfile, tx_sig.');     %Write to binary file
      end
      fprintf('Wrote TX IQ samples to %s.\n', txfile);
   elseif ~no_rx
      %--Add gaussian noise and attenuation to signal
      %Noise power desired in channel (units dBW)
      noise_pow   = -20;
      %gain magnitude desired in channel (this should be less than 1)
      chan_gain   = 0.6;
      %phase change from channel
      chan_phase  = rand*2*pi;
      rx_sig_chan = chan_gain*exp(1j*chan_phase)*tx_sig;  %RX sig with only channel effects
      noise       = wgn(1, length(tx_sig), noise_pow, 'complex');

      rx_sig      = rx_sig_chan + noise;

      %measure SNR, using only the portion of rx_sig that contains the FSK signal
      snr_meas    = 10*log10( mean(abs(rx_sig_chan(null_amt+1:end-null_amt)).^2) /
                              mean(abs(noise      (null_amt+1:end-null_amt)).^2) );
      fprintf('Note: SNR = %.2f dB\n', snr_meas);

      %normalize to [-1.0, 1.0] mimicking our 12 bit limit
      rx_sig = rx_sig/max(abs(rx_sig));

%      %DEBUG: estimate SNR using portion of rx_sig containing signal, comparing to noise only
%      %we get S+N and N. S ~= (S+N) - N
%      signoise_pwr = mean(abs(rx_sig(null_amt+1:end-null_amt)).^2)
%      noise_pwr    = mean(abs(rx_sig([1:null_amt,end-null_amt+1:end])).^2)
%      sig_pwr_est  = signoise_pwr - noise_pwr;
%      snr_est      = 10*log10(sig_pwr_est / noise_pwr);
%      fprintf('Note: SNR estimate = %.2f dB\n', snr_est);
   end
end

%%---------------------RECEIVE---------------------------
if ~no_rx
   if use_file
      %Wait for user to transmit/receive these samples with bladeRF
      fprintf('Press any key when RX IQ samples file %s is ready...\n', rxfile);
      pause;
      if csv
         rx_sig = load_csv(rxfile).';     %Load from CSV file
      else
         rx_sig = load_sc16q11(rxfile).'; %Load from binary file
      end
   end

   %Get the modulated IQ waveform for the preamble
   preamble_waveform = fsk_mod(preamble, samps_per_symb, h, 0);
   %Filter/Normalize/Detect/Demodulate FSK signal
   if ~no_tx
      rx_nbytes = size(tx_bits, 1);
   end
   [rx_bits, rx_info] = fsk_receive(preamble_waveform, rx_sig, dec_factor, ...
                                    samps_per_symb, h, rx_nbytes, scrambling_seed);

   rx_nsym = numel(training_seq) + numel(preamble) + rx_nbytes*8;

   %--RX plots
   %time vector: 1/Fs increments
   %RX IQ samples
   figure('position', [0 0 960 200]);
   plot(real(rx_sig)); hold on;
   plot(imag(rx_sig));
   ylim([-1, 1]);
   xlabel('sample index');
   title('RX raw IQ samples');

   %RX raw spectrum
   figure('position', [0 0 960 200]);
   pwelch(rx_sig, [], [], 16384, Fs, plot_freqrange, 'dB');
   xlabel('frequency [Hz]');
   ylabel('power [dB]');
   title('RX raw spectrum');

   %RX filtered/normalized spectrum
   figure('position', [0 0 960 200]);
   pwelch(rx_info.iq_filt_norm, [], [], 16384, Fs, plot_freqrange, 'dB');
   xlabel('frequency [Hz]');
   ylabel('power [dB]');
   title('RX filtered+normalized spectrum');

   %RX filtered/normalized IQ samples
   figure('position', [0 0 960 200]);
   plot(real(rx_info.iq_filt_norm));
   hold on;
   plot(imag(rx_info.iq_filt_norm));
   if rx_bits(1) ~= -1
      %symbol boundaries
      first_sym_start = rx_info.sig_start_idx-(numel(training_seq)+numel(preamble))*samps_per_symb;
      preamble_start  = first_sym_start + numel(training_seq)*samps_per_symb;
      data_start      = preamble_start  + numel(preamble)    *samps_per_symb;
      last_sym_end    = data_start      + rx_nbytes*8        *samps_per_symb;

      plot(repmat(first_sym_start, 1, 2), [-1 1], '--');
      plot(repmat(preamble_start,  1, 2), [-1 1], '--');
      plot(repmat(data_start,      1, 2), [-1 1], '--');
      plot(repmat(last_sym_end,    1, 2), [-1 1], '--');

      plot(first_sym_start:samps_per_symb  :last_sym_end, zeros(1, rx_nsym+1),   '*r');
      plot(first_sym_start:samps_per_symb*8:last_sym_end, zeros(1, rx_nsym/8+1), '*g');
      legend('I', 'Q', 'training start', 'preamble start', 'data start', 'data end', 'sym boundaries', 'byte boundaries');
   else
      legend('I', 'Q');
   end
   xlabel('samp idx');
   title('RX filtered & normalized IQ samples');

   %RX cross correlation with preamble. Plot power (i^2 + q^2).
   figure('position', [0 0 960 200]);
   plot(abs(rx_info.preamble_corr).^2); hold on;
   plot([1, length(rx_info.preamble_corr)], repmat(rx_info.corr_thresh, 1, 2), '--r');
   title('RX cross correlation with preamble (power)');
   legend('correlation power', 'threshold');

   %RX dphase (data signal only, training/preamble not included)
   if rx_info.dphase ~= -1
      figure('position', [0 0 960, 200]);
      plot(rx_info.dphase);
      %sym boundaries
      hold on;
      plot(1:samps_per_symb:1+rx_nbytes*8*samps_per_symb, zeros(1, rx_nbytes*8+1), '*r');
      %byte boundaries
      plot(1:samps_per_symb*8:1+rx_nbytes*8*samps_per_symb, zeros(1, rx_nbytes*8/8+1), '*g');
      legend('change in phase', 'sym boundaries', 'byte boundaries');
      title('RX dphase (data signal only)');
   end

   %RX dphase total per symbol (data signal only, training/preamble not included)
   if rx_info.dphase_sym ~= -1
      figure('position', [0 0 960, 200]);
      plot(rx_info.dphase_sym); hold on;
      hold on; plot(rx_info.dphase_sym, '.');
      %byte boundaries
      plot(1:8:length(rx_info.dphase_sym), zeros(1, length(rx_info.dphase_sym)/8), '*g');;
      legend('total change in phase', 'data points', 'byte boundaries (first bit)');
      title('RX total dphase per symbol (data signal only)');
   end

   %---------------------Print received data-------------------
   if rx_bits(1) ~= -1
      fprintf('Received           : ''%s''\n', bin2dec(rx_bits));
      if ~no_tx
         if isequal(rx_bits, tx_bits)
            fprintf('RX data matched TX data\n');
         else
            tx_bits_flat   = reshape(tx_bits(:,end:-1:1).', 1, []);
            rx_bits_flat   = reshape(rx_bits(:,end:-1:1).', 1, []);
            bit_mismatches = tx_bits_flat ~= rx_bits_flat;

            fprintf(2, 'RX data did not match TX data. %d bit mismatches (%.2f%%)\n', ...
                       sum(bit_mismatches), sum(bit_mismatches)/length(bit_mismatches)*100);

            figure('position', [0 0 960, 200]);
            fprintf('   First bit mismatch at index %d\n', find(bit_mismatches, 1));
            plot(bit_mismatches);
            title('bit mismatches (1=mismatch)');
            ylim([0 2]);
         end
%         if isequal(rx_bits(6:end-4,:), tx_bits(6:end-4,:))
%            fprintf('RX link payload data matched TX data\n');
%         else
%            fprintf(2, 'RX link payload did not match TX data\n');
%         end
      end
   end

end
