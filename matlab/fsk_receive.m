%-------------------------------------------------------------------------
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

function [bits, info] = fsk_receive(preamble_waveform, iq_signal, dec_factor, sps, h, ...
                                    num_bytes, scrambling_seed)
% FSK_RECEIVE Demodulate/receive data bits from an FSK baseband IQ signal.
% Correlates iq_signal with the given preamble waveform to find the start of data signal
%    [BITS, INFO] = fsk_receive(PREAMBLE_WAVEFORM, IQ_SIGNAL, DEC_FACTOR, SPS, H,
%                               NUM_BYTES, SCRAMBLING_SEED);
%
%    PREAMBLE_WAVEFORM preamble iq samples waveform to determine start of data in the the
%    iq_signal. Must already factor in decimation factor, so samples per symbol must equal
%    SPS/DEC_FACTOR in this waveform.
%
%    IQ_SIGNAL baseband CPFSK signal to receive
%
%    DEC_FACTOR amount to decimate by during filtering
%
%    SPS number of samples per symbol in waveform BEFORE decimation
%
%    H modulation index used on the transmitted signal
%
%    NUM_BYTES number of data bytes to demodulate from the iq_signal
%
%    SCRAMBLING_SEED the unsigned 64 bit seed used to create descrambling sequence. Must
%    match scrambling seed on the TX side for proper descrambling. Leave empty [] to
%    bypass descrambling.
%
%    BITS is an Nx8 matrix of received bits. Each bit element in
%    the matrix is a char which can either be '1' or '0'. Each row of the
%    matrix is a byte - containing 8 of these bit elements. Each byte is
%    received in order from smallest row index to largest row index. Each
%    bit in the byte is received in order from largest column index to
%    smallest column index (i.e. LSb first, MSb last))
%          Example: if bits is   | '0' '0' '0' '0' '1' '1' '1' '1'|
%                                | '0' '0' '1' '1' '1' '0' '1' '0'|
%                   then the received bit stream was:
%                                     "11110000 01011100"
%    Note: You can use dec2bin('test string', 8) to convert a string to
%    this bit matrix format, and bin2dec(bits) to convert a bit matrix to a
%    string
%
%    INFO a struct containing information to use for debug plots
%        INFO.iq_filt_norm    = filtered + normalized IQ signal
%        INFO.preamble_corr   = cross correlation of .iq_filt_norm and preamble waveform
%        INFO.dphase          = vector of phase changes in the .iq_filt_norm
%        INFO.sig_start_idx   = starting index of signal in .iq_filt_norm, detected by
%                               correlator. Start of first data symbol period.
%        INFO.dphase_sym      = vector of accumulated phase changes for each symbol in
%                               .iq_filt_norm. The sign determines whether it's a 0 or 1

info = struct('iq_filt_norm' , -1, ...
              'preamble_corr', -1, ...
              'corr_thresh'  , -1, ...
              'dphase'       , -1, ...
              'sig_start_idx', -1, ...
              'dphase_sym'   , -1);

corr_peak_thresh = 0.5625 * length(preamble_waveform)^2;
info.corr_thresh = corr_peak_thresh;

%Low pass filter the IQ signal with decimation
passband =       1/sps * h*2/pi;
stopband = 1/3 * 8/sps * h*2/pi;
if (stopband < 1)
   %Design and apply decimating filter
   b         = remez(31, [0 passband stopband 1], [1 1 0 0]).';
   iq_signal = fir_filter_dec(iq_signal, b, dec_factor, 0, 1);
else
   %downsample without filter (not ideal)
   iq_signal = iq_signal(1:dec_factor:end);
end
%update samples per symbol after decimation
sps = sps/dec_factor;

%Power normalize signal to roughly [-1, 1], following algorithm in C implementation
pnorm_alpha    = 0.95;
pnorm_min_gain = 0.1;
pnorm_max_gain = 50;
[iq_signal,est_power,~,pnorm_settle_time] = pnorm(iq_signal, 1, pnorm_alpha, ...
                                                  pnorm_min_gain, pnorm_max_gain);
info.iq_filt_norm = iq_signal;

%Correlate input IQ signal with known preamble waveform
corr               = xcorr(iq_signal, preamble_waveform);
%remove initial 0s (from implicit zero padding of preamble_waveform inside xcorr())
num_padded_zeros   = length(iq_signal) - length(preamble_waveform);
corr               = corr(num_padded_zeros+1:end);
info.preamble_corr = corr;

%Find peak in cross correlation power (i^2 + q^2)
[peak, sig_start_idx] = max(abs(corr).^2);
%Stop if peak does not exceed threshold (no preamble match)
if peak < corr_peak_thresh
   fprintf(2, '%s: No preamble match in signal\n', mfilename);
   bits = -1;
   return;
end

info.sig_start_idx = sig_start_idx;

%Estimate SNR using the power estimate in the middle of the frame (signal+noise, S+N) and
%the power estimate just after the frame has ended (noise, N). S = (S+N) - N.

%(S+N) power: Use est_power at frame data start. At this point, the training sequence and
%and preamble have come through, so est_power has had time to stabilize.
signoise_est_pwr = est_power(sig_start_idx);
%N power: Use est_power after end of frame
%frame end index: after all bytes and ramp down. +1 for setting init phase
frame_end_idx    = sig_start_idx + 1 + num_bytes*8*sps + sps;
%est_power noise index: add time for pnorm to settle, +10 for extra settling time
noise_est_idx    = frame_end_idx + pnorm_settle_time + 10;

if noise_est_idx > length(est_power)
   fprintf(2, ['%s: Not enough samples to allow time for noise power estimate to ' ...
               'fully settle. SNR estimate may not be accurate. Add more 0 samples to' ...
               ' end of signal to fix.\n'], mfilename);
   noise_est_idx = length(est_power);
end

training_len = 4; %training sequence length in bytes
if (training_len*8*sps + length(preamble_waveform)) < pnorm_settle_time
   fprintf(2, ['%s: Signal+noise power estimate will not be stable yet at start of ' ...
               'data just after training+preamble, because pnorm settle time = %d, but ' ...
               'training+preamble length = %d. SNR estimate may be innaccurate; suggest ' ...
               'editing the algorithm to use later samples after pnorm has settled\n'], ...
               mfilename, pnorm_settle_time, training_len*8*sps + length(preamble_waveform));
end

noise_est_pwr = est_power(noise_est_idx);
sig_est_pwr   = signoise_est_pwr - noise_est_pwr;
snr_est_db    = 10*log10(sig_est_pwr / noise_est_pwr);
fprintf('Note: RX post-filter SNR estimate = %.2f dB\n', snr_est_db);

%Demodulate bits from the IQ signal at the start index
[bits, info.dphase, info.dphase_sym] = ...
   fsk_demod(iq_signal(sig_start_idx:end), sps, num_bytes);

if ~isempty(scrambling_seed)
   %--Descramble data bits
   scrambling_seq = prng(scrambling_seed, size(bits, 1)).';
   %XOR data bytes with scrambling sequence
   bits           = dec2bin( bitxor(bin2dec(bits), scrambling_seq), 8 );
end

end
