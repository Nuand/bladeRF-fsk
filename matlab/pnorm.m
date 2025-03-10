%-------------------------------------------------------------------------
%Power normalize an input signal of range [-max_abs, max_abs] (representing [-1, 1]) using
%an IIR filter. Matches bladeRF-fsk C implementation. The output signal adjusts itself to
%try to maintain a consistent power of 1.0, reacting to power changes in the input.
%
%INPUTS
%  iq_in:      Input complex samples
%  max_abs:    Maximum absolute value of input samples, representing their range. A 12 bit
%              input would have max_abs=2048
%  alpha:      Weighting factor [0,1) representing how quickly the output responds to
%              changes in the input power. Higher=slower response, more stable output
%              power. Lower=faster response, less stable output power. Suggest 0.95.
%  min_gain:   Minimum gain that is allowed to be applied to input samples
%  max_gain:   Maximum gain that is allowed to be applied to input samples
%OUTPUTS
%  iq_out:     Output complex samples
%  est_power:  Power estimates computed for each input sample, using a weighted average
%              of that sample's power and the previous estimate
%  gain:       Gains applied to each input sample to produce the output sample
%  settle_time:Amount of time (in samples) for iq_out/est_power/gain to settle 99.9% of
%              the way after a change in the input power
%
% This file is part of the bladeRF-fsk project
%
% Copyright (C) 2025 Nuand LLC
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
function [iq_out, est_power, gain, settle_time] = pnorm(iq_in, max_abs, alpha, ...
                                                        min_gain, max_gain)

clamp_val_abs = 1.5*max_abs;
iq_out        = zeros(1, length(iq_in));
est_power     = zeros(1, length(iq_in));
gain          = zeros(1, length(iq_in));

%compute settling time
%equation derivation:
%p            = 1 - alpha^n; p = the output percentage, n = num samples to reach it
%alpha^n      = 1 - p
%n*log(alpha) = log(1-p)
%n            = log(1-p)/log(alpha)
settle_percent = .999;  %compute for 99.9%
settle_time    = ceil(log(1-settle_percent)/log(alpha));

%calculate instantaneous power normalized to max sample value
inst_power = (real(iq_in).^2 + imag(iq_in).^2)/max_abs^2;

%IIR filter to calculate running power estimate
%y[n] = alpha*y[n-1] + (1-alpha)*x[n]     (y[n]=est_power, x[n]=inst_power)
%b = [1-alpha] and a = [1, -alpha]
%want initial y[n-1] to be 1.0, so initial condition of alpha*y[n-1] = alpha
est_power = filter([1-alpha], [1, -alpha], inst_power, alpha);

%ideal power is 1.0, so multiply by 1.0/sqrt(est_power) to try to reach 1.0
gain = 1./sqrt(est_power);

%check if exceeds bounds of min/max gain
gain(gain < min_gain) = min_gain;
gain(gain > max_gain) = max_gain;

%apply gain
iq_out_i = real(iq_in).*gain;
iq_out_q = imag(iq_in).*gain;

%clamp
%I
iq_out_i(iq_out_i >  clamp_val_abs) =  clamp_val_abs;
iq_out_i(iq_out_i < -clamp_val_abs) = -clamp_val_abs;
%Q
iq_out_q(iq_out_q >  clamp_val_abs) =  clamp_val_abs;
iq_out_q(iq_out_q < -clamp_val_abs) = -clamp_val_abs;

iq_out = complex(iq_out_i, iq_out_q);

%blank impulse power
out_power               = (real(iq_out).^2 + imag(iq_out).^2)/max_abs^2;
iq_out(out_power >= 10) = 0;

end
