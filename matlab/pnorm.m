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
%  est_powers: Power estimates computed for each input sample, using a weighted average
%              of that sample and the previous estimate
%  gains:      Gains applied to each input sample to produce the output sample
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
function [iq_out, est_powers, gains] = pnorm(iq_in, max_abs, alpha, min_gain, max_gain)

clamp_val_abs = 1.5*max_abs;
est_power     = 1; %initial power estimate
iq_out        = zeros(1, length(iq_in));
est_powers    = zeros(1, length(iq_in));
gains         = zeros(1, length(iq_in));

for i=1:length(iq_in)
   %calculate instantaneous power normalized to max sample value
   inst_power = (real(iq_in(i))^2 + imag(iq_in(i))^2)/max_abs^2;
   %calculate running power estimate using current estimate + instantaneous power
   est_power = alpha*est_power + (1-alpha)*inst_power;
   %ideal power is 1.0, so multiply by 1.0/sqrt(est_power) to try to reach 1.0
   gain = 1/sqrt(est_power);

   %check if exceeds bounds of min/max gain
   if gain < min_gain
      gain = min_gain;
   elseif gain > max_gain;
      gain = max_gain;
   end

   %apply gain
   iq_out_i = real(iq_in(i))*gain;
   iq_out_q = imag(iq_in(i))*gain;

   %clamp
   %I
   if iq_out_i > clamp_val_abs
      iq_out_i = clamp_val_abs;
   elseif iq_out_i < -clamp_val_abs
      iq_out_i = -clamp_val_abs;
   end
   %Q
   if iq_out_q > clamp_val_abs
      iq_out_q = clamp_val_abs;
   elseif iq_out_q < -clamp_val_abs
      iq_out_q = -clamp_val_abs;
   end
   iq_out(i) = complex(iq_out_i, iq_out_q);

   %blank impulse power
   out_power = (real(iq_out(i))^2 + imag(iq_out(i))^2)/max_abs^2;
   if out_power >= 10
      iq_out(i) = 0;
   end

   est_powers(i) = est_power;
   gains(i)      = gain;
end

end
