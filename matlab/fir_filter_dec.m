%-------------------------------------------------------------------------
%Apply decimating FIR filter
%Logically this applies a FIR filter h then downsamples by M. If model_lvl>0, this only
%computes the filter outputs that will not be thrown out by downsampling, making it more
%efficient than upfirdn().
%
%INPUTS
%  din:        input complex samples
%  h:          FIR filter coefficients to apply to din
%  M:          Decimation factor. Must be an integer.
%  init_phase: Initial phase/phase offset in output signal, 0-(M-1)
%  model_lvl:  Model level of computation. 0=high level, 1=mid level
%              model_lvl>0 only computes filter outputs that will not be thrown out by
%              downsampling, making it more efficient. All model levels should produce the
%              same output, only possible difference being miniscule floating point error
%OUTPUTS
%  dout:       output complex samples
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

function dout = fir_filter_dec(din, h, M, init_phase, model_lvl)

if model_lvl==0
   %--high level algorithm
   %inefficiently computes outputs that will be thrown out by downsampling
   dout = filter(h, 1, din);
   dout = dout(init_phase+1:M:end);
elseif model_lvl==1
   %--mid level algorithm
   %only computes outputs that will not be thrown out by downsampling
   dout    = zeros(1, ceil((length(din)-init_phase)/M) );
   %add leading zeros to din for initial filter ramp up
   din_pad = [zeros(1, length(h)-1), din];

   j = length(h) + init_phase; %top index (newest sample) into din_pad
   for i=1:length(dout) %index into dout
      %compute filter output sample
      dout(i) = sum( din_pad(j:-1:j-length(h)+1) .* h );
      j       = j + M;
   end
end

end
