%-------------------------------------------------------------------------
%Pseudorandom number generator
%Generates a sequence of pseudorandom bytes matching the algorithm in prng.c. Algorithm
%applies a 64-bit multiplication, which requires multiplication by parts to do properly in
%matlab/octave without inducing matlab's implicit saturation
%
%INPUTS
%  seed:       64 bit unsigned seed for generator
%  len:        (length) number of bytes to generate
%OUTPUTS
%  bytes_out:  pseudorandom bytes
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
function bytes_out = prng(seed, len)

assert(seed != 0);

state     = uint64(seed);
bytes_out = zeros(1, len);
i         = 1;    %index into bytes_out

for word=0:ceil(len/8)-1   %for each 8-byte word
   if i == 41
      fprintf('');
   end

   %update/scramble 8-byte state
   state = bitxor( state, bitshift(state, -12) );
   state = bitxor( state, bitshift(state,  25) );
   state = bitxor( state, bitshift(state, -27) );
   state = mult64( state, uint64(2685821657736338717) );   % * 0x2545f4914f6cdd1d

   %output the 8 bytes from state
   for b=0:7
      bytes_out(i) = bitand( bitshift(state, -b*8), hex2dec('FF') );
      i = i + 1;
      if i > len
         break;
      end
   end
end

%helper function to multiply 64 bit values in matlab without inducing saturation
%64 bit out result may wrap, but no saturation will happen
function out = mult64(a, b)
   a = uint64(a);
   b = uint64(b);

   %Split a
   lo1 = bitand  (a, uint64(2^32 - 1));
   hi1 = bitshift(a, -32);

   %Split b
   lo2 = bitand  (b, uint64(2^32 - 1));
   hi2 = bitshift(b, -32);

   %--Perform multiplication in parts, adding the parts together with shifts to compute
   %the final result

   %--Compute the multiplication parts
   %Each number is 32 bits so the multiplication cannot exceed 64 bits
   %hi1*hi2 is not needed, because that would get shifted up by 64, beyond the range we
   %need, not affecting bottom 64 bits
   lolo = lo1 * lo2;
   mid1 = lo1 * hi2;
   mid2 = hi1 * lo2;

   %--Add the parts together with appropriate shifts
   out = bitand(lolo, uint64(2^64 - 1));        %add in lolo
   %Add +32 shifted mid1
   %Due to the shift, we are adding only the top half of out with the bottom half of mid1,
   %bottom half of out is unchanged.
   %compute the top half of out and OR with the unchanged bottom half of out
   top_half = bitand(bitshift(out, -32) + bitand(mid1, 2^32-1), 2^32-1);
   out      = bitor (bitand(out, 2^32-1), bitshift(top_half, 32));
   %Add +32 shifted mid2 (same operation as with mid1)
   top_half = bitand(bitshift(out, -32) + bitand(mid2, 2^32-1), 2^32-1);
   out      = bitor (bitand(out, 2^32-1), bitshift(top_half, 32));
end

end
