/**
 * @file
 * @brief   Common data structures
 *
 * This file is part of the bladeRF-fsk project
 *
 * Copyright (C) 2016 Nuand LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <libbladeRF.h>

struct complex_sample {
    int16_t i;
    int16_t q;
};

struct radio_params {
    //TX
    bladerf_frequency   tx_freq;
    int                 tx_chan;         //Range: 0 to 1
    int                 tx_vga1_gain;    //Range: -35 to -4 dB
    int                 tx_vga2_gain;    //Range: 0 to 25 dB
    bool                tx_use_unified;  //Boolean
    bladerf_gain        tx_unified_gain; //Range: device dependent
    bool                tx_biastee;      //Boolean

    //RX
    bladerf_frequency   rx_freq;
    int                 rx_chan;         //Range 0 to 1
    bladerf_lna_gain    rx_lna_gain;     //Range: 0 to 6 dB
    int                 rx_vga1_gain;    //Range: 5 to 30 dB
    int                 rx_vga2_gain;    //Range: 0 to 30 dB
    bool                rx_use_unified;  //Boolean
    bladerf_gain        rx_unified_gain; //Range: device dependent
    bool                rx_biastee;      //Boolean
    bool                rx_agc;          //Boolean

    //common
    bladerf_sample_rate samplerate;      //Range: device dependent
};

#endif
