/*
 * Joint huffman tables handling (for decoder)
 *
 * Copyright (c) 2002-2014 Michael Niedermayer <michaelni@gmx.at>
 * Copyright (c) 2016 Christophe Gisquet
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "huffjoint.h"

void* ff_huff_joint_alloc(int numbits)
{
    return av_mallocz(5 << numbits);
}

int ff_huff_joint_gen(VLC *vlc, void *array, int num, int numbits,
                      const uint32_t* bits0, const uint32_t* bits1,
                      const uint8_t* len0, const uint8_t* len1,
                      const uint16_t* lut0, const uint16_t* lut1)
{
    uint16_t *symbols = array;
    uint16_t *bits    = symbols + (1 << numbits);
    uint8_t  *len     = (uint8_t *)(bits + (1 << numbits));
    int i, t0, t1;

    for (i = t0 = 0; t0 < num; t0++) {
        int idx0 = lut0 ? lut0[t0] : t0;
        int l0   = len0[idx0];
        int limit = numbits - l0;
        if (limit <= 0 || !l0)
            continue;
        if ((sign_extend(t0, 8) & (num-1)) != t0)
            continue;
        for (t1 = 0; t1 < num; t1++) {
            int idx1 = lut1 ? lut1[t1] : t1;
            int l1   = len1[idx1];
            if (l1 > limit || !l1)
                continue;
            if ((sign_extend(t1, 8) & (num-1)) != t1)
                continue;
            av_assert0(i < (1 << numbits));
            len[i]     = l0 + l1;
            bits[i]    = (bits0[idx0] << l1) + bits1[idx1];
            symbols[i] = (t0 << 8) + (t1 & 0xFF);
                i++;
        }
    }
    ff_free_vlc(vlc);
    return ff_init_vlc_sparse(vlc, numbits, i, len, 1, 1,
                              bits, 2, 2, symbols, 2, 2, 0);
}
