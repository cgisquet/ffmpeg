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
                      const uint16_t* lut0, const uint16_t* lut1,
                      int mode)
{
    uint16_t *symbols = array;
    uint16_t *bits    = symbols + (1 << numbits);
    uint8_t  *len     = (uint8_t *)(bits + (1 << numbits));
    int i = 0, t0, t1, min = 32, max = 0;

    for (t1 = 0; t1 < num; t1++) {
        int idx = lut1 ? lut1[t1] : t1;
        if (idx == 0xFFFF)
            break;
        if (len1[idx] >= 1 && len1[idx] < min)
            min = len1[idx];
    }
    if (mode == 2) {
        while (max < num/2) {
            for (t0 = 0; t0 < 2; t0++) {
                int t = t0 ? num-1-max : max;
                int idx = lut0 ? lut0[t] : t;
                if (idx == 0xFFFF || !len0[idx] || len0[idx]+min > numbits)
                    goto end;
            }
            max++;
        }
    } else {
        while (len0[max] && len0[max]+min < numbits && max < num)
            max++;
    }

end:
    if (mode == 2) {
    int j, k, l, m;
    for (j = 0; j < max; j++) {
        for (k = 0; k < 2; k++) {
        int idx0, l0, limit;
        if (k && num-1-j == j) break; // same symbol
        t0 = k ? num-1-j : j;
        idx0 = lut0 ? lut0[t0] : t0;
        if (idx0 == 0xFFFF)
            break;
        l0 = len0[idx0];
        if (!l0) break;
        if (l0+min > numbits)
            break;
        limit = numbits - l0;
        if ((sign_extend(t0, 8) & (num-1)) != t0)
            break;
        for (l = 0; l < max; l++) {
        for (m = 0; m < 2; m++) {
            int idx1, l1;
            if (m && num-1-l == l) break; // same symbol
            t1 = m ? num-1-l : l;
            idx1 = lut1 ? lut1[t1] : t1;
            if (idx1 == 0xFFFF)
                break;
            l1 = len1[idx1];
            if (l1 > limit)
                break;
            if ((sign_extend(t1, 8) & (num-1)) != t1)
                break;
            av_assert0(i < (1 << numbits));
            len[i]     = l0 + l1;
            bits[i]    = (bits0[idx0] << l1) + bits1[idx1];
            symbols[i] = (t0 << 8) + (t1 & 0xFF);
            i++;
        }
        }
        }
    }
    } else
    for (t0 = 0; t0 < max; t0++) {
        int idx0 = lut0 ? lut0[t0] : t0;
        int l0, limit;
        if (idx0 == 0xFFFF)
            continue;
        l0 = len0[idx0];
        if (l0+min > numbits || !l0) {
            if (mode) break;
            continue;
        }
        if ((sign_extend(t0, 8) & (num-1)) != t0)
            continue;
        limit = numbits - l0;
        for (t1 = 0; t1 < max; t1++) {
            int idx1 = lut1 ? lut1[t1] : t1;
            int l1;
            if (idx1 == 0xFFFF) {
                if (mode) break;
                continue;
            }
            l1 = len1[idx1];
            if (l1 > limit) {
                if (mode) break;
                continue;
            }
            if (!l1) continue;
            if ((sign_extend(t1, 8) & (num-1)) != t1)
                continue;
            av_assert0(i < (1 << numbits));
            len[i]     = l0 + l1;
            bits[i]    = (bits0[idx0] << l1) + bits1[idx1];
            symbols[i] = (t0 << 8) + (t1 & 0xFF);
            i++;
        }
    }
    av_log(NULL, AV_LOG_DEBUG, "Joint: %d codes\n", i);
    ff_free_vlc(vlc);
    return ff_init_vlc_sparse(vlc, numbits, i, len, 1, 1,
                              bits, 2, 2, symbols, 2, 2, 0);
}
