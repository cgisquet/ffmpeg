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

#include "libavutil/avassert.h"
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
    //printf("%i 2-VLCs\n", i);
    ff_free_vlc(vlc);
    return ff_init_vlc_sparse(vlc, numbits, i, len, 1, 1,
                              bits, 2, 2, symbols, 2, 2, 0);
}

uint32_t *ff_huff_joint4same_gen(VLC *vlc, void *array, int num, int numbits,
                                 const uint32_t* bits0, const uint32_t* bits1,
                                 const uint8_t* l0, const uint8_t* l1,
                                 const uint16_t* lut0, const uint16_t* lut1)
{
    uint16_t *jsym  = array;
    uint16_t *jbits = jsym + (1 << numbits);
    uint8_t  *jlen  = (uint8_t *)(jbits + (1 << numbits));
    uint32_t *outlut = NULL;
    int i, t0;

    outlut = av_malloc((1 << numbits)*sizeof(uint32_t));
    if (!outlut) return NULL;

    for (i = t0 = 0; t0 < num; t0++) {
        int t1, idx0 = lut0 ? lut0[t0] : t0;
        int len0  = l0[idx0];
        int limit0 = numbits - len0;
        if (limit0 < 3 || !len0) {
            if (t0 < num/2 && t0 < num-t0-1) t0 = num-t0-2;
            continue;
        }
        for (t1 = 0; t1 < num; t1++) {
            int t2, idx1 = lut1 ? lut1[t1] : t1;
            int len1 = l1[idx1];
            int limit1 = limit0 - len1;
            if (limit1 < 2 || !len1) {
                if (t1 < num/2 && t1 < num-t1-1) t1 = num-t1-2;
                continue;
            }

            for (t2 = 0; t2 < num; t2++) {
                int t3, idx2 = lut0 ? lut0[t2] : t2;
                int len2 = l0[idx2];
                int limit2 = limit1 - len2;
                if (limit2 < 1 || !len2) {
                    if (t2 < num/2 && t2 < num-t2-1) t2 = num-t2-2;
                    continue;
                }

                for (t3 = 0; t3 < num; t3++) {
                    union {
                        uint32_t v32;
                        uint8_t  v8[4];
                    } a;

                    int code, idx3 = lut1 ? lut1[t3] : t3;
                    int len3 = l1[idx3];
                    if (limit2 < len3 || !len3) {
                        if (t3 < num/2 && t3 < num-t3-1) t3 = num-t3-2;
                        continue;
                    }
                    av_assert0(i < (1 << numbits));
                    code = (bits0[idx0] << len1) | bits1[idx1];
                    code = (code << len2) | bits0[idx2];
                    jbits[i] = (code << len3) | bits1[idx3];
                    jlen[i] = len0 + len1 + len2 + len3;
                    a.v8[0] = t0; a.v8[1] = t1;
                    a.v8[2] = t2; a.v8[3] = t3;
                    outlut[i] = a.v32;
                    jsym[i]  = i;
                    i++;
                }
            }
        }
    }

    //printf("%i 4-VLCs\n", i);
    ff_free_vlc(vlc);
    if (ff_init_vlc_sparse(vlc, numbits, i, jlen, 1, 1,
                           jbits, 2, 2, jsym, 2, 2, 0))
        return NULL;
    return outlut;
}
