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

#define MAX_JOINT_CODES   1500

int ff_huff_multi_gen(VLC_MULTI* table, const VLC *single,
                      int num, int numbits,
                      const uint32_t* bits, const uint8_t* len,
                      const uint16_t* lut, int mode)
{
    VLC vlc2, vlc3;
    uint16_t bits2[MAX_JOINT_CODES], bits3[MAX_JOINT_CODES];
    uint8_t  len2[MAX_JOINT_CODES], len3[MAX_JOINT_CODES];
    uint8_t  val2[2*MAX_JOINT_CODES], val3[3*MAX_JOINT_CODES];
    int i = 0, i3 = 0, t0, t1, t2, min = 32, max = 0;

    for (int j = 0; j < num; j++) {
        int idx = lut ? lut[j] : j;
        if (idx == 0xFFFF)
            continue;
        if (len[idx] >= 1 && len[idx] < min)
            min = len[idx];
    }
    while (max < num) {
        int idx = lut ? lut[max] : max;
        if (idx == 0xFFFF || len[idx]+min > numbits)
            break;
        max++;
    }

    if (mode == 2) {
        int j, k, l, m, n, o;
        for (j = 0; j < max; j++) {
            for (k = 0; k < 2; k++) {
                int idx0, l0, limit;
                if (k && num-1-j == j) break; // same symbol
                t0 = k ? num-1-j : j;
                idx0 = lut ? lut[t0] : t0;
                if (idx0 == 0xFFFF)
                    break;
                l0 = len[idx0];
                if (l0+min > numbits || !l0)
                    break;
                limit = numbits - l0;
                if ((sign_extend(t0, 8) & (num-1)) != t0)
                    continue;
                for (l = 0; l < max; l++) {
                    for (m = 0; m < 2; m++) {
                        int idx1, l1, limit2;
                        if (m && num-1-l == l) break; // same symbol
                        t1 = m ? num-1-l : l;
                        idx1 = lut ? lut[t1] : t1;
                        if (idx1 == 0xFFFF)
                            break;
                        l1 = len[idx1];
                        if (l1 > limit)
                            break;
                        if ((sign_extend(t1, 8) & (num-1)) != t1)
                            continue;
                        len2 [i]    = l0 + l1;
                        bits2[i]    = (bits[idx0] << l1) + bits[idx1];
                        val2[2*i+0] = t0&0xFF;
                        val2[2*i+1] = t1&0xFF;
                        i++;

                        limit2 = numbits - l0 - l1;
                        if (limit >= min && i3 < MAX_JOINT_CODES)
                        for (n = 0; n < max; n++) {
                            for (o = 0; o < 2; o++) {
                                int idx2, l2;
                                if (o && num-1-n == n) break; // same symbol
                                t2 = o ? num-1-n : n;
                                idx2 = lut ? lut[t2] : t2;
                                if (idx2 == 0xFFFF)
                                    break;
                                l2 = len[idx2];
                                if (l2 > limit2 || !l2)
                                    break;
                                if ((sign_extend(t2, 8) & (num-1)) != t2)
                                    continue;
                                len3 [i3]    = l0 + l1 + l2;
                                bits3[i3]    = (bits[idx0] << (l1+l2)) + (bits[idx1]<<l2) + bits[idx2];
                                val3[3*i3+0] = t0&0xFF;
                                val3[3*i3+1] = t1&0xFF;
                                val3[3*i3+2] = t2&0xFF;
                                i3++;
                                if (i3 == MAX_JOINT_CODES)
                                    goto next2;
                            }
                        }

next2:
                        if (i == MAX_JOINT_CODES)
                            goto end;
                    }
                }
            }
        }
    } else {
        for (t0 = 0; t0 < max; t0++) {
            int idx0 = lut ? lut[t0] : t0;
            int l0, limit;
            if (idx0 == 0xFFFF)
                continue;
            l0 = len[idx0];
            if (l0 && l0 < min) min = l0;
            if (l0+min > numbits) {
                if (mode) break;
                continue;
            }
            limit = numbits - l0;
            if (!l0) continue;
            if ((sign_extend(t0, 8) & (num-1)) != t0)
                continue;
            for (t1 = 0; t1 < max; t1++) {
                int idx1 = lut ? lut[t1] : t1;
                int limit2, l1;
                if (idx1 == 0xFFFF) {
                    if (mode) break;
                    continue;
                }
                l1 = len[idx1];
                if (l1 > limit) {
                    if (mode) break;
                    continue;
                }
                if (!l1) continue;
                if ((sign_extend(t1, 8) & (num-1)) != t1)
                    continue;
                av_assert0(i < (1 << numbits));
                len2 [i]    = l0 + l1;
                bits2[i]    = (bits[idx0] << l1) + bits[idx1];
                val2[2*i+0] = t0;
                val2[2*i+1] = t1;
                i++;

                limit2 = limit - l1;
                if (limit2+min < numbits && i3 < MAX_JOINT_CODES)
                for (t2 = 0; t2 < max; t2++) {
                    int idx2 = lut ? lut[t2] : t2;
                    int l2;
                    if (idx2 == 0xFFFF) {
                        if (mode) break;
                        continue;
                    }
                    l2 = len[idx2];
                    if (l2 > limit2) {
                        if (mode) break;
                        continue;
                    }
                    if (!l2) continue;
                    if ((sign_extend(t2, 8) & (num-1)) != t2)
                        break;
                    len3 [i3]    = l0 + l1 + l2;
                    bits3[i3]    = (bits[idx0] << (l1+l2)) + (bits[idx1]<<l2) + bits[idx2];
                    val3[3*i3+0] = t0;
                    val3[3*i3+1] = t1;
                    val3[3*i3+2] = t2;
                    i3++;
                    if (i3 == MAX_JOINT_CODES)
                        goto next0;
                }

next0:
                if (i == MAX_JOINT_CODES)
                    goto end;
            }
        }
    }

end:
    av_log(NULL, AV_LOG_DEBUG, "Joint: %d/%d codes min=%ubits max=%u\n", i, i3, min, max);
    if (ff_init_vlc_sparse(&vlc2, numbits, i, len2, 1, 1,
                           bits2, 2, 2, NULL, 0, 0, 0))
        return AVERROR_INVALIDDATA;
    if (ff_init_vlc_sparse(&vlc3, numbits, i3, len3, 1, 1,
                           bits3, 2, 2, NULL, 0, 0, 0))
        return AVERROR_INVALIDDATA;

    for (i = 0; i < 1<<numbits; i++) {
        int idx;
        if (vlc3.table[i][1] > 0) {
            idx = vlc3.table[i][0];
            table[i].len = vlc3.table[i][1];
            table[i].num = 3;
            table[i].val[0] = val3[3*idx+0]; table[i].val[1] = val3[3*idx+1]; table[i].val[2] = val3[3*idx+2];
        } else if (vlc2.table[i][1] > 0) {
            idx = vlc2.table[i][0];
            table[i].len = vlc2.table[i][1];
            table[i].num = 2;
            table[i].val[0] = val2[2*idx+0]; table[i].val[1] = val2[2*idx+1];
        } else {
            table[i].len = single->table[i][1];
            table[i].num = single->table[i][1] > 0 ? 1 : 0;
            AV_WN16(table[i].val, single->table[i][0]);
        }
    }

    ff_free_vlc(&vlc2);
    ff_free_vlc(&vlc3);

    return 0;
}