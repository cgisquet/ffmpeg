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

#ifndef AVCODEC_HUFF_JOINT_H
#define AVCODEC_HUFF_JOINT_H

#include "get_bits.h"

/** Subset of GET_VLC for use in hand-roller VLC code */
#define VLC_INTERN(dst, table, gb, name, bits, max_depth)   \
    code = table[index][0];                                 \
    n    = table[index][1];                                 \
    if (max_depth > 1 && n < 0) {                           \
        LAST_SKIP_BITS(name, gb, bits);                     \
        UPDATE_CACHE(name, gb);                             \
                                                            \
        nb_bits = -n;                                       \
        index   = SHOW_UBITS(name, gb, nb_bits) + code;     \
        code    = table[index][0];                          \
        n       = table[index][1];                          \
        if (max_depth > 2 && n < 0) {                       \
            LAST_SKIP_BITS(name, gb, nb_bits);              \
            UPDATE_CACHE(name, gb);                         \
                                                            \
            nb_bits = -n;                                   \
            index   = SHOW_UBITS(name, gb, nb_bits) + code; \
            code    = table[index][0];                      \
            n       = table[index][1];                      \
        }                                                   \
    }                                                       \
    dst = code;                                             \
    LAST_SKIP_BITS(name, gb, n)

/**
 * Try to read into dst0 and dst1, using operation OP, 2 symbols
 * by using joint VLC dtable, otherwise read them using table1 and
 * table2 respectively.
 */
#define GET_VLC_DUAL(dst0, dst1, name, gb, dtable, table1, table2,  \
                     bits, max_depth, OP)                           \
    do {                                                            \
        unsigned int index = SHOW_UBITS(name, gb, bits);            \
        int          code, n = dtable[index][1];                    \
                                                                    \
        if (n<=0) {                                                 \
            int nb_bits;                                            \
            VLC_INTERN(dst0, table1, gb, name, bits, max_depth);    \
                                                                    \
            UPDATE_CACHE(re, gb);                                   \
            index = SHOW_UBITS(name, gb, bits);                     \
            VLC_INTERN(dst1, table2, gb, name, bits, max_depth);    \
        } else {                                                    \
            code = dtable[index][0];                                \
            OP(dst0, dst1, code);                                   \
            LAST_SKIP_BITS(name, gb, n);                            \
        }                                                           \
    } while (0)

/** Default operation for reading 8 bits elements */
#define OP8bits(dst0, dst1, code) dst0 = code>>8; dst1 = code

/** Default operation for reading <= 14 bits elements */
#define OP14bits(dst0, dst1, code) dst0 = code>>8; dst1 = sign_extend(code, 8)

/**
 * Generate an array suitable for ff_huff_joint_table_generate
 * @param numbits Number of bits of the VLC lut
 * @see ff_init_vlc_sparse second parameter
 */
void* ff_huff_joint_alloc(int numbits);

/**
 * Generate a table suitable for decoding of 2 VLCs at a time
 * @param vlc      The VLC table
 * @param array    The array allocated by ff_huff_joint_alloc
 * @param num      Maximum symbol value
 * @param numbits  Value passed to ff_huff_joint_alloc
 * @param bits0    Codewords for 1st element of pair
 * @param bits1    Codewords for 2nd element of pair
 * @param len0     Codeword lengths for 1st element of pair
 * @param len1     Codeword lengths for 2nd element of pair
 * @param lut0     LUT (NULL for same) for value to index of 1st element
 * @param lut1     LUT (NULL for same) for value to index of 2nd element
 */
int ff_huff_joint_gen(VLC *vlc, void *array, int num, int numbits,
                      const uint32_t* bits0, const uint32_t* bits1,
                      const uint8_t* len0, const uint8_t* len1,
                      const uint16_t* lut0, const uint16_t* lut1);

#endif /* AVCODEC_HUFF_JOINT_H */
