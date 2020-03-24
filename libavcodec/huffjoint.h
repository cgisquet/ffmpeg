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

// Beware decoder and encoder must have the same definition for GetBitContext
#define CACHED_BITSTREAM_READER 1

#include "get_bits.h"

/** Subset of GET_VLC for use in hand-roller VLC code */
#define VLC_INTERN(dst, table, gb, bits, max_depth)         \
    code = table[index][0];                                 \
    n    = table[index][1];                                 \
    if (max_depth > 1 && n < 0) {                           \
        skip_remaining(gb, bits);                           \
        code = set_idx(gb, code, &n, &nb_bits, table);      \
        if (max_depth > 2 && n < 0) {                       \
            skip_remaining(gb, nb_bits);                    \
            code = set_idx(gb, code, &n, &nb_bits, table);  \
        }                                                   \
    }                                                       \
    dst = code;                                             \
    skip_remaining(gb, n)

/**
 * Try to read into dst0 and dst1, using operation OP, 2 symbols
 * by using joint VLC dtable, otherwise read them using table1 and
 * table2 respectively.
 */
#define GET_VLC_DUAL(dst0, dst1, gb, dtable, table1, table2,        \
                     bits, max_depth, OP)                           \
    do {                                                            \
        unsigned int index = show_bits(gb, bits);                   \
        int          code, n = dtable[index][1];                    \
                                                                    \
        if (n<=0) {                                                 \
            int nb_bits;                                            \
            VLC_INTERN(dst0, table1, gb, bits, max_depth);          \
                                                                    \
            index = show_bits(gb, bits);                            \
            VLC_INTERN(dst1, table2, gb, bits, max_depth);          \
        } else {                                                    \
            code = dtable[index][0];                                \
            OP(dst0, dst1, code);                                   \
            skip_remaining(gb, n);                                  \
        }                                                           \
    } while (0)

#define GET_VLC_ITER(dst, off, gb, Dtable, table, bits, max_depth, OP)\
    do {                                                            \
        unsigned int index = show_bits(gb, bits);                   \
        int          code, n = Dtable[index][1];                    \
                                                                    \
        if (n<=0) {                                                 \
            int nb_bits;                                            \
            VLC_INTERN(dst[off], table, gb, bits, max_depth);       \
            off++;                                                  \
        } else {                                                    \
            code = Dtable[index][0];                                \
            OP(dst[off+0], dst[off+1], code);                       \
            off += 2;                                               \
            skip_remaining(gb, n);                                  \
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
 * A value of 0xFFFF in the LUT is assumed to be an unpopulated entry
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
 * @param mode     What loop conditions to stop on
 *                 0: continue other codes if current too long
 *                 1: stop
 *                 2: wrap (consider code and max-code) and stop
 */
int ff_huff_joint_gen(VLC *vlc, void *array, int num, int numbits,
                      const uint32_t* bits0, const uint32_t* bits1,
                      const uint8_t* len0, const uint8_t* len1,
                      const uint16_t* lut0, const uint16_t* lut1,
                      int mode);

#define VLC_MULTI_MAX_SYMBOLS  4

typedef struct VLC_MULTI {
    uint8_t val[6];
    int8_t len; // -31,32
    uint8_t num;
} VLC_MULTI;

#define WRITE_MULTI8b(dst, off, entry)                              \
        if (entry.num) {                                            \
            AV_COPY32(dst+off, &(entry));                           \
            off += entry.num;                                       \
        } else {

#define WRITE_MULTI16b(dst, off, entry)                             \
        if (entry.num) {                                            \
            AV_COPY64(dst+off, &(entry));                           \
            off += entry.num;                                       \
        } else {

#define GET_VLC_MULTI(dst, off, gb, Jtable, table, bits, max_depth, OP) \
    do {                                                            \
        unsigned int index = show_bits(gb, bits);                   \
        int nb_bits, code, n = Jtable[index].len;                   \
        OP(dst, off, Jtable[index])                                 \
            code = AV_RN16(Jtable[index].val);                      \
            skip_remaining(gb, bits);                               \
            code = set_idx(gb, code, &n, &nb_bits, table);          \
            if (max_depth > 2 && n < 0) {                           \
                skip_remaining(gb, bits);                           \
                code = set_idx(gb, code, &n, &nb_bits, table);      \
            }                                                       \
            dst[off++] = code;                                      \
        }                                                           \
        skip_remaining(gb, n);                                      \
    } while (0)

int ff_huff_multi_gen(VLC_MULTI* table, const VLC *single,
                      int num, int numbits,
                      const uint32_t* bits, const uint8_t* len,
                      const uint16_t* lut, int mode);

#endif /* AVCODEC_HUFF_JOINT_H */
