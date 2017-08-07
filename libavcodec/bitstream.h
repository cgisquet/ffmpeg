/*
 * Copyright (c) 2016 Alexandra Hájková
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

/**
 * @file
 * functions for reading bits from a buffer
 */

#ifndef AVCODEC_BITSTREAM_H
#define AVCODEC_BITSTREAM_H

#include <stdint.h>

#include "libavutil/common.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"

#include "mathops.h"
#include "vlc.h"

#ifndef BITSTREAM_BITS
# if HAVE_FAST_64BIT || defined(LONG_BITSTREAM_READER)
#   define BITSTREAM_BITS   64
# else
#   define BITSTREAM_BITS   32
# endif
#endif

#if BITSTREAM_BITS == 64
# define BITSTREAM_HBITS  32
typedef uint64_t cache_type;
# ifdef BITSTREAM_READER_LE
#   define AV_RALL  AV_RL64
#   define AV_RHALF AV_RL32
# else
#   define AV_RALL  AV_RB64
#   define AV_RHALF AV_RB32
# endif
#else
# define BITSTREAM_HBITS  16
typedef uint32_t cache_type;
# ifdef BITSTREAM_READER_LE
#   define AV_RALL  AV_RL32
#   define AV_RHALF AV_RL16
# else
#   define AV_RALL  AV_RB32
#   define AV_RHALF AV_RB16
# endif
#endif

typedef struct BitstreamContext {
    cache_type bits;    // stores bits read from the buffer
    const uint8_t *buffer, *buffer_end;
    const uint8_t *ptr; // position inside a buffer
    unsigned bits_left; // number of bits left in bits field
    unsigned size_in_bits;
} BitstreamContext;

static inline void refill_all(BitstreamContext *bc)
{
    if (bc->ptr >= bc->buffer_end)
        return;

    bc->bits       = AV_RALL(bc->ptr);
    bc->ptr       += BITSTREAM_BITS/8;
    bc->bits_left  = BITSTREAM_BITS;
}

static inline void refill_half(BitstreamContext *bc)
{
    if (bc->ptr >= bc->buffer_end)
        return;

#if BITSTREAM_BITS == 32
    if (bc->bits_left > 16) {
# ifdef BITSTREAM_READER_LE
        bc->bits |= (uint32_t)bc->ptr[0] << bc->bits_left;
# else
        bc->bits |= (uint32_t)bc->ptr[0] << (32 - bc->bits_left);
# endif
        bc->ptr++;
        bc->bits_left += 8;
        return;
    }
#endif

#ifdef BITSTREAM_READER_LE
    bc->bits      |= (cache_type)AV_RHALF(bc->ptr) << bc->bits_left;
#else
    bc->bits      |= (cache_type)AV_RHALF(bc->ptr) << (BITSTREAM_HBITS - bc->bits_left);
#endif
    bc->ptr       += BITSTREAM_HBITS/8;
    bc->bits_left += BITSTREAM_HBITS;
}

/* Initialize BitstreamContext. Input buffer must have an additional zero
 * padding of AV_INPUT_BUFFER_PADDING_SIZE bytes at the end. */
static inline int bitstream_init(BitstreamContext *bc, const uint8_t *buffer,
                                 unsigned bit_size)
{
    unsigned buffer_size;

    if (bit_size > INT_MAX - 7 || !buffer) {
        buffer        =
        bc->buffer    =
        bc->ptr       = NULL;
        bc->bits_left = 0;
        return AVERROR_INVALIDDATA;
    }

    buffer_size = (bit_size + 7) >> 3;

    bc->buffer       = buffer;
    bc->buffer_end   = buffer + buffer_size;
    bc->ptr          = bc->buffer;
    bc->size_in_bits = bit_size;
    bc->bits_left    = 0;
    bc->bits         = 0;

    refill_all(bc);

    return 0;
}

/* Initialize BitstreamContext with buffer size in bytes instead of bits. */
static inline int bitstream_init8(BitstreamContext *bc, const uint8_t *buffer,
                                  unsigned byte_size)
{
    if (byte_size > INT_MAX / 8)
        return AVERROR_INVALIDDATA;
    return bitstream_init(bc, buffer, byte_size * 8);
}

/* Return number of bits already read. */
static inline int bitstream_tell(const BitstreamContext *bc)
{
    return (bc->ptr - bc->buffer) * 8 - bc->bits_left;
}

/* Return buffer size in bits. */
static inline int bitstream_tell_size(const BitstreamContext *bc)
{
    return bc->size_in_bits;
}

/* Return the number of the bits left in a buffer. */
static inline int bitstream_bits_left(const BitstreamContext *bc)
{
    return (bc->buffer - bc->ptr) * 8 + bc->size_in_bits + bc->bits_left;
}

static inline cache_type get_val(BitstreamContext *bc, unsigned n)
{
#ifdef BITSTREAM_READER_LE
    cache_type ret = bc->bits & ((UINT##BITSTREAM_BITS##_C(1) << n) - 1);
    bc->bits >>= n;
#else
    cache_type ret = bc->bits >> (BITSTREAM_BITS - n);
    bc->bits <<= n;
#endif
    bc->bits_left -= n;

    return ret;
}

/* Return one bit from the buffer. */
static inline unsigned bitstream_read_bit(BitstreamContext *bc)
{
    if (!bc->bits_left)
        refill_all(bc);

    return get_val(bc, 1);
}

/* Return n bits from the buffer. n has to be in the 0-63 range. */
static inline uint64_t bitstream_read_63(BitstreamContext *bc, unsigned n)
{
    uint64_t ret = 0;
#ifdef BITSTREAM_READER_LE
    uint64_t left = 0;
#endif

    if (!n)
        return 0;

    if (n > bc->bits_left) {
        n -= bc->bits_left;
#ifdef BITSTREAM_READER_LE
        left = bc->bits_left;
#endif
        ret = get_val(bc, bc->bits_left);
        refill_all(bc);
    }

#ifdef BITSTREAM_READER_LE
    ret = get_val(bc, n) << left | ret;
#else
    ret = get_val(bc, n) | ret << n;
#endif

    return ret;
}

/* Return n bits from the buffer. n has to be in the 0-32 range. */
static inline uint32_t bitstream_read(BitstreamContext *bc, unsigned n)
{
    if (!n)
        return 0;

    if (n > bc->bits_left) {
        refill_half(bc);
        if (bc->bits_left < BITSTREAM_HBITS)
            bc->bits_left = n;
    }

    return get_val(bc, n);
}

/* Return n bits from the buffer as a signed integer.
 * n has to be in the 0-32 range. */
static inline int32_t bitstream_read_signed(BitstreamContext *bc, unsigned n)
{
    return sign_extend(bitstream_read(bc, n), n);
}

static inline unsigned show_val(const BitstreamContext *bc, unsigned n)
{
#ifdef BITSTREAM_READER_LE
    return bc->bits & ((UINT##BITSTREAM_BITS##_C(1) << n) - 1);
#else
    return bc->bits >> (BITSTREAM_BITS - n);
#endif
}

/* Return n bits from the buffer, but do not change the buffer state.
 * n has to be in the 0-32 range. */
static inline unsigned bitstream_peek(BitstreamContext *bc, unsigned n)
{
    if (n > bc->bits_left)
        refill_half(bc);

    return show_val(bc, n);
}

#if BITSTREAM_BITS == 32
/* For read of potentially more than 24bits
 * Intermediate between bitstream_read and bitstream_read_63 */
static inline uint32_t bitstream_read_mid(BitstreamContext *bc, unsigned n)
{
    unsigned ret = 0;

    if (n > bc->bits_left) {
        n -= bc->bits_left;
        ret = bc->bits >> (BITSTREAM_BITS - bc->bits_left);
        bc->bits = AV_RALL(bc->ptr);
        bc->ptr += BITSTREAM_BITS/8;
        bc->bits_left = BITSTREAM_BITS;
    }

    return get_val(bc, n) | ret << n;
}

/* For reads of 16 bits or less */
static inline uint32_t bitstream_peek_short(BitstreamContext *bc, unsigned n)
{
    if (!n)
        return 0;

    if (n > bc->bits_left && bc->ptr < bc->buffer_end) {
#ifdef BITSTREAM_READER_LE
        bc->bits      |= (cache_type)AV_RHALF(bc->ptr) << bc->bits_left;
#else
        bc->bits      |= (cache_type)AV_RHALF(bc->ptr) << (BITSTREAM_HBITS - bc->bits_left);
#endif
        bc->ptr       += BITSTREAM_HBITS/8;
        bc->bits_left += BITSTREAM_HBITS;
    }

    return show_val(bc, n);
}

static inline uint32_t bitstream_read_short(BitstreamContext *bc, unsigned n)
{
    if (!n)
        return 0;

    if (n > bc->bits_left) {
        if (bc->ptr < bc->buffer_end) {
#ifdef BITSTREAM_READER_LE
            bc->bits      |= (cache_type)AV_RHALF(bc->ptr) << bc->bits_left;
#else
            bc->bits      |= (cache_type)AV_RHALF(bc->ptr) << (BITSTREAM_HBITS - bc->bits_left);
#endif
            bc->ptr       += BITSTREAM_HBITS/8;
            bc->bits_left += BITSTREAM_HBITS;
        } else
            bc->bits_left = n;
    }

    return get_val(bc, n);
}
#else
# define bitstream_read_mid   bitstream_read
# define bitstream_read_short bitstream_read
# define bitstream_peek_short bitstream_peek
#endif

/* Return n bits from the buffer as a signed integer, but do not change the
 * buffer state. n has to be in the 0-32 range. */
static inline int bitstream_peek_signed(BitstreamContext *bc, unsigned n)
{
    return sign_extend(bitstream_peek(bc, n), n);
}

static inline void skip_remaining(BitstreamContext *bc, unsigned n)
{
#ifdef BITSTREAM_READER_LE
    bc->bits >>= n;
#else
    bc->bits <<= n;
#endif
    bc->bits_left -= n;
}

/* Skip n bits in the buffer. */
static inline void bitstream_skip(BitstreamContext *bc, unsigned n)
{
    if (n <= bc->bits_left)
        skip_remaining(bc, n);
    else {
        n -= bc->bits_left;
        skip_remaining(bc, bc->bits_left);
        if (n >= BITSTREAM_BITS) {
            unsigned skip = n / 8;

            n -= skip * 8;
            bc->ptr += skip;
        }
        refill_all(bc);
        if (n)
            skip_remaining(bc, n);
    }
}

/* Seek to the given bit position. */
static inline void bitstream_seek(BitstreamContext *bc, unsigned pos)
{
    bc->ptr       = bc->buffer;
    bc->bits      = 0;
    bc->bits_left = 0;

    bitstream_skip(bc, pos);
}

/* Skip bits to a byte boundary. */
static inline const uint8_t *bitstream_align(BitstreamContext *bc)
{
    unsigned n = -bitstream_tell(bc) & 7;
    if (n)
        bitstream_skip(bc, n);
    return bc->buffer + (bitstream_tell(bc) >> 3);
}

/* Read MPEG-1 dc-style VLC (sign bit + mantissa with no MSB).
 * If MSB not set it is negative. */
static inline int bitstream_read_xbits(BitstreamContext *bc, unsigned length)
{
    int sign;
    int32_t cache;

    if (length > bc->bits_left)
        refill_half(bc);
#if BITSTREAM_BITS == 32
    cache = bc->bits;
#elif defined(BITSTREAM_READER_LE)
    cache = bc->bits & 0xFFFFFFFF;
#else
    cache = bc->bits >> 32;
#endif
    sign = ~cache >> 31;
    skip_remaining(bc, length);

    return ((((uint32_t)(sign ^ cache)) >> (32 - length)) ^ sign) - sign;
}

/* Return the LUT element for the given bitstream configuration. */
static inline int set_idx(BitstreamContext *bc, int code, int *n, int *nb_bits,
                          VLC_TYPE (*table)[2])
{
    unsigned idx;

    *nb_bits = -*n;
    idx = bitstream_peek_short(bc, *nb_bits) + code;
    *n = table[idx][1];

    return table[idx][0];
}

/**
 * Parse a VLC code.
 * @param bits      is the number of bits which will be read at once, must be
 *                  identical to nb_bits in init_vlc()
 * @param max_depth is the number of times bits bits must be read to completely
 *                  read the longest VLC code
 *                  = (max_vlc_length + bits - 1) / bits
 * If the VLC code is invalid and max_depth = 1, then no bits will be removed.
 * If the VLC code is invalid and max_depth > 1, then the number of bits removed
 * is undefined. */
static inline int bitstream_read_vlc(BitstreamContext *bc, VLC_TYPE (*table)[2],
                                     int bits, int max_depth)
{
    int nb_bits;
    unsigned idx = bitstream_peek_short(bc, bits);
    int code = table[idx][0];
    int n    = table[idx][1];

    if (max_depth > 1 && n < 0) {
        skip_remaining(bc, bits);
        code = set_idx(bc, code, &n, &nb_bits, table);
        if (max_depth > 2 && n < 0) {
            skip_remaining(bc, bits);
            code = set_idx(bc, code, &n, &nb_bits, table);
        }
    }
    skip_remaining(bc, n);

    return code;
}

#define BITSTREAM_RL_VLC(level, run, bc, table, bits, max_depth) \
    do {                                                         \
        int n, nb_bits;                                          \
        unsigned index = bitstream_peek_short(bc, bits);         \
        level = table[index].level;                              \
        n     = table[index].len;                                \
                                                                 \
        if (max_depth > 1 && n < 0) {                            \
            skip_remaining(bc, bits);                            \
                                                                 \
            nb_bits = -n;                                        \
                                                                 \
            index = bitstream_peek_short(bc, nb_bits) + level;   \
            level = table[index].level;                          \
            n     = table[index].len;                            \
            if (max_depth > 2 && n < 0) {                        \
                skip_remaining(bc, nb_bits);                     \
                nb_bits = -n;                                    \
                                                                 \
                index = bitstream_peek_short(bc, nb_bits) + level;\
                level = table[index].level;                      \
                n     = table[index].len;                        \
            }                                                    \
        }                                                        \
        run = table[index].run;                                  \
        skip_remaining(bc, n);                                   \
    } while (0)

/* Return decoded truncated unary code for the values 0, 1, 2. */
static inline int bitstream_decode012(BitstreamContext *bc)
{
    if (!bitstream_read_bit(bc))
        return 0;
    else
        return bitstream_read_bit(bc) + 1;
}

/* Return decoded truncated unary code for the values 2, 1, 0. */
static inline int bitstream_decode210(BitstreamContext *bc)
{
    if (bitstream_read_bit(bc))
        return 0;
    else
        return 2 - bitstream_read_bit(bc);
}

/* Read sign bit and flip the sign of the provided value accordingly. */
static inline int bitstream_apply_sign(BitstreamContext *bc, int val)
{
    int sign = bitstream_read_signed(bc, 1);
    return (val ^ sign) - sign;
}

/* Unwind the cache so a refill_half can fill it again. */
static inline void bitstream_unwind(BitstreamContext *bc)
{
    int unwind = 4;
    int unwind_bits = unwind * 8;

    if (bc->bits_left < unwind_bits)
        return;

    bc->bits      >>= unwind_bits;
    bc->bits      <<= unwind_bits;
    bc->bits_left  -= unwind_bits;
    bc->ptr        -= unwind;
}

/* Unget up to 32 bits. */
static inline void bitstream_unget(BitstreamContext *bc, uint64_t value,
                                   size_t amount)
{
    size_t cache_size = sizeof(bc->bits) * 8;

    if (bc->bits_left + amount > cache_size)
        bitstream_unwind(bc);

    bc->bits       = (bc->bits >> amount) | (value << (cache_size - amount));
    bc->bits_left += amount;
}

#endif /* AVCODEC_BITSTREAM_H */
