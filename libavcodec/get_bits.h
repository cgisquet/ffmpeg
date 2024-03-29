/*
 * Copyright (c) 2004 Michael Niedermayer <michaelni@gmx.at>
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
 * bitstream reader API header.
 */

#ifndef AVCODEC_GET_BITS_H
#define AVCODEC_GET_BITS_H

#include <stdint.h>

#include "libavutil/common.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"
#include "libavutil/avassert.h"
#include "avcodec.h"
#include "mathops.h"
#include "vlc.h"

/*
 * Safe bitstream reading:
 * optionally, the get_bits API can check to ensure that we
 * don't read past input buffer boundaries. This is protected
 * with CONFIG_SAFE_BITSTREAM_READER at the global level, and
 * then below that with UNCHECKED_BITSTREAM_READER at the per-
 * decoder level. This means that decoders that check internally
 * can "#define UNCHECKED_BITSTREAM_READER 1" to disable
 * overread checks.
 * Boundary checking causes a minor performance penalty so for
 * applications that won't want/need this, it can be disabled
 * globally using "#define CONFIG_SAFE_BITSTREAM_READER 0".
 */
#ifndef UNCHECKED_BITSTREAM_READER
#define UNCHECKED_BITSTREAM_READER !CONFIG_SAFE_BITSTREAM_READER
#endif

#ifndef CACHED_BITSTREAM_READER
#define CACHED_BITSTREAM_READER 0
#endif

#if CACHED_BITSTREAM_READER

# ifndef BITSTREAM_BITS
#   if HAVE_FAST_64BIT || defined(LONG_BITSTREAM_READER)
#     define BITSTREAM_BITS   64
#   else
#     define BITSTREAM_BITS   32
#   endif
# endif

# if BITSTREAM_BITS == 64
#   define BITSTREAM_HBITS  32
typedef uint64_t cache_type;
#   define AV_RB_ALL  AV_RB64
#   define AV_RL_ALL  AV_RL64
#   define AV_RB_HALF AV_RB32
#   define AV_RL_HALF AV_RL32
#   define CACHE_TYPE(a) UINT64_C(a)
# else
#   define BITSTREAM_HBITS  16
typedef uint32_t cache_type;
#   define AV_RB_ALL  AV_RB32
#   define AV_RL_ALL  AV_RL32
#   define AV_RB_HALF AV_RB16
#   define AV_RL_HALF AV_RL16
#   define CACHE_TYPE(a) UINT32_C(a)
#endif

#endif

typedef struct GetBitContext {
    const uint8_t *buffer, *buffer_end;
#if CACHED_BITSTREAM_READER
    const uint8_t *ptr;
    cache_type cache;
    unsigned bits_left;
#else
    int index; // Cached version advances ptr instead
#endif
    int size_in_bits;
    int size_in_bits_plus8;
} GetBitContext;

static inline unsigned int get_bits(GetBitContext *s, int n);
static inline void skip_bits(GetBitContext *s, int n);
static inline unsigned int show_bits(GetBitContext *s, int n);

/* Bitstream reader API docs:
 * name
 *   arbitrary name which is used as prefix for the internal variables
 *
 * gb
 *   getbitcontext
 *
 * OPEN_READER(name, gb)
 *   load gb into local variables
 *
 * CLOSE_READER(name, gb)
 *   store local vars in gb
 *
 * UPDATE_CACHE(name, gb)
 *   Refill the internal cache from the bitstream.
 *   After this call at least MIN_CACHE_BITS will be available.
 *
 * GET_CACHE(name, gb)
 *   Will output the contents of the internal cache,
 *   next bit is MSB of 32 or 64 bits (FIXME 64 bits).
 *
 * SHOW_UBITS(name, gb, num)
 *   Will return the next num bits.
 *
 * SHOW_SBITS(name, gb, num)
 *   Will return the next num bits and do sign extension.
 *
 * SKIP_BITS(name, gb, num)
 *   Will skip over the next num bits.
 *   Note, this is equivalent to SKIP_CACHE; SKIP_COUNTER.
 *
 * SKIP_CACHE(name, gb, num)
 *   Will remove the next num bits from the cache (note SKIP_COUNTER
 *   MUST be called before UPDATE_CACHE / CLOSE_READER).
 *
 * SKIP_COUNTER(name, gb, num)
 *   Will increment the internal bit counter (see SKIP_CACHE & SKIP_BITS).
 *
 * LAST_SKIP_BITS(name, gb, num)
 *   Like SKIP_BITS, to be used if next call is UPDATE_CACHE or CLOSE_READER.
 *
 * BITS_LEFT(name, gb)
 *   Return the number of bits left
 *
 * For examples see get_bits, show_bits, skip_bits, get_vlc.
 */

#if CACHED_BITSTREAM_READER
#   define MIN_CACHE_BITS (BITSTREAM_BITS-7)
#elif defined LONG_BITSTREAM_READER
#   define MIN_CACHE_BITS 32
#else
#   define MIN_CACHE_BITS 25
#endif

#if !CACHED_BITSTREAM_READER

#define OPEN_READER_NOSIZE(name, gb)            \
    unsigned int name ## _index = (gb)->index;  \
    unsigned int av_unused name ## _cache

#if UNCHECKED_BITSTREAM_READER
#define OPEN_READER(name, gb) OPEN_READER_NOSIZE(name, gb)

#define BITS_AVAILABLE(name, gb) 1
#else
#define OPEN_READER(name, gb)                   \
    OPEN_READER_NOSIZE(name, gb);               \
    unsigned int name ## _size_plus8 = (gb)->size_in_bits_plus8

#define BITS_AVAILABLE(name, gb) name ## _index < name ## _size_plus8
#endif

#define CLOSE_READER(name, gb) (gb)->index = name ## _index

# ifdef LONG_BITSTREAM_READER

# define UPDATE_CACHE_LE(name, gb) name ## _cache = \
      AV_RL64((gb)->buffer + (name ## _index >> 3)) >> (name ## _index & 7)

# define UPDATE_CACHE_BE(name, gb) name ## _cache = \
      AV_RB64((gb)->buffer + (name ## _index >> 3)) >> (32 - (name ## _index & 7))

#else

# define UPDATE_CACHE_LE(name, gb) name ## _cache = \
      AV_RL32((gb)->buffer + (name ## _index >> 3)) >> (name ## _index & 7)

# define UPDATE_CACHE_BE(name, gb) name ## _cache = \
      AV_RB32((gb)->buffer + (name ## _index >> 3)) << (name ## _index & 7)

#endif


#ifdef BITSTREAM_READER_LE

# define UPDATE_CACHE(name, gb) UPDATE_CACHE_LE(name, gb)

# define SKIP_CACHE(name, gb, num) name ## _cache >>= (num)

#else

# define UPDATE_CACHE(name, gb) UPDATE_CACHE_BE(name, gb)

# define SKIP_CACHE(name, gb, num) name ## _cache <<= (num)

#endif

#if UNCHECKED_BITSTREAM_READER
#   define SKIP_COUNTER(name, gb, num) name ## _index += (num)
#else
#   define SKIP_COUNTER(name, gb, num) \
    name ## _index = FFMIN(name ## _size_plus8, name ## _index + (num))
#endif

#define BITS_LEFT(name, gb) ((int)((gb)->size_in_bits - name ## _index))

#define SKIP_BITS(name, gb, num)                \
    do {                                        \
        SKIP_CACHE(name, gb, num);              \
        SKIP_COUNTER(name, gb, num);            \
    } while (0)

#define LAST_SKIP_BITS(name, gb, num) SKIP_COUNTER(name, gb, num)

#define SHOW_UBITS_LE(name, gb, num) zero_extend(name ## _cache, num)
#define SHOW_SBITS_LE(name, gb, num) sign_extend(name ## _cache, num)

#define SHOW_UBITS_BE(name, gb, num) NEG_USR32(name ## _cache, num)
#define SHOW_SBITS_BE(name, gb, num) NEG_SSR32(name ## _cache, num)

#ifdef BITSTREAM_READER_LE
#   define SHOW_UBITS(name, gb, num) SHOW_UBITS_LE(name, gb, num)
#   define SHOW_SBITS(name, gb, num) SHOW_SBITS_LE(name, gb, num)
#else
#   define SHOW_UBITS(name, gb, num) SHOW_UBITS_BE(name, gb, num)
#   define SHOW_SBITS(name, gb, num) SHOW_SBITS_BE(name, gb, num)
#endif

#define GET_CACHE(name, gb) ((uint32_t) name ## _cache)

#endif

static inline int get_bits_count(const GetBitContext *s)
{
#if CACHED_BITSTREAM_READER
    return 8*(s->ptr - s->buffer) - s->bits_left;
#else
    return s->index;
#endif
}

#if CACHED_BITSTREAM_READER
// See variant 4 in the following article:
// https://fgiesen.wordpress.com/2018/02/20/reading-bits-in-far-too-many-ways-part-2/
static inline void refill_gb(GetBitContext *s, int is_le)
{
#if !UNCHECKED_BITSTREAM_READER
    if (s->ptr >= s->buffer_end)
        return;
#endif

    if (is_le)
    s->cache     |= (cache_type)AV_RL_ALL(s->ptr) << s->bits_left;
    else
    s->cache     |= (cache_type)AV_RB_ALL(s->ptr) >> s->bits_left;
    s->ptr       += (BITSTREAM_BITS-1 - s->bits_left) >> 3;
    s->bits_left |= BITSTREAM_BITS-8;
}

static inline cache_type get_val(GetBitContext *s, unsigned n, int is_le)
{
    cache_type ret;
    av_assert2(n>0 && n<=63);
    if (is_le) {
        ret = s->cache & ((CACHE_TYPE(1) << n) - 1);
        s->cache >>= n;
    } else {
        ret = s->cache >> (BITSTREAM_BITS - n);
        s->cache <<= n;
    }
    s->bits_left -= n;
    return ret;
}

static inline unsigned show_val(const GetBitContext *s, unsigned n)
{
#ifdef BITSTREAM_READER_LE
    return s->cache & ((CACHE_TYPE(1) << n) - 1);
#else
    return s->cache >> (BITSTREAM_BITS - n);
#endif
}
#endif // ~CACHED_BITSTREAM_READER

/**
 * Skips the specified number of bits.
 * @param n the number of bits to skip,
 *          For the UNCHECKED_BITSTREAM_READER this must not cause the distance
 *          from the start to overflow int32_t. Staying within the bitstream + padding
 *          is sufficient, too.
 */
static inline void skip_bits_long(GetBitContext *s, int n)
{
#if CACHED_BITSTREAM_READER
    skip_bits(s, n);
#else
#if UNCHECKED_BITSTREAM_READER
    s->index += n;
#else
    s->index += av_clip(n, -s->index, s->size_in_bits_plus8 - s->index);
#endif
#endif
}

#if CACHED_BITSTREAM_READER
static inline void skip_remaining(GetBitContext *s, unsigned n)
{
#ifdef BITSTREAM_READER_LE
    s->cache >>= n;
#else
    s->cache <<= n;
#endif
    s->bits_left -= n;
}
#endif

/**
 * Read MPEG-1 dc-style VLC (sign bit + mantissa with no MSB).
 * if MSB not set it is negative
 * @param n length in bits
 */
static inline int get_xbits(GetBitContext *s, int n)
{
#if CACHED_BITSTREAM_READER
    int32_t cache;
    int sign;

    if (n > s->bits_left)
#ifdef BITSTREAM_READER_LE
        refill_gb(s, 1);
#else
        refill_gb(s, 0);
#endif

#if BITSTREAM_BITS == 32
    cache = s->cache;
#elif defined(BITSTREAM_READER_LE)
    cache = s->cache & 0xFFFFFFFF;
#else
    cache = s->cache >> 32;
#endif
    sign = ~cache >> 31;
    skip_remaining(s, n);

    return ((((uint32_t)(sign ^ cache)) >> (32 - n)) ^ sign) - sign;
#else
    register int sign;
    register int32_t cache;
    OPEN_READER(re, s);
    av_assert2(n>0 && n<=25);
    UPDATE_CACHE(re, s);
    cache = GET_CACHE(re, s);
    sign  = ~cache >> 31;
    LAST_SKIP_BITS(re, s, n);
    CLOSE_READER(re, s);
    return (NEG_USR32(sign ^ cache, n) ^ sign) - sign;
#endif
}

#if !CACHED_BITSTREAM_READER
static inline int get_xbits_le(GetBitContext *s, int n)
{
    register int sign;
    register int32_t cache;
    OPEN_READER(re, s);
    av_assert2(n>0 && n<=25);
    UPDATE_CACHE_LE(re, s);
    cache = GET_CACHE(re, s);
    sign  = sign_extend(~cache, n) >> 31;
    LAST_SKIP_BITS(re, s, n);
    CLOSE_READER(re, s);
    return (zero_extend(sign ^ cache, n) ^ sign) - sign;
}
#endif

static inline int get_sbits(GetBitContext *s, int n)
{
    register int tmp;
#if CACHED_BITSTREAM_READER
    av_assert2(n>0 && n<=25);
    tmp = sign_extend(get_bits(s, n), n);
#else
    OPEN_READER(re, s);
    av_assert2(n>0 && n<=25);
    UPDATE_CACHE(re, s);
    tmp = SHOW_SBITS(re, s, n);
    LAST_SKIP_BITS(re, s, n);
    CLOSE_READER(re, s);
#endif
    return tmp;
}

/**
 * Read 1-25 bits.
 */
static inline unsigned int get_bits(GetBitContext *s, int n)
{
    register unsigned int tmp;
#if CACHED_BITSTREAM_READER

    av_assert2(n>0 && n<=32);
    if (n > s->bits_left) {
#ifdef BITSTREAM_READER_LE
        refill_gb(s, 1);
#else
        refill_gb(s, 0);
#endif
        if (s->bits_left < BITSTREAM_HBITS)
            s->bits_left = n;
    }

#ifdef BITSTREAM_READER_LE
    tmp = get_val(s, n, 1);
#else
    tmp = get_val(s, n, 0);
#endif
#else
    OPEN_READER(re, s);
    av_assert2(n>0 && n<=25);
    UPDATE_CACHE(re, s);
    tmp = SHOW_UBITS(re, s, n);
    LAST_SKIP_BITS(re, s, n);
    CLOSE_READER(re, s);
#endif
    av_assert2(tmp < UINT64_C(1) << n);
    return tmp;
}

/**
 * Read 0-25 bits.
 */
static av_always_inline int get_bitsz(GetBitContext *s, int n)
{
    return n ? get_bits(s, n) : 0;
}

static inline unsigned int get_bits_le(GetBitContext *s, int n)
{
#if CACHED_BITSTREAM_READER
    av_assert2(n>0 && n<=32);
    if (n > s->bits_left) {
        refill_gb(s, 1);
        if (s->bits_left < BITSTREAM_HBITS)
            s->bits_left = n;
    }

    return get_val(s, n, 1);
#else
    register int tmp;
    OPEN_READER(re, s);
    av_assert2(n>0 && n<=25);
    UPDATE_CACHE_LE(re, s);
    tmp = SHOW_UBITS_LE(re, s, n);
    LAST_SKIP_BITS(re, s, n);
    CLOSE_READER(re, s);
    return tmp;
#endif
}

/**
 * Show 1-25 bits.
 */
static inline unsigned int show_bits(GetBitContext *s, int n)
{
    register unsigned int tmp;
#if CACHED_BITSTREAM_READER
    if (n > s->bits_left)
#ifdef BITSTREAM_READER_LE
        refill_gb(s, 1);
#else
        refill_gb(s, 0);
#endif

    tmp = show_val(s, n);
#else
    OPEN_READER_NOSIZE(re, s);
    av_assert2(n>0 && n<=25);
    UPDATE_CACHE(re, s);
    tmp = SHOW_UBITS(re, s, n);
#endif
    return tmp;
}

static inline void skip_bits(GetBitContext *s, int n)
{
#if CACHED_BITSTREAM_READER
    if (n < s->bits_left)
        skip_remaining(s, n);
    else {
        n -= s->bits_left;

        if (n >= BITSTREAM_BITS) {
            unsigned skip = n / 8;

            n -= 8*skip;
            s->ptr += skip;
        }

#ifdef BITSTREAM_READER_LE
        s->cache     = AV_RL_ALL(s->ptr);
#else
        s->cache     = AV_RB_ALL(s->ptr);
#endif
        s->ptr      += sizeof(cache_type);
        s->bits_left = BITSTREAM_BITS;
        if (n)
            skip_remaining(s, n);
    }
#else
    OPEN_READER(re, s);
    LAST_SKIP_BITS(re, s, n);
    CLOSE_READER(re, s);
#endif
}

static inline unsigned int get_bits1(GetBitContext *s)
{
#if CACHED_BITSTREAM_READER
    if (!s->bits_left) {
#ifdef BITSTREAM_READER_LE
        s->cache     = AV_RL_ALL(s->ptr);
#else
        s->cache     = AV_RB_ALL(s->ptr);
#endif
        s->ptr      += sizeof(cache_type);
        s->bits_left = BITSTREAM_BITS;
    }

#ifdef BITSTREAM_READER_LE
    return get_val(s, 1, 1);
#else
    return get_val(s, 1, 0);
#endif
#else
    unsigned int index = s->index;
    uint8_t result     = s->buffer[index >> 3];
#ifdef BITSTREAM_READER_LE
    result >>= index & 7;
    result  &= 1;
#else
    result <<= index & 7;
    result >>= 8 - 1;
#endif
#if !UNCHECKED_BITSTREAM_READER
    if (s->index < s->size_in_bits_plus8)
#endif
        index++;
    s->index = index;

    return result;
#endif
}

static inline unsigned int show_bits1(GetBitContext *s)
{
    return show_bits(s, 1);
}

static inline void skip_bits1(GetBitContext *s)
{
    skip_bits(s, 1);
}

/**
 * Read 0-32 bits.
 */
static inline unsigned int get_bits_long(GetBitContext *s, int n)
{
    unsigned ret = 0;
    av_assert2(n>=0 && n<=32);
    if (!n) {
        return 0;
#if CACHED_BITSTREAM_READER
    }

# ifdef BITSTREAM_READER_LE
    unsigned left = 0;
# endif
    if (n > s->bits_left) {
        n -= s->bits_left;
# ifdef BITSTREAM_READER_LE
        ret = s->cache & ((CACHE_TYPE(1) << s->bits_left) - 1);
        s->cache = AV_RL_ALL(s->ptr);
# else
        ret = s->cache >> (BITSTREAM_BITS - s->bits_left);
        s->cache = AV_RB_ALL(s->ptr);
# endif
        s->ptr += sizeof(cache_type);
        s->bits_left = BITSTREAM_BITS; // OK because consumed right after
    }

# ifdef BITSTREAM_READER_LE
    ret = get_val(s, n, 1) << left | ret;
# else
    ret = get_val(s, n, 0) | ret << n;
# endif

    return ret;
#else
    } else if (n <= MIN_CACHE_BITS) {
        return get_bits(s, n);
    } else {
#ifdef BITSTREAM_READER_LE
        ret = get_bits(s, 16);
        return ret | (get_bits(s, n - 16) << 16);
#else
        ret = get_bits(s, 16) << (n - 16);
        return ret | get_bits(s, n - 16);
#endif
    }
#endif
}

/**
 * Read 0-64 bits.
 */
static inline uint64_t get_bits64(GetBitContext *s, int n)
{
    if (n <= 32) {
        return get_bits_long(s, n);
    } else {
#ifdef BITSTREAM_READER_LE
        uint64_t ret = get_bits_long(s, 32);
        return ret | (uint64_t) get_bits_long(s, n - 32) << 32;
#else
        uint64_t ret = (uint64_t) get_bits_long(s, n - 32) << 32;
        return ret | get_bits_long(s, 32);
#endif
    }
}

/**
 * Read 0-32 bits as a signed integer.
 */
static inline int get_sbits_long(GetBitContext *s, int n)
{
    // sign_extend(x, 0) is undefined
    if (!n)
        return 0;

    return sign_extend(get_bits_long(s, n), n);
}

/**
 * Show 0-32 bits.
 */
static inline unsigned int show_bits_long(GetBitContext *s, int n)
{
    if (n <= MIN_CACHE_BITS) {
        return show_bits(s, n);
    } else {
        GetBitContext gb = *s;
        return get_bits_long(&gb, n);
    }
}

static inline int check_marker(void *logctx, GetBitContext *s, const char *msg)
{
    int bit = get_bits1(s);
    if (!bit)
        av_log(logctx, AV_LOG_INFO, "Marker bit missing at %d of %d %s\n",
               get_bits_count(s) - 1, s->size_in_bits, msg);

    return bit;
}

static inline int init_get_bits_xe(GetBitContext *s, const uint8_t *buffer,
                                   int bit_size, int is_le)
{
    int buffer_size;
    int ret = 0;

    if (bit_size >= INT_MAX - FFMAX(7, AV_INPUT_BUFFER_PADDING_SIZE*8) || bit_size < 0 || !buffer) {
        bit_size    = 0;
        buffer      = NULL;
        ret         = AVERROR_INVALIDDATA;
    }

    buffer_size = (bit_size + 7) >> 3;

    s->buffer             = buffer;
    s->size_in_bits       = bit_size;
    s->size_in_bits_plus8 = bit_size + 8;
    s->buffer_end         = buffer + buffer_size;

#if CACHED_BITSTREAM_READER
    if (is_le)
        s->cache          = AV_RL_ALL(buffer);
    else
        s->cache          = AV_RB_ALL(buffer);
    s->ptr                = buffer+sizeof(cache_type);
    s->bits_left          = BITSTREAM_BITS;
#else
    s->index              = 0;
#endif

    return ret;
}

/**
 * Initialize GetBitContext.
 * @param buffer bitstream buffer, must be AV_INPUT_BUFFER_PADDING_SIZE bytes
 *        larger than the actual read bits because some optimized bitstream
 *        readers read 32 or 64 bit at once and could read over the end
 * @param bit_size the size of the buffer in bits
 * @return 0 on success, AVERROR_INVALIDDATA if the buffer_size would overflow.
 */
static inline int init_get_bits(GetBitContext *s, const uint8_t *buffer,
                                int bit_size)
{
#ifdef BITSTREAM_READER_LE
    return init_get_bits_xe(s, buffer, bit_size, 1);
#else
    return init_get_bits_xe(s, buffer, bit_size, 0);
#endif
}

/**
 * Initialize GetBitContext.
 * @param buffer bitstream buffer, must be AV_INPUT_BUFFER_PADDING_SIZE bytes
 *        larger than the actual read bits because some optimized bitstream
 *        readers read 32 or 64 bit at once and could read over the end
 * @param byte_size the size of the buffer in bytes
 * @return 0 on success, AVERROR_INVALIDDATA if the buffer_size would overflow.
 */
static inline int init_get_bits8(GetBitContext *s, const uint8_t *buffer,
                                 int byte_size)
{
    if (byte_size > INT_MAX / 8 || byte_size < 0)
        byte_size = -1;
    return init_get_bits(s, buffer, byte_size * 8);
}

static inline int init_get_bits8_le(GetBitContext *s, const uint8_t *buffer,
                                    int byte_size)
{
    if (byte_size > INT_MAX / 8 || byte_size < 0)
        byte_size = -1;
    return init_get_bits_xe(s, buffer, byte_size * 8, 1);
}

static inline const uint8_t *align_get_bits(GetBitContext *s)
{
    int n = -get_bits_count(s) & 7;
    if (n)
        skip_bits(s, n);
#if CACHED_BITSTREAM_READER
    return s->buffer;
#else
    return s->buffer + (s->index >> 3);
#endif
}

/**
 * If the vlc code is invalid and max_depth=1, then no bits will be removed.
 * If the vlc code is invalid and max_depth>1, then the number of bits removed
 * is undefined.
 */
#define GET_VLC(code, name, gb, table, bits, max_depth)         \
    do {                                                        \
        int n, nb_bits;                                         \
        unsigned int index;                                     \
                                                                \
        index = SHOW_UBITS(name, gb, bits);                     \
        code  = table[index][0];                                \
        n     = table[index][1];                                \
                                                                \
        if (max_depth > 1 && n < 0) {                           \
            LAST_SKIP_BITS(name, gb, bits);                     \
            UPDATE_CACHE(name, gb);                             \
                                                                \
            nb_bits = -n;                                       \
                                                                \
            index = SHOW_UBITS(name, gb, nb_bits) + code;       \
            code  = table[index][0];                            \
            n     = table[index][1];                            \
            if (max_depth > 2 && n < 0) {                       \
                LAST_SKIP_BITS(name, gb, bits);                 \
                UPDATE_CACHE(name, gb);                         \
                                                                \
                nb_bits = -n;                                   \
                                                                \
                index = SHOW_UBITS(name, gb, nb_bits) + code;   \
                code  = table[index][0];                        \
                n     = table[index][1];                        \
            }                                                   \
        }                                                       \
        SKIP_BITS(name, gb, n);                                 \
    } while (0)

#define GET_RL_VLC(level, run, name, gb, table, bits,  \
                   max_depth, need_update)                      \
    do {                                                        \
        int n, nb_bits;                                         \
        unsigned int index;                                     \
                                                                \
        index = SHOW_UBITS(name, gb, bits);                     \
        level = table[index].level;                             \
        n     = table[index].len;                               \
                                                                \
        if (max_depth > 1 && n < 0) {                           \
            SKIP_BITS(name, gb, bits);                          \
            if (need_update) {                                  \
                UPDATE_CACHE(name, gb);                         \
            }                                                   \
                                                                \
            nb_bits = -n;                                       \
                                                                \
            index = SHOW_UBITS(name, gb, nb_bits) + level;      \
            level = table[index].level;                         \
            n     = table[index].len;                           \
            if (max_depth > 2 && n < 0) {                       \
                LAST_SKIP_BITS(name, gb, nb_bits);              \
                if (need_update) {                              \
                    UPDATE_CACHE(name, gb);                     \
                }                                               \
                nb_bits = -n;                                   \
                                                                \
                index = SHOW_UBITS(name, gb, nb_bits) + level;  \
                level = table[index].level;                     \
                n     = table[index].len;                       \
            }                                                   \
        }                                                       \
        run = table[index].run;                                 \
        SKIP_BITS(name, gb, n);                                 \
    } while (0)

/* Return the LUT element for the given bitstream configuration. */
static inline int set_idx(GetBitContext *s, int code, int *n, int *nb_bits,
                          VLC_TYPE (*table)[2])
{
    unsigned idx;

    *nb_bits = -*n;
    idx = show_bits(s, *nb_bits) + code;
    *n = table[idx][1];

    return table[idx][0];
}

/**
 * Parse a vlc code.
 * @param bits is the number of bits which will be read at once, must be
 *             identical to nb_bits in init_vlc()
 * @param max_depth is the number of times bits bits must be read to completely
 *                  read the longest vlc code
 *                  = (max_vlc_length + bits - 1) / bits
 * @returns the code parsed or -1 if no vlc matches
 */
static av_always_inline int get_vlc2(GetBitContext *s, VLC_TYPE (*table)[2],
                                     int bits, int max_depth)
{
#if CACHED_BITSTREAM_READER
    int nb_bits;
    unsigned idx = show_bits(s, bits);
    int code = table[idx][0];
    int n    = table[idx][1];

    if (max_depth > 1 && n < 0) {
        skip_remaining(s, bits);
        code = set_idx(s, code, &n, &nb_bits, table);
        if (max_depth > 2 && n < 0) {
            skip_remaining(s, bits);
            code = set_idx(s, code, &n, &nb_bits, table);
        }
    }
    skip_remaining(s, n);

    return code;
#else
    int code;

    OPEN_READER(re, s);
    UPDATE_CACHE(re, s);

    GET_VLC(code, re, s, table, bits, max_depth);

    CLOSE_READER(re, s);

    return code;
#endif
}

static inline int decode012(GetBitContext *gb)
{
    int n;
    n = get_bits1(gb);
    if (n == 0)
        return 0;
    else
        return get_bits1(gb) + 1;
}

static inline int decode210(GetBitContext *gb)
{
    if (get_bits1(gb))
        return 0;
    else
        return 2 - get_bits1(gb);
}

static inline int get_bits_left(GetBitContext *gb)
{
    return gb->size_in_bits - get_bits_count(gb);
}

static inline int skip_1stop_8data_bits(GetBitContext *gb)
{
    if (get_bits_left(gb) <= 0)
        return AVERROR_INVALIDDATA;

    while (get_bits1(gb)) {
        skip_bits(gb, 8);
        if (get_bits_left(gb) <= 0)
            return AVERROR_INVALIDDATA;
    }

    return 0;
}

#endif /* AVCODEC_GET_BITS_H */
