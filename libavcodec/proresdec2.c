/*
 * Copyright (c) 2010-2011 Maxim Poliakovski
 * Copyright (c) 2010-2011 Elvis Presley
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
 * Known FOURCCs: 'apch' (HQ), 'apcn' (SD), 'apcs' (LT), 'acpo' (Proxy), 'ap4h' (4444)
 */

//#define DEBUG

#include "libavutil/internal.h"
#include "avcodec.h"
#include "idctdsp.h"
#include "internal.h"
#include "profiles.h"
#include "simple_idct.h"
#include "proresdec.h"
#include "proresdata.h"
#include "thread.h"

static void permute(uint8_t *dst, const uint8_t *src, const uint8_t permutation[64])
{
    int i;
    for (i = 0; i < 64; i++)
        dst[i] = permutation[src[i]];
}

#define ALPHA_SHIFT_16_TO_10(alpha_val) (alpha_val >> 6)
#define ALPHA_SHIFT_8_TO_10(alpha_val)  ((alpha_val << 2) | (alpha_val >> 6))
#define ALPHA_SHIFT_16_TO_12(alpha_val) (alpha_val >> 4)
#define ALPHA_SHIFT_8_TO_12(alpha_val)  ((alpha_val << 4) | (alpha_val >> 4))

static void inline unpack_alpha(GetBitContext *gb, uint16_t *dst, int num_coeffs,
                                const int num_bits, const int decode_precision) {
    const int mask = (1 << num_bits) - 1;
    int i, idx, val, alpha_val;

    idx       = 0;
    alpha_val = mask;
    do {
        do {
            if (get_bits1(gb)) {
                val = get_bits(gb, num_bits);
            } else {
                int sign;
                val  = get_bits(gb, num_bits == 16 ? 7 : 4);
                sign = val & 1;
                val  = (val + 2) >> 1;
                if (sign)
                    val = -val;
            }
            alpha_val = (alpha_val + val) & mask;
            if (num_bits == 16) {
                if (decode_precision == 10) {
                    dst[idx++] = ALPHA_SHIFT_16_TO_10(alpha_val);
                } else { /* 12b */
                    dst[idx++] = ALPHA_SHIFT_16_TO_12(alpha_val);
                }
            } else {
                if (decode_precision == 10) {
                    dst[idx++] = ALPHA_SHIFT_8_TO_10(alpha_val);
                } else { /* 12b */
                    dst[idx++] = ALPHA_SHIFT_8_TO_12(alpha_val);
                }
            }
            if (idx >= num_coeffs)
                break;
        } while (get_bits_left(gb)>0 && get_bits1(gb));
        val = get_bits(gb, 4);
        if (!val)
            val = get_bits(gb, 11);
        if (idx + val > num_coeffs)
            val = num_coeffs - idx;
        if (num_bits == 16) {
            for (i = 0; i < val; i++) {
                if (decode_precision == 10) {
                    dst[idx++] = ALPHA_SHIFT_16_TO_10(alpha_val);
                } else { /* 12b */
                    dst[idx++] = ALPHA_SHIFT_16_TO_12(alpha_val);
                }
            }
        } else {
            for (i = 0; i < val; i++) {
                if (decode_precision == 10) {
                    dst[idx++] = ALPHA_SHIFT_8_TO_10(alpha_val);
                } else { /* 12b */
                    dst[idx++] = ALPHA_SHIFT_8_TO_12(alpha_val);
                }
            }
        }
    } while (idx < num_coeffs);
}

static void unpack_alpha_10(GetBitContext *gb, uint16_t *dst, int num_coeffs,
                            const int num_bits)
{
    if (num_bits == 16) {
        unpack_alpha(gb, dst, num_coeffs, 16, 10);
    } else { /* 8 bits alpha */
        unpack_alpha(gb, dst, num_coeffs, 8, 10);
    }
}

static void unpack_alpha_12(GetBitContext *gb, uint16_t *dst, int num_coeffs,
                            const int num_bits)
{
    if (num_bits == 16) {
        unpack_alpha(gb, dst, num_coeffs, 16, 12);
    } else { /* 8 bits alpha */
        unpack_alpha(gb, dst, num_coeffs, 8, 12);
    }
}

#define AC_BITS 12
#define PRORES_LEV_BITS 9

static const uint8_t ac_info[] = { 0x04, 0x0A, 0x05, 0x06, 0x28, 0x29 };

static av_cold int decode_init(AVCodecContext *avctx)
{
    int ret = 0;
    ProresContext *ctx = avctx->priv_data;
    uint8_t idct_permutation[64];
    int i;

    avctx->bits_per_raw_sample = 10;

    switch (avctx->codec_tag) {
    case MKTAG('a','p','c','o'):
        avctx->profile = FF_PROFILE_PRORES_PROXY;
        break;
    case MKTAG('a','p','c','s'):
        avctx->profile = FF_PROFILE_PRORES_LT;
        break;
    case MKTAG('a','p','c','n'):
        avctx->profile = FF_PROFILE_PRORES_STANDARD;
        break;
    case MKTAG('a','p','c','h'):
        avctx->profile = FF_PROFILE_PRORES_HQ;
        break;
    case MKTAG('a','p','4','h'):
        avctx->profile = FF_PROFILE_PRORES_4444;
        avctx->bits_per_raw_sample = 12;
        break;
    case MKTAG('a','p','4','x'):
        avctx->profile = FF_PROFILE_PRORES_XQ;
        avctx->bits_per_raw_sample = 12;
        break;
    default:
        avctx->profile = FF_PROFILE_UNKNOWN;
        av_log(avctx, AV_LOG_WARNING, "Unknown prores profile %d\n", avctx->codec_tag);
    }

    if (avctx->bits_per_raw_sample == 10) {
        av_log(avctx, AV_LOG_DEBUG, "Auto bitdepth precision. Use 10b decoding based on codec tag.\n");
    } else { /* 12b */
        av_log(avctx, AV_LOG_DEBUG, "Auto bitdepth precision. Use 12b decoding based on codec tag.\n");
    }

    ff_blockdsp_init(&ctx->bdsp, avctx);
    ret = ff_proresdsp_init(&ctx->prodsp, avctx);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "Fail to init proresdsp for bits per raw sample %d\n", avctx->bits_per_raw_sample);
        return ret;
    }

    ff_init_scantable_permutation(idct_permutation,
                                  ctx->prodsp.idct_permutation_type);

    permute(ctx->progressive_scan, ff_prores_progressive_scan, idct_permutation);
    permute(ctx->interlaced_scan, ff_prores_interlaced_scan, idct_permutation);

    // init dc_tables
    for (i = 0; i < sizeof(ac_info); i++) {
        uint32_t ac_codes[1<<AC_BITS];
        uint8_t ac_bits[1<<AC_BITS];
        unsigned int rice_order, exp_order, switch_bits, switch_val;
        int ac, max_bits = 0, codebook = ac_info[i];

        /* number of prefix bits to switch between Rice and expGolomb */
        switch_bits = (codebook & 3);
        rice_order  =  codebook >> 5;       /* rice code order */
        exp_order   = (codebook >> 2) & 7;  /* exp golomb code order */

        switch_val  = (switch_bits+1) << rice_order;

        // Values are actually transformed, but this is more a wrapping
        for (ac = 0; ac <1<<AC_BITS; ac++) {
            int exponent, bits, val = ac;
            unsigned int code;

            if (val >= switch_val) {
                val += (1 << exp_order) - switch_val;
                exponent = av_log2(val);
                bits = exponent+1+switch_bits-exp_order/*0*/ + exponent+1/*val*/;
                code = val;
            } else if (rice_order) {
                bits = (val >> rice_order)/*0*/ + 1/*1*/ + rice_order/*val*/;
                code = (1 << rice_order) | val;
            } else {
                bits = val/*0*/ + 1/*1*/;
                code = 1;
            }
            if (bits > max_bits) max_bits = bits;
            ac_bits [ac] = bits;
            ac_codes[ac] = code;
        }

        ff_free_vlc(ctx->ac_vlc+i);

        if (init_vlc(ctx->ac_vlc+i, PRORES_LEV_BITS, 1<<AC_BITS,
                     ac_bits, 1, 1, ac_codes, 4, 4, 0) < 0) {
            av_log(avctx, AV_LOG_ERROR, "Error for %d(0x%02X), max bits %d\n",
                   i, codebook, max_bits);
            return AVERROR_BUG;
        }
    }

    if (avctx->bits_per_raw_sample == 10){
        ctx->unpack_alpha = unpack_alpha_10;
    } else if (avctx->bits_per_raw_sample == 12){
        ctx->unpack_alpha = unpack_alpha_12;
    } else {
        av_log(avctx, AV_LOG_ERROR, "Fail to set unpack_alpha for bits per raw sample %d\n", avctx->bits_per_raw_sample);
        return AVERROR_BUG;
    }
    return ret;
}

static int decode_frame_header(ProresContext *ctx, const uint8_t *buf,
                               const int data_size, AVCodecContext *avctx)
{
    int hdr_size, width, height, flags;
    int version;
    const uint8_t *ptr;

    hdr_size = AV_RB16(buf);
    ff_dlog(avctx, "header size %d\n", hdr_size);
    if (hdr_size > data_size) {
        av_log(avctx, AV_LOG_ERROR, "error, wrong header size\n");
        return AVERROR_INVALIDDATA;
    }

    version = AV_RB16(buf + 2);
    ff_dlog(avctx, "%.4s version %d\n", buf+4, version);
    if (version > 1) {
        av_log(avctx, AV_LOG_ERROR, "unsupported version: %d\n", version);
        return AVERROR_PATCHWELCOME;
    }

    width  = AV_RB16(buf + 8);
    height = AV_RB16(buf + 10);

    if (width != avctx->width || height != avctx->height) {
        int ret;

        av_log(avctx, AV_LOG_WARNING, "picture resolution change: %dx%d -> %dx%d\n",
               avctx->width, avctx->height, width, height);
        if ((ret = ff_set_dimensions(avctx, width, height)) < 0)
            return ret;
    }

    ctx->frame_type = (buf[12] >> 2) & 3;
    ctx->alpha_info = buf[17] & 0xf;

    if (ctx->alpha_info > 2) {
        av_log(avctx, AV_LOG_ERROR, "Invalid alpha mode %d\n", ctx->alpha_info);
        return AVERROR_INVALIDDATA;
    }
    if (avctx->skip_alpha) ctx->alpha_info = 0;

    ff_dlog(avctx, "frame type %d\n", ctx->frame_type);

    if (ctx->frame_type == 0) {
        ctx->scan = ctx->progressive_scan; // permuted
    } else {
        ctx->scan = ctx->interlaced_scan; // permuted
        ctx->frame->interlaced_frame = 1;
        ctx->frame->top_field_first = ctx->frame_type == 1;
    }

    if (ctx->alpha_info) {
        if (avctx->bits_per_raw_sample == 10) {
            avctx->pix_fmt = (buf[12] & 0xC0) == 0xC0 ? AV_PIX_FMT_YUVA444P10 : AV_PIX_FMT_YUVA422P10;
        } else { /* 12b */
            avctx->pix_fmt = (buf[12] & 0xC0) == 0xC0 ? AV_PIX_FMT_YUVA444P12 : AV_PIX_FMT_YUVA422P12;
        }
    } else {
        if (avctx->bits_per_raw_sample == 10) {
            avctx->pix_fmt = (buf[12] & 0xC0) == 0xC0 ? AV_PIX_FMT_YUV444P10 : AV_PIX_FMT_YUV422P10;
        } else { /* 12b */
            avctx->pix_fmt = (buf[12] & 0xC0) == 0xC0 ? AV_PIX_FMT_YUV444P12 : AV_PIX_FMT_YUV422P12;
        }
    }

    avctx->color_primaries = buf[14];
    avctx->color_trc       = buf[15];
    avctx->colorspace      = buf[16];
    avctx->color_range     = AVCOL_RANGE_MPEG;

    ptr   = buf + 20;
    flags = buf[19];
    ff_dlog(avctx, "flags %x\n", flags);

    if (flags & 2) {
        if(buf + data_size - ptr < 64) {
            av_log(avctx, AV_LOG_ERROR, "Header truncated\n");
            return AVERROR_INVALIDDATA;
        }
        permute(ctx->qmat_luma, ctx->prodsp.idct_permutation, ptr);
        ptr += 64;
    } else {
        memset(ctx->qmat_luma, 4, 64);
    }

    if (flags & 1) {
        if(buf + data_size - ptr < 64) {
            av_log(avctx, AV_LOG_ERROR, "Header truncated\n");
            return AVERROR_INVALIDDATA;
        }
        permute(ctx->qmat_chroma, ctx->prodsp.idct_permutation, ptr);
    } else {
        memset(ctx->qmat_chroma, 4, 64);
    }

    return hdr_size;
}

static int decode_picture_header(AVCodecContext *avctx, const uint8_t *buf, const int buf_size)
{
    ProresContext *ctx = avctx->priv_data;
    int i, hdr_size, slice_count;
    unsigned pic_data_size;
    int log2_slice_mb_width, log2_slice_mb_height;
    int slice_mb_count, mb_x, mb_y;
    const uint8_t *data_ptr, *index_ptr;

    hdr_size = buf[0] >> 3;
    if (hdr_size < 8 || hdr_size > buf_size) {
        av_log(avctx, AV_LOG_ERROR, "error, wrong picture header size\n");
        return AVERROR_INVALIDDATA;
    }

    pic_data_size = AV_RB32(buf + 1);
    if (pic_data_size > buf_size) {
        av_log(avctx, AV_LOG_ERROR, "error, wrong picture data size\n");
        return AVERROR_INVALIDDATA;
    }

    log2_slice_mb_width  = buf[7] >> 4;
    log2_slice_mb_height = buf[7] & 0xF;
    if (log2_slice_mb_width > 3 || log2_slice_mb_height) {
        av_log(avctx, AV_LOG_ERROR, "unsupported slice resolution: %dx%d\n",
               1 << log2_slice_mb_width, 1 << log2_slice_mb_height);
        return AVERROR_INVALIDDATA;
    }

    ctx->mb_width  = (avctx->width  + 15) >> 4;
    if (ctx->frame_type)
        ctx->mb_height = (avctx->height + 31) >> 5;
    else
        ctx->mb_height = (avctx->height + 15) >> 4;

    // QT ignores the written value
    // slice_count = AV_RB16(buf + 5);
    slice_count = ctx->mb_height * ((ctx->mb_width >> log2_slice_mb_width) +
                                    av_popcount(ctx->mb_width & (1 << log2_slice_mb_width) - 1));

    if (ctx->slice_count != slice_count || !ctx->slices) {
        av_freep(&ctx->slices);
        ctx->slice_count = 0;
        ctx->slices = av_mallocz_array(slice_count, sizeof(*ctx->slices));
        if (!ctx->slices)
            return AVERROR(ENOMEM);
        ctx->slice_count = slice_count;
    }

    if (!slice_count)
        return AVERROR(EINVAL);

    if (hdr_size + slice_count*2 > buf_size) {
        av_log(avctx, AV_LOG_ERROR, "error, wrong slice count\n");
        return AVERROR_INVALIDDATA;
    }

    // parse slice information
    index_ptr = buf + hdr_size;
    data_ptr  = index_ptr + slice_count*2;

    slice_mb_count = 1 << log2_slice_mb_width;
    mb_x = 0;
    mb_y = 0;

    for (i = 0; i < slice_count; i++) {
        SliceContext *slice = &ctx->slices[i];

        slice->data = data_ptr;
        data_ptr += AV_RB16(index_ptr + i*2);

        while (ctx->mb_width - mb_x < slice_mb_count)
            slice_mb_count >>= 1;

        slice->mb_x = mb_x;
        slice->mb_y = mb_y;
        slice->mb_count = slice_mb_count;
        slice->data_size = data_ptr - slice->data;

        if (slice->data_size < 6) {
            av_log(avctx, AV_LOG_ERROR, "error, wrong slice data size\n");
            return AVERROR_INVALIDDATA;
        }

        mb_x += slice_mb_count;
        if (mb_x == ctx->mb_width) {
            slice_mb_count = 1 << log2_slice_mb_width;
            mb_x = 0;
            mb_y++;
        }
        if (data_ptr > buf + buf_size) {
            av_log(avctx, AV_LOG_ERROR, "error, slice out of bounds\n");
            return AVERROR_INVALIDDATA;
        }
    }

    if (mb_x || mb_y != ctx->mb_height) {
        av_log(avctx, AV_LOG_ERROR, "error wrong mb count y %d h %d\n",
               mb_y, ctx->mb_height);
        return AVERROR_INVALIDDATA;
    }

    return pic_data_size;
}

/* bitstream_read may fail on 32bits ARCHS for >24 bits, so use long version there */
#if BITSTREAM_BITS == 32
# define READ_BITS get_bits_long
#else
# define READ_BITS get_bits
#endif

/* Kept for reference and because clearer for first DC */
#define DECODE_CODEWORD(val, codebook)                                  \
    do {                                                                \
        unsigned int rice_order, exp_order, switch_bits;                \
        unsigned int q, buf, bits;                                      \
                                                                        \
        buf = show_bits(gb, 14);                                        \
                                                                        \
        /* number of bits to switch between rice and exp golomb */      \
        switch_bits =  codebook & 3;                                    \
        rice_order  =  codebook >> 5;                                   \
        exp_order   = (codebook >> 2) & 7;                              \
                                                                        \
        q = 13 - av_log2(buf);                                          \
                                                                        \
        if (q > switch_bits) { /* exp golomb */                         \
            bits = exp_order - switch_bits + (q<<1);                    \
            val = READ_BITS(gb, bits) - (1 << exp_order) +              \
                ((switch_bits + 1) << rice_order);                      \
        } else {                                                        \
            skip_remaining(gb, q+1);                                    \
            val = rice_order ? (q << rice_order) + get_bits(gb, rice_order) : q;\
        }                                                               \
    } while (0)

/* number of bits to switch between rice and exp golomb */
#define DECODE_CODEWORD2(val, switch_bits, rice_order, diff, offset)    \
    do {                                                                \
        unsigned int q, buf, bits;                                      \
                                                                        \
        buf = show_bits(gb, 14);                                        \
        q = 13 - av_log2(buf);                                          \
                                                                        \
        if (q > switch_bits) { /* exp golomb */                         \
            bits = (q<<1) + (int)diff;                                  \
            val = READ_BITS(gb, bits) + (int)offset;                    \
        } else {                                                        \
            skip_remaining(gb, q+1);                                    \
            val = rice_order ? (q << rice_order) + show_val(gb, rice_order) : q;   \
            skip_remaining(gb, rice_order);                             \
        }                                                               \
    } while (0)


#define TOSIGNED(x) (((x) >> 1) ^ (-((x) & 1)))

#define FIRST_DC_CB 0xB8

static const char dc_codebook[6][4] = {
    { 0, 0, 1, -1 }, { 0, 1, 2, -2 }, { 0, 1, 2, -2 },
    { 1, 2, 2,  0 }, { 1, 2, 2,  0 }, { 0, 3, 4, -8 }
};

static av_always_inline int decode_dc_coeffs(GetBitContext *gb, int16_t *out,
                                              int blocks_per_slice)
{
    int16_t prev_dc;
    int code, i, sign;
    DECODE_CODEWORD(code, FIRST_DC_CB);
    prev_dc = TOSIGNED(code);
    out[0] = prev_dc;

    out += 64; // dc coeff for the next block

    code = 5;
    sign = 0;
    for (i = 1; i < blocks_per_slice; i++, out += 64) {
        unsigned int dccb = FFMIN(code, 5U);
        DECODE_CODEWORD2(code, dc_codebook[dccb][0], dc_codebook[dccb][1],
                               dc_codebook[dccb][2], dc_codebook[dccb][3]);
        if(code) sign ^= -(code & 1);
        else     sign  = 0;
        prev_dc += (((code + 1) >> 1) ^ sign) - sign;
        out[0] = prev_dc;
    }
    return 0;
}

static av_always_inline int decode_ac_coeffs(AVCodecContext *avctx, GetBitContext *gb,
                                             int16_t *out, int blocks_per_slice)
{
    ProresContext *ctx = avctx->priv_data;
    int block_mask, sign;
    unsigned pos, run, level;
    int max_coeffs, i, bits_left;
    int log2_block_count = av_log2(blocks_per_slice);

    run   = 4;
    level = 2;

    max_coeffs = 64 << log2_block_count;
    block_mask = blocks_per_slice - 1;

    for (pos = block_mask;;) {
        bits_left = get_bits_left(gb);
        if (!bits_left || (bits_left < 14 && !show_bits(gb, bits_left)))
            break;

        if (run < 15) {
            static const uint8_t ctx_to_tbl[] = { 3, 3, 2, 2, 0, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4 };
            const VLC* tbl = ctx->ac_vlc + ctx_to_tbl[run];
            run = get_vlc2(gb, tbl->table, PRORES_LEV_BITS, 3);
        } else {
            unsigned int bits = 21 - 2*av_log2(show_bits(gb, 10));
            run = READ_BITS(gb, bits) - 4; // up to 17 bits
        }
        pos += run + 1;
        if (pos >= max_coeffs) {
            av_log(avctx, AV_LOG_ERROR, "ac tex damaged %d, %d\n", pos, max_coeffs);
            return AVERROR_INVALIDDATA;
        }

        if (level < 9) {
            static const uint8_t ctx_to_tbl[] = { 0, 1, 2, 3, 0, 4, 4, 4, 4 };
            const VLC* tbl = ctx->ac_vlc + ctx_to_tbl[level];
            level = 1+get_vlc2(gb, tbl->table, PRORES_LEV_BITS, 3);
        } else {
            unsigned int bits = 25 - 2*av_log2(show_bits(gb, 12));
            level = READ_BITS(gb, bits) - 4 + 1; // up to 21 bits
        }

        i = pos >> log2_block_count;

        sign = -get_bits1(gb);
        out[((pos & block_mask) << 6) + ctx->scan[i]] = ((level ^ sign) - sign);
    }

    return 0;
}

static int decode_slice_luma(AVCodecContext *avctx, SliceContext *slice,
                             uint16_t *dst, int dst_stride,
                             const uint8_t *buf, unsigned buf_size,
                             const int16_t *qmat)
{
    ProresContext *ctx = avctx->priv_data;
    LOCAL_ALIGNED_32(int16_t, blocks, [8*4*64]);
    int16_t *block;
    GetBitContext gb;
    int i, blocks_per_slice = slice->mb_count<<2;
    int ret;

    for (i = 0; i < blocks_per_slice; i++)
        ctx->bdsp.clear_block(blocks+(i<<6));

    init_get_bits(&gb, buf, buf_size << 3);

    if ((ret = decode_dc_coeffs(&gb, blocks, blocks_per_slice)) < 0)
        return ret;
    if ((ret = decode_ac_coeffs(avctx, &gb, blocks, blocks_per_slice)) < 0)
        return ret;

    block = blocks;
    for (i = 0; i < slice->mb_count; i++) {
        ctx->prodsp.idct_put(dst, dst_stride, block+(0<<6), qmat);
        ctx->prodsp.idct_put(dst             +8, dst_stride, block+(1<<6), qmat);
        ctx->prodsp.idct_put(dst+4*dst_stride  , dst_stride, block+(2<<6), qmat);
        ctx->prodsp.idct_put(dst+4*dst_stride+8, dst_stride, block+(3<<6), qmat);
        block += 4*64;
        dst += 16;
    }
    return 0;
}

static int decode_slice_chroma(AVCodecContext *avctx, SliceContext *slice,
                               uint16_t *dst, int dst_stride,
                               const uint8_t *buf, unsigned buf_size,
                               const int16_t *qmat, int log2_blocks_per_mb)
{
    ProresContext *ctx = avctx->priv_data;
    LOCAL_ALIGNED_32(int16_t, blocks, [8*4*64]);
    int16_t *block;
    GetBitContext gb;
    int i, j, blocks_per_slice = slice->mb_count << log2_blocks_per_mb;
    int ret;

    for (i = 0; i < blocks_per_slice; i++)
        ctx->bdsp.clear_block(blocks+(i<<6));

    init_get_bits(&gb, buf, buf_size << 3);

    if ((ret = decode_dc_coeffs(&gb, blocks, blocks_per_slice)) < 0)
        return ret;
    if ((ret = decode_ac_coeffs(avctx, &gb, blocks, blocks_per_slice)) < 0)
        return ret;

    block = blocks;
    for (i = 0; i < slice->mb_count; i++) {
        for (j = 0; j < log2_blocks_per_mb; j++) {
            ctx->prodsp.idct_put(dst,              dst_stride, block+(0<<6), qmat);
            ctx->prodsp.idct_put(dst+4*dst_stride, dst_stride, block+(1<<6), qmat);
            block += 2*64;
            dst += 8;
        }
    }
    return 0;
}

/**
 * Decode alpha slice plane.
 */
static void decode_slice_alpha(ProresContext *ctx,
                               uint16_t *dst, int dst_stride,
                               const uint8_t *buf, int buf_size,
                               int blocks_per_slice)
{
    GetBitContext gb;
    int i;
    LOCAL_ALIGNED_32(int16_t, blocks, [8*4*64]);
    int16_t *block;

    for (i = 0; i < blocks_per_slice<<2; i++)
        ctx->bdsp.clear_block(blocks+(i<<6));

    init_get_bits(&gb, buf, buf_size << 3);

    if (ctx->alpha_info == 2) {
        ctx->unpack_alpha(&gb, blocks, blocks_per_slice * 4 * 64, 16);
    } else {
        ctx->unpack_alpha(&gb, blocks, blocks_per_slice * 4 * 64, 8);
    }

    block = blocks;

    for (i = 0; i < 16; i++) {
        memcpy(dst, block, 16 * blocks_per_slice * sizeof(*dst));
        dst   += dst_stride >> 1;
        block += 16 * blocks_per_slice;
    }
}

static int decode_slice_thread(AVCodecContext *avctx, void *arg, int jobnr, int threadnr)
{
    ProresContext *ctx = avctx->priv_data;
    SliceContext *slice = &ctx->slices[jobnr];
    const uint8_t *buf = slice->data;
    AVFrame *pic = ctx->frame;
    int i, hdr_size, qscale, log2_chroma_blocks_per_mb;
    int luma_stride, chroma_stride;
    int y_data_size, u_data_size, v_data_size, a_data_size;
    uint8_t *dest_y, *dest_u, *dest_v, *dest_a;
    LOCAL_ALIGNED_16(int16_t, qmat_luma_scaled,  [64]);
    LOCAL_ALIGNED_16(int16_t, qmat_chroma_scaled,[64]);
    int mb_x_shift;
    int ret;
    uint16_t val_no_chroma;

    slice->ret = -1;
    //av_log(avctx, AV_LOG_INFO, "slice %d mb width %d mb x %d y %d\n",
    //       jobnr, slice->mb_count, slice->mb_x, slice->mb_y);

    // slice header
    hdr_size = buf[0] >> 3;
    qscale = av_clip(buf[1], 1, 224);
    qscale = qscale > 128 ? qscale - 96 << 2: qscale;
    y_data_size = AV_RB16(buf + 2);
    u_data_size = AV_RB16(buf + 4);
    v_data_size = slice->data_size - y_data_size - u_data_size - hdr_size;
    if (hdr_size > 7) v_data_size = AV_RB16(buf + 6);
    a_data_size = slice->data_size - y_data_size - u_data_size -
                  v_data_size - hdr_size;

    if (y_data_size < 0 || u_data_size < 0 || v_data_size < 0
        || hdr_size+y_data_size+u_data_size+v_data_size > slice->data_size){
        av_log(avctx, AV_LOG_ERROR, "invalid plane data size\n");
        return AVERROR_INVALIDDATA;
    }

    buf += hdr_size;

    for (i = 0; i < 64; i++) {
        qmat_luma_scaled  [i] = ctx->qmat_luma  [i] * qscale;
        qmat_chroma_scaled[i] = ctx->qmat_chroma[i] * qscale;
    }

    if (ctx->frame_type == 0) {
        luma_stride   = pic->linesize[0];
        chroma_stride = pic->linesize[1];
    } else {
        luma_stride   = pic->linesize[0] << 1;
        chroma_stride = pic->linesize[1] << 1;
    }

    if (avctx->pix_fmt == AV_PIX_FMT_YUV444P10 || avctx->pix_fmt == AV_PIX_FMT_YUVA444P10 ||
        avctx->pix_fmt == AV_PIX_FMT_YUV444P12 || avctx->pix_fmt == AV_PIX_FMT_YUVA444P12) {
        mb_x_shift = 5;
        log2_chroma_blocks_per_mb = 2;
    } else {
        mb_x_shift = 4;
        log2_chroma_blocks_per_mb = 1;
    }

    dest_y = pic->data[0] + (slice->mb_y << 4) * luma_stride + (slice->mb_x << 5);
    dest_u = pic->data[1] + (slice->mb_y << 4) * chroma_stride + (slice->mb_x << mb_x_shift);
    dest_v = pic->data[2] + (slice->mb_y << 4) * chroma_stride + (slice->mb_x << mb_x_shift);
    dest_a = pic->data[3] + (slice->mb_y << 4) * luma_stride + (slice->mb_x << 5);

    if (ctx->frame_type && ctx->first_field ^ ctx->frame->top_field_first) {
        dest_y += pic->linesize[0];
        dest_u += pic->linesize[1];
        dest_v += pic->linesize[2];
        dest_a += pic->linesize[3];
    }

    ret = decode_slice_luma(avctx, slice, (uint16_t*)dest_y, luma_stride,
                            buf, y_data_size, qmat_luma_scaled);
    if (ret < 0)
        return ret;

    if (!(avctx->flags & AV_CODEC_FLAG_GRAY) && (u_data_size + v_data_size) > 0) {
        ret = decode_slice_chroma(avctx, slice, (uint16_t*)dest_u, chroma_stride,
                                  buf + y_data_size, u_data_size,
                                  qmat_chroma_scaled, log2_chroma_blocks_per_mb);
        if (ret < 0)
            return ret;

        ret = decode_slice_chroma(avctx, slice, (uint16_t*)dest_v, chroma_stride,
                                  buf + y_data_size + u_data_size, v_data_size,
                                  qmat_chroma_scaled, log2_chroma_blocks_per_mb);
        if (ret < 0)
            return ret;
    }
    else {
        size_t mb_max_x = slice->mb_count << (mb_x_shift - 1);
        size_t i, j;
        if (avctx->bits_per_raw_sample == 10) {
            val_no_chroma = 511;
        } else { /* 12b */
            val_no_chroma = 511 * 4;
        }
        for (i = 0; i < 16; ++i)
            for (j = 0; j < mb_max_x; ++j) {
                *(uint16_t*)(dest_u + (i * chroma_stride) + (j << 1)) = val_no_chroma;
                *(uint16_t*)(dest_v + (i * chroma_stride) + (j << 1)) = val_no_chroma;
            }
    }

    /* decode alpha plane if available */
    if (ctx->alpha_info && pic->data[3] && a_data_size)
        decode_slice_alpha(ctx, (uint16_t*)dest_a, luma_stride,
                           buf + y_data_size + u_data_size + v_data_size,
                           a_data_size, slice->mb_count);

    slice->ret = 0;
    return 0;
}

static int decode_picture(AVCodecContext *avctx)
{
    ProresContext *ctx = avctx->priv_data;
    int i;
    int error = 0;

    avctx->execute2(avctx, decode_slice_thread, NULL, NULL, ctx->slice_count);

    for (i = 0; i < ctx->slice_count; i++)
        error += ctx->slices[i].ret < 0;

    if (error)
        ctx->frame->decode_error_flags = FF_DECODE_ERROR_INVALID_BITSTREAM;
    if (error < ctx->slice_count)
        return 0;

    return ctx->slices[0].ret;
}

static int decode_frame(AVCodecContext *avctx, void *data, int *got_frame,
                        AVPacket *avpkt)
{
    ProresContext *ctx = avctx->priv_data;
    ThreadFrame tframe = { .f = data };
    AVFrame *frame = data;
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    int frame_hdr_size, pic_size, ret;

    if (buf_size < 28 || AV_RL32(buf + 4) != AV_RL32("icpf")) {
        av_log(avctx, AV_LOG_ERROR, "invalid frame header\n");
        return AVERROR_INVALIDDATA;
    }

    ctx->frame = frame;
    ctx->frame->pict_type = AV_PICTURE_TYPE_I;
    ctx->frame->key_frame = 1;
    ctx->first_field = 1;

    buf += 8;
    buf_size -= 8;

    frame_hdr_size = decode_frame_header(ctx, buf, buf_size, avctx);
    if (frame_hdr_size < 0)
        return frame_hdr_size;

    buf += frame_hdr_size;
    buf_size -= frame_hdr_size;

 decode_picture:
    pic_size = decode_picture_header(avctx, buf, buf_size);
    if (pic_size < 0) {
        av_log(avctx, AV_LOG_ERROR, "error decoding picture header\n");
        return pic_size;
    }

    if (ctx->first_field)
        if ((ret = ff_thread_get_buffer(avctx, &tframe, 0)) < 0)
            return ret;

    if ((ret = decode_picture(avctx)) < 0) {
        av_log(avctx, AV_LOG_ERROR, "error decoding picture\n");
        return ret;
    }

    buf += pic_size;
    buf_size -= pic_size;

    if (ctx->frame_type && buf_size > 0 && ctx->first_field) {
        ctx->first_field = 0;
        goto decode_picture;
    }

    *got_frame      = 1;

    return avpkt->size;
}

#if HAVE_THREADS
static int decode_init_thread_copy(AVCodecContext *avctx)
{
    ProresContext *ctx = avctx->priv_data;

    ctx->slices = NULL;

    return 0;
}
#endif

static av_cold int decode_close(AVCodecContext *avctx)
{
    ProresContext *ctx = avctx->priv_data;
    int i;

    if (!avctx->internal->is_copy)
        for (i = 0; i < sizeof(ac_info); i++)
            ff_free_vlc(ctx->ac_vlc+i);

    av_freep(&ctx->slices);

    return 0;
}

AVCodec ff_prores_decoder = {
    .name           = "prores",
    .long_name      = NULL_IF_CONFIG_SMALL("ProRes (iCodec Pro)"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_PRORES,
    .priv_data_size = sizeof(ProresContext),
    .init           = decode_init,
    .init_thread_copy = ONLY_IF_THREADS_ENABLED(decode_init_thread_copy),
    .close          = decode_close,
    .decode         = decode_frame,
    .capabilities   = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_SLICE_THREADS | AV_CODEC_CAP_FRAME_THREADS,
    .profiles       = NULL_IF_CONFIG_SMALL(ff_prores_profiles),
};
