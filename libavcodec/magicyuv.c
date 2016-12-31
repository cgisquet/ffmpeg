/*
 * MagicYUV decoder
 * Copyright (c) 2016 Paul B Mahol
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

#include <stdlib.h>
#include <string.h>

#include "libavutil/pixdesc.h"
#include "libavutil/qsort.h"

#include "avcodec.h"
#include "bytestream.h"
#include "huffjoint.h"
#include "huffyuvdsp.h"
#include "internal.h"
#include "lossless_videodsp.h"
#include "thread.h"

#define VLC_BITS 12
#define MAX_VLC_N 16384

typedef struct Slice {
    uint32_t start;
    uint32_t size;
} Slice;

typedef enum Prediction {
    LEFT = 1,
    GRADIENT,
    MEDIAN,
} Prediction;

typedef struct HuffEntry {
    uint16_t sym;
    uint8_t  len;
    uint32_t code;
} HuffEntry;

typedef struct JointTable {
    int8_t  len;
    uint8_t type;
    union {
        uint16_t  for2;
        uint32_t  for4;
    } code;
} JointTable;

typedef struct MagicYUVContext {
    AVFrame          *p;
    int               max;
    int               slice_height;
    int               nb_slices;
    int               planes;         // number of encoded planes in bitstream
    int               decorrelate;    // postprocessing work
    int               color_matrix;   // video color matrix
    int               flags;
    int               interlaced;     // video is interlaced
    int               vlc_n;
    uint8_t          *buf;            // pointer to AVPacket->data
    int               hshift[4];
    int               vshift[4];
    Slice            *slices[4];      // slice bitstream positions for each plane
    unsigned int      slices_size[4]; // slice sizes for each plane
    uint8_t           len[4][1024];   // table of code lengths for each plane
    VLC               vlc[12];        // VLC for each plane
    JointTable       *mem[4];
    int (*huff_build)(struct MagicYUVContext *s, int plane, int mask);
    int (*magy_decode_slice)(AVCodecContext *avctx, void *tdata,
                             int j, int threadnr);
    LLVidDSPContext   llviddsp;
} MagicYUVContext;

static int huff_cmp_len(const void *a, const void *b)
{
    const HuffEntry *aa = a, *bb = b;
    return (aa->len - bb->len) * 256 + aa->sym - bb->sym;
}

static int huff_cmp_len10(const void *a, const void *b)
{
    const HuffEntry *aa = a, *bb = b;
    return (aa->len - bb->len) * 1024 + aa->sym - bb->sym;
}

static int huff_build10(MagicYUVContext *s, int p, int mask)
{
    VLC *vlc = s->vlc+p;
    uint8_t *len = (uint8_t *)(s->len+p);
    HuffEntry he[1024];
    uint32_t codes[1024];
    uint8_t bits[1024];
    uint16_t syms[1024];
    uint16_t lut[1024];
    uint16_t *jsym = ff_huff_joint_alloc(VLC_BITS);
    uint32_t code;
    int i;

    if (!jsym)
        return AVERROR(ENOMEM);

    for (i = 0; i < 1024; i++) {
        he[i].sym = 1023 - i;
        he[i].len = len[i];
        if (len[i] == 0)
            return AVERROR_INVALIDDATA;
    }
    AV_QSORT(he, 1024, HuffEntry, huff_cmp_len10);

    code = 1;
    for (i = 1023; i >= 0; i--) {
        codes[i] = code >> (32 - he[i].len);
        bits[i]  = he[i].len;
        syms[i]  = 1023 - he[i].sym;
        lut[syms[i]] = i;
        code += 0x80000000u >> (he[i].len - 1);
    }

    ff_free_vlc(vlc);
    if (ff_init_vlc_sparse(vlc, FFMIN(he[1023].len, VLC_BITS), 1023,
                           bits,  sizeof(*bits),  sizeof(*bits),
                           codes, sizeof(*codes), sizeof(*codes),
                           syms,  sizeof(*syms),  sizeof(*syms), 0)) {
        av_freep(&jsym);
        return AVERROR_INVALIDDATA;
    }

    // generate 2-joint table
    vlc += 4;
    return ff_huff_joint_gen(vlc, jsym, mask, VLC_BITS,
                             codes, codes, bits, bits, lut, lut);
}

static int huff_build(MagicYUVContext *s, int p, int mask)
{
    VLC *vlc = s->vlc+p;
    uint8_t *len = (uint8_t *)(s->len+p);
    HuffEntry he[256];
    uint32_t codes[256];
    uint8_t bits[256];
    uint8_t syms[256];
    uint16_t lut[256];
    uint16_t *jsym = ff_huff_joint_alloc(VLC_BITS);
    uint32_t code;
    uint32_t *lut4;
    int i;
    int val = 0;

    if (!jsym)
        return AVERROR(ENOMEM);

    av_freep(&s->mem[p]);
    s->mem[p] = av_malloc((1<<VLC_BITS)*sizeof(JointTable));
    if (!s->mem[p])
        return AVERROR(ENOMEM);

    for (i = 0; i < 256; i++) {
        he[i].sym = 255 - i;
        he[i].len = len[i];
        if (len[i] == 0)
            return AVERROR_INVALIDDATA;
    }
    AV_QSORT(he, 256, HuffEntry, huff_cmp_len);

    code = 1;
    for (i = 255; i >= 0; i--) {
        codes[i] = code >> (32 - he[i].len);
        bits[i]  = he[i].len;
        syms[i]  = 255 - he[i].sym;
        lut[syms[i]] = i;
        code += 0x80000000u >> (he[i].len - 1);
    }
    if (len[0] == 1) {
        // Symbol for 8 zeros
        val = bits[0] ? 0x100 : 0x1;
    }

    ff_free_vlc(vlc);
    if (ff_init_vlc_sparse(vlc, FFMIN(he[255].len, VLC_BITS), 256,
                           bits,  sizeof(*bits),  sizeof(*bits),
                           codes, sizeof(*codes), sizeof(*codes),
                           syms,  sizeof(*syms),  sizeof(*syms), 0)) {
        goto err;
    }

    // generate 2-joint table
    vlc += 4;
    if (ff_huff_joint_gen(vlc, jsym, mask, VLC_BITS,
                          codes, codes, bits, bits, lut, lut)) {
        goto err;
    }

    // 4-joint table
    vlc += 4;
    lut4 = ff_huff_joint4same_gen(vlc, jsym, mask, VLC_BITS,
                                  codes, bits, lut);
    if (!lut4) {
        goto err;
    }

    for (i = 0; i < 1<<VLC_BITS; i++) {
        if (s->vlc[8+p].table[i][1] > 0) {
            s->mem[p][i].len  = s->vlc[8+p].table[i][1];
            s->mem[p][i].type = 1;
            s->mem[p][i].code.for4 = lut4[s->vlc[8+p].table[i][0]];
        } else if (s->vlc[4+p].table[i][1] > 0) {
            s->mem[p][i].len  = s->vlc[4+p].table[i][1];
            s->mem[p][i].type = 0;
            AV_WB16(&s->mem[p][i].code.for2, s->vlc[4+p].table[i][0]);
        } else
            s->mem[p][i].len = -1;
    }

    av_free(lut4);

    if (val) {
        // Add 8 wide entries for it
        for (i = (val-1)<<(VLC_BITS-8); i < val<<(VLC_BITS-8); i++) {
            s->mem[p][i].len = 8;
            s->mem[p][i].type = 2;
        }
    }

    return 0;

err:
    av_freep(&jsym);
    return AVERROR_INVALIDDATA;
}

static void magicyuv_median_pred10(uint16_t *dst, const uint16_t *src1,
                                   const uint16_t *diff, intptr_t w,
                                   int *left, int *left_top)
{
    int i;
    uint16_t l, lt;

    l  = *left;
    lt = *left_top;

    for (i = 0; i < w; i++) {
        l      = mid_pred(l, src1[i], (l + src1[i] - lt)) + diff[i];
        l     &= 0x3FF;
        lt     = src1[i];
        dst[i] = l;
    }

    *left     = l;
    *left_top = lt;
}

#define READ_2PIX_PLANE(dst0, dst1, plane, OP) \
    GET_VLC_DUAL(dst0, dst1, &bc, s->vlc[4+plane].table, \
                 s->vlc[plane].table, s->vlc[plane].table, \
                 VLC_BITS, 3, OP)

#define GET_VLC_ITER(dst, off, bc, JTable, table, bits, max_depth)  \
    do {                                                            \
        unsigned int index = bitstream_peek(bc, bits);              \
        int          code, n = JTable[index].len;                   \
                                                                    \
        if (n>0) {                                                  \
            if (JTable[index].type == 2) {                          \
                AV_ZERO64(dst+off);                                 \
                off += 8;                                           \
                bitstream_skip(bc, 8);                              \
            } else if (JTable[index].type) {                        \
                AV_WN32(dst+off, JTable[index].code.for4);          \
                off += 4;                                           \
                bitstream_skip(bc, n);                              \
            } else {                                                \
                AV_WN16(dst+off, JTable[index].code.for2);          \
                off += 2;                                           \
                bitstream_skip(bc, n);                              \
            }                                                       \
        } else {                                                    \
            int nb_bits;                                            \
            VLC_INTERN(dst[off], table, bc, bits, max_depth);       \
            off++;                                                  \
        }                                                           \
    } while (0)

#define READ_4PIX_PLANE(dst, off, plane) \
    GET_VLC_ITER(dst, off, &bc, s->mem[plane], s->vlc[plane].table, VLC_BITS, 3)


static int magy_decode_slice10(AVCodecContext *avctx, void *tdata,
                               int j, int threadnr)
{
    MagicYUVContext *s = avctx->priv_data;
    int interlaced = s->interlaced;
    AVFrame *p = s->p;
    int i, k, x;
    BitstreamContext bc;
    uint16_t *dst;

    for (i = 0; i < s->planes; i++) {
        int left, lefttop, top;
        int height = AV_CEIL_RSHIFT(FFMIN(s->slice_height, avctx->coded_height - j * s->slice_height), s->vshift[i]);
        int width = AV_CEIL_RSHIFT(avctx->coded_width, s->hshift[i]);
        int sheight = AV_CEIL_RSHIFT(s->slice_height, s->vshift[i]);
        ptrdiff_t fake_stride = (p->linesize[i] / 2) * (1 + interlaced);
        ptrdiff_t stride = p->linesize[i] / 2;
        int flags, pred;
        int ret = bitstream_init8(&bc, s->buf + s->slices[i][j].start,
                                  s->slices[i][j].size);

        if (ret < 0)
            return ret;

        flags = bitstream_read(&bc, 8);
        pred  = bitstream_read(&bc, 8);

        dst = (uint16_t *)p->data[i] + j * sheight * stride;
        if (flags & 1) {
            for (k = 0; k < height; k++) {
                for (x = 0; x < width; x++)
                    dst[x] = bitstream_read(&bc, 10);

                dst += stride;
            }
        } else {
            int count = width/2;
            for (k = 0; k < height; k++) {
                if (count >= bitstream_bits_left(&bc) / (32 * 2)) {
                    for (x = 0; x < count && bitstream_bits_left(&bc) > 0; x++) {
                        READ_2PIX_PLANE(dst[2 * x], dst[2 * x + 1], i, OP14bits);
                    }
                } else {
                    for (x = 0; x < count; x++) {
                        READ_2PIX_PLANE(dst[2 * x], dst[2 * x + 1], i, OP14bits);
                    }
                }
                if( width&1 && bitstream_bits_left(&bc)>0 ) {
                    unsigned int index;
                    int nb_bits, code, n;
                    index = bitstream_peek(&bc, VLC_BITS);
                    VLC_INTERN(dst[width-1], s->vlc[i].table,
                               &bc, VLC_BITS, 3);
                }
                dst += stride;
            }
        }

        switch (pred) {
        case LEFT:
            dst = (uint16_t *)p->data[i] + j * sheight * stride;
            s->llviddsp.add_left_pred_int16(dst, dst, 1023, width, 0);
            dst += stride;
            if (interlaced) {
                s->llviddsp.add_left_pred_int16(dst, dst, 1023, width, 0);
                dst += stride;
            }
            for (k = 1 + interlaced; k < height; k++) {
                s->llviddsp.add_left_pred_int16(dst, dst, 1023, width, dst[-fake_stride]);
                dst += stride;
            }
            break;
        case GRADIENT:
            dst = (uint16_t *)p->data[i] + j * sheight * stride;
            s->llviddsp.add_left_pred_int16(dst, dst, 1023, width, 0);
            left = lefttop = 0;
            dst += stride;
            if (interlaced) {
                s->llviddsp.add_left_pred_int16(dst, dst, 1023, width, 0);
                left = lefttop = 0;
                dst += stride;
            }
            for (k = 1 + interlaced; k < height; k++) {
                top = dst[-fake_stride];
                left = top + dst[0];
                dst[0] = left & 0x3FF;
                for (x = 1; x < width; x++) {
                    top = dst[x - fake_stride];
                    lefttop = dst[x - (fake_stride + 1)];
                    left += top - lefttop + dst[x];
                    dst[x] = left & 0x3FF;
                }
                dst += stride;
            }
            break;
        case MEDIAN:
            dst = (uint16_t *)p->data[i] + j * sheight * stride;
            lefttop = left = dst[0];
            s->llviddsp.add_left_pred_int16(dst, dst, 1023, width, 0);
            dst += stride;
            if (interlaced) {
                lefttop = left = dst[0];
                s->llviddsp.add_left_pred_int16(dst, dst, 1023, width, 0);
                dst += stride;
            }
            for (k = 1 + interlaced; k < height; k++) {
                magicyuv_median_pred10(dst, dst - fake_stride, dst, width, &left, &lefttop);
                lefttop = left = dst[0];
                dst += stride;
            }
            break;
        default:
            avpriv_request_sample(avctx, "Unknown prediction: %d", pred);
        }
    }

    if (s->decorrelate) {
        int height = FFMIN(s->slice_height, avctx->coded_height - j * s->slice_height);
        int width = avctx->coded_width;
        uint16_t *r = (uint16_t *)p->data[0] + j * s->slice_height * p->linesize[0] / 2;
        uint16_t *g = (uint16_t *)p->data[1] + j * s->slice_height * p->linesize[1] / 2;
        uint16_t *b = (uint16_t *)p->data[2] + j * s->slice_height * p->linesize[2] / 2;

        for (i = 0; i < height; i++) {
            for (k = 0; k < width; k++) {
                b[k] = (b[k] + g[k]) & 0x3FF;
                r[k] = (r[k] + g[k]) & 0x3FF;
            }
            b += p->linesize[0] / 2;
            g += p->linesize[1] / 2;
            r += p->linesize[2] / 2;
        }
    }

    return 0;
}

static int magy_decode_slice(AVCodecContext *avctx, void *tdata,
                             int j, int threadnr)
{
    MagicYUVContext *s = avctx->priv_data;
    int interlaced = s->interlaced;
    AVFrame *p = s->p;
    int i, k, x;
    BitstreamContext bc;
    uint8_t *dst;

    for (i = 0; i < s->planes; i++) {
        int left, lefttop, top;
        int height = AV_CEIL_RSHIFT(FFMIN(s->slice_height, avctx->coded_height - j * s->slice_height), s->vshift[i]);
        int width = AV_CEIL_RSHIFT(avctx->coded_width, s->hshift[i]);
        int sheight = AV_CEIL_RSHIFT(s->slice_height, s->vshift[i]);
        ptrdiff_t fake_stride = p->linesize[i] * (1 + interlaced);
        ptrdiff_t stride = p->linesize[i];
        int flags, pred;
        int ret = bitstream_init8(&bc, s->buf + s->slices[i][j].start,
                                  s->slices[i][j].size);

        if (ret < 0)
            return ret;

        flags = bitstream_read(&bc, 8);
        pred  = bitstream_read(&bc, 8);

        dst = p->data[i] + j * sheight * stride;
        if (flags & 1) {
            for (k = 0; k < height; k++) {
                for (x = 0; x < width; x++)
                    dst[x] = bitstream_read(&bc, 8);

                dst += stride;
            }
        } else {
            for (k = 0; k < height; k++) {
                if (width >= bitstream_bits_left(&bc) / 32) {
                    for (x = 0; x < width-8 && bitstream_bits_left(&bc) > 0;) {
                        READ_4PIX_PLANE(dst, x, i);
                    }
                } else {
                    for (x = 0; x < width-8;) {
                        READ_4PIX_PLANE(dst, x, i);
                    }
                }

                for( ; x < width && bitstream_bits_left(&bc)>0; x++ ) {
                    unsigned int index;
                    int nb_bits, code, n;
                    index = bitstream_peek(&bc, VLC_BITS);
                    VLC_INTERN(dst[x], s->vlc[i].table,
                               &bc, VLC_BITS, 3);
                }
                dst += stride;
            }
        }

        switch (pred) {
        case LEFT:
            dst = p->data[i] + j * sheight * stride;
            s->llviddsp.add_left_pred(dst, dst, width, 0);
            dst += stride;
            if (interlaced) {
                s->llviddsp.add_left_pred(dst, dst, width, 0);
                dst += stride;
            }
            for (k = 1 + interlaced; k < height; k++) {
                s->llviddsp.add_left_pred(dst, dst, width, dst[-fake_stride]);
                dst += stride;
            }
            break;
        case GRADIENT:
            dst = p->data[i] + j * sheight * stride;
            s->llviddsp.add_left_pred(dst, dst, width, 0);
            left = lefttop = 0;
            dst += stride;
            if (interlaced) {
                s->llviddsp.add_left_pred(dst, dst, width, 0);
                left = lefttop = 0;
                dst += stride;
            }
            for (k = 1 + interlaced; k < height; k++) {
                top = dst[-fake_stride];
                left = top + dst[0];
                dst[0] = left;
                for (x = 1; x < width; x++) {
                    top = dst[x - fake_stride];
                    lefttop = dst[x - (fake_stride + 1)];
                    left += top - lefttop + dst[x];
                    dst[x] = left;
                }
                dst += stride;
            }
            break;
        case MEDIAN:
            dst = p->data[i] + j * sheight * stride;
            lefttop = left = dst[0];
            s->llviddsp.add_left_pred(dst, dst, width, 0);
            dst += stride;
            if (interlaced) {
                lefttop = left = dst[0];
                s->llviddsp.add_left_pred(dst, dst, width, 0);
                dst += stride;
            }
            for (k = 1 + interlaced; k < height; k++) {
                s->llviddsp.add_median_pred(dst, dst - fake_stride,
                                             dst, width, &left, &lefttop);
                lefttop = left = dst[0];
                dst += stride;
            }
            break;
        default:
            avpriv_request_sample(avctx, "Unknown prediction: %d", pred);
        }
    }

    if (s->decorrelate) {
        int height = FFMIN(s->slice_height, avctx->coded_height - j * s->slice_height);
        int width = avctx->coded_width;
        uint8_t *b = p->data[0] + j * s->slice_height * p->linesize[0];
        uint8_t *g = p->data[1] + j * s->slice_height * p->linesize[1];
        uint8_t *r = p->data[2] + j * s->slice_height * p->linesize[2];

        for (i = 0; i < height; i++) {
            s->llviddsp.add_bytes(b, g, width);
            s->llviddsp.add_bytes(r, g, width);
            b += p->linesize[0];
            g += p->linesize[1];
            r += p->linesize[2];
        }
    }

    return 0;
}

static int build_huffman(AVCodecContext *avctx, BitstreamContext *bc, int max)
{
    MagicYUVContext *s = avctx->priv_data;
    int i = 0, j = 0, k;

    memset(s->len, 0, sizeof(s->len));
    while (bitstream_bits_left(bc) >= 8) {
        int b = bitstream_read(bc, 4);
        int x = bitstream_read(bc, 4);
        int l = bitstream_read(bc, b) + 1;

        for (k = 0; k < l; k++)
            if (j + k < max)
                s->len[i][j + k] = x;

        j += l;
        if (j == max) {
            j = 0;
            if (s->huff_build(s, i, s->vlc_n)) {
                av_log(avctx, AV_LOG_ERROR, "Cannot build Huffman codes\n");
                return AVERROR_INVALIDDATA;
            }
            i++;
            if (i == s->planes) {
                break;
            }
        } else if (j > max) {
            return AVERROR_INVALIDDATA;
        }
    }

    if (i != s->planes) {
        av_log(avctx, AV_LOG_ERROR, "Huffman tables too short\n");
        return AVERROR_INVALIDDATA;
    }

    return 0;
}

static int magy_decode_frame(AVCodecContext *avctx, void *data,
                             int *got_frame, AVPacket *avpkt)
{
    MagicYUVContext *s = avctx->priv_data;
    ThreadFrame frame = { .f = data };
    AVFrame *p = data;
    GetByteContext gbyte;
    BitstreamContext bc;
    uint32_t first_offset, offset, next_offset, header_size, slice_width;
    int width, height, format, version, table_size;
    int ret, i, j;

    bytestream2_init(&gbyte, avpkt->data, avpkt->size);
    if (bytestream2_get_le32(&gbyte) != MKTAG('M', 'A', 'G', 'Y'))
        return AVERROR_INVALIDDATA;

    header_size = bytestream2_get_le32(&gbyte);
    if (header_size < 32 || header_size >= avpkt->size) {
        av_log(avctx, AV_LOG_ERROR,
               "header or packet too small %"PRIu32"\n", header_size);
        return AVERROR_INVALIDDATA;
    }

    version = bytestream2_get_byte(&gbyte);
    if (version != 7) {
        avpriv_request_sample(avctx, "Version %d", version);
        return AVERROR_PATCHWELCOME;
    }

    s->hshift[1] =
    s->vshift[1] =
    s->hshift[2] =
    s->vshift[2] = 0;
    s->decorrelate = 0;
    s->max = 256;
    s->huff_build = huff_build;
    s->magy_decode_slice = magy_decode_slice;

    format = bytestream2_get_byte(&gbyte);
    switch (format) {
    case 0x65:
        avctx->pix_fmt = AV_PIX_FMT_GBRP;
        s->decorrelate = 1;
        break;
    case 0x66:
        avctx->pix_fmt = AV_PIX_FMT_GBRAP;
        s->decorrelate = 1;
        break;
    case 0x67:
        avctx->pix_fmt = AV_PIX_FMT_YUV444P;
        break;
    case 0x68:
        avctx->pix_fmt = AV_PIX_FMT_YUV422P;
        s->hshift[1] =
        s->hshift[2] = 1;
        break;
    case 0x69:
        avctx->pix_fmt = AV_PIX_FMT_YUV420P;
        s->hshift[1] =
        s->vshift[1] =
        s->hshift[2] =
        s->vshift[2] = 1;
        break;
    case 0x6a:
        avctx->pix_fmt = AV_PIX_FMT_YUVA444P;
        break;
    case 0x6b:
        avctx->pix_fmt = AV_PIX_FMT_GRAY8;
        break;
    case 0x6c:
        avctx->pix_fmt = AV_PIX_FMT_YUV422P10;
        s->hshift[1] =
        s->hshift[2] = 1;
        s->max = 1024;
        s->huff_build = huff_build10;
        s->magy_decode_slice = magy_decode_slice10;
        break;
    case 0x6d:
        avctx->pix_fmt = AV_PIX_FMT_GBRP10;
        s->decorrelate = 1;
        s->max = 1024;
        s->huff_build = huff_build10;
        s->magy_decode_slice = magy_decode_slice10;
        break;
    case 0x6e:
        avctx->pix_fmt = AV_PIX_FMT_GBRAP10;
        s->decorrelate = 1;
        s->max = 1024;
        s->huff_build = huff_build10;
        s->magy_decode_slice = magy_decode_slice10;
        break;
    case 0x73:
        avctx->pix_fmt = AV_PIX_FMT_GRAY10;
        s->max = 1024;
        s->huff_build = huff_build10;
        s->magy_decode_slice = magy_decode_slice10;
        break;
    default:
        avpriv_request_sample(avctx, "Format 0x%X", format);
        return AVERROR_PATCHWELCOME;
    }
    s->planes = av_pix_fmt_count_planes(avctx->pix_fmt);

    s->vlc_n = FFMIN(s->max, MAX_VLC_N);

    bytestream2_skip(&gbyte, 1);
    s->color_matrix = bytestream2_get_byte(&gbyte);
    s->flags        = bytestream2_get_byte(&gbyte);
    s->interlaced   = !!(s->flags & 2);
    bytestream2_skip(&gbyte, 3);

    width  = bytestream2_get_le32(&gbyte);
    height = bytestream2_get_le32(&gbyte);
    ret = ff_set_dimensions(avctx, width, height);
    if (ret < 0)
        return ret;

    slice_width = bytestream2_get_le32(&gbyte);
    if (slice_width != avctx->coded_width) {
        avpriv_request_sample(avctx, "Slice width %"PRIu32, slice_width);
        return AVERROR_PATCHWELCOME;
    }
    s->slice_height = bytestream2_get_le32(&gbyte);
    if (s->slice_height <= 0 || s->slice_height > INT_MAX - avctx->coded_height) {
        av_log(avctx, AV_LOG_ERROR,
               "invalid slice height: %d\n", s->slice_height);
        return AVERROR_INVALIDDATA;
    }

    bytestream2_skip(&gbyte, 4);

    s->nb_slices = (avctx->coded_height + s->slice_height - 1) / s->slice_height;
    if (s->nb_slices > INT_MAX / sizeof(Slice)) {
        av_log(avctx, AV_LOG_ERROR,
               "invalid number of slices: %d\n", s->nb_slices);
        return AVERROR_INVALIDDATA;
    }

    for (i = 0; i < s->planes; i++) {
        av_fast_malloc(&s->slices[i], &s->slices_size[i], s->nb_slices * sizeof(Slice));
        if (!s->slices[i])
            return AVERROR(ENOMEM);

        offset = bytestream2_get_le32(&gbyte);
        if (offset >= avpkt->size - header_size)
            return AVERROR_INVALIDDATA;

        if (i == 0)
            first_offset = offset;

        for (j = 0; j < s->nb_slices - 1; j++) {
            s->slices[i][j].start = offset + header_size;

            next_offset = bytestream2_get_le32(&gbyte);
            if (next_offset <= offset || next_offset >= avpkt->size - header_size)
                return AVERROR_INVALIDDATA;

            s->slices[i][j].size = next_offset - offset;
            offset = next_offset;
        }

        s->slices[i][j].start = offset + header_size;
        s->slices[i][j].size  = avpkt->size - s->slices[i][j].start;
    }

    if (bytestream2_get_byte(&gbyte) != s->planes)
        return AVERROR_INVALIDDATA;

    bytestream2_skip(&gbyte, s->nb_slices * s->planes);

    table_size = header_size + first_offset - bytestream2_tell(&gbyte);
    if (table_size < 2)
        return AVERROR_INVALIDDATA;

    ret = bitstream_init8(&bc, avpkt->data + bytestream2_tell(&gbyte), table_size);
    if (ret < 0)
        return ret;

    ret = build_huffman(avctx, &bc, s->max);
    if (ret < 0)
        return ret;

    p->pict_type = AV_PICTURE_TYPE_I;
    p->key_frame = 1;

    if ((ret = ff_thread_get_buffer(avctx, &frame, 0)) < 0)
        return ret;

    s->buf = avpkt->data;
    s->p = p;
    avctx->execute2(avctx, s->magy_decode_slice, NULL, NULL, s->nb_slices);

    if (avctx->pix_fmt == AV_PIX_FMT_GBRP   ||
        avctx->pix_fmt == AV_PIX_FMT_GBRAP  ||
        avctx->pix_fmt == AV_PIX_FMT_GBRP10 ||
        avctx->pix_fmt == AV_PIX_FMT_GBRAP10) {
        FFSWAP(uint8_t*, p->data[0], p->data[1]);
        FFSWAP(int, p->linesize[0], p->linesize[1]);
    } else {
        switch (s->color_matrix) {
        case 1:
            p->colorspace = AVCOL_SPC_BT470BG;
            break;
        case 2:
            p->colorspace = AVCOL_SPC_BT709;
            break;
        }
        p->color_range = (s->flags & 4) ? AVCOL_RANGE_JPEG : AVCOL_RANGE_MPEG;
    }

    *got_frame = 1;

    return avpkt->size;
}

#if HAVE_THREADS
static int magy_init_thread_copy(AVCodecContext *avctx)
{
    MagicYUVContext *s = avctx->priv_data;
    int i;

    memset(s->mem, 0, sizeof(s->mem));

    for (i = 0; i < FF_ARRAY_ELEMS(s->slices); i++) {
        s->slices[i] = NULL;
        s->slices_size[i] = 0;
    }

    for (i = 0; i < 12; i++)
        s->vlc[i].table = NULL;

    return 0;
}
#endif

static av_cold int magy_decode_init(AVCodecContext *avctx)
{
    MagicYUVContext *s = avctx->priv_data;
    ff_llviddsp_init(&s->llviddsp);
    return 0;
}

static av_cold int magy_decode_end(AVCodecContext *avctx)
{
    MagicYUVContext * const s = avctx->priv_data;
    int i;

    for (i = 0; i < FF_ARRAY_ELEMS(s->slices); i++) {
        av_freep(&s->slices[i]);
        s->slices_size[i] = 0;
    }
    for (i = 0; i < 12; i++)
        ff_free_vlc(&s->vlc[i]);
    for (i = 0; i < 4; i++)
        av_freep(&s->mem[i]);

    return 0;
}

AVCodec ff_magicyuv_decoder = {
    .name             = "magicyuv",
    .long_name        = NULL_IF_CONFIG_SMALL("MagicYUV video"),
    .type             = AVMEDIA_TYPE_VIDEO,
    .id               = AV_CODEC_ID_MAGICYUV,
    .priv_data_size   = sizeof(MagicYUVContext),
    .init             = magy_decode_init,
    .init_thread_copy = ONLY_IF_THREADS_ENABLED(magy_init_thread_copy),
    .close            = magy_decode_end,
    .decode           = magy_decode_frame,
    .capabilities     = AV_CODEC_CAP_DR1 |
                        AV_CODEC_CAP_FRAME_THREADS |
                        AV_CODEC_CAP_SLICE_THREADS,
    .caps_internal    = FF_CODEC_CAP_INIT_THREADSAFE,
};
