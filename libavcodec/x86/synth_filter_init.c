/*
 * Copyright (c) 2012-2014 Christophe Gisquet <christophe.gisquet@gmail.com>
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

#include "libavutil/attributes.h"
#include "libavutil/cpu.h"
#include "libavutil/x86/cpu.h"
#include "libavcodec/synth_filter.h"

#define FP_SYNTH_FILTER_FUNC(opt)                                                 \
void ff_fp_synth_filter_inner32_##opt(int32_t *synth_buf_ptr, int32_t synth_buf2[64], \
                                   const int32_t window[1024],                 \
                                   int32_t out[64], intptr_t offset);          \
static void fp_synth_filter64_##opt(DCADCTContext *imdct,                      \
                                    int32_t *synth_buf_ptr, int *synth_buf_offset, \
                                    int32_t synth_buf2[64], const int32_t window[1024], \
                                    int32_t out[64], const int32_t in[64])     \
{                                                                              \
    int32_t *synth_buf= synth_buf_ptr + *synth_buf_offset;                     \
                                                                               \
    imdct->imdct_half[1](synth_buf, in);                                       \
                                                                               \
    ff_fp_synth_filter_inner32_##opt(synth_buf, synth_buf2, window,            \
                                     out, *synth_buf_offset);                  \
                                                                               \
    *synth_buf_offset = (*synth_buf_offset - 64) & 1023;                       \
}                                                                              \
                                                                               \
void ff_fp_synth_filter_inner16_##opt(int32_t *synth_buf_ptr, int32_t synth_buf2[32], \
                                      const int32_t window[512],               \
                                      int32_t out[32], intptr_t offset);       \
static void fp_synth_filter_##opt(DCADCTContext *imdct,                        \
                                  int32_t *synth_buf_ptr, int *synth_buf_offset, \
                                  int32_t synth_buf2[64], const int32_t window[1024], \
                                  int32_t out[64], const int32_t in[64])       \
{                                                                              \
    int32_t *synth_buf= synth_buf_ptr + *synth_buf_offset;                     \
                                                                               \
    imdct->imdct_half[0](synth_buf, in);                                       \
                                                                               \
    ff_fp_synth_filter_inner16_##opt(synth_buf, synth_buf2, window,            \
                                     out, *synth_buf_offset);                  \
                                                                               \
    *synth_buf_offset = (*synth_buf_offset - 32) & 511;                        \
}

#define SYNTH_FILTER_FUNC(opt)                                                 \
void ff_synth_filter_inner32_##opt(float *synth_buf_ptr, float synth_buf2[64], \
                                   const float window[1024],                   \
                                   float out[64], intptr_t offset, float scale); \
static void synth_filter64_##opt(FFTContext *imdct,                            \
                                 float *synth_buf_ptr, int *synth_buf_offset,  \
                                 float synth_buf2[64], const float window[1024], \
                                 float out[64], const float in[64], float scale) \
{                                                                              \
    float *synth_buf= synth_buf_ptr + *synth_buf_offset;                       \
                                                                               \
    imdct->imdct_half(imdct, synth_buf, in);                                   \
                                                                               \
    ff_synth_filter_inner32_##opt(synth_buf, synth_buf2, window,               \
                                  out, *synth_buf_offset, scale);              \
                                                                               \
    *synth_buf_offset = (*synth_buf_offset - 64) & 1023;                       \
}                                                                              \
                                                                               \
void ff_synth_filter_inner16_##opt(float *synth_buf_ptr, float synth_buf2[32], \
                                 const float window[512],                      \
                                 float out[32], intptr_t offset, float scale); \
static void synth_filter_##opt(FFTContext *imdct,                              \
                               float *synth_buf_ptr, int *synth_buf_offset,    \
                               float synth_buf2[32], const float window[512],  \
                               float out[32], const float in[32], float scale) \
{                                                                              \
    float *synth_buf= synth_buf_ptr + *synth_buf_offset;                       \
                                                                               \
    imdct->imdct_half(imdct, synth_buf, in);                                   \
                                                                               \
    ff_synth_filter_inner16_##opt(synth_buf, synth_buf2, window,               \
                                out, *synth_buf_offset, scale);                \
                                                                               \
    *synth_buf_offset = (*synth_buf_offset - 32) & 511;                        \
}                                                                              \

#if HAVE_YASM
#if ARCH_X86_32
SYNTH_FILTER_FUNC(sse)
#endif
SYNTH_FILTER_FUNC(sse2)
SYNTH_FILTER_FUNC(avx)
SYNTH_FILTER_FUNC(fma3)

FP_SYNTH_FILTER_FUNC(sse4)
#endif /* HAVE_YASM */

av_cold void ff_synth_filter_init_x86(SynthFilterContext *s)
{
#if HAVE_YASM
    int cpu_flags = av_get_cpu_flags();

#if ARCH_X86_32
    if (EXTERNAL_SSE(cpu_flags)) {
        s->synth_filter_float = synth_filter_sse;
        s->synth_filter_float_64 = synth_filter64_sse;
    }
#endif
    if (EXTERNAL_SSE2(cpu_flags)) {
        s->synth_filter_float = synth_filter_sse2;
        s->synth_filter_float_64 = synth_filter64_sse2;
    }
    if (EXTERNAL_SSE4(cpu_flags)) {
        s->synth_filter_fixed    = fp_synth_filter_sse4;
        s->synth_filter_fixed_64 = fp_synth_filter64_sse4;
    }
    if (EXTERNAL_AVX_FAST(cpu_flags)) {
        s->synth_filter_float = synth_filter_avx;
        s->synth_filter_float_64 = synth_filter64_avx;
    }
    if (EXTERNAL_FMA3_FAST(cpu_flags)) {
        s->synth_filter_float = synth_filter_fma3;
        s->synth_filter_float_64 = synth_filter64_fma3;
    }
#endif /* HAVE_YASM */
}
