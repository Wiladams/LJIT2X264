--[[
/*****************************************************************************
 * x264.h: x264 public header
 *****************************************************************************
 * Copyright (C) 2003-2012 x264 project
 *
 * Authors: Laurent Aimar <fenrir@via.ecp.fr>
 *          Loren Merritt <lorenm@u.washington.edu>
 *          Jason Garrett-Glaser <darkshikari@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at licensing@x264.com.
 *****************************************************************************/
--]]

local ffi = require "ffi"
local C = ffi.C
local bit = require "bit"
local lshift = bit.lshift

local libx264 = ffi.load("libx264-120.dll")
local x264 = {
	lib = libx264
	}

--[[
#include "x264_config.h"
--]]
local X264_BUILD = 120





ffi.cdef[[
/* x264_t:
 *      opaque handler for encoder */
typedef struct x264_t x264_t;

/****************************************************************************
 * NAL structure and functions
 ****************************************************************************/


enum nal_unit_type_e
{
    NAL_UNKNOWN     = 0,
    NAL_SLICE       = 1,
    NAL_SLICE_DPA   = 2,
    NAL_SLICE_DPB   = 3,
    NAL_SLICE_DPC   = 4,
    NAL_SLICE_IDR   = 5,    /* ref_idc != 0 */
    NAL_SEI         = 6,    /* ref_idc == 0 */
    NAL_SPS         = 7,
    NAL_PPS         = 8,
    NAL_AUD         = 9,
    NAL_FILLER      = 12,
    /* ref_idc == 0 for 6,9,10,11,12 */
};
enum nal_priority_e
{
    NAL_PRIORITY_DISPOSABLE = 0,
    NAL_PRIORITY_LOW        = 1,
    NAL_PRIORITY_HIGH       = 2,
    NAL_PRIORITY_HIGHEST    = 3,
};

/* The data within the payload is already NAL-encapsulated; the ref_idc and type
 * are merely in the struct for easy access by the calling application.
 * All data returned in an x264_nal_t, including the data in p_payload, is no longer
 * valid after the next call to x264_encoder_encode.  Thus it must be used or copied
 * before calling x264_encoder_encode or x264_encoder_headers again. */
typedef struct
{
    int i_ref_idc;  /* nal_priority_e */
    int i_type;     /* nal_unit_type_e */
    int b_long_startcode;
    int i_first_mb; /* If this NAL is a slice, the index of the first MB in the slice. */
    int i_last_mb;  /* If this NAL is a slice, the index of the last MB in the slice. */

    /* Size of payload in bytes. */
    int     i_payload;
    /* If param->b_annexb is set, Annex-B bytestream with startcode.
     * Otherwise, startcode is replaced with a 4-byte size.
     * This size is the size used in mp4/similar muxing; it is equal to i_payload-4 */
    uint8_t *p_payload;
} x264_nal_t;



/* Zones: override ratecontrol or other options for specific sections of the video.
 * See x264_encoder_reconfig() for which options can be changed.
 * If zones overlap, whichever comes later in the list takes precedence. */
typedef struct
{
    int i_start, i_end; /* range of frame numbers */
    int b_force_qp; /* whether to use qp vs bitrate factor */
    int i_qp;
    float f_bitrate_factor;
    struct x264_param_t *param;
} x264_zone_t;

typedef struct x264_param_t
{
    /* CPU flags */
    unsigned int cpu;
    int         i_threads;       /* encode multiple frames in parallel */
    int         b_sliced_threads;  /* Whether to use slice-based threading. */
    int         b_deterministic; /* whether to allow non-deterministic optimizations when threaded */
    int         b_cpu_independent; /* force canonical behavior rather than cpu-dependent optimal algorithms */
    int         i_sync_lookahead; /* threaded lookahead buffer */

    /* Video Properties */
    int         i_width;
    int         i_height;
    int         i_csp;         /* CSP of encoded bitstream */
    int         i_level_idc;
    int         i_frame_total; /* number of frames to encode if known, else 0 */

    /* NAL HRD
     * Uses Buffering and Picture Timing SEIs to signal HRD
     * The HRD in H.264 was not designed with VFR in mind.
     * It is therefore not recommendeded to use NAL HRD with VFR.
     * Furthermore, reconfiguring the VBV (via x264_encoder_reconfig)
     * will currently generate invalid HRD. */
    int         i_nal_hrd;

    struct
    {
        /* they will be reduced to be 0 < x <= 65535 and prime */
        int         i_sar_height;
        int         i_sar_width;

        int         i_overscan;    /* 0=undef, 1=no overscan, 2=overscan */

        /* see h264 annex E for the values of the following */
        int         i_vidformat;
        int         b_fullrange;
        int         i_colorprim;
        int         i_transfer;
        int         i_colmatrix;
        int         i_chroma_loc;    /* both top & bottom */
    } vui;

    /* Bitstream parameters */
    int         i_frame_reference;  /* Maximum number of reference frames */
    int         i_dpb_size;         /* Force a DPB size larger than that implied by B-frames and reference frames.
                                     * Useful in combination with interactive error resilience. */
    int         i_keyint_max;       /* Force an IDR keyframe at this interval */
    int         i_keyint_min;       /* Scenecuts closer together than this are coded as I, not IDR. */
    int         i_scenecut_threshold; /* how aggressively to insert extra I frames */
    int         b_intra_refresh;    /* Whether or not to use periodic intra refresh instead of IDR frames. */

    int         i_bframe;   /* how many b-frame between 2 references pictures */
    int         i_bframe_adaptive;
    int         i_bframe_bias;
    int         i_bframe_pyramid;   /* Keep some B-frames as references: 0=off, 1=strict hierarchical, 2=normal */
    int         b_open_gop;
    int         b_bluray_compat;

    int         b_deblocking_filter;
    int         i_deblocking_filter_alphac0;    /* [-6, 6] -6 light filter, 6 strong */
    int         i_deblocking_filter_beta;       /* [-6, 6]  idem */

    int         b_cabac;
    int         i_cabac_init_idc;

    int         b_interlaced;
    int         b_constrained_intra;

    int         i_cqm_preset;
    char        *psz_cqm_file;      /* JM format */
    uint8_t     cqm_4iy[16];        /* used only if i_cqm_preset == X264_CQM_CUSTOM */
    uint8_t     cqm_4py[16];
    uint8_t     cqm_4ic[16];
    uint8_t     cqm_4pc[16];
    uint8_t     cqm_8iy[64];
    uint8_t     cqm_8py[64];
    uint8_t     cqm_8ic[64];
    uint8_t     cqm_8pc[64];

    /* Log */
    void        (*pf_log)( void *, int i_level, const char *psz, va_list );
    void        *p_log_private;
    int         i_log_level;
    int         b_visualize;
    char        *psz_dump_yuv;  /* filename for reconstructed frames */

    /* Encoder analyser parameters */
    struct
    {
        unsigned int intra;     /* intra partitions */
        unsigned int inter;     /* inter partitions */

        int          b_transform_8x8;
        int          i_weighted_pred; /* weighting for P-frames */
        int          b_weighted_bipred; /* implicit weighting for B-frames */
        int          i_direct_mv_pred; /* spatial vs temporal mv prediction */
        int          i_chroma_qp_offset;

        int          i_me_method; /* motion estimation algorithm to use (X264_ME_*) */
        int          i_me_range; /* integer pixel motion estimation search range (from predicted mv) */
        int          i_mv_range; /* maximum length of a mv (in pixels). -1 = auto, based on level */
        int          i_mv_range_thread; /* minimum space between threads. -1 = auto, based on number of threads. */
        int          i_subpel_refine; /* subpixel motion estimation quality */
        int          b_chroma_me; /* chroma ME for subpel and mode decision in P-frames */
        int          b_mixed_references; /* allow each mb partition to have its own reference number */
        int          i_trellis;  /* trellis RD quantization */
        int          b_fast_pskip; /* early SKIP detection on P-frames */
        int          b_dct_decimate; /* transform coefficient thresholding on P-frames */
        int          i_noise_reduction; /* adaptive pseudo-deadzone */
        float        f_psy_rd; /* Psy RD strength */
        float        f_psy_trellis; /* Psy trellis strength */
        int          b_psy; /* Toggle all psy optimizations */

        /* the deadzone size that will be used in luma quantization */
        int          i_luma_deadzone[2]; /* {inter, intra} */

        int          b_psnr;    /* compute and print PSNR stats */
        int          b_ssim;    /* compute and print SSIM stats */
    } analyse;

    /* Rate control parameters */
    struct
    {
        int         i_rc_method;    /* X264_RC_* */

        int         i_qp_constant;  /* 0 to (51 + 6*(x264_bit_depth-8)). 0=lossless */
        int         i_qp_min;       /* min allowed QP value */
        int         i_qp_max;       /* max allowed QP value */
        int         i_qp_step;      /* max QP step between frames */

        int         i_bitrate;
        float       f_rf_constant;  /* 1pass VBR, nominal QP */
        float       f_rf_constant_max;  /* In CRF mode, maximum CRF as caused by VBV */
        float       f_rate_tolerance;
        int         i_vbv_max_bitrate;
        int         i_vbv_buffer_size;
        float       f_vbv_buffer_init; /* <=1: fraction of buffer_size. >1: kbit */
        float       f_ip_factor;
        float       f_pb_factor;

        int         i_aq_mode;      /* psy adaptive QP. (X264_AQ_*) */
        float       f_aq_strength;
        int         b_mb_tree;      /* Macroblock-tree ratecontrol. */
        int         i_lookahead;

        /* 2pass */
        int         b_stat_write;   /* Enable stat writing in psz_stat_out */
        char        *psz_stat_out;
        int         b_stat_read;    /* Read stat from psz_stat_in and use it */
        char        *psz_stat_in;

        /* 2pass params (same as ffmpeg ones) */
        float       f_qcompress;    /* 0.0 => cbr, 1.0 => constant qp */
        float       f_qblur;        /* temporally blur quants */
        float       f_complexity_blur; /* temporally blur complexity */
        x264_zone_t *zones;         /* ratecontrol overrides */
        int         i_zones;        /* number of zone_t's */
        char        *psz_zones;     /* alternate method of specifying zones */
    } rc;

    /* Cropping Rectangle parameters: added to those implicitly defined by
       non-mod16 video resolutions. */
    struct
    {
        unsigned int i_left;
        unsigned int i_top;
        unsigned int i_right;
        unsigned int i_bottom;
    } crop_rect;

    /* frame packing arrangement flag */
    int i_frame_packing;

    /* Muxing parameters */
    int b_aud;                  /* generate access unit delimiters */
    int b_repeat_headers;       /* put SPS/PPS before each keyframe */
    int b_annexb;               /* if set, place start codes (4 bytes) before NAL units,
                                 * otherwise place size (4 bytes) before NAL units. */
    int i_sps_id;               /* SPS and PPS id number */
    int b_vfr_input;            /* VFR input.  If 1, use timebase and timestamps for ratecontrol purposes.
                                 * If 0, use fps only. */
    int b_pulldown;             /* use explicity set timebase for CFR */
    uint32_t i_fps_num;
    uint32_t i_fps_den;
    uint32_t i_timebase_num;    /* Timebase numerator */
    uint32_t i_timebase_den;    /* Timebase denominator */

    int b_tff;

    /* Pulldown:
     * The correct pic_struct must be passed with each input frame.
     * The input timebase should be the timebase corresponding to the output framerate. This should be constant.
     * e.g. for 3:2 pulldown timebase should be 1001/30000
     * The PTS passed with each frame must be the PTS of the frame after pulldown is applied.
     * Frame doubling and tripling require b_vfr_input set to zero (see H.264 Table D-1)
     *
     * Pulldown changes are not clearly defined in H.264. Therefore, it is the calling app's responsibility to manage this.
     */

    int b_pic_struct;

    /* Fake Interlaced.
     *
     * Used only when b_interlaced=0. Setting this flag makes it possible to flag the stream as PAFF interlaced yet
     * encode all frames progessively. It is useful for encoding 25p and 30p Blu-Ray streams.
     */

    int b_fake_interlaced;

    /* Slicing parameters */
    int i_slice_max_size;    /* Max size per slice in bytes; includes estimated NAL overhead. */
    int i_slice_max_mbs;     /* Max number of MBs per slice; overrides i_slice_count. */
    int i_slice_count;       /* Number of slices per frame: forces rectangular slices. */

    /* Optional callback for freeing this x264_param_t when it is done being used.
     * Only used when the x264_param_t sits in memory for an indefinite period of time,
     * i.e. when an x264_param_t is passed to x264_t in an x264_picture_t or in zones.
     * Not used when x264_encoder_reconfig is called directly. */
    void (*param_free)( void* );

    /* Optional low-level callback for low-latency encoding.  Called for each output NAL unit
     * immediately after the NAL unit is finished encoding.  This allows the calling application
     * to begin processing video data (e.g. by sending packets over a network) before the frame
     * is done encoding.
     *
     * This callback MUST do the following in order to work correctly:
     * 1) Have available an output buffer of at least size nal->i_payload*3/2 + 5 + 16.
     * 2) Call x264_nal_encode( h, dst, nal ), where dst is the output buffer.
     * After these steps, the content of nal is valid and can be used in the same way as if
     * the NAL unit were output by x264_encoder_encode.
     *
     * This does not need to be synchronous with the encoding process: the data pointed to
     * by nal (both before and after x264_nal_encode) will remain valid until the next
     * x264_encoder_encode call.  The callback must be re-entrant.
     *
     * This callback does not work with frame-based threads; threads must be disabled
     * or sliced-threads enabled.  This callback also does not work as one would expect
     * with HRD -- since the buffering period SEI cannot be calculated until the frame
     * is finished encoding, it will not be sent via this callback.
     *
     * Note also that the NALs are not necessarily returned in order when sliced threads is
     * enabled.  Accordingly, the variable i_first_mb and i_last_mb are available in
     * x264_nal_t to help the calling application reorder the slices if necessary.
     *
     * When this callback is enabled, x264_encoder_encode does not return valid NALs;
     * the calling application is expected to acquire all output NALs through the callback.
     *
     * It is generally sensible to combine this callback with a use of slice-max-mbs or
     * slice-max-size. */
    void (*nalu_process) ( x264_t *h, x264_nal_t *nal );
} x264_param_t;
]]

ffi.cdef[[
void x264_nal_encode( x264_t *h, uint8_t *dst, x264_nal_t *nal );

/****************************************************************************
 * H.264 level restriction information
 ****************************************************************************/

typedef struct
{
    int level_idc;
    int mbps;        /* max macroblock processing rate (macroblocks/sec) */
    int frame_size;  /* max frame size (macroblocks) */
    int dpb;         /* max decoded picture buffer (bytes) */
    int bitrate;     /* max bitrate (kbit/sec) */
    int cpb;         /* max vbv buffer (kbit) */
    int mv_range;    /* max vertical mv component range (pixels) */
    int mvs_per_2mb; /* max mvs per 2 consecutive mbs. */
    int slice_rate;  /* ?? */
    int mincr;       /* min compression ratio */
    int bipred8x8;   /* limit bipred to >=8x8 */
    int direct8x8;   /* limit b_direct to >=8x8 */
    int frame_only;  /* forbid interlacing */
} x264_level_t;

/* all of the levels defined in the standard, terminated by .level_idc=0 */
extern const x264_level_t x264_levels[];

]]


ffi.cdef[[
/****************************************************************************
 * Basic parameter handling functions
 ****************************************************************************/

/* x264_param_default:
 *      fill x264_param_t with default values and do CPU detection */
void    x264_param_default( x264_param_t * );

int x264_param_parse( x264_param_t *, const char *name, const char *value );

/****************************************************************************
 * Advanced parameter handling functions
 ****************************************************************************/

/* These functions expose the full power of x264's preset-tune-profile system for
 * easy adjustment of large numbers of internal parameters.
 *
 * In order to replicate x264CLI's option handling, these functions MUST be called
 * in the following order:
 * 1) x264_param_default_preset
 * 2) Custom user options (via param_parse or directly assigned variables)
 * 3) x264_param_apply_fastfirstpass
 * 4) x264_param_apply_profile
 *
 * Additionally, x264CLI does not apply step 3 if the preset chosen is "placebo"
 * or --slow-firstpass is set. */

int     x264_param_default_preset( x264_param_t *, const char *preset, const char *tune );
void    x264_param_apply_fastfirstpass( x264_param_t * );
int     x264_param_apply_profile( x264_param_t *, const char *profile );
]]

--[[
/* x264_param_parse:
 *  set one parameter by name.
 *  returns 0 on success, or returns one of the following errors.
 *  note: BAD_VALUE occurs only if it can't even parse the value,
 *  numerical range is not checked until x264_encoder_open() or
 *  x264_encoder_reconfig().
 *  value=NULL means "true" for boolean options, but is a BAD_VALUE for non-booleans. */
--]]
X264_PARAM_BAD_NAME  = -1
X264_PARAM_BAD_VALUE = -2



ffi.cdef[[
/****************************************************************************
 * Picture structures and functions
 ****************************************************************************/

/* x264_bit_depth:
 *      Specifies the number of bits per pixel that x264 uses. This is also the
 *      bit depth that x264 encodes in. If this value is > 8, x264 will read
 *      two bytes of input data for each pixel sample, and expect the upper
 *      (16-x264_bit_depth) bits to be zero.
 *      Note: The flag X264_CSP_HIGH_DEPTH must be used to specify the
 *      colorspace depth as well. */
extern const int x264_bit_depth;

/* x264_chroma_format:
 *      Specifies the chroma formats that x264 supports encoding. When this
 *      value is non-zero, then it represents a X264_CSP_* that is the only
 *      chroma format that x264 supports encoding. If the value is 0 then
 *      there are no restrictions. */
extern const int x264_chroma_format;

enum pic_struct_e
{
    PIC_STRUCT_AUTO              = 0, // automatically decide (default)
    PIC_STRUCT_PROGRESSIVE       = 1, // progressive frame
    // "TOP" and "BOTTOM" are not supported in x264 (PAFF only)
    PIC_STRUCT_TOP_BOTTOM        = 4, // top field followed by bottom
    PIC_STRUCT_BOTTOM_TOP        = 5, // bottom field followed by top
    PIC_STRUCT_TOP_BOTTOM_TOP    = 6, // top field, bottom field, top field repeated
    PIC_STRUCT_BOTTOM_TOP_BOTTOM = 7, // bottom field, top field, bottom field repeated
    PIC_STRUCT_DOUBLE            = 8, // double frame
    PIC_STRUCT_TRIPLE            = 9, // triple frame
};

typedef struct
{
    double cpb_initial_arrival_time;
    double cpb_final_arrival_time;
    double cpb_removal_time;

    double dpb_output_time;
} x264_hrd_t;

/* Arbitrary user SEI:
 * Payload size is in bytes and the payload pointer must be valid.
 * Payload types and syntax can be found in Annex D of the H.264 Specification.
 * SEI payload alignment bits as described in Annex D must be included at the
 * end of the payload if needed.
 * The payload should not be NAL-encapsulated.
 * Payloads are written first in order of input, apart from in the case when HRD
 * is enabled where payloads are written after the Buffering Period SEI. */

typedef struct
{
    int payload_size;
    int payload_type;
    uint8_t *payload;
} x264_sei_payload_t;

typedef struct
{
    int num_payloads;
    x264_sei_payload_t *payloads;
    /* In: optional callback to free each payload AND x264_sei_payload_t when used. */
    void (*sei_free)( void* );
} x264_sei_t;

typedef struct
{
    int     i_csp;       /* Colorspace */
    int     i_plane;     /* Number of image planes */
    int     i_stride[4]; /* Strides for each plane */
    uint8_t *plane[4];   /* Pointers to each plane */
} x264_image_t;

typedef struct
{
    /* In: an array of quantizer offsets to be applied to this image during encoding.
     *     These are added on top of the decisions made by x264.
     *     Offsets can be fractional; they are added before QPs are rounded to integer.
     *     Adaptive quantization must be enabled to use this feature.  Behavior if quant
     *     offsets differ between encoding passes is undefined.
     *
     *     Array contains one offset per macroblock, in raster scan order.  In interlaced
     *     mode, top-field MBs and bottom-field MBs are interleaved at the row level. */
    float *quant_offsets;
    /* In: optional callback to free quant_offsets when used.
     *     Useful if one wants to use a different quant_offset array for each frame. */
    void (*quant_offsets_free)( void* );
} x264_image_properties_t;

typedef struct
{
    int     i_type;
    int     i_qpplus1;
    int     i_pic_struct;
    int     b_keyframe;
    int64_t i_pts;
    int64_t i_dts;
    x264_param_t *param;
    x264_image_t img;
     x264_image_properties_t prop;
    x264_hrd_t hrd_timing;
    x264_sei_t extra_sei;
    void *opaque;
} x264_picture_t;

void x264_picture_init( x264_picture_t *pic );
int x264_picture_alloc( x264_picture_t *pic, int i_csp, int i_width, int i_height );
void x264_picture_clean( x264_picture_t *pic );
]]



--[[
/* Force a link error in the case of linking against an incompatible API version.
 * Glue #defines exist to force correct macro expansion; the final output of the macro
 * is x264_encoder_open_##X264_BUILD (for purposes of dlopen). */
#define x264_encoder_glue1(x,y) x##y
#define x264_encoder_glue2(x,y) x264_encoder_glue1(x,y)
#define x264_encoder_open x264_encoder_glue2(x264_encoder_open_,X264_BUILD)
--]]


ffi.cdef[[
/****************************************************************************
 * Encoder functions
 ****************************************************************************/

x264_t *x264_encoder_open_120( x264_param_t * );

int     x264_encoder_reconfig( x264_t *, x264_param_t * );

void    x264_encoder_parameters( x264_t *, x264_param_t * );

int     x264_encoder_headers( x264_t *, x264_nal_t **pp_nal, int *pi_nal );

int     x264_encoder_encode( x264_t *, x264_nal_t **pp_nal, int *pi_nal, x264_picture_t *pic_in, x264_picture_t *pic_out );

void    x264_encoder_close  ( x264_t * );

int     x264_encoder_delayed_frames( x264_t * );

int     x264_encoder_maximum_delayed_frames( x264_t *h );

void    x264_encoder_intra_refresh( x264_t * );

int x264_encoder_invalidate_reference( x264_t *, int64_t pts );
]]

x264.encoder_open = function (params)
	if not params then
		error("Must include params for encoder_open")
	end

	return libx264.x264_encoder_open_120(params)
end


--****************************************************************************
-- Encoder parameters
--****************************************************************************
-- CPU flags

x264.CPU_CACHELINE_32    = 0x0000001  -- avoid memory loads that span the border between two cachelines
x264.CPU_CACHELINE_64    = 0x0000002  -- 32/64 is the size of a cacheline in bytes
x264.CPU_ALTIVEC         = 0x0000004
x264.CPU_MMX             = 0x0000008
x264.CPU_MMX2            = 0x0000010  -- MMX2 aka MMXEXT aka ISSE
x264.CPU_MMXEXT          = x264.CPU_MMX2
x264.CPU_SSE             = 0x0000020
x264.CPU_SSE2            = 0x0000040
x264.CPU_SSE2_IS_SLOW    = 0x0000080  -- avoid most SSE2 functions on Athlon64
x264.CPU_SSE2_IS_FAST    = 0x0000100  -- a few functions are only faster on Core2 and Phenom
x264.CPU_SSE3            = 0x0000200
x264.CPU_SSSE3           = 0x0000400
x264.CPU_SHUFFLE_IS_FAST = 0x0000800  -- Penryn, Nehalem, and Phenom have fast shuffle units
x264.CPU_STACK_MOD4      = 0x0001000  -- if stack is only mod4 and not mod16
x264.CPU_SSE4            = 0x0002000  -- SSE4.1
x264.CPU_SSE42           = 0x0004000  -- SSE4.2
x264.CPU_SSE_MISALIGN    = 0x0008000  -- Phenom support for misaligned SSE instruction arguments
x264.CPU_LZCNT           = 0x0010000  -- Phenom support for "leading zero count" instruction.
x264.CPU_ARMV6           = 0x0020000
x264.CPU_NEON            = 0x0040000  -- ARM NEON
x264.CPU_FAST_NEON_MRC   = 0x0080000  -- Transfer from NEON to ARM register is fast (Cortex-A9)
x264.CPU_SLOW_CTZ        = 0x0100000  -- BSR/BSF x86 instructions are really slow on some CPUs
x264.CPU_SLOW_ATOM       = 0x0200000  -- The Atom just sucks
x264.CPU_AVX             = 0x0400000  -- AVX support: requires OS support even if YMM registers aren't used.
x264.CPU_XOP             = 0x0800000  -- AMD XOP
x264.CPU_FMA4            = 0x1000000  -- AMD FMA4
x264.CPU_AVX2            = 0x2000000  -- AVX2
x264.CPU_FMA3            = 0x4000000  -- Intel FMA3
x264.CPU_BMI1            = 0x8000000  -- BMI1
x264.CPU_BMI2            = 0x10000000  -- BMI2
x264.CPU_TBM             = 0x20000000  -- AMD TBM


-- Analyse flags
x264.ANALYSE_I4x4       	= 0x0001  -- Analyse i4x4
x264.ANALYSE_I8x8       	= 0x0002  -- Analyse i8x8 (requires 8x8 transform)
x264.ANALYSE_PSUB16x16  	= 0x0010  -- Analyse p16x8, p8x16 and p8x8
x264.ANALYSE_PSUB8x8    	= 0x0020  -- Analyse p8x4, p4x8, p4x4
x264.ANALYSE_BSUB16x16  	= 0x0100  -- Analyse b16x8, b8x16 and b8x8
x264.DIRECT_PRED_NONE       = 0
x264.DIRECT_PRED_SPATIAL    = 1
x264.DIRECT_PRED_TEMPORAL   = 2
x264.DIRECT_PRED_AUTO       = 3
x264.ME_DIA                 = 0
x264.ME_HEX                 = 1
x264.ME_UMH                 = 2
x264.ME_ESA                 = 3
x264.ME_TESA                = 4
x264.CQM_FLAT               = 0
x264.CQM_JVT                = 1
x264.CQM_CUSTOM             = 2
x264.RC_CQP                 = 0
x264.RC_CRF                 = 1
x264.RC_ABR                 = 2
x264.QP_AUTO                = 0
x264.AQ_NONE                = 0
x264.AQ_VARIANCE            = 1
x264.AQ_AUTOVARIANCE        = 2
x264.B_ADAPT_NONE           = 0
x264.B_ADAPT_FAST           = 1
x264.B_ADAPT_TRELLIS        = 2
x264.WEIGHTP_NONE           = 0
x264.WEIGHTP_SIMPLE         = 1
x264.WEIGHTP_SMART          = 2
x264.B_PYRAMID_NONE         = 0
x264.B_PYRAMID_STRICT       = 1
x264.B_PYRAMID_NORMAL       = 2
x264.KEYINT_MIN_AUTO        = 0
x264.KEYINT_MAX_INFINITE    = lshift(1,30)


x264.direct_pred_names = { "none", "spatial", "temporal", "auto"};
x264.motion_est_names = { "dia", "hex", "umh", "esa", "tesa"};
x264.b_pyramid_names = { "none", "strict", "normal"};
x264.overscan_names = { "undef", "show", "crop"};
x264.vidformat_names = { "component", "pal", "ntsc", "secam", "mac", "undef"};
x264.fullrange_names = { "off", "on"};
x264.colorprim_names = { "", "bt709", "undef", "", "bt470m", "bt470bg", "smpte170m", "smpte240m", "film"};
x264.transfer_names = { "", "bt709", "undef", "", "bt470m", "bt470bg", "smpte170m", "smpte240m", "linear", "log100", "log316"};
x264.colmatrix_names = { "GBR", "bt709", "undef", "", "fcc", "bt470bg", "smpte170m", "smpte240m", "YCgCo"};
x264.nal_hrd_names = { "none", "vbr", "cbr"};


-- Colorspace type
x264.CSP_MASK           = 0x00ff  --
x264.CSP_NONE           = 0x0000  -- Invalid mode
x264.CSP_I420           = 0x0001  -- yuv 4:2:0 planar
x264.CSP_YV12           = 0x0002  -- yvu 4:2:0 planar
x264.CSP_NV12           = 0x0003  -- yuv 4:2:0, with one y plane and one packed u+v
x264.CSP_I422           = 0x0004  -- yuv 4:2:2 planar
x264.CSP_YV16           = 0x0005  -- yvu 4:2:2 planar
x264.CSP_NV16           = 0x0006  -- yuv 4:2:2, with one y plane and one packed u+v
x264.CSP_I444           = 0x0007  -- yuv 4:4:4 planar
x264.CSP_YV24           = 0x0008  -- yvu 4:4:4 planar
x264.CSP_BGR            = 0x0009  -- packed bgr 24bits
x264.CSP_BGRA           = 0x000a  -- packed bgr 32bits
x264.CSP_RGB            = 0x000b  -- packed rgb 24bits
x264.CSP_MAX            = 0x000c  -- end of list
x264.CSP_VFLIP          = 0x1000  -- the csp is vertically flipped
x264.CSP_HIGH_DEPTH     = 0x2000  -- the csp has a depth of 16 bits per pixel component


-- Slice type
x264.TYPE_AUTO          = 0x0000  -- Let x264 choose the right type
x264.TYPE_IDR           = 0x0001
x264.TYPE_I             = 0x0002
x264.TYPE_P             = 0x0003
x264.TYPE_BREF          = 0x0004  -- Non-disposable B-frame
x264.TYPE_B             = 0x0005
x264.TYPE_KEYFRAME      = 0x0006  -- IDR or I depending on b_open_gop option


function IS_X264_TYPE_I(x)
	return ((x)==x264.TYPE_I or (x)==x264.TYPE_IDR)
end

function IS_X264_TYPE_B(x)
	return ((x)==x264.TYPE_B or (x)==x264.TYPE_BREF)
end



-- Log level
x264.LOG_NONE          =-1
x264.LOG_ERROR         = 0
x264.LOG_WARNING       = 1
x264.LOG_INFO          = 2
x264.LOG_DEBUG         = 3

-- Threading
x264.THREADS_AUTO 			= 0 	-- Automatically select optimal number of threads
x264.SYNC_LOOKAHEAD_AUTO 	= -1 	-- Automatically select optimal lookahead thread buffer size */


-- HRD
x264.NAL_HRD_NONE            = 0
x264.NAL_HRD_VBR             = 1
x264.NAL_HRD_CBR             = 2


x264.preset_names = {
	"ultrafast",
	"superfast",
	"veryfast",
	"faster",
	"fast",
	"medium",
	"slow",
	"slower",
	"veryslow",
	"placebo"
	};

x264.tune_names = {
	"film",
	"animation",
	"grain",
	"stillimage",
	"psnr",
	"ssim",
	"fastdecode",
	"zerolatency"
	};

x264.profile_names = {
	"baseline",
	"main",
	"high",
	"high10",
	"high422",
	"high444"
	};


return x264
