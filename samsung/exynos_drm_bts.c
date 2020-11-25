// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * BTS file for Samsung EXYNOS DPU driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <soc/google/bts.h>
#if defined(CONFIG_SOC_GS101)
#include <soc/google/exynos-devfreq.h>
#include <dt-bindings/soc/google/gs101-devfreq.h>
#include <soc/google/cal-if.h>
#include <dt-bindings/clock/gs101.h>
#endif
#if defined(CONFIG_SOC_EXYNOS9610)
#include <dt-bindings/clock/exynos9610.h>
#elif defined(CONFIG_SOC_EXYNOS9820)
#include <dt-bindings/clock/exynos9820.h>
#endif

#include <linux/kernel.h>
#include "exynos_drm_decon.h"
#include "exynos_drm_format.h"
#include "exynos_drm_writeback.h"

#define DISP_FACTOR_PCT		100UL
#define MULTI_FACTOR		(1UL << 10)
#define ROT_READ_BYTE		(32) /* unit : BYTE(= pixel, based on NV12) */

#define ACLK_100MHZ_PERIOD	10000UL
#define FRAME_TIME_NSEC		1000000000UL	/* 1sec */

#define DPU_DEBUG_BTS(fmt, args...)	pr_debug("[BTS] "fmt,  ##args)
#define DPU_INFO_BTS(fmt, args...)	pr_info("[BTS] "fmt,  ##args)
#define DPU_ERR_BTS(fmt, args...)	pr_err("[BTS] "fmt, ##args)

/*
 * 1. function clock
 *    clk[i] = dst_w * dst_h * scale_ratio_h * scale_ratio_h * fps * 1.1 / ppc
 *    aclk1 = max(clk[i])
 * 2. AXI throughput clock
 *    1) fps based
 *       clk_bw[i] = src_w * src_h * fps * (bpp/8) / (bus_width * bus_util_pct)
 *    2) rotation throughput for initial latency
 *       clk_r[i] = src_h * 32 * (bpp/8) / (bus_width * rot_util_pct) / (v_blank)
 *       # v_blank : command - TE_hi_pulse
 *                   video - (vbp) @initial-frame, (vbp+vfp) @inter-frame
 *    if (clk_bw[i] < clk_r[i])
 *       clk_bw[i] = clk_r[i]
 *    aclk2 = max(clk for sum(same axi bw[i])) --> clk of peak_bw
 *
 * => aclk_dpu = max(aclk1, aclk2)
 */

/* unit : usec x 1000 -> 5592 (5.592us) for WQHD+ case */
static inline u32 dpu_bts_get_one_line_time(u32 lcd_height, u32 vbp, u32 vfp,
		u32 vsa, u32 fps)
{
	u32 tot_v;
	int tmp;

	tot_v = lcd_height + vfp + vsa + vbp;
	tmp = DIV_ROUND_UP(FRAME_TIME_NSEC, fps);

	return (tmp / tot_v);
}

/* framebuffer compressor(AFBC, SBWC) line delay is usually 4 */
static inline u32 dpu_bts_comp_latency(u32 src_w, u32 ppc, u32 line_delay)
{
	return mult_frac(src_w, line_delay, ppc);
}

/* scaler line delay is usually 3
 * scaling order : horizontal -> vertical scale
 * -> need to reflect scale-ratio
 */
static inline u32 dpu_bts_scale_latency(u32 src_w, u32 dst_w, u32 ppc,
				u32 line_delay)
{
	if (src_w > dst_w)
		return mult_frac(src_w * line_delay, src_w, dst_w * ppc);
	else
		return DIV_ROUND_CLOSEST(src_w * line_delay, ppc);
}

/* rotator ppc is usually 4 or 8
 * 1-read : 32BYTE (pixel)
 */
static inline u32 dpu_bts_rotate_latency(u32 src_w, u32 r_ppc)
{
	return (src_w * (ROT_READ_BYTE / r_ppc));
}

/*
 * [DSC]
 * Line memory is necessary like following.
 *  1EA(1ppc) : 2-line for 2-slice, 1-line for 1-slice
 *  2EA(2ppc) : 3.5-line for 4-slice (DSCC 0.5-line + DSC 3-line)
 *        2.5-line for 2-slice (DSCC 0.5-line + DSC 2-line)
 *
 * [DECON] none
 * When 1H is filled at OUT_FIFO, it immediately transfers to DSIM.
 */
static inline u32 dpu_bts_dsc_latency(u32 slice_num, u32 dsc_cnt,
		u32 dst_w, u32 ppc)
{
	u32 lat_dsc = dst_w;

	switch (slice_num) {
	case 1:
		/* DSC: 1EA */
		lat_dsc = dst_w * 1;
		break;
	case 2:
		if (dsc_cnt == 1)
			lat_dsc = dst_w * 2;
		else
			lat_dsc = (dst_w * 25) / (10 * ppc);
		break;
	case 4:
		/* DSC: 2EA */
		lat_dsc = (dst_w * 35) / (10 * ppc);
		break;
	default:
		break;
	}

	return lat_dsc;
}

/*
 * unit : nsec x 1000
 * reference aclk : 100MHz (-> 10ns x 1000)
 * # cycles = usec * aclk_mhz
 */
static inline u32 dpu_bts_convert_aclk_to_ns(u32 aclk_mhz)
{
	return ((ACLK_100MHZ_PERIOD * 100) / aclk_mhz);
}

/*
 * return : kHz value based on 1-pixel processing pipe-line
 */
static u64 dpu_bts_get_resol_clock(u32 xres, u32 yres, u32 fps)
{
	u64 margin;
	u64 resol_khz;

	/*
	 * aclk_khz = vclk_1pix * ( 1.1 + (48+20)/WIDTH ) : x1000
	 * @ (1.1)   : BUS Latency Considerable Margin (10%)
	 * @ (48+20) : HW bubble cycles
	 *      - 48 : 12 cycles per slice, total 4 slice
	 *      - 20 : hblank cycles for other HW module
	 */
	margin = 1100 + ((48000 + 20000) / xres);
	/* convert to kHz unit */
	resol_khz = (xres * yres * fps * margin / 1000) / 1000;

	return resol_khz;
}

static u32 dpu_bts_get_vblank_time_ns(struct decon_device *decon)
{
	u32 line_t_ns, v_blank_t_ns;

	line_t_ns = dpu_bts_get_one_line_time(decon->config.image_height,
		decon->bts.vbp, decon->bts.vfp, decon->bts.vsa, decon->bts.fps);
	if (decon->config.mode.op_mode == DECON_VIDEO_MODE)
		v_blank_t_ns = (decon->bts.vbp + decon->bts.vfp) * line_t_ns;
	else
		v_blank_t_ns = decon->config.vblank_usec * 1000U;

	DPU_DEBUG_BTS("\t-line_t_ns(%d) v_blank_t_ns(%d)\n",
			line_t_ns, v_blank_t_ns);

	return v_blank_t_ns;
}

static u32 dpu_bts_find_nearest_high_freq(struct decon_device *decon, u32 aclk_base)
{
	int i;

	if (aclk_base > decon->bts.dfs_lv_khz[0]) {
		DPU_DEBUG_BTS("\taclk_base is greater than L0 frequency!");
		i = 0;
	} else {
		/* search from low frequency level */
		for (i = (decon->bts.dfs_lv_cnt - 1); i >= 0; i--) {
			if (aclk_base <= decon->bts.dfs_lv_khz[i])
				break;
		}
	}
	DPU_DEBUG_BTS("\tNearest DFS: %d KHz @L%d\n", decon->bts.dfs_lv_khz[i], i);

	return i;
}

/*
 * [caution] src_w/h is rotated size info
 * - src_w : src_h @original input image
 * - src_h : src_w @original input image
 */
static u64 dpu_bts_calc_rotate_aclk(struct decon_device *decon, u32 aclk_base,
		u32 ppc, u32 src_w, u32 dst_w,
		bool is_comp, bool is_scale, bool is_dsc)
{
	u32 dfs_idx = 0;
	u32 dpu_cycle, basic_cycle, dsi_cycle, module_cycle = 0;
	u32 comp_cycle = 0, rot_cycle = 0, scale_cycle = 0, dsc_cycle = 0;
	u32 rot_init_bw = 0; /* KB/s */
	u64 rot_clk, rot_need_clk;
	u32 aclk_x_1k_ns, dpu_lat_t_ns, max_lat_t_ns, tx_allow_t_ns;
	u32 bus_perf;
	u32 temp_clk;
	bool retry_flag = false;

	DPU_DEBUG_BTS("[ROT+] BEFORE latency check: %d KHz\n", aclk_base);

	dfs_idx = dpu_bts_find_nearest_high_freq(decon, aclk_base);
	rot_clk = decon->bts.dfs_lv_khz[dfs_idx];

	/* post DECON OUTFIFO based on 1H transfer */
	dsi_cycle = decon->config.image_width;

	/* get additional pipeline latency */
	if (is_comp) {
		comp_cycle = dpu_bts_comp_latency(src_w, ppc,
			decon->bts.delay_comp);
		DPU_DEBUG_BTS("\tCOMP: lat_cycle(%d)\n", comp_cycle);
		module_cycle += comp_cycle;
	} else {
		rot_cycle = dpu_bts_rotate_latency(src_w,
			decon->bts.ppc_rotator);
		DPU_DEBUG_BTS("\tROT: lat_cycle(%d)\n", rot_cycle);
		module_cycle += rot_cycle;
	}
	if (is_scale) {
		scale_cycle = dpu_bts_scale_latency(src_w, dst_w,
			decon->bts.ppc_scaler, decon->bts.delay_scaler);
		DPU_DEBUG_BTS("\tSCALE: lat_cycle(%d)\n", scale_cycle);
		module_cycle += scale_cycle;
	}
	if (is_dsc) {
		dsc_cycle = dpu_bts_dsc_latency(decon->config.dsc.slice_count,
			decon->config.dsc.dsc_count, dst_w, ppc);
		DPU_DEBUG_BTS("\tDSC: lat_cycle(%d)\n", dsc_cycle);
		module_cycle += dsc_cycle;
		dsi_cycle = (dsi_cycle + 2) / 3;
	}

	/*
	 * basic cycle(+ bubble: 10%) + additional cycle based on function
	 * cycle count increases when ACLK goes up due to other conditions
	 * At latency monitor experiment using unit test,
	 *  cycles at 400Mhz were increased by about 800 compared to 200Mhz.
	 * Here, (aclk_mhz * 2) cycles are reflected referring to the result
	 *  because the exact value is unknown.
	 */
	basic_cycle = (decon->config.image_width * 11 / 10 + dsi_cycle) / ppc;

retry_hi_freq:
	dpu_cycle = (basic_cycle + module_cycle) + rot_clk * 2 / 1000U;
	aclk_x_1k_ns = dpu_bts_convert_aclk_to_ns(rot_clk / 1000U);
	dpu_lat_t_ns = mult_frac(aclk_x_1k_ns, dpu_cycle, 1000);
	max_lat_t_ns = dpu_bts_get_vblank_time_ns(decon);
	if (max_lat_t_ns > dpu_lat_t_ns) {
		tx_allow_t_ns = max_lat_t_ns - dpu_lat_t_ns;
	} else {
		/* abnormal case : apply bus_util_pct of v_blank */
		tx_allow_t_ns = (max_lat_t_ns * decon->bts.bus_util_pct) / 100;
		DPU_DEBUG_BTS("\tWARN: latency calc is abnormal!(-> %d%)\n",
				decon->bts.bus_util_pct);
	}

	bus_perf = decon->bts.bus_width * decon->bts.rot_util_pct;
	/* apply as worst(P010: 3) case to simplify */
	rot_init_bw = mult_frac(NSEC_PER_SEC, src_w * ROT_READ_BYTE * 3, tx_allow_t_ns) / 1000;
	rot_need_clk = rot_init_bw * 100 / bus_perf;

	if (rot_need_clk > rot_clk) {
		/* not max level */
		if (dfs_idx) {
			/* check if calc_clk is greater than 1-step */
			dfs_idx--;
			temp_clk = decon->bts.dfs_lv_khz[dfs_idx];
			if ((rot_need_clk > temp_clk) && (!retry_flag)) {
				DPU_DEBUG_BTS("\t-allow_ns(%d) dpu_ns(%d)\n",
					tx_allow_t_ns, dpu_lat_t_ns);
				rot_clk = temp_clk;
				retry_flag = true;
				goto retry_hi_freq;
			}
		}
		rot_clk = rot_need_clk;
	}

	DPU_DEBUG_BTS("\t-dpu_cycle(%d) aclk_x_1k_ns(%d) dpu_lat_t_ns(%d)\n",
			dpu_cycle, aclk_x_1k_ns, dpu_lat_t_ns);
	DPU_DEBUG_BTS("\t-tx_allow_t_ns(%d) rot_init_bw(%d) rot_need_clk(%lld)\n",
			tx_allow_t_ns, rot_init_bw, rot_need_clk);
	DPU_DEBUG_BTS("[ROT-] AFTER latency check: %lld KHz\n", rot_clk);

	return rot_clk;
}

static u64 dpu_bts_calc_aclk_disp(struct decon_device *decon,
		struct dpu_bts_win_config *config, u64 resol_clk, u32 max_clk)
{
	u64 s_ratio_h, s_ratio_v;
	u64 aclk_disp, aclk_base;
	u32 ppc;
	u32 src_w, src_h;
	u32 is_scale = false;
	u32 is_dsc = false;

	if (config->is_rot) {
		src_w = config->src_h;
		src_h = config->src_w;
	} else {
		src_w = config->src_w;
		src_h = config->src_h;
	}

	s_ratio_h = (src_w <= config->dst_w) ? MULTI_FACTOR :
				mult_frac(MULTI_FACTOR, src_w, config->dst_w);
	s_ratio_v = (src_h <= config->dst_h) ? MULTI_FACTOR :
				mult_frac(MULTI_FACTOR, src_h, config->dst_h);
	if ((s_ratio_h != MULTI_FACTOR) || (s_ratio_v != MULTI_FACTOR))
		is_scale = true;

	/* case for using dsc encoder 1ea at decon0 or decon1 */
	if ((decon->id != 2) && (decon->config.dsc.dsc_count == 1))
		ppc = ((decon->bts.ppc / 2UL) >= 1UL) ?
				(decon->bts.ppc / 2UL) : 1UL;
	else
		ppc = decon->bts.ppc;

	if (is_scale) {
		aclk_disp = mult_frac(resol_clk * s_ratio_h * s_ratio_v,
				DISP_FACTOR_PCT, 100UL * MULTI_FACTOR * MULTI_FACTOR);
		if (aclk_disp < resol_clk)
			aclk_disp = resol_clk;
		aclk_disp /= ppc;
	} else {
		aclk_disp = mult_frac(resol_clk, DISP_FACTOR_PCT, 100UL * ppc);
	}

	if (!config->is_rot)
		return aclk_disp;

	/* rotation case: check if latency conditions are met */
	if (aclk_disp > max_clk)
		aclk_base = aclk_disp;
	else
		aclk_base = max_clk;

	if (decon->config.dsc.enabled)
		is_dsc = true;

	aclk_disp = dpu_bts_calc_rotate_aclk(decon, (u32)aclk_base, ppc,
			src_w, config->dst_w, config->is_comp, is_scale, is_dsc);

	return aclk_disp;
}

static void dpu_bts_sum_all_decon_bw(struct decon_device *decon, u32 ch_bw[])
{
	int i, j;

	if (decon->id < 0 || decon->id >= MAX_DECON_CNT) {
		DPU_INFO_BTS("[%s] undefined decon id(%d)!\n", __func__,
				decon->id);
		return;
	}

	for (i = 0; i < MAX_DECON_CNT; ++i)
		decon->bts.ch_bw[decon->id][i] = ch_bw[i];

	for (i = 0; i < MAX_DECON_CNT; ++i) {
		if (decon->id == i)
			continue;

		for (j = 0; j < MAX_DECON_CNT; ++j)
			ch_bw[j] += decon->bts.ch_bw[i][j];
	}
}

static u32 dpu_bts_calc_disp_with_full_size(struct decon_device *decon)
{
	struct dpu_bts_win_config config;

	memset(&config, 0, sizeof(struct dpu_bts_win_config));
	config.src_w = config.dst_w = decon->config.image_width;
	config.src_h = config.dst_h = decon->config.image_height;
	config.format = DRM_FORMAT_ARGB8888;

	decon->bts.resol_clk = dpu_bts_get_resol_clock(
			decon->config.image_width,
			decon->config.image_height, decon->bts.fps);

	return dpu_bts_calc_aclk_disp(decon, &config, decon->bts.resol_clk,
		decon->bts.resol_clk);
}

static void dpu_bts_find_max_disp_freq(struct decon_device *decon)
{
	int i, j;
	u32 disp_ch_bw[MAX_DECON_CNT];
	u32 max_disp_ch_bw;
	u32 disp_op_freq = 0, freq = 0;
	struct dpu_bts_win_config *config = decon->bts.win_config;

	memset(disp_ch_bw, 0, sizeof(disp_ch_bw));

	for (i = 0; i < MAX_DPP_CNT; ++i)
		for (j = 0; j < MAX_DECON_CNT; ++j)
			if (decon->bts.bw[i].ch_num == j)
				disp_ch_bw[j] += decon->bts.bw[i].val;

	/* must be considered other decon's bw */
	dpu_bts_sum_all_decon_bw(decon, disp_ch_bw);

	for (i = 0; i < MAX_DECON_CNT; ++i)
		if (disp_ch_bw[i])
			DPU_DEBUG_BTS("\tAXI_DPU%d = %d\n", i, disp_ch_bw[i]);

	max_disp_ch_bw = disp_ch_bw[0];
	for (i = 1; i < MAX_DECON_CNT; ++i)
		if (max_disp_ch_bw < disp_ch_bw[i])
			max_disp_ch_bw = disp_ch_bw[i];

	decon->bts.peak = max_disp_ch_bw;
	if (max_disp_ch_bw < decon->bts.write_bw)
		decon->bts.peak = decon->bts.write_bw;
	decon->bts.max_disp_freq = decon->bts.peak * 100 /
			(decon->bts.bus_width * decon->bts.bus_util_pct);
	disp_op_freq = decon->bts.max_disp_freq;

	DPU_DEBUG_BTS("\tDECON%d : resol clock = %d Khz\n", decon->id,
			decon->bts.resol_clk);

	for (i = 0; i < decon->win_cnt; ++i) {
		if ((config[i].state != DPU_WIN_STATE_BUFFER) &&
				(config[i].state != DPU_WIN_STATE_COLOR))
			continue;

		freq = dpu_bts_calc_aclk_disp(decon, &config[i],
				(u64)decon->bts.resol_clk, disp_op_freq);
		if (disp_op_freq < freq)
			disp_op_freq = freq;
	}

	/*
	 * At least one window is used for colormap if there is a request of
	 * disabling all windows. So, disp frequency for a window of LCD full
	 * size is necessary.
	 */
	if (disp_op_freq == 0)
		disp_op_freq = dpu_bts_calc_disp_with_full_size(decon);

	DPU_DEBUG_BTS("\tDISP bus freq(%d), operating freq(%d)\n",
			decon->bts.max_disp_freq, disp_op_freq);

	if (decon->bts.max_disp_freq < disp_op_freq)
		decon->bts.max_disp_freq = disp_op_freq;

	DPU_DEBUG_BTS("\tMAX DISP CH FREQ = %d\n", decon->bts.max_disp_freq);
}

static void dpu_bts_share_bw_info(int id)
{
	int i, j;
	struct decon_device *decon[3];

	for (i = 0; i < MAX_DECON_CNT; i++)
		decon[i] = NULL;

	for (i = 0; i < MAX_DECON_CNT; i++)
		decon[i] = get_decon_drvdata(i);

	for (i = 0; i < MAX_DECON_CNT; ++i) {
		if (id == i || decon[i] == NULL)
			continue;

		for (j = 0; j < MAX_DECON_CNT; ++j)
			decon[i]->bts.ch_bw[id][j] =
				decon[id]->bts.ch_bw[id][j];
	}
}

static void
dpu_bts_calc_dpp_bw(struct bts_dpp_info *dpp, u32 fps, u32 vblank_us, int idx)
{
	u32 ch_bw = 0, rot_bw;
	u32 src_w = dpp->src_w;
	u32 src_h = dpp->src_h;
	u32 bpp = dpp->bpp;

	/* BW(KB) : sw * sh * fps * (bpp/8) * 1.1 */
	ch_bw = mult_frac(src_w * src_h * bpp / 8, fps  * 11, 10 * 1000);

	if (dpp->rotation) {
		/* BW(KB) : sh * 32B * (bpp/8) / v_blank */
		rot_bw = mult_frac(src_h * ROT_READ_BYTE * bpp / 8,
				USEC_PER_SEC, vblank_us) / 1000;

		if (rot_bw > ch_bw)
			ch_bw = rot_bw;
	}

	dpp->bw = ch_bw;

	DPU_DEBUG_BTS("\tDPP%d bandwidth = %d\n", idx, dpp->bw);
}

static void dpu_bts_convert_config_to_info(struct bts_dpp_info *dpp,
				const struct dpu_bts_win_config *config)
{
	const struct dpu_fmt *fmt_info;

	fmt_info = dpu_find_fmt_info(config->format);
	dpp->bpp = fmt_info->bpp + fmt_info->padding;
	dpp->src_w = config->src_w;
	dpp->src_h = config->src_h;
	dpp->dst.x1 = config->dst_x;
	dpp->dst.x2 = config->dst_x + config->dst_w;
	dpp->dst.y1 = config->dst_y;
	dpp->dst.y2 = config->dst_y + config->dst_h;
	dpp->rotation = config->is_rot;

	DPU_DEBUG_BTS("\tDPP%d : bpp(%d) src w(%d) h(%d) rot(%d)\n",
			DPU_DMA2CH(config->dpp_ch), dpp->bpp, dpp->src_w,
			dpp->src_h, dpp->rotation);
	DPU_DEBUG_BTS("\t\t\t\tdst x(%d) right(%d) y(%d) bottom(%d)\n",
			dpp->dst.x1, dpp->dst.x2, dpp->dst.y1, dpp->dst.y2);
}

static void dpu_bts_calc_bw(struct decon_device *decon)
{
	struct dpu_bts_win_config *config;
	struct bts_decon_info bts_info;
	int idx, i, wb_idx;
	u32 read_bw = 0, write_bw;
	u64 resol_clock;
	u32 vblank_us;

	if (!decon->bts.enabled)
		return;

	DPU_DEBUG_BTS("\n%s + : DECON%d\n", __func__, decon->id);

	memset(&bts_info, 0, sizeof(struct bts_decon_info));

	resol_clock = dpu_bts_get_resol_clock(decon->config.image_width,
				decon->config.image_height, decon->bts.fps);
	decon->bts.resol_clk = (u32)resol_clock;
	DPU_DEBUG_BTS("[Run: D%d] resol clock = %d Khz @%d fps\n",
		decon->id, decon->bts.resol_clk, decon->bts.fps);

	bts_info.vclk = decon->bts.resol_clk;
	bts_info.lcd_w = decon->config.image_width;
	bts_info.lcd_h = decon->config.image_height;
	vblank_us = dpu_bts_get_vblank_time_ns(decon) / 1000U;
	/* reflect bus_util_pct for dpu processing latency when rotation */
	vblank_us = (vblank_us * decon->bts.bus_util_pct) / 100;

	/* read bw calculation */
	config = decon->bts.win_config;
	for (i = 0; i < decon->win_cnt; ++i) {
		if (config[i].state != DPU_WIN_STATE_BUFFER)
			continue;

		idx = config[i].dpp_ch;
		dpu_bts_convert_config_to_info(&bts_info.rdma[idx], &config[i]);
		dpu_bts_calc_dpp_bw(&bts_info.rdma[idx], decon->bts.fps, vblank_us, idx);
		read_bw += bts_info.rdma[idx].bw;
	}

	/* write bw calculation */
	config = &decon->bts.wb_config;
	if (config->state == DPU_WIN_STATE_BUFFER) {
		wb_idx = config->dpp_ch;
		dpu_bts_convert_config_to_info(&bts_info.odma, config);
		dpu_bts_calc_dpp_bw(&bts_info.odma, decon->bts.fps, vblank_us, wb_idx);
		write_bw = bts_info.odma.bw;
	} else {
		wb_idx = -1;
		write_bw = 0;
	}

	for (i = 0; i < MAX_DPP_CNT; i++) {
		if (i < MAX_WIN_PER_DECON)
			decon->bts.bw[i].val = bts_info.rdma[i].bw;
		else if (i == wb_idx)
			decon->bts.bw[i].val = bts_info.odma.bw;
		else
			decon->bts.bw[i].val = 0;
	}

	decon->bts.read_bw = read_bw;
	decon->bts.write_bw = write_bw;
	decon->bts.total_bw = read_bw + write_bw;

	DPU_DEBUG_BTS("\tDECON%d total bw = %d, read bw = %d, write bw = %d\n",
			decon->id, decon->bts.total_bw, decon->bts.read_bw,
			decon->bts.write_bw);

	dpu_bts_find_max_disp_freq(decon);

	/* update bw for other decons */
	dpu_bts_share_bw_info(decon->id);

	DPU_EVENT_LOG(DPU_EVT_BTS_CALC_BW, decon->id, NULL);
	DPU_DEBUG_BTS("%s -\n", __func__);
}

static void dpu_bts_update_bw(struct decon_device *decon, bool shadow_updated)
{
	struct bts_bw bw = { 0, };

	DPU_DEBUG_BTS("%s +\n", __func__);

	if (!decon->bts.enabled)
		return;

	/* update peak & R/W bandwidth per DPU port */
	bw.peak = decon->bts.peak;
	bw.read = decon->bts.read_bw;
	bw.write = decon->bts.write_bw;
	DPU_DEBUG_BTS("\tpeak = %d, read = %d, write = %d\n",
			bw.peak, bw.read, bw.write);

	if (shadow_updated) {
		/* after DECON h/w configs are updated to shadow SFR */
		if (decon->bts.total_bw < decon->bts.prev_total_bw)
			bts_update_bw(decon->bts.bw_idx, bw);

		if (decon->bts.max_disp_freq < decon->bts.prev_max_disp_freq)
			exynos_pm_qos_update_request(&decon->bts.disp_qos,
					decon->bts.max_disp_freq);

		decon->bts.prev_total_bw = decon->bts.total_bw;
		decon->bts.prev_max_disp_freq = decon->bts.max_disp_freq;
	} else {
		if (decon->bts.total_bw > decon->bts.prev_total_bw)
			bts_update_bw(decon->bts.bw_idx, bw);

		if (decon->bts.max_disp_freq > decon->bts.prev_max_disp_freq)
			exynos_pm_qos_update_request(&decon->bts.disp_qos,
					decon->bts.max_disp_freq);
	}

	DPU_EVENT_LOG(DPU_EVT_BTS_UPDATE_BW, decon->id, NULL);
	DPU_DEBUG_BTS("%s -\n", __func__);
}

static void dpu_bts_release_bw(struct decon_device *decon)
{
	struct bts_bw bw = { 0, };

	DPU_DEBUG_BTS("%s +\n", __func__);

	if (!decon->bts.enabled)
		return;

	if (decon->config.out_type & DECON_OUT_DSI) {
		bts_update_bw(decon->bts.bw_idx, bw);
		decon->bts.prev_total_bw = 0;
		exynos_pm_qos_update_request(&decon->bts.disp_qos, 0);
		decon->bts.prev_max_disp_freq = 0;
	}

	DPU_EVENT_LOG(DPU_EVT_BTS_RELEASE_BW, decon->id, NULL);
	DPU_DEBUG_BTS("%s -\n", __func__);
}

#define MAX_IDX_NAME_SIZE	16
static void dpu_bts_init(struct decon_device *decon)
{
	int i;
	char bts_idx_name[MAX_IDX_NAME_SIZE];
	const struct drm_encoder *encoder;

	DPU_DEBUG_BTS("%s +\n", __func__);

	decon->bts.enabled = false;

	if (!IS_ENABLED(CONFIG_EXYNOS_BTS) ||
			(!IS_ENABLED(CONFIG_EXYNOS_PM_QOS) &&
			 !IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE))) {
		DPU_ERR_BTS("decon%d bts feature is disabled\n", decon->id);
		pr_info("%s:%d\n", __func__, __LINE__);
		return;
	}

	memset(bts_idx_name, 0, MAX_IDX_NAME_SIZE);
	snprintf(bts_idx_name, MAX_IDX_NAME_SIZE, "DECON%d", decon->id);
	decon->bts.bw_idx = bts_get_bwindex(bts_idx_name);

	decon->bts.dvfs_max_disp_freq =
			(u32)cal_dfs_get_max_freq(ACPM_DVFS_DISP);

	for (i = 0; i < MAX_DECON_CNT; i++)
		decon->bts.ch_bw[decon->id][i] = 0;

	DPU_DEBUG_BTS("BTS_BW_TYPE(%d)\n", decon->bts.bw_idx);
	exynos_pm_qos_add_request(&decon->bts.mif_qos,
					PM_QOS_BUS_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&decon->bts.int_qos,
					PM_QOS_DEVICE_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&decon->bts.disp_qos,
					PM_QOS_DISPLAY_THROUGHPUT, 0);

	for (i = 0; i < decon->dpp_cnt; ++i) { /* dma type order */
		decon->bts.bw[i].ch_num = decon->dpp[DPU_DMA2CH(i)]->port;
		DPU_INFO_BTS("IDMA_TYPE(%d) CH(%d) Port(%d)\n", i,
				DPU_DMA2CH(i), decon->bts.bw[i].ch_num);
	}

	drm_for_each_encoder(encoder, decon->drm_dev) {
		const struct writeback_device *wb;

		if (encoder->encoder_type == DRM_MODE_ENCODER_VIRTUAL) {
			wb = enc_to_wb_dev(encoder);
			decon->bts.bw[wb->id].ch_num = wb->port;
			break;
		}
	}

	decon->bts.enabled = true;

	DPU_INFO_BTS("decon%d bts feature is enabled\n", decon->id);
}

static void dpu_bts_deinit(struct decon_device *decon)
{
	if (!decon->bts.enabled)
		return;

	DPU_DEBUG_BTS("%s +\n", __func__);
	exynos_pm_qos_remove_request(&decon->bts.disp_qos);
	exynos_pm_qos_remove_request(&decon->bts.int_qos);
	exynos_pm_qos_remove_request(&decon->bts.mif_qos);
	DPU_DEBUG_BTS("%s -\n", __func__);
}

struct dpu_bts_ops dpu_bts_control = {
	.init		= dpu_bts_init,
	.calc_bw	= dpu_bts_calc_bw,
	.update_bw	= dpu_bts_update_bw,
	.release_bw	= dpu_bts_release_bw,
	.deinit		= dpu_bts_deinit,
};
