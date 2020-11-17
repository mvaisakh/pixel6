// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based s6e3fc3 AMOLED LCD panel driver.
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <video/mipi_display.h>

#include "panel-samsung-drv.h"

static const unsigned char PPS_SETTING[] = {
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09, 0x60,
	0x04, 0x38, 0x00, 0x30, 0x02, 0x1C, 0x02, 0x1C,
	0x02, 0x00, 0x02, 0x0E, 0x00, 0x20, 0x04, 0xA6,
	0x00, 0x07, 0x00, 0x0C, 0x02, 0x0B, 0x02, 0x1F,
	0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,
	0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
	0x7D, 0x7E, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA, 0x19, 0xF8,
	0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,
	0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define S6E3FC3_WRCTRLD_DIMMING_BIT    0x08
#define S6E3FC3_WRCTRLD_BCTRL_BIT      0x20
#define S6E3FC3_WRCTRLD_HBM_BIT        0xC0
#define S6E3FC3_WRCTRLD_LOCAL_HBM_BIT  0x10

static void s6e3fc3_change_frequency(struct exynos_panel *ctx,
				     unsigned int vrefresh)
{
	if (!ctx || (vrefresh != 60 && vrefresh != 90))
		return;

	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF0, 0x5A, 0x5A);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0x60, (vrefresh == 90) ? 0x08 : 0x00);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF7, 0x0F);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF0, 0xA5, 0xA5);

	dev_dbg(ctx->dev, "%s: change to %uhz\n", __func__, vrefresh);
}

static void s6e3fc3_write_display_mode(struct exynos_panel *ctx,
				       const struct drm_display_mode *mode)
{
	u8 val = S6E3FC3_WRCTRLD_BCTRL_BIT;

	if (ctx->hbm_mode)
		val |= S6E3FC3_WRCTRLD_HBM_BIT;

	if (ctx->local_hbm.enabled)
		val |= S6E3FC3_WRCTRLD_LOCAL_HBM_BIT;

	dev_dbg(ctx->dev, "%s(wrctrld:0x%x, hbm: %s, local_hbm: %s)\n",
		__func__, val, ctx->hbm_mode ? "on" : "off",
		ctx->local_hbm.enabled ? "on" : "off");

	EXYNOS_DCS_WRITE_SEQ(ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, val);

	/* TODO: need to perform gamma updates */
}

static int s6e3fc3_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx;
	const struct drm_display_mode *mode;

	ctx = container_of(panel, struct exynos_panel, panel);

	mode = ctx->current_mode;
	if (!mode) {
		dev_err(ctx->dev, "no current mode set\n");
		return -EINVAL;
	}

	dev_dbg(ctx->dev, "%s\n", __func__);

	exynos_panel_reset(ctx);

	EXYNOS_DCS_WRITE_SEQ_DELAY(ctx, 120, 0x11); /* sleep out: 120ms delay */

	EXYNOS_DCS_WRITE_SEQ(ctx, 0x35); /* TE on */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0x2A, 0x00, 0x00, 0x04, 0x37); /* CASET */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0x2B, 0x00, 0x00, 0x09, 0x5F); /* PASET */

	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF0, 0x5A, 0x5A); /* TEST_KEY_ON_F0 */

	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x27, 0xF2); /* FQ_CON_GLOBAL */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF2, 0x00); /* FQ_CON_0 */

	/* Local HBM circle location setting */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF1, 0x5A, 0x5A); /* TEST_KEY_ON_F1 */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x28, 0xF2); /* global para */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF2, 0xCC); /* global para 10bit */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x01, 0x34, 0x68); /* global para */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0x68, 0x21, 0xC6, 0xE9); /* circle location */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x00, 0x28, 0xF2); /* global para */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF2, 0xC4); /* global para 8bit */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF1, 0xA5, 0xA5); /* TEST_KEY_OFF_F1 */

	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF0, 0xA5, 0xA5); /* TEST_KEY_OFF_F0 */

	s6e3fc3_change_frequency(ctx, drm_mode_vrefresh(mode));

	/* DSC related configuration */
	exynos_dcs_compression_mode(ctx, 0x1); /* DSC_DEC_ON */
	EXYNOS_PPS_LONG_WRITE(ctx); /* PPS_SETTING */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xC2, 0x14); /* PPS_MIC_OFF */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0x9D, 0x01); /* PPS_DSC_EN */

	s6e3fc3_write_display_mode(ctx, mode); /* dimming and HBM */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0x51, 0x02, 0x58); /* brightness level */

	EXYNOS_DCS_WRITE_SEQ(ctx, 0x29); /* display on */

	ctx->enabled = true;

	return 0;
}

static void s6e3fc3_set_hbm_mode(struct exynos_panel *exynos_panel,
				 bool hbm_mode)
{
	exynos_panel->hbm_mode = hbm_mode;

	s6e3fc3_write_display_mode(exynos_panel, exynos_panel->current_mode);
}

static void s6e3fc3_set_local_hbm_mode(struct exynos_panel *exynos_panel,
				 bool local_hbm_en)
{
	if (exynos_panel->local_hbm.enabled == local_hbm_en)
		return;

	mutex_lock(&exynos_panel->local_hbm.lock);
	exynos_panel->local_hbm.enabled = local_hbm_en;
	s6e3fc3_write_display_mode(exynos_panel, exynos_panel->current_mode);
	mutex_unlock(&exynos_panel->local_hbm.lock);
	/* TODO: need to wait two drm_crtc_wait_one_vblank() */

}

static void s6e3fc3_mode_set(struct exynos_panel *ctx,
			     const struct drm_display_mode *mode)
{
	if (!ctx->enabled)
		return;

	s6e3fc3_write_display_mode(ctx, mode);
	s6e3fc3_change_frequency(ctx, drm_mode_vrefresh(mode));
}

static bool s6e3fc3_is_mode_seamless(const struct exynos_panel *ctx,
				     const struct drm_display_mode *mode)
{
	/* seamless mode switch is possible if only changing refresh rate */
	return drm_mode_equal_no_clocks(ctx->current_mode, mode);
}

static const struct exynos_display_mode s6e3fc3_mode_private = {
	.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.vblank_usec = 120,
	.bpc = 8,
	.dsc = {
		.enabled = true,
		.dsc_count = 2,
		.slice_count = 2,
		.slice_height = 48,
	},
};

static const struct drm_display_mode s6e3fc3_modes[] = {
	{
		/* 1080x2400 @ 60Hz */
		.clock = 168498,
		.hdisplay = 1080,
		.hsync_start = 1080 + 32, // add hfp
		.hsync_end = 1080 + 32 + 12, // add hsa
		.htotal = 1080 + 32 + 12 + 26, // add hbp
		.vdisplay = 2400,
		.vsync_start = 2400 + 12, // add vfp
		.vsync_end = 2400 + 12 + 4, // add vsa
		.vtotal = 2400 + 12 + 4 + 26, // add vbp
		.flags = 0,
		.width_mm = 67,
		.height_mm = 148,
		.private = (int *)&s6e3fc3_mode_private,
		.private_flags = EXYNOS_DISPLAY_MODE_FLAG_EXYNOS_PANEL,
	},
	{
		/* 1080x2400 @ 90Hz */
		.clock = 252747,
		.hdisplay = 1080,
		.hsync_start = 1080 + 32, // add hfp
		.hsync_end = 1080 + 32 + 12, // add hsa
		.htotal = 1080 + 32 + 12 + 26, // add hbp
		.vdisplay = 2400,
		.vsync_start = 2400 + 12, // add vfp
		.vsync_end = 2400 + 12 + 4, // add vsa
		.vtotal = 2400 + 12 + 4 + 26, // add vbp
		.flags = 0,
		.width_mm = 67,
		.height_mm = 148,
		.private = (int *)&s6e3fc3_mode_private,
		.private_flags = EXYNOS_DISPLAY_MODE_FLAG_EXYNOS_PANEL,
	},
};

static const struct drm_panel_funcs s6e3fc3_drm_funcs = {
	.disable = exynos_panel_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = exynos_panel_prepare,
	.enable = s6e3fc3_enable,
	.get_modes = exynos_panel_get_modes,
};

static const struct exynos_panel_funcs s6e3fc3_exynos_funcs = {
	.set_brightness = exynos_panel_set_brightness,
	.set_hbm_mode = s6e3fc3_set_hbm_mode,
	.set_local_hbm_mode = s6e3fc3_set_local_hbm_mode,
	.is_mode_seamless = s6e3fc3_is_mode_seamless,
	.mode_set = s6e3fc3_mode_set,
};

const struct exynos_panel_desc samsung_s6e3fc3 = {
	.dsc_pps = PPS_SETTING,
	.dsc_pps_len = ARRAY_SIZE(PPS_SETTING),
	.data_lane_cnt = 4,
	.max_brightness = 2047,
	.dft_brightness = 1023,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2),
	.max_luminance = 5400000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.modes = s6e3fc3_modes,
	.num_modes = ARRAY_SIZE(s6e3fc3_modes),
	.panel_func = &s6e3fc3_drm_funcs,
	.exynos_panel_func = &s6e3fc3_exynos_funcs,
};

static const struct of_device_id exynos_panel_of_match[] = {
	{ .compatible = "samsung,s6e3fc3", .data = &samsung_s6e3fc3 },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_panel_of_match);

static struct mipi_dsi_driver exynos_panel_driver = {
	.probe = exynos_panel_probe,
	.remove = exynos_panel_remove,
	.driver = {
		.name = "panel-samsung-s6e3fc3",
		.of_match_table = exynos_panel_of_match,
	},
};
module_mipi_dsi_driver(exynos_panel_driver);

MODULE_AUTHOR("Jiun Yu <jiun.yu@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung s6e3fc3 panel driver");
MODULE_LICENSE("GPL");
