// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based s6e3hc2 AMOLED LCD panel driver.
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <video/mipi_display.h>

#include "panel-samsung-drv.h"

static const unsigned char SEQ_PPS_WQHD[] = {
	// WQHD+ :1440x3040
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x0B, 0xE0,
	0x05, 0xA0, 0x00, 0x28, 0x02, 0xD0, 0x02, 0xD0,
	0x02, 0x00, 0x02, 0x68, 0x00, 0x20, 0x04, 0x6C,
	0x00, 0x0A, 0x00, 0x0C, 0x02, 0x77, 0x01, 0xE9,
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
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const unsigned char SEQ_PPS_FHD[] = {
	//FHD+ :1080x2340
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09, 0x24,
	0x04, 0x38, 0x00, 0x41, 0x02, 0x1C, 0x02, 0x1C,
	0x02, 0x00, 0x02, 0x0E, 0x00, 0x20, 0x06, 0x50,
	0x00, 0x07, 0x00, 0x0C, 0x01, 0x80, 0x01, 0x91,
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
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#define S6E3HC2_WRCTRLD_DIMMING_BIT    0x08
#define S6E3HC2_WRCTRLD_FRAME_RATE_BIT 0x10
#define S6E3HC2_WRCTRLD_BCTRL_BIT      0x20
#define S6E3HC2_WRCTRLD_HBM_BIT        0xC0

static const u8 unlock_cmd_f0[] = { 0xF0, 0x5A, 0x5A };
static const u8 lock_cmd_f0[]   = { 0xF0, 0xA5, 0xA5 };
static const u8 swire_refresh_global[] = { 0xB0, 0x3F, 0xB5 };
static const u8 swire_refresh_off[] = { 0xB5, 0x05 };
static const u8 swire_refresh_on[] = { 0xB5, 0x85 };
static const u8 aid_trans_global[] = { 0xB0, 0xA8, 0xB1 };
static const u8 aid_trans_off[] = { 0xB1, 0x00 };
static const u8 aid_trans_on[] = { 0xB1, 0x01 };
static const u8 hlpm_ctrl_global[] = { 0xB0, 0x05, 0xBB };
static const u8 hlpm_ctrl_50nit[] = { 0xBB, 0x00 };
static const u8 hlpm_ctrl_10nit[] = { 0xBB, 0x80 };
static const u8 hlpm_on_50nit[] = { 0x53, 0x02 };
static const u8 hlpm_on_10nit[] = { 0x53, 0x03 };
static const u8 gamma_aid_update[] = { 0xF7, 0x02 };
static const u8 display_off[] = { 0x28 };
static const u8 display_on[] = { 0x29 };
static const u8 sleep_in[] = { 0x10 };

static const struct exynos_dsi_cmd s6e3hc2_off_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 10),
	EXYNOS_DSI_CMD(sleep_in, 120),
};

static const struct exynos_dsi_cmd_set s6e3hc2_off_cmd_set = {
	.num_cmd = ARRAY_SIZE(s6e3hc2_off_cmds),
	.cmds = s6e3hc2_off_cmds
};

static const struct exynos_dsi_cmd s6e3hc2_lp_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 17),
	EXYNOS_DSI_CMD(unlock_cmd_f0, 0),
	EXYNOS_DSI_CMD(swire_refresh_global, 0),
	EXYNOS_DSI_CMD(swire_refresh_off, 0),
	EXYNOS_DSI_CMD(lock_cmd_f0, 0),
};

static const struct exynos_dsi_cmd_set s6e3hc2_lp_cmd_set = {
	.num_cmd = ARRAY_SIZE(s6e3hc2_lp_cmds),
	.cmds = s6e3hc2_lp_cmds
};

static const struct exynos_dsi_cmd s6e3hc2_lp_off_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 0)
};

static const struct exynos_dsi_cmd s6e3hc2_lp_low_cmds[] = {
	EXYNOS_DSI_CMD(unlock_cmd_f0, 0),
	EXYNOS_DSI_CMD(hlpm_on_10nit, 17),
	EXYNOS_DSI_CMD(aid_trans_global, 0),
	EXYNOS_DSI_CMD(aid_trans_off, 0),
	EXYNOS_DSI_CMD(hlpm_ctrl_global, 0),
	EXYNOS_DSI_CMD(hlpm_ctrl_10nit, 0),
	EXYNOS_DSI_CMD(gamma_aid_update, 0),
	EXYNOS_DSI_CMD(lock_cmd_f0, 0),
	EXYNOS_DSI_CMD(display_on, 0)
};

static const struct exynos_dsi_cmd s6e3hc2_lp_high_cmds[] = {
	EXYNOS_DSI_CMD(unlock_cmd_f0, 0),
	EXYNOS_DSI_CMD(hlpm_on_50nit, 17),
	EXYNOS_DSI_CMD(aid_trans_global, 0),
	EXYNOS_DSI_CMD(aid_trans_off, 0),
	EXYNOS_DSI_CMD(hlpm_ctrl_global, 0),
	EXYNOS_DSI_CMD(hlpm_ctrl_50nit, 0),
	EXYNOS_DSI_CMD(gamma_aid_update, 0),
	EXYNOS_DSI_CMD(lock_cmd_f0, 0),
	EXYNOS_DSI_CMD(display_on, 0)
};

static const struct exynos_binned_lp s6e3hc2_binned_lp[] = {
	BINNED_LP_MODE("off",     0, s6e3hc2_lp_off_cmds),
	BINNED_LP_MODE("low",    40, s6e3hc2_lp_low_cmds),
	BINNED_LP_MODE("high", 1023, s6e3hc2_lp_high_cmds)
};

static void s6e3hc2_write_display_mode(struct exynos_panel *ctx,
				       const struct drm_display_mode *mode)
{
	u8 val = S6E3HC2_WRCTRLD_BCTRL_BIT;

	if (drm_mode_vrefresh(mode) == 90)
		val |= S6E3HC2_WRCTRLD_FRAME_RATE_BIT;

	if (ctx->hbm_mode)
		val |= S6E3HC2_WRCTRLD_HBM_BIT;

	dev_dbg(ctx->dev, "%s(wrctrld:0x%x, hbm: %s, refresh: %uhz)\n",
		__func__, val, ctx->hbm_mode ? "on" : "off", drm_mode_vrefresh(mode));

	EXYNOS_DCS_WRITE_SEQ(ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, val);

	/* TODO: need to perform gamma updates */
}

static void s6e3hc2_set_nolp_mode(struct exynos_panel *ctx,
				  const struct exynos_panel_mode *pmode)
{
	if (!ctx->enabled)
		return;

	EXYNOS_DCS_WRITE_TABLE(ctx, display_off);
	EXYNOS_DCS_WRITE_TABLE(ctx, unlock_cmd_f0);
	EXYNOS_DCS_WRITE_TABLE(ctx, aid_trans_global);
	EXYNOS_DCS_WRITE_TABLE(ctx, aid_trans_on);

	s6e3hc2_write_display_mode(ctx, &pmode->mode);

	EXYNOS_DCS_WRITE_TABLE(ctx, swire_refresh_global);
	EXYNOS_DCS_WRITE_TABLE_DELAY(ctx, 34, swire_refresh_on);
	EXYNOS_DCS_WRITE_TABLE_DELAY(ctx, 17, lock_cmd_f0);
	EXYNOS_DCS_WRITE_TABLE(ctx, display_on);

	dev_info(ctx->dev, "exit LP mode\n");
}

static int s6e3hc2_common_pre_enable(struct exynos_panel *ctx)
{
	const struct exynos_panel_mode *pmode = ctx->current_mode;

	if (!pmode) {
		dev_err(ctx->dev, "no current mode set\n");
		return -EINVAL;
	}

	dev_dbg(ctx->dev, "%s\n", __func__);

	exynos_panel_reset(ctx);

	EXYNOS_DCS_WRITE_SEQ(ctx, 0xf0, 0x5a, 0x5a);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xfc, 0x5a, 0x5a);

	/* DSC related configuration */
	exynos_dcs_compression_mode(ctx, 0x1);

	EXYNOS_PPS_LONG_WRITE(ctx);
	/* sleep out: 120ms delay */
	EXYNOS_DCS_WRITE_SEQ_DELAY(ctx, 120, 0x11);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB9, 0x00, 0xB0, 0x8F, 0x09, 0x00, 0x00,
			0x00, 0x11, 0x01);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0x1A, 0x1F, 0x00, 0x00, 0x00, 0x00);

	EXYNOS_DCS_WRITE_SEQ(ctx, 0x35); /* TE on */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xED, 0x44);

#if defined(CONFIG_EXYNOS_PLL_SLEEP)
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB9, 0x01, 0xB0, 0x91, 0x09);
#else
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB9, 0x00, 0xB0, 0x9C, 0x09);
#endif
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xCE, 0x0D, 0x58, 0x14, 0x64, 0x38, 0xB8,
			0xF2, 0x03, 0x00, 0xFF, 0x02, 0x0A, 0x0A, 0x0A, 0x0A,
			0x0F, 0x23);

	s6e3hc2_write_display_mode(ctx, &pmode->mode);

	return 0;
}

static void s6e3hc2_common_post_enable(struct exynos_panel *ctx)
{
	const struct exynos_panel_mode *pmode;

	dev_dbg(ctx->dev, "%s\n", __func__);

	ctx->enabled = true;

	pmode = ctx->current_mode;
	if (pmode->exynos_mode.is_lp_mode)
		exynos_panel_set_lp_mode(ctx, pmode);
	else
		EXYNOS_DCS_WRITE_SEQ(ctx, 0x29); /* display on */

	backlight_update_status(ctx->bl);
}

static int s6e3hc2_wqhd_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx;
	int ret;

	ctx = container_of(panel, struct exynos_panel, panel);

	ret = s6e3hc2_common_pre_enable(ctx);
	if (ret)
		return ret;

	s6e3hc2_common_post_enable(ctx);

	return 0;
}

static int s6e3hc2_fhd_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx;
	int ret;

	ctx = container_of(panel, struct exynos_panel, panel);

	ret = s6e3hc2_common_pre_enable(ctx);
	if (ret)
		return ret;

	/* TSP HSYNC Enable*/
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x07, 0xB9);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB9, 0x11, 0x03);

	/*TOUT Enable*/
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xFF, 0x00, 0x01);

	s6e3hc2_common_post_enable(ctx);

	return 0;
}

static void s6e3hc2_set_hbm_mode(struct exynos_panel *exynos_panel,
				bool hbm_mode)
{
	const struct exynos_panel_mode *pmode = exynos_panel->current_mode;

	exynos_panel->hbm_mode = hbm_mode;

	s6e3hc2_write_display_mode(exynos_panel, &pmode->mode);
}

static void s6e3hc2_mode_set(struct exynos_panel *ctx,
			     const struct exynos_panel_mode *pmode)
{
	if (!ctx->enabled)
		return;

	s6e3hc2_write_display_mode(ctx, &pmode->mode);
}

static bool s6e3hc2_is_mode_seamless(const struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	/* seamless mode switch is possible if only changing refresh rate */
	return drm_mode_equal_no_clocks(&ctx->current_mode->mode, &pmode->mode);
}

static const struct exynos_display_underrun_param wqhd_underrun_param = {
	.te_idle_us = 1000,
	.te_var = 1,
	.max_vrefresh = 90,
};

static const u32 s6e3hc2_bl_range[] = {
	129, 157, 187, 1023,
};

static const struct exynos_panel_mode s6e3hc2_wqhd_modes[] = {
	{
		/* 1440x3040 @ 60Hz */
		.mode = {
			.clock = 266568,
			.hdisplay = 1440,
			.hsync_start = 1440 + 2,
			.hsync_end = 1440 + 2 + 2,
			.htotal = 1440 + 6 + 2 + 2,
			.vdisplay = 3040,
			.vsync_start = 3040 + 8,
			.vsync_end = 3040 + 8 + 1,
			.vtotal = 3040 + 8 + 1 + 15,
			.flags = 0,
			.width_mm = 69,
			.height_mm = 142,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 40,
			},
			.underrun_param = &wqhd_underrun_param,
		},
	},
	{
		/* 1440x3040 @ 90Hz */
		.mode = {
			.clock = 399852,
			.hdisplay = 1440,
			.hsync_start = 1440 + 2,
			.hsync_end = 1440 + 2 + 2,
			.htotal = 1440 + 6 + 2 + 2,
			.vdisplay = 3040,
			.vsync_start = 3040 + 8,
			.vsync_end = 3040 + 8 + 1,
			.vtotal = 3040 + 8 + 1 + 15,
			.flags = 0,
			.width_mm = 69,
			.height_mm = 142,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 40,
			},
			.underrun_param = &wqhd_underrun_param,
		},
	}
};

static const struct exynos_display_underrun_param fhd_underrun_param = {
	.te_idle_us = 1000,
	.te_var = 1,
	.max_vrefresh = 90,
};

static const struct exynos_panel_mode s6e3hc2_fhd_modes[] = {
	{
		/* 1080x2340 @ 60Hz */
		.mode = {
			.clock = 164358,
			.hdisplay = 1080,
			.hsync_start = 1080 + 32, // add hfp
			.hsync_end = 1080 + 32 + 12, // add hsa
			.htotal = 1080 + 32 + 12 + 26, // add hbp
			.vdisplay = 2340,
			.vsync_start = 2340 + 12, // add vfp
			.vsync_end = 2340 + 12 + 4, // add vsa
			.vtotal = 2340 + 12 + 4 + 26, // add vbp
			.flags = 0,
			.width_mm = 63,
			.height_mm = 137,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 65,
			},
			.underrun_param = &fhd_underrun_param,
		},
	},
	{
		/* 1080x2340 @ 90Hz */
		.mode = {
			.clock = 246537,
			.hdisplay = 1080,
			.hsync_start = 1080 + 32, // add hfp
			.hsync_end = 1080 + 32 + 12, // add hsa
			.htotal = 1080 + 32 + 12 + 26, // add hbp
			.vdisplay = 2340,
			.vsync_start = 2340 + 12, // add vfp
			.vsync_end = 2340 + 12 + 4, // add vsa
			.vtotal = 2340 + 12 + 4 + 26, // add vbp
			.flags = 0,
			.width_mm = 63,
			.height_mm = 137,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 65,
			},
			.underrun_param = &fhd_underrun_param,
		},
	},
};

static const struct exynos_panel_mode s6e3hc2_wqhd_lp_mode = {
	.mode = {
		/* 1440x3040 @ 30Hz */
		.name = "1440x3040x30",
		.clock = 133284,
		.hdisplay = 1440,
		.hsync_start = 1440 + 2,
		.hsync_end = 1440 + 2 + 2,
		.htotal = 1440 + 6 + 2 + 2,
		.vdisplay = 3040,
		.vsync_start = 3040 + 8,
		.vsync_end = 3040 + 8 + 1,
		.vtotal = 3040 + 8 + 1 + 15,
		.flags = 0,
		.type = DRM_MODE_TYPE_DRIVER,
		.width_mm = 69,
		.height_mm = 142,
	},
	.exynos_mode = {
		.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
		.vblank_usec = 120,
		.bpc = 8,
		.dsc = {
			.enabled = true,
			.dsc_count = 2,
			.slice_count = 2,
			.slice_height = 40,
		},
		.is_lp_mode = true,
	}
};

static const struct exynos_panel_mode s6e3hc2_fhd_lp_mode = {
	.mode = {
		/* 1080x2340 @ 30Hz */
		.name = "1080x2340x30",
		.clock = 82179,
		.hdisplay = 1080,
		.hsync_start = 1080 + 32, // add hfp
		.hsync_end = 1080 + 32 + 12, // add hsa
		.htotal = 1080 + 32 + 12 + 26, // add hbp
		.vdisplay = 2340,
		.vsync_start = 2340 + 12, // add vfp
		.vsync_end = 2340 + 12 + 4, // add vsa
		.vtotal = 2340 + 12 + 4 + 26, // add vbp
		.flags = 0,
		.type = DRM_MODE_TYPE_DRIVER,
		.width_mm = 63,
		.height_mm = 137,
	},
	.exynos_mode = {
		.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
		.vblank_usec = 120,
		.bpc = 8,
		.dsc = {
			.enabled = true,
			.dsc_count = 2,
			.slice_count = 2,
			.slice_height = 65,
		},
		.is_lp_mode = true,
	},
};

static const struct drm_panel_funcs s6e3hc2_wqhd_drm_funcs = {
	.disable = exynos_panel_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = exynos_panel_prepare,
	.enable = s6e3hc2_wqhd_enable,
	.get_modes = exynos_panel_get_modes,
};

static const struct drm_panel_funcs s6e3hc2_fhd_drm_funcs = {
	.disable = exynos_panel_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = exynos_panel_prepare,
	.enable = s6e3hc2_fhd_enable,
	.get_modes = exynos_panel_get_modes,
};

static const struct exynos_panel_funcs s6e3hc2_exynos_funcs = {
	.set_brightness = exynos_panel_set_brightness,
	.set_lp_mode = exynos_panel_set_lp_mode,
	.set_nolp_mode = s6e3hc2_set_nolp_mode,
	.set_binned_lp = exynos_panel_set_binned_lp,
	.set_hbm_mode = s6e3hc2_set_hbm_mode,
	.is_mode_seamless = s6e3hc2_is_mode_seamless,
	.mode_set = s6e3hc2_mode_set,
};

const struct brightness_capability s6e3hc2_brightness_capability = {
	.normal = {
		.nits = {
			.min = 2,
			.max = 450,
		},
		.level = {
			.min = 4,
			.max = 1023,
		},
		.percentage = {
			.min = 0,
			.max = 80,
		},
	},
	.hbm = {
		.nits = {
			.min = 600,
			.max = 600,
		},
		.level = { /* not valid for one-step hbm */
			.min = 1023,
			.max = 1023,
		},
		.percentage = {
			.min = 80,
			.max = 100,
		},
	},
};

const struct exynos_panel_desc samsung_s6e3hc2_wqhd = {
	.dsc_pps = SEQ_PPS_WQHD,
	.dsc_pps_len = ARRAY_SIZE(SEQ_PPS_WQHD),
	.data_lane_cnt = 4,
	.max_brightness = 1023,
	.dft_brightness = 511,
	.brt_capability = &s6e3hc2_brightness_capability,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2),
	.max_luminance = 5400000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.bl_range = s6e3hc2_bl_range,
	.bl_num_ranges = ARRAY_SIZE(s6e3hc2_bl_range),
	.modes = s6e3hc2_wqhd_modes,
	.num_modes = ARRAY_SIZE(s6e3hc2_wqhd_modes),
	.off_cmd_set = &s6e3hc2_off_cmd_set,
	.lp_mode = &s6e3hc2_wqhd_lp_mode,
	.lp_cmd_set = &s6e3hc2_lp_cmd_set,
	.binned_lp = s6e3hc2_binned_lp,
	.num_binned_lp = ARRAY_SIZE(s6e3hc2_binned_lp),
	.panel_func = &s6e3hc2_wqhd_drm_funcs,
	.exynos_panel_func = &s6e3hc2_exynos_funcs,
};

const struct exynos_panel_desc samsung_s6e3hc2_fhd = {
	.dsc_pps = SEQ_PPS_FHD,
	.dsc_pps_len = ARRAY_SIZE(SEQ_PPS_FHD),
	.data_lane_cnt = 4,
	.max_brightness = 1023,
	.dft_brightness = 511,
	.brt_capability = &s6e3hc2_brightness_capability,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2),
	.max_luminance = 5400000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.bl_range = s6e3hc2_bl_range,
	.bl_num_ranges = ARRAY_SIZE(s6e3hc2_bl_range),
	.modes = s6e3hc2_fhd_modes,
	.num_modes = ARRAY_SIZE(s6e3hc2_fhd_modes),
	.off_cmd_set = &s6e3hc2_off_cmd_set,
	.lp_mode = &s6e3hc2_fhd_lp_mode,
	.lp_cmd_set = &s6e3hc2_lp_cmd_set,
	.binned_lp = s6e3hc2_binned_lp,
	.num_binned_lp = ARRAY_SIZE(s6e3hc2_binned_lp),
	.panel_func = &s6e3hc2_fhd_drm_funcs,
	.exynos_panel_func = &s6e3hc2_exynos_funcs,
};

static const struct of_device_id exynos_panel_of_match[] = {
	{ .compatible = "samsung,s6e3hc2", .data = &samsung_s6e3hc2_wqhd },
	{ .compatible = "samsung,s6e3hc2-fhd", .data = &samsung_s6e3hc2_fhd },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_panel_of_match);

static struct mipi_dsi_driver exynos_panel_driver = {
	.probe = exynos_panel_probe,
	.remove = exynos_panel_remove,
	.driver = {
		.name = "panel-samsung-s6e3hc2",
		.of_match_table = exynos_panel_of_match,
	},
};
module_mipi_dsi_driver(exynos_panel_driver);

MODULE_AUTHOR("Jiun Yu <jiun.yu@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung s6e3hc2 panel driver");
MODULE_LICENSE("GPL");
