// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based s6e3hc3 AMOLED LCD panel driver.
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <drm/drm_vblank.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <video/mipi_display.h>

#include "panel-samsung-drv.h"

static const unsigned char PPS_SETTING[] = {
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x0C, 0x30,
	0x05, 0xA0, 0x00, 0x34, 0x02, 0xD0, 0x02, 0xD0,
	0x02, 0x00, 0x02, 0x68, 0x00, 0x20, 0x05, 0xC6,
	0x00, 0x0A, 0x00, 0x0C, 0x01, 0xE2, 0x01, 0x78,
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

#define S6E3HC3_WRCTRLD_DIMMING_BIT    0x08
#define S6E3HC3_WRCTRLD_BCTRL_BIT      0x20
#define S6E3HC3_WRCTRLD_HBM_BIT        0xC0
#define S6E3HC3_WRCTRLD_LOCAL_HBM_BIT  0x10

#define S6E3HC3_TE2_CHANGEABLE 0x31
#define S6E3HC3_TE2_FIXED      0x41

static const u8 unlock_cmd_f0[] = { 0xF0, 0x5A, 0x5A };
static const u8 lock_cmd_f0[]   = { 0xF0, 0xA5, 0xA5 };
static const u8 hlpm_gamma[] = { 0x49, 0x01 };
static const u8 display_off[] = { 0x28 };
static const u8 display_on[] = { 0x29 };
static const u8 sleep_in[] = { 0x10 };

static const struct exynos_dsi_cmd s6e3hc3_off_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 20),
	EXYNOS_DSI_CMD(sleep_in, 100),
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc3_off);

static const struct exynos_dsi_cmd s6e3hc3_lp_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 17),
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc3_lp);

static const struct exynos_dsi_cmd s6e3hc3_lp_off_cmds[] = {
	EXYNOS_DSI_CMD(display_off, 0)
};

static const struct exynos_dsi_cmd s6e3hc3_lp_low_cmds[] = {
	EXYNOS_DSI_CMD(unlock_cmd_f0, 0),
	EXYNOS_DSI_CMD_SEQ(0x53, 0x25),	/* aod 10 nit */
	EXYNOS_DSI_CMD(hlpm_gamma, 0),
	EXYNOS_DSI_CMD(lock_cmd_f0, 17),
	EXYNOS_DSI_CMD(display_on, 0)
};

static const struct exynos_dsi_cmd s6e3hc3_lp_high_cmds[] = {
	EXYNOS_DSI_CMD(unlock_cmd_f0, 0),
	EXYNOS_DSI_CMD_SEQ(0x53, 0x24),	/* aod 50 nit */
	EXYNOS_DSI_CMD(hlpm_gamma, 0),
	EXYNOS_DSI_CMD(lock_cmd_f0, 17),
	EXYNOS_DSI_CMD(display_on, 0)
};

static const struct exynos_binned_lp s6e3hc3_binned_lp[] = {
	BINNED_LP_MODE("off", 0, s6e3hc3_lp_off_cmds),
	/* rising time = 0, falling time = 48 */
	BINNED_LP_MODE_TIMING("low", 80, s6e3hc3_lp_low_cmds, 0, 48),
	BINNED_LP_MODE_TIMING("high", 2047, s6e3hc3_lp_high_cmds, 0, 48)
};

static const u8 freq_update[] = { 0xF7, 0x0F };

static const struct exynos_dsi_cmd s6e3hc3_mode_60_cmds[] = {
	/* disable fast exit */
	EXYNOS_DSI_CMD_SEQ(0xBD, 0x21, 0x82),
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x10, 0xBD),
	EXYNOS_DSI_CMD_SEQ(0xBD, 0x00),
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x21, 0xBD),
	EXYNOS_DSI_CMD_SEQ(0xBD, 0x03, 0x00, 0x09, 0x00, 0x21, 0x00, 0x21, 0x00, 0x21, 0x00, 0x21,
			   0x00, 0x21, 0x00, 0x00, 0x00, 0x03, 0x00, 0x06, 0x00, 0x09, 0x00, 0x0C,
			   0x00, 0x0F, 0x00, 0x0F, 0x00, 0x0F),

	EXYNOS_DSI_CMD_SEQ(0xBD, 0x21),	/* manual mode */
	EXYNOS_DSI_CMD_SEQ(0x60, 0x01),	/* 60hz */
	EXYNOS_DSI_CMD0(freq_update),
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc3_mode_60);

static const struct exynos_dsi_cmd s6e3hc3_mode_120_cmds[] = {
	/* enable fast exit */
	EXYNOS_DSI_CMD_SEQ(0xBD, 0x21, 0x02),
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x10, 0xBD),
	EXYNOS_DSI_CMD_SEQ(0xBD, 0x10),
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x21, 0xBD),
	EXYNOS_DSI_CMD_SEQ(0xBD, 0x01, 0x00, 0x03, 0x00, 0x0B, 0x00, 0x0B, 0x00, 0x0B, 0x00, 0x0B,
			   0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			   0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x12, 0xBD),
	EXYNOS_DSI_CMD_SEQ(0xBD, 0x01, 0x00, 0x0B, 0x00, 0x03, 0x01), /* 10hz step setting */

	EXYNOS_DSI_CMD_SEQ(0xBD, 0x23),	/* auto mode */
	EXYNOS_DSI_CMD_SEQ(0x60, 0x00),	/* 120 hz */
	EXYNOS_DSI_CMD0(freq_update),
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc3_mode_120);

static u8 s6e3hc3_get_te2_option(struct exynos_panel *ctx)
{
	const struct drm_display_mode *mode;

	if (!ctx || !ctx->current_mode)
		return S6E3HC3_TE2_CHANGEABLE;

	/* proto 1.0 only supports changeable TE2 */
	if (ctx->panel_rev == PANEL_REV_PROTO1)
		return S6E3HC3_TE2_CHANGEABLE;

	mode = &ctx->current_mode->mode;

	if (ctx->current_mode->exynos_mode.is_lp_mode ||
	    drm_mode_vrefresh(mode) == 120)
		return S6E3HC3_TE2_FIXED;

	return S6E3HC3_TE2_CHANGEABLE;
}

static void s6e3hc3_update_te2(struct exynos_panel *ctx)
{
	struct exynos_panel_te2_timing timing;
	u8 width[7] = {0xB9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30}; /* default timing */
	u32 rising, falling;
	u8 option = s6e3hc3_get_te2_option(ctx);
	int ret;

	if (!ctx)
		return;

	ret = exynos_panel_get_current_mode_te2(ctx, &timing);
	if (!ret) {
		rising = timing.rising_edge;
		falling = timing.falling_edge;

		width[1] = (rising >> 8) & 0xF;
		width[2] = rising & 0xFF;

		if (option == S6E3HC3_TE2_CHANGEABLE) {
			width[3] = width[4] = 0;
		} else { /* S6E3HC3_TE2_FIXED */
			width[3] = width[1];
			width[4] = width[2];
		}

		width[5] = (falling >> 8) & 0xF;
		width[6] = falling & 0xFF;
	} else if (ret == -EAGAIN) {
		dev_dbg(ctx->dev, "Panel is not ready, use default setting\n");
	} else {
		return;
	}

	dev_dbg(ctx->dev,
		"TE2 updated: option %s, width 0xb9 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		(option == S6E3HC3_TE2_CHANGEABLE) ? "changeable" : "fixed",
		width[1], width[2], width[3], width[4], width[5], width[6]);

	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF0, 0x5A, 0x5A);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x00, 0x4F, 0xF2);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF2, 0x0D);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB9, 0x00, option);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x00, 0x14, 0xB9);
	EXYNOS_DCS_WRITE_TABLE(ctx, width);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF0, 0xA5, 0xA5);
}

static void s6e3hc3_change_frequency(struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	const struct exynos_dsi_cmd_set *mode_cmd_set;

	if (unlikely(!ctx))
		return;

	mode_cmd_set = pmode->priv_data;
	if (!mode_cmd_set)
		return;

	EXYNOS_DCS_WRITE_TABLE(ctx, unlock_cmd_f0);
	exynos_panel_send_cmd_set(ctx, mode_cmd_set);
	EXYNOS_DCS_WRITE_TABLE(ctx, lock_cmd_f0);

	dev_dbg(ctx->dev, "%s: change to %uhz\n", __func__, drm_mode_vrefresh(&pmode->mode));
}

static void s6e3hc3_write_display_mode(struct exynos_panel *ctx,
				       const struct drm_display_mode *mode)
{
	u8 val = S6E3HC3_WRCTRLD_BCTRL_BIT;

	if (ctx->hbm_mode)
		val |= S6E3HC3_WRCTRLD_HBM_BIT;

	if (ctx->hbm.local_hbm.enabled)
		val |= S6E3HC3_WRCTRLD_LOCAL_HBM_BIT;

	if (ctx->dimming_on)
		val |= S6E3HC3_WRCTRLD_DIMMING_BIT;

	dev_dbg(ctx->dev,
		"%s(wrctrld:0x%x, hbm: %s, dimming: %s local_hbm: %s)\n",
		__func__, val, ctx->hbm_mode ? "on" : "off",
		ctx->dimming_on ? "on" : "off",
		ctx->hbm.local_hbm.enabled ? "on" : "off");

	EXYNOS_DCS_WRITE_SEQ(ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, val);
}

static void s6e3hc3_set_nolp_mode(struct exynos_panel *ctx,
				  const struct exynos_panel_mode *pmode)
{
	unsigned int vrefresh = drm_mode_vrefresh(&pmode->mode);
	u32 delay_us = mult_frac(1000, 1020, vrefresh);

	if (!ctx->enabled)
		return;

	EXYNOS_DCS_WRITE_TABLE(ctx, display_off);
	usleep_range(delay_us, delay_us + 10);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xF0, 0x5A, 0x5A);
	s6e3hc3_write_display_mode(ctx, &pmode->mode);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0x49, 0x02);
	s6e3hc3_change_frequency(ctx, pmode);
	usleep_range(delay_us, delay_us + 10);
	EXYNOS_DCS_WRITE_TABLE(ctx, display_on);

	dev_info(ctx->dev, "exit LP mode\n");
}

static const struct exynos_dsi_cmd s6e3hc3_init_cmds[] = {
	EXYNOS_DSI_CMD_SEQ(0x9D, 0x01),			/* PPS_DSC_EN */

	EXYNOS_DSI_CMD_SEQ_DELAY(120, 0x11),		/* sleep out: 120ms delay */

	EXYNOS_DSI_CMD_SEQ(0x35),			/* TE on */

	EXYNOS_DSI_CMD0(unlock_cmd_f0),
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x22, 0xB9),	/* SEQ_GLOBAL_TSP_SYNC */
	EXYNOS_DSI_CMD_SEQ(0xB9, 0xB1, 0xA1),		/* SEQ_TSP_SYNC_ON */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x05, 0xF2),	/* SEQ_GLOBAL_TSP_SYNC */
	EXYNOS_DSI_CMD_SEQ(0xF2, 0x52),			/* SEQ_TSP_SYNC_ON */

	EXYNOS_DSI_CMD_SEQ(0x2A, 0x00, 0x00, 0x05, 0x9F), /* CASET */
	EXYNOS_DSI_CMD_SEQ(0x2B, 0x00, 0x00, 0x0C, 0x2F), /* PASET */
};
static DEFINE_EXYNOS_CMD_SET(s6e3hc3_init);

static int s6e3hc3_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	const struct exynos_panel_mode *pmode = ctx->current_mode;
	const struct drm_display_mode *mode;

	if (!pmode) {
		dev_err(ctx->dev, "no current mode set\n");
		return -EINVAL;
	}
	mode = &pmode->mode;

	dev_dbg(ctx->dev, "%s\n", __func__);

	exynos_panel_reset(ctx);

	/* DSC related configuration */
	EXYNOS_PPS_LONG_WRITE(ctx); /* PPS_SETTING */
	exynos_panel_send_cmd_set(ctx, &s6e3hc3_init_cmd_set);
	s6e3hc3_write_display_mode(ctx, mode); /* dimming and HBM */
	s6e3hc3_change_frequency(ctx, pmode);

	ctx->enabled = true;

	if (pmode->exynos_mode.is_lp_mode)
		exynos_panel_set_lp_mode(ctx, pmode);
	else
		EXYNOS_DCS_WRITE_TABLE(ctx, display_on);

	backlight_update_status(ctx->bl);

	return 0;
}

static void s6e3hc3_set_hbm_mode(struct exynos_panel *ctx,
				 bool hbm_mode)
{
	const struct exynos_panel_mode *pmode = ctx->current_mode;

	ctx->hbm_mode = hbm_mode;

	if (ctx->panel_rev == PANEL_REV_PROTO1) {
		EXYNOS_DCS_WRITE_SEQ(ctx, 0xF0, 0x5A, 0x5A);
		if (ctx->hbm_mode) {
			EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x00, 0x01, 0x49);
			EXYNOS_DCS_WRITE_SEQ(ctx, 0x49, 0x00);
			usleep_range(17000, 17010);
		} else {
			usleep_range(17000, 17010);
			EXYNOS_DCS_WRITE_SEQ(ctx, 0xB0, 0x00, 0x01, 0x49);
			EXYNOS_DCS_WRITE_SEQ(ctx, 0x49, 0x01);
		}
		EXYNOS_DCS_WRITE_SEQ(ctx, 0xF0, 0xA5, 0xA5);
	}
	s6e3hc3_write_display_mode(ctx, &pmode->mode);
}

static void s6e3hc3_set_dimming_on(struct exynos_panel *ctx,
				 bool dimming_on)
{
	const struct exynos_panel_mode *pmode = ctx->current_mode;

	ctx->dimming_on = dimming_on;
	s6e3hc3_write_display_mode(ctx, &pmode->mode);
}

static void s6e3hc3_set_local_hbm_mode(struct exynos_panel *ctx,
				 bool local_hbm_en)
{
	struct drm_mode_config *config;
	struct drm_crtc *crtc = NULL;

	if (ctx->hbm.local_hbm.enabled == local_hbm_en)
		return;

	mutex_lock(&ctx->hbm.local_hbm.lock);
	ctx->hbm.local_hbm.enabled = local_hbm_en;
	s6e3hc3_write_display_mode(ctx, &ctx->current_mode->mode);
	mutex_unlock(&ctx->hbm.local_hbm.lock);

	config = &ctx->exynos_connector.base.dev->mode_config;
	drm_modeset_lock(&config->connection_mutex, NULL);
	if (ctx->exynos_connector.base.state)
		crtc = ctx->exynos_connector.base.state->crtc;
	drm_modeset_unlock(&config->connection_mutex);
	if (crtc) {
		drm_crtc_wait_one_vblank(crtc);
		drm_crtc_wait_one_vblank(crtc);
	}
}

static void s6e3hc3_mode_set(struct exynos_panel *ctx,
			     const struct exynos_panel_mode *pmode)
{
	if (!ctx->enabled)
		return;

	s6e3hc3_change_frequency(ctx, pmode);
}

static bool s6e3hc3_is_mode_seamless(const struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	const struct drm_display_mode *c = &ctx->current_mode->mode;
	const struct drm_display_mode *n = &pmode->mode;

	/* seamless mode set can happen if active region resolution is same */
	return (c->vdisplay == n->vdisplay) && (c->hdisplay == n->hdisplay) &&
	       (c->flags == n->flags);
}

static void s6e3hc3_panel_init(struct exynos_panel *ctx)
{
	struct dentry *csroot = ctx->debugfs_cmdset_entry;

	exynos_panel_debugfs_create_cmdset(ctx, csroot, &s6e3hc3_init_cmd_set, "init");
	exynos_panel_debugfs_create_cmdset(ctx, csroot, &s6e3hc3_mode_120_cmd_set, "120");
	exynos_panel_debugfs_create_cmdset(ctx, csroot, &s6e3hc3_mode_60_cmd_set, "60");
}

static u32 s6e3hc3_get_panel_rev(u32 id)
{
	u8 build_code;

	/* extract command 0xDB */
	build_code = (id & 0xFF00) >> 8;

	return (((build_code & 0xE0) >> 3) | (build_code & 0x03));
}

static const struct exynos_display_underrun_param underrun_param = {
	.te_idle_us = 350,
	.te_var = 1,
};

static const u32 s6e3hc3_bl_range[] = {
	94, 180, 270, 360, 2047
};

static const struct exynos_panel_mode s6e3hc3_modes[] = {
	{
		/* 1440x3120 @ 60Hz */
		.mode = {
			.clock = 298620,
			.hdisplay = 1440,
			.hsync_start = 1440 + 80, // add hfp
			.hsync_end = 1440 + 80 + 24, // add hsa
			.htotal = 1440 + 80 + 24 + 36, // add hbp
			.vdisplay = 3120,
			.vsync_start = 3120 + 12, // add vfp
			.vsync_end = 3120 + 12 + 4, // add vsa
			.vtotal = 3120 + 12 + 4 + 14, // add vbp
			.flags = 0,
			.width_mm = 71,
			.height_mm = 155,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 52,
			},
			.underrun_param = &underrun_param,
		},
		.priv_data = &s6e3hc3_mode_60_cmd_set,
		.te2_timing = {
			.rising_edge = 0,
			.falling_edge = 48,
		},
	},
	{
		/* 1440x3120 @ 120Hz */
		.mode = {
			.clock = 597240,
			.hdisplay = 1440,
			.hsync_start = 1440 + 80, // add hfp
			.hsync_end = 1440 + 80 + 24, // add hsa
			.htotal = 1440 + 80 + 24 + 36, // add hbp
			.vdisplay = 3120,
			.vsync_start = 3120 + 12, // add vfp
			.vsync_end = 3120 + 12 + 4, // add vsa
			.vtotal = 3120 + 12 + 4 + 14, // add vbp
			.flags = 0,
			.width_mm = 71,
			.height_mm = 155,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.bpc = 8,
			.dsc = {
				.enabled = true,
				.dsc_count = 2,
				.slice_count = 2,
				.slice_height = 52,
			},
			.underrun_param = &underrun_param,
		},
		.priv_data = &s6e3hc3_mode_120_cmd_set,
		.te2_timing = {
			.rising_edge = 0,
			.falling_edge = 48,
		},
	},
};

static const struct exynos_panel_mode s6e3hc3_lp_mode = {
	.mode = {
		/* 1440x3120 @ 30Hz */
		.name = "1440x3120x30",
		.clock = 149310,
		.hdisplay = 1440,
		.hsync_start = 1440 + 80, // add hfp
		.hsync_end = 1440 + 80 + 24, // add hsa
		.htotal = 1440 + 80 + 24 + 36, // add hbp
		.vdisplay = 3120,
		.vsync_start = 3120 + 12, // add vfp
		.vsync_end = 3120 + 12 + 4, // add vsa
		.vtotal = 3120 + 12 + 4 + 14, // add vbp
		.flags = 0,
		.width_mm = 71,
		.height_mm = 155,
	},
	.exynos_mode = {
		.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
		.vblank_usec = 120,
		.bpc = 8,
		.dsc = {
			.enabled = true,
			.dsc_count = 2,
			.slice_count = 2,
			.slice_height = 52,
		},
		.underrun_param = &underrun_param,
		.is_lp_mode = true,
	}
};

static const struct drm_panel_funcs s6e3hc3_drm_funcs = {
	.disable = exynos_panel_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = exynos_panel_prepare,
	.enable = s6e3hc3_enable,
	.get_modes = exynos_panel_get_modes,
};

static const struct exynos_panel_funcs s6e3hc3_exynos_funcs = {
	.set_brightness = exynos_panel_set_brightness,
	.set_lp_mode = exynos_panel_set_lp_mode,
	.set_nolp_mode = s6e3hc3_set_nolp_mode,
	.set_binned_lp = exynos_panel_set_binned_lp,
	.set_hbm_mode = s6e3hc3_set_hbm_mode,
	.set_dimming_on = s6e3hc3_set_dimming_on,
	.set_local_hbm_mode = s6e3hc3_set_local_hbm_mode,
	.is_mode_seamless = s6e3hc3_is_mode_seamless,
	.mode_set = s6e3hc3_mode_set,
	.panel_init = s6e3hc3_panel_init,
	.get_panel_rev = s6e3hc3_get_panel_rev,
	.get_te2_edges = exynos_panel_get_te2_edges,
	.configure_te2_edges = exynos_panel_configure_te2_edges,
	.update_te2 = s6e3hc3_update_te2,
};

const struct brightness_capability s6e3hc3_brightness_capability = {
	.normal = {
		.nits = {
			.min = 2,
			.max = 500,
		},
		.level = {
			.min = 4,
			.max = 2047,
		},
		.percentage = {
			.min = 0,
			.max = 80,
		},
	},
	.hbm = {
		.nits = {
			.min = 550,
			.max = 800,
		},
		.level = {
			.min = 3311,
			.max = 4095,
		},
		.percentage = {
			.min = 80,
			.max = 100,
		},
	},
};

const struct exynos_panel_desc samsung_s6e3hc3 = {
	.dsc_pps = PPS_SETTING,
	.dsc_pps_len = ARRAY_SIZE(PPS_SETTING),
	.data_lane_cnt = 4,
	.max_brightness = 4095,
	.dft_brightness = 1023,
	.brt_capability = &s6e3hc3_brightness_capability,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2),
	.max_luminance = 5400000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.bl_range = s6e3hc3_bl_range,
	.bl_num_ranges = ARRAY_SIZE(s6e3hc3_bl_range),
	.modes = s6e3hc3_modes,
	.num_modes = ARRAY_SIZE(s6e3hc3_modes),
	.off_cmd_set = &s6e3hc3_off_cmd_set,
	.lp_mode = &s6e3hc3_lp_mode,
	.lp_cmd_set = &s6e3hc3_lp_cmd_set,
	.binned_lp = s6e3hc3_binned_lp,
	.num_binned_lp = ARRAY_SIZE(s6e3hc3_binned_lp),
	.panel_func = &s6e3hc3_drm_funcs,
	.exynos_panel_func = &s6e3hc3_exynos_funcs,
};

static const struct of_device_id exynos_panel_of_match[] = {
	{ .compatible = "samsung,s6e3hc3", .data = &samsung_s6e3hc3 },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_panel_of_match);

static struct mipi_dsi_driver exynos_panel_driver = {
	.probe = exynos_panel_probe,
	.remove = exynos_panel_remove,
	.driver = {
		.name = "panel-samsung-s6e3hc3",
		.of_match_table = exynos_panel_of_match,
	},
};
module_mipi_dsi_driver(exynos_panel_driver);

MODULE_AUTHOR("Jiun Yu <jiun.yu@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung s6e3hc3 panel driver");
MODULE_LICENSE("GPL");
