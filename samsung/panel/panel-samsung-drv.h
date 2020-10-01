/* SPDX-License-Identifier: GPL-2.0-only
 *
 * MIPI-DSI based Samsung common panel driver header
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PANEL_SAMSUNG_DRV_
#define _PANEL_SAMSUNG_DRV_

#include <linux/printk.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/backlight.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_property.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/exynos_display_common.h>

#include "../exynos_drm_connector.h"

#define MAX_REGULATORS		3
#define MAX_HDR_FORMATS		4

#define HDR_DOLBY_VISION	BIT(1)
#define HDR_HDR10		BIT(2)
#define HDR_HLG			BIT(3)

struct exynos_panel;
struct exynos_panel_funcs {
	/**
	 * @set_brightness:
	 *
	 * This callback is used to implement driver specific logic for brightness
	 * configuration. Otherwise defaults to sending brightness commands through
	 * dcs command update
	 */
	int (*set_brightness)(struct exynos_panel *exynos_panel, u16 br);

	/**
	 * @set_hbm_mode:
	 *
	 * This callback is used to implement panel specific logic for high brightness
	 * mode enablement. If this is not defined, it means that panel does not
	 * support HBM
	 */
	void (*set_hbm_mode)(struct exynos_panel *exynos_panel, bool hbm_mode);

	/**
	 * @set_local_hbm_mode:
	 *
	 * This callback is used to implement panel specific logic for local high
	 * brightness mode enablement. If this is not defined, it means that panel
	 * does not support local HBM
	 */
	void (*set_local_hbm_mode)(struct exynos_panel *exynos_panel,
				 bool local_hbm_en);
	/**
	 * @is_mode_seamless:
	 *
	 * This callback is used to check if a switch to a particular mode can be done
	 * seamlessly without full mode set given the current hardware configuration
	 */
	bool (*is_mode_seamless)(const struct exynos_panel *exynos_panel,
				 const struct drm_display_mode *mode);

	/**
	 * @mode_set:
	 *
	 * This callback is used to perform driver specific logic for mode_set.
	 * This could be called while display is on or off, should check internal
	 * state to perform appropriate mode set configuration depending on this state.
	 */
	void (*mode_set)(struct exynos_panel *exynos_panel,
			 const struct drm_display_mode *mode);
};

struct exynos_panel_desc {
	const u8 *dsc_pps;
	u32 dsc_pps_len;
	u32 data_lane_cnt;
	u32 hdr_formats; /* supported HDR formats bitmask */
	u32 max_luminance;
	u32 max_avg_luminance;
	u32 min_luminance;
	u32 max_brightness;
	u32 dft_brightness; /* default brightness */
	const struct drm_display_mode *modes;
	/* @lp_mode: provides a low power mode if available, otherwise null */
	const struct drm_display_mode *lp_mode;
	size_t num_modes;
	const struct drm_panel_funcs *panel_func;
	const struct exynos_panel_funcs *exynos_panel_func;
};


#define PANEL_ID_MAX		32
#define PANEL_EXTINFO_MAX	16
#define LOCAL_HBM_MAX_TIMEOUT_MS 1500 /* 1500 ms */

struct exynos_panel {
	struct device *dev;
	struct drm_panel panel;
	struct dentry *debugfs_entry;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;
	struct regulator *vci;
	struct regulator *vddi;
	struct exynos_drm_connector exynos_connector;
	struct drm_bridge bridge;
	const struct exynos_panel_desc *desc;
	const struct drm_display_mode *current_mode;
	struct backlight_device *bl;
	bool enabled;
	bool initialized;
	bool hbm_mode;

	char panel_id[PANEL_ID_MAX];
	char panel_extinfo[PANEL_EXTINFO_MAX];

	struct device_node *touch_dev;

	struct {
		/* indicate if local hbm enabled or not */
		bool enabled;
		/* max local hbm on period in ms */
		u32 max_timeout_ms;
		/* used to protect local hbm operation */
		struct mutex lock;
		/* workqueue used to turn off local hbm if reach max_timeout */
		struct workqueue_struct *wq;
		struct delayed_work timeout_work;
	} local_hbm;
};

static inline int exynos_dcs_write(struct exynos_panel *ctx, const void *data,
		size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	return mipi_dsi_dcs_write_buffer(dsi, data, len);
}

static inline int exynos_dcs_compression_mode(struct exynos_panel *ctx, bool enable)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	return mipi_dsi_compression_mode(dsi, enable);
}

static inline int exynos_dcs_set_brightness(struct exynos_panel *ctx, u16 br)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	return mipi_dsi_dcs_set_display_brightness(dsi, br);
}

static inline void exynos_bin2hex(const void *buf, size_t len,
				 char *linebuf, size_t linebuflen)
{
	const size_t max_size = (linebuflen - 1) / 2;
	const size_t count = min(max_size, len);
	char *end;

	end = bin2hex(linebuf, buf, count);
	*end = '\0';
}

#define EXYNOS_DCS_WRITE_SEQ(ctx, seq...) do {				\
	u8 d[] = { seq };						\
	int ret;							\
	ret = exynos_dcs_write(ctx, d, ARRAY_SIZE(d));			\
	if (ret < 0)							\
		dev_err(ctx->dev, "failed to write cmd(%d)\n", ret);	\
} while (0)

#define EXYNOS_DCS_WRITE_SEQ_DELAY(ctx, delay, seq...) do {		\
	EXYNOS_DCS_WRITE_SEQ(ctx, seq);					\
	msleep(delay);							\
} while (0)

#define EXYNOS_DCS_WRITE_TABLE(ctx, table) do {				\
	int ret;							\
	ret = exynos_dcs_write(ctx, table, ARRAY_SIZE(table));		\
	if (ret < 0)							\
		dev_err(ctx->dev, "failed to write cmd(%d)\n", ret);	\
} while (0)

#define EXYNOS_PPS_LONG_WRITE(ctx) do {					\
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);	\
	int ret;							\
	ret = mipi_dsi_picture_parameter_set(dsi,			\
		(struct drm_dsc_picture_parameter_set *)ctx->desc->dsc_pps);	\
	if (ret < 0)							\
		dev_err(ctx->dev, "failed to write cmd(%d)\n", ret);	\
} while (0)

int exynos_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector);
int exynos_panel_disable(struct drm_panel *panel);
int exynos_panel_unprepare(struct drm_panel *panel);
int exynos_panel_prepare(struct drm_panel *panel);
void exynos_panel_reset(struct exynos_panel *ctx);
int exynos_panel_set_power(struct exynos_panel *ctx, bool on);
int exynos_panel_set_brightness(struct exynos_panel *exynos_panel, u16 br);

int exynos_panel_probe(struct mipi_dsi_device *dsi);
int exynos_panel_remove(struct mipi_dsi_device *dsi);
#endif /* _PANEL_SAMSUNG_DRV_ */
