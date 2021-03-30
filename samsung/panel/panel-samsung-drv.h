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

#include "../exynos_drm_connector.h"

#define MAX_REGULATORS		3
#define MAX_HDR_FORMATS		4
#define MAX_BL_RANGES		10
#define MAX_TE2_TYPE		10

#define HDR_DOLBY_VISION	BIT(1)
#define HDR_HDR10		BIT(2)
#define HDR_HLG			BIT(3)

#define BL_STATE_STANDBY	BL_CORE_FBBLANK
#define BL_STATE_LP		BIT(30) /* backlight is in LP mode */

#define DEFAULT_GAMMA_STR	"default"

#define PANEL_REV_PROTO1	BIT(0)
#define PANEL_REV_PROTO1_1	BIT(1)
#define PANEL_REV_EVT		BIT(2)
#define PANEL_REV_DVT		BIT(3)
#define PANEL_REV_PVT		BIT(4)
#define PANEL_REV_ALL		(~0)
#define PANEL_REV_GE(rev)	(~((rev) - 1))
#define PANEL_REV_LT(rev)	((rev) - 1)
#define PANEL_REV_ALL_BUT(rev)	(PANEL_REV_ALL & ~(rev))

enum exynos_panel_state {
	PANEL_STATE_ON = 0,
	PANEL_STATE_LP,
	PANEL_STATE_OFF
};

struct exynos_panel;

struct exynos_panel_te2_timing {
	/* @rising_edge: vertical start point */
	u16 rising_edge;
	/* @falling_edge: vertical end point */
	u16 falling_edge;
};

/**
 * struct exynos_panel_mode - panel mode info
 */
struct exynos_panel_mode {
	/* @mode: drm display mode info */
	struct drm_display_mode mode;

	/* @exynos_mode: exynos driver specific mode info */
	struct exynos_display_mode exynos_mode;

	/* @priv_data: per mode panel driver private data */
	const void *priv_data;

	/* @te2_timing: TE2 signal timing */
	struct exynos_panel_te2_timing te2_timing;
};

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
	 * @set_lp_mode:
	 *
	 * This callback is used to handle command sequences to enter low power modes.
	 */
	void (*set_lp_mode)(struct exynos_panel *exynos_panel,
			    const struct exynos_panel_mode *mode);

	/**
	 * @set_nolp_mode:
	 *
	 * This callback is used to handle command sequences to exit from low power
	 * modes.
	 */
	void (*set_nolp_mode)(struct exynos_panel *exynos_panel,
			      const struct exynos_panel_mode *mode);

	/**
	 * @set_binned_lp:
	 *
	 * This callback is used to handle additional command sequences for low power
	 * modes based on different brightness threshold.
	 */
	void (*set_binned_lp)(struct exynos_panel *exynos_panel, u16 br);

	/**
	 * @set_hbm_mode:
	 *
	 * This callback is used to implement panel specific logic for high brightness
	 * mode enablement. If this is not defined, it means that panel does not
	 * support HBM
	 */
	void (*set_hbm_mode)(struct exynos_panel *exynos_panel, bool hbm_mode);

	/**
	 * @set_dimming_on:
	 *
	 * This callback is used to implement panel specific logic for dimming mode
	 * enablement. If this is not defined, it means that panel does not support
	 * dimmimg.
	 */
	void (*set_dimming_on)(struct exynos_panel *exynos_panel,
				 bool dimming_on);

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
				 const struct exynos_panel_mode *mode);

	/**
	 * @mode_set:
	 *
	 * This callback is used to perform driver specific logic for mode_set.
	 * This could be called while display is on or off, should check internal
	 * state to perform appropriate mode set configuration depending on this state.
	 */
	void (*mode_set)(struct exynos_panel *exynos_panel,
			 const struct exynos_panel_mode *mode);

	/**
	 * @panel_init:
	 *
	 * This callback is used to do one time initialization for any panel
	 * specific functions.
	 */
	void (*panel_init)(struct exynos_panel *exynos_panel);

	/**
	 * @print_gamma:
	 *
	 * This callback is used to print the hex dump of gamma address/data
	 * for the provided mode.
	 *
	 * The expected format:
	 * [gamma address]: [gamma data...]
	 */
	void (*print_gamma)(struct seq_file *seq,
			    const struct drm_display_mode *mode);

	/**
	 * @gamma_store:
	 *
	 * This callback is used to store the user-provided gamma table.
	 * The user-provided table should include FPS, register, register
	 * data size and gamma data.
	 *
	 * The expected format:
	 * [FPS1]
	 * [register1]
	 * [register1 data size]
	 * [gamma data]
	 * [register2]
	 * [register2 data size]
	 * [gamma data]
	 * [FPS2]
	 * [register1]
	 * .....
	 */
	ssize_t (*gamma_store)(struct exynos_panel *exynos_panel,
			       const char *buf, size_t len);

	/**
	 * @restore_native_gamma
	 *
	 * This callback is used to replace current gamma table by the
	 * original gamma.
	 */
	ssize_t (*restore_native_gamma)(struct exynos_panel *exynos_panel);

	/**
	 * @get_panel_rev:
	 *
	 * This callback is used to get panel HW revision from panel_extinfo.
	 */
	u32 (*get_panel_rev)(u32 id);

	/**
	 * @get_te2_edges:
	 *
	 * This callback is used to get the rising and falling edges of TE2 signal.
	 * The input buf is used to store the results in string.
	 */
	ssize_t (*get_te2_edges)(struct exynos_panel *exynos_panel,
				 char *buf, bool lp_mode);

	/**
	 * @configure_te2_edges:
	 *
	 * This callback is used to configure the rising and falling edges of TE2
	 * signal. The input timings include the values we need to configure.
	 */
	int (*configure_te2_edges)(struct exynos_panel *exynos_panel,
				   u32 *timings, bool lp_mode);

	/**
	 * @update_te2:
	 *
	 * This callback is used to update the TE2 signal via DCS commands.
	 * This should be called when the display state is changed between
	 * normal and LP modes, or the refresh rate and LP brightness are
	 * changed.
	 */
	void (*update_te2)(struct exynos_panel *exynos_panel);
};

/**
 * struct exynos_dsi_cmd - information for a dsi command.
 * @cmd_len:  Length of a dsi command.
 * @cmd:      Pointer to a dsi command.
 * @delay_ms: Delay time after executing this dsi command.
 * @panel_rev:Send the command only when the panel revision is matched.
 */
struct exynos_dsi_cmd {
	u32 cmd_len;
	const u8 *cmd;
	u32 delay_ms;
	u32 panel_rev;
	u8 type;
};

/**
 * struct exynos_dsi_cmd_set - a dsi command sequence.
 * @num_cmd:  Number of dsi commands in this sequence.
 * @cmds:     Pointer to a dsi command sequence.
 */
struct exynos_dsi_cmd_set {
	const u32 num_cmd;
	const struct exynos_dsi_cmd *cmds;
};

/**
 * struct exynos_binned_lp - information for binned lp mode.
 * @name:         Name of this binned lp mode.
 * @bl_threshold: Max brightness supported by this mode
 * @cmd_set:      A dsi command sequence to enter this mode.
 * @te2_timing:   TE2 signal timing.
 */
struct exynos_binned_lp {
	const char *name;
	u32 bl_threshold;
	struct exynos_dsi_cmd_set cmd_set;
	struct exynos_panel_te2_timing te2_timing;
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
	u32 min_brightness;
	u32 dft_brightness; /* default brightness */
	bool is_partial;
	const struct brightness_capability *brt_capability;
	const u32 *bl_range;
	u32 bl_num_ranges;
	const struct exynos_panel_mode *modes;
	size_t num_modes;
	const struct exynos_dsi_cmd_set *off_cmd_set;
	/* @lp_mode: provides a low power mode if available, otherwise null */
	const struct exynos_panel_mode *lp_mode;
	const struct exynos_dsi_cmd_set *lp_cmd_set;
	const struct exynos_binned_lp *binned_lp;
	const size_t num_binned_lp;
	const struct drm_panel_funcs *panel_func;
	const struct exynos_panel_funcs *exynos_panel_func;
};

#define PANEL_ID_MAX		32
#define PANEL_EXTINFO_MAX	16
#define LOCAL_HBM_MAX_TIMEOUT_MS 1500 /* 1500 ms */

struct exynos_bl_notifier {
	u32 ranges[MAX_BL_RANGES];
	u32 num_ranges;
	u32 current_range;
};

struct te2_mode_data {
	/* @mode: normal or LP mode data */
	const struct drm_display_mode *mode;
	/* @binned_lp: LP mode data */
	const struct exynos_binned_lp *binned_lp;
	/* @timing: normal or LP mode timing */
	struct exynos_panel_te2_timing timing;
};

struct te2_data {
	struct te2_mode_data mode_data[MAX_TE2_TYPE];
	struct mutex timing_lock;
};

struct exynos_panel {
	struct device *dev;
	struct drm_panel panel;
	struct dentry *debugfs_entry;
	struct dentry *debugfs_cmdset_entry;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;
	struct regulator *vci;
	struct regulator *vddi;
	struct regulator *vddd;
	struct regulator *vddr_en;
	struct regulator *vddr;
	struct exynos_drm_connector exynos_connector;
	struct drm_bridge bridge;
	const struct exynos_panel_desc *desc;
	const struct exynos_panel_mode *current_mode;
	bool enabled;
	bool initialized;

	bool hbm_mode;
	bool dimming_on;
	struct backlight_device *bl;
	struct mutex bl_state_lock;
	struct exynos_bl_notifier bl_notifier;

	struct mutex lp_state_lock;
	const struct exynos_binned_lp *current_binned_lp;

	char panel_id[PANEL_ID_MAX];
	char panel_extinfo[PANEL_EXTINFO_MAX];
	u32 panel_rev;
	bool is_secondary;

	struct device_node *touch_dev;

	struct te2_data te2;

	struct {
		struct local_hbm {
			/* indicate if local hbm enabled or not */
			bool enabled;
			/* max local hbm on period in ms */
			u32 max_timeout_ms;
			/* used to protect local hbm operation */
			struct mutex lock;
			/* work used to turn off local hbm if reach max_timeout */
			struct delayed_work timeout_work;
		} local_hbm;

		struct global_hbm {
			bool update_hbm;
			bool hbm_mode;
			bool update_bl;
			/* send ghbm mipi commands asynchronously after frame start */
			struct work_struct ghbm_work;
			struct mutex ghbm_work_lock;
		} global_hbm;
		struct workqueue_struct *wq;
	} hbm;
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

#define EXYNOS_DSI_CMD_REV(cmd, delay, rev) { sizeof(cmd), cmd, delay, rev }
#define EXYNOS_DSI_CMD(cmd, delay) EXYNOS_DSI_CMD_REV(cmd, delay, PANEL_REV_ALL)
#define EXYNOS_DSI_CMD0(cmd) EXYNOS_DSI_CMD(cmd, 0)

#define EXYNOS_DSI_CMD_SEQ_DELAY_REV(rev, delay, seq...) \
	{ sizeof((const u8 []){seq}), (const u8 []){seq}, delay, rev }
#define EXYNOS_DSI_CMD_SEQ_DELAY(delay, seq...) \
	EXYNOS_DSI_CMD_SEQ_DELAY_REV(PANEL_REV_ALL, delay, seq)
#define EXYNOS_DSI_CMD_SEQ_REV(rev, seq...) \
	EXYNOS_DSI_CMD_SEQ_DELAY_REV(rev, 0, seq)
#define EXYNOS_DSI_CMD_SEQ(seq...) EXYNOS_DSI_CMD_SEQ_DELAY(0, seq)

#define DEFINE_EXYNOS_CMD_SET(name) \
	const struct exynos_dsi_cmd_set name ## _cmd_set = {			\
		.num_cmd = ARRAY_SIZE(name ## _cmds), .cmds = name ## _cmds }

#define BINNED_LP_MODE(mode_name, bl_thr, cmdset)	\
{							\
	.name = mode_name,				\
	.bl_threshold = bl_thr,				\
	{ .num_cmd = ARRAY_SIZE(cmdset),		\
	  .cmds = cmdset },				\
	{ .rising_edge = 0,				\
	  .falling_edge = 0 }				\
}

#define BINNED_LP_MODE_TIMING(mode_name, bl_thr, cmdset, rising, falling)	\
{										\
	.name = mode_name,							\
	.bl_threshold = bl_thr,							\
	{ .num_cmd = ARRAY_SIZE(cmdset),					\
	  .cmds = cmdset },							\
	{ .rising_edge = rising,						\
	  .falling_edge = falling }						\
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
	usleep_range(delay * 1000, delay * 1000 + 10);			\
} while (0)

#define EXYNOS_DCS_WRITE_TABLE(ctx, table) do {				\
	int ret;							\
	ret = exynos_dcs_write(ctx, table, ARRAY_SIZE(table));		\
	if (ret < 0)							\
		dev_err(ctx->dev, "failed to write cmd(%d)\n", ret);	\
} while (0)

#define EXYNOS_DCS_WRITE_TABLE_DELAY(ctx, delay, table) do {		\
	EXYNOS_DCS_WRITE_TABLE(ctx, table);				\
	usleep_range(delay * 1000, delay * 1000 + 10);			\
} while (0)

#define EXYNOS_PPS_LONG_WRITE(ctx) do {					\
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);	\
	int ret;							\
	ret = mipi_dsi_picture_parameter_set(dsi,			\
		(struct drm_dsc_picture_parameter_set *)ctx->desc->dsc_pps);	\
	if (ret < 0)							\
		dev_err(ctx->dev, "failed to write cmd(%d)\n", ret);	\
} while (0)

#define for_each_display_mode(i, mode, ctx)			\
	for (i = 0, mode = &ctx->desc->modes[i].mode;		\
		i < ctx->desc->num_modes; i++,			\
		mode = &ctx->desc->modes[i].mode)		\

#define for_each_exynos_binned_lp(i, binned_lp, ctx)		\
	for (i = 0, binned_lp = &ctx->desc->binned_lp[i];	\
		i < ctx->desc->num_binned_lp; i++,		\
		binned_lp = &ctx->desc->binned_lp[i])		\

#define for_each_te2_timing(ctx, lp_mode, data, i)					\
	for (data = ctx->te2.mode_data + (!(lp_mode) ? 0 : (ctx)->desc->num_modes),	\
	i = !(lp_mode) ? (ctx)->desc->num_modes : (ctx)->desc->num_binned_lp - 1;	\
	i > 0;										\
	i--, data++)									\

int exynos_panel_configure_te2_edges(struct exynos_panel *ctx,
				     u32 *timings, bool lp_mode);
ssize_t exynos_panel_get_te2_edges(struct exynos_panel *ctx,
				   char *buf, bool lp_mode);
int exynos_panel_get_current_mode_te2(struct exynos_panel *ctx,
				      struct exynos_panel_te2_timing *timing);
int exynos_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector);
int exynos_panel_disable(struct drm_panel *panel);
int exynos_panel_unprepare(struct drm_panel *panel);
int exynos_panel_prepare(struct drm_panel *panel);
int exynos_panel_init(struct exynos_panel *ctx);
void exynos_panel_reset(struct exynos_panel *ctx);
int exynos_panel_set_power(struct exynos_panel *ctx, bool on);
int exynos_panel_set_brightness(struct exynos_panel *exynos_panel, u16 br);
void exynos_panel_debugfs_create_cmdset(struct exynos_panel *ctx,
					struct dentry *parent,
					const struct exynos_dsi_cmd_set *cmdset,
					const char *name);
void exynos_panel_send_cmd_set(struct exynos_panel *ctx,
			       const struct exynos_dsi_cmd_set *cmd_set);
void exynos_panel_set_lp_mode(struct exynos_panel *ctx, const struct exynos_panel_mode *pmode);
void exynos_panel_set_binned_lp(struct exynos_panel *ctx, const u16 brightness);
int exynos_panel_common_init(struct mipi_dsi_device *dsi,
				struct exynos_panel *ctx);

int exynos_panel_probe(struct mipi_dsi_device *dsi);
int exynos_panel_remove(struct mipi_dsi_device *dsi);
#endif /* _PANEL_SAMSUNG_DRV_ */
