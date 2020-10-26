// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based Samsung common panel driver.
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_encoder.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <video/mipi_display.h>

#include <drm/exynos_display_common.h>

#include "../exynos_drm_connector.h"
#include "panel-samsung-drv.h"

#define PANEL_ID_REG		0xA1
#define PANEL_ID_LEN		7
#define PANEL_ID_OFFSET		6
#define PANEL_ID_READ_SIZE	(PANEL_ID_LEN + PANEL_ID_OFFSET)

static const char ext_info_regs[] = { 0xDA, 0xDB, 0xDC };

#define exynos_connector_to_panel(c)					\
	container_of((c), struct exynos_panel, exynos_connector)

#define bridge_to_exynos_panel(b) \
	container_of((b), struct exynos_panel, bridge)

static void exynos_panel_set_backlight_state(struct exynos_panel *ctx,
					enum exynos_panel_state panel_state);

static inline bool in_tui(struct exynos_panel *ctx)
{
	const struct drm_connector_state *conn_state = ctx->exynos_connector.base.state;

	if (conn_state && conn_state->crtc) {
		const struct drm_crtc_state *crtc_state =
						conn_state->crtc->state;

		if (crtc_state && (crtc_state->adjusted_mode.private_flags &
				 EXYNOS_DISPLAY_MODE_FLAG_TUI))
			return true;
	}

	return false;
}

static inline bool is_backlight_off_state(const struct backlight_device *bl)
{
	return (bl->props.state & BL_STATE_STANDBY) != 0;
}

static inline bool is_backlight_lp_state(const struct backlight_device *bl)
{
	return (bl->props.state & BL_STATE_LP) != 0;
}

static void backlight_state_changed(struct backlight_device *bl)
{
	sysfs_notify(&bl->dev.kobj, NULL, "state");
}

static int exynos_panel_parse_gpios(struct exynos_panel *ctx)
{
	struct device *dev = ctx->dev;

	dev_dbg(ctx->dev, "%s +\n", __func__);

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR)) {
		dev_info(ctx->dev, "no reset/enable pins on emulator\n");
		return 0;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "failed to get reset-gpios %ld",
				PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	ctx->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable_gpio))
		ctx->enable_gpio = NULL;

	dev_dbg(ctx->dev, "%s -\n", __func__);
	return 0;
}

static int exynos_panel_parse_regulators(struct exynos_panel *ctx)
{
	struct device *dev = ctx->dev;

	ctx->vddi = devm_regulator_get(dev, "vddi");
	if (IS_ERR(ctx->vddi)) {
		dev_warn(ctx->dev, "failed to get panel vddi.\n");
		return -EPROBE_DEFER;
	}

	ctx->vci = devm_regulator_get(dev, "vci");
	if (IS_ERR(ctx->vci)) {
		dev_warn(ctx->dev, "failed to get panel vci.\n");
		return -EPROBE_DEFER;
	}

	return 0;
}

static int exynos_panel_read_id(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	char buf[PANEL_ID_READ_SIZE];
	int ret;

	ret = mipi_dsi_dcs_read(dsi, PANEL_ID_REG, buf, PANEL_ID_READ_SIZE);
	if (ret != PANEL_ID_READ_SIZE) {
		dev_warn(ctx->dev, "Unable to read panel id (%d)\n", ret);
		return ret;
	}

	exynos_bin2hex(buf + PANEL_ID_OFFSET, PANEL_ID_LEN,
		       ctx->panel_id, sizeof(ctx->panel_id));

	return 0;
}

static int exynos_panel_read_extinfo(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	const size_t extinfo_len = ARRAY_SIZE(ext_info_regs);
	char buf[extinfo_len];
	int i, ret;

	for (i = 0; i < extinfo_len; i++) {
		ret = mipi_dsi_dcs_read(dsi, ext_info_regs[i], buf + i, 1);
		if (ret != 1) {
			dev_warn(ctx->dev,
				 "Unable to read panel extinfo (0x%x: %d)\n",
				 ext_info_regs[i], ret);
			return ret;
		}

	}
	exynos_bin2hex(buf, i, ctx->panel_extinfo, sizeof(ctx->panel_extinfo));

	return 0;
}

static int exynos_panel_init(struct exynos_panel *ctx)
{
	int ret;

	if (ctx->initialized)
		return 0;


	ret = exynos_panel_read_id(ctx);
	if (ret)
		return ret;

	ret = exynos_panel_read_extinfo(ctx);
	if (!ret)
		ctx->initialized = true;

	return ret;
}

void exynos_panel_reset(struct exynos_panel *ctx)
{
	dev_dbg(ctx->dev, "%s +\n", __func__);

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return;

	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);

	dev_dbg(ctx->dev, "%s -\n", __func__);

	exynos_panel_init(ctx);
}
EXPORT_SYMBOL(exynos_panel_reset);

int exynos_panel_set_power(struct exynos_panel *ctx, bool on)
{
	int ret;

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return 0;

	if (on) {
		if (ctx->enable_gpio) {
			gpiod_set_value(ctx->enable_gpio, 1);
			usleep_range(10000, 11000);
		}

		if (ctx->vddi) {
			ret = regulator_enable(ctx->vddi);
			if (ret) {
				dev_err(ctx->dev, "vddi enable failed\n");
				return ret;
			}
			usleep_range(5000, 6000);
		}

		if (ctx->vci) {
			ret = regulator_enable(ctx->vci);
			if (ret) {
				dev_err(ctx->dev, "vci enable failed\n");
				return ret;
			}
		}
	} else {
		gpiod_set_value(ctx->reset_gpio, 0);
		if (ctx->enable_gpio)
			gpiod_set_value(ctx->enable_gpio, 0);

		if (ctx->vddi) {
			ret = regulator_disable(ctx->vddi);
			if (ret) {
				dev_err(ctx->dev, "vddi disable failed\n");
				return ret;
			}
		}

		if (ctx->vci > 0) {
			ret = regulator_disable(ctx->vci);
			if (ret) {
				dev_err(ctx->dev, "vci disable failed\n");
				return ret;
			}
		}
	}

	ctx->bl->props.power = on ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;

	return 0;
}
EXPORT_SYMBOL(exynos_panel_set_power);

static void exynos_panel_handoff(struct exynos_panel *ctx)
{
	ctx->enabled = gpiod_get_raw_value(ctx->reset_gpio) > 0;
	if (ctx->enabled) {
		dev_info(ctx->dev, "panel enabled at boot\n");
		exynos_panel_set_power(ctx, true);
	} else {
		gpiod_direction_output(ctx->reset_gpio, 0);
	}
}

static int exynos_panel_parse_dt(struct exynos_panel *ctx)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(ctx->dev->of_node)) {
		dev_err(ctx->dev, "no device tree information of exynos panel\n");
		return -EINVAL;
	}

	ret = exynos_panel_parse_gpios(ctx);
	if (ret)
		goto err;

	ret = exynos_panel_parse_regulators(ctx);
	if (ret)
		goto err;

	ctx->touch_dev = of_parse_phandle(ctx->dev->of_node, "touch", 0);
	ctx->is_secondary = of_property_read_bool(ctx->dev->of_node, "is_secondary");

err:
	return ret;
}

static void exynos_panel_mode_set_name(struct drm_display_mode *mode)
{
	scnprintf(mode->name, DRM_DISPLAY_MODE_LEN, "%dx%dx%d",
		  mode->hdisplay, mode->vdisplay, drm_mode_vrefresh(mode));
}

int exynos_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);
	struct drm_display_mode *preferred_mode = NULL;
	const struct exynos_panel_mode *current_mode = ctx->current_mode;
	int i;

	dev_dbg(ctx->dev, "%s +\n", __func__);

	for (i = 0; i < ctx->desc->num_modes; i++) {
		const struct exynos_panel_mode *pmode = &ctx->desc->modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &pmode->mode);
		if (!mode)
			return -ENOMEM;

		if (!mode->name[0])
			exynos_panel_mode_set_name(mode);

		mode->type |= DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);

		dev_dbg(ctx->dev, "added display mode: %s\n", mode->name);

		if (!preferred_mode || (mode->type & DRM_MODE_TYPE_PREFERRED)) {
			preferred_mode = mode;
			/* if enabled at boot, assume preferred mode was set */
			if (ctx->enabled && !current_mode)
				ctx->current_mode = pmode;
		}
	}

	if (preferred_mode) {
		dev_dbg(ctx->dev, "preferred display mode: %s\n", preferred_mode->name);
		preferred_mode->type |= DRM_MODE_TYPE_PREFERRED;
		connector->display_info.width_mm = preferred_mode->width_mm;
		connector->display_info.height_mm = preferred_mode->height_mm;
	}

	dev_dbg(ctx->dev, "%s -\n", __func__);

	return i;
}
EXPORT_SYMBOL(exynos_panel_get_modes);

int exynos_panel_disable(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);
	const struct exynos_panel_funcs *exynos_panel_func;

	ctx->enabled = false;
	ctx->hbm_mode = false;

	exynos_panel_func = ctx->desc->exynos_panel_func;
	if (exynos_panel_func && exynos_panel_func->set_local_hbm_mode) {
		ctx->local_hbm.enabled = false;
		cancel_delayed_work_sync(&ctx->local_hbm.timeout_work);
	}

	dev_dbg(ctx->dev, "%s +\n", __func__);
	return 0;
}
EXPORT_SYMBOL(exynos_panel_disable);

int exynos_panel_unprepare(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);

	dev_dbg(ctx->dev, "%s +\n", __func__);
	exynos_panel_set_power(ctx, false);
	dev_dbg(ctx->dev, "%s -\n", __func__);
	return 0;
}
EXPORT_SYMBOL(exynos_panel_unprepare);

int exynos_panel_prepare(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);

	dev_dbg(ctx->dev, "%s +\n", __func__);
	exynos_panel_set_power(ctx, true);
	dev_dbg(ctx->dev, "%s -\n", __func__);

	return 0;
}
EXPORT_SYMBOL(exynos_panel_prepare);

void exynos_panel_send_cmd_set(struct exynos_panel *ctx,
			       const struct exynos_dsi_cmd_set *cmd_set)
{
	int i;

	if (!cmd_set)
		return;

	for (i = 0; i < cmd_set->num_cmd; i++) {
		u32 delay_ms = cmd_set->cmds[i].delay_ms;

		exynos_dcs_write(ctx, cmd_set->cmds[i].cmd,
				cmd_set->cmds[i].cmd_len);
		if (delay_ms)
			usleep_range(delay_ms * 1000, delay_ms * 1000 + 10);
	}
}
EXPORT_SYMBOL(exynos_panel_send_cmd_set);

void exynos_panel_set_lp_mode(struct exynos_panel *ctx, const struct exynos_panel_mode *pmode)
{
	if (!ctx->enabled)
		return;

	exynos_panel_send_cmd_set(ctx, ctx->desc->lp_cmd_set);

	dev_info(ctx->dev, "enter %dhz LP mode\n", drm_mode_vrefresh(&pmode->mode));
}
EXPORT_SYMBOL(exynos_panel_set_lp_mode);

void exynos_panel_set_binned_lp(struct exynos_panel *ctx, const u16 brightness)
{
	int i;
	const struct exynos_binned_lp *binned_lp;

	for (i = 0; i < ctx->desc->num_binned_lp; i++) {
		binned_lp = &ctx->desc->binned_lp[i];
		if (brightness <= binned_lp->bl_threshold)
			break;
	}
	if (i == ctx->desc->num_binned_lp)
		return;

	exynos_panel_send_cmd_set(ctx, &binned_lp->cmd_set);

	exynos_panel_set_backlight_state(ctx,
		!binned_lp->bl_threshold ? PANEL_STATE_OFF : PANEL_STATE_LP);
}
EXPORT_SYMBOL(exynos_panel_set_binned_lp);

int exynos_panel_set_brightness(struct exynos_panel *exynos_panel, u16 br)
{
	u16 brightness;

	if (exynos_panel->current_mode->exynos_mode.is_lp_mode) {
		const struct exynos_panel_funcs *funcs;

		funcs = exynos_panel->desc->exynos_panel_func;
		if (funcs && funcs->set_binned_lp)
			funcs->set_binned_lp(exynos_panel, br);
		return 0;
	}

	brightness = (br & 0xff) << 8 | br >> 8;

	return exynos_dcs_set_brightness(exynos_panel, brightness);
}
EXPORT_SYMBOL(exynos_panel_set_brightness);

static int exynos_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int exynos_update_status(struct backlight_device *bl)
{
	struct exynos_panel *ctx = bl_get_data(bl);
	const struct exynos_panel_funcs *exynos_panel_func;
	int brightness = bl->props.brightness;

	if (!ctx->enabled || !ctx->initialized) {
		dev_dbg(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	/* check if backlight is forced off */
	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	dev_info(ctx->dev, "req: %d, br: %d\n", bl->props.brightness,
		brightness);

	exynos_panel_func = ctx->desc->exynos_panel_func;
	if (exynos_panel_func && exynos_panel_func->set_brightness)
		exynos_panel_func->set_brightness(ctx, brightness);
	else
		exynos_dcs_set_brightness(ctx, brightness);

	return 0;
}

static const struct backlight_ops exynos_backlight_ops = {
	.get_brightness = exynos_get_brightness,
	.update_status = exynos_update_status,
};

static ssize_t serial_number_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!ctx->initialized)
		return -EPERM;

	return snprintf(buf, PAGE_SIZE, "%s\n", ctx->panel_id);
}

static ssize_t panel_extinfo_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!ctx->initialized)
		return -EPERM;

	return snprintf(buf, PAGE_SIZE, "%s\n", ctx->panel_extinfo);
}

static ssize_t panel_name_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n", dsi->name);
}

static DEVICE_ATTR_RO(serial_number);
static DEVICE_ATTR_RO(panel_extinfo);
static DEVICE_ATTR_RO(panel_name);

static const struct attribute *panel_attrs[] = {
	&dev_attr_serial_number.attr,
	&dev_attr_panel_extinfo.attr,
	&dev_attr_panel_name.attr,
	NULL
};

static void exynos_panel_connector_print_state(struct drm_printer *p,
					       const struct exynos_drm_connector_state *state)
{
	const struct exynos_drm_connector *exynos_connector =
		to_exynos_connector(state->base.connector);
	const struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);
	const struct exynos_panel_desc *desc = ctx->desc;

	drm_printf(p, "\tenabled: %d\n", ctx->enabled);
	if (ctx->current_mode) {
		const struct drm_display_mode *m = &ctx->current_mode->mode;

		drm_printf(p, " \tcurrent mode: %dx%d@%d\n", m->hdisplay,
			   m->vdisplay, drm_mode_vrefresh(m));
	}
	drm_printf(p, "\text_info: %s\n", ctx->panel_extinfo);
	drm_printf(p, "\tluminance: [%u, %u] avg: %u\n",
		   desc->min_luminance, desc->max_luminance,
		   desc->max_avg_luminance);
	drm_printf(p, "\thdr_formats: 0x%x\n", desc->hdr_formats);
}

static const struct exynos_drm_connector_funcs exynos_panel_connector_funcs = {
	.atomic_print_state = exynos_panel_connector_print_state,
};

static int exynos_drm_connector_modes(struct drm_connector *connector)
{
	struct exynos_drm_connector *exynos_connector = to_exynos_connector(connector);
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);
	int ret;

	ret = drm_panel_get_modes(&ctx->panel, connector);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to get panel display modes\n");
		return ret;
	}

	return ret;
}

static const struct exynos_panel_mode *exynos_panel_get_mode(struct exynos_panel *ctx,
							     const struct drm_display_mode *mode)
{
	const struct exynos_panel_mode *pmode;
	int i;

	for (i = 0; i < ctx->desc->num_modes; i++) {
		pmode = &ctx->desc->modes[i];

		if (drm_mode_equal(&pmode->mode, mode))
			return pmode;
	}

	pmode = ctx->desc->lp_mode;
	if (pmode && drm_mode_equal(&pmode->mode, mode))
		return pmode;

	return NULL;
}

static void exynos_drm_connector_attach_touch(struct exynos_panel *ctx,
					      const struct drm_connector_state *connector_state,
					      const struct drm_crtc_state *crtc_state)
{
	struct drm_encoder *encoder = connector_state->best_encoder;
	struct drm_bridge *bridge;

	if (!encoder) {
		dev_warn(ctx->dev, "%s encoder is null\n", __func__);
		return;
	}

	bridge = of_drm_find_bridge(ctx->touch_dev);
	if (!bridge || bridge->dev)
		return;

	drm_bridge_attach(encoder, bridge, NULL, 0);
	dev_info(ctx->dev, "attach bridge %p to encoder %p\n", bridge, encoder);
}

/*
 * Check whether transition to new mode can be done seamlessly without having
 * to turn display off before mode change. This is currently only possible if
 * only clocks/refresh rate is changing
 */
static bool exynos_panel_is_mode_seamless(const struct exynos_panel *ctx,
					  const struct exynos_panel_mode *mode)
{
	const struct exynos_panel_funcs *funcs;

	/* no need to go through seamless mode set if panel is disabled */
	if (!ctx->enabled || !ctx->initialized)
		return false;

	funcs = ctx->desc->exynos_panel_func;
	if (!funcs || !funcs->is_mode_seamless)
		return false;

	return funcs->is_mode_seamless(ctx, mode);
}

static int exynos_drm_connector_check_mode(struct exynos_panel *ctx,
					   struct drm_connector_state *connector_state,
					   const struct drm_display_mode *mode)
{
	struct exynos_drm_connector_state *exynos_connector_state =
		to_exynos_connector_state(connector_state);
	const struct exynos_panel_mode *pmode = exynos_panel_get_mode(ctx, mode);

	if (!pmode) {
		dev_warn(ctx->dev, "invalid mode %s\n", mode->name);
		return -EINVAL;
	}

	exynos_connector_state->seamless_possible = exynos_panel_is_mode_seamless(ctx, pmode);
	exynos_connector_state->exynos_mode = pmode->exynos_mode;

	return 0;
}

static int exynos_drm_connector_atomic_check(struct drm_connector *connector,
					     struct drm_atomic_state *state)
{
	struct exynos_drm_connector *exynos_connector = to_exynos_connector(connector);
	struct drm_connector_state *connector_state =
		drm_atomic_get_new_connector_state(state, connector);
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);
	struct drm_crtc_state *crtc_state;

	/* nothing to do if disabled or if mode is unchanged */
	if (!connector_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, connector_state->crtc);
	if (!drm_atomic_crtc_needs_modeset(crtc_state))
		return 0;

	if (ctx->touch_dev)
		exynos_drm_connector_attach_touch(ctx, connector_state, crtc_state);

	 return exynos_drm_connector_check_mode(ctx, connector_state, &crtc_state->mode);
}

static const struct drm_connector_helper_funcs exynos_connector_helper_funcs = {
	.atomic_check = exynos_drm_connector_atomic_check,
	.get_modes = exynos_drm_connector_modes,
};

#ifdef CONFIG_DEBUG_FS
static ssize_t hbm_mode_write(struct file *file, const char *user_buf,
			      size_t count, loff_t *f_pos)
{
	struct seq_file *m = file->private_data;
	struct exynos_panel *ctx = m->private;
	const struct exynos_panel_funcs *exynos_panel_func;
	int ret;
	bool hbm_en;

	if (!ctx->enabled || !ctx->initialized) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	exynos_panel_func = ctx->desc->exynos_panel_func;
	if (!exynos_panel_func || !exynos_panel_func->set_hbm_mode) {
		dev_err(ctx->dev, "hbm is not support\n");
		return -EPERM;
	}

	ret = kstrtobool_from_user(user_buf, count, &hbm_en);
	if (ret) {
		dev_err(ctx->dev, "invalid hbm_mode value\n");
		return ret;
	}

	exynos_panel_func->set_hbm_mode(ctx, hbm_en);

	if (ctx->bl)
		backlight_state_changed(ctx->bl);

	return count;
}

static int hbm_mode_show(struct seq_file *m, void *data)
{
	struct exynos_panel *ctx = m->private;

	seq_printf(m, "%d\n", ctx->hbm_mode);

	return 0;
}

static int hbm_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, hbm_mode_show, inode->i_private);
}

static const struct file_operations hbm_mode_fops = {
	.owner          = THIS_MODULE,
	.open           = hbm_mode_open,
	.write          = hbm_mode_write,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int hbm_mode_debugfs_add(struct exynos_panel *ctx, struct dentry *parent)
{
	if (!debugfs_create_file("hbm_mode", 0600, parent, ctx, &hbm_mode_fops))
		return -ENOMEM;

	return 0;
}

static ssize_t exynos_dsi_dcs_transfer(struct mipi_dsi_device *dsi, u8 type,
				     const void *data, size_t len)
{
	const struct mipi_dsi_host_ops *ops = dsi->host->ops;
	struct mipi_dsi_msg msg = {
		.channel = dsi->channel,
		.tx_buf = data,
		.tx_len = len,
		.type = type,
	};

	if (!ops || !ops->transfer)
		return -ENOSYS;

	if (dsi->mode_flags & MIPI_DSI_MODE_LPM)
		msg.flags |= MIPI_DSI_MSG_USE_LPM;
	msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;

	return ops->transfer(dsi->host, &msg);
}


static ssize_t exynos_dsi_dcs_write_buffer(struct mipi_dsi_device *dsi,
				  const void *data, size_t len)
{
	u8 type;

	switch (len) {
	case 0:
		return -EINVAL;

	case 1:
		type = MIPI_DSI_DCS_SHORT_WRITE;
		break;

	case 2:
		type = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
		break;

	default:
		type = MIPI_DSI_DCS_LONG_WRITE;
		break;
	}

	return exynos_dsi_dcs_transfer(dsi, type, data, len);
}

static int exynos_dsi_name_show(struct seq_file *m, void *data)
{
	struct mipi_dsi_device *dsi = m->private;

	seq_puts(m, dsi->name);
	seq_putc(m, '\n');

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(exynos_dsi_name);

static ssize_t parse_byte_buf(u8 *out, size_t len, char *src)
{
	const char *skip = "\n ";
	size_t i = 0;
	int rc = 0;
	char *s;

	while (src && !rc && i < len) {
		s = strsep(&src, skip);
		if (*s != '\0') {
			rc = kstrtou8(s, 16, out + i);
			i++;
		}
	}

	return rc ? : i;
}


struct exynos_dsi_reg_data {
	struct mipi_dsi_device *dsi;
	u8 address;
	u8 type;
	size_t count;
};

static ssize_t exynos_dsi_payload_write(struct file *file,
			       const char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct exynos_dsi_reg_data *reg_data = m->private;
	char *buf;
	char *payload;
	size_t len;
	int ret;

	buf = memdup_user_nul(user_buf, count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	/* calculate length for worst case (1 digit per byte + whitespace) */
	len = (count + 1) / 2;
	payload = kmalloc(len, GFP_KERNEL);
	if (!payload) {
		kfree(buf);
		return -ENOMEM;
	}

	ret = parse_byte_buf(payload, len, buf);
	if (ret <= 0) {
		ret = -EINVAL;
	} else if (reg_data->type) {
		ret = exynos_dsi_dcs_transfer(reg_data->dsi, reg_data->type,
					    payload, ret);
	} else {
		ret = exynos_dsi_dcs_write_buffer(reg_data->dsi, payload, ret);
	}

	kfree(buf);
	kfree(payload);

	return ret ? : count;
}

static int exynos_dsi_payload_show(struct seq_file *m, void *data)
{
	struct exynos_dsi_reg_data *reg_data = m->private;
	char *buf;
	ssize_t rc;

	if (!reg_data->count)
		return -EINVAL;

	buf = kmalloc(reg_data->count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	rc = mipi_dsi_dcs_read(reg_data->dsi, reg_data->address, buf,
			       reg_data->count);
	if (rc > 0) {
		seq_hex_dump(m, "", DUMP_PREFIX_NONE, 16, 1, buf, rc, false);
		rc = 0;
	} else if (rc == 0) {
		pr_debug("no response back\n");
	}
	kfree(buf);

	return 0;
}

static int exynos_dsi_payload_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_dsi_payload_show, inode->i_private);
}

static const struct file_operations exynos_dsi_payload_fops = {
	.owner		= THIS_MODULE,
	.open		= exynos_dsi_payload_open,
	.write		= exynos_dsi_payload_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int exynos_dsi_debugfs_add(struct mipi_dsi_device *dsi,
			 struct dentry *parent)
{
	struct dentry *reg_root;
	struct exynos_dsi_reg_data *reg_data;

	reg_root = debugfs_create_dir("reg", parent);
	if (!reg_root)
		return -EFAULT;

	reg_data = devm_kzalloc(&dsi->dev, sizeof(*reg_data), GFP_KERNEL);
	if (!reg_data)
		return -ENOMEM;

	reg_data->dsi = dsi;

	debugfs_create_u8("address", 0600, reg_root, &reg_data->address);
	debugfs_create_u8("type", 0600, reg_root, &reg_data->type);
	debugfs_create_size_t("count", 0600, reg_root, &reg_data->count);
	debugfs_create_file("payload", 0600, reg_root, reg_data,
			    &exynos_dsi_payload_fops);

	debugfs_create_file("name", 0600, parent, dsi, &exynos_dsi_name_fops);

	return 0;
}

static int exynos_debugfs_panel_add(struct exynos_panel *ctx, struct dentry *parent)
{
	struct dentry *root;

	if (!parent)
		return -EINVAL;

	root = debugfs_create_dir("panel", parent);
	if (!root)
		return -EPERM;

	ctx->debugfs_entry = root;

	return 0;
}

static void exynos_debugfs_panel_remove(struct exynos_panel *ctx)
{
	if (!ctx->debugfs_entry)
		return;

	debugfs_remove_recursive(ctx->debugfs_entry);

	ctx->debugfs_entry = NULL;
}
#else
static int hbm_mode_debugfs_add(struct exynos_panel *ctx, struct dentry *parent)
{
	return 0;
}

static int exynos_dsi_debugfs_add(struct mipi_dsi_device *dsi,
			 struct dentry *parent)
{
	return 0;
}

static int exynos_debugfs_panel_add(struct exynos_panel *ctx, struct dentry *parent)
{
	return 0;
}

static void exynos_debugfs_panel_remove(struct exynos_panel *ctx)
{
	return;
}
#endif

static int exynos_panel_attach_lp_mode(struct exynos_drm_connector *exynos_conn,
				       const struct drm_display_mode *lp_mode)
{
	struct exynos_drm_connector_properties *p =
		exynos_drm_connector_get_properties(exynos_conn);
	struct drm_mode_modeinfo umode;
	struct drm_property_blob *blob;

	if (!lp_mode)
		return -ENOENT;

	drm_mode_convert_to_umode(&umode, lp_mode);
	blob = drm_property_create_blob(exynos_conn->base.dev, sizeof(umode), &umode);
	if (IS_ERR(blob))
		return PTR_ERR(blob);

	drm_object_attach_property(&exynos_conn->base.base, p->lp_mode, blob->base.id);

	return 0;
}

static ssize_t local_hbm_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	bool local_hbm_en;
	int ret;

	if (!ctx->enabled || !ctx->initialized) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	ret = kstrtobool(buf, &local_hbm_en);
	if (ret) {
		dev_err(ctx->dev, "invalid local_hbm_mode value\n");
		return ret;
	}

	ctx->desc->exynos_panel_func->set_local_hbm_mode(ctx, local_hbm_en);
	if (local_hbm_en) {
		queue_delayed_work(ctx->local_hbm.wq,
			 &ctx->local_hbm.timeout_work,
			 msecs_to_jiffies(ctx->local_hbm.max_timeout_ms));
	} else {
		cancel_delayed_work(&ctx->local_hbm.timeout_work);
	}

	return count;
}

static ssize_t local_hbm_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->local_hbm.enabled);
}
static DEVICE_ATTR_RW(local_hbm_mode);

static ssize_t local_hbm_max_timeout_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	int ret;

	ret = kstrtou32(buf, 0, &ctx->local_hbm.max_timeout_ms);
	if (ret) {
		dev_err(ctx->dev, "invalid local_hbm_max_timeout_ms value\n");
		return ret;
	}

	return count;
}

static ssize_t local_hbm_max_timeout_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->local_hbm.max_timeout_ms);
}

static DEVICE_ATTR_RW(local_hbm_max_timeout);

static ssize_t state_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bl);
	bool show_mode = true;
	const char *statestr;
	int rc;

	mutex_lock(&ctx->bl_state_lock);

	if (is_backlight_off_state(bl)) {
		statestr = "Off";
		show_mode = false;
	} else if (is_backlight_lp_state(bl)) {
		statestr = "LP";
	} else {
		statestr = (ctx->hbm_mode) ? "HBM" : "On";
	}

	mutex_unlock(&ctx->bl_state_lock);

	if (show_mode) {
		const struct exynos_panel_mode *pmode = ctx->current_mode;

		rc = snprintf(buf, PAGE_SIZE, "%s: %dx%d@%d\n", statestr,
			      pmode->mode.hdisplay, pmode->mode.vdisplay,
			      drm_mode_vrefresh(&pmode->mode));
	} else {
		rc = snprintf(buf, PAGE_SIZE, "%s\n", statestr);
	}

	return rc;
}

static DEVICE_ATTR_RO(state);

static struct attribute *bl_device_attrs[] = {
	&dev_attr_local_hbm_mode.attr,
	&dev_attr_local_hbm_max_timeout.attr,
	&dev_attr_state.attr,
	NULL,
};
ATTRIBUTE_GROUPS(bl_device);

static unsigned long get_backlight_state_from_panel(struct backlight_device *bl,
					enum exynos_panel_state panel_state)
{
	unsigned long state = bl->props.state;

	switch (panel_state) {
	case PANEL_STATE_ON:
		state &= ~(BL_STATE_STANDBY | BL_STATE_LP);
		break;
	case PANEL_STATE_LP:
		state |= BL_STATE_LP;
		break;
	case PANEL_STATE_OFF:
		state &= ~(BL_STATE_LP);
		state |= BL_STATE_STANDBY;
		break;
	}

	return state;
}

static void exynos_panel_set_backlight_state(struct exynos_panel *ctx,
					enum exynos_panel_state panel_state)
{
	struct backlight_device *bl = ctx->bl;

	if (!bl)
		return;

	mutex_lock(&ctx->bl_state_lock);

	bl->props.state = get_backlight_state_from_panel(bl, panel_state);

	mutex_unlock(&ctx->bl_state_lock);

	backlight_state_changed(bl);

	dev_info(ctx->dev, "%s: panel:%d, bl:0x%x\n", __func__,
		 panel_state, bl->props.state);
}

static int exynos_panel_attach_properties(struct exynos_panel *ctx)
{
	struct exynos_drm_connector_properties *p =
		exynos_drm_connector_get_properties(&ctx->exynos_connector);
	struct drm_mode_object *obj = &ctx->exynos_connector.base.base;
	const struct exynos_panel_desc *desc = ctx->desc;
	int ret = 0;

	if (!p || !desc)
		return -ENOENT;

	drm_object_attach_property(obj, p->min_luminance, desc->min_luminance);
	drm_object_attach_property(obj, p->max_luminance, desc->max_luminance);
	drm_object_attach_property(obj, p->max_avg_luminance, desc->max_avg_luminance);
	drm_object_attach_property(obj, p->hdr_formats, desc->hdr_formats);

	if (desc->lp_mode) {
		ret = exynos_panel_attach_lp_mode(&ctx->exynos_connector, &desc->lp_mode->mode);
		if (ret)
			dev_err(ctx->dev, "Failed to attach lp mode (%d)\n", ret);
	}

	return ret;
}

static int exynos_panel_bridge_attach(struct drm_bridge *bridge,
				      enum drm_bridge_attach_flags flags)
{
	struct drm_device *dev = bridge->dev;
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct drm_connector *connector = &ctx->exynos_connector.base;
	int ret;

	ret = exynos_drm_connector_init(dev, &ctx->exynos_connector,
					&exynos_panel_connector_funcs,
					DRM_MODE_CONNECTOR_DSI);
	if (ret) {
		dev_err(ctx->dev, "failed to initialize connector with drm\n");
		return ret;
	}

	ret = exynos_panel_attach_properties(ctx);
	if (ret) {
		dev_err(ctx->dev, "failed to attach connector properties\n");
		return ret;
	}

	drm_connector_helper_add(connector, &exynos_connector_helper_funcs);

	drm_connector_register(connector);

	drm_connector_attach_encoder(connector, bridge->encoder);
	connector->funcs->reset(connector);
	connector->status = connector_status_connected;

	ret = sysfs_create_link(&connector->kdev->kobj, &ctx->dev->kobj,
				"panel");
	if (ret)
		dev_warn(ctx->dev, "unable to link panel sysfs (%d)\n", ret);

	ret = drm_panel_attach(&ctx->panel, connector);
	if (ret) {
		dev_err(ctx->dev, "unable to attach drm panel\n");
		return ret;
	}

	exynos_debugfs_panel_add(ctx, connector->debugfs_entry);
	exynos_dsi_debugfs_add(to_mipi_dsi_device(ctx->dev), ctx->debugfs_entry);
	hbm_mode_debugfs_add(ctx, ctx->debugfs_entry);

	drm_kms_helper_hotplug_event(connector->dev);

	if (!ctx->is_secondary)
		ret = sysfs_create_link(&bridge->dev->dev->kobj, &ctx->dev->kobj, "primary-panel");
	else
		ret = sysfs_create_link(&bridge->dev->dev->kobj, &ctx->dev->kobj, "secondary-panel");

	if (ret)
		dev_warn(ctx->dev, "unable to link %s sysfs (%d)\n",
			(!ctx->is_secondary) ? "primary-panel" : "secondary-panel", ret);
	return 0;
}

static void exynos_panel_bridge_detach(struct drm_bridge *bridge)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct drm_connector *connector = &ctx->exynos_connector.base;

	if (!ctx->is_secondary)
		sysfs_remove_link(&bridge->dev->dev->kobj, "primary-panel");
	else
		sysfs_remove_link(&bridge->dev->dev->kobj, "secondary-panel");

	exynos_debugfs_panel_remove(ctx);
	sysfs_remove_link(&connector->kdev->kobj, "panel");
	drm_panel_detach(&ctx->panel);
	drm_connector_unregister(connector);
	drm_connector_cleanup(&ctx->exynos_connector.base);
}

static void exynos_panel_bridge_enable(struct drm_bridge *bridge)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);

	/* this handles the case where panel may be enabled while booting already */
	if (ctx->enabled && !exynos_panel_init(ctx))
		return;

	if (in_tui(ctx)) {
		dev_info(ctx->dev, "tui state : skip %s\n", __func__);
		return;
	}

	drm_panel_enable(&ctx->panel);

	exynos_panel_set_backlight_state(ctx, PANEL_STATE_ON);
}

static void exynos_panel_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);

	if (ctx->enabled)
		return;

	if (in_tui(ctx)) {
		dev_info(ctx->dev, "tui state : skip %s\n", __func__);
		return;
	}

	drm_panel_prepare(&ctx->panel);
}

static void exynos_panel_bridge_disable(struct drm_bridge *bridge)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);

	if (in_tui(ctx)) {
		dev_info(ctx->dev, "tui state : skip %s\n", __func__);
		return;
	}

	drm_panel_disable(&ctx->panel);
}

static void exynos_panel_bridge_post_disable(struct drm_bridge *bridge)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);

	if (in_tui(ctx)) {
		dev_info(ctx->dev, "tui state : skip %s\n", __func__);
		return;
	}

	drm_panel_unprepare(&ctx->panel);

	exynos_panel_set_backlight_state(ctx, PANEL_STATE_OFF);
}

static void exynos_panel_bridge_mode_set(struct drm_bridge *bridge,
				  const struct drm_display_mode *mode,
				  const struct drm_display_mode *adjusted_mode)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	const struct exynos_panel_mode *pmode = exynos_panel_get_mode(ctx, mode);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	bool need_update_backlight = false;

	if (WARN_ON(!pmode))
		return;

	if (!ctx->initialized && ctx->enabled) {
		/* if panel was enabled at boot and there's no mode change skip mode set */
		if (ctx->current_mode == pmode)
			return;

		WARN(1, "mode change at boot to %s\n", adjusted_mode->name);

		/*
		 * This is unexpected, but the best we can do is to set as disable which will
		 * force panel reset on next enable. That way it will go into new mode
		 */
		ctx->enabled = false;
		exynos_panel_set_power(ctx, false);
	}

	dev_dbg(ctx->dev, "changing display mode to %dx%d@%d\n",
		pmode->mode.hdisplay, pmode->mode.vdisplay, drm_mode_vrefresh(&pmode->mode));

	dsi->mode_flags = pmode->exynos_mode.mode_flags;

	if (funcs) {
		const bool was_lp_mode = ctx->current_mode->exynos_mode.is_lp_mode;
		const bool is_lp_mode = pmode->exynos_mode.is_lp_mode;

		if (is_lp_mode && funcs->set_lp_mode) {
			funcs->set_lp_mode(ctx, pmode);
			need_update_backlight = true;
		} else if (was_lp_mode && !is_lp_mode && funcs->set_nolp_mode) {
			funcs->set_nolp_mode(ctx, pmode);
			exynos_panel_set_backlight_state(ctx, PANEL_STATE_ON);
			need_update_backlight = true;
		} else if (funcs->mode_set) {
			funcs->mode_set(ctx, pmode);
			if (ctx->bl)
				backlight_state_changed(ctx->bl);
		}
	}
	ctx->current_mode = pmode;

	if (need_update_backlight && ctx->bl)
		backlight_update_status(ctx->bl);
}

static void local_hbm_timeout_work(struct work_struct *work)
{
	struct exynos_panel *ctx =
			 container_of(work, struct exynos_panel, local_hbm.timeout_work.work);

	dev_dbg(ctx->dev, "%s\n", __func__);

	ctx->desc->exynos_panel_func->set_local_hbm_mode(ctx, false);
}

static void local_hbm_data_init(struct exynos_panel *ctx)
{
	mutex_init(&ctx->local_hbm.lock);
	ctx->local_hbm.max_timeout_ms = LOCAL_HBM_MAX_TIMEOUT_MS;
	ctx->local_hbm.enabled = false;
	ctx->local_hbm.wq =
		create_singlethread_workqueue("local_hbm_workq");
	if (!ctx->local_hbm.wq)
		dev_err(ctx->dev, "failed to create local hbm workq!\n");
	else
		INIT_DELAYED_WORK(&ctx->local_hbm.timeout_work, local_hbm_timeout_work);
}

static const struct drm_bridge_funcs exynos_panel_bridge_funcs = {
	.attach = exynos_panel_bridge_attach,
	.detach = exynos_panel_bridge_detach,
	.pre_enable = exynos_panel_bridge_pre_enable,
	.enable = exynos_panel_bridge_enable,
	.disable = exynos_panel_bridge_disable,
	.post_disable = exynos_panel_bridge_post_disable,
	.mode_set = exynos_panel_bridge_mode_set,
};

int exynos_panel_probe(struct mipi_dsi_device *dsi)
{
	static atomic_t panel_index = ATOMIC_INIT(-1);
	struct device *dev = &dsi->dev;
	struct exynos_panel *ctx;
	int ret = 0;
	char name[32];
	const struct exynos_panel_funcs *exynos_panel_func;

	ctx = devm_kzalloc(dev, sizeof(struct exynos_panel), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	dev_dbg(dev, "%s +\n", __func__);

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;
	ctx->desc = of_device_get_match_data(dev);

	dsi->lanes = ctx->desc->data_lane_cnt;
	dsi->format = MIPI_DSI_FMT_RGB888;

	ret = exynos_panel_parse_dt(ctx);
	if (ret)
		return ret;

	scnprintf(name, sizeof(name), "panel%d-backlight", atomic_inc_return(&panel_index));

	ctx->bl = devm_backlight_device_register(ctx->dev, name, dev,
			ctx, &exynos_backlight_ops, NULL);
	if (IS_ERR(ctx->bl)) {
		dev_err(ctx->dev, "failed to register backlight device\n");
		return PTR_ERR(ctx->bl);
	}
	ctx->bl->props.max_brightness = ctx->desc->max_brightness;
	ctx->bl->props.brightness = ctx->desc->dft_brightness;

	exynos_panel_func = ctx->desc->exynos_panel_func;
	if (exynos_panel_func && exynos_panel_func->set_local_hbm_mode)
		local_hbm_data_init(ctx);

	mutex_init(&ctx->bl_state_lock);

	drm_panel_init(&ctx->panel, dev, ctx->desc->panel_func, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ctx->bridge.funcs = &exynos_panel_bridge_funcs;
#ifdef CONFIG_OF
	ctx->bridge.of_node = ctx->dev->of_node;
#endif
	drm_bridge_add(&ctx->bridge);

	ret = sysfs_create_files(&dev->kobj, panel_attrs);
	if (ret)
		pr_warn("unable to add panel sysfs files (%d)\n", ret);

	ret = sysfs_create_groups(&ctx->bl->dev.kobj, bl_device_groups);
	if (ret)
		dev_err(ctx->dev, "unable to create bl_device_groups groups\n");

	exynos_panel_handoff(ctx);

	ret = mipi_dsi_attach(dsi);
	if (ret)
		goto err_panel;

	dev_info(ctx->dev, "samsung common panel driver has been probed\n");

	return 0;

err_panel:
	drm_panel_detach(&ctx->panel);
	drm_panel_remove(&ctx->panel);
	dev_err(ctx->dev, "failed to probe samsung panel driver(%d)\n", ret);

	return ret;
}
EXPORT_SYMBOL(exynos_panel_probe);

int exynos_panel_remove(struct mipi_dsi_device *dsi)
{
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
	drm_bridge_remove(&ctx->bridge);

	sysfs_remove_groups(&ctx->bl->dev.kobj, bl_device_groups);
	devm_backlight_device_unregister(ctx->dev, ctx->bl);

	return 0;
}
EXPORT_SYMBOL(exynos_panel_remove);

MODULE_AUTHOR("Jiun Yu <jiun.yu@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung common panel driver");
MODULE_LICENSE("GPL");
