// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#define pr_fmt(fmt)  "[PARTIAL]: " fmt

#include <linux/device.h>
#include <linux/of.h>
#include <video/mipi_display.h>
#include <drm/drm_fourcc.h>
#include "exynos_drm_decon.h"
#include "exynos_drm_format.h"
#include "exynos_drm_dsim.h"
#include "cal_common/dsim_cal.h"

#define MIN_WIN_BLOCK_WIDTH	8
#define MIN_WIN_BLOCK_HEIGHT	1

static int exynos_partial_init(struct exynos_partial *partial)
{
	struct decon_device *decon = partial->decon;
	struct decon_config *cfg;

	if (!decon)
		return -EFAULT;

	cfg = &decon->config;

	if (cfg->dsc.enabled) {
		partial->min_w = cfg->dsc.slice_width;
		partial->min_h = cfg->dsc.slice_height;
	} else {
		partial->min_w = MIN_WIN_BLOCK_WIDTH;
		partial->min_h = MIN_WIN_BLOCK_HEIGHT;
	}

	partial->prev_partial_region.x1 = 0;
	partial->prev_partial_region.y1 = 0;
	partial->prev_partial_region.x2 = cfg->image_width;
	partial->prev_partial_region.y2 = cfg->image_height;

	if ((cfg->image_width % partial->min_w) ||
			(cfg->image_height % partial->min_h)) {
		pr_err("cannot support partial update(%dx%d, %dx%d)\n",
				cfg->image_width, cfg->image_height,
				partial->min_w, partial->min_h);
		return -EINVAL;
	}

	return 0;
}

static void
exynos_partial_set_full(struct decon_device *decon, struct drm_rect *partial_r)
{
	partial_r->x1 = 0;
	partial_r->y1 = 0;
	partial_r->x2 = decon->config.image_width;
	partial_r->y2 = decon->config.image_height;
	pr_debug("changed full\n");
}

static struct drm_rect *
exynos_partial_adjust_region(struct exynos_partial *partial,
			struct exynos_drm_crtc_state *exynos_crtc_state,
			struct drm_rect *req)
{
	struct decon_device *decon = partial->decon;
	struct drm_rect *r = &exynos_crtc_state->partial_region;

	pr_debug("requested update region[%d %d %d %d]\n",
			req->x1, req->y1, req->x2 - req->x1, req->y2 - req->y1);

	if (!req->x1 && !req->y1 && !req->x2 && !req->y2) {
		pr_err("invalid partial update region[%d %d %d %d]\n",
				req->x1, req->y1, req->x2 - req->x1,
				req->y2 - req->y1);
		return NULL;
	}

	if ((req->x2 > decon->config.image_width) ||
			(req->y2 > decon->config.image_height)) {
		exynos_partial_set_full(decon, r);
		goto end;
	}

	/* adjusted update region */
	r->y1 = rounddown(req->y1, partial->min_h);
	r->y2 = roundup(req->y2, partial->min_h);
	/*
	 * TODO: Currently, partial width is fixed by LCD width. This will be
	 * changed to be configurable in the future.
	 */
	r->x1 = 0;
	r->x2 = decon->config.image_width;

	pr_debug("adjusted update region[%d %d %d %d]\n", r->x1, r->y1,
			drm_rect_width(r), drm_rect_height(r));

end:
	return r;
}

static void exynos_plane_print_info(const struct drm_plane_state *state)
{
	const struct drm_plane *plane = state->plane;
	const struct drm_rect src  = drm_plane_state_src(state);
	const struct drm_rect dest = drm_plane_state_dest(state);

	pr_debug("Plane%d src=" DRM_RECT_FP_FMT " crtc=" DRM_RECT_FMT "\n",
			drm_plane_index(plane),
			DRM_RECT_FP_ARG(&src), DRM_RECT_ARG(&dest));
}

static inline bool
exynos_plane_state_rotation(const struct drm_plane_state *state)
{
	unsigned int simplified_rot;

	simplified_rot = drm_rotation_simplify(state->rotation,
			DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_90 |
			DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y);

	return (simplified_rot & DRM_MODE_ROTATE_90) != 0;
}

static inline bool
exynos_plane_state_scaling(const struct drm_plane_state *state)
{
	return (state->src_w >> 16 != state->crtc_w) ||
		(state->src_h >> 16 != state->crtc_h);
}

static bool is_partial_supported(const struct drm_plane_state *state,
		const struct drm_rect *crtc_r, const struct drm_rect *partial_r,
		const struct dpp_restriction *res)
{
	const struct dpu_fmt *fmt_info;
	unsigned int adj_src_x = 0, adj_src_y = 0;
	u32 format;
	int sz_align = 1;

	exynos_plane_print_info(state);

	if (exynos_plane_state_rotation(state)) {
		pr_debug("rotation is detected. partial->full\n");
		return false;
	}

	if (exynos_plane_state_scaling(state)) {
		pr_debug("scaling is detected. partial->full\n");
		return false;
	}

	format = state->fb->format->format;
	fmt_info = dpu_find_fmt_info(format);
	if (IS_YUV(fmt_info)) {
		adj_src_x = state->src_x >> 16;
		adj_src_y = state->src_y >> 16;
		sz_align = 2;

		if (partial_r->x1 > state->crtc_x)
			adj_src_x += partial_r->x1 - state->crtc_x;

		if (partial_r->y1 > state->crtc_y)
			adj_src_y += partial_r->y1 - state->crtc_y;

		/* YUV format must be aligned to 2 */
		if (!IS_ALIGNED(adj_src_x, sz_align) ||
				!IS_ALIGNED(adj_src_y, sz_align)) {
			pr_debug("align limitation. src_x/y[%d/%d] align[%d]\n",
					adj_src_x, adj_src_y, sz_align);
			return false;
		}
	}

	if ((drm_rect_width(crtc_r) < res->src_f_w.min * sz_align) ||
			(drm_rect_height(crtc_r) < res->src_f_h.min * sz_align)) {
		pr_debug("min size limitation. width[%d] height[%d]\n",
				drm_rect_width(crtc_r), drm_rect_height(crtc_r));
		return false;
	}

	return true;
}

#define to_dpp_device(x)	container_of(x, struct dpp_device, plane)
static void exynos_partial_check(struct exynos_partial *partial,
			struct exynos_drm_crtc_state *exynos_crtc_state)
{
	struct drm_crtc_state *crtc_state = &exynos_crtc_state->base;
	struct drm_crtc *crtc = crtc_state->crtc;
	struct decon_device *decon = partial->decon;
	struct drm_plane *plane;
	struct drm_rect *partial_r = &exynos_crtc_state->partial_region;
	struct drm_rect r;
	struct dpp_device *dpp;
	const struct dpp_restriction *res;

	drm_for_each_plane_mask(plane, crtc->dev, crtc_state->plane_mask) {
		r = drm_plane_state_dest(plane->state);

		if (!drm_rect_intersect(&r, partial_r))
			continue;

		dpp = to_dpp_device(to_exynos_plane(plane));
		res = &dpp->restriction;
		pr_debug("DPP%d, ATTR(0x%lx)\n", dpp->id, dpp->attr);

		if (!is_partial_supported(plane->state, &r, partial_r, res)) {
			exynos_partial_set_full(decon,
					&exynos_crtc_state->partial_region);
			return;
		}
	}
}

static void exynos_partial_reconfig_coords(struct exynos_partial *partial,
			struct exynos_drm_crtc_state *exynos_crtc_state)
{
	struct drm_crtc_state *crtc_state = &exynos_crtc_state->base;
	struct drm_crtc *crtc = crtc_state->crtc;
	struct drm_plane *plane;
	struct drm_rect *partial_r = &exynos_crtc_state->partial_region;
	struct drm_rect origin_src, origin_dst;

	drm_for_each_plane_mask(plane, crtc->dev, crtc_state->plane_mask) {
		origin_src = drm_plane_state_src(plane->state);
		origin_dst = drm_plane_state_dest(plane->state);

		pr_debug("original coordinates:\n");
		if (!drm_rect_intersect(&origin_dst, partial_r)) {
			plane->state->visible = false;
			continue;
		}
		origin_dst = drm_plane_state_dest(plane->state);

		exynos_plane_print_info(plane->state);

		/* reconfigure destination coordinates */
		if (partial_r->x1 > origin_dst.x1)
			plane->state->crtc_w = min(drm_rect_width(partial_r),
					origin_dst.x2 - partial_r->x1);
		else if (partial_r->x2 < origin_dst.x2)
			plane->state->crtc_w = min(drm_rect_width(&origin_dst),
					partial_r->x2 - origin_dst.x1);

		if (partial_r->y1 > origin_dst.y1)
			plane->state->crtc_h = min(drm_rect_height(partial_r),
					origin_dst.y2 - partial_r->y1);
		else if (partial_r->y2 < origin_dst.y2)
			plane->state->crtc_h = min(drm_rect_height(&origin_dst),
					partial_r->y2 - origin_dst.y1);

		plane->state->crtc_x = max(origin_dst.x1 - partial_r->x1, 0);
		plane->state->crtc_y = max(origin_dst.y1 - partial_r->y1, 0);

		/* reconfigure source coordinates */
		if (partial_r->y1 > origin_dst.y1)
			plane->state->src_y +=
				((partial_r->y1 - origin_dst.y1) << 16);
		if (partial_r->x1 > origin_dst.x1)
			plane->state->src_x +=
				((partial_r->x1 - origin_dst.x1) << 16);
		plane->state->src_w = plane->state->crtc_w << 16;
		plane->state->src_h = plane->state->crtc_h << 16;

		pr_debug("reconfigured coordinates:\n");
		exynos_plane_print_info(plane->state);
	}
}

static int exynos_partial_send_command(struct exynos_partial *partial,
					const struct drm_rect *partial_r)
{
	struct decon_device *decon = partial->decon;
	struct dsim_device *dsim;
	int ret;

	pr_debug("partial command: [%d %d %d %d]\n",
			partial_r->x1, partial_r->y1,
			drm_rect_width(partial_r), drm_rect_height(partial_r));

	if (!decon)
		return -ENODEV;

	dsim = decon_get_dsim(decon);
	if (!dsim)
		return -ENODEV;

	ret = mipi_dsi_dcs_set_column_address(dsim->dsi_device, partial_r->x1,
			partial_r->x2 - 1);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_set_page_address(dsim->dsi_device, partial_r->y1,
			partial_r->y2 - 1);
	if (ret)
		return ret;

	return ret;
}

static void exynos_partial_find_included_slice(struct exynos_dsc *dsc,
				struct drm_rect *rect, bool in_slice[])
{
	int slice_left, slice_right;
	int i;

	for (i = 0; i < dsc->slice_count; ++i) {
		slice_left = dsc->slice_width * i;
		slice_right = slice_left + dsc->slice_width;
		in_slice[i] = (slice_left >= rect->x1) && (slice_right <= rect->x2);

		pr_debug("slice left(%d) right(%d)\n", slice_left, slice_right);
		pr_debug("slice[%d] is %s\n", i, in_slice[i] ? "in" : "out");
	}
}

#define MAX_DSC_SLICE_CNT	4
static void exynos_partial_set_size(struct exynos_partial *partial,
					struct drm_rect *partial_r)
{
	struct decon_device *decon = partial->decon;
	struct dsim_device *dsim;
	struct dsim_reg_config dsim_config;
	bool in_slice[MAX_DSC_SLICE_CNT];
	bool dsc_en;
	u32 partial_w, partial_h;

	if (!decon)
		return;

	dsim = decon_get_dsim(decon);
	if (!dsim)
		return;

	dsc_en = dsim->config.dsc.enabled;
	partial_w = drm_rect_width(partial_r);
	partial_h = drm_rect_height(partial_r);

	memcpy(&dsim_config, &dsim->config, sizeof(struct dsim_reg_config));
	dsim_config.p_timing.hactive = partial_w;
	dsim_config.p_timing.vactive = partial_h;
	dsim_config.p_timing.hfp +=
		((dsim->config.p_timing.hactive - partial_w) / (dsc_en ? 3 : 1));
	dsim_config.p_timing.vfp += (dsim->config.p_timing.vactive - partial_h);
	dsim_reg_set_partial_update(dsim->id, &dsim_config);

	exynos_partial_find_included_slice(&decon->config.dsc, partial_r,
				in_slice);
	decon_reg_set_partial_update(decon->id, &decon->config, in_slice,
			partial_w, partial_h);

	if (decon->dqe) {
		dqe_reg_set_size(partial_w, partial_h);
		decon_reg_update_req_dqe(decon->id);
	}

	pr_debug("partial[%dx%d] vporch[%d %d %d] hporch[%d %d %d]\n",
			partial_w, partial_h,
			dsim_config.p_timing.vbp, dsim_config.p_timing.vfp,
			dsim_config.p_timing.vsa, dsim_config.p_timing.hbp,
			dsim_config.p_timing.hfp, dsim_config.p_timing.hsa);
}

static const struct exynos_partial_funcs partial_funcs = {
	.init			 = exynos_partial_init,
	.check			 = exynos_partial_check,
	.adjust_partial_region	 = exynos_partial_adjust_region,
	.reconfig_coords	 = exynos_partial_reconfig_coords,
	.send_partial_command	 = exynos_partial_send_command,
	.set_partial_size	 = exynos_partial_set_size,
};

void exynos_partial_initialize(struct exynos_partial *partial)
{
	int ret;

	ret = partial->funcs->init(partial);
	if (ret) {
		partial->decon = NULL;
		pr_err("failed to initialize partial update\n");
	}

	pr_debug("INIT: min rect(%dx%d)\n",
			partial->min_w, partial->min_h);
	DPU_EVENT_LOG(DPU_EVT_PARTIAL_INIT, partial->decon->id, partial);
}

static void
exynos_partial_save_log(struct dpu_log_partial *plog, struct drm_rect *prev,
				struct drm_rect *req, struct drm_rect *adj,
				bool need_partial_update, bool reconfigure)
{
	memcpy(&plog->prev, prev, sizeof(struct drm_rect));
	memcpy(&plog->req, req, sizeof(struct drm_rect));
	memcpy(&plog->adj, adj, sizeof(struct drm_rect));

	plog->need_partial_update = need_partial_update;
	plog->reconfigure = reconfigure;
}

static bool
exynos_partial_is_full(struct decon_device *decon, const struct drm_rect *rect)
{
	struct drm_rect full;

	exynos_partial_set_full(decon, &full);

	return drm_rect_equals(&full, rect);
}

void exynos_partial_prepare(struct exynos_partial *partial,
			struct exynos_drm_crtc_state *exynos_crtc_state)
{
	struct drm_crtc_state *crtc_state = &exynos_crtc_state->base;
	struct drm_rect *partial_r;
	struct decon_device *decon = partial->decon;
	struct dpu_log_partial plog;
	struct drm_clip_rect *req_region;
	struct drm_rect req;
	bool reconfigure = false;

	pr_debug("PREPARE: plane mask[0x%x]\n", crtc_state->plane_mask);

	exynos_crtc_state->need_partial_update = false;

	if (!crtc_state->plane_mask)
		return;

	req_region = exynos_crtc_state->partial->data;
	req.x1 = req_region->x1;
	req.y1 = req_region->y1;
	req.x2 = req_region->x2;
	req.y2 = req_region->y2;

	/* find adjusted update region on LCD */
	partial_r = partial->funcs->adjust_partial_region(partial,
			exynos_crtc_state, &req);
	if (!partial_r)
		return;

	/* check DPP hw limit if violated, update region is changed to full */
	partial->funcs->check(partial, exynos_crtc_state);

	pr_debug("PREPARE: final update region[%d %d %d %d]\n",
			partial_r->x1, partial_r->y1,
			drm_rect_width(partial_r), drm_rect_height(partial_r));

	/*
	 * If update region is changed, need_partial_update flag is set.
	 * That means hw configuration is required.
	 */
	if (!drm_rect_equals(&partial->prev_partial_region, partial_r))
		exynos_crtc_state->need_partial_update = true;

	/*
	 * If partial update region is requested, source and destination
	 * coordinates are needed to change if overlapped with update region.
	 */
	reconfigure = !exynos_partial_is_full(decon, partial_r);

	pr_debug("PREPARE: need_partial_update(%d), reconfigure(%d)\n",
			exynos_crtc_state->need_partial_update, reconfigure);

	exynos_partial_save_log(&plog, &partial->prev_partial_region, &req,
			partial_r, exynos_crtc_state->need_partial_update,
			reconfigure);
	DPU_EVENT_LOG(DPU_EVT_PARTIAL_PREPARE, decon->id, &plog);

	/* Reconfigure source and destination coordinates, if needed. */
	if (reconfigure)
		partial->funcs->reconfig_coords(partial, exynos_crtc_state);
}

void exynos_partial_update(struct exynos_partial *partial,
			struct exynos_drm_crtc_state *exynos_crtc_state)
{
	struct decon_device *decon = partial->decon;
	struct drm_rect *partial_r;

	if (!decon)
		return;

	if (exynos_crtc_state->need_partial_update) {
		partial_r = &exynos_crtc_state->partial_region;
		partial->funcs->send_partial_command(partial, partial_r);
		partial->funcs->set_partial_size(partial, partial_r);
		memcpy(&partial->prev_partial_region, partial_r,
				sizeof(*partial_r));
		DPU_EVENT_LOG(DPU_EVT_PARTIAL_UPDATE, decon->id, NULL);
		pr_debug("UPDATE: [%d %d %d %d]\n",
				partial_r->x1, partial_r->y1,
				drm_rect_width(partial_r),
				drm_rect_height(partial_r));
	}
}

void exynos_partial_restore(struct exynos_partial *partial)
{
	struct decon_device *decon = partial->decon;

	if (!decon)
		return;

	if (exynos_partial_is_full(decon, &partial->prev_partial_region))
		return;

	partial->funcs->set_partial_size(partial,
			&partial->prev_partial_region);
	DPU_EVENT_LOG(DPU_EVT_PARTIAL_RESTORE, decon->id, partial);
	pr_debug("RESTORE: [%d %d %d %d]\n",
			partial->prev_partial_region.x1,
			partial->prev_partial_region.y1,
			drm_rect_width(&partial->prev_partial_region),
			drm_rect_height(&partial->prev_partial_region));
}

struct exynos_partial *exynos_partial_register(struct decon_device *decon)
{
	struct device_node *np;
	struct device *dev = decon->dev;
	struct exynos_partial *partial;

	np = dev->of_node;
	if (!of_property_read_bool(np, "partial-update")) {
		pr_debug("partial update feature is not supported\n");
		return NULL;
	}

	partial = devm_kzalloc(dev, sizeof(struct exynos_partial), GFP_KERNEL);
	if (!partial)
		return NULL;

	partial->decon = decon;
	partial->funcs = &partial_funcs;

	pr_debug("partial update feature is supported\n");

	return partial;
}
