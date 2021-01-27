/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Headef file for LCD partial update.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_DRM_PARTIAL_H__
#define __EXYNOS_DRM_PARTIAL_H__

#include <drm/drm_rect.h>

struct decon_device;
struct exynos_partial;

struct exynos_partial_funcs {
	int (*init)(struct exynos_partial *partial,
			const struct exynos_display_partial *partial_mode);
	struct drm_rect *(*adjust_partial_region)(struct exynos_partial *partial,
			struct exynos_drm_crtc_state *exynos_crtc_state,
			struct drm_rect *req);
	void (*check)(struct exynos_partial *partial,
			struct exynos_drm_crtc_state *exynos_crtc_state);
	void (*reconfig_coords)(struct exynos_partial *partial,
			struct exynos_drm_crtc_state *exynos_crtc_state);
	int (*send_partial_command)(struct exynos_partial *partial,
			const struct drm_rect *partial_r);
	void (*set_partial_size)(struct exynos_partial *partial,
			struct drm_rect *partial_r);
};

struct exynos_partial {
	u32 min_w;
	u32 min_h;
	struct decon_device *decon;
	struct drm_rect prev_partial_region;
	const struct exynos_partial_funcs *funcs;
};

struct exynos_partial *exynos_partial_initialize(struct decon_device *decon,
			const struct exynos_display_partial *partial_mode);
void exynos_partial_prepare(struct exynos_partial *partial,
			struct exynos_drm_crtc_state *exynos_crtc_state);
void exynos_partial_update(struct exynos_partial *partial,
			struct exynos_drm_crtc_state *exynos_crtc_state);
void exynos_partial_restore(struct exynos_partial *partial);

#endif /* __EXYNOS_DRM_PARTIAL_H__ */
