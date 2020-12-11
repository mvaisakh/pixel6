/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Headef file for Display Quality Enhancer.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_DRM_DQE_H__
#define __EXYNOS_DRM_DQE_H__

#include <drm/samsung_drm.h>

struct decon_device;
struct exynos_dqe;
struct exynos_dqe_state;

struct exynos_dqe_funcs {
	void (*update)(struct exynos_dqe *dqe, struct exynos_dqe_state *state,
			u32 width, u32 height);
};

struct exynos_dqe_state {
	const struct drm_color_lut *degamma_lut;
	const struct exynos_matrix *linear_matrix;
	const struct exynos_matrix *gamma_matrix;
	const struct cgc_lut *cgc_lut;
	struct drm_color_lut *regamma_lut;
	struct dither_config *disp_dither_config;
	struct dither_config *cgc_dither_config;
};

struct dither_debug_override {
	bool force_en;
	bool verbose;
	struct dither_config val;
};

enum elem_size {
	ELEM_SIZE_16 = 16,
	ELEM_SIZE_32 = 32,
};

#define MAX_NAME_SIZE		32
struct debugfs_lut {
	void *lut_ptr;
	struct drm_color_lut *dlut_ptr;
	char name[MAX_NAME_SIZE];
	enum elem_size elem_size;
	size_t count;
	size_t pcount;
};

struct exynos_dqe {
	void __iomem *regs;
	bool initialized;
	const struct exynos_dqe_funcs *funcs;
	struct exynos_dqe_state state;
	struct decon_device *decon;

	struct dither_debug_override cgc_dither_override;
	struct dither_debug_override disp_dither_override;

	bool cgc_first_write;

	bool force_lm;
	struct exynos_matrix force_linear_matrix;

	bool force_gm;
	struct exynos_matrix force_gamma_matrix;
};

void exynos_dqe_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state,
		u32 width, u32 height);
void exynos_dqe_reset(struct exynos_dqe *dqe);
struct exynos_dqe *exynos_dqe_register(struct decon_device *decon);

#endif /* __EXYNOS_DRM_DQE_H__ */
