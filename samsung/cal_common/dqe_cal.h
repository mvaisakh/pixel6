/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for GS101 DQE CAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DQE_CAL_H__
#define __SAMSUNG_DQE_CAL_H__

#include <linux/types.h>
#include <drm/samsung_drm.h>
#include <drm/drm_mode.h>

#define DEGAMMA_LUT_SIZE		65
#define REGAMMA_LUT_SIZE		65
#define CGC_LUT_SIZE			4913
#define HIST_BIN_SIZE			256
#define GAMMA_MATRIX_COEFFS_CNT		9
#define GAMMA_MATRIX_OFFSETS_CNT	3
#define LINEAR_MATRIX_COEFFS_CNT	9
#define LINEAR_MATRIX_OFFSETS_CNT	3

enum dqe_version {
	DQE_V1,
	DQE_V2,
};

enum dqe_dither_type {
	CGC_DITHER = 0,
	DISP_DITHER = 1,
};

void
dqe_regs_desc_init(void __iomem *regs, const char *name, enum dqe_version ver);
void dqe_reg_init(u32 width, u32 height);
void dqe_reg_set_degamma_lut(const struct drm_color_lut *lut);
void dqe_reg_set_cgc_lut(const struct cgc_lut *lut);
void dqe_reg_set_regamma_lut(const struct drm_color_lut *lut);
void dqe_reg_set_cgc_dither(struct dither_config *config);
void dqe_reg_set_disp_dither(struct dither_config *config);
void dqe_reg_set_linear_matrix(const struct exynos_matrix *lm);
void dqe_reg_set_gamma_matrix(const struct exynos_matrix *matrix);
void dqe_reg_print_dither(enum dqe_dither_type dither);
void dqe_reg_print_degamma_lut(void);
void dqe_reg_print_cgc_lut(u32 count);
void dqe_reg_print_regamma_lut(void);
void dqe_reg_print_hist(void);
void dqe_reg_print_gamma_matrix(void);
void dqe_reg_print_linear_matrix(void);
#endif /* __SAMSUNG_DQE_CAL_H__ */
