// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Register access functions for Samsung Display Quality Enhancer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <cal_config.h>
#include <dqe_cal.h>
#include <decon_cal.h>
#include <drm/samsung_drm.h>

#include "regs-dqe.h"

#define DISP_DITHER_OFFSET_B0	0x400

struct cal_regs_dqe {
	struct cal_regs_desc desc;
	enum dqe_version version;
};

static struct cal_regs_dqe regs_dqe;

#define dqe_read(offset)	cal_read((&regs_dqe.desc), offset)
#define dqe_write(offset, val)	cal_write((&regs_dqe.desc), offset, val)
#define dqe_read_mask(offset, mask)		\
		cal_read_mask((&regs_dqe.desc), offset, mask)
#define dqe_write_mask(offset, val, mask)	\
		cal_write_mask((&regs_dqe.desc), offset, val, mask)
#define dqe_write_relaxed(offset, val)		\
		cal_write_relaxed((&regs_dqe.desc), offset, val)

#define dither_offset(ver)	(ver > DQE_V1 ? DISP_DITHER_OFFSET_B0 : 0)
#define dither_read(offset)		\
	dqe_read(offset + dither_offset(regs_dqe.version))
#define dither_write(offset, val)	\
	dqe_write(offset + dither_offset(regs_dqe.version), val)

void
dqe_regs_desc_init(void __iomem *regs, const char *name, enum dqe_version ver)
{
	regs_dqe.version = ver;
	regs_dqe.desc.regs = regs;
	regs_dqe.desc.name = name;
}

static void dqe_reg_set_img_size(u32 width, u32 height)
{
	u32 val;

	val = DQE_IMG_VSIZE(height) | DQE_IMG_HSIZE(width);
	dqe_write(DQE0_TOP_IMG_SIZE, val);
}

static void dqe_reg_set_full_img_size(u32 width, u32 height)
{
	u32 val;

	val = DQE_FULL_IMG_VSIZE(height) | DQE_FULL_IMG_HSIZE(width);
	dqe_write(DQE0_TOP_FRM_SIZE, val);

	val = DQE_FULL_PXL_NUM(width * height);
	dqe_write(DQE0_TOP_FRM_PXL_NUM, val);
}

/* exposed to driver layer for DQE CAL APIs */
void dqe_reg_init(u32 width, u32 height)
{
	cal_log_debug(0, "%s +\n", __func__);
	decon_reg_set_dqe_enable(0, true);
	dqe_reg_set_img_size(width, height);
	dqe_reg_set_full_img_size(width, height);
	cal_log_debug(0, "%s -\n", __func__);
}

void dqe_reg_set_degamma_lut(const struct drm_color_lut *lut)
{
	int i;
	u32 val;

	cal_log_debug(0, "%s +\n", __func__);

	if (!lut) {
		dqe_write(DQE0_DEGAMMA_CON, 0);
		return;
	}

	dqe_write(DQE0_DEGAMMA_CON, DEGAMMA_EN);
	for (i = 0; i < DIV_ROUND_UP(DEGAMMA_LUT_SIZE, 2); ++i) {
		val = DEGAMMA_LUT_H(lut[i * 2 + 1].red) |
			DEGAMMA_LUT_L(lut[i * 2].red);
		dqe_write(DQE0_DEGAMMALUT(i), val);

		cal_log_debug(0, "[%d] 0x%x\n", i, val);
	}

	cal_log_debug(0, "%s -\n", __func__);
}

void dqe_reg_set_cgc_lut(const struct cgc_lut *lut)
{
	int i;

	cal_log_debug(0, "%s +\n", __func__);

	if (!lut) {
		dqe_write_mask(DQE0_CGC_CON, 0, CGC_EN);
		return;
	}

	for (i = 0; i < DRM_SAMSUNG_CGC_LUT_REG_CNT; ++i) {
		dqe_write_relaxed(DQE0_CGC_LUT_R(i), lut->r_values[i]);
		dqe_write_relaxed(DQE0_CGC_LUT_G(i), lut->g_values[i]);
		dqe_write_relaxed(DQE0_CGC_LUT_B(i), lut->b_values[i]);
	}

	dqe_write_mask(DQE0_CGC_CON, ~0, CGC_EN);

	cal_log_debug(0, "%s -\n", __func__);
}

void dqe_reg_set_regamma_lut(const struct drm_color_lut *lut)
{
	int i;
	u32 val;

	cal_log_debug(0, "%s +\n", __func__);

	if (!lut) {
		dqe_write(DQE0_REGAMMA_CON, 0);
		return;
	}

	dqe_write(DQE0_REGAMMA_CON, REGAMMA_EN);
	for (i = 0; i < DIV_ROUND_UP(REGAMMA_LUT_SIZE, 2); ++i) {
		val = REGAMMA_LUT_H(lut[i * 2 + 1].red) |
				REGAMMA_LUT_L(lut[i * 2].red);
		dqe_write(DQE0_REGAMMALUT_R(i), val);
		cal_log_debug(0, "[%d]   red: 0x%x\n", i, val);

		val = REGAMMA_LUT_H(lut[i * 2 + 1].green) |
				REGAMMA_LUT_L(lut[i * 2].green);
		dqe_write(DQE0_REGAMMALUT_G(i), val);
		cal_log_debug(0, "[%d] green: 0x%x\n", i, val);

		val = REGAMMA_LUT_H(lut[i * 2 + 1].blue) |
				REGAMMA_LUT_L(lut[i * 2].blue);
		dqe_write(DQE0_REGAMMALUT_B(i), val);
		cal_log_debug(0, "[%d]  blue: 0x%x\n", i, val);
	}

	cal_log_debug(0, "%s -\n", __func__);
}

void dqe_reg_set_cgc_dither(struct dither_config *config)
{
	u32 value = config ? cpu_to_le32(*(u32 *)config) : 0;

	dither_write(DQE0_CGC_DITHER, value);
}

void dqe_reg_set_disp_dither(struct dither_config *config)
{
	u32 value = config ? cpu_to_le32(*(u32 *)config) : 0;

	dither_write(DQE0_DISP_DITHER, value);
}

void dqe_reg_print_dither(enum dqe_dither_type dither)
{
	u32 val;
	const char * const dither_name[] = {
		[CGC_DITHER] = "CGC",
		[DISP_DITHER] = "DISP"
	};

	if (dither == CGC_DITHER)
		val = dither_read(DQE0_CGC_DITHER);
	else if (dither == DISP_DITHER)
		val = dither_read(DQE0_DISP_DITHER);
	else
		return;

	cal_log_info(0, "DQE: %s dither %s\n", dither_name[dither],
		(val & DITHER_EN_MASK) ? "on" : "off");
	cal_log_info(0, "%s mode, frame control %s, frame offset: %d\n",
		(val & DITHER_MODE) ? "Shift" : "Dither",
		(val & DITHER_FRAME_CON) ? "on" : "off",
		(val & DITHER_FRAME_OFFSET_MASK) >> DITHER_FRAME_OFFSET_SHIFT);
	cal_log_info(0, "Table red(%c) green(%c) blue(%c)\n",
		(val & DITHER_TABLE_SEL_R) ? 'B' : 'A',
		(val & DITHER_TABLE_SEL_G) ? 'B' : 'A',
		(val & DITHER_TABLE_SEL_B) ? 'B' : 'A');
}

void dqe_reg_set_linear_matrix(const struct exynos_matrix *lm)
{
	int i, reg_cnt;
	u32 val;

	cal_log_debug(0, "%s +\n", __func__);

	if (!lm) {
		dqe_write(DQE0_LINEAR_MATRIX_CON, 0);
		return;
	}

	reg_cnt = DIV_ROUND_UP(LINEAR_MATRIX_COEFFS_CNT , 2);
	for (i = 0; i < reg_cnt; ++i) {
		if (i == reg_cnt - 1)
			val = LINEAR_MATRIX_COEFF_L(lm->coeffs[i * 2]);
		else
			val = LINEAR_MATRIX_COEFF_H(lm->coeffs[i * 2 + 1]) |
				LINEAR_MATRIX_COEFF_L(lm->coeffs[i * 2]);
		dqe_write(DQE0_LINEAR_MATRIX_COEFF(i), val);
	}

	dqe_write(DQE0_LINEAR_MATRIX_OFFSET0,
			LINEAR_MATRIX_OFFSET_1(lm->offsets[1]) |
			LINEAR_MATRIX_OFFSET_0(lm->offsets[0]));
	dqe_write(DQE0_LINEAR_MATRIX_OFFSET1,
			LINEAR_MATRIX_OFFSET_2(lm->offsets[2]));

	dqe_write(DQE0_LINEAR_MATRIX_CON, LINEAR_MATRIX_EN);

	cal_log_debug(0, "%s -\n", __func__);
}

void dqe_reg_set_gamma_matrix(const struct exynos_matrix *matrix)
{
	int i, reg_cnt;
	u32 val;

	cal_log_debug(0, "%s +\n", __func__);

	if (!matrix) {
		dqe_write(DQE0_GAMMA_MATRIX_CON, 0);
		return;
	}

	reg_cnt = DIV_ROUND_UP(GAMMA_MATRIX_COEFFS_CNT , 2);
	for (i = 0; i < reg_cnt; ++i) {
		if (i == reg_cnt - 1)
			val = GAMMA_MATRIX_COEFF_L(matrix->coeffs[i * 2]);
		else
			val = GAMMA_MATRIX_COEFF_H(matrix->coeffs[i * 2 + 1]) |
				GAMMA_MATRIX_COEFF_L(matrix->coeffs[i * 2]);
		dqe_write(DQE0_GAMMA_MATRIX_COEFF(i), val);
	}

	dqe_write(DQE0_GAMMA_MATRIX_OFFSET0,
			GAMMA_MATRIX_OFFSET_1(matrix->offsets[1]) |
			GAMMA_MATRIX_OFFSET_0(matrix->offsets[0]));
	dqe_write(DQE0_GAMMA_MATRIX_OFFSET1,
			GAMMA_MATRIX_OFFSET_2(matrix->offsets[2]));

	dqe_write(DQE0_GAMMA_MATRIX_CON, GAMMA_MATRIX_EN);

	cal_log_debug(0, "%s -\n", __func__);
}
