/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Google LLC
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 */

#ifndef __EXYNOS_DISPLAY_COMMON_H__
#define __EXYNOS_DISPLAY_COMMON_H__

#include <drm/drm_modes.h>

/* Structures and definitions shared across exynos display pipeline. */

#define EXYNOS_DISPLAY_MODE_FLAG_EXYNOS_PANEL	(1 << 0)
#define EXYNOS_DISPLAY_MODE_FLAG_TUI		(1 << 1)
#define EXYNOS_DISPLAY_MODE_FLAG_SEAMLESS	(1 << 2)
#define EXYNOS_DISPLAY_MODE_FLAG_LP_MODE	(1 << 3)

struct exynos_display_dsc {
	bool enabled;
	unsigned int dsc_count;
	unsigned int slice_count;
	unsigned int slice_height;
};

struct exynos_display_underrun_param {
	/* te idle (us) to calculate underrun_lp_ref */
	unsigned int te_idle_us;
	/* te variation (percentage) to calculate underrun_lp_ref */
	unsigned int te_var;
	/* display mode used to calculate underrun_lp_ref */
	const struct drm_display_mode *ref_mode;
};

struct exynos_display_mode {
	/* DSI mode flags in drm_mipi_dsi.h */
	unsigned long mode_flags;
	/* command: TE pulse time, video: vbp+vfp time */
	unsigned int vblank_usec;
	unsigned int bpc;
	struct exynos_display_dsc dsc;
	/* parameters to calculate underrun_lp_ref when hs_clock changes */
	const struct exynos_display_underrun_param *underrun_param;
};

static inline const struct exynos_display_mode *drm_mode_to_exynos(
					const struct drm_display_mode *mode)
{
	if ((mode &&
		(mode->private_flags & EXYNOS_DISPLAY_MODE_FLAG_EXYNOS_PANEL)))
		return (const struct exynos_display_mode *) mode->private;

	return NULL;
}

static inline bool
exynos_drm_mode_is_seamless(const struct drm_display_mode *mode)
{
	return (mode->private_flags & EXYNOS_DISPLAY_MODE_FLAG_SEAMLESS) != 0;
}
#endif
