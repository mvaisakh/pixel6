/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _EXYNOS_DRM_CONNECTOR_H_
#define _EXYNOS_DRM_CONNECTOR_H_

#include <drm/drm_connector.h>

struct exynos_drm_connector;

struct exynos_drm_connector_properties {
	struct drm_property *max_luminance;
	struct drm_property *max_avg_luminance;
	struct drm_property *min_luminance;
	struct drm_property *hdr_formats;
	struct drm_property *lp_mode;
};

struct exynos_drm_connector_state {
	struct drm_connector_state base;
};

#define to_exynos_connector_state(connector_state) \
	container_of((connector_state), struct exynos_drm_connector_state, base)

struct exynos_drm_connector_funcs {
	void (*atomic_print_state)(struct drm_printer *p,
				   const struct exynos_drm_connector_state *state);
	int (*atomic_set_property)(struct exynos_drm_connector *exynos_connector,
				   struct exynos_drm_connector_state *exynos_state,
				   struct drm_property *property,
				   uint64_t val);
	int (*atomic_get_property)(struct exynos_drm_connector *exynos_connector,
				   const struct exynos_drm_connector_state *exynos_state,
				   struct drm_property *property,
				   uint64_t *val);
};

struct exynos_drm_connector {
	struct drm_connector base;
	const struct exynos_drm_connector_funcs *funcs;
};

#define to_exynos_connector(connector) \
	container_of((connector), struct exynos_drm_connector, base)

bool is_exynos_drm_connector(const struct drm_connector *connector);
int exynos_drm_connector_init(struct drm_device *dev,
			      struct exynos_drm_connector *exynos_connector,
			      const struct exynos_drm_connector_funcs *funcs,
			      int connector_type);
int exynos_drm_connector_create_properties(struct drm_device *dev);
struct exynos_drm_connector_properties *
exynos_drm_connector_get_properties(struct exynos_drm_connector *exynos_conector);

#endif /* _EXYNOS_DRM_CONNECTOR_H_ */
