/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google Battery Management System
 *
 * Copyright 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __GBMS_POWER_SUPPLY_H__
#define __GBMS_POWER_SUPPLY_H__

#include <linux/power_supply.h>

enum {
	POWER_SUPPLY_TAPER_CONTROL_OFF = 0,
	POWER_SUPPLY_TAPER_CONTROL_MODE_STEPPER = 1,
	POWER_SUPPLY_TAPER_CONTROL_MODE_IMMEDIATE = 2,
};

/* Indicates USB Type-C CC connection status */
enum power_supply_typec_mode {
	POWER_SUPPLY_TYPEC_NONE,

	/* Acting as source */
	POWER_SUPPLY_TYPEC_SINK,		/* Rd only */
	POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE,	/* Rd/Ra */
	POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY,/* Rd/Rd */
	POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER,	/* Ra/Ra */
	POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY,	/* Ra only */

	/* Acting as sink */
	POWER_SUPPLY_TYPEC_SOURCE_DEFAULT,	/* Rp default */
	POWER_SUPPLY_TYPEC_SOURCE_MEDIUM,	/* Rp 1.5A */
	POWER_SUPPLY_TYPEC_SOURCE_HIGH,		/* Rp 3A */
	POWER_SUPPLY_TYPEC_DAM_DEFAULT,		/* Rp-1.5A/Rp-3A */
	POWER_SUPPLY_TYPEC_DAM_MEDIUM,		/* Rp-Default/Rp-1.5A */
	POWER_SUPPLY_TYPEC_DAM_HIGH,		/* Rp-Default/Rp-3A */

	/* Non Compliant */
	POWER_SUPPLY_TYPEC_NON_COMPLIANT,
	POWER_SUPPLY_TYPEC_RP_STD_STD,		/* Rp-Default/Rp-Default */
	POWER_SUPPLY_TYPEC_RP_MED_MED,		/* Rp-1.5A/Rp-1.5A */
	POWER_SUPPLY_TYPEC_RP_HIGH_HIGH,	/* Rp-3A/Rp-3A */
};

enum power_supply_typec_src_rp {
	POWER_SUPPLY_TYPEC_SRC_RP_STD,
	POWER_SUPPLY_TYPEC_SRC_RP_1P5A,
	POWER_SUPPLY_TYPEC_SRC_RP_3A
};

enum power_supply_typec_power_role {
	POWER_SUPPLY_TYPEC_PR_NONE,		/* CC lines in high-Z */
	POWER_SUPPLY_TYPEC_PR_DUAL,
	POWER_SUPPLY_TYPEC_PR_SINK,
	POWER_SUPPLY_TYPEC_PR_SOURCE,
};

#endif /* __GBMS_POWER_SUPPLY_H__ */
