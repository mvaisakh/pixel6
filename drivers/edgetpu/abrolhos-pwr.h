/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Power management header for Abrolhos.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_PWR_H__
#define __EDGETPU_PWR_H__

/* Can't build out of tree with acpm_dvfs unless kernel supports ACPM */
#if IS_ENABLED(CONFIG_ACPM_DVFS)

#include <linux/acpm_dvfs.h>

#else

static int exynos_acpm_set_rate(unsigned int id, unsigned long rate)
{
	return 0;
}
static int exynos_acpm_set_init_freq(unsigned int dfs_id, unsigned long freq)
{
	return 0;
}
static unsigned long exynos_acpm_get_rate(unsigned int id,
					  unsigned long dbg_val)
{
	return 0;
}
static int exynos_acpm_set_policy(unsigned int id, unsigned long policy)
{
	return 0;
}

#endif /* IS_ENABLED(CONFIG_ACPM_DVFS) */

/*
 * TPU Power States:
 * 0:			Off
 * 1:			Deep Sleep Clocks Off
 * 2:			Deep Sleep Clocks Slow
 * 3:			Deep Sleep Clocks Fast
 * 4:			Sleep Clocks Off
 * 5:			Sleep Clocks Slow
 * 6:			Retention Clocks Slow
 * 500000000:	Super Underdrive @500MHz
 * 800000000:	Underdrive @800MHz
 * 1066000000:	Nominal @1066MHz
 * 1230000000:	Overdrive @1230MHz
 */
enum tpu_pwr_state {
	TPU_OFF = 0,
	TPU_DEEP_SLEEP_CLOCKS_OFF  = 1,
	TPU_DEEP_SLEEP_CLOCKS_SLOW = 2,
	TPU_DEEP_SLEEP_CLOCKS_FAST = 3,
	TPU_SLEEP_CLOCKS_OFF       = 4,
	TPU_SLEEP_CLOCKS_SLOW      = 5,
	TPU_RETENTION_CLOCKS_SLOW  = 6,
	TPU_ACTIVE_SUD = 500000000,
	TPU_ACTIVE_UD  = 800000000,
	TPU_ACTIVE_NOM = 1066000000,
	TPU_ACTIVE_OD  = 1230000000,
};

#define TPU_POLICY_MAX	TPU_ACTIVE_OD

#endif /* __EDGETPU_PWR_H__ */
