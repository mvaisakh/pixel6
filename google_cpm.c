/*
 * Copyright 2020 Google, Inc
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#ifdef CONFIG_PM_SLEEP
#define SUPPORT_PM_SLEEP 1
#endif

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/pmic-voter.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include "gvotable.h"
#include "google_bms.h"
#include "google_dc_pps.h"
#include "google_psy.h"
#include "logbuffer.h"

#include <linux/debugfs.h>

#define GCPM_MAX_CHARGERS 4
#define GCPM_DEFAULT_TA_DC_LIMIT 0

/* TODO: move to configuration */
#define DC_TA_VMAX_MV		9800000
/* TODO: move to configuration */
#define DC_TA_VMIN_MV		8000000
/* TODO: move to configuration */
#define DC_VBATT_HEADROOM_MV	500000

enum gcpm_dc_state_t {
	DC_DISABLED = 0,
	DC_ENABLE,
	DC_RUNNING,
	DC_ENABLE_PASSTHROUGH,
	DC_PASSTHROUGH,
};

struct gcpm_drv  {
	struct device *device;
	struct power_supply *psy;
	struct delayed_work init_work;

	int chg_psy_retries;
	struct power_supply *chg_psy_avail[GCPM_MAX_CHARGERS];
	const char *chg_psy_names[GCPM_MAX_CHARGERS];
	struct mutex chg_psy_lock;
	int chg_psy_active;
	int chg_psy_count;
	/* force a charger, this might have side effects */
	int force_active;

	/* TCPM state for PPS charging */
	struct power_supply *tcpm_psy;
	const char *tcpm_psy_name;
	int log_psy_ratelimit;
	u32 tcpm_phandle;

	/* set to force PPS negoatiation */
	bool force_pps;
	/* pps state and detect */
	struct pd_pps_data pps_data;
	struct delayed_work pps_work;
	/* request of output ua, */
	int out_ua;
	int out_uv;

	int dcen_gpio;
	u32 dcen_gpio_default;

	/* >0 when enabled */
	int dc_index;
	/* dc_charging state */
	int dc_state;
	/* force disable */
	bool taper_control;
	/* policy: power demand limit for DC charging */
	u32 ta_dc_limit;

	/* cc_max and fv_uv are demand from google_charger */
	int cc_max;
	int fv_uv;

	struct notifier_block chg_nb;
	bool init_complete;
	bool resume_complete;

	/* tie up to charger mode */
	struct gvotable_election *gbms_mode;

	/* debug fs */
	struct dentry *debug_entry;
};

/* TODO: place a lock around the operation? */
static struct power_supply *gcpm_chg_get_active(struct gcpm_drv *gcpm)
{
	if (gcpm->chg_psy_active == -1)
		return NULL;

	return gcpm->chg_psy_avail[gcpm->chg_psy_active];
}

static int gcpm_chg_ping(struct gcpm_drv *gcpm, int index, bool online)
{
	struct power_supply *chg_psy;
	int ret;

	chg_psy = gcpm->chg_psy_avail[index];
	if (!chg_psy)
		return 0;

	ret = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_ONLINE, 0);
	if (ret < 0)
		pr_debug("adapter %d cannot ping (%d)", index, ret);

	return 0;
}

/*
 * Switch between chargers using ONLINE.
 * NOTE: online doesn't enable charging.
 * NOTE: call holding a lock on charger
 */
static int gcpm_chg_offline(struct gcpm_drv *gcpm)
{
	struct power_supply *chg_psy;
	int ret;

	chg_psy = gcpm_chg_get_active(gcpm);
	if (!chg_psy)
		return 0;

	ret = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_ONLINE, 0);
	if (ret == 0)
		gcpm->chg_psy_active = -1;

	pr_debug("active=%d offline=%d\n", gcpm->chg_psy_active, ret == 0);
	return ret;
}

/* turn current offline (if a current exists), switch to new */
static int gcpm_chg_set_online(struct gcpm_drv *gcpm, int index)
{
	const int index_old = gcpm->chg_psy_active;
	struct power_supply *active;
	int ret;

	if (index < 0 || index >= gcpm->chg_psy_count)
		return -ERANGE;

	if (!gcpm->chg_psy_avail[index]) {
		pr_err("invalid index %d\n", index);
		return -EINVAL;
	}

	ret = gcpm_chg_offline(gcpm);
	if (ret < 0) {
		pr_err("cannot turn %d offline\n", index_old);
		return -EIO;
	}

	active = gcpm->chg_psy_avail[index];

	ret = GPSY_SET_PROP(active, POWER_SUPPLY_PROP_ONLINE, 1);
	if (ret < 0) {
		/* TODO: re-enable the old one if this fail???  */
		goto error_exit;
	}

	gcpm->chg_psy_active = index;

error_exit:
	pr_info("active charger %d->%d (%d)\n", index_old, index, ret);
	return ret;
}

/* use the charger one when avaiable or fallback to the generated one */
static uint64_t gcpm_get_charger_state(const struct gcpm_drv *gcpm,
				       struct power_supply *chg_psy)
{
	union gbms_charger_state chg_state;
	int rc;

	rc = gbms_read_charger_state(&chg_state, chg_psy);
	if (rc < 0)
		return 0;

	return chg_state.v;
}

/*
 * Select the DC charger using the thermal policy.
 * NOTE: program target before enabling chaging.
 */
static int gcpm_chg_dc_select(const struct gcpm_drv *gcpm, int ta_demand)
{
	bool dc_req = !gcpm->taper_control && gcpm->ta_dc_limit &&
		      ta_demand > gcpm->ta_dc_limit;
	int index = 0; /* 0 is the default */

	/* could select different modes here depending on capabilities */
	if (dc_req && gcpm->chg_psy_count > 1)
		index = 1;

	/* add margin .... debounce etc... */

	return index;
}

/* Enable DirectCharge mode, PPS and DC charger must be already initialized */
static int gcpm_dc_enable(struct gcpm_drv *gcpm, bool enabled)
{
	if (!gcpm->gbms_mode) {
		struct gvotable_election *v;

		v = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
		if (IS_ERR_OR_NULL(v))
			return -ENODEV;
		gcpm->gbms_mode = v;
	}

	return gvotable_cast_vote(gcpm->gbms_mode, "GCPM",
				  (void*)GBMS_CHGR_MODE_CHGR_DC,
				  enabled);
}

/* disable DC, and switch back to the default charger */
/* NOTE: call with a lock around gcpm->chg_psy_lock */
static int gcpm_dc_stop(struct gcpm_drv *gcpm)
{
	int ret;

	/* enabled in dc_ready after programming the charger  */
	if (gcpm->dcen_gpio >= 0 && !gcpm->dcen_gpio_default)
		gpio_set_value(gcpm->dcen_gpio, 0);

	switch (gcpm->dc_state) {
	case DC_RUNNING:
	case DC_PASSTHROUGH:
		ret = gcpm_dc_enable(gcpm, false);
		if (ret < 0) {
			pr_err("DC_PPS: Cannot disable DC (%d)", ret);
			break;
		}
		gcpm->dc_state = DC_ENABLE;
		/* Fall Through */
	case DC_ENABLE:
	case DC_ENABLE_PASSTHROUGH:
		ret = gcpm_chg_set_online(gcpm, 0);
		if (ret < 0) {
			pr_err("DC_PPS: Cannot enable default charger (%d)",
			       ret);
			break;
		}
		/* Fall Through */
	default:
		gcpm->dc_state  = DC_DISABLED;
		ret = 0;
		break;
	}

	return ret;
}

/* NOTE: call with a lock around gcpm->chg_psy_lock */
static int gcpm_dc_start(struct gcpm_drv *gcpm, int index)
{
	struct power_supply *dc_psy;
	int ret;

	ret = gcpm_chg_set_online(gcpm, index);
	if (ret < 0) {
		pr_err("PPS_DC: cannot online index=%d (%d)\n", index, ret);
		return ret;
	}

	dc_psy = gcpm_chg_get_active(gcpm);
	if (!dc_psy) {
		pr_err("PPS_DC: gcpm->dc_state == DC_READY, no adapter\n");
		return -ENODEV;
	}

	/* VFLOAT = vbat */
	ret = GPSY_SET_PROP(dc_psy,
			   POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
			    gcpm->fv_uv);
	if (ret < 0) {
		pr_err("PPS_DC: no fv_uv (%d)\n", ret);
		return ret;
	}

	/* ICHG_CHG = cc_max */
	ret = GPSY_SET_PROP(dc_psy,
			    POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
			    gcpm->cc_max);
	if (ret < 0) {
		pr_err("PPS_DC: no cc_max (%d)\n", ret);
		return ret;
	}

	/* set IIN_CFG, */
	ret = GPSY_SET_PROP(dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX,
			    gcpm->out_ua);
	if (ret < 0) {
		pr_err("PPS_DC: no IIN (%d)\n", ret);
		return ret;
	}

	/* enabled in dc_ready after programming the charger  */
	if (gcpm->dcen_gpio >= 0 && !gcpm->dcen_gpio_default)
		gpio_set_value(gcpm->dcen_gpio, 1);

	ret = gcpm_dc_enable(gcpm, true);
	if (ret < 0) {
		pr_info("PPS_DC: dc_ready failed=%d\n", ret);
		return ret;
	}

	pr_info("PPS_DC: dc_ready ok fv_uv=%d cc_max=%d, out_ua=%d\n",
		gcpm->dc_state, gcpm->fv_uv, gcpm->cc_max, gcpm->out_ua);

	return 0;
}

/* NOTE: call with a lock around gcpm->chg_psy_lock */
static int gcpm_dc_charging(struct gcpm_drv *gcpm)
{
	struct power_supply *dc_psy;
	int vchg, ichg, status;

	dc_psy = gcpm_chg_get_active(gcpm);
	if (!dc_psy) {
		pr_err("DC_CHG: invalid charger\n");
		return -ENODEV;
	}

	vchg = GPSY_GET_PROP(dc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	ichg = GPSY_GET_PROP(dc_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	status = GPSY_GET_PROP(dc_psy, POWER_SUPPLY_PROP_STATUS);

	pr_err("DC_CHG: vchg=%d, ichg=%d status=%d\n",
	       vchg, ichg, status);

	return 0;
}

/* Will need to handle capabilities based on index number */
#define GCPM_INDEX_DC_ENABLE	1


/* TODO: do not enable PPS when ->taper_control is set? */
/* TODO: revert to fixed profile if cc_max is 0, remember failures */
/* TODO: use a structure to describe charger types "needs_pps", "is_dc" etc */
static int gcpm_chg_pps_check_enable(const struct gcpm_drv *gcpm, int index)
{
	return gcpm->force_pps || index == GCPM_INDEX_DC_ENABLE;
}

/* TODO: handle PPS-DC and WLC-DC, return dc_stage*/
/* NOTE: DC requires PPS, disable DC in taper control */
static int gcpm_chg_dc_check_enable(const struct gcpm_drv *gcpm, int index)
{
	return index == GCPM_INDEX_DC_ENABLE;
}

/*
 * need to run through the whole function even when index == gcpm->force_active
 * because I have multiple steps and multiple failure points.
 * Call with lock on chg_psy_lock
 */
static int gcpm_chg_check(struct gcpm_drv *gcpm)
{
	struct pd_pps_data *pps_data = &gcpm->pps_data;
	bool schedule_pps_dc = false;
	bool dc_ena, pps_ena;
	int ret = 0, index;

	/* might have more than one policy */
	if (gcpm->force_active >= 0)
		index = gcpm->force_active;
	else
		index = gcpm_chg_dc_select(gcpm, gcpm->cc_max * gcpm->fv_uv);

	pr_debug("CHG_CHK: index:%d->%d\n", gcpm->chg_psy_active, index);

	/* first check if PPS needs to be enabled (or disable it) */
	pps_ena = gcpm->tcpm_psy && gcpm_chg_pps_check_enable(gcpm, index);
	pr_debug("CHG_CHK: PPS pps_stage=%d pps_ena=%d\n",
		 pps_data->stage, pps_ena);

	if (pps_ena && pps_data->stage != PPS_NOTSUPP) {

		/* Could reset gcpm->out_uv and gcpm->out_ua to -1 */
		if (pps_data->stage == PPS_DISABLED) {
			pps_data->stage = PPS_NONE;
			schedule_pps_dc = true;
		}

	} else if (pps_data->stage == PPS_NOTSUPP) {
		/* keep PPS_NOTSUPP, continue to make sure DC is disabled */
	} else if (pps_data->stage != PPS_DISABLED) {

		/* force state, PPS_DC will take cate of it */

		ret = pps_prog_offline(&gcpm->pps_data, gcpm->tcpm_psy);
		pr_debug("CHG_CHK: PPS offline ret=%d\n", ret);
		if (ret < 0) {
			pr_debug("CHG_CHK: PPS pps_ena=%d dc_ena=%d->%d ret=%d\n",
				 pps_ena, gcpm->dc_index, dc_ena, ret);

			/* force state, PPS_DC will take care of it */
			pps_data->stage = PPS_DISABLED;

			/*
			 * transitions between MAIN<->PPS<->PPS+DC are handled
			 * in gcpm_pps_dc_work() and the state of DC for this
			 * index must be handled regardless of the success of
			 * the immediate disable of pps.
			 */
		}

		/* needs to continue of DC was enabled */
		schedule_pps_dc = true;
	}

	/*
	 * NOTE: disabling DC might need to transition to charger mode 0
	 * same might apply when switching between WLC-DC and PPS-DC.
	 */
	dc_ena = gcpm_chg_dc_check_enable(gcpm, index);
	pr_debug("CHG_CHK: DC pps_ena=%d dc_ena=%d->%d \n",
			pps_ena, gcpm->dc_index, dc_ena);
	if (!dc_ena) {

		if (gcpm->dc_index) {
			gcpm->dc_index = 0;
			schedule_pps_dc = true;
		}
	} else if (gcpm->dc_state == DC_DISABLED) {
		gcpm->out_ua = -1;
		gcpm->out_uv = -1;

		/* TODO: DC_ENABLE or DC_PASSTHROUGH depending on index */
		gcpm->dc_state = DC_ENABLE_PASSTHROUGH;
		gcpm->dc_index = index;
		schedule_pps_dc = true;
	}

	if (schedule_pps_dc)
		mod_delayed_work(system_wq, &gcpm->pps_work, 0);

	return ret;
}

#define PPS_ERROR_RETRY_MS 1000

/* DC_ERROR_RETRY_MS <= DC_RUN_DELAY_MS */
#define DC_RUN_DELAY_MS		5000
#define DC_ERROR_RETRY_MS	PPS_ERROR_RETRY_MS

/*
 * pps_data->stage:
 *  PPS_NONE -> PPS_AVAILABLE -> PPS_ACTIVE
 * 	     -> PPS_DISABLED  -> PPS_DISABLED
 */
static void gcpm_pps_dc_work(struct work_struct *work)
{
	struct gcpm_drv *gcpm =
		container_of(work, struct gcpm_drv, pps_work.work);
	struct pd_pps_data *pps_data = &gcpm->pps_data;
	struct power_supply *tcpm_psy = gcpm->tcpm_psy;
	const bool log_on_done = tcpm_psy != 0 && pps_data->pd_online &&
			         (gcpm->pps_data.stage || gcpm->dc_state);
	const int pre_out_ua = pps_data->op_ua;
	const int pre_out_uv = pps_data->out_uv;
	int ret, pps_ui = -ENODEV;

	mutex_lock(&gcpm->chg_psy_lock);

	pr_info("PPS_Work: tcpm_psy_ok=%d pd_online=%d pps_stage=%d dc_state=%d",
		 tcpm_psy != 0, pps_data->pd_online,  gcpm->pps_data.stage,
		 gcpm->dc_state);

	/* should not be calling this with !tcpm_psy */
	if (!gcpm->tcpm_psy)
		goto pps_dc_done;

	/* disable DC if not enabled */
	if (gcpm->dc_index <= 0 && gcpm->dc_state != DC_DISABLED) {
		const int dc_state = gcpm->dc_state;

		ret = gcpm_dc_stop(gcpm);
		if (ret < 0 || gcpm->dc_state != DC_DISABLED) {
			pr_info("PPS_DC: fail disable dc_state=%d (%d)\n",
				gcpm->dc_state, ret);

			pps_ui = DC_ERROR_RETRY_MS;
			goto pps_dc_reschedule;
		}

		pr_info("PPS_DC: disable DC dc_state=%d->%d\n",
			dc_state, gcpm->dc_state);
	}

	if (pps_is_disabled(pps_data->stage))
		goto pps_dc_done;

	/* DC depends on PPS so run PPS first.
	 * - read source capabilities when stage transition from PPS_NONE to
	 *   PPS_AVAILABLE (NOTE: pd_online=TCPM_PSY_PROG_ONLINE in this case)
	 * DISABLED  ->DISABLED
	 * 	     ->NONE
	 *    NONE   ->AVAILABLE
	 * 	     ->DISABLED
	 * 	     ->NOTSUPP
	 * AVAILABLE ->ACTIVE
	 *	     ->DISABLED
	 * NOTSUPP   ->NOTSUPP
	 */
	pps_ui = pps_work(pps_data, tcpm_psy);
	pr_info("PPS_Work: pps_ui=%d pd_online=%d pps_stage=%d dc_state=%d\n",
		pps_ui, pps_data->pd_online,
		gcpm->pps_data.stage, gcpm->dc_state);
	if (pps_ui < 0) {
		pps_ui = PPS_ERROR_RETRY_MS;
	} else if (!pps_data->pd_online) {

		/* disable DC, revert to default charger */
		if (gcpm->dc_state != DC_DISABLED) {
			ret = gcpm_dc_stop(gcpm);
			if (ret < 0 || gcpm->dc_state != DC_DISABLED) {
				pr_info("PPS_DC: fail disable dc_state=%d (%d)\n",
					gcpm->dc_state, ret);

				/* will keep trying to disable */
				pps_ui = DC_ERROR_RETRY_MS;
			}
		}

		/* reset state on offline (should be already done) */
		pr_info("PPS_Work: PPS Offline\n");
		pps_init_state(pps_data);
	} else if (pps_data->stage == PPS_DISABLED) {
		/* PPS was disabled (not available) for this TA */
		pr_info("PPS_Work: PPS Disabled dc_ena=%d, dc_state=%d\n",
			gcpm->dc_index, gcpm->dc_state);

		/* reschedule to clear dc_ena for this session */
		if (gcpm->dc_index > 0) {
			pps_ui = PPS_ERROR_RETRY_MS;
			gcpm->dc_index = 0;
		}
	} else if (pps_data->stage != PPS_ACTIVE) {
		/*
		 * DC run only when PPS is active, the only sane state for DC
		 * when PPS is not ACTIVE is DC_DISABLE:
		 * DC state is forced to DC_DISABLE when ->dc_index <= 0 at
		 * the beginning of the loop
		 */

		pr_info("PPS_Work: PPS Wait stage=%d, pps_ui=%d dc_ena=%d dc_state=%d \n",
			pps_data->stage, pps_ui, gcpm->dc_index, gcpm->dc_state);

		/* TODO: keep track of time waiting for ACTIVE */
	} else if (gcpm->dc_state == DC_ENABLE) {
		struct pd_pps_data *pps_data = &gcpm->pps_data;
		bool pwr_ok;

		/* must run at the end of PPS negotiation */
		if (gcpm->out_ua == -1)
			gcpm->out_ua = min(gcpm->cc_max, pps_data->max_ua);
		if (gcpm->out_uv == -1) {
			struct power_supply *chg_psy =
						gcpm_chg_get_active(gcpm);
			unsigned long ta_max_v, value;
			int vbatt = -1;

			ta_max_v = pps_data->max_ua * pps_data->max_uv;
			ta_max_v /= gcpm->out_ua;
			if (ta_max_v > DC_TA_VMAX_MV)
				ta_max_v = DC_TA_VMAX_MV;

			if (chg_psy)
				vbatt = GPSY_GET_PROP(chg_psy,
						POWER_SUPPLY_PROP_VOLTAGE_NOW);
			if (vbatt < 0)
				vbatt = gcpm->fv_uv;
			if (vbatt < 0)
				vbatt = 0;

			/* good for pca9468 */
			value = 2 * vbatt + DC_VBATT_HEADROOM_MV;
			if (value < DC_TA_VMIN_MV)
				value = DC_TA_VMIN_MV;

			/* PPS voltage in 20mV steps */
			gcpm->out_uv = value - value % 20000;
		}

		pr_info("CHG_CHK: max_uv=%d,max_ua=%d  out_uv=%d,out_ua=%d\n",
			pps_data->max_uv, pps_data->max_ua,
			gcpm->out_uv, gcpm->out_ua);

		pps_ui = pps_update_adapter(pps_data, gcpm->out_uv,
					    gcpm->out_ua, tcpm_psy);
		if (pps_ui < 0)
			pps_ui = PPS_ERROR_RETRY_MS;

		/* wait until adapter is at or over request */
		pwr_ok = pps_data->out_uv == gcpm->out_uv &&
			 pps_data->op_ua == gcpm->out_ua;
		if (pwr_ok) {
			ret = gcpm_chg_offline(gcpm);
			if (ret == 0)
				ret = gcpm_dc_start(gcpm, gcpm->dc_index);
			if (ret == 0) {
				gcpm->dc_state = DC_RUNNING;
				pps_ui = DC_RUN_DELAY_MS;
			}  else if (pps_ui > DC_ERROR_RETRY_MS) {
				pps_ui = DC_ERROR_RETRY_MS;
			}
		}

		/*
		 * TODO: add retries and switch to DC_ENABLE again or to
		 * DC_DISABLED on timeout.
		 */

		pr_info("PPS_DC: dc_state=%d out_uv=%d %d->%d, out_ua=%d %d->%d\n",
			gcpm->dc_state,
			pps_data->out_uv, pre_out_uv, gcpm->out_uv,
			pps_data->op_ua, pre_out_ua, gcpm->out_ua);
	} else if (gcpm->dc_state == DC_RUNNING)  {

		ret = gcpm_chg_ping(gcpm, 0, 0);
		if (ret < 0)
			pr_err("PPS_DC: ping failed with %d\n", ret);

		/* update gcpm->out_uv, gcpm->out_ua */
		pr_info("PPS_DC: dc_state=%d out_uv=%d %d->%d out_ua=%d %d->%d\n",
			gcpm->dc_state,
			pps_data->out_uv, pre_out_uv, gcpm->out_uv,
			pps_data->op_ua, pre_out_ua, gcpm->out_ua);

		ret = gcpm_dc_charging(gcpm);
		if (ret < 0)
			pps_ui = DC_ERROR_RETRY_MS;

		ret = pps_update_adapter(&gcpm->pps_data,
						gcpm->out_uv, gcpm->out_ua,
						tcpm_psy);
		if (ret < 0)
			pps_ui = PPS_ERROR_RETRY_MS;
	} else if (gcpm->dc_state == DC_ENABLE_PASSTHROUGH) {
		struct pd_pps_data *pps_data = &gcpm->pps_data;

		pps_ui = pps_update_adapter(pps_data, gcpm->out_uv,
					    gcpm->out_ua, tcpm_psy);
		if (pps_ui < 0)
			pps_ui = PPS_ERROR_RETRY_MS;

		/* TODO: handoff handling of PPS to the DC charger */
		ret = gcpm_chg_offline(gcpm);
		if (ret == 0)
			ret = gcpm_dc_start(gcpm, gcpm->dc_index);
		if (ret == 0) {
			gcpm->dc_state = DC_PASSTHROUGH;
			pps_ui = DC_RUN_DELAY_MS;
		} else if (pps_ui > DC_ERROR_RETRY_MS) {
			pps_ui = DC_ERROR_RETRY_MS;
		}
	} else if (gcpm->dc_state == DC_PASSTHROUGH) {
		struct power_supply *dc_psy;

		dc_psy = gcpm_chg_get_active(gcpm);
		if (!dc_psy) {
			pr_err("PPS_Work: no adapter while in DC_PASSTHROUGH\n");
			pps_ui = DC_ERROR_RETRY_MS;
		} else {
			ret = GPSY_SET_PROP(dc_psy,
					    POWER_SUPPLY_PROP_CHARGING_ENABLED,
					    1);
			if (ret == 0) {
				ret = gcpm_chg_ping(gcpm, 0, 0);
				if (ret < 0)
					pr_err("PPS_DC: ping failed with %d\n",
					       ret);

			} else if (ret == -EBUSY) {
				pps_ui = DC_ERROR_RETRY_MS;
			} else {
				pr_err("PPS_Work: cannot enable DC_charging\n");

				ret = gcpm_chg_set_online(gcpm, 0);
				if (ret < 0)
					pr_err("PPS_Work: online default %d\n",
					       ret);
			}
		}

	} else {
		/* steady on PPS, DC is not enabled */
		pps_ui = pps_update_adapter(&gcpm->pps_data, -1, -1, tcpm_psy);

		pr_info("PPS_Work: STEADY pd_online=%d pps_ui=%d dc_ena=%d dc_state=%d\n",
			pps_data->pd_online, pps_ui, gcpm->dc_index,
			gcpm->dc_state);
		if (pps_ui < 0)
			pps_ui = PPS_ERROR_RETRY_MS;
	}

pps_dc_reschedule:
	if (pps_ui <= 0) {
		pr_info("PPS_Work: done=%d pps_stage=%d dc_state=%d",
			pps_ui, gcpm->pps_data.stage, gcpm->dc_state);
		if (log_on_done)
			pps_log(pps_data, "PPS_Work: done=%d pd_ol=%d pps_stage=%d dc_state=%d\n",
				pps_ui, pps_data->pd_online, gcpm->pps_data.stage,
				gcpm->dc_state);
	} else {
		pr_info("PPS_Work: reschedule in %d pps_stage=%d (%d:%d) dc_state=%d (%d:%d)",
			pps_ui, gcpm->pps_data.stage,
			gcpm->pps_data.out_uv, gcpm->pps_data.op_ua,
			gcpm->dc_state, gcpm->out_uv, gcpm->out_ua);
		schedule_delayed_work(&gcpm->pps_work,
				      msecs_to_jiffies(pps_ui));
	}

pps_dc_done:
	mutex_unlock(&gcpm->chg_psy_lock);
}

static int gcpm_psy_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *pval)
{
	struct gcpm_drv *gcpm = power_supply_get_drvdata(psy);
	bool taper_control, ta_check = false;
	struct power_supply *chg_psy = NULL;
	bool route = true;
	int ret = 0;

	pm_runtime_get_sync(gcpm->device);
	if (!gcpm->init_complete || !gcpm->resume_complete) {
		pm_runtime_put_sync(gcpm->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(gcpm->device);

	mutex_lock(&gcpm->chg_psy_lock);
	switch (psp) {

		/* do not route to the active charger */
		case POWER_SUPPLY_PROP_TAPER_CONTROL:
			taper_control = (pval->intval != 0);
			ta_check = taper_control != gcpm->taper_control;
			gcpm->taper_control = taper_control;
			route = false;
			break;

		/* also route to the active charger */
		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			psp = POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX;
			/* compat, fall through */
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
			ta_check = gcpm->fv_uv != pval->intval;
			gcpm->fv_uv = pval->intval;
			break;

		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
			ta_check = gcpm->cc_max != pval->intval;
			gcpm->cc_max = pval->intval;
			break;

		/* just route to the active charger */
		default:
			break;
	}

	/* logic that select the active charging */
	if (ta_check)
		gcpm_chg_check(gcpm);
	/*  route to active charger when needed */
	if (route)
		chg_psy = gcpm_chg_get_active(gcpm);
	if (chg_psy) {
		ret = power_supply_set_property(chg_psy, psp, pval);
		if (ret < 0) {
			const char *name= (chg_psy->desc && chg_psy->desc->name) ?
				chg_psy->desc->name : "???";

			pr_err("cannot route prop=%d to %d:%s\n",
				psp, gcpm->chg_psy_active, name);
		}
	} else {
		pr_err("invalid charger=%d for prop=%d\n",
			gcpm->chg_psy_active, psp);
	}

	/* the charger should not call into gcpm: this can change though */
	mutex_unlock(&gcpm->chg_psy_lock);
	return ret;
}

static int gcpm_psy_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *pval)
{
	struct gcpm_drv *gcpm = power_supply_get_drvdata(psy);
	struct power_supply *chg_psy;

	pm_runtime_get_sync(gcpm->device);
	if (!gcpm->init_complete || !gcpm->resume_complete) {
		pm_runtime_put_sync(gcpm->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(gcpm->device);

	mutex_lock(&gcpm->chg_psy_lock);
	chg_psy = gcpm_chg_get_active(gcpm);
	mutex_unlock(&gcpm->chg_psy_lock);
	if (!chg_psy)
		return -ENODEV;

	switch (psp) {

		/* handle locally for now */
		case POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE:
			pval->int64val = gcpm_get_charger_state(gcpm, chg_psy);
			return 0;

		/* route to the active charger */
		default:
			break;
	}

	return power_supply_get_property(chg_psy, psp, pval);
}

static int gcpm_psy_is_writeable(struct power_supply *psy,
				 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CHARGE_DISABLE:
	case POWER_SUPPLY_PROP_TAPER_CONTROL:
		return 1;
	default:
		break;
	}

	return 0;
}

/*
 * TODO: POWER_SUPPLY_PROP_RERUN_AICL, POWER_SUPPLY_PROP_TEMP
 */
static enum power_supply_property gcpm_psy_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_DISABLE,
	POWER_SUPPLY_PROP_CHARGE_DONE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	/* pixel battery management subsystem */
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,	/* cc_max */
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,	/* fv_uv */
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CHARGE_CHARGER_STATE,
	POWER_SUPPLY_PROP_CURRENT_MAX,	/* input current limit */
	POWER_SUPPLY_PROP_VOLTAGE_MAX,	/* set float voltage, compat */
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_STATUS,
};

static struct power_supply_desc gcpm_psy_desc = {
	.name = "gcpm",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property = gcpm_psy_get_property,
	.set_property = gcpm_psy_set_property,
	.property_is_writeable = gcpm_psy_is_writeable,
	.properties = gcpm_psy_properties,
	.num_properties = ARRAY_SIZE(gcpm_psy_properties),
};

static int gcpm_psy_changed(struct notifier_block *nb, unsigned long action,
			    void *data)
{
	struct gcpm_drv *gcpm = container_of(nb, struct gcpm_drv, chg_nb);
	const int index = gcpm->chg_psy_active;
	struct power_supply *psy = data;

	if (index == -1)
		return NOTIFY_OK;

	if ((action != PSY_EVENT_PROP_CHANGED) ||
	    (psy == NULL) || (psy->desc == NULL) || (psy->desc->name == NULL))
		return NOTIFY_OK;

	if (strcmp(psy->desc->name, gcpm->chg_psy_names[index]) == 0) {
		/* route upstream when the charger active and found */
		if (gcpm->chg_psy_avail[index])
			power_supply_changed(gcpm->psy);
		mod_delayed_work(system_wq, &gcpm->pps_work, 0);
	} else if (strcmp(psy->desc->name, gcpm->chg_psy_names[0]) == 0) {
		/* something is up with the default charger */
		mod_delayed_work(system_wq, &gcpm->pps_work, 0);
	} else if (gcpm->tcpm_psy_name &&
	      !strcmp(psy->desc->name, gcpm->tcpm_psy_name))
	{
		/* kick off PPS */
		mod_delayed_work(system_wq, &gcpm->pps_work, 0);
	}

	return NOTIFY_OK;
}

#define INIT_DELAY_MS 100
#define INIT_RETRY_DELAY_MS 1000

#define GCPM_TCPM_PSY_MAX 2

static void gcpm_init_work(struct work_struct *work)
{
	struct gcpm_drv *gcpm = container_of(work, struct gcpm_drv,
					     init_work.work);
	int i, found = 0, ret = 0;

	/*
	 * could call pps_init() in probe() and use lazy init for ->tcpm_psy
	 * when the device an APDO in the sink capabilities.
	 */
	if (gcpm->tcpm_phandle && !gcpm->tcpm_psy) {
		struct power_supply *tcpm_psy;

		tcpm_psy = pps_get_tcpm_psy(gcpm->device->of_node,
					    GCPM_TCPM_PSY_MAX);
		if (!IS_ERR_OR_NULL(tcpm_psy)) {
			gcpm->tcpm_psy_name = tcpm_psy->desc->name;
			gcpm->tcpm_psy = tcpm_psy;

			/* PPS charging: needs an APDO */
			ret = pps_init(&gcpm->pps_data, gcpm->device);
			if (ret == 0 && gcpm->debug_entry)
				pps_init_fs(&gcpm->pps_data, gcpm->debug_entry);
			if (ret < 0) {
				pr_err("PPS init failure for %s (%d)\n",
				       tcpm_psy->desc->name, ret);
			} else {
				pps_init_state(&gcpm->pps_data);
				pr_info("PPS available for %s\n",
					gcpm->tcpm_psy_name);
			}

		} else if (!tcpm_psy || !gcpm->log_psy_ratelimit) {
			pr_warn("PPS not available\n");
			gcpm->tcpm_phandle = 0;
		} else {
			pr_warn("tcpm power supply not found, retrying... ret:%d\n",
				ret);
			gcpm->log_psy_ratelimit--;
		}

	}

	/* default is index 0 */
	for (i = 0; i < gcpm->chg_psy_count; i++) {
		if (!gcpm->chg_psy_avail[i]) {
			const char *name = gcpm->chg_psy_names[i];

			gcpm->chg_psy_avail[i] = power_supply_get_by_name(name);
			if (gcpm->chg_psy_avail[i])
				pr_info("init_work found %d:%s\n", i, name);
		}

		found += !!gcpm->chg_psy_avail[i];
	}

	/* we done when we have (at least) the primary */
	if (gcpm->chg_psy_avail[0]) {

		/* register the notifier only when have one (the default) */
		if (!gcpm->init_complete) {
			gcpm->chg_nb.notifier_call = gcpm_psy_changed;
			ret = power_supply_reg_notifier(&gcpm->chg_nb);
			if (ret < 0)
				pr_err("cannot register power supply notifer, ret=%d\n",
				ret);
		}

		gcpm->resume_complete = true;
		gcpm->init_complete = true;
	}

	/* keep looking for late arrivals && TCPM */
	if (found != gcpm->chg_psy_count && gcpm->chg_psy_retries)
		gcpm->chg_psy_retries--;

	if (gcpm->chg_psy_retries || (gcpm->tcpm_phandle && !gcpm->tcpm_psy)) {
		const unsigned long jif = msecs_to_jiffies(INIT_RETRY_DELAY_MS);

		schedule_delayed_work(&gcpm->init_work, jif);
	} else {
		pr_info("init_work done %d/%d pps=%d\n", found,
			gcpm->chg_psy_count, !!gcpm->tcpm_psy);
	}
}

static int gcpm_debug_get_active(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->chg_psy_active;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_set_active(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = val;
	int ret;

	if (intval != -1 && (intval < 0 || intval >= gcpm->chg_psy_count))
		return -ERANGE;
	if (intval != -1 && !gcpm->chg_psy_avail[intval])
		return -EINVAL;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->force_active = val;
	ret = gcpm_chg_check(gcpm);
	mutex_unlock(&gcpm->chg_psy_lock);

	return (ret < 0) ? ret : 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_active_fops, gcpm_debug_get_active,
			gcpm_debug_set_active, "%llu\n");

static int gcpm_debug_pps_stage_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->pps_data.stage;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_pps_stage_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	if (intval < PPS_DISABLED || intval > PPS_ACTIVE)
		return -EINVAL;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->pps_data.stage = intval;
	gcpm->force_pps = !pps_is_disabled(intval);
	mod_delayed_work(system_wq, &gcpm->pps_work, 0);
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_pps_stage_fops, gcpm_debug_pps_stage_get,
			gcpm_debug_pps_stage_set, "%llu\n");

static struct dentry *gcpm_init_fs(struct gcpm_drv *gcpm)
{
	struct dentry *de;

	de = debugfs_create_dir("google_cpm", 0);
	if (IS_ERR_OR_NULL(de))
		return NULL;

	debugfs_create_file("active", 0644, de, gcpm, &gcpm_debug_active_fops);
	debugfs_create_u32("dc_ta_limit", 0644, de, &gcpm->ta_dc_limit);
	debugfs_create_file("pps_stage", 0644, de, gcpm,
			    &gcpm_debug_pps_stage_fops);

	return de;
}

static int gcpm_probe_psy_names(struct gcpm_drv *gcpm)
{
	struct device *dev = gcpm->device;
	int i, count, ret;

	if (!gcpm->device)
		return -EINVAL;

	count = of_property_count_strings(dev->of_node,
					  "google,chg-power-supplies");
	if (count <= 0 || count > GCPM_MAX_CHARGERS)
		return -ERANGE;

	ret = of_property_read_string_array(dev->of_node,
					    "google,chg-power-supplies",
					    (const char**)&gcpm->chg_psy_names,
					    count);
	if (ret != count)
		return -ERANGE;

	for (i = 0; i < count; i++)
		dev_info(gcpm->device, "%d:%s\n", i, gcpm->chg_psy_names[i]);

	return count;
}

#define LOG_PSY_RATELIMIT_CNT	200

static int google_cpm_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = { 0 };
	const char *tmp_name = NULL;
	struct gcpm_drv *gcpm;
	int ret;

	gcpm = devm_kzalloc(&pdev->dev, sizeof(*gcpm), GFP_KERNEL);
	if (!gcpm)
		return -ENOMEM;

	gcpm->device = &pdev->dev;
	gcpm->force_active = -1;
	gcpm->log_psy_ratelimit = LOG_PSY_RATELIMIT_CNT;
	gcpm->chg_psy_retries = 10; /* chg_psy_retries *  INIT_RETRY_DELAY_MS */
	gcpm->out_uv = -1;
	gcpm->out_ua = -1;
	INIT_DELAYED_WORK(&gcpm->pps_work, gcpm_pps_dc_work);
	INIT_DELAYED_WORK(&gcpm->init_work, gcpm_init_work);
	mutex_init(&gcpm->chg_psy_lock);

	/* this is my name */
	ret = of_property_read_string(pdev->dev.of_node, "google,psy-name",
				      &tmp_name);
	if (ret == 0) {
		gcpm_psy_desc.name = devm_kstrdup(&pdev->dev, tmp_name,
						  GFP_KERNEL);
		if (!gcpm_psy_desc.name)
			return -ENOMEM;
	}

	/* sub power supply names */
	gcpm->chg_psy_count = gcpm_probe_psy_names(gcpm);
	if (gcpm->chg_psy_count <= 0)
		return -ENODEV;

	/* PPS needs a TCPM power supply */
	ret = of_property_read_u32(pdev->dev.of_node,
				   "google,tcpm-power-supply",
				   &gcpm->tcpm_phandle);
	if (ret < 0)
		pr_warn("google,tcpm-power-supply not defined\n");

	/* Direct Charging: valid only with PPS */
	gcpm->dcen_gpio = of_get_named_gpio(pdev->dev.of_node,
					    "google,dc-en", 0);
	if (gcpm->dcen_gpio >= 0) {
		of_property_read_u32(pdev->dev.of_node, "google,dc-en-value",
				     &gcpm->dcen_gpio_default);

		/* make sure that the DC is DISABLED before doing this */
		ret = gpio_direction_output(gcpm->dcen_gpio,
					    gcpm->dcen_gpio_default);
		pr_info("google,dc-en value = %d ret=%d\n",
			gcpm->dcen_gpio_default, ret);
	}

	ret = of_property_read_u32(pdev->dev.of_node, "google,dc-limit",
				   &gcpm->ta_dc_limit);
	if (ret == 0)
		gcpm->ta_dc_limit = GCPM_DEFAULT_TA_DC_LIMIT;

	/* sysfs & debug */
	gcpm->debug_entry = gcpm_init_fs(gcpm);
	if (!gcpm->debug_entry)
		pr_warn("No debug control\n");

	platform_set_drvdata(pdev, gcpm);

	psy_cfg.drv_data = gcpm;
	psy_cfg.of_node = pdev->dev.of_node;
	gcpm->psy = devm_power_supply_register(gcpm->device, &gcpm_psy_desc,
					       &psy_cfg);
	if (IS_ERR(gcpm->psy)) {
		ret = PTR_ERR(gcpm->psy);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		/* TODO: fail with -ENODEV */
		dev_err(gcpm->device, "Couldn't register as power supply, ret=%d\n",
			ret);
	}

	/* give time to fg driver to start */
	schedule_delayed_work(&gcpm->init_work,
			      msecs_to_jiffies(INIT_DELAY_MS));

	return 0;
}

static int google_cpm_remove(struct platform_device *pdev)
{
	struct gcpm_drv *gcpm = platform_get_drvdata(pdev);
	int i;

	if (!gcpm)
		return 0;

	for (i = 0; i < gcpm->chg_psy_count; i++) {
		if (!gcpm->chg_psy_avail[i])
			continue;

		power_supply_put(gcpm->chg_psy_avail[i]);
		gcpm->chg_psy_avail[i] = NULL;
	}

	return 0;
}

static const struct of_device_id google_cpm_of_match[] = {
	{.compatible = "google,cpm"},
	{},
};
MODULE_DEVICE_TABLE(of, google_cpm_of_match);


static struct platform_driver google_cpm_driver = {
	.driver = {
		   .name = "google_cpm",
		   .owner = THIS_MODULE,
		   .of_match_table = google_cpm_of_match,
#ifdef SUPPORT_PM_SLEEP
		   /* .pm = &gcpm_pm_ops, */
#endif
		   /* .probe_type = PROBE_PREFER_ASYNCHRONOUS, */
		   },
	.probe = google_cpm_probe,
	.remove = google_cpm_remove,
};

static int __init google_cpm_init(void)
{
	int ret;

	ret = platform_driver_register(&google_cpm_driver);
	if (ret < 0) {
		pr_err("device registration failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static void __init google_cpm_exit(void)
{
	platform_driver_unregister(&google_cpm_driver);
	pr_info("unregistered platform driver\n");
}

module_init(google_cpm_init);
module_exit(google_cpm_exit);
MODULE_DESCRIPTION("Google Charging Policy Manager");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_LICENSE("GPL");
