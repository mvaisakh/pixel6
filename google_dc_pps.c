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

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/time.h>
#include <linux/usb/pd.h>
#include <linux/usb/tcpm.h>
#include <linux/alarmtimer.h>
#include "google_bms.h"
#include "google_psy.h"
#include "google_dc_pps.h"
#include "logbuffer.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#define PPS_UPDATE_DELAY_MS		2000

#define OP_SNK_MW			7600 /* see b/159863291 */
#define PD_SNK_MAX_MV			9000
#define PD_SNK_MIN_MV			5000
#define PD_SNK_MAX_MA			3000
#define PD_SNK_MAX_MA_9V		2200


#define PDO_FIXED_FLAGS \
	(PDO_FIXED_DUAL_ROLE | PDO_FIXED_DATA_SWAP | PDO_FIXED_USB_COMM)

/*
 * There is a similar one in tcpm.c
 * NOTE: we don't really need to replicate the values in tcpm.c in the
 * internal state, we just need to know to set 2 to the TCPM power supply.
 */
enum tcpm_psy_online_states {
	TCPM_PSY_OFFLINE = 0,
	TCPM_PSY_FIXED_ONLINE,
	TCPM_PSY_PROG_ONLINE,
};

#define get_boot_sec() div_u64(ktime_to_ns(ktime_get_boottime()), NSEC_PER_SEC)

void pps_log(struct pd_pps_data *pps, const char *fmt, ...)
{
	va_list args;

	if (!pps || !pps->log)
		return;

	va_start(args, fmt);
	logbuffer_vlog(pps->log, fmt, args);
	va_end(args);
}

/* SW enable detect setting ->stage = PPS_NONE and calling pps_work() */
void pps_init_state(struct pd_pps_data *pps_data)
{
	/* pps_data->src_caps can be NULL */
	tcpm_put_partner_src_caps(&pps_data->src_caps);

	pps_data->pd_online = TCPM_PSY_OFFLINE;
	pps_data->stage = PPS_DISABLED;
	pps_data->keep_alive_cnt = 0;
	pps_data->nr_src_cap = 0;
	pps_data->src_caps = NULL;

	/* not reference counted */
	if (pps_data->stay_awake)
		__pm_relax(&pps_data->pps_ws);

}
EXPORT_SYMBOL_GPL(pps_init_state);

struct tcpm_port *chg_get_tcpm_port(struct power_supply *tcpm_psy)
{
	return tcpm_psy ? power_supply_get_drvdata(tcpm_psy) : NULL;
}

/* PUBLIC TODO: pass the actual PDO, deprecate */
int chg_update_capability(struct power_supply *tcpm_psy,
				 unsigned int nr_pdo,
				 u32 pps_cap)
{
	struct tcpm_port *port = chg_get_tcpm_port(tcpm_psy);
	const u32 pdo[] = {PDO_FIXED(5000, PD_SNK_MAX_MA, PDO_FIXED_FLAGS),
			   PDO_FIXED(PD_SNK_MAX_MV, PD_SNK_MAX_MA_9V, 0),
			   pps_cap};

	if (!port)
		return -ENODEV;

	if (!nr_pdo || nr_pdo > PDO_MAX_SUPP)
		return -EINVAL;

	return tcpm_update_sink_capabilities(port, pdo, nr_pdo, OP_SNK_MW);
}

/* false when not present or error (either way don't run) */
static unsigned int pps_is_avail(struct pd_pps_data *pps,
				 struct power_supply *tcpm_psy)
{
	pps->max_uv = GPSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX);
	pps->min_uv = GPSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_VOLTAGE_MIN);
	pps->max_ua = GPSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_CURRENT_MAX);
	pps->out_uv = GPSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	pps->op_ua = GPSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	if (pps->max_uv < 0 || pps->min_uv < 0 || pps->max_ua < 0 ||
		pps->out_uv < 0 || pps->op_ua < 0)
		return PPS_NONE;

	/* TODO: lower the loglevel after the development stage */
	pps_log(pps, "max_v %d, min_v %d, max_c %d, out_v %d, op_c %d",
		pps->max_uv, pps->min_uv, pps->max_ua, pps->out_uv,
		pps->op_ua);

	/* FIXME: set interval to PD_T_PPS_TIMEOUT here may cause timeout */
	return PPS_AVAILABLE;
}

/* make sure that the adapter doesn't revert back to FIXED PDO */
int pps_ping(struct pd_pps_data *pps, struct power_supply *tcpm_psy)
{
	int rc;

	if (!tcpm_psy)
		return -ENODEV;

	rc = GPSY_SET_PROP(tcpm_psy, POWER_SUPPLY_PROP_ONLINE,
			   TCPM_PSY_PROG_ONLINE);
	if (rc == 0)
		pps->pd_online = TCPM_PSY_PROG_ONLINE;
	else if (rc != -EAGAIN && rc != -EOPNOTSUPP)
		pps_log(pps, "failed to ping, ret = %d", rc);

	return rc;
}
EXPORT_SYMBOL_GPL(pps_ping);

int pps_get_src_cap(struct pd_pps_data *pps, struct power_supply *tcpm_psy)
{
	struct tcpm_port *port = chg_get_tcpm_port(tcpm_psy);

	if (!pps || !port)
		return -EINVAL;

	pps->nr_src_cap = tcpm_get_partner_src_caps(port, &pps->src_caps);

	return pps->nr_src_cap;
}
EXPORT_SYMBOL_GPL(pps_get_src_cap);

/* assume that we are already online and in PPS stage */
bool pps_prog_check_online(struct pd_pps_data *pps_data,
			  struct power_supply *tcpm_psy)
{
	int pd_online = 0;

	pd_online = GPSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_ONLINE);
	if (pd_online == 0) {
		pps_init_state(pps_data);
		return false;
	}

	if (pd_online != TCPM_PSY_PROG_ONLINE)
		goto not_supp;

	if (pps_data->stage != PPS_ACTIVE) {
		int rc;

		pps_data->stage = pps_is_avail(pps_data, tcpm_psy);
		if (pps_data->stage != PPS_AVAILABLE) {
			pr_debug("%s: not available\n", __func__);
			goto not_supp;
		}

		rc = pps_ping(pps_data, tcpm_psy);
		if (rc < 0) {
			pr_debug("%s: ping failed %d\n", __func__, rc);
			goto not_supp;
		}

		pps_data->last_update = get_boot_sec();
		rc = pps_get_src_cap(pps_data, tcpm_psy);
		if (rc < 0) {
			pr_debug("%s: no source caps %d\n", __func__, rc);
			goto not_supp;
		}

		pps_data->pd_online = TCPM_PSY_PROG_ONLINE;
		pps_data->stage = PPS_ACTIVE;
	}

	return true;
not_supp:
	pps_init_state(pps_data);
	pps_data->stage = PPS_NOTSUPP;
	return false;
}
EXPORT_SYMBOL_GPL(pps_prog_check_online);

/* enable PPS prog mode (Internal), also start the negotiation */
static int pps_prog_online(struct pd_pps_data *pps,
			   struct power_supply *tcpm_psy)
{
	int ret;

	ret = GPSY_SET_PROP(tcpm_psy, POWER_SUPPLY_PROP_ONLINE,
			    TCPM_PSY_PROG_ONLINE);
	if (ret == 0) {
		pps->pd_online = TCPM_PSY_PROG_ONLINE;
		pps->stage = PPS_NONE;
	}

	pps->last_update = get_boot_sec();
	return ret;
}

/* Disable PPS prog mode, will end up in PPS_NOTSUP */
int pps_prog_offline(struct pd_pps_data *pps, struct power_supply *tcpm_psy)
{
	int ret;

	if (!tcpm_psy)
		return -ENODEV;

	/* pd_online = TCPM_PSY_OFFLINE, stage = PPS_NONE; */
	ret = GPSY_SET_PROP(tcpm_psy, POWER_SUPPLY_PROP_ONLINE,
			    TCPM_PSY_FIXED_ONLINE);
	if (ret == -EOPNOTSUPP)
		ret = 0;
	if (ret == 0)
		pps_init_state(pps);

	return ret;
}
EXPORT_SYMBOL_GPL(pps_prog_offline);


void pps_adjust_volt(struct pd_pps_data *pps, int mod)
{
	if (mod > 0) {
		pps->out_uv = (pps->out_uv + mod) < pps->max_uv ?
			      (pps->out_uv + mod) : pps->max_uv;
	} else if (mod < 0) {
		pps->out_uv = (pps->out_uv + mod) > pps->min_uv ?
			      (pps->out_uv + mod) : pps->min_uv;
	}
}

/*
 * Note: Some adapters have several PPS profiles providing different voltage
 * ranges and different maximal currents. If the device demands more power from
 * the adapter but reached the maximum it can get in the current profile,
 * search if there exists another profile providing more power. If it demands
 * less power, search if there exists another profile providing enough power
 * with higher current.
 *
 * return negative on errors or no suitable profile
 * return 0 on successful profile switch
 */
int pps_switch_profile(struct pd_pps_data *pps, struct power_supply *tcpm_psy,
		       bool more_pwr)
{
	int i, ret = -ENODATA;
	u32 pdo;
	u32 max_mv, max_ma, max_mw;
	u32 current_mw, current_ma;

	if (pps->nr_src_cap < 2)
		return -EINVAL;

	current_ma = pps->op_ua / 1000;
	current_mw = (pps->out_uv / 1000) * current_ma / 1000;

	for (i = 1; i < pps->nr_src_cap; i++) {
		pdo = pps->src_caps[i];

		if (pdo_type(pdo) != PDO_TYPE_APDO)
			continue;

		/* TODO: use pps_data sink capabilities */
		max_mv = min_t(u32, PD_SNK_MAX_MV,
			       pdo_pps_apdo_max_voltage(pdo));
		/* TODO: use pps_data sink capabilities */
		max_ma = min_t(u32, PD_SNK_MAX_MA,
			       pdo_pps_apdo_max_current(pdo));
		max_mw = max_mv * max_ma / 1000;

		if (more_pwr && max_mw > current_mw) {
			/* export maximal capability, TODO: use sink cap */
			pdo = PDO_PPS_APDO(PD_SNK_MIN_MV,
					   PD_SNK_MAX_MV,
					   PD_SNK_MAX_MA);
			ret = chg_update_capability(tcpm_psy, PDO_PPS, pdo);
			if (ret < 0)
				pps_log(pps, "Failed to update sink caps, ret %d",
					ret);
			break;
		} else if (!more_pwr && max_mw >= current_mw &&
			   max_ma > current_ma) {

			/* TODO: tune the max_mv, fix this */
			pdo = PDO_PPS_APDO(PD_SNK_MIN_MV, 6000, PD_SNK_MAX_MA);
			ret = chg_update_capability(tcpm_psy, PDO_PPS, pdo);
			if (ret < 0)
				pps_log(pps, "Failed to update sink caps, ret %d",
					ret);
			break;
		}
	}

	if (ret == 0) {
		pps->keep_alive_cnt = 0;
		pps->stage = PPS_NONE;
	}

	return ret;
}

/* ------------------------------------------------------------------------ */

static int debug_get_pps_out_uv(void *data, u64 *val)
{
	struct pd_pps_data *pps_data = data;

	*val = pps_data->out_uv;
	return 0;
}

static int debug_set_pps_out_uv(void *data, u64 val)
{
	struct pd_pps_data *pps_data = data;

	/* TODO: use votable */
	pps_data->out_uv = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_pps_out_uv_fops,
				debug_get_pps_out_uv,
				debug_set_pps_out_uv, "%llu\n");

static int debug_get_pps_op_ua(void *data, u64 *val)
{
	struct pd_pps_data *pps_data = data;

	*val = pps_data->op_ua;
	return 0;
}

static int debug_set_pps_op_ua(void *data, u64 val)
{
	struct pd_pps_data *pps_data = data;

	/* TODO: use votable */
	pps_data->op_ua = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_pps_op_ua_fops,
					debug_get_pps_op_ua,
					debug_set_pps_op_ua, "%llu\n");

int pps_init_fs(struct pd_pps_data *pps_data, struct dentry *de)
{
	if (!de)
		return -ENODEV;

	debugfs_create_file("pps_out_uv", 0600, de, pps_data,
			    &debug_pps_out_uv_fops);
	debugfs_create_file("pps_out_ua", 0600, de, pps_data,
			    &debug_pps_op_ua_fops);
	debugfs_create_file("pps_op_ua", 0600, de, pps_data,
			    &debug_pps_op_ua_fops);

	return 0;
}

/* ------------------------------------------------------------------------ */

static int pps_get_sink_pdo(u32 *snk_pdo, int max, struct device_node *node)
{
	struct device_node *dn, *conn;
	unsigned int nr_snk_pdo;
	const __be32 *prop;
	int length, ret;

	prop = of_get_property(node, "google,usbc-connector", NULL);
	if (!prop)
		prop = of_get_property(node, "google,usb-c-connector", NULL);
	if (!prop)
		prop = of_get_property(node, "google,tcpm-power-supply", NULL);
	if (!prop) {
		pr_err("Couldn't find property \n");
		return -ENOENT;
	}

	dn = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!dn) {
		pr_err("Couldn't find usb_con node\n");
		return -ENOENT;
	}

	conn = of_get_child_by_name(dn, "connector");
	if (conn) {
		of_node_put(dn);
		dn = conn;
	}

	prop = of_get_property(dn, "sink-pdos", &length);
	if (!prop) {
		pr_err("Couldn't find sink-pdos property\n");
		of_node_put(dn);
		return -ENOENT;
	}

	nr_snk_pdo = length / sizeof(u32);
	if (nr_snk_pdo == 0 || nr_snk_pdo > max) {
		pr_err("Invalid length of sink-pdos\n");
		of_node_put(dn);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(dn, "sink-pdos", snk_pdo, nr_snk_pdo);
	of_node_put(dn);
	if (ret < 0)
		return ret;

	return nr_snk_pdo;
}

static int pps_find_apdo(struct pd_pps_data *pps_data, int nr_snk_pdo)
{
	int i;

	for (i = 0; i < nr_snk_pdo; i++) {
		u32 pdo = pps_data->snk_pdo[i];

		if (pdo_type(pdo) != PDO_TYPE_FIXED)
			pr_debug("%s %d type=%d", __func__, i, pdo_type(pdo));
		else
			pr_debug("%s %d FIXED v=%d c=%d", __func__, i,
				 pdo_fixed_voltage(pdo),
				 pdo_max_current(pdo));

		if (pdo_type(pdo) == PDO_TYPE_APDO) {
			/* TODO: check APDO_TYPE_PPS */
			pps_data->default_pps_pdo = pdo;
			return 0;
		}
	}

	return -ENODATA;
}

static int pps_init_node(struct pd_pps_data *pps_data,
			 struct device_node *node)
{
	int ret, nr_snk_pdo;

	memset(pps_data, 0, sizeof(*pps_data));

	nr_snk_pdo = pps_get_sink_pdo(pps_data->snk_pdo, PDO_MAX_OBJECTS, node);
	if (nr_snk_pdo < 0) {
		pr_err("Couldn't read sink-pdos, ret %d\n", nr_snk_pdo);
		return -ENOENT;
	}

	ret = pps_find_apdo(pps_data, nr_snk_pdo);
	if (ret < 0) {
		pr_err("nr_sink_pdo=%d sink APDO not found ret=%d\n",
			nr_snk_pdo, ret);
		return -ENOENT;
	}

	/*
	 * The port needs to ping or update the PPS adapter every 10 seconds
	 * (maximum). However, Qualcomm PD phy returns error when system is
	 * waking up. To prevent the timeout when system is resumed from
	 * suspend, hold a wakelock while PPS is active.
	 *
	 * Remove this wakeup source once we fix the Qualcomm PD phy issue.
	 */
	pps_data->stay_awake = of_property_read_bool(node, "google,pps-awake");
	if (pps_data->stay_awake)
		wakeup_source_init(&pps_data->pps_ws, "google-pps");

	pps_data->log = debugfs_logbuffer_register("pps");
	if (IS_ERR(pps_data->log))
		pps_data->log = NULL;

	pps_data->nr_snk_pdo = nr_snk_pdo;

	return 0;
}

/* Look for the connector and retrieve source capabilities.
 * pps_data->nr_snk_pdo == 0 means no PPS
 */
int pps_init(struct pd_pps_data *pps_data, struct device *dev)
{
	return pps_init_node(pps_data, dev->of_node);
}
EXPORT_SYMBOL_GPL(pps_init);

/* ------------------------------------------------------------------------- */

/*
 * This is the first part of the DC/PPS charging state machine.
 * Detect and configure the PPS adapter for the profile.
 *
 * returns:
 * . the max update interval pps should vote for
 * . 0 to disable the PPS update interval voter
 * . <0 for error
 */
int pps_work(struct pd_pps_data *pps, struct power_supply *tcpm_psy)
{
	int pd_online, usbc_type;
	unsigned int stage;

	if (!tcpm_psy)
		return -ENODEV;

	/*
	 * 2) pps->pd_online == TCPM_PSY_PROG_ONLINE && stage == PPS_NONE
	 *  If the source really support PPS (set in 1): set stage to
	 *  PPS_AVAILABLE and reschedule after PD_T_PPS_TIMEOUT
	 */
	if (pps->pd_online == TCPM_PSY_PROG_ONLINE && pps->stage == PPS_NONE) {
		int rc;

		pps->stage = pps_is_avail(pps, tcpm_psy);
		if (pps->stage == PPS_AVAILABLE) {
			rc = pps_ping(pps, tcpm_psy);
			if (rc < 0) {
				pps->pd_online = TCPM_PSY_FIXED_ONLINE;
				return 0;
			}

			if (pps->stay_awake)
				__pm_stay_awake(&pps->pps_ws);

			pps->last_update = get_boot_sec();
			rc = pps_get_src_cap(pps, tcpm_psy);
			if (rc < 0)
				pps_log(pps, "Cannot get partner src caps");
		}

		return PD_T_PPS_TIMEOUT;
	}

	/*
	 * no need to retry (error) when I cannot read POWER_SUPPLY_PROP_ONLINE.
	 * The prop is set to TCPM_PSY_PROG_ONLINE (from TCPM_PSY_FIXED_ONLINE)
	 * when usbc_type is POWER_SUPPLY_USB_TYPE_PD_PPS.
	 */
	pd_online = GPSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_ONLINE);
	if (pd_online < 0)
		return 0;

	/*
	 * 3) pd_online == TCPM_PSY_PROG_ONLINE == pps->pd_online
	 * pps is active now, we are done here. pd_online will change to
	 * if pd_online is !TCPM_PSY_PROG_ONLINE go back to 1) OR exit.
	 */
	stage = (pd_online == pps->pd_online) &&
		     (pd_online == TCPM_PSY_PROG_ONLINE) &&
		     (pps->stage == PPS_AVAILABLE || pps->stage == PPS_ACTIVE) ?
		     PPS_ACTIVE : PPS_NONE;
	if (stage != pps->stage) {
		pps_log(pps, "work: pd_online %d->%d stage %d->%d", __func__,
			pps->pd_online, pd_online,
			pps->stage, stage);
		pps->stage = stage;
	}

	if (pps->stage == PPS_ACTIVE)
		return 0;

	/*
	 * 1) stage == PPS_NONE && pps->pd_online!=TCPM_PSY_PROG_ONLINE
	 *  If usbc_type is POWER_SUPPLY_USB_TYPE_PD_PPS and pd_online is
	 *  TCPM_PSY_FIXED_ONLINE, enable PSPS (set POWER_SUPPLY_PROP_ONLINE to
	 *  TCPM_PSY_PROG_ONLINE and reschedule in PD_T_PPS_TIMEOUT.
	 */
	usbc_type = GPSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_USB_TYPE);
	if (pd_online == TCPM_PSY_FIXED_ONLINE &&
	    usbc_type == POWER_SUPPLY_USB_TYPE_PD_PPS) {
		int rc;

		rc = pps_prog_online(pps, tcpm_psy);
		switch (rc) {
		case 0:
			return PD_T_PPS_TIMEOUT;
		case -EAGAIN:
			pps_log(pps, "work: not in SNK_READY, rerun");
			return rc;

		case -EOPNOTSUPP:
			pps->stage = PPS_NOTSUPP;
			if (pps->stay_awake)
				__pm_relax(&pps->pps_ws);

			pps_log(pps, "work: PPS not supported for this TA");
			break;
		default:
			pps_log(pps, "work: PROP_ONLINE (%d)", rc);
			break;
		}

		return 0;
	}

	pps->pd_online = pd_online;
	return 0;
}

int pps_keep_alive(struct pd_pps_data *pps, struct power_supply *tcpm_psy)
{
	int ret;

	if (!tcpm_psy)
		return -ENODEV;

	ret = pps_ping(pps, tcpm_psy);
	if (ret < 0) {
		pps->pd_online = TCPM_PSY_FIXED_ONLINE;
		pps->keep_alive_cnt = 0;
		return ret;
	}

	pps->keep_alive_cnt += (pps->keep_alive_cnt < UINT_MAX);
	pps->last_update = get_boot_sec();
	return 0;
}

int pps_check_adapter(struct pd_pps_data *pps,
		      int pending_uv, int pending_ua,
		      struct power_supply *tcpm_psy)
{
	const int scale = 50000; /* HACK */
	int out_uv, op_ua;

	out_uv = GPSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	op_ua = GPSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_CURRENT_NOW);

	pr_debug("%s: mv=%d->%d ua=%d,%d\n", __func__,
		out_uv, pending_uv, op_ua, pending_ua);

	if (out_uv < 0 || op_ua < 0)
		return -EIO;

	return (out_uv / scale) >= (pending_uv /scale) &&
	       (op_ua / scale) >= (pending_ua / scale);
}

static int pps_set_prop(struct pd_pps_data *pps,
			enum power_supply_property prop, int val,
			struct power_supply *tcpm_psy)
{
	int ret;

	ret = GPSY_SET_PROP(tcpm_psy, prop, val);
	if (ret == 0) {
		pps->keep_alive_cnt = 0;
	} else if (ret == -EOPNOTSUPP) {
		pps->pd_online = TCPM_PSY_FIXED_ONLINE;
		pps->keep_alive_cnt = 0;
		if (pps->stay_awake)
			__pm_relax(&pps->pps_ws);
	}

	return ret;
}

/*
 * return negative values on errors
 * return PD_T_PPS_TIMEOUT after successful updates or pings
 * return PPS_UPDATE_DELAY_MS when the update interval is less than
 *	  PPS_UPDATE_DELAY_MS
 * return the delta to the nex ping deadline otherwise
 */
int pps_update_adapter(struct pd_pps_data *pps,
		       int pending_uv, int pending_ua,
		       struct power_supply *tcpm_psy)
{
	int interval = get_boot_sec() - pps->last_update;
	int ret;

	pps->out_uv = GPSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	pps->op_ua = GPSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	if (pps->out_uv < 0 || pps->op_ua < 0)
		return -EIO;

	if (pending_uv < 0)
		pending_uv = pps->out_uv;
	if (pending_ua < 0)
		pending_ua = pps->op_ua;

	pr_debug("%s: mv=%d->%d ua=%d->%d interval=%d\n", __func__,
		pps->out_uv, pending_uv, pps->op_ua, pending_ua, interval);

	/*
	 * TCPM accepts one change per power negotiation cycle.
	 * TODO: change voltage, current or current, voltage depending on
	 *       the values.
	 */
	if (pps->op_ua != pending_ua) {
		if (interval * 1000 < PPS_UPDATE_DELAY_MS)
			return PPS_UPDATE_DELAY_MS;

		ret = pps_set_prop(pps, POWER_SUPPLY_PROP_CURRENT_NOW,
				   pending_ua, tcpm_psy);
		pr_debug("%s: SET_UA out_ua %d->%d, ret=%d", __func__,
			pps->op_ua, pending_ua, ret);

		pps_log(pps, "SET_UA out_ua %d->%d, ret=%d",
			pps->op_ua, pending_ua, ret);

		if (ret == 0) {
			pps->last_update = get_boot_sec();
			pps->op_ua = pending_ua;
			return PD_T_PPS_TIMEOUT;
		}

		if (ret != -EAGAIN && ret != -EOPNOTSUPP)
			pps_log(pps, "failed to set CURRENT_NOW, ret = %d",
				ret);
	} else if (pps->out_uv != pending_uv) {
		if (interval * 1000 < PPS_UPDATE_DELAY_MS)
			return PPS_UPDATE_DELAY_MS;

		ret = pps_set_prop(pps, POWER_SUPPLY_PROP_VOLTAGE_NOW,
				   pending_uv,  tcpm_psy);
		pr_debug("%s: SET_UV out_v %d->%d, ret=%d\n", __func__,
			pps->out_uv, pending_uv, ret);

		pps_log(pps, "SET_UV out_v %d->%d, ret=%d",
			pps->out_uv, pending_uv, ret);

		if (ret == 0) {
			pps->last_update = get_boot_sec();
			pps->out_uv = pending_uv;
			return PD_T_PPS_TIMEOUT;
		}

		if (ret != -EAGAIN && ret != -EOPNOTSUPP)
			pps_log(pps, "failed to set CURRENT_NOW, ret = %d",
				ret);

	} else if (interval < PD_T_PPS_DEADLINE_S) {
		/* TODO: tune this, now assume that PD_T_PPS_TIMEOUT >= 7s */
		return PD_T_PPS_TIMEOUT - (interval * MSEC_PER_SEC);
	} else {
		ret = pps_keep_alive(pps, tcpm_psy);

		pr_debug("%s: KEEP ALIVE out_v %d, op_c %d (%d)", __func__,
			pps->out_uv, pps->op_ua, ret);
		pps_log(pps, "KEEP ALIVE out_v %d, op_c %d (%d)",
			pps->out_uv, pps->op_ua, ret);

		if (ret == 0)
			return PD_T_PPS_TIMEOUT;
	}

	if (ret == -EOPNOTSUPP)
		pps_log(pps, "PPS deactivated while updating");

	return ret;
}

struct power_supply *pps_get_tcpm_psy(struct device_node *node, size_t size)
{
	const char *propname = "google,tcpm-power-supply";
	struct power_supply *tcpm_psy = NULL;
	struct power_supply *psy[size];
	int i, ret;

	ret = power_supply_get_by_phandle_array(node, propname, psy,
						ARRAY_SIZE(psy));
	if (ret < 0)
		return ERR_PTR(-EAGAIN);

	for (i = 0; i < ret; i++) {
		const char *name = psy[i]->desc ? psy[i]->desc->name : NULL;

		if (!tcpm_psy && name && !strncmp(name, "tcpm", 4))
			tcpm_psy = psy[i];
		else
			power_supply_put(psy[i]);
	}

	return tcpm_psy;
}
EXPORT_SYMBOL_GPL(pps_get_tcpm_psy);

/* TODO:  */
int pps_request_pdo(struct pd_pps_data *pps_data, unsigned int ta_idx,
		    unsigned int ta_max_vol, unsigned int ta_max_cur,
		    struct power_supply *tcpm_psy)
{
	struct tcpm_port *port = chg_get_tcpm_port(tcpm_psy);
	const unsigned int max_mw = ta_max_vol * ta_max_cur;

	if (!port)
		return -ENODEV;
	if (ta_idx > PDO_MAX_OBJECTS)
		return -EINVAL;

	/* max_mw was, now using the max OP_SNK_MW */
	return tcpm_update_sink_capabilities(port, pps_data->snk_pdo,
					     ta_idx, max_mw);
}
EXPORT_SYMBOL_GPL(pps_request_pdo);


/* ------------------------------------------------------------------------- */

/* max APDO power from the TCPM source */
int pps_get_apdo_max_power(struct pd_pps_data *pps_data, unsigned int *ta_idx,
			   unsigned int *ta_max_vol, unsigned int *ta_max_cur,
			   unsigned long *ta_max_pwr)
{
	int max_current, max_voltage, max_power;
	const int ta_max_vol_mv = *ta_max_vol / 1000;
	int i;

	if (pps_data->nr_src_cap <= 0)
		return -ENOENT;

	/* already done that */
	if (*ta_idx != 0)
		return -ENOTSUPP;

	/* find the new max_uv, max_ua, and max_pwr */
	for (i = 0; i < pps_data->nr_src_cap; i++) {
		const u32 pdo = pps_data->src_caps[i];

		if (pdo_type(pdo) != PDO_TYPE_FIXED)
			continue;

		max_voltage = pdo_max_voltage(pdo); /* mV */
		max_current = pdo_max_current(pdo); /* mA */
		max_power = pdo_max_power(pdo); /* mW */
		*ta_max_pwr = max_power * 1000; /* uW */
	}

	/* Get the TA  maximum current and voltage for APDOs */
	for (i = 0; i < pps_data->nr_src_cap; i++) {
		const u32 pdo = pps_data->src_caps[i];

		if (pdo_type(pdo) != PDO_TYPE_APDO)
			continue;

		max_current = pdo_pps_apdo_max_current(pdo); /* mA */
		max_voltage = pdo_pps_apdo_max_voltage(pdo); /* mV */
		/* stop on first */
		if (max_voltage > ta_max_vol_mv) {
			*ta_max_vol = max_voltage * 1000;	/* uV */
			*ta_max_cur = max_current * 1000;	/* uA */
			*ta_idx = i + 1;
			return 0;
		}
	}

	pr_debug("%s: max_uv (%u) and max_ua (%u) out of APDO src caps\n",
		 __func__, *ta_max_vol, *ta_max_cur);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(pps_get_apdo_max_power);