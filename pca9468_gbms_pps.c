/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCA9468 Direct Charger PPS Integration
 *
 * Copyright (C) 2021 Google, LLC
 *
 */


#include <linux/err.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include "pca9468_regs.h"
#include "pca9468_charger.h"

/* DC PPS integration ------------------------------------------------------ */

static struct device_node *pca9468_find_config(struct device_node * node)
{
	struct device_node *temp;

	if (!node)
		return node;
	temp = of_parse_phandle(node, "pca9468,google_cpm", 0);
	if (temp)
		node = temp;
	return node;
}

int pca9468_probe_pps(struct pca9468_charger *pca9468_chg)
{
	const char *tmp_name = NULL;
	bool pps_available = false;
	struct device_node *node;
	int ret;

	node = pca9468_find_config(pca9468_chg->dev->of_node);
	if (!node)
		return -ENODEV;

	ret = of_property_read_u32(node, "google,tcpm-power-supply",
				   &pca9468_chg->tcpm_phandle);
	if (ret < 0)
		pr_warn("pca9468: pca,tcpm-power-supply not defined\n");
	else
		pps_available |= true;

	ret = of_property_read_string(node, "google,wlc_dc-power-supply",
				      &tmp_name);
	if (ret < 0)
		pr_warn("pca9468: google,wlc_dc-power-supply not defined\n");
	if (ret == 0) {
		pca9468_chg->wlc_psy_name =
			devm_kstrdup(pca9468_chg->dev, tmp_name, GFP_KERNEL);
		if (!pca9468_chg->wlc_psy_name)
			return -ENOMEM;

		pps_available |= true;
	}

	return pps_available ? 0 : -ENODEV;
}

/* Configure GCPM not needed */
int pca9468_set_switching_charger(bool enable,
					 unsigned int input_current,
					 unsigned int charging_current,
					 unsigned int vfloat)
{
	int ret = 0;

	/* handled in GCPM */

	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* force OFF, handled in GCPM */
int pca9468_get_swc_property(enum power_supply_property prop,
			     union power_supply_propval *val)
{
	int ret = 0;


	switch (prop) {
	case GBMS_PROP_CHARGING_ENABLED:
		val->intval = 0;
		break;
	default:
		pr_err("%s: cannot set prop %d\n", __func__, prop);
		break;
	}

	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* ------------------------------------------------------------------------ */

/* switch PDO if needed */
int pca9468_request_pdo(struct pca9468_charger *pca9468)
{
	int ret = 0;

	pr_debug("%s: ta_objpos=%u, ta_vol=%u, ta_cur=%u\n", __func__,
		  pca9468->ta_objpos, pca9468->ta_vol, pca9468->ta_cur);

	/*
	 * the reference implementation call pps_request_pdo() twice with a
	 * 100 ms delay between the calls when the function returns -EBUSY:
	 *
	 * 	ret = pps_request_pdo(&pca9468->pps_data, pca9468->ta_objpos,
	 *				pca9468->ta_vol, pca9468->ta_cur,
	 * 				pca9468->pd);
	 *
	 * The wrapper in google_dc_pps route the calls to the tcpm engine
	 * via tcpm_update_sink_capabilities(). The sync capabilities are
	 * in pps_data, ->ta_objpos select the (A)PDO index, ->ta_vol and
	 * ->ta_cur are the desired TA voltage and current.
	 *
	 * this is now handled by pps_update_adapter()
	 *
	 * TODO: verify the timing and make sure that there are no races that
	 * cause the targets
	 */

	return ret;
}

int pca9468_usbpd_setup(struct pca9468_charger *pca9468)
{
	struct power_supply *tcpm_psy;
	bool online;
	int ret = 0;

	if (pca9468->pd != NULL)
		goto check_online;

	if (pca9468->tcpm_psy_name) {
		tcpm_psy = power_supply_get_by_name(pca9468->tcpm_psy_name);
		if (!tcpm_psy)
			return -ENODEV;

		pca9468->pd = tcpm_psy;
	} else if (pca9468->tcpm_phandle) {
		struct device_node *node;

		node = pca9468_find_config(pca9468->dev->of_node);
		if (!node)
			return -ENODEV;
		tcpm_psy = pps_get_tcpm_psy(node, 2);
		if (IS_ERR(tcpm_psy))
			return PTR_ERR(tcpm_psy);
		if (!tcpm_psy) {
			pca9468->tcpm_phandle = 0;
			return -ENODEV;
		}

		pca9468->tcpm_psy_name = tcpm_psy->desc->name;
		pca9468->pd = tcpm_psy;
	} else {
		pr_err("%s: TCPM DC not defined\n", __func__);
		return -ENODEV;
	}

	/* not needed if tcpm-power-supply is not there */
	ret = pps_init(&pca9468->pps_data, pca9468->dev, tcpm_psy);
	if (ret == 0) {
		const char *logname = "pca9468_tcpm";
		int tmp;

		tmp = pps_register_logbuffer(&pca9468->pps_data, logname);
		pr_info("pca9468: PPS direct available, log:%s\n",
			 tmp < 0 ? "none" : logname);

		pps_init_state(&pca9468->pps_data);
	}

check_online:
	online = pps_prog_check_online(&pca9468->pps_data, pca9468->pd);
	if (!online)
		return -ENODEV;

	return ret;
}

int pca9468_send_pd_message(struct pca9468_charger *pca9468,
				   unsigned int msg_type)
{
	struct pd_pps_data *pps_data = &pca9468->pps_data;
	struct power_supply *tcpm_psy = pca9468->pd;
	bool online;
	int pps_ui;
	int ret;

	mutex_lock(&pca9468->lock);

	if (!tcpm_psy || (pca9468->charging_state == DC_STATE_NO_CHARGING &&
	    msg_type == PD_MSG_REQUEST_APDO) || !pca9468->mains_online) {
		pr_debug("%s: failure tcpm_psy_ok=%d charging_state=%u online=%d",
			__func__,  tcpm_psy != 0, pca9468->charging_state,
			pca9468->mains_online);
		mutex_unlock(&pca9468->lock);
		return -EINVAL;
	}

	/* false when offline (0) or not in prog (1) mode */
	online = pps_prog_check_online(&pca9468->pps_data, tcpm_psy);
	if (!online) {
		pr_debug("%s: not online", __func__);
		mutex_unlock(&pca9468->lock);
		return -EINVAL;
	}

	/* request offline */
	if (msg_type == PD_MSG_REQUEST_FIXED_PDO) {
		ret = pps_prog_offline(&pca9468->pps_data, tcpm_psy);
		pr_debug("%s: requesting offline ret=%d\n", __func__, ret);
		/* TODO: reset state? */
		mutex_unlock(&pca9468->lock);
		return ret;
	}

	pr_debug("%s: tcpm_psy_ok=%d pd_online=%d pps_stage=%d charging_state=%u",
		__func__,  tcpm_psy != 0,  pps_data->pd_online,
		pps_data->stage, pca9468->charging_state);

	if (pca9468->pps_data.stage == PPS_ACTIVE) {

		/* not sure I need to do this */
		ret = pca9468_request_pdo(pca9468);
		if (ret == 0) {
			const int pre_out_uv = pps_data->out_uv;
			const int pre_out_ua = pps_data->op_ua;

			pr_debug("%s: ta_vol=%u, ta_cur=%u, ta_objpos=%u\n",
				__func__, pca9468->ta_vol, pca9468->ta_cur,
				pca9468->ta_objpos);

			pps_ui = pps_update_adapter(&pca9468->pps_data,
						    pca9468->ta_vol,
						    pca9468->ta_cur,
						    tcpm_psy);
			pr_debug("%s: out_uv=%d %d->%d, out_ua=%d %d->%d (%d)\n",
				 __func__,
				 pps_data->out_uv, pre_out_uv, pca9468->ta_vol,
				 pps_data->op_ua, pre_out_ua, pca9468->ta_cur,
				 pps_ui);

			if (pps_ui == 0)
				pps_ui = PCA9468_PDMSG_WAIT_T;
			if (pps_ui < 0)
				pps_ui = PCA9468_PDMSG_RETRY_T;
		} else {
			pr_debug("%s: request_pdo failed ret=%d\n",
				 __func__, ret);
			pps_ui = PCA9468_PDMSG_RETRY_T;
		}

	} else {
		ret = pps_keep_alive(pps_data, tcpm_psy);
		if (ret == 0)
			pps_ui = PD_T_PPS_TIMEOUT;

		pr_debug("%s: keep alive ret=%d\n", __func__, ret);
	}

	if (((pca9468->charging_state == DC_STATE_NO_CHARGING) &&
		(msg_type == PD_MSG_REQUEST_APDO)) ||
		(pca9468->mains_online == false)) {

		/*
		 *  Vbus reset might occour even when PD comms is successful.
		 * Check again.
		 */
		pps_ui = -EINVAL;
	}

	/* PPS_Work: will reschedule */
	pr_debug("%s: pps_ui = %d\n", __func__, pps_ui);
	if (pps_ui > 0)
		schedule_delayed_work(&pca9468->pps_work, msecs_to_jiffies(pps_ui));

	mutex_unlock(&pca9468->lock);
	return pps_ui;
}

/* Get the max current/voltage/power of APDO from the CC/PD driver */
/* This function needs some modification by a customer */
int pca9468_get_apdo_max_power(struct pca9468_charger *pca9468)
{
	int ret = 0;

	/* check the phandle */
	ret = pca9468_usbpd_setup(pca9468);
	if (ret != 0) {
		dev_err(pca9468->dev, "cannot find TCPM %d\n", ret);
		pca9468->pd = NULL;
		return ret;
	}

	/* technically already in pda_data since check online does it */
	ret = pps_get_src_cap(&pca9468->pps_data, pca9468->pd);
	if (ret < 0)
		return ret;

	ret = pps_get_apdo_max_power(&pca9468->pps_data, &pca9468->ta_objpos,
				     &pca9468->ta_max_vol, &pca9468->ta_max_cur,
				     &pca9468->ta_max_pwr);
	if (ret < 0) {
		pr_err("cannot determine the apdo max power ret = %d\n", ret);
		return ret;
	}

	pr_debug("%s APDO pos=%u max_v=%u max_c=%u max_pwr=%lu\n", __func__,
		 pca9468->ta_objpos, pca9468->ta_max_vol, pca9468->ta_max_cur,
		 pca9468->ta_max_pwr);

	return 0;
}

/******************/
/* Set RX voltage */
/******************/

struct power_supply *pca9468_get_rx_psy(struct pca9468_charger *pca9468)
{
	const char *wlc_psy_name = pca9468->wlc_psy_name;

	if (!wlc_psy_name) {
		dev_err(pca9468->dev, "%s: WLC PS name not defined\n", __func__);
		wlc_psy_name = "wireless";
	}

	if (!pca9468->wlc_psy) {
		struct power_supply *psy;

		/* Get power supply name */
		psy = power_supply_get_by_name(wlc_psy_name);
		if (!psy) {
			dev_err(pca9468->dev, "%s Cannot find %s power supply\n",
				__func__, wlc_psy_name);
			return NULL;
		}

		pca9468->wlc_psy = psy;
	}

	return pca9468->wlc_psy;
}

/* Send RX voltage to RX IC */
/* This function needs some modification by a customer */
int pca9468_send_rx_voltage(struct pca9468_charger *pca9468,
				   unsigned int msg_type)
{
	union power_supply_propval pro_val;
	struct power_supply *wlc_psy;
	int ret = -EINVAL;

	mutex_lock(&pca9468->lock);

	/* Vbus reset happened in the previous PD communication */
	if (pca9468->mains_online == false)
		goto out;

	wlc_psy = pca9468_get_rx_psy(pca9468);
	if (!wlc_psy) {
		dev_err(pca9468->dev, "Cannot find wireless power supply\n");
		ret = -ENODEV;
		goto out;
	}

	pro_val.intval = pca9468->ta_vol;
	ret = power_supply_set_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
					&pro_val);
	pr_debug("%s: rx_vol=%d ret=%d\n", __func__, pca9468->ta_vol, ret);
	if (ret < 0)
		dev_err(pca9468->dev, "Cannot set RX voltage to %d (%d)\n",
			pro_val.intval, ret);

	/* Vbus reset might happen, check the charging state again */
	if (pca9468->mains_online == false) {
		pr_warn("%s: mains offline\n", __func__);
		ret = -EINVAL;
	}

out:
	mutex_unlock(&pca9468->lock);
	return ret;
}

/************************/
/* Get RX max power    */
/************************/

/* Get the max current/voltage/power of RXIC from the WCRX driver */
/* TODO: just ask the ->pd since GCPM will take care of this */
int pca9468_get_rx_max_power(struct pca9468_charger *pca9468)
{
	union power_supply_propval pro_val;
	struct power_supply *wlc_psy;
	int ret = 0;

	wlc_psy = pca9468_get_rx_psy(pca9468);
	if (!wlc_psy)
		return -ENODEV;

	/* Get the maximum voltage */
	ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX,
					&pro_val);
	if (ret < 0) {
		dev_err(pca9468->dev, "%s Cannot get the maximum RX voltage (%d)\n",
			__func__, ret);
		return ret;
	}

	/* RX IC cannot support the request maximum voltage */
	if (pca9468->ta_max_vol > pro_val.intval) {
		dev_err(pca9468->dev, "%s max %d cannot support ta_max %d voltage\n",
			__func__, pro_val.intval, pca9468->ta_max_vol);
		return -EINVAL;
	}

	pca9468->ta_max_vol = pro_val.intval;

	/* Get the maximum current */
	ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_CURRENT_MAX,
					&pro_val);
	if (ret < 0) {
		dev_err(pca9468->dev, "%s Cannot get the maximum RX current (%d)\n",
			__func__, ret);
		return ret;
	}

	pca9468->ta_max_cur = pro_val.intval;
	pca9468->ta_max_pwr = (pca9468->ta_max_vol / 1000) *
			      (pca9468->ta_max_cur / 1000);

	dev_info(pca9468->dev, "%s wlc max_cur=%d max_pwr=%ld\n",
		 __func__, pca9468->ta_max_cur, pca9468->ta_max_pwr);

	return 0;
}

static int pca9468_get_ta_type(struct pca9468_charger *pca9468)
{
	int ta_type = TA_TYPE_UNKNOWN;

	if (pca9468->pps_index == PPS_INDEX_TCPM) {
		int ret;

		ret = pca9468_usbpd_setup(pca9468);
		if (ret != 0) {
			dev_err(pca9468->dev, "Cannot find the TA %d\n", ret);
			return ret;
		}

		ta_type = TA_TYPE_USBPD;
	} else if (pca9468->pps_index == PPS_INDEX_WLC) {
		ta_type = TA_TYPE_WIRELESS;
	}

	return ta_type;
}

int pca9468_set_ta_type(struct pca9468_charger *pca9468)
{
	pca9468->ta_type = pca9468_get_ta_type(pca9468);
	if (pca9468->ta_type == TA_TYPE_UNKNOWN)
		return -EINVAL;

	if (pca9468->ta_type == PPS_INDEX_TCPM)
		pca9468->pdata->chg_mode = CHG_2TO1_DC_MODE;
	else if (pca9468->ta_type == PPS_INDEX_WLC)
		pca9468->pdata->chg_mode = CHG_4TO1_DC_MODE;

	return 0;
}


/* GBMS integration ------------------------------------------------------ */

int pca9468_get_charge_type(struct pca9468_charger *pca9468)
{
	int ret, sts;

	if (!pca9468->mains_online)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	/*
	 * HW will reports PCA9468_BIT_IIN_LOOP_STS (CC) or
	 * PCA9468_BIT_VFLT_LOOP_STS (CV) or inactive (i.e. openloop).
	 */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_A, &sts);
	if (ret < 0)
		return ret;

	pr_debug("%s: sts_a=%0x2 VFLT=%d IIN=%d charging_state=%d\n",
		__func__, sts, !!(sts & PCA9468_BIT_VFLT_LOOP_STS),
		 !!(sts & PCA9468_BIT_IIN_LOOP_STS),
		 pca9468->charging_state);

	/* Use SW state for now */
	switch (pca9468->charging_state) {
	case DC_STATE_ADJUST_CC:
	case DC_STATE_CC_MODE:
	case DC_STATE_ADJUST_TAVOL:
	case DC_STATE_ADJUST_TACUR:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case DC_STATE_START_CV:
	case DC_STATE_CV_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT;
	case DC_STATE_CHECK_ACTIVE: /* in preset */
	case DC_STATE_CHARGING_DONE:
		break;
	}

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

#define PCA9468_NOT_CHARGING \
	(PCA9468_BIT_SHUTDOWN_STATE_STS | PCA9468_BIT_STANDBY_STATE_STS)
#define PCA9468_ANY_CHARGING_LOOP \
	(PCA9468_BIT_CHG_LOOP_STS | PCA9468_BIT_IIN_LOOP_STS | \
	PCA9468_BIT_VFLT_LOOP_STS)

int pca9468_get_status(struct pca9468_charger *pca9468)
{
	u8 val[8];
	int ret;

	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1_STS,
			       &val[PCA9468_REG_INT1_STS], 5);
	if (ret < 0) {
		pr_debug("%s: ioerr=%d", __func__, ret);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	pr_debug("%s: int1_sts=0x%x,sts_a=0x%x,sts_b=0x%x,sts_c=0x%x,sts_d=0x%x\n",
		 __func__,  val[3], val[4], val[5], val[6], val[7]);

	if ((val[PCA9468_REG_STS_B] & PCA9468_BIT_ACTIVE_STATE_STS) == 0 ||
	    (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_V_OK_STS) == 0) {
		const bool online = pca9468->mains_online;

		/* no disconnect during charger transition */
		return online ? POWER_SUPPLY_STATUS_NOT_CHARGING :
		       POWER_SUPPLY_STATUS_DISCHARGING;
	}

	/* Use SW state (for now) */
	switch (pca9468->charging_state) {
	case DC_STATE_NO_CHARGING:
	case DC_STATE_CHECK_VBAT:
	case DC_STATE_PRESET_DC:
	case DC_STATE_CHECK_ACTIVE:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case DC_STATE_ADJUST_CC:
	case DC_STATE_CC_MODE:
	case DC_STATE_START_CV:
	case DC_STATE_CV_MODE:
		return POWER_SUPPLY_STATUS_CHARGING;
	case DC_STATE_CHARGING_DONE:
		return POWER_SUPPLY_STATUS_FULL;
	default:
		break;
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

#define PCA9468_PRESENT_MASK \
	(PCA9468_BIT_ACTIVE_STATE_STS | PCA9468_BIT_STANDBY_STATE_STS)

int pca9468_is_present(struct pca9468_charger *pca9468)
{
	int sts = 0;

	regmap_read(pca9468->regmap, PCA9468_REG_STS_B, &sts);
	return !!(sts & PCA9468_PRESENT_MASK);
}

int pca9468_get_chg_chgr_state(struct pca9468_charger *pca9468,
				      union gbms_charger_state *chg_state)
{
	int vchrg;

	chg_state->v = 0;
	chg_state->f.chg_status = pca9468_get_status(pca9468);
	chg_state->f.chg_type = pca9468_get_charge_type(pca9468);
	chg_state->f.flags = gbms_gen_chg_flags(chg_state->f.chg_status,
						chg_state->f.chg_type);

	vchrg = pca9468_read_adc(pca9468, ADCCH_VBAT);
	if (vchrg > 0)
		chg_state->f.vchrg = vchrg / 1000;

	if (chg_state->f.chg_status != POWER_SUPPLY_STATUS_DISCHARGING) {
		int rc;

		rc = pca9468_input_current_limit(pca9468);
		if (rc > 0)
			chg_state->f.icl = rc / 1000;
	}

	return 0;
}
