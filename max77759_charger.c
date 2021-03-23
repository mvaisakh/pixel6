/* SPDX-License-Identifier: GPL-2.0 */
/*
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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/ctype.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/thermal.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/pmic_class.h>
#include <misc/gvotable.h>
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "max_m5.h"
#include "max77759.h"

#define VD_BATTERY_VOLTAGE 4200
#define VD_UPPER_LIMIT 3350
#define VD_LOWER_LIMIT 2600
#define VD_STEP 50
#define VD_DELAY 300
#define THERMAL_IRQ_COUNTER_LIMIT 5
#define THERMAL_HYST_LEVEL 100

enum PMIC_VDROOP_SENSOR {
	VDROOP1,
	VDROOP2,
	VDROOP_MAX,
};

#define MAX77759_DEFAULT_MODE	MAX77759_CHGR_MODE_ALL_OFF

/* CHG_DETAILS_01:CHG_DTLS */
#define CHGR_DTLS_DEAD_BATTERY_MODE			0x00
#define CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE	0x01
#define CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE	0x02
#define CHGR_DTLS_TOP_OFF_MODE				0x03
#define CHGR_DTLS_DONE_MODE				0x04
#define CHGR_DTLS_TIMER_FAULT_MODE			0x06
#define CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE		0x07
#define CHGR_DTLS_OFF_MODE				0x08
#define CHGR_DTLS_OFF_HIGH_TEMP_MODE			0x0a
#define CHGR_DTLS_OFF_WATCHDOG_MODE			0x0b
#define CHGR_DTLS_OFF_JEITA				0x0c
#define CHGR_DTLS_OFF_TEMP				0x0d

struct uvilo_stats {
	ktime_t _time;
	int capacity;
	int voltage;
};

struct max77759_chgr_data {
	struct device *dev;
	struct device *mdev;

	struct power_supply *psy;
	struct power_supply *wcin_psy;
	struct power_supply *chgin_psy;

	struct power_supply *wlc_psy;
	struct regmap *regmap;

	struct gvotable_election *mode_votable;
	struct max77759_usecase_data uc_data;

	struct gvotable_election *dc_icl_votable;
	struct gvotable_election *dc_suspend_votable;

	bool chgin_input_suspend;
	bool wcin_input_suspend;

	int irq_gpio;

	struct i2c_client *fg_i2c_client;
	struct i2c_client *pmic_i2c_client;

	struct dentry *de;
	atomic_t sysuvlo1_cnt;
	atomic_t sysuvlo2_cnt;
	atomic_t batoilo_cnt;
	struct uvilo_stats sysuvlo1;
	struct uvilo_stats sysuvlo2;
	struct uvilo_stats batoilo;

	atomic_t insel_cnt;
	bool insel_clear;	/* when set, irq clears CHGINSEL_MASK */

	struct mutex io_lock;
	bool resume_complete;
	bool init_complete;

	int fship_dtls;
	bool online;
	bool wden;

	/* debug interface, register to read or write */
	u32 debug_reg_address;

	/* thermal */
	struct thermal_zone_device *tz_vdroop[VDROOP_MAX];
	int vdroop_counter[VDROOP_MAX];
	unsigned int vdroop_lvl[VDROOP_MAX];
	unsigned int vdroop_irq[VDROOP_MAX];
	struct mutex vdroop_irq_lock[VDROOP_MAX];
	struct delayed_work vdroop_irq_work[VDROOP_MAX];

	int chg_term_voltage;
};

static inline int max77759_reg_read(struct regmap *regmap, uint8_t reg,
				    uint8_t *val)
{
	int ret, ival;

	ret = regmap_read(regmap, reg, &ival);
	if (ret == 0)
		*val = 0xFF & ival;

	return ret;
}

static inline int max77759_reg_write(struct regmap *regmap, uint8_t reg,
				     uint8_t val)
{
	return regmap_write(regmap, reg, val);
}

static inline int max77759_readn(struct regmap *regmap, uint8_t reg,
				 uint8_t *val, int count)
{
	return regmap_bulk_read(regmap, reg, val, count);
}

static inline int max77759_writen(struct regmap *regmap, uint8_t reg,
				  const uint8_t *val, int count)
{
	return regmap_bulk_write(regmap, reg, val, count);
}

static inline int max77759_reg_update(struct max77759_chgr_data *data,
				      uint8_t reg, uint8_t msk, uint8_t val)
{
	int ret;
	unsigned tmp;

	mutex_lock(&data->io_lock);
	ret = regmap_read(data->regmap, reg, &tmp);
	if (!ret) {
		tmp &= ~msk;
		tmp |= val;
		ret = regmap_write(data->regmap, reg, tmp);
	}
	mutex_unlock(&data->io_lock);

	return ret;
}

/* ----------------------------------------------------------------------- */

int max77759_chg_reg_write(struct i2c_client *client, u8 reg, u8 value)
{
	struct max77759_chgr_data *data = i2c_get_clientdata(client);

	if (!data || !data->regmap)
		return -ENODEV;

	return max77759_reg_write(data->regmap, reg, value);
}
EXPORT_SYMBOL_GPL(max77759_chg_reg_write);

int max77759_chg_reg_update(struct i2c_client *client,
			    u8 reg, u8 mask, u8 value)
{
	struct max77759_chgr_data *data = i2c_get_clientdata(client);

	if (!data || !data->regmap)
		return -ENODEV;

	return regmap_write_bits(data->regmap, reg, mask, value);
}
EXPORT_SYMBOL_GPL(max77759_chg_reg_update);

int max77759_chg_mode_write(struct i2c_client *client,
			    enum max77759_charger_modes mode)
{
	struct max77759_chgr_data *data = i2c_get_clientdata(client);

	if (!data || !data->regmap)
		return -ENODEV;

	return regmap_write_bits(data->regmap, MAX77759_CHG_CNFG_00,
				 MAX77759_CHG_CNFG_00_MODE_MASK,
				 mode);
}
EXPORT_SYMBOL_GPL(max77759_chg_mode_write);

/* ----------------------------------------------------------------------- */

static int max77759_find_pmic(struct max77759_chgr_data *data)
{
	struct device_node *dn;

	if (data->pmic_i2c_client)
		return 0;

	dn = of_parse_phandle(data->dev->of_node, "max77759,pmic", 0);
	if (!dn)
		return -ENXIO;

	data->pmic_i2c_client = of_find_i2c_device_by_node(dn);
	if (!data->pmic_i2c_client)
		return -EAGAIN;

	return 0;
}

static int max77759_find_fg(struct max77759_chgr_data *data)
{
	struct device_node *dn;

	if (data->fg_i2c_client)
		return 0;

	dn = of_parse_phandle(data->dev->of_node, "max77759,max_m5", 0);
	if (!dn)
		return -ENXIO;

	data->fg_i2c_client = of_find_i2c_device_by_node(dn);
	if (!data->fg_i2c_client)
		return -EAGAIN;

	return 0;
}

static int max77759_read_vbatt(struct max77759_chgr_data *data, int *vbatt)
{
	int ret;

	ret = max77759_find_fg(data);
	if (ret == 0)
		ret = max1720x_get_voltage_now(data->fg_i2c_client, vbatt);

	return ret;
}

/* ----------------------------------------------------------------------- */

/* set WDTEN in CHG_CNFG_18 (0xCB), tWD = 80s */
static int max77759_wdt_enable(struct max77759_chgr_data *data, bool enable)
{
	int ret;
	u8 reg;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &reg);
	if (ret < 0)
		return -EIO;

	if ((!!_chg_cnfg_18_wdten_get(reg)) == enable)
		return 0;

	/* this register is protected, read back to check if it worked */
	reg = _chg_cnfg_18_wdten_set(reg, enable);
	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_18, reg);
	if (ret < 0)
		return -EIO;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &reg);
	if (ret < 0)
		return -EIO;

	return (ret == 0 && (!!_chg_cnfg_18_wdten_get(reg)) == enable) ?
		0 : -EINVAL;
}

/* First step to convert votes to a usecase and a setting for mode */
static int max77759_foreach_callback(void *data, const char *reason,
				     void *vote)
{
	struct max77759_foreach_cb_data *cb_data = data;
	int mode = (long)vote; /* max77759_mode is an int election */

	switch (mode) {
	/* Direct raw modes last come fist served */
	case MAX77759_CHGR_MODE_ALL_OFF:
	case MAX77759_CHGR_MODE_BUCK_ON:
	case MAX77759_CHGR_MODE_CHGR_BUCK_ON:
	case MAX77759_CHGR_MODE_BOOST_UNO_ON:
	case MAX77759_CHGR_MODE_BOOST_ON:
	case MAX77759_CHGR_MODE_OTG_BOOST_ON:
	case MAX77759_CHGR_MODE_BUCK_BOOST_UNO_ON:
	case MAX77759_CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON:
	case MAX77759_CHGR_MODE_OTG_BUCK_BOOST_ON:
	case MAX77759_CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON:
		if (cb_data->use_raw)
			break;
		pr_debug("%s: RAW vote=0x%x\n", __func__, mode);
		cb_data->raw_value = mode;
		cb_data->reason = reason;
		cb_data->use_raw = true;
		break;

	/* temporary, can be used to program the WLC chip, remove */
	case GBMS_CHGR_MODE_BOOST_UNO_ON:
		if (!cb_data->boost_on || !cb_data->uno_on)
			cb_data->reason = reason;
		pr_debug("%s: BOOST_UNO vote=0x%x\n", __func__, mode);
		cb_data->boost_on += 1;
		cb_data->uno_on += 1;
		break;

	/* SYSTEM modes can add complex transactions */

	/* MAX77759: on disconnect */
	case GBMS_CHGR_MODE_STBY_ON:
		if (!cb_data->stby_on)
			cb_data->reason = reason;
		pr_debug("%s: STBY_ON %s vote=0x%x\n",
			 __func__, reason ? reason : "<>", mode);
		cb_data->stby_on += 1;
		break;
	case GBMS_CHGR_MODE_INFLOW_OFF:
		if (!cb_data->inflow_off)
			cb_data->reason = reason;
		pr_debug("%s: INFLOW_OFF vote=0x%x\n", __func__, mode);
		cb_data->inflow_off += 1;
		break;
	/* MAX77759: charging on via CC_MAX (needs inflow, buck_on on) */
	case GBMS_CHGR_MODE_CHGR_BUCK_ON:
		if (!cb_data->chgr_on)
			cb_data->reason = reason;
		pr_debug("%s: CHGR_BUCK_ON %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->chgr_on += 1;
		break;

	/* USB: inflow, actual charging controlled via BUCK_ON */
	case GBMS_USB_BUCK_ON:
		if (!cb_data->buck_on)
			cb_data->reason = reason;
		pr_debug("%s: BUCK_ON %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->buck_on += 1;
		break;
	/* USB: OTG, source, fast role swap case */
	case GBMS_USB_OTG_FRS_ON:
		if (!cb_data->frs_on)
			cb_data->reason = reason;
		pr_debug("%s: FRS_ON vote=0x%x\n", __func__, mode);
		cb_data->frs_on += 1;
		break;
	/* USB: boost mode, source, normally external boost */
	case GBMS_USB_OTG_ON:
		if (!cb_data->otg_on)
			cb_data->reason = reason;
		pr_debug("%s: OTG_ON %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->otg_on += 1;
		break;
	/* DC Charging: mode=0, set CP_EN */
	case GBMS_CHGR_MODE_CHGR_DC:
		if (!cb_data->dc_on)
			cb_data->reason = reason;
		pr_debug("%s: DC_ON vote=0x%x\n", __func__, mode);
		cb_data->dc_on += 1;
		break;
	/* WLC Tx */
	case GBMS_CHGR_MODE_WLC_TX:
		if (!cb_data->wlc_tx)
			cb_data->reason = reason;
		pr_debug("%s: WLC_TX vote=%x\n", __func__, mode);
		cb_data->wlc_tx += 1;
		break;

	default:
		pr_err("mode=%x not supported\n", mode);
		break;
	}

	return 0;
}

/* control VENDOR_EXTBST_CTRL (from TCPCI module) */
static int max77759_ls_mode(struct max77759_usecase_data *uc_data, int mode)
{
	int ret;

	pr_debug("%s: mode=%d ext_bst_ctl=%d lsw1_ok=%d\n", __func__, mode,
		uc_data->ext_bst_ctl, !!uc_data->lsw1_is_closed +
		!!uc_data->lsw1_is_open);

	if (uc_data->ext_bst_ctl < 0)
		return 0;
	if (uc_data->lsw1_is_open < 0 || uc_data->lsw1_is_closed < 0)
		return 0;

	/* VENDOR_EXTBST_CTRL control LSW1, the read will check the state */
	gpio_set_value_cansleep(uc_data->ext_bst_ctl, mode);

	/* ret <= 0 if *_is* is not true and > 1 if true */
	switch (mode) {
	case 0:
		/* the OVP open right away */
		ret = gpio_get_value_cansleep(uc_data->lsw1_is_open);
		break;
	case 1:
		/* it takes 11 ms to turn on the OVP */
		msleep(11);
		ret = gpio_get_value_cansleep(uc_data->lsw1_is_closed);
		break;
	default:
		return -EINVAL;
	}

	return (ret <= 0) ? -EIO : 0;
}

/* OVP LS2 */
static int max77759_ls2_mode(struct max77759_usecase_data *uc_data, int mode)
{
	pr_debug("%s: ls2_en=%d mode=%d\n", __func__, uc_data->ls2_en, mode);

	if (uc_data->ls2_en >= 0)
		gpio_set_value_cansleep(uc_data->ls2_en, !!mode);

	return 0;
}

static bool max77759_is_vin_valid(struct max77759_usecase_data *uc_data)
{

	if (uc_data->vin_is_valid < 0) {
		pr_err("%s: vin-valid GPIO not set\n", __func__);
		return false;
	}

	return gpio_get_value_cansleep(uc_data->vin_is_valid) == 1;
}

/* control external boost mode
 * can be done controlling ls1, ls2
 */
#define EXT_MODE_OFF		0
#define EXT_MODE_OTG_5_0V	1
#define EXT_MODE_OTG_7_5V	2

/*
 * bst_on=GPIO5 on Max77759 on canopy and on all whitefins,
 * bst_sel=Granville
 */
static int max77759_ext_mode(struct max77759_usecase_data *uc_data, int mode)
{
	int ret = 0;

	pr_debug("%s: mode=%d on=%d sel=%d\n", __func__, mode,
		 uc_data->bst_on, uc_data->bst_sel);

	if (uc_data->bst_on < 0 || uc_data->bst_sel < 0)
		return 0;

	switch (mode) {
	case EXT_MODE_OFF:
		gpio_set_value_cansleep(uc_data->bst_on, 0);
		break;
	case EXT_MODE_OTG_5_0V:
		gpio_set_value_cansleep(uc_data->bst_sel, 0);
		mdelay(100);
		gpio_set_value_cansleep(uc_data->bst_on, 1);
		break;
	case EXT_MODE_OTG_7_5V: /* TODO: verify this */
		gpio_set_value_cansleep(uc_data->bst_sel, 1);
		mdelay(100);
		gpio_set_value_cansleep(uc_data->bst_on, 1);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

/* RTX reverse wireless charging */
static int gs101_wlc_tx_enable(struct max77759_usecase_data *uc_data,
			       bool enable)
{
	int ret;

	if (enable) {

		ret = max77759_ls2_mode(uc_data, 1);
		if (ret == 0)
			ret = max77759_ext_mode(uc_data, EXT_MODE_OTG_7_5V);

		mdelay(100);

		if (uc_data->cpout21_en >= 0)
			gpio_set_value_cansleep(uc_data->cpout21_en, 0);
	} else {
		/* NOTE: turn off WLC, no need to reset cpout */
		ret = max77759_ext_mode(uc_data, EXT_MODE_OFF);
		if (ret == 0)
			ret = max77759_ls2_mode(uc_data, 0);

	}

	return ret;
}

/*
 * Transition to standby (if needed) at the beginning of the sequences
 * @return <0 on error, 0 on success. ->use_case becomes GSU_MODE_STANDBY
 * if the transition is necessary (and successful).
 */
static int max77759_to_standby(struct max77759_usecase_data *uc_data,
			       int use_case)
{
	const int from_uc = uc_data->use_case;
	bool need_stby = false;
	int ret;

	switch (from_uc) {
		case GSU_MODE_USB_CHG:
			need_stby = use_case != GSU_MODE_USB_CHG_WLC_TX &&
				    use_case != GSU_MODE_WLC_RX &&
				    use_case != GSU_MODE_USB_DC &&
				    use_case != GSU_MODE_USB_OTG_FRS;
			break;
		case GSU_MODE_WLC_RX:
			need_stby = use_case != GSU_MODE_USB_OTG_WLC_RX &&
				    use_case != GSU_MODE_WLC_DC;
			break;
		case GSU_MODE_WLC_TX:
			need_stby = use_case != GSU_MODE_USB_OTG_WLC_TX &&
				    use_case != GSU_MODE_USB_CHG_WLC_TX &&
				    use_case != GSU_MODE_USB_DC_WLC_TX;
			break;
		case GSU_MODE_USB_CHG_WLC_TX:
			need_stby = use_case != GSU_MODE_USB_CHG;
			break;

		case GSU_MODE_USB_OTG:

			if (use_case == GSU_MODE_USB_OTG_FRS)
				break;
			if (use_case == GSU_MODE_USB_OTG_WLC_TX)
				break;
			if (use_case == GSU_MODE_USB_OTG_WLC_RX)
				break;
			if (use_case == GSU_MODE_USB_OTG_WLC_DC)
				break;

			/* From 5. USB OTG to anything else, go to stby */
			ret = max77759_ls_mode(uc_data, 0);
			if (ret == 0)
				ret = max77759_ext_mode(uc_data, 0);
			if (ret < 0)
				return -EIO;

			/* TODO:Discharge IN/OUT with AO37 is done in TCPM */
			mdelay(100);

			need_stby = true;
			break;

		case GSU_MODE_USB_OTG_WLC_RX:
			need_stby = use_case != GSU_MODE_WLC_RX;
			break;

		case GSU_MODE_USB_OTG_FRS:
			if (use_case == GSU_MODE_USB_OTG || use_case == GSU_MODE_USB_OTG_FRS)
				break;
			need_stby = true;
			break;
		case GSU_RAW_MODE:
			need_stby = true;
			break;
		case GSU_MODE_USB_OTG_WLC_TX:
		case GSU_MODE_STANDBY:
		default:
			need_stby = false;
			break;
	}

	pr_debug("%s: use_case=%d->%d need_stby=%x\n", __func__,
		 from_uc, use_case, need_stby);

	if (!need_stby)
		return 0;

	/* from WLC_TX to STBY */
	if (from_uc == GSU_MODE_WLC_TX) {
		ret = gs101_wlc_tx_enable(uc_data, false);
		if (ret < 0)
			return ret;
	}

	/* transition to STBY (might need to be up) */
	ret = max77759_chg_mode_write(uc_data->client, MAX77759_CHGR_MODE_ALL_OFF);
	if (ret < 0)
		return -EIO;

	uc_data->use_case = GSU_MODE_STANDBY;
	return ret;
}

/* enable/disable soft-start. No need soft start from OTG->OTG_FRS */
static int max7759_ramp_bypass(struct max77759_usecase_data *uc_data, bool enable)
{
	const u8 value = enable ? MAX77759_CHG_CNFG_00_BYPV_RAMP_BYPASS : 0;

	if (uc_data->is_a1 <= 0)
		return 0;

	return max77759_chg_reg_update(uc_data->client, MAX77759_CHG_CNFG_00,
				       MAX77759_CHG_CNFG_00_BYPV_RAMP_BYPASS,
				       value);
}

/* cleanup from every usecase */
static int max77759_force_standby(struct max77759_chgr_data *data)
{
	struct max77759_usecase_data *uc_data = &data->uc_data;
	const u8 insel_mask = MAX77759_CHG_CNFG_12_CHGINSEL_MASK |
			      MAX77759_CHG_CNFG_12_WCINSEL_MASK;
	const u8 insel_value = MAX77759_CHG_CNFG_12_CHGINSEL |
			       MAX77759_CHG_CNFG_12_WCINSEL;
	int ret;

	ret = max77759_ls_mode(uc_data, 0);
	if (ret < 0)
		dev_err(data->dev, "%s: cannot change ls_mode (%d)\n",
			__func__, ret);

	ret = max77759_ext_mode(uc_data, 0);
	if (ret < 0)
		dev_err(data->dev, "%s: cannot change ext mode (%d)\n",
			__func__, ret);

	ret = max7759_ramp_bypass(uc_data, false);
	if (ret < 0)
		dev_err(data->dev, "%s: cannot reset ramp_bypass (%d)\n",
			__func__, ret);

	ret = max77759_chg_mode_write(uc_data->client, MAX77759_CHGR_MODE_ALL_OFF);
	if (ret < 0)
		dev_err(data->dev, "%s: cannot reset mode register (%d)\n",
			__func__, ret);

	ret = max77759_chg_reg_update(uc_data->client, MAX77759_CHG_CNFG_12,
				      insel_mask, insel_value);
	if (ret < 0)
		dev_err(data->dev, "%s: cannot reset insel (%d)\n",
			__func__, ret);

	return 0;
}

/* From OTG <-> OTG_FRS */
static int gs101_otg_mode(struct max77759_usecase_data *uc_data, int to)
{
	int ret = -EINVAL;

	if (to == GSU_MODE_USB_OTG) {

		ret = max77759_ext_mode(uc_data, EXT_MODE_OTG_5_0V);
		if (ret < 0)
			return ret;

		mdelay(5);

		ret = max77759_chg_mode_write(uc_data->client,
					      MAX77759_CHGR_MODE_ALL_OFF);

	} else if (to == GSU_MODE_USB_OTG_FRS) {
		int rc;

		ret = max7759_ramp_bypass(uc_data, true);
		if (ret == 0)
			ret = max77759_chg_mode_write(uc_data->client,
						      MAX77759_CHGR_MODE_OTG_BOOST_ON);
		if (ret < 0)
			return ret;

		mdelay(5);

		rc =  max7759_ramp_bypass(uc_data, false);
		if (rc < 0)
			pr_err("%s: cannot clear bypass rc:%d\n",  __func__, rc);

		ret = max77759_ext_mode(uc_data, EXT_MODE_OFF);
		if (ret < 0)
			return ret;

	}

	return ret;
}

/* Badhri/Chao workaround */
static int max7759_otg_enable(struct max77759_usecase_data *uc_data, int mode)
{
	int ret;

	ret = max77759_ls_mode(uc_data, 1);
	if (ret < 0) {
		pr_debug("%s: cannot close load switch (%d)\n", __func__, ret);
		return ret;
	}

	ret = max77759_chg_mode_write(uc_data->client, MAX77759_CHGR_MODE_OTG_BOOST_ON);
	if (ret < 0) {
		pr_debug("%s: cannot set CNFG_00 to 0xa ret:%d\n",  __func__, ret);
		return ret;
	}

	if (!max77759_is_vin_valid(uc_data)) {
		pr_debug("%s: VIN not VALID\n",  __func__);
		return -EIO;
	}

	ret = max77759_ext_mode(uc_data, mode);
	if (ret < 0)
		pr_debug("%s: cannot change extmode ret:%d\n",  __func__, ret);

	return ret;
}


/* configure ilim wlctx */
static int gs101_wlctx_otg_en(struct max77759_usecase_data *uc_data, bool enable)
{
	int ret;

	if (enable) {
		if (uc_data->sw_en > 0)
			gpio_set_value_cansleep(uc_data->sw_en, 1);

		ret = max77759_chg_reg_update(uc_data->client, MAX77759_CHG_CNFG_05,
					MAX77759_CHG_CNFG_05_OTG_ILIM_MASK,
					uc_data->otg_ilim);
		if (ret < 0)
			return ret;

		ret = max77759_chg_reg_update(uc_data->client, MAX77759_CHG_CNFG_18,
					MAX77759_CHG_CNFG_18_OTG_V_PGM,
					enable ? MAX77759_CHG_CNFG_18_OTG_V_PGM : 0);
		if (ret < 0)
			return ret;

		ret = max77759_chg_reg_write(uc_data->client, MAX77759_CHG_CNFG_11,
					uc_data->otg_vbyp);
	} else {
		/* TODO: Discharge IN/OUT nodes with AO37 should be done in TCPM */

		mdelay(100);

		if (uc_data->sw_en > 0)
			gpio_set_value_cansleep(uc_data->sw_en, 0);

		ret = 0;
		/* TODO: restore initial value on MAX77759_CHG_CNFG_05 */
		/* TODO: restore initial value on !MAX77759_CHG_CNFG_18 */
		/* TODO: restore initial value on !MAX77759_CHG_CNFG_11 */
	}


	return ret;
}

/* change p9412 CPOUT and adjust WCIN_REG */
#define GS101_WLCRX_CPOUT_DFLT	0
#define GS101_WLCRX_CPOUT_5_2V	1

static int gs101_cpout_mode(struct max77759_usecase_data *uc_data, int mode)
{
	int ret;

	/* do not change MW unless p9412 can be changed as well */
	if (uc_data->cpout_ctl < 0)
		return 0;

	if (mode == GS101_WLCRX_CPOUT_5_2V) {
		/* p9412: set CPOUT==5.2 only if on BPP */
		gpio_set_value_cansleep(uc_data->cpout_ctl, 1);

		/* NOTE: no DC_IN to MW when WCIN_REG==4_85 unless CPOUT==5.2 */
		ret =  max77759_chg_reg_update(uc_data->client, MAX77759_CHG_CNFG_12,
					       MAX77759_CHG_CNFG_12_WCIN_REG_MASK,
					       MAX77759_CHG_CNFG_12_WCIN_REG_4_85);
	} else {
		/* p9412: reset CPOUT to default */
		gpio_set_value_cansleep(uc_data->cpout_ctl, 0);

		ret =  max77759_chg_reg_update(uc_data->client, MAX77759_CHG_CNFG_12,
					       MAX77759_CHG_CNFG_12_WCIN_REG_MASK,
					       MAX77759_CHG_CNFG_12_WCIN_REG_4_5);
	}

	return ret;
}

/*
 * Case	USB_chg USB_otg	WLC_chg	WLC_TX	PMIC_Charger	Ext_B	LSxx	Name
 * -------------------------------------------------------------------------------------
 * 4-1	0	1	10	0	IF-PMIC-WCIN	1	1/0	USB_OTG_WLC_RX
 * 4-2	0	1	01	0	DC WCIN		1	1/0	USB_OTG_WLC_DC
 * 5-1	0	1	0	0	0		1	1/0	USB_OTG
 * 5-2	0	1	0	0	OTG_5V		0	0/0	USB_OTG_FRS
 * 7-2	0	1	0	1	OTG_5V		2	0/1	USB_OTG_WLC_TX
 * -------------------------------------------------------------------------------------
 * WLC_chg = 0 off, 1 = on, 2 = PPS
 * Ext_Boost = 0 off, 1 = OTG 5V, 2 = WTX 7.5
 *
 * 5-1: mode=0x0 in MW, EXT_B=1, LS1=1, LS2=0, IDLE <-> OTG (ext)
 * 5-2: mode=0xa in MW, EXT_B=0, LS1=0, LS2=0, IDLE <-> OTG_FRS
 * 7-2: mode=0xa in MW, EXT_B=2, LS1=0, LS2=1
 *
 * AO37 + GPIO5 MW (canopy 3, whitev2p2)
 * . AO_ls1 <&max20339_gpio 0 GPIO_ACTIVE_HIGH> - bit0
 * . AO_ls2 <&max20339_gpio 1 GPIO_ACTIVE_HIGH> - bit4
 *
 * ls1 can be controlled poking the AO37 OR using a MW_GPIO _> EXT_BST_EN
 *
 * max77759,bst_on = <&max777x9_gpio 4 GPIO_ACTIVE_HIGH>
 * max77759,bst-sel = <&gpp27 3 GPIO_ACTIVE_HIGH>
 * max77759,bst_on=0, max77759,bst_sel=x => OFF
 * max77759,bst_on=1, max77759,bst_sel=0 => 5V
 * max77759,bst_on=1, max77759,bst_sel=1 => 7.5V
 *
 * Ext_Boost = 0 off
 * 	MW_gpio5	: Ext_B = 0, MW_gpio5 -> LOW
 * 	AO_ls1/ls2	: 0/0
 *
 * Ext_Boost = 1 = OTG 5V
 * 	MW_gpio5	: Ext_B = 1, MW_gpio5 -> HIGH
 * 	AO_ls1/ls2	: 1/0
 *
 * Ext_Boost = 2 WTX 7.5
 * 	MW_gpio5	: Ext_B = 2, MW_gpio5 -> HIGH
 * 	AO_ls1/ls2	: 0/1
 *
 * NOTE: do not call with (cb_data->wlc_rx && cb_data->wlc_tx)
 */
static int max77759_to_otg_usecase(struct max77759_usecase_data *uc_data, int use_case)
{
	const int from_uc = uc_data->use_case;
	int ret = 0;

	if (!uc_data->init_done)
		return -EPROBE_DEFER;

	switch (from_uc) {
	/* 5-1: #3: stby to USB OTG, mode = 1 */
	/* 5-2: #3: stby to USB OTG_FRS, mode = 0 */
	case GSU_MODE_STANDBY: {
		const int mode = use_case == GSU_MODE_USB_OTG_FRS ?
					     EXT_MODE_OFF :
					     EXT_MODE_OTG_5_0V;

		/* Badhri/Chao workaround */
		ret = max7759_otg_enable(uc_data, mode);
		if (ret < 0)
			break;

		mdelay(5);

		/*
		 * Assumption: max77759_to_usecase() will write back cached values to
		 * CHG_CNFG_00.Mode. At the moment, the cached value at
		 * max77759_mode_callback is 0. If the cached value changes to someting
		 * other than 0, then, the code has to be revisited.
		 */
	} break;

	case GSU_MODE_WLC_TX:
		/* b/179820595: WLC_TX -> WLC_TX + OTG */
		if (use_case == GSU_MODE_USB_OTG_WLC_TX)
			ret = gs101_wlctx_otg_en(uc_data, true);
	break;

	case GSU_MODE_WLC_RX:
		/* b/179816224 WLC_RX -> WLC_RX + OTG (Transition #10) */
		if (use_case == GSU_MODE_USB_OTG_WLC_RX) {

			/* TODO: debounce the sequence */
			if (uc_data->cpout_en >= 0)
				gpio_set_value_cansleep(uc_data->cpout_en, 0);

			/* Badhri/Chao workaround */
			ret = max7759_otg_enable(uc_data, EXT_MODE_OTG_5_0V);
			if (ret == 0) {
				mdelay(5);

				ret = max77759_chg_reg_write(uc_data->client,
							     MAX77759_CHG_CNFG_00,
							     MAX77759_CHGR_MODE_ALL_OFF);
				if (ret == 0 && uc_data->ext_bst_mode >= 0)
					gpio_set_value_cansleep(uc_data->ext_bst_mode, 1);
				if (ret == 0)
					ret = gs101_cpout_mode(uc_data, GS101_WLCRX_CPOUT_5_2V);
			}

			if (ret < 0)
				break;

			if (uc_data->cpout_en >= 0)
				gpio_set_value_cansleep(uc_data->cpout_en, 1);

			/* get_otg_usecase() will set mode */
		}

	break;

	case GSU_MODE_USB_OTG:
		/* b/179820595: OTG -> WLC_TX + OTG (see b/181371696) */
		if (use_case == GSU_MODE_USB_OTG_WLC_TX) {
			ret = gs101_otg_mode(uc_data, GSU_MODE_USB_OTG_FRS);
			if (ret == 0)
				ret = gs101_wlc_tx_enable(uc_data, true);
		}
		/* b/179816224: OTG -> WLC_RX + OTG */
		if (use_case == GSU_MODE_USB_OTG_WLC_RX) {
			ret = gs101_cpout_mode(uc_data, GS101_WLCRX_CPOUT_5_2V);
			if (ret == 0 && uc_data->ext_bst_mode >= 0)
				gpio_set_value_cansleep(uc_data->ext_bst_mode, 1);
		}
	break;
	case GSU_MODE_USB_OTG_WLC_TX:
		/*  b/179820595: WLC_TX + OTG -> OTG */
		if (use_case == GSU_MODE_USB_OTG) {
			ret = gs101_wlc_tx_enable(uc_data, false);
			if (ret == 0)
				ret = gs101_otg_mode(uc_data, GSU_MODE_USB_OTG);
		}
	break;
	case GSU_MODE_USB_OTG_WLC_RX:
		/* b/179816224: WLC_RX + OTG -> OTG */
		if (use_case == GSU_MODE_USB_OTG) {
			/* it's in STBY, no need to reset gs101_otg_mode()  */
			if (uc_data->ext_bst_mode >= 0)
				gpio_set_value_cansleep(uc_data->ext_bst_mode, 0);
			ret = gs101_cpout_mode(uc_data, GS101_WLCRX_CPOUT_DFLT);
		}
	break;
	case GSU_MODE_USB_OTG_FRS: {
		/*
		 * OTG source handover: OTG_FRS -> OTG
		 * from EXT_BST (Regular OTG) to IF-PMIC OTG (FRS OTG)
		 */
	} break;

	default:
		return -ENOTSUPP;
	}

	return ret;
}

/* handles the transition data->use_case ==> use_case */
static int max77759_to_usecase(struct max77759_usecase_data *uc_data, int use_case)
{
	const int from_uc = uc_data->use_case;
	int ret = 0;

	switch (use_case) {
	case GSU_MODE_USB_OTG:
	case GSU_MODE_USB_OTG_FRS:
	case GSU_MODE_USB_OTG_WLC_RX:
	case GSU_MODE_USB_OTG_WLC_DC:
	case GSU_MODE_USB_OTG_WLC_TX:
		ret = max77759_to_otg_usecase(uc_data, use_case);
		if (ret < 0)
			return ret;
		break;
	case GSU_MODE_WLC_TX:
	case GSU_MODE_USB_CHG_WLC_TX:
		/* Coex Case #4, WLC_TX + OTG -> WLC_TX */
		if (from_uc == GSU_MODE_USB_OTG_WLC_TX) {
			ret = max77759_chg_mode_write(uc_data->client,
						      MAX77759_CHGR_MODE_ALL_OFF);
			if (ret == 0)
				ret = gs101_wlctx_otg_en(uc_data, false);
		} else {
			ret = gs101_wlc_tx_enable(uc_data, true);
		}

		break;
	case GSU_MODE_WLC_RX:
		if (from_uc == GSU_MODE_USB_OTG_WLC_RX) {
			/* to_stby brought to stby */
			if (uc_data->ext_bst_mode >= 0)
				gpio_set_value_cansleep(uc_data->ext_bst_mode, 0);
			ret = gs101_cpout_mode(uc_data, GS101_WLCRX_CPOUT_DFLT);
			if (ret == 0)
				ret = gs101_otg_mode(uc_data, GSU_MODE_USB_OTG);

		}
		break;
	case GSU_MODE_USB_CHG:
		if (from_uc == GSU_MODE_WLC_TX || from_uc == GSU_MODE_USB_CHG_WLC_TX) {
			ret = gs101_wlc_tx_enable(uc_data, false);
			if (ret < 0)
				return ret;
		}
		break;
	case GSU_RAW_MODE:
		/* just write the value to the register (it's in stby) */
		break;
	default:
		break;
	}

	return ret;
}

/*
 * Case	USB_chg USB_otg	WLC_chg	WLC_TX	PMIC_Charger	Ext_B	LSxx	Name
 * -------------------------------------------------------------------------------------
 * 4-1	0	1	10	0	IF-PMIC-WCIN	1	1/0	USB_OTG_WLC_RX
 * 4-2	0	1	01	0	DC WCIN		1	1/0	USB_OTG_WLC_DC
 * 5-1	0	1	0	0	0		1	1/0	USB_OTG
 * 5-2	0	1	0	0	OTG_5V		0	0/0	USB_OTG_FRS
 * 7-2	0	1	0	1	OTG_5V		2	0/1	USB_OTG_WLC_TX
 * -------------------------------------------------------------------------------------
 * Ext_Boost = 0 off, 1 = OTG 5V, 2 = WTX 7.5
 * WLC_chg = 0 off, 1 = on, 2 = PPS
 *
 * NOTE: do not call with (cb_data->wlc_rx && cb_data->wlc_tx)
 */
static int max77759_get_otg_usecase(struct max77759_foreach_cb_data *cb_data)
{
	int usecase;
	u8 mode;

	/* invalid, cannot do OTG stuff with USB power */
	if (cb_data->buck_on) {
		pr_err("%s: buck_on with OTG\n", __func__);
		return -EINVAL;
	}

	/* pure OTG defaults to ext boost */
	if (!cb_data->wlc_rx && !cb_data->wlc_tx) {
		/* 5-1: USB_OTG or  5-2: USB_OTG_FRS */

		if (cb_data->frs_on) {
			usecase = GSU_MODE_USB_OTG_FRS;
			mode = MAX77759_CHGR_MODE_OTG_BOOST_ON;
		} else {
			usecase = GSU_MODE_USB_OTG;
			mode = MAX77759_CHGR_MODE_ALL_OFF;
		}

		if (cb_data->dc_on)
			pr_err("%s: TODO enable pps+OTG\n", __func__);
		cb_data->dc_on = 0;
	} else if (cb_data->wlc_tx) {
		/* 7-2: WLC_TX -> WLC_TX + OTG */
		usecase = GSU_MODE_USB_OTG_WLC_TX;
		mode = MAX77759_CHGR_MODE_OTG_BOOST_ON;
	} else if (cb_data->wlc_rx) {
		usecase = GSU_MODE_USB_OTG_WLC_RX;
		if (cb_data->chgr_on)
			mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
		else
			mode = MAX77759_CHGR_MODE_BUCK_ON;

	} else if (cb_data->dc_on) {
		return -EINVAL;
	} else {
		return -EINVAL;
	}

	cb_data->reg = _chg_cnfg_00_cp_en_set(cb_data->reg, cb_data->dc_on);
	cb_data->reg = _chg_cnfg_00_mode_set(cb_data->reg, mode);
	return usecase;
}

/*
 * Determines the use case to switch to. This is device/system dependent and
 * will likely be factored to a separate file (compile module).
 */
static int max77759_get_usecase(struct max77759_foreach_cb_data *cb_data)
{
	const int buck_on = cb_data->inflow_off ? 0 : cb_data->buck_on;
	const int chgr_on = cb_data->stby_on ? 0 : cb_data->chgr_on;
	int wlc_tx = cb_data->wlc_tx;
	int usecase;
	u8 mode;

	/* consistency check, TOD: add more */
	if (wlc_tx) {
		if (cb_data->wlc_rx) {
			pr_err("%s: wlc_tx and wlc_rx\n", __func__);
			return -EINVAL;
		}

		if (cb_data->dc_on) {
			pr_warn("%s: no wlc_tx with dc_on for now\n", __func__);
			wlc_tx = 0;
		}
	}

	/* OTG modes override the others */
	if (cb_data->otg_on || cb_data->frs_on)
		return max77759_get_otg_usecase(cb_data);

	/* buck_on is wired, wlc_rx is wireless, might still need rTX */
	if (!buck_on && !cb_data->wlc_rx) {
		mode = MAX77759_CHGR_MODE_ALL_OFF;

		/* Rtx using the internal battery */
		usecase = GSU_MODE_STANDBY;
		if (wlc_tx)
			usecase = GSU_MODE_WLC_TX;

	} else if (buck_on && wlc_tx) {

		/* dc_on + wlc_tx handled up */
		usecase = GSU_MODE_USB_CHG_WLC_TX;
		mode = (chgr_on) ?
			MAX77759_CHGR_MODE_CHGR_BUCK_ON :
			MAX77759_CHGR_MODE_BUCK_ON;
	} else {

		/* MODE_BUCK_ON is inflow */
		if (chgr_on) {
			mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
			usecase = GSU_MODE_USB_CHG;
		} else {
			mode = MAX77759_CHGR_MODE_BUCK_ON;
			usecase = GSU_MODE_USB_CHG;
		}

		/*
		 * NOTE: OTG cases handled in max77759_get_otg_usecase()
		 * NOTE: usecases with !(buck|wlc)_on same as.
		 * NOTE: mode=0 if standby, mode=5 if charging, mode=0xa on otg
		 * TODO: handle rTx + DC and some more.
		 */
		if (cb_data->dc_on) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_USB_DC;
			// usecase = GSU_MODE_WLC_DC;
		} else if (wlc_tx) {
			/* buck_on, inflow_off or OTG */
			usecase = GSU_MODE_WLC_TX;
		} else if (cb_data->wlc_rx) {
			/* mode = _CHGR_BUCK_ON || _BUCK_ON*/
			usecase = GSU_MODE_WLC_RX;
		} else if (cb_data->stby_on && !cb_data->chgr_on) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_STANDBY;
		}

	}

	/* reg might be ignored later */
	cb_data->reg = _chg_cnfg_00_cp_en_set(cb_data->reg, cb_data->dc_on);
	cb_data->reg = _chg_cnfg_00_mode_set(cb_data->reg, mode);

	return usecase;
}

static int max77759_otg_ilim_ma_to_code(u8 *code, int otg_ilim)
{
	if (otg_ilim == 0)
		*code = 0;
	else if (otg_ilim >= 500 && otg_ilim <= 1500)
		*code = 1 + (otg_ilim - 500) / 100;
	else
		return -EINVAL;

	return 0;
}

static int max77759_otg_vbyp_mv_to_code(u8 *code, int vbyp)
{
	if (vbyp >= 12000)
		*code = 0x8c;
	else if (vbyp > 5000)
		*code = (vbyp - 5000) / 20;
	else
		return -EINVAL;

	return 0;
}

#define GS101_OTG_ILIM_DEFAULT_MA	1500
#define GS101_OTG_VBYPASS_DEFAULT_MV	5100

/* lazy init on the switched */
static bool max77759_setup_usecases(struct max77759_usecase_data *uc_data,
				    struct device_node *node)
{
	int ret;

	if (!node) {
		uc_data->init_done = false;
		uc_data->bst_on = -EPROBE_DEFER;
		uc_data->bst_sel = -EPROBE_DEFER;
		uc_data->ext_bst_ctl = -EPROBE_DEFER;
		uc_data->ls2_en = -EPROBE_DEFER;
		uc_data->lsw1_is_closed = -EPROBE_DEFER;
		uc_data->lsw1_is_open = -EPROBE_DEFER;
		uc_data->vin_is_valid = -EPROBE_DEFER;
		uc_data->cpout_ctl = -EPROBE_DEFER;
		uc_data->cpout_en = -EPROBE_DEFER;
		uc_data->cpout21_en = -EPROBE_DEFER;
		uc_data->sw_en = -EPROBE_DEFER;
		uc_data->is_a1 = -1;
		return 0;
	}

	if (uc_data->bst_on == -EPROBE_DEFER)
		uc_data->bst_on = of_get_named_gpio(node, "max77759,bst-on", 0);
	if (uc_data->bst_sel == -EPROBE_DEFER)
		uc_data->bst_sel = of_get_named_gpio(node, "max77759,bst-sel", 0);
	if (uc_data->ext_bst_ctl == -EPROBE_DEFER)
		uc_data->ext_bst_ctl = of_get_named_gpio(node, "max77759,extbst-ctl", 0);
	if (uc_data->ls2_en == -EPROBE_DEFER)
		uc_data->ls2_en = of_get_named_gpio(node, "max77759,ls2-en", 0);

	if (uc_data->lsw1_is_closed == -EPROBE_DEFER)
		uc_data->lsw1_is_closed = of_get_named_gpio(node, "max77759,lsw1-is_closed", 0);
	if (uc_data->lsw1_is_open == -EPROBE_DEFER)
		uc_data->lsw1_is_open = of_get_named_gpio(node, "max77759,lsw1-is_open", 0);

	if (uc_data->vin_is_valid == -EPROBE_DEFER)
		uc_data->vin_is_valid = of_get_named_gpio(node, "max77759,vin-is_valid", 0);

	/*  wlc_rx -> wlc_rx+otg disable cpout */
	if (uc_data->cpout_en == -EPROBE_DEFER)
		uc_data->cpout_en = of_get_named_gpio(node, "max77759,cpout-en", 0);
	/* ->wlc_tx disable 2:1 cpout */
	if (uc_data->cpout21_en == -EPROBE_DEFER)
		uc_data->cpout21_en = of_get_named_gpio(node, "max77759,cpout_21-en", 0);
	/* to 5.2V in p9412 */
	if (uc_data->cpout_ctl == -EPROBE_DEFER)
		uc_data->cpout_ctl = of_get_named_gpio(node, "max77759,cpout-ctl", 0);

	/* OPTIONAL: only in P1.1+ (TPS61372) */
	if (uc_data->ext_bst_mode == -EPROBE_DEFER)
		uc_data->ext_bst_mode = of_get_named_gpio(node, "max77759,extbst-mode", 0);

	/* OTG+RTXL: IN-OUT switch of AO37 (forced always) */
	if (uc_data->sw_en == -EPROBE_DEFER)
		uc_data->sw_en = of_get_named_gpio(node, "max77759,sw-en", 0);

	ret = max77759_otg_ilim_ma_to_code(&uc_data->otg_ilim, GS101_OTG_ILIM_DEFAULT_MA);
	if (ret < 0)
		uc_data->otg_ilim = MAX77759_CHG_CNFG_05_OTG_ILIM_1500MA;

	ret = max77759_otg_vbyp_mv_to_code(&uc_data->otg_vbyp, GS101_OTG_VBYPASS_DEFAULT_MV);
	if (ret < 0)
		uc_data->otg_vbyp = MAX77759_CHG_CNFG_11_OTG_VBYP_5100MV;

	return uc_data->bst_on != -EPROBE_DEFER &&
	       uc_data->bst_sel != -EPROBE_DEFER &&
	       uc_data->ext_bst_ctl != -EPROBE_DEFER &&
	       uc_data->ls2_en != -EPROBE_DEFER &&
	       uc_data->lsw1_is_closed != -EPROBE_DEFER &&
	       uc_data->lsw1_is_open != -EPROBE_DEFER &&
	       uc_data->vin_is_valid != -EPROBE_DEFER &&
		uc_data->cpout_ctl != -EPROBE_DEFER &&
		uc_data->cpout_en != -EPROBE_DEFER &&
		uc_data->cpout21_en != -EPROBE_DEFER;
}

/*
 * adjust *INSEL (only one source can be enabled at a given time)
 * TODO: should we mask the interrupts for CHGIN,WCIN as well?
 */
static int max77759_set_insel(struct max77759_usecase_data *uc_data,
			      struct max77759_foreach_cb_data *cb_data,
			      int use_case)
{
	const u8 insel_mask = MAX77759_CHG_CNFG_12_CHGINSEL_MASK |
			      MAX77759_CHG_CNFG_12_WCINSEL_MASK;
	u8 insel_value = 0;
	int ret;

	if (cb_data->buck_on) {
		insel_value |= MAX77759_CHG_CNFG_12_CHGINSEL;
	} else if (cb_data->wlc_rx) {
		insel_value |= MAX77759_CHG_CNFG_12_WCINSEL;
	} else if (cb_data->otg_on) {
		/* all OTG cases mask CHGIN */
		insel_value |= MAX77759_CHG_CNFG_12_WCINSEL;
	} else {
		insel_value |= MAX77759_CHG_CNFG_12_CHGINSEL;
		insel_value |= MAX77759_CHG_CNFG_12_WCINSEL;
	}

	ret = max77759_chg_reg_update(uc_data->client, MAX77759_CHG_CNFG_12,
				      insel_mask, insel_value);

	pr_debug("%s: usecase=%d mask=%x insel=%x (%d)\n", __func__,
		 use_case, insel_mask, insel_value, ret);

	return ret;
}

/* switch to a use case, handle the transitions */
static int max77759_set_usecase(struct max77759_chgr_data *data,
				struct max77759_foreach_cb_data *cb_data,
				int use_case)
{
	struct max77759_usecase_data *uc_data = &data->uc_data;
	const int from_uc = uc_data->use_case;
	int ret;

	if (uc_data->is_a1 == -1) {

		ret = max77759_find_pmic(data);
		if (ret == 0) {
			u8 id, rev;

			ret = max777x9_pmic_get_id(data->pmic_i2c_client, &id, &rev);
			if (ret == 0)
				uc_data->is_a1 = id == MAX77759_PMIC_PMIC_ID_MW &&
						 rev >= MAX77759_PMIC_REV_A0;
		}
	}

	/* Need this only for for usecases that control the switches */
	if (!uc_data->init_done) {
		uc_data->init_done = max77759_setup_usecases(uc_data, data->dev->of_node);

		dev_info(data->dev, "bst_on:%d, bst_sel:%d, ext_bst_ctl:%d lsw1_o:%d lsw1_c:%d\n",
			 uc_data->bst_on, uc_data->bst_sel, uc_data->ext_bst_ctl,
			 uc_data->lsw1_is_open, uc_data->lsw1_is_closed);
	}

	/* just set the mode register to handle chgr_on/off and inflow */
	if (from_uc == use_case)
		goto exit_done;

	ret = max77759_set_insel(uc_data, cb_data, use_case);
	if (ret < 0) {
		dev_err(data->dev, "use_case=%d->%d set_insel failed ret:%d\n",
			from_uc, use_case, ret);
		return ret;
	}

	/* transition to STBY if requested from the use case. */
	ret = max77759_to_standby(uc_data, use_case);
	if (ret < 0) {
		dev_err(data->dev, "use_case=%d->%d to_stby failed ret:%d\n",
			from_uc, use_case, ret);
		return ret;
	}

	/* transition from data->use_case to use_case */
	ret = max77759_to_usecase(uc_data, use_case);
	if (ret < 0) {
		dev_err(data->dev, "use_case=%d->%d to_usecase failed ret:%d\n",
			from_uc, use_case, ret);
		return ret;
	}

exit_done:

	/* finally set mode register */
	ret = max77759_chg_reg_write(uc_data->client, MAX77759_CHG_CNFG_00,
				     cb_data->reg);
	pr_debug("%s: CHARGER_MODE=%x reg:%x\n", __func__, cb_data->reg, ret);
	if (ret < 0) {
		dev_err(data->dev,  "use_case=%d->%d CNFG_00=%x failed ret:%d\n",
			from_uc, use_case, cb_data->reg, ret);
		return ret;
	}

	return ret;
}

static int max77759_wcin_is_online(struct max77759_chgr_data *data);

/*
 * I am using a the comparator_none, need scan all the votes to determine
 * the actual.
 */
static void max77759_mode_callback(struct gvotable_election *el,
				   const char *trigger, void *value)
{
	struct max77759_chgr_data *data = gvotable_get_data(el);
	struct max77759_foreach_cb_data cb_data = { 0 };
	const char *reason;
	int use_case, ret;
	bool nope;
	u8 reg;

	mutex_lock(&data->io_lock);

	reason = trigger;
	use_case = data->uc_data.use_case;

	/* no caching */
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_00, &reg);
	if (ret < 0) {
		dev_err(data->dev, "cannot read CNFG_00 (%d)\n", ret);
		goto unlock_done;
	}

	/* this is the last vote of the election */
	cb_data.reg = reg;	/* current */
	cb_data.el = el;	/* election */
	cb_data.wlc_rx = max77759_wcin_is_online(data);

	/* now scan all the reasons, accumulate in cb_data */
	gvotable_election_for_each(el, max77759_foreach_callback, &cb_data);
	nope = !cb_data.use_raw && !cb_data.stby_on && !cb_data.dc_on &&
	       !cb_data.chgr_on && !cb_data.buck_on && ! cb_data.boost_on &&
	       !cb_data.otg_on && !cb_data.uno_on && !cb_data.wlc_tx &&
	       !cb_data.wlc_rx;
	if (nope) {
		pr_debug("%s: nope callback\n", __func__);
		goto unlock_done;
	}

	dev_info(data->dev, "%s:%s raw=%d stby_on=%d, dc_on=%d, chgr_on=%d, buck_on=%d, "
		"boost_on=%d, otg_on=%d, uno_on=%d wlc_tx=%d wlc_rx=%d inflow=%d\n",
		__func__, trigger ? trigger : "<>",
		cb_data.use_raw, cb_data.stby_on, cb_data.dc_on,
		cb_data.chgr_on, cb_data.buck_on, cb_data.boost_on,
		cb_data.otg_on, cb_data.uno_on, cb_data.wlc_tx,
		cb_data.wlc_rx, !cb_data.inflow_off);

	/* just use raw "as is", no changes to switches etc */
	if (cb_data.use_raw) {
		cb_data.reg = cb_data.raw_value;
		use_case = GSU_RAW_MODE;
	} else {
		/* figure out next use case if not in raw mode */
		use_case = max77759_get_usecase(&cb_data);
		if (use_case < 0) {
			dev_err(data->dev, "no valid use case %d\n", use_case);
			goto unlock_done;
		}
	}

	/* state machine that handle transition between states */
	ret = max77759_set_usecase(data, &cb_data, use_case);
	if (ret < 0) {
		ret = max77759_force_standby(data);
		if (ret < 0) {
			dev_err(data->dev, "use_case=%d->%d to_stby failed ret:%d\n",
				data->uc_data.use_case, use_case, ret);
			goto unlock_done;
		}

		cb_data.reg = MAX77759_CHGR_MODE_ALL_OFF;
		cb_data.reason = "error";
		use_case = GSU_MODE_STANDBY;
	}

	/* the election is an int election */
	if (cb_data.reason)
		reason = cb_data.reason;
	if (!reason)
		reason = "<>";

	/* this changes the trigger */
	ret = gvotable_election_set_result(el, reason, (void*)(uintptr_t)cb_data.reg);
	if (ret < 0) {
		dev_err(data->dev, "cannot update election %d\n", ret);
		goto unlock_done;
	}

	/* mode */
	data->uc_data.use_case = use_case;

unlock_done:
	dev_info(data->dev, "%s:%s use_case=%d->%d CHG_CNFG_00=%x->%x\n",
		 __func__, trigger ? trigger : "<>",
		 data->uc_data.use_case, use_case,
		 reg, cb_data.reg);
	mutex_unlock(&data->io_lock);
}

static int max77759_get_charge_enabled(struct max77759_chgr_data *data,
				       int *enabled)
{
	int ret;
	const void *vote = (const void *)0;

	ret = gvotable_get_current_vote(data->mode_votable, &vote);
	if (ret < 0)
		return ret;

	switch ((uintptr_t)vote) {
	case MAX77759_CHGR_MODE_CHGR_BUCK_ON:
	case MAX77759_CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON:
	case MAX77759_CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON:
		*enabled = 1;
		break;
	default:
		*enabled = 0;
		break;
	}

	return ret;
}

/* called from gcpm and for CC_MAX == 0 */
static int max77759_set_charge_enabled(struct max77759_chgr_data *data,
				       int enabled, const char *reason)
{
	return gvotable_cast_vote(data->mode_votable, reason,
				  (void*)GBMS_CHGR_MODE_CHGR_BUCK_ON,
				  enabled);
}

/* google_charger on disconnect */
static int max77759_set_charge_disable(struct max77759_chgr_data *data,
				       int enabled, const char *reason)
{
	return gvotable_cast_vote(data->mode_votable, reason,
				  (void*)GBMS_CHGR_MODE_STBY_ON,
				  enabled);
}

/* google_charger on disconnect */
static int max77759_set_input_suspend(struct max77759_chgr_data *data,
				      int enabled, const char *reason)
{
	return gvotable_cast_vote(data->mode_votable, reason,
				  (void*)GBMS_CHGR_MODE_INFLOW_OFF,
				  enabled);
}

/* turn off CHGIN_INSEL: works when max77559 registers are not protected */
static int max77759_chgin_input_suspend(struct max77759_chgr_data *data,
					bool enabled, const char *reason)
{
	const u8 value = (!enabled) << MAX77759_CHG_CNFG_12_CHGINSEL_SHIFT;
	u8 reg_val;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_12, &reg_val);
	if (ret < 0 || ((reg_val & value) == value))
		return ret;

	ret = max77759_reg_update(data, MAX77759_CHG_CNFG_12,
				  MAX77759_CHG_CNFG_12_CHGINSEL_MASK,
				  value);
	if (ret == 0)
		ret = max77759_set_input_suspend(data, enabled, "CHGIN_SUSP");
	if (ret == 0)
		data->chgin_input_suspend = enabled; /* cache */

	return ret;
}

/* turn off WCIN_INSEL: works when max77559 registers are not protected */
static int max77759_wcin_input_suspend(struct max77759_chgr_data *data,
				       bool enabled, const char *reason)
{
	const u8 value = (!enabled) << MAX77759_CHG_CNFG_12_WCINSEL_SHIFT;
	u8 reg_val;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_12, &reg_val);
	if (ret < 0 || ((reg_val & value) == value))
		return ret;

	ret =  max77759_reg_update(data, MAX77759_CHG_CNFG_12,
				   MAX77759_CHG_CNFG_12_WCINSEL_MASK,
				   value);
	if (ret == 0)
		ret = max77759_set_input_suspend(data, enabled, "WCIN_SUSP");
	if (ret == 0)
		data->wcin_input_suspend = enabled; /* cache */

	return ret;
}

static int max77759_set_regulation_voltage(struct max77759_chgr_data *data,
					   int voltage_uv)
{
	u8 value;

	if (voltage_uv >= 4500000)
		value = 0x32;
	else if (voltage_uv < 4000000)
		value = 0x38 + (voltage_uv - 3800000) / 100000;
	else
		value = (voltage_uv - 4000000) / 10000;

	value = VALUE2FIELD(MAX77759_CHG_CNFG_04_CHG_CV_PRM, value);
	return max77759_reg_update(data, MAX77759_CHG_CNFG_04,
				   MAX77759_CHG_CNFG_04_CHG_CV_PRM_MASK,
				   value);
}

static int max77759_get_regulation_voltage_uv(struct max77759_chgr_data *data,
					      int *voltage_uv)
{
	u8 value;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_04, &value);
	if (ret < 0)
		return ret;

	if  (value < 0x33)
		*voltage_uv = (4000 + value * 10) * 1000;
	else if (value == 0x38)
		*voltage_uv = 3800 * 1000;
	else if (value == 0x39)
		*voltage_uv = 3900 * 1000;
	else
		return -EINVAL;

	return 0;
}

/* set charging current to 0 to disable charging (MODE=0) */
static int max77759_set_charger_current_max_ua(struct max77759_chgr_data *data,
					       int current_ua)
{
	const int disabled = current_ua == 0;
	u8 value;
	int ret;

	if (current_ua < 0)
		return 0;

	/* ilim=0 -> switch to mode 0 and suspend charging */
	if  (current_ua == 0)
		value = 0x0;
	else if (current_ua <= 200000)
		value = 0x03;
	else if (current_ua >= 4000000)
		value = 0x3c;
	else
		value = 0x3 + (current_ua - 200000) / 66670;

	value = VALUE2FIELD(MAX77759_CHG_CNFG_02_CHGCC, value);
	ret = max77759_reg_update(data, MAX77759_CHG_CNFG_02,
				   MAX77759_CHG_CNFG_02_CHGCC_MASK,
				   value);
	if (ret == 0)
		ret = max77759_set_charge_enabled(data, !disabled, "CC_MAX");

	return ret;
}

static int max77759_get_charger_current_max_ua(struct max77759_chgr_data *data,
					       int *current_ua)
{
	u8 value;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_02,
				&value);
	if (ret < 0)
		return ret;

	/* TODO: fix the rounding */
	value = VALUE2FIELD(MAX77759_CHG_CNFG_02_CHGCC, value);

	/* ilim=0 -> mode 0 with charging suspended */
	if (value == 0)
		*current_ua = 0;
	else if (value < 3)
		*current_ua = 133 * 1000;
	else if (value >= 0x3C)
		*current_ua = 4000 * 1000;
	else
		*current_ua = 133000 + (value - 2) * 66670;

	return 0;
}

/* enable autoibus and charger mode */
static int max77759_chgin_set_ilim_max_ua(struct max77759_chgr_data *data,
					  int ilim_ua)
{
	const bool suspend = ilim_ua == 0;
	u8 value;
	int ret;

	/* TODO: disable charging */
	if (ilim_ua < 0)
		return 0;

	if (ilim_ua == 0)
		value = 0x00;
	else if (ilim_ua > 3200000)
		value = 0x7f;
	else
		value = 0x04 + (ilim_ua - 125000) / 25000;

	value = VALUE2FIELD(MAX77759_CHG_CNFG_09_NO_AUTOIBUS, 1) |
		VALUE2FIELD(MAX77759_CHG_CNFG_09_CHGIN_ILIM, value);
	ret = max77759_reg_update(data, MAX77759_CHG_CNFG_09,
					MAX77759_CHG_CNFG_09_NO_AUTOIBUS |
					MAX77759_CHG_CNFG_09_CHGIN_ILIM_MASK,
					value);
	if (ret == 0)
		ret = max77759_chgin_input_suspend(data, suspend, "ILIM");

	return ret;
}

static int max77759_chgin_get_ilim_max_ua(struct max77759_chgr_data *data,
					  int *ilim_ua)
{
	int icl, ret;
	u8 value;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_09, &value);
	if (ret < 0)
		return ret;

	value = FIELD2VALUE(MAX77759_CHG_CNFG_09_CHGIN_ILIM, value);
	if (value == 0)
		icl = 0;
	else if (value > 3)
		icl = 100 + (value - 3) * 25;
	else
		icl = 100;

	*ilim_ua = icl * 1000;

	if (data->chgin_input_suspend)
		*ilim_ua = 0;

	return 0;
}

static int max77759_wcin_set_ilim_max_ua(struct max77759_chgr_data *data,
					 int ilim_ua)
{
	const bool suspend = ilim_ua == 0;
	u8 value;
	int ret;

	if (ilim_ua < 0)
		return -EINVAL;

	if (ilim_ua == 0)
		value = 0x00;
	else if (ilim_ua <= 125000)
		value = 0x01;
	else
		value = 0x3 + (ilim_ua - 125000) / 31250;

	value = VALUE2FIELD(MAX77759_CHG_CNFG_10_WCIN_ILIM, value);
	ret = max77759_reg_update(data, MAX77759_CHG_CNFG_10,
					MAX77759_CHG_CNFG_10_WCIN_ILIM_MASK,
					value);

	if (ret == 0)
		ret = max77759_wcin_input_suspend(data, suspend, "DC_ICL");

	return ret;
}

static int max77759_wcin_get_ilim_max_ua(struct max77759_chgr_data *data,
					 int *ilim_ua)
{
	int ret;
	u8 value;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_10, &value);
	if (ret < 0)
		return ret;

	value = FIELD2VALUE(MAX77759_CHG_CNFG_10_WCIN_ILIM, value);
	if (value == 0)
		*ilim_ua = 0;
	else if (value < 4)
		*ilim_ua = 125000;
	else
		*ilim_ua = 125000 + (value - 3) * 31250;

	if (data->wcin_input_suspend)
		*ilim_ua = 0;

	return 0;
}

/* default is no suspend, any valid vote will suspend  */
static void max77759_dc_suspend_vote_callback(struct gvotable_election *el,
					      const char *reason, void *value)
{
	struct max77759_chgr_data *data = gvotable_get_data(el);
	int ret, suspend = (long)value > 0;

	ret = max77759_wcin_input_suspend(data, suspend, "DC_SUSPEND");
	if (ret < 0)
		return;

	/* enable charging when DC_SUSPEND is not set */
	ret = max77759_set_charge_enabled(data, !suspend, "DC_SUSPEND");

	dev_info(data->dev, "DC_SUSPEND reason=%s, value=%ld suspend=%d (%d)\n",
		reason ? reason : "", (long)value, suspend, ret);
}

static void max77759_dcicl_callback(struct gvotable_election *el,
				    const char *reason,
				    void *value)
{
	struct max77759_chgr_data *data = gvotable_get_data(el);
	int dc_icl = (long)value;
	const bool suspend = dc_icl == 0;
	int ret;

	pr_debug("%s: dc_icl=%d suspend=%d\n", __func__,
		 dc_icl, suspend);

	ret = max77759_wcin_set_ilim_max_ua(data, dc_icl);
	if (ret < 0)
		dev_err(data->dev, "cannot set DC_ICL (%d)\n", ret);

	ret = gvotable_cast_vote(data->dc_suspend_votable, "DC_ICL",
				  (void *)1,
				  suspend);

	if (ret < 0)
		dev_err(data->dev, "cannot set DC_SUSPEND=%d (%d)\n",
			suspend, ret);
}

/*************************
 * WCIN PSY REGISTRATION   *
 *************************/
static enum power_supply_property max77759_wcin_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static int max77759_wcin_is_valid(struct max77759_chgr_data *data)
{
	uint8_t int_ok;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	return (ret == 0) && _chg_int_ok_wcin_ok_get(int_ok);
}

static int max77759_wcin_is_online(struct max77759_chgr_data *data)
{
	uint8_t val;
	int ret;

	ret = max77759_wcin_is_valid(data);
	if (ret <= 0)
		return ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02, &val);
	return (ret == 0) && _chg_details_02_wcin_sts_get(val);
}

static int max77759_wcin_voltage_max(struct max77759_chgr_data *chg,
				     union power_supply_propval *val)
{
	int rc;

	if (!chg->wlc_psy) {
		chg->wlc_psy = power_supply_get_by_name("wireless");
		if (!chg->wlc_psy)
			return -ENODEV;
	}

	if (!max77759_wcin_is_valid(chg)) {
		val->intval = 0;
		return 0;
	}

	rc = power_supply_get_property(chg->wlc_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_MAX, val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get VOLTAGE_MAX, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int max77759_wcin_voltage_now(struct max77759_chgr_data *chg,
				     union power_supply_propval *val)
{
	int rc;

	if (!chg->wlc_psy) {
		chg->wlc_psy = power_supply_get_by_name("wireless");
		if (!chg->wlc_psy)
			return -ENODEV;
	}

	if (!max77759_wcin_is_valid(chg)) {
		val->intval = 0;
		return 0;
	}

	rc = power_supply_get_property(chg->wlc_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get VOLTAGE_NOW, rc=%d\n", rc);

	return rc;
}

#define MAX77759_WCIN_RAW_TO_UA	156

static bool max77759_current_check_mode(struct max77759_chgr_data *data)
{
	int ret;
	u8 reg;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_00, &reg);
	if (ret < 0)
		return false;

	return reg == 5 || reg == 6 || reg == 7 || reg == 0xe || reg == 0xf;
}

/* only valid in mode 5, 6, 7, e, f */
static int max77759_wcin_current_now(struct max77759_chgr_data *data, int *iic)
{
	int ret, iic_raw;

	ret = max77759_find_fg(data);
	if (ret < 0)
		return ret;

	ret = max_m5_read_actual_input_current_ua(data->fg_i2c_client, &iic_raw);
	if (ret < 0)
		return ret;

	*iic = iic_raw * MAX77759_WCIN_RAW_TO_UA;
	return 0;
}

static int max77759_wcin_get_prop(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct max77759_chgr_data *chgr = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = max77759_wcin_is_valid(chgr);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = max77759_wcin_is_online(chgr);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = max77759_wcin_voltage_now(chgr, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = max77759_wcin_get_ilim_max_ua(chgr, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = max77759_wcin_voltage_max(chgr, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = 0;
		if (max77759_wcin_is_online(chgr))
			rc = max77759_wcin_current_now(chgr, &val->intval);
		break;
	default:
		return -EINVAL;
	}
	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}
	return 0;
}

static int max77759_wcin_set_prop(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct max77759_chgr_data *chgr = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = max77759_wcin_set_ilim_max_ua(chgr, val->intval);
		break;
	/* called from google_cpm when switching chargers */
	case GBMS_PROP_CHARGING_ENABLED:
		rc = max77759_set_charge_enabled(chgr, val->intval > 0,
						 "DC_PSP_ENABLED");
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int max77759_wcin_prop_is_writeable(struct power_supply *psy,
					   enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGING_ENABLED:
		return 1;
	default:
		break;
	}

	return 0;
}

static const struct power_supply_desc max77759_wcin_psy_desc = {
	.name = "dc",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = max77759_wcin_props,
	.num_properties = ARRAY_SIZE(max77759_wcin_props),
	.get_property = max77759_wcin_get_prop,
	.set_property = max77759_wcin_set_prop,
	.property_is_writeable = max77759_wcin_prop_is_writeable,
};

static int max77759_init_wcin_psy(struct max77759_chgr_data *data)
{
	struct power_supply_config wcin_cfg = {};

	wcin_cfg.drv_data = data;
	wcin_cfg.of_node = data->dev->of_node;
	data->wcin_psy = devm_power_supply_register(
		data->dev, &max77759_wcin_psy_desc, &wcin_cfg);
	if (IS_ERR(data->wcin_psy))
		return PTR_ERR(data->wcin_psy);

	return 0;
}


static int max77759_chg_is_valid(struct max77759_chgr_data *data)
{
	uint8_t int_ok;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	return (ret == 0) && _chg_int_ok_chgin_ok_get(int_ok);
}

static int max77759_chgin_is_online(struct max77759_chgr_data *data)
{
	uint8_t val;
	int ret;

	ret = max77759_chg_is_valid(data);
	if (ret <= 0)
		return ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02, &val);
	return (ret == 0) && _chg_details_02_chgin_sts_get(val);
}

/*
 * NOTE: could also check aicl to determine whether the adapter is, in fact,
 * at fault. Possibly qualify this with battery voltage as subpar adapters
 * are likely to flag AICL when the battery is at high voltage.
 */
static int max77759_is_limited(struct max77759_chgr_data *data)
{
	int ret;
	u8 value;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &value);
	return (ret == 0) && _chg_int_ok_inlim_ok_get(value) == 0;
}

static int max77759_is_valid(struct max77759_chgr_data *data)
{
	uint8_t int_ok;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	return (ret == 0) && (_chg_int_ok_chgin_ok_get(int_ok) ||
	       _chg_int_ok_wcin_ok_get(int_ok));
}

/* WCIN || CHGIN present, valid  && CHGIN FET is closed */
static int max77759_is_online(struct max77759_chgr_data *data)
{
	uint8_t val;
	int ret;

	ret = max77759_is_valid(data);
	if (ret <= 0)
		return 0;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02, &val);
	return (ret == 0) && (_chg_details_02_chgin_sts_get(val) ||
	       _chg_details_02_wcin_sts_get(val));
}

static int max77759_get_charge_type(struct max77759_chgr_data *data)
{
	int ret;
	uint8_t reg;

	if (!max77759_is_online(data))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &reg);
	if (ret < 0)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	switch(_chg_details_01_chg_dtls_get(reg)) {
	case CHGR_DTLS_DEAD_BATTERY_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE:
	case CHGR_DTLS_TOP_OFF_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT;

	case CHGR_DTLS_DONE_MODE:
	case CHGR_DTLS_TIMER_FAULT_MODE:
	case CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE:
	case CHGR_DTLS_OFF_MODE:
	case CHGR_DTLS_OFF_HIGH_TEMP_MODE:
	case CHGR_DTLS_OFF_WATCHDOG_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		break;
	}

	return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
}

static bool max77759_is_full(struct max77759_chgr_data *data)
{
	int ret, vbatt = 0;

	/*
	 * Set voltage level to leave CHARGER_DONE (BATT_RL_STATUS_DISCHARGE)
	 * and enter BATT_RL_STATUS_RECHARGE. It sets STATUS_DISCHARGE again
	 * once CHARGER_DONE flag set (return true here)
	 */
	ret = max77759_read_vbatt(data, &vbatt);
	if (ret == 0)
		vbatt = vbatt / 1000;

	/* true when chg_term_voltage==0, false if read error (vbatt==0) */
	return vbatt >= data->chg_term_voltage;
}

static int max77759_get_status(struct max77759_chgr_data *data)
{
	uint8_t val;
	int ret;

	if (!max77759_is_online(data))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &val);
	if (ret < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	switch (_chg_details_01_chg_dtls_get(val)) {
		case CHGR_DTLS_DEAD_BATTERY_MODE:
		case CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE:
		case CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE:
		case CHGR_DTLS_TOP_OFF_MODE:
			return POWER_SUPPLY_STATUS_CHARGING;
		case CHGR_DTLS_DONE_MODE:
			/* same as POWER_SUPPLY_PROP_CHARGE_DONE */
			if (max77759_is_full(data))
				return POWER_SUPPLY_STATUS_FULL;
			else
				return POWER_SUPPLY_STATUS_NOT_CHARGING;
		case CHGR_DTLS_TIMER_FAULT_MODE:
		case CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE:
		case CHGR_DTLS_OFF_MODE:
		case CHGR_DTLS_OFF_HIGH_TEMP_MODE:
		case CHGR_DTLS_OFF_WATCHDOG_MODE:
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		default:
			break;
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static int max77759_get_chg_chgr_state(struct max77759_chgr_data *data,
				       union gbms_charger_state *chg_state)
{
	int usb_present, usb_valid, dc_present, dc_valid;
	const char *source = "";
	uint8_t int_ok, dtls;
	int vbatt, icl = 0;
	int rc;

	chg_state->v = 0;
	chg_state->f.chg_status = max77759_get_status(data);
	chg_state->f.chg_type = max77759_get_charge_type(data);
	chg_state->f.flags = gbms_gen_chg_flags(chg_state->f.chg_status,
						chg_state->f.chg_type);

	rc = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	if (rc == 0)
		rc = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02,
					&dtls);

	/* present when connected, valid when FET is closed */
	usb_present = (rc == 0) && _chg_int_ok_chgin_ok_get(int_ok);
	usb_valid = usb_present && _chg_details_02_chgin_sts_get(dtls);

	/* present if in field, valid when FET is closed */
	dc_present = (rc == 0) && _chg_int_ok_wcin_ok_get(int_ok);
	dc_valid = dc_present && _chg_details_02_wcin_sts_get(dtls);


	rc = max77759_read_vbatt(data, &vbatt);
	if (rc == 0)
		chg_state->f.vchrg = vbatt / 1000;

	if (chg_state->f.chg_status == POWER_SUPPLY_STATUS_DISCHARGING)
		goto exit_done;

	rc = max77759_is_limited(data);
	if (rc > 0)
		chg_state->f.flags |= GBMS_CS_FLAG_ILIM;

	/* TODO: b/ handle input MUX corner cases */
	if (usb_valid) {
		max77759_chgin_get_ilim_max_ua(data, &icl);
		/* TODO: 'u' only when in sink */
		source = dc_present ? "Uw" : "u";
	} else if (dc_valid) {
		max77759_wcin_get_ilim_max_ua(data, &icl);

		/* TODO: 'u' only when in sink */
		source = usb_present ? "uW" : "w";
	} else if (usb_present && dc_present) {
		source = "-";
	} else if (usb_present) {
		source = "u?";
	} else if (dc_present) {
		source = "w?";
	}

	chg_state->f.icl = icl / 1000;

exit_done:
	pr_debug("MSC_PCS chg_state=%lx [0x%x:%d:%d:%d:%d] chg=%s\n",
		 (unsigned long)chg_state->v,
		 chg_state->f.flags,
		 chg_state->f.chg_type,
		 chg_state->f.chg_status,
		 chg_state->f.vchrg,
		 chg_state->f.icl,
		 source);

	return 0;
}

#define MAX77759_CHGIN_RAW_TO_UA	125

/* only valid in mode 5, 6, 7, e, f */
static int max77759_chgin_current_now(struct max77759_chgr_data *data, int *iic)
{
	int ret, iic_raw;

	ret = max77759_find_fg(data);
	if (ret < 0)
		return ret;

	ret = max_m5_read_actual_input_current_ua(data->fg_i2c_client, &iic_raw);
	if (ret < 0)
		return ret;

	*iic = iic_raw * MAX77759_CHGIN_RAW_TO_UA;
	return 0;
}

static int max77759_wd_tickle(struct max77759_chgr_data *data)
{
	int ret;
	u8 reg, reg_new;

	mutex_lock(&data->io_lock);
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_00, &reg);
	if (ret == 0) {
		reg_new  = _chg_cnfg_00_wdtclr_set(reg, 0x1);
		ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_00,
					 reg_new);
	}

	if (ret < 0)
		pr_err("WD Tickle failed %d\n", ret);

	mutex_unlock(&data->io_lock);
	return ret;
}

/* online is used from DC charging to tickle the watchdog (if enabled) */
static int max77759_set_online(struct max77759_chgr_data *data, bool online)
{
	int ret = 0;

	if (data->wden) {
		ret = max77759_wd_tickle(data);
		if (ret < 0)
			pr_err("cannot tickle the watchdog\n");
	}

	if (data->online != online) {
		ret = gvotable_cast_vote(data->mode_votable, "OFFLINE",
					 (void *)GBMS_CHGR_MODE_STBY_ON,
					 !online);
		data->online = online;
	}

	return ret;
}

static int max77759_psy_set_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     const union power_supply_propval *pval)
{
	struct max77759_chgr_data *data = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	pm_runtime_get_sync(data->dev);
	if (!data->init_complete || !data->resume_complete) {
		pm_runtime_put_sync(data->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(data->dev);

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = max77759_chgin_set_ilim_max_ua(data, pval->intval);
		pr_debug("%s: icl=%d (%d)\n", __func__, pval->intval, ret);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = max77759_set_charger_current_max_ua(data, pval->intval);
		pr_debug("%s: charge_current=%d (%d)\n",
			__func__, pval->intval, ret);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = max77759_set_regulation_voltage(data, pval->intval);
		pr_debug("%s: charge_voltage=%d (%d)\n",
			__func__, pval->intval, ret);
		break;
	/* called from google_cpm when switching chargers */
	case GBMS_PROP_CHARGING_ENABLED:
		ret = max77759_set_charge_enabled(data, pval->intval,
						  "PSP_ENABLED");
		pr_debug("%s: charging_enabled=%d (%d)\n",
			__func__, pval->intval, ret);
		break;
	/* called from google_charger on disconnect */
	case GBMS_PROP_CHARGE_DISABLE:
		ret = max77759_set_charge_disable(data, pval->intval,
						  "PSP_DISABLE");
		pr_debug("%s: charge_disable=%d (%d)\n",
			__func__, pval->intval, ret);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = max77759_set_online(data, pval->intval != 0);
		break;
	default:
		break;
	}

	if (ret == 0 && data->wden)
		max77759_wd_tickle(data);


	return ret;
}

static int max77759_psy_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *pval)
{
	struct max77759_chgr_data *data = power_supply_get_drvdata(psy);
	union gbms_charger_state chg_state;
	int rc, ret = 0;

	pm_runtime_get_sync(data->dev);
	if (!data->init_complete || !data->resume_complete) {
		pm_runtime_put_sync(data->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(data->dev);

	switch (psp) {
	case GBMS_PROP_CHARGE_DISABLE:
		rc = max77759_get_charge_enabled(data, &pval->intval);
		if (rc == 0)
			pval->intval = !pval->intval;
		else
			pval->intval = rc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		pval->intval = max77759_get_charge_type(data);
		break;
	case GBMS_PROP_CHARGING_ENABLED:
		ret = max77759_get_charge_enabled(data, &pval->intval);
		break;
	case GBMS_PROP_CHARGE_CHARGER_STATE:
		rc = max77759_get_chg_chgr_state(data, &chg_state);
		if (rc == 0)
			gbms_propval_int64val(pval) = chg_state.v;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = max77759_get_charger_current_max_ua(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = max77759_get_regulation_voltage_uv(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		pval->intval = max77759_is_online(data);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		pval->intval = max77759_is_valid(data);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = max77759_chgin_get_ilim_max_ua(data, &pval->intval);
		break;
	case GBMS_PROP_INPUT_CURRENT_LIMITED:
		pval->intval = max77759_is_limited(data);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		pval->intval = max77759_get_status(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = max77759_read_vbatt(data, &pval->intval);
		if (rc < 0)
			pval->intval = -1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!max77759_current_check_mode(data)) {
			pval->intval = 0;
			break;
		}

		if (max77759_wcin_is_online(data))
			rc = max77759_wcin_current_now(data, &pval->intval);
		else if (max77759_chgin_is_online(data))
			rc = max77759_chgin_current_now(data, &pval->intval);
		else
			rc = -EINVAL;

		if (rc < 0)
			pval->intval = rc;
		break;
	default:
		dev_err(data->dev, "property (%d) unsupported.\n", psp);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int max77759_psy_is_writeable(struct power_supply *psy,
				 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX: /* compat, same the next */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGE_DISABLE:
		return 1;
	default:
		break;
	}

	return 0;
}

/*
 * TODO: POWER_SUPPLY_PROP_RERUN_AICL, POWER_SUPPLY_PROP_TEMP
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX
 */
static enum power_supply_property max77759_psy_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,		/* compat */
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
};

static struct power_supply_desc max77759_psy_desc = {
	.name = "max77759-charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = max77759_psy_props,
	.num_properties = ARRAY_SIZE(max77759_psy_props),
	.get_property = max77759_psy_get_property,
	.set_property = max77759_psy_set_property,
	.property_is_writeable = max77759_psy_is_writeable,
};

static ssize_t show_fship_dtls(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct max77759_chgr_data *data = dev_get_drvdata(dev);
	static char *fship_reason[] = {"None", "PWRONB1", "PWRONB1", "PWR"};
	u8 pmic_rd;
	int ret;

	if (data->fship_dtls != -1)
		goto exit_done;


	ret = max77759_find_pmic(data);
	if (ret < 0)
		return ret;

	ret = max777x9_pmic_reg_read(data->pmic_i2c_client,
				     MAX77759_FSHIP_EXIT_DTLS,
				     &pmic_rd, 1);
	if (ret < 0)
		return -EIO;

	if (pmic_rd & MAX77759_FSHIP_EXIT_DTLS_RD) {
		u8 fship_dtls;

		ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_03,
					&fship_dtls);
		if (ret < 0)
			return -EIO;

		data->fship_dtls =
			_chg_details_03_fship_exit_dtls_get(fship_dtls);

		pmic_rd &= ~MAX77759_FSHIP_EXIT_DTLS_RD;
		ret = max777x9_pmic_reg_write(data->pmic_i2c_client,
					      MAX77759_FSHIP_EXIT_DTLS,
					      &pmic_rd, 1);
		if (ret < 0)
			pr_err("FSHIP: cannot update RD (%d)\n", ret);

	} else {
		data->fship_dtls = 0;
	}

exit_done:
	return scnprintf(buf, PAGE_SIZE, "%d %s\n", data->fship_dtls,
			 fship_reason[data->fship_dtls]);
}

static DEVICE_ATTR(fship_dtls, 0444, show_fship_dtls, NULL);

static ssize_t show_sysuvlo1_cnt(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct max77759_chgr_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 atomic_read(&data->sysuvlo1_cnt));
}

static DEVICE_ATTR(sysuvlo1_cnt, 0444, show_sysuvlo1_cnt, NULL);

static ssize_t show_sysuvlo2_cnt(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct max77759_chgr_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 atomic_read(&data->sysuvlo2_cnt));
}

static DEVICE_ATTR(sysuvlo2_cnt, 0444, show_sysuvlo2_cnt, NULL);

static ssize_t show_batoilo_cnt(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct max77759_chgr_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 atomic_read(&data->batoilo_cnt));
}

static DEVICE_ATTR(batoilo_cnt, 0444, show_batoilo_cnt, NULL);

static int vdroop2_ok_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_dtls1;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &chg_dtls1);
	if (ret < 0)
		return -ENODEV;

	*val = _chg_details_01_vdroop2_ok_get(chg_dtls1);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vdroop2_ok_fops, vdroop2_ok_get, NULL, "%llu\n");

static int vdp1_stp_bst_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg18;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &chg_cnfg18);
	if (ret < 0)
		return -ENODEV;

	*val = _chg_cnfg_18_vdp1_stp_bst_get(chg_cnfg18);
	return 0;
}

static int vdp1_stp_bst_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg18;
	const u8 vdp1_stp_bst = (val > 0)? 0x1 : 0x0;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &chg_cnfg18);
	if (ret < 0)
		return -ENODEV;

	chg_cnfg18 = _chg_cnfg_18_vdp1_stp_bst_set(chg_cnfg18, vdp1_stp_bst);
	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_18, chg_cnfg18);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(vdp1_stp_bst_fops, vdp1_stp_bst_get, vdp1_stp_bst_set, "%llu\n");

static int vdp2_stp_bst_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg18;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &chg_cnfg18);
	if (ret < 0)
		return -ENODEV;

	*val = _chg_cnfg_18_vdp2_stp_bst_get(chg_cnfg18);
	return 0;
}

static int vdp2_stp_bst_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg18;
	const u8 vdp2_stp_bst = (val > 0)? 0x1 : 0x0;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &chg_cnfg18);
	if (ret < 0)
		return -ENODEV;

	chg_cnfg18 = _chg_cnfg_18_vdp2_stp_bst_set(chg_cnfg18, vdp2_stp_bst);
	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_18, chg_cnfg18);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(vdp2_stp_bst_fops, vdp2_stp_bst_get, vdp2_stp_bst_set, "%llu\n");

static int bat_oilo_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg14;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_14, &chg_cnfg14);
	if (ret < 0)
		return -EIO;

	*val = chg_cnfg14;
	return 0;
}

static int bat_oilo_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret;

	if (val > 0xf)
		return -EINVAL;

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_14, (u8) val);
	if (ret < 0)
		return -EIO;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(bat_oilo_fops, bat_oilo_get, bat_oilo_set, "0x%llx\n");

static int sys_uvlo1_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg15;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_15, &chg_cnfg15);
	if (ret < 0)
		return -EIO;

	*val = chg_cnfg15;
	return 0;
}

static int sys_uvlo1_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret;

	if (val > 0xf)
		return -EINVAL;

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_15, (u8) val);
	if (ret < 0)
		return -EIO;

	data->vdroop_lvl[VDROOP1] = VD_BATTERY_VOLTAGE - (VD_STEP * val + VD_LOWER_LIMIT);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sys_uvlo1_fops, sys_uvlo1_get, sys_uvlo1_set, "0x%llx\n");

static int sys_uvlo2_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg16;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_16, &chg_cnfg16);
	if (ret < 0)
		return -EIO;

	*val = chg_cnfg16;
	return 0;
}

static int sys_uvlo2_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret;

	if (val > 0xf)
		return -EINVAL;

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_16, (u8) val);
	if (ret < 0)
		return -EIO;

	data->vdroop_lvl[VDROOP2] = VD_BATTERY_VOLTAGE - (VD_STEP * val + VD_LOWER_LIMIT);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sys_uvlo2_fops, sys_uvlo2_get, sys_uvlo2_set, "0x%llx\n");


/* write to INPUT_MASK_CLR in to re-enable detection */
static int max77759_chgr_input_mask_clear(struct max77759_chgr_data *data)
{
	u8 value;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_10, &value);
	if (ret < 0)
		return -ENODEV;

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_10,
				 _chg_cnfg_10_input_mask_clr_set(value, 1));
	if (ret < 0)
		pr_err("%s: cannot clear input_mask ret=%d\n", __func__, ret);

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_10,
				 _chg_cnfg_10_input_mask_clr_set(value, 0));
	if (ret < 0)
		pr_err("%s: cannot reset input_mask ret=%d\n", __func__, ret);

	return ret;
}


static int input_mask_clear_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;

	return max77759_chgr_input_mask_clear(data);
}

DEFINE_SIMPLE_ATTRIBUTE(input_mask_clear_fops, NULL, input_mask_clear_set, "%llu\n");

static int max77759_chg_debug_reg_read(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	u8 reg = 0;
	int ret;

	ret = max77759_reg_read(data->regmap, data->debug_reg_address, &reg);
	if (ret)
		return ret;

	*val = reg;
	return 0;
}

static int max77759_chg_debug_reg_write(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	u8 reg = (u8) val;

	pr_warn("debug write reg 0x%x, 0x%x", data->debug_reg_address, reg);
	return max77759_reg_write(data->regmap, data->debug_reg_address, reg);
}
DEFINE_SIMPLE_ATTRIBUTE(debug_reg_rw_fops, max77759_chg_debug_reg_read,
			max77759_chg_debug_reg_write, "%02llx\n");

static int dbg_init_fs(struct max77759_chgr_data *data)
{
	int ret;

	ret = device_create_file(data->dev, &dev_attr_fship_dtls);
	if (ret != 0)
		pr_err("Failed to create fship_dtls, ret=%d\n", ret);
	ret = device_create_file(data->dev, &dev_attr_sysuvlo1_cnt);
	if (ret != 0)
		pr_err("Failed to create sysuvlo1_cnt, ret=%d\n", ret);
	ret = device_create_file(data->dev, &dev_attr_sysuvlo2_cnt);
	if (ret != 0)
		pr_err("Failed to create sysuvlo2_cnt, ret=%d\n", ret);
	ret = device_create_file(data->dev, &dev_attr_batoilo_cnt);
	if (ret != 0)
		pr_err("Failed to create bat_oilo_cnt, ret=%d\n", ret);

	data->de = debugfs_create_dir("max77759_chg", 0);
	if (IS_ERR_OR_NULL(data->de))
		return -EINVAL;

	debugfs_create_atomic_t("sysuvlo1_cnt", 0644, data->de,
				&data->sysuvlo1_cnt);
	debugfs_create_atomic_t("sysuvlo2_cnt", 0644, data->de,
				&data->sysuvlo2_cnt);
	debugfs_create_atomic_t("batoilo_cnt", 0644, data->de,
				&data->batoilo_cnt);

	debugfs_create_atomic_t("insel_cnt", 0644, data->de, &data->insel_cnt);
	debugfs_create_bool("insel_clear", 0644, data->de, &data->insel_clear);

	debugfs_create_file("vdroop2_ok", 0400, data->de, data,
			    &vdroop2_ok_fops);
	debugfs_create_file("vdp1_stp_bst", 0600, data->de, data,
			    &vdp1_stp_bst_fops);
	debugfs_create_file("vdp2_stp_bst", 0600, data->de, data,
			    &vdp2_stp_bst_fops);
	debugfs_create_file("sys_uvlo1", 0600, data->de, data,
			    &sys_uvlo1_fops);
	debugfs_create_file("sys_uvlo2", 0600, data->de, data,
			    &sys_uvlo2_fops);
	debugfs_create_file("bat_oilo", 0600, data->de, data,
			    &bat_oilo_fops);
	debugfs_create_file("input_mask_clear", 0600, data->de, data,
			    &input_mask_clear_fops);

	debugfs_create_u32("address", 0600, data->de, &data->debug_reg_address);
	debugfs_create_file("data", 0600, data->de, data, &debug_reg_rw_fops);
	return 0;
}

static bool max77759_chg_is_reg(struct device *dev, unsigned int reg)
{
	return (reg >= MAX77759_CHG_INT) && (reg <= MAX77759_CHG_CNFG_19);
}

static const struct regmap_config max77759_chg_regmap_cfg = {
	.name = "max77759_charger",
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX77759_CHG_CNFG_19,
	.readable_reg = max77759_chg_is_reg,
	.volatile_reg = max77759_chg_is_reg,

};

static u8 max77759_int_mask[MAX77759_CHG_INT_COUNT] = {
	~(MAX77759_CHG_INT_MASK_CHGIN_M |
	  MAX77759_CHG_INT_MASK_WCIN_M |
	  MAX77759_CHG_INT_MASK_CHG_M |
	  MAX77759_CHG_INT_MASK_BAT_M),
	(u8)~(MAX77759_CHG_INT2_MASK_INSEL_M |
	  MAX77759_CHG_INT2_MASK_SYS_UVLO1_M |
	  MAX77759_CHG_INT2_MASK_SYS_UVLO2_M |
	  MAX77759_CHG_INT2_MASK_CHG_STA_CV_M |
	  MAX77759_CHG_INT2_MASK_CHG_STA_TO_M |
	  MAX77759_CHG_INT2_MASK_CHG_STA_DONE_M),
};

static int max77759_irq_work(struct max77759_chgr_data *data, u8 idx)
{
	u8 chg_dtls1;
	int ret;
	u64 val;
	struct delayed_work *irq_wq = &data->vdroop_irq_work[idx];

	mutex_lock(&data->vdroop_irq_lock[idx]);
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &chg_dtls1);
	if (ret < 0) {
		mutex_unlock(&data->vdroop_irq_lock[idx]);
		return -ENODEV;
	}
	val = _chg_details_01_vdroop2_ok_get(chg_dtls1);

	if (((val >> 7) & 0x1) == 0) {
		/* Still below threshold */
		mod_delayed_work(system_wq, irq_wq,
				   msecs_to_jiffies(VD_DELAY));
	} else {
		data->vdroop_counter[idx] = 0;
		enable_irq(data->vdroop_irq[idx]);
	}
	mutex_unlock(&data->vdroop_irq_lock[idx]);
	return 0;
}

static void max77759_vdroop1_work(struct work_struct *work)
{
	struct max77759_chgr_data *chg_data =
	    container_of(work, struct max77759_chgr_data, vdroop_irq_work[VDROOP1].work);

	max77759_irq_work(chg_data, VDROOP1);
}

static void max77759_vdroop2_work(struct work_struct *work)
{
	struct max77759_chgr_data *chg_data =
	    container_of(work, struct max77759_chgr_data, vdroop_irq_work[VDROOP2].work);

	max77759_irq_work(chg_data, VDROOP2);
}

static void max77759_vdroop_irq_work(void *data, int id)
{
	struct max77759_chgr_data *chg_data = data;
	struct thermal_zone_device *tvid = chg_data->tz_vdroop[id];

	mutex_lock(&chg_data->vdroop_irq_lock[id]);
	if (chg_data->vdroop_counter[id] == 0) {
		chg_data->vdroop_counter[id] += 1;
		if (tvid)
			thermal_zone_device_update(tvid, THERMAL_EVENT_UNSPECIFIED);
	}
	disable_irq_nosync(chg_data->vdroop_irq[id]);
	mod_delayed_work(system_wq, &chg_data->vdroop_irq_work[id],
			 msecs_to_jiffies(VD_DELAY));
	mutex_unlock(&chg_data->vdroop_irq_lock[id]);
}

static int uvilo_read_stats(struct uvilo_stats *dst, struct max77759_chgr_data *data)
{
	int ret;

	if (!dst || !data)
		return -EINVAL;

	ret = max77759_find_fg(data);
	if (ret == 0)
		ret = max1720x_get_capacity(data->fg_i2c_client, &dst->capacity);
	if (ret == 0)
		ret = max1720x_get_voltage_now(data->fg_i2c_client, &dst->voltage);
	if (ret < 0)
		return -EINVAL;

	dst->_time = ktime_to_ms(ktime_get());
	return (dst->capacity == -1 || dst->voltage == -1) ? -EIO : 0;
}

static irqreturn_t max77759_chgr_irq(int irq, void *client)
{
	struct max77759_chgr_data *data = client;
	u8 chg_int[MAX77759_CHG_INT_COUNT];
	bool broadcast = false;
	int ret;

	ret = max77759_readn(data->regmap, MAX77759_CHG_INT, chg_int,
			     sizeof(chg_int));
	if (ret < 0)
		return IRQ_NONE;

	if ((chg_int[0] & ~max77759_int_mask[0]) == 0 &&
	    (chg_int[1] & ~max77759_int_mask[1]) == 0)
		return IRQ_NONE;

	ret = max77759_writen(data->regmap, MAX77759_CHG_INT, chg_int,
			      sizeof(chg_int));
	if (ret < 0)
		return IRQ_NONE;

	pr_debug("INT : %02x %02x\n", chg_int[0], chg_int[1]);

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_INSEL_M && data->insel_clear) {
		ret = max77759_chgr_input_mask_clear(data);
		if (ret < 0)
			pr_info("INT : %x %x : clear=%d\n",
				chg_int[0], chg_int[1], ret);
		else
			atomic_inc(&data->insel_cnt);
	}

	if (chg_int[1] & MAX77759_CHG_INT2_SYS_UVLO1_I) {
		pr_debug("%s: SYS_UVLO1\n", __func__);

		atomic_inc(&data->sysuvlo1_cnt);
		uvilo_read_stats(&data->sysuvlo1, data);
		max77759_vdroop_irq_work(data, VDROOP1);
	}

	if (chg_int[1] & MAX77759_CHG_INT2_SYS_UVLO2_I) {
		pr_debug("%s: SYS_UVLO2\n", __func__);

		atomic_inc(&data->sysuvlo2_cnt);
		uvilo_read_stats(&data->sysuvlo2, data);
		max77759_vdroop_irq_work(data, VDROOP2);
	}

	if (chg_int[1] & MAX77759_CHG_INT2_BAT_OILO_I) {
		pr_debug("%s: BAT_OILO\n", __func__);

		atomic_inc(&data->batoilo_cnt);
		uvilo_read_stats(&data->batoilo, data);
	}

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_CHG_STA_TO_M) {
		pr_debug("%s: TOP_OFF\n", __func__);
		/*
		 * TODO: rewrite  to FV_UV when if entering TOP off far
		 * from terminal voltage
		 */
	}

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_CHG_STA_CV_M)
		pr_debug("%s: CV_MODE\n", __func__);

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_CHG_STA_DONE_M) {
		pr_debug("%s: CHARGE DONE\n", __func__);

		if (data->psy)
			power_supply_changed(data->psy);
	}

	/* wired input is changed */
	if (chg_int[0] & MAX77759_CHG_INT_MASK_CHGIN_M) {
		pr_debug("%s: CHGIN\n", __func__);

		if (data->chgin_psy)
			power_supply_changed(data->chgin_psy);
		else
			power_supply_changed(data->psy);
	}

	/* wireless input is changed */
	if (chg_int[0] & MAX77759_CHG_INT_MASK_WCIN_M) {
		pr_debug("%s: WCIN\n", __func__);

		if (data->wcin_psy)
			power_supply_changed(data->wcin_psy);
		else
			power_supply_changed(data->psy);
	}

	/* someting else is changed */
	broadcast = (chg_int[0] & MAX77759_CHG_INT_MASK_CHG_M) |
		    (chg_int[0] & MAX77759_CHG_INT_MASK_BAT_M);
	if (data->psy && broadcast)
		power_supply_changed(data->psy);

	return IRQ_HANDLED;
}

static int max77759_setup_votables(struct max77759_chgr_data *data)
{
	int ret;

	/* votes might change mode */
	data->mode_votable = gvotable_create_int_election(NULL, NULL,
					max77759_mode_callback,
					data);
	if (IS_ERR_OR_NULL(data->mode_votable)) {
		ret = PTR_ERR(data->mode_votable);
		dev_err(data->dev, "no mode votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_vote2str(data->mode_votable, gvotable_v2s_uint);
	/* will use gvotable_get_default() when available */
	gvotable_set_default(data->mode_votable, (void *)GBMS_CHGR_MODE_STBY_ON);
	gvotable_election_set_name(data->mode_votable, GBMS_MODE_VOTABLE);

	/* Wireless charging, DC name is for compat */
	data->dc_suspend_votable =
		gvotable_create_bool_election(NULL,
					     max77759_dc_suspend_vote_callback,
					     data);
	if (IS_ERR_OR_NULL(data->dc_suspend_votable)) {
		ret = PTR_ERR(data->dc_suspend_votable);
		dev_err(data->dev, "no dc_suspend votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_vote2str(data->dc_suspend_votable, gvotable_v2s_int);
	gvotable_election_set_name(data->dc_suspend_votable, "DC_SUSPEND");

	data->dc_icl_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     max77759_dcicl_callback,
					     data);
	if (IS_ERR_OR_NULL(data->dc_icl_votable)) {
		ret = PTR_ERR(data->dc_icl_votable);
		dev_err(data->dev, "no dc_icl votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_vote2str(data->dc_icl_votable, gvotable_v2s_uint);
	gvotable_set_default(data->dc_icl_votable, (void *)700000);
	gvotable_election_set_name(data->dc_icl_votable, "DC_ICL");
	gvotable_use_default(data->dc_icl_votable, true);

	return 0;
}

static int max77759_vdroop_read_level(void *data, int *val, int id)
{
	struct max77759_chgr_data *chg_data = data;
	int vdroop_counter = chg_data->vdroop_counter[id];
	unsigned int vdroop_lvl = chg_data->vdroop_lvl[id];

	if ((vdroop_counter != 0) && (vdroop_counter < THERMAL_IRQ_COUNTER_LIMIT)) {
		*val = vdroop_lvl + THERMAL_HYST_LEVEL;
		vdroop_counter += 1;
	} else {
		*val = vdroop_lvl;
		vdroop_counter = 0;
	}
	chg_data->vdroop_counter[id] = vdroop_counter;

	return 0;
}

static int max77759_vdroop1_read_temp(void *data, int *val)
{
	return max77759_vdroop_read_level(data, val, VDROOP1);
}

static int max77759_vdroop2_read_temp(void *data, int *val)
{
	return max77759_vdroop_read_level(data, val, VDROOP2);
}

static const struct thermal_zone_of_device_ops vdroop1_tz_ops = {
	.get_temp = max77759_vdroop1_read_temp,
};

static const struct thermal_zone_of_device_ops vdroop2_tz_ops = {
	.get_temp = max77759_vdroop2_read_temp,
};



static int max77759_init_vdroop(void *data_)
{
	struct max77759_chgr_data *data = data_;
	int ret = 0;
	u8 regdata;

	INIT_DELAYED_WORK(&data->vdroop_irq_work[VDROOP1], max77759_vdroop1_work);
	INIT_DELAYED_WORK(&data->vdroop_irq_work[VDROOP2], max77759_vdroop2_work);
	data->vdroop_counter[VDROOP1] = 0;
	data->vdroop_counter[VDROOP2] = 0;
	mutex_init(&data->vdroop_irq_lock[VDROOP1]);
	mutex_init(&data->vdroop_irq_lock[VDROOP2]);

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_15, &regdata);
	if (ret < 0)
		return -ENODEV;

	data->vdroop_lvl[VDROOP1] = VD_BATTERY_VOLTAGE - (VD_STEP * regdata + VD_LOWER_LIMIT);

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_16, &regdata);
	if (ret < 0)
		return -ENODEV;

	data->vdroop_lvl[VDROOP2] = VD_BATTERY_VOLTAGE - (VD_STEP * regdata + VD_LOWER_LIMIT);

	data->tz_vdroop[VDROOP1] = thermal_zone_of_sensor_register(data->dev,
							      VDROOP1, data,
							      &vdroop1_tz_ops);
	if (IS_ERR(data->tz_vdroop[VDROOP1])) {
		dev_err(data->dev, "TZ register vdroop%d failed, err:%ld\n", VDROOP1,
			PTR_ERR(data->tz_vdroop[VDROOP1]));
	} else {
		thermal_zone_device_enable(data->tz_vdroop[VDROOP1]);
		thermal_zone_device_update(data->tz_vdroop[VDROOP1], THERMAL_DEVICE_UP);
	}
	data->tz_vdroop[VDROOP2] = thermal_zone_of_sensor_register(data->dev,
							      VDROOP2, data,
							      &vdroop2_tz_ops);
	if (IS_ERR(data->tz_vdroop[VDROOP2])) {
		dev_err(data->dev, "TZ register vdroop%d failed, err:%ld\n", VDROOP2,
			PTR_ERR(data->tz_vdroop[VDROOP2]));
	} else {
		thermal_zone_device_enable(data->tz_vdroop[VDROOP2]);
		thermal_zone_device_update(data->tz_vdroop[VDROOP2], THERMAL_DEVICE_UP);
	}
	return 0;
}

static ssize_t triggered_stats_show(struct device *dev, struct device_attribute *attr,
                                    char *buf)
{
	struct max77759_chgr_data *data = dev_get_drvdata(dev);
	int len = 0;

	len = scnprintf(buf, PAGE_SIZE, "%-10s\t%s\t%s\t%s\t%s\n",
			"Source", "Count", "Last Triggered", "Last SOC", "Last Voltage");

	len += scnprintf(buf + len, PAGE_SIZE - len, "%-15s\t%d\t%lld\t\t%d\t\t%d\n", "VDROOP1",
			 atomic_read(&data->sysuvlo1_cnt),
			 data->sysuvlo1._time, data->sysuvlo1.capacity, data->sysuvlo1.voltage);
	len += scnprintf(buf + len, PAGE_SIZE - len, "%-15s\t%d\t%lld\t\t%d\t\t%d\n", "VDROOP2",
			 atomic_read(&data->sysuvlo2_cnt),
			 data->sysuvlo2._time, data->sysuvlo2.capacity, data->sysuvlo2.voltage);
	len += scnprintf(buf + len, PAGE_SIZE - len, "%-15s\t%d\t%lld\t\t%d\t\t%d\n", "BATOILO",
			 atomic_read(&data->batoilo_cnt),
			 data->batoilo._time, data->batoilo.capacity, data->batoilo.voltage);

	return len;
}

static DEVICE_ATTR_RO(triggered_stats);

static int max77759_charger_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct power_supply_config chgr_psy_cfg = { 0 };
	struct device *dev = &client->dev;
	struct max77759_chgr_data *data;
	struct regmap *regmap;
	const char *tmp;
	int ret = 0;
	u8 ping;

	regmap = devm_regmap_init_i2c(client, &max77759_chg_regmap_cfg);
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	ret = max77759_reg_read(regmap, MAX77759_CHG_CNFG_00, &ping);
	if (ret < 0)
		return -ENODEV;

	/* TODO: PING or read HW version from PMIC */

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->regmap = regmap;
	data->fship_dtls = -1;
	data->wden = false; /* TODO: read from DT */
	mutex_init(&data->io_lock);
	atomic_set(&data->sysuvlo1_cnt, 0);
	atomic_set(&data->sysuvlo2_cnt, 0);
	atomic_set(&data->batoilo_cnt, 0);
	atomic_set(&data->insel_cnt, 0);
	i2c_set_clientdata(client, data);

	/* NOTE: only one instance */
	ret = of_property_read_string(dev->of_node, "max77759,psy-name", &tmp);
	if (ret == 0)
		max77759_psy_desc.name = devm_kstrdup(dev, tmp, GFP_KERNEL);

	chgr_psy_cfg.drv_data = data;
	chgr_psy_cfg.supplied_to = NULL;
	chgr_psy_cfg.num_supplicants = 0;
	data->psy = devm_power_supply_register(dev, &max77759_psy_desc,
		&chgr_psy_cfg);
	if (IS_ERR(data->psy)) {
		dev_err(dev, "Failed to register psy rc = %ld\n",
			PTR_ERR(data->psy));
		return -EINVAL;
	}

	/* CHARGER_MODE needs this (initialized to -EPROBE_DEFER) */
	max77759_setup_usecases(&data->uc_data, NULL);
	data->uc_data.client = client;

	/* other drivers (ex tcpci) need this. */
	ret = max77759_setup_votables(data);
	if (ret < 0)
		return ret;

	if (max77759_init_vdroop(data) < 0)
		dev_err(dev, "vdroop initialization failed %d.\n", ret);

	data->irq_gpio = of_get_named_gpio(dev->of_node, "max77759,irq-gpio", 0);
	if (data->irq_gpio < 0) {
		dev_err(dev, "failed get irq_gpio\n");
	} else {
		client->irq = gpio_to_irq(data->irq_gpio);

		ret = devm_request_threaded_irq(data->dev, client->irq, NULL,
						max77759_chgr_irq,
						IRQF_TRIGGER_LOW |
						IRQF_SHARED |
						IRQF_ONESHOT,
						"max77759_charger",
						data);
		if (ret == 0) {
			enable_irq_wake(client->irq);

			/* might cause the isr to be called */
			max77759_chgr_irq(-1, data);
			ret = max77759_writen(regmap, MAX77759_CHG_INT_MASK,
					      max77759_int_mask,
					      sizeof(max77759_int_mask));
			if (ret < 0)
				dev_err(dev, "cannot set irq_mask (%d)\n", ret);
		}
	}

	ret = dbg_init_fs(data);
	if (ret < 0)
		dev_err(dev, "Failed to initialize debug fs\n");

	mutex_lock(&data->io_lock);
	ret = max77759_wdt_enable(data, data->wden);
	if (ret < 0)
		dev_err(dev, "wd enable=%d failed %d\n", data->wden, ret);
	mutex_unlock(&data->io_lock);

	ret = of_property_read_u32(dev->of_node, "max77759,chg-term-voltage",
				   &data->chg_term_voltage);
	if (ret < 0)
		data->chg_term_voltage = 0;

	data->init_complete = 1;
	data->resume_complete = 1;

	ret = max77759_init_wcin_psy(data);
	if (ret < 0)
		pr_err("Couldn't register dc power supply (%d)\n", ret);

	data->mdev = pmic_device_create(data, "max77759-mitigation");
	if (data->mdev) {
		ret = device_create_file(data->mdev, &dev_attr_triggered_stats);
		if (ret)
			dev_err(dev, "No mitigation device (%d)\n", ret);
	}

	dev_info(dev, "registered as %s\n", max77759_psy_desc.name);
	return 0;
}

static int max77759_charger_remove(struct i2c_client *client)
{
	struct max77759_chgr_data *data = i2c_get_clientdata(client);

	if (data->tz_vdroop[VDROOP1])
		thermal_zone_of_sensor_unregister(data->dev, data->tz_vdroop[VDROOP2]);
	if (data->tz_vdroop[VDROOP2])
		thermal_zone_of_sensor_unregister(data->dev, data->tz_vdroop[VDROOP2]);
	if (data->mdev)
		pmic_device_destroy(data->mdev->devt);

	return 0;
}


static const struct of_device_id max77759_charger_of_match_table[] = {
	{ .compatible = "maxim,max77759chrg"},
	{},
};
MODULE_DEVICE_TABLE(of, max77759_charger_of_match_table);

static const struct i2c_device_id max77759_id[] = {
	{"max77759_charger", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77759_id);

#if defined CONFIG_PM
static int max77759_charger_pm_suspend(struct device *dev)
{
	/* TODO: is there anything to do here? */
	return 0;
}

static int max77759_charger_pm_resume(struct device *dev)
{
	/* TODO: is there anything to do here? */
	return 0;
}
#endif

static const struct dev_pm_ops max77759_charger_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(
		max77759_charger_pm_suspend,
		max77759_charger_pm_resume)
};

static struct i2c_driver max77759_charger_i2c_driver = {
	.driver = {
		.name = "max77759-charger",
		.owner = THIS_MODULE,
		.of_match_table = max77759_charger_of_match_table,
#ifdef CONFIG_PM
		.pm = &max77759_charger_pm_ops,
#endif
	},
	.id_table = max77759_id,
	.probe    = max77759_charger_probe,
	.remove   = max77759_charger_remove,
};

module_i2c_driver(max77759_charger_i2c_driver);

MODULE_DESCRIPTION("Maxim 77759 Charger Driver");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_LICENSE("GPL");
