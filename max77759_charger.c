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
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <misc/gvotable.h>
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "max_m5.h"
#include "max77759.h"

/* Hardware modes */
enum max77759_charger_modes {
	MAX77759_CHGR_MODE_ALL_OFF = 0x00,
	MAX77759_CHGR_MODE_BUCK_ON = 0x04,
	MAX77759_CHGR_MODE_CHGR_BUCK_ON = 0x05,
	MAX77759_CHGR_MODE_BOOST_UNO_ON = 0x08,
	MAX77759_CHGR_MODE_BOOST_ON = 0x09,
	MAX77759_CHGR_MODE_OTG_BOOST_ON = 0x0a,
	MAX77759_CHGR_MODE_BUCK_BOOST_UNO_ON = 0x0c,
	MAX77759_CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON = 0x0d,
	MAX77759_CHGR_MODE_OTG_BUCK_BOOST_ON = 0x0e,
	MAX77759_CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON = 0x0f,
};

/* internal system values */
enum {
	GBMS_CHGR_MODE_STBY_ON	    = 0x10 + MAX77759_CHGR_MODE_ALL_OFF,
	GBMS_CHGR_MODE_CHGR_BUCK_ON = 0x10 + MAX77759_CHGR_MODE_CHGR_BUCK_ON,
	GBMS_CHGR_MODE_BOOST_UNO_ON = 0x10 + MAX77759_CHGR_MODE_BOOST_UNO_ON,
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

/* for usecases */
struct max77759_usecase_data {
	int bst_on;	/* */
	int bst_sel;	/* */
	int ext_bst_ctl;	/* MW VENDOR_EXTBST_CTRL */

	bool init_done;
};

struct max77759_chgr_data {
	struct device *dev;
	struct power_supply *psy;
	struct power_supply *wcin_psy;
	struct power_supply *chgin_psy;

	struct power_supply *fg_psy;
	struct power_supply *wlc_psy;
	struct regmap *regmap;

	struct gvotable_election *mode_votable;
	enum gbms_charger_modes last_mode;
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

	atomic_t insel_cnt;

	struct mutex io_lock;
	bool resume_complete;
	bool init_complete;

	int use_case;

	int fship_dtls;
	bool online;
	bool wden;
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


struct max77759_foreach_cb_data {
	struct gvotable_election *el;

	const char *reason;

	bool chgr_on;	/* CC_MAX != 0 */
	bool stby_on;	/* on disconnect */

	bool buck_on;	/* wired power in (chgin_on) from TCPCI */

	bool otg_on;	/* power out, usually external */
	bool frs_on;	/* fast role swap */

	bool wlc_on;	/* charging wireless */
	bool wlc_tx;	/* battery share */

	bool pps_dc;	/* PPS enabled - wired */
	bool wlc_dc;	/* PPS enabled - wireless */

	bool boost_on;	/* old for WLC program */
	bool uno_on;	/* old for WLC program */

	u8 raw_value;	/* hard override */
	bool use_raw;

	u8 reg;
};

/* First step to convert votes to a usecase and a setting for mode */
static int max77759_foreach_callback(void *data, const char *reason,
				     void *vote)
{
	struct max77759_foreach_cb_data *cb_data = data;
	int mode = (int)vote; /* max77759_mode is an int election */

	pr_info("%s: %s : reason=%s mode=%x\n", __func__,
		 cb_data->reason, reason ? reason : "", mode);

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
		pr_info("%s:%d RAW vote=%x\n", __func__, __LINE__, mode);
		cb_data->raw_value = mode;
		cb_data->reason = reason;
		cb_data->use_raw = true;
		break;

	/* temporary, can be used to program the WLC chip, remove */
	case GBMS_CHGR_MODE_BOOST_UNO_ON:
		if (!cb_data->boost_on || !cb_data->uno_on)
			cb_data->reason = reason;
		pr_info("%s:%d BOOST_UNO vote=%x\n", __func__, __LINE__, mode);
		cb_data->boost_on += 1;
		cb_data->uno_on += 1;
		break;

	/* SYSTEM modes can add complex transactions */

	/* MAX77759: on disconnect */
	case GBMS_CHGR_MODE_STBY_ON:
		if (!cb_data->stby_on)
			cb_data->reason = reason;
		pr_info("%s:%d FORCE_OFF vote=%x\n", __func__, __LINE__, mode);
		cb_data->stby_on += 1;
		break;

	/* MAX77759: charging on via CC_MAX (needs inflow, buck_on on) */
	case GBMS_CHGR_MODE_CHGR_BUCK_ON:
		if (!cb_data->chgr_on)
			cb_data->reason = reason;
		pr_info("%s:%d CHGR_BUCK_ON vote=%x\n", __func__, __LINE__, mode);
		cb_data->chgr_on += 1;
		break;

	/* USB: inflow, actual charging controlled via BUCK_ON */
	case GBMS_USB_BUCK_ON:
		if (!cb_data->buck_on)
			cb_data->reason = reason;
		pr_info("%s:%d BUCK_ON vote=%x\n", __func__, __LINE__, mode);
		cb_data->buck_on += 1;
		break;
	/* USB: OTG, source, fast role swap case */
	case GBMS_USB_OTG_FRS_ON:
		if (!cb_data->frs_on)
			cb_data->reason = reason;
		pr_info("%s:%d FRS_ON vote=%x\n", __func__, __LINE__, mode);
		cb_data->frs_on += 1;
		break;
	/* USB: boost mode, source, normally external boost */
	case GBMS_USB_OTG_ON:
		if (!cb_data->otg_on)
			cb_data->reason = reason;
		pr_info("%s:%d OTG_ON vote=%x\n", __func__, __LINE__, mode);
		cb_data->otg_on += 1;
		break;
	/* DC Charging: mode=0, set CP_EN */
	case GBMS_CHGR_MODE_CHGR_DC:
		if (!cb_data->pps_dc)
			cb_data->reason = reason;
		pr_info("%s:%d DC_ON vote=%x\n", __func__, __LINE__, mode);
		cb_data->pps_dc += 1;
		break;

	default:
		pr_info("mode=%x not supported\n", mode);
		break;
	}

	return 0;
}

/*
 *  Use cases, these are platform specific and need to be outside the driver.
 * Case	USB_chg USB_otg	WLC_chg	WLC_TX	PMIC_Charger	Ext_B	LSx	Name
 * ----------------------------------------------------------------------------
 * 1-1	1	0	x	0	IF-PMIC-VBUS	0	0/0	USB_CHG
 * 1-2	2	0	x	0	DC VBUS		0	0/0	USB_DC
 * 2-1	1	0	0	1	IF-PMIC-VBUS	2	0/1	USB_CHG_WLC_TX
 * 2-2	2	0	0	1	DC CHG		2	0/1	USB_DC_WLC_TX
 * 3-1	0	0	1	0	IF-PMIC-WCIN	0	0/0	WLC_RX
 * 3-2	0	0	2	0	DC WCIN		0	0/0	WLC_DC
 * 4-1	0	1	1	0	IF-PMIC-WCIN	1	1/0	USB_OTG_WLC_RX
 * 4-2	0	1	2	0	DC WCIN		1	1/0	USB_OTG_WLC_DC
 * 5-1	0	1	0	0	0		1	1/0	USB_OTG
 * 5-2	0	1	0	0	OTG 5V		0	0/0	USB_OTG_FRS
 * 6-2	0	0	0	1	0		2	0/1	WLC_TX
 * 7-2	0	1	0	1	MW OTG 5V	2	0/1	USB_OTG_WLC_TX
 * 8	0	0	0	0	0		0	0/0	IDLE
 * ----------------------------------------------------------------------------
 *
 * Ext_Boost = 0 off, 1 = OTG 5V, 2 = WTX 7.5
 * USB_chg = 0 off, 1 = on, 2 = PPS
 * WLC_chg = 0 off, 1 = on, 2 = PPS
 */

enum gsu_usecases {
	GSU_RAW_MODE 		= -1,	/* raw mode, default, */

	GSU_MODE_STANDBY	= 0,	/* 8, PMIC mode 0 */
	GSU_MODE_USB_CHG	= 1,	/* 1-1 wired mode 0x4, mode 0x5 */
	GSU_MODE_USB_DC 	= 2,	/* 1-2 wired mode 0x0 */
	GSU_MODE_USB_CHG_WLC_TX = 3,	/* 2-1, 1041, */
	GSU_MODE_USB_DC_WLC_TX	= 4,	/* 2-2 1042, */

	GSU_MODE_WLC_RX		= 5,	/* 3-1, mode 0x4, mode 0x5 */
	GSU_MODE_WLC_DC		= 6,	/* 3-2, mode 0x0 */

	GSU_MODE_USB_OTG_WLC_RX = 7,	/* 4-1, 524, */
	GSU_MODE_USB_OTG_WLC_DC = 8,	/* 4-2, 532, */
	GSU_MODE_USB_OTG 	= 9,	/* 5-1, 516,*/
	GSU_MODE_USB_OTG_FRS	= 10,	/* 5-2, PMIC mode 0x0a */

	GSU_MODE_WLC_TX 	= 11,	/* 6-2, 1056, */
	GSU_MODE_USB_OTG_WLC_TX	= 12,	/* 7-2, 1060, */
};


/* control VENDOR_EXTBST_CTRL (from TCPCI module) */
static int max77759_ls_mode(struct max77759_chgr_data *data, int mode)
{
	int ret = 0;

	pr_info("%s: mode=%d ext_bst_ctl=%d\n", __func__, mode,
		data->uc_data.ext_bst_ctl);

	if (data->uc_data.ext_bst_ctl < 0)
		return 0;

	switch (mode) {
	case 0:
		gpio_set_value_cansleep(data->uc_data.ext_bst_ctl, 0);
		break;
	case 1:
		gpio_set_value_cansleep(data->uc_data.ext_bst_ctl, 1);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

/* control external boost mode
 * can be done controlling ls1, ls2
 */
#define EXT_MODE_OFF		0
#define EXT_MODE_OTG_5_0V	1
#define EXT_MODE_OTG_7_5V	2

/* GPIO5 on Max77759 on canopy and on all whitefins */
static int max77759_ext_mode(struct max77759_chgr_data *data, int mode)
{
	int ret = 0;

	pr_info("%s: mode=%d on=%d sel=%d\n", __func__, mode,
		data->uc_data.bst_on, data->uc_data.bst_sel);

	if (data->uc_data.bst_on < 0 || data->uc_data.bst_sel < 0)
		return 0;

	switch (mode) {
	case EXT_MODE_OFF:
		gpio_set_value_cansleep(data->uc_data.bst_on, 0);
		break;
	case EXT_MODE_OTG_5_0V:
		gpio_set_value_cansleep(data->uc_data.bst_sel, 0);
		mdelay(100);
		gpio_set_value_cansleep(data->uc_data.bst_on, 1);
		break;
	case EXT_MODE_OTG_7_5V: /* TODO: verify this */
		gpio_set_value_cansleep(data->uc_data.bst_sel, 1);
		mdelay(100);
		gpio_set_value_cansleep(data->uc_data.bst_on, 1);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

/*
 * Transition to standby (if needed) at the beginning of the sequences
 * @return <0 on error, 0 on success. ->use_case becomes GSU_MODE_STANDBY
 * if the transition is necessary (and successful).
 */
static int max77759_to_standby(struct max77759_chgr_data *data, int use_case)
{
	bool need_stby = false;
	int ret;

	switch (data->use_case) {
		case GSU_MODE_USB_CHG:
			need_stby = use_case != GSU_MODE_USB_CHG_WLC_TX &&
				    use_case != GSU_MODE_WLC_RX &&
				    use_case != GSU_MODE_USB_DC;
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
		case GSU_MODE_USB_OTG: 	/* From 5. USB OTG to 8. standby */


			if (use_case == GSU_MODE_USB_OTG_FRS)
				break;

			/* missing setting EXT_BST_EN in MW (TCPM) */

			ret = max77759_ext_mode(data, 0);
			if (ret < 0) {
				dev_err(data->dev, "cannot turn off ext (%d)\n",
					ret);
				return -EIO;
			}

			/* missing Discharge IN/OUT nodes with AO37  */
			mdelay(100);

			need_stby = true;
			break;

		case GSU_MODE_USB_OTG_FRS:

			if (use_case == GSU_MODE_USB_OTG)
				break;

			need_stby = true;
			break;
		case GSU_RAW_MODE:
			need_stby = true;
			break;
		/* no need to transition to stby for these modes */
		case GSU_MODE_USB_OTG_WLC_RX:
		case GSU_MODE_USB_OTG_WLC_TX:
		case GSU_MODE_STANDBY:
		default:
			need_stby = false;
			break;
	}

	pr_info("%s: use_case=%d->%d need_stby=%x\n", __func__,
		data->use_case, use_case, need_stby);

	if (!need_stby)
		return 0;

	if (data->use_case == GSU_MODE_WLC_TX) {
		/* not yet */
	}

	/* transition to STBY (might need to be up) */
	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_00, 0);
	if (ret < 0) {
		dev_err(data->dev, "cannot reset mode (%d)\n", ret);
		return -EIO;
	}

	data->use_case = GSU_MODE_STANDBY;
	return 0;
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
 * NOTE: do not call with (cb_data->wlc_on && cb_data->wlc_tx)
 */
static int max77759_to_otg_usecase(struct max77759_chgr_data *data,
				   int use_case, u8 reg)
{
	int ret = 0;

	if (!data->uc_data.init_done)
		return -EPROBE_DEFER;

	if (data->use_case == GSU_MODE_STANDBY) {
		/* 5-1: #3: stby to USB OTG, mode = 1 */
		/* 5-2: #3: stby to USB OTG_FRS, mode = 0 */
		const int mode = use_case == GSU_MODE_USB_OTG_FRS ?
					     EXT_MODE_OFF :
					     EXT_MODE_OTG_5_0V;

		/* Write 0b11 to IN_CTR(0x10).INSwEn[1:0] */
		/* Write 0b1 to AO37 SwCntl (0xA).LSw1En */
		/* TCPCM controls EXT_BST_EN? */
		ret = max77759_ls_mode(data, 1);
		if (ret == 0)
			ret = max77759_ext_mode(data, mode);

	} else if (data->use_case == GSU_MODE_USB_OTG) {
		/*
		 * OTG source handover: OTG -> OTG_FRS
		 * from IF-PMIC OTG (FRS OTG) to EXT_BST (regular OTG)
		 */
	} else if (data->use_case == GSU_MODE_USB_OTG_FRS) {
		/*
		 * OTG source handover: OTG_FRS -> OTG
		 * from EXT_BST (Regular OTG) to IF-PMIC OTG (FRS OTG)
		 */
	}

	return ret;
}

/* handles the transition data->use_case ==> use_case */
static int max77759_to_usecase(struct max77759_chgr_data *data,
			       int use_case, u8 reg)
{
	int ret = 0;

	pr_info("%s: use_case=%d->%d reg=%x\n", __func__,
		data->use_case, use_case, reg);

	switch (use_case) {
	case GSU_MODE_USB_OTG:
	case GSU_MODE_USB_OTG_FRS:
		ret = max77759_to_otg_usecase(data, use_case, reg);
		if (ret < 0)
			return ret;
		break;
	case GSU_MODE_WLC_TX:
		break;
	case GSU_RAW_MODE:
		/* just write the value to the register */
		break;
	default:
		break;
	}

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_00, reg);
	if (ret < 0) {
		dev_err(data->dev, "cannot set CNFG_00 (%d)\n", ret);
		return -EIO;
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
 * NOTE: do not call with (cb_data->wlc_on && cb_data->wlc_tx)
 */
static int max77759_get_otg_usecase(struct max77759_foreach_cb_data *cb_data)
{
	int usecase;
	u8 mode;

	/* invalid, not with USB power */
	if (cb_data->buck_on) {
		pr_err("%s: buck_on with OTG\n", __func__);
		return -EINVAL;
	}

	/* pure OTG default to FRS */
	if (!cb_data->wlc_on && !cb_data->wlc_tx) {
		/* 5-1: USB_OTG or  5-2: USB_OTG_FRS */

		/* HACK: force FRS */
		//cb_data->frs_on = 1;

		if (cb_data->frs_on) {
			usecase = GSU_MODE_USB_OTG_FRS;
			mode = MAX77759_CHGR_MODE_OTG_BOOST_ON;
		} else {
			usecase = GSU_MODE_USB_OTG;
			mode = MAX77759_CHGR_MODE_ALL_OFF;
		}

		if (cb_data->pps_dc)
			pr_err("%s: charge pump on with OTG\n", __func__);
		cb_data->pps_dc = 0;
	} else if (cb_data->wlc_tx) {
		/* 7-2 */
		usecase = GSU_MODE_USB_OTG_WLC_TX;
		pr_err("%s: GSU_MODE_WLC_TX not yet\n", __func__);

		return -EINVAL;
	} else if (cb_data->wlc_dc) {
		/* 4-2, maybe use pps_dc */

		pr_err("%s: GSU_MODE_WLC_TX not yet\n", __func__);

		usecase = GSU_MODE_USB_OTG_WLC_DC;
		mode = MAX77759_CHGR_MODE_ALL_OFF;

		/* TODO: enable Ext_B 5V  */
		return -EINVAL;
	} else {

		usecase = GSU_MODE_USB_OTG_WLC_RX;
		mode = MAX77759_CHGR_MODE_ALL_OFF;

		pr_debug("%s: chgr_on with OTG\n", __func__);

	}

	cb_data->reg = _chg_cnfg_00_cp_en_set(cb_data->reg, cb_data->pps_dc);
	cb_data->reg = _chg_cnfg_00_mode_set(cb_data->reg, mode);
	return usecase;
}

/*
 * Determines the use case to switch to. This is device/system dependent and
 * will likely be  factored to a separate file (compile module).
 */
static int max77759_get_usecase(struct max77759_foreach_cb_data *cb_data)
{
	int usecase;
	u8 mode;

	/* consistency check, TOD: add more */
	if (cb_data->wlc_tx && cb_data->wlc_on) {
		pr_err("%s: wlc_tx and wlc_rx\n", __func__);
		return -EINVAL;
	}

	/* OTG modes override the others */
	if (cb_data->otg_on || cb_data->frs_on)
		return max77759_get_otg_usecase(cb_data);

	/* buck_on is wired, wlc_on is wireless */
	if (!cb_data->buck_on && !cb_data->wlc_on) {
		mode = MAX77759_CHGR_MODE_ALL_OFF;
		usecase = GSU_MODE_STANDBY;
	} else {
		/* MODE_BUCK_ON is inflow */
		if (cb_data->chgr_on) {
			mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
			usecase = GSU_MODE_USB_CHG;
		} else {
			mode = MAX77759_CHGR_MODE_BUCK_ON;
			usecase = GSU_MODE_USB_CHG;
		}

		/* direct charging or off mode */
		if (cb_data->pps_dc) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_USB_DC;
		} else if (cb_data->wlc_dc) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_WLC_DC;
		} else if (cb_data->stby_on) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_STANDBY;
		}

	}

	/* reg might be ignored later */
	cb_data->reg = _chg_cnfg_00_cp_en_set(cb_data->reg, cb_data->pps_dc);
	cb_data->reg = _chg_cnfg_00_mode_set(cb_data->reg, mode);

	return usecase;
}

/* switch to a use case, handle the transitions */
static int max77759_set_usecase(struct max77759_chgr_data *data, u8 reg,
				int use_case)
{
	int ret;

	/* transition to STBY if requested from the use case. */
	ret = max77759_to_standby(data, use_case);
	if (ret < 0)
		return ret;

	/* transition from data->use_case to use_case */
	ret = max77759_to_usecase(data, use_case, reg);
	if (ret < 0)
		return ret;

	return 0;
}

static int max77759_wcin_is_online(struct max77759_chgr_data *data);

/* lazy init on the */
static bool max77759_setup_usecases(struct max77759_usecase_data *uc_data,
				    struct device_node *node)
{

	if (!node) {
		uc_data->init_done = false;
		uc_data->bst_on = -EPROBE_DEFER;
		uc_data->bst_sel = -EPROBE_DEFER;
		uc_data->ext_bst_ctl = -EPROBE_DEFER;
		return 0;
	}

	if (uc_data->bst_on == -EPROBE_DEFER)
		uc_data->bst_on = of_get_named_gpio(node, "max77759,bst-on", 0);

	if (uc_data->bst_sel == -EPROBE_DEFER)
		uc_data->bst_sel = of_get_named_gpio(node, "max77759,bst-sel", 0);

	if (uc_data->ext_bst_ctl == -EPROBE_DEFER)
		uc_data->ext_bst_ctl = of_get_named_gpio(node, "max77759,extbst-ctl", 0);

	return uc_data->bst_on != -EPROBE_DEFER &&
	       uc_data->bst_sel != -EPROBE_DEFER &&
	       uc_data->ext_bst_ctl != -EPROBE_DEFER;
}

/*
 * I am using a the comparator_none, need scan all the votes to determine
 * the actual.
 */
static void max77759_mode_callback(struct gvotable_election *el,
				   const char *reason, void *value)
{
	struct max77759_chgr_data *data = gvotable_get_data(el);
	struct max77759_usecase_data *uc_data = &data->uc_data;
	struct max77759_foreach_cb_data cb_data = { 0 };
	int use_case, ret;
	u8 reg;

	/* reason and value are the last voted on */
	pr_debug("%s: current reason=%s, value=0x%x\n", __func__,
		 reason ? reason : "<>", (int)value);

	mutex_lock(&data->io_lock);

	/* wait for usecases */
	if (!uc_data->init_done) {
		uc_data->init_done = max77759_setup_usecases(uc_data, data->dev->of_node);
		dev_info(data->dev, "bst_on:%d, bst_sel:%d, ext_bst_ctl:%d",
			 uc_data->bst_on, uc_data->bst_sel, uc_data->ext_bst_ctl);
	}

	/* no caching */
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_00, &reg);
	if (ret < 0) {
		dev_err(data->dev, "cannot read CNFG_00 (%d)\n", ret);
		goto unlock_done;
	}

	/* this is the last vote of the election */
	cb_data.reg = reg;	/* current */
	cb_data.el = el;	/* election */
	cb_data.wlc_on = max77759_wcin_is_online(data);

	/* now scan all the reasons, accumulate in cb_data */
	gvotable_election_for_each(el, max77759_foreach_callback, &cb_data);

	dev_info(data->dev, "max77759_charger: CHARGER_MODE=%d reason=%s reg:%x\n",
		 cb_data.raw_value, cb_data.reason ? cb_data.reason : "",
		 reg);

	dev_info(data->dev, "%s: raw=%d stby_on=%d, pps_dc=%d, chgr_on=%d, buck_on=%d, "
		"boost_on=%d, otg_on=%d, uno_on=%d\n", __func__,
		cb_data.use_raw, cb_data.stby_on, cb_data.pps_dc,
		cb_data.chgr_on, cb_data.buck_on, cb_data.boost_on,
		cb_data.otg_on, cb_data.uno_on);

	/* just use raw as is*/
	if (cb_data.use_raw) {
		cb_data.reg = cb_data.raw_value;
		use_case = GSU_RAW_MODE;
	} else {
		/* figure out next use case if not in raw mode */
		use_case = max77759_get_usecase(&cb_data);
		if (use_case < 0) {
			dev_err(data->dev, "max77759_charger: no valid use case %d\n",
				use_case);
			goto unlock_done;
		}
	}

	dev_info(data->dev, "%s: use_case=%x->%x reg=%x->%x\n", __func__,
		data->use_case, use_case, reg, cb_data.reg);

	/* TODO: state machine that handle transition between states */
	ret = max77759_set_usecase(data, cb_data.reg, use_case);
	if (ret < 0) {
		dev_err(data->dev, "%s: use_case=%x->%x reg=%x->%x ret=%d\n",
			__func__, data->use_case, use_case, reg, cb_data.reg,
			ret);
		goto unlock_done;
	}

	/* the election is an int election */
	if (cb_data.reason)
		reason = cb_data.reason;
	if (!reason)
		reason = "<>";

	ret = gvotable_election_set_result(el, reason,
					(void*)(uintptr_t)cb_data.reg);
	if (ret < 0) {
		dev_err(data->dev, "max77759_charger: cannot update election %d\n",
			ret);
		goto unlock_done;
	}

	/* mode */
	data->use_case = use_case;

unlock_done:
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

/* called from gcpm, DC_SUSPEND and for CC_MAX == 0 */
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

/* turn off CHGIN_INSEL: works when max77559 registers are not protected */
static int max77759_chgin_input_suspend(struct max77759_chgr_data *data,
					bool enabled, const char *reason)
{
	const u8 value = (!enabled) << MAX77759_CHG_CNFG_12_CHGINSEL_SHIFT;

	data->chgin_input_suspend = enabled; /* cache */

	return max77759_reg_update(data, MAX77759_CHG_CNFG_12,
				   MAX77759_CHG_CNFG_12_CHGINSEL_MASK,
				   value);
}

/* turn off WCIN_INSEL: works when max77559 registers are not protected */
static int max77759_wcin_input_suspend(struct max77759_chgr_data *data,
				       bool enabled, const char *reason)
{
	const u8 value = (!enabled) << MAX77759_CHG_CNFG_12_WCINSEL_SHIFT;

	data->wcin_input_suspend = enabled; /* cache */

	return max77759_reg_update(data, MAX77759_CHG_CNFG_12,
				   MAX77759_CHG_CNFG_12_WCINSEL_MASK,
				   value);
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
	else if (current_ua > 4000000)
		value = 0x3F;
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
	int ret, suspend = (int)value > 0;

	ret = max77759_wcin_input_suspend(data, suspend, "DC_SUSPEND");
	if (ret < 0)
		return;

	/* enable charging when DC_SUSPEND is not set */
	ret = max77759_set_charge_enabled(data, !suspend, "DC_SUSPEND");

	dev_info(data->dev, "DC_SUSPEND reason=%s, value=%d suspend=%d (%d)\n",
		reason ? reason : "", (int)value, suspend, ret);
}

static void max77759_dcicl_callback(struct gvotable_election *el,
				    const char *reason,
				    void *value)
{
	struct max77759_chgr_data *data = gvotable_get_data(el);
	int dc_icl = (int)value;
	const bool suspend = dc_icl == 0;
	int ret;

	pr_debug("%s: dc_icl=%d suspend=%d (%d)\n", __func__,
		 dc_icl, suspend, ret);

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
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static int max77759_wcin_is_present(struct max77759_chgr_data *data)
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

	ret = max77759_wcin_is_present(data) &&
	      max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02, &val);
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

	if (!max77759_wcin_is_present(chg)) {
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

	if (!max77759_wcin_is_present(chg)) {
		val->intval = 0;
		return 0;
	}

	rc = power_supply_get_property(chg->wlc_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get VOLTAGE_NOW, rc=%d\n", rc);

	return rc;
}

static int max77759_wcin_get_prop(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct max77759_chgr_data *chgr = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = max77759_wcin_is_present(chgr);
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
	.type = POWER_SUPPLY_TYPE_WIRELESS_EXT,
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
	if (IS_ERR(data->wcin_psy)) {
		pr_err("Couldn't register wlc power supply\n");
		return PTR_ERR(data->wcin_psy);
	}

	return 0;
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

static int max77759_is_present(struct max77759_chgr_data *data)
{
	uint8_t int_ok;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	return (ret == 0) && (_chg_int_ok_chgin_ok_get(int_ok) ||
	       _chg_int_ok_wcin_ok_get(int_ok));
}

static int max77759_is_online(struct max77759_chgr_data *data)
{
	uint8_t val;
	int ret;

	ret = max77759_is_present(data) &&
	      max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02, &val);
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
			return POWER_SUPPLY_STATUS_CHARGING;
		case CHGR_DTLS_TOP_OFF_MODE:
		case CHGR_DTLS_DONE_MODE:
			/* same as POWER_SUPPLY_PROP_CHARGE_DONE */
			return POWER_SUPPLY_STATUS_FULL;
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

static int max77759_read_from_fg(struct max77759_chgr_data *data,
				 enum power_supply_property psp,
				 union power_supply_propval *pval)
{
	int ret = -ENODEV;

	if (!data->fg_psy)
		data->fg_psy = power_supply_get_by_name("maxfg");
	if (data->fg_psy)
		ret = power_supply_get_property(data->fg_psy, psp, pval);
	if (ret < 0)
		pval->intval = -1;
	return 0;
}

static int max77759_get_chg_chgr_state(struct max77759_chgr_data *data,
				       union gbms_charger_state *chg_state)
{
	int usb_present, usb_valid, dc_present, dc_valid;
	union power_supply_propval pval;
	const char *source = "";
	uint8_t int_ok, dtls;
	int icl = 0;
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

	rc = max77759_read_from_fg(data, POWER_SUPPLY_PROP_VOLTAGE_NOW,
				   &pval);
	if (rc == 0)
		chg_state->f.vchrg = pval.intval / 1000;

	if (chg_state->f.chg_status == POWER_SUPPLY_STATUS_DISCHARGING)
		goto exit_done;

	rc = max77759_is_limited(data);
	if (rc > 0)
		chg_state->f.flags |= GBMS_CS_FLAG_ILIM;

	/* TODO: b/ handle input MUX corner cases */
	if (usb_valid) {
		max77759_chgin_get_ilim_max_ua(data, &icl);
		source = dc_present ? "Uw" : "u";
	} else if (dc_valid) {
		max77759_wcin_get_ilim_max_ua(data, &icl);
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

static int max77759_find_pmic(struct max77759_chgr_data *data)
{
	if (!data->pmic_i2c_client) {
		struct device_node *dn;

		dn = of_parse_phandle(data->dev->of_node, "max77759,pmic", 0);
		if (!dn)
			return -ENXIO;

		data->pmic_i2c_client = of_find_i2c_device_by_node(dn);
		if (!data->pmic_i2c_client)
			return -EAGAIN;
	}

	return !!data->pmic_i2c_client;
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

/* only valid in mode 5, 6, 7, e, f */
static int max77759_read_iic_from_fg(struct max77759_chgr_data *data, int *iic)
{
	int ret;

	ret = max77759_find_fg(data);
	if (ret < 0)
		return ret;

	return max_m5_read_actual_input_current_ua(data->fg_i2c_client, iic);
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


static int max77759_set_online(struct max77759_chgr_data *data, bool online)
{
	int ret;

	ret = max77759_wd_tickle(data);
	if (ret < 0)
		pr_err("cannot tickle the watchdog\n");

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
		pval->intval = max77759_is_present(data);
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
		rc = max77759_read_from_fg(data, psp, pval);
		if (rc < 0)
			dev_err(data->dev, "cannot read voltage now=%d\n", rc);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = max77759_read_iic_from_fg(data, &pval->intval);
		if (rc < 0)
			pval->intval = -1;
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

static int sys_uvlo1_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg15;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_15, &chg_cnfg15);
	if (ret < 0)
		return -ENODEV;

	*val = chg_cnfg15;
	return 0;
}

static int sys_uvlo1_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_15, (u8) val);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(sys_uvlo1_fops, sys_uvlo1_get, sys_uvlo1_set, "0x%x\n");

static int sys_uvlo2_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg16;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_16, &chg_cnfg16);
	if (ret < 0)
		return -ENODEV;

	*val = chg_cnfg16;
	return 0;
}

static int sys_uvlo2_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_16, (u8) val);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(sys_uvlo2_fops, sys_uvlo2_get, sys_uvlo2_set, "0x%x\n");


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

	data->de = debugfs_create_dir("max77759_chg", 0);
	if (IS_ERR_OR_NULL(data->de))
		return -EINVAL;

	debugfs_create_atomic_t("sysuvlo1_cnt", 0644, data->de,
				&data->sysuvlo1_cnt);
	debugfs_create_atomic_t("sysuvlo2_cnt", 0644, data->de,
				&data->sysuvlo2_cnt);
	debugfs_create_atomic_t("insel_cnt", 0644, data->de,
				&data->insel_cnt);

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
	debugfs_create_file("input_mask_clear", 0600, data->de, data,
			    &input_mask_clear_fops);

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

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_INSEL_M) {
		ret = max77759_chgr_input_mask_clear(data);
		if (ret < 0)
			pr_info("INT : %x %x : clear=%d\n",
				chg_int[0], chg_int[1], ret);
		else
			atomic_inc(&data->insel_cnt);
	}

	pr_debug("INT : %x %x\n", chg_int[0], chg_int[1]);

	if (chg_int[1] & MAX77759_CHG_INT2_SYS_UVLO1_I)
		atomic_inc(&data->sysuvlo1_cnt);

	if (chg_int[1] & MAX77759_CHG_INT2_SYS_UVLO2_I)
		atomic_inc(&data->sysuvlo2_cnt);

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

		if (data->chgin_psy)
			power_supply_changed(data->chgin_psy);
		else
			power_supply_changed(data->psy);
	}

	/* wireless input is changed */
	if (data->wcin_psy && (chg_int[0] & MAX77759_CHG_INT_MASK_WCIN_M))
		power_supply_changed(data->wcin_psy);

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

	/* initialized to -EPROBE_DEFER */
	max77759_setup_usecases(&data->uc_data, NULL);

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

static int max77759_charger_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct power_supply_config chgr_psy_cfg = { 0 };
	struct device *dev = &client->dev;
	struct max77759_chgr_data *data;
	struct regmap *regmap;
	const char *psy_name;
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
	atomic_set(&data->insel_cnt, 0);
	i2c_set_clientdata(client, data);

	ret = of_property_read_string(dev->of_node, "max77759,psy-name",
				      &psy_name);
	if (ret == 0)
		max77759_psy_desc.name = devm_kstrdup(dev, psy_name,
						      GFP_KERNEL);

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

	/* other drivers (ex tcpci) need this. */
	ret = max77759_setup_votables(data);
	if (ret < 0)
		return ret;

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

	data->init_complete = 1;
	data->resume_complete = 1;

	dev_info(dev, "registered as %s\n", max77759_psy_desc.name);
	max77759_init_wcin_psy(data);
	return 0;
}

static int max77759_charger_remove(struct i2c_client *client)
{
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
