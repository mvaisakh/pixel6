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

#ifndef MAX77759_H_
#define MAX77759_H_

#include <linux/i2c.h>

#define SCNPRINTF(...) scnprintf(__VA_ARGS__)
#include "max77759_regs.h"

#define MAX77759_CHG_INT_COUNT 2
#define MAX77759_PMIC_PMIC_ID_MW	0x3b

/*
 * b/156527175: workaround for read only MAX77759_CHG_DETAILS_03
 * MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7 is used to detect exit from fshipmode.
 * The register (MAX77759_PMIC_TOPSYS_INT_MASK) is type S and the bit is reset
 * to 1 on power loss. The reports MAX77759_CHG_DETAILS_03 when the bit
 * is 1 and report 0 when the bit is set to 0.
 */
#define MAX77759_FSHIP_EXIT_DTLS	  MAX77759_PMIC_TOPSYS_INT_MASK
#define MAX77759_FSHIP_EXIT_DTLS_RD \
				MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7
#define MAX77759_FSHIP_EXIT_DTLS_RD_SHIFT \
				MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7_SHIFT
#define MAX77759_FSHIP_EXIT_DTLS_RD_MASK \
				MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7_MASK
#define MAX77759_FSHIP_EXIT_DTLS_RD_CLEAR \
				MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7_CLEAR

int max777x9_pmic_reg_read(struct i2c_client *client,
			   u8 addr, u8 *val, int len);
int max777x9_pmic_reg_write(struct i2c_client *client,
			    u8 addr, const u8 *val, int len);
int max777x9_pmic_reg_update(struct i2c_client *client,
			     u8 reg, u8 mask, u8 value);

#if IS_ENABLED(CONFIG_PMIC_MAX77729)
extern int max77759_read_batt_conn(struct i2c_client *client, int *temp);
extern int max77759_read_usb_temp(struct i2c_client *client, int *temp);
extern int max77759_read_batt_id(struct i2c_client *client, unsigned int *id);
#else
static inline int max77759_read_batt_conn(struct i2c_client *client, int *temp)
{
	return -ENODEV;
}
static inline int max77759_read_usb_temp(struct i2c_client *client, int *temp)
{
	return -ENODEV;
}
static inline int max77759_read_batt_id(struct i2c_client *client,
					unsigned int *id)
{
	return -ENODEV;
}
#endif

/* mux configuration in MAX77759_PMIC_CONTROL_FG */
#define THMIO_MUX_BATT_PACK	0
#define THMIO_MUX_USB_TEMP	1
#define THMIO_MUX_BATT_ID	2

int max77759_read_batt_conn(struct i2c_client *client, int *temp);
int max77759_read_usb_temp(struct i2c_client *client, int *temp);

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

/*
 * GS101 usecases
 */

struct max77759_foreach_cb_data {
	struct gvotable_election *el;

	const char *reason;

	bool chgr_on;	/* CC_MAX != 0 */
	bool stby_on;	/* on disconnect */
	bool inflow_off;

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

struct max77759_usecase_data {
	int bst_on;	/* */
	int bst_sel;	/* */
	int ext_bst_ctl;/* MW VENDOR_EXTBST_CTRL */

	int ls2_en;	/* OVP LS2 */

	int vin_is_valid;	/* MAX20339 STATUS1.vinvalid */
	int lsw1_is_open;	/* MAX20339 STATUS2.lsw1open */
	int lsw1_is_closed;	/* MAX20339 STATUS2.lsw1closed */

	bool init_done;
};

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

/* internal system values */
enum {
	GBMS_CHGR_MODE_STBY_ON		= 0x10 + MAX77759_CHGR_MODE_ALL_OFF,
	GBMS_CHGR_MODE_INFLOW_OFF	= 0x11 + MAX77759_CHGR_MODE_ALL_OFF,
	GBMS_CHGR_MODE_CHGR_BUCK_ON	= 0x10 + MAX77759_CHGR_MODE_CHGR_BUCK_ON,
	GBMS_CHGR_MODE_BOOST_UNO_ON	= 0x10 + MAX77759_CHGR_MODE_BOOST_UNO_ON,
};


#endif
