/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Platform data for the NXP PCA9468 battery charger driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PCA9468_CHARGER_H_
#define _PCA9468_CHARGER_H_

#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/thermal.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>

/* Google integration */
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "google_dc_pps.h"

struct pca9468_platform_data {
	int	irq_gpio;		/* GPIO pin that's connected to INT# */
	unsigned int	iin_cfg;	/* Input Current Limit - uA unit */
	unsigned int 	ichg_cfg;	/* Charging Current - uA unit */
	unsigned int	v_float;	/* V_Float Voltage - uV unit */
	unsigned int 	iin_topoff;	/* Input Topoff current -uV unit */
	/* Switching frequency: 0 - 833kHz, ... , 3 - 980kHz */
	unsigned int 	fsw_cfg;
	/* NTC voltage threshold : 0~2.4V - uV unit */
	unsigned int	ntc_th;
	/*
	 * Default charging mode:
	 *	0 - No direct charging
	 *	1 - 2:1 charging mode
	 *	2 - 4:1 charging mode
	 */
	unsigned int	chg_mode;

#ifdef CONFIG_THERMAL
	const char *usb_tz_name;
#endif
};

/* - PPS Integration Shared definitions ---------------------------------- */

/**
 * struct pca9468_charger - pca9468 charger instance
 * @monitor_wake_lock: lock to enter the suspend mode
 * @lock: protects concurrent access to online variables
 * @dev: pointer to device
 * @regmap: pointer to driver regmap
 * @mains: power_supply instance for AC/DC power
 * @dc_wq: work queue for the algorithm and monitor timer
 * @timer_work: timer work for charging
 * @timer_id: timer id for timer_work
 * @timer_period: timer period for timer_work
 * @last_update_time: last update time after sleep
 * @pps_index: psy index
 * @tcpm_psy_name: name of TCPM power supply
 * @tcpm_phandle: lookup for tcpm power supply
 * @pps_work: pps work for PPS periodic time
 * @pps_data: internal data for dc_pps
 * @pd: phandle for qualcomm PMI usbpd-phy
 * @wlc_psy_name: power supply for wlc DC
 * @wlc_psy: wlc DC ps
 * @mains_online: is AC/DC input connected
 * @charging_state: direct charging state
 * @ret_state: return direct charging state after DC_STATE_ADJUST_TAVOL is done
 * @iin_cc: input current for the direct charging in cc mode, uA
 * @ta_cur: AC/DC(TA) current, uA
 * @ta_vol: AC/DC(TA) voltage, uV
 * @ta_objpos: AC/DC(TA) PDO object position
 * @ta_max_cur: TA maximum current of APDO, uA
 * @ta_max_vol: TA maximum voltage for the direct charging, uV
 * @ta_max_pwr: TA maximum power, uW
 * @prev_iin: Previous IIN ADC of PCA9468, uA
 * @prev_inc: Previous TA voltage or current increment factor
 * @req_new_iin: Request for new input current limit, true or false
 * @req_new_vfloat: Request for new vfloat, true or false
 * @fv_uv: requested float voltage
 * @cc_max: requested charge current max
 * @new_iin: New request input current limit, uA
 * @new_vfloat: New request vfloat, uV
 * @adc_comp_gain: adc gain for compensation
 * @retry_cnt: retry counter for re-starting charging if charging stop happens
 * @ta_type: TA type for the direct charging, USBPD TA or Wireless Charger.
 * @chg_mode: supported DC charging mode 2:1 or 4:1 mode
 * @pdata: pointer to platform data
 * @usb_tzd: device for thermal zone
 * @debug_root: debug entry
 * @debug_address: debug register address
 * @debug_adc_channel: ADC channel to read
 * @init_done: true when initialization is complete
 * @dc_start_time: start time (sec since boot) of the DC session
 * @irdrop_comp_ok: when true clear GBMS_CS_FLAG_NOCOMP in flags
 */
struct pca9468_charger {
	struct wakeup_source	*monitor_wake_lock;
	struct mutex		lock;
	struct device		*dev;
	struct regmap		*regmap;
	struct power_supply	*mains;

	struct workqueue_struct	*dc_wq;
	struct delayed_work	timer_work;
	unsigned int		timer_id;
	unsigned long		timer_period;
	unsigned long		last_update_time;

	bool			mains_online;
	unsigned int 		charging_state;
	unsigned int		ret_state;

	unsigned int		iin_cc;

	unsigned int		ta_cur;
	unsigned int		ta_vol;
	unsigned int		ta_objpos;

	/* same as pps_data */
	unsigned int		ta_max_cur;
	unsigned int		ta_max_vol;
	unsigned long		ta_max_pwr;

	unsigned int		prev_iin;
	unsigned int		prev_inc;

	bool			req_new_iin;
	bool			req_new_vfloat;

	unsigned int		new_iin;
	unsigned int		new_vfloat;

	int			adc_comp_gain;

	int			retry_cnt;

	struct pca9468_platform_data *pdata;

/* Google Integration Start */
	int pps_index;		/* 0=disabled, 1=tcpm, 2=wireless */

	/* PPS_wireless */
	const char 		*wlc_psy_name;
	struct power_supply 	*wlc_psy;
	/*  PPS_wired with TCPM */
	u32			tcpm_phandle;
	const char 		*tcpm_psy_name;
	struct power_supply 	*pd;
	struct delayed_work	pps_work;
	struct pd_pps_data	pps_data;

#ifdef CONFIG_THERMAL
	struct thermal_zone_device *usb_tzd;
#endif
	int			ta_type;
	unsigned int		chg_mode;
	/* requested charging current and voltage */
	int			fv_uv;
	int			cc_max;
	struct power_supply	*batt_psy;

	/* debug */
	struct dentry		*debug_root;
	u32			debug_address;
	int			debug_adc_channel;

	bool			init_done;

	ktime_t dc_start_time;
	bool irdrop_comp_ok;
/* Google Integration END */
};

/* Direct Charging State */
enum {
	DC_STATE_NO_CHARGING,	/* No charging */
	DC_STATE_CHECK_VBAT,	/* Check min battery level */
	DC_STATE_PRESET_DC, 	/* Preset TA voltage/current for DC */
	DC_STATE_CHECK_ACTIVE,	/* Check active status before Adjust CC mode */
	DC_STATE_ADJUST_CC,	/* Adjust CC mode */
	DC_STATE_CC_MODE,	/* Check CC mode status */
	DC_STATE_START_CV,	/* Start CV mode */
	DC_STATE_CV_MODE,	/* Check CV mode status */
	DC_STATE_CHARGING_DONE,	/* Charging Done */
	DC_STATE_ADJUST_TAVOL,	/* Adjust TA voltage, new TA current < 1000mA */
	DC_STATE_ADJUST_TACUR,	/* Adjust TA current, new TA current < 1000mA */
	DC_STATE_MAX,
};

/* PD Message Type */
enum {
	PD_MSG_REQUEST_APDO,
	PD_MSG_REQUEST_FIXED_PDO,
	WCRX_REQUEST_VOLTAGE,
};

/* TA Type for the direct charging */
enum {
	TA_TYPE_UNKNOWN,
	TA_TYPE_USBPD,
	TA_TYPE_WIRELESS,
};

/* Direct Charging Mode for the direct charging */
enum {
	CHG_NO_DC_MODE,
	CHG_2TO1_DC_MODE,
	CHG_4TO1_DC_MODE,
};

/* PPS timers */
#define PCA9468_PDMSG_WAIT_T	250	/* 250ms */
#define PCA9468_PDMSG_RETRY_T	1000	/* 1000ms */
#define PCA9468_PPS_PERIODIC_T	10000	/* 10000ms */

/* - Core driver  ---------------------------- */

int pca9468_read_adc(struct pca9468_charger *pca9468, u8 adc_ch);
int pca9468_input_current_limit(struct pca9468_charger *pca9468);

/* - PPS Integration (move to a separate file) ---------------------------- */

/* */
enum {
	PPS_INDEX_DISABLED = 0,
	PPS_INDEX_TCPM = 1,
	PPS_INDEX_WLC,
	PPS_INDEX_MAX,
};

int pca9468_probe_pps(struct pca9468_charger *pca9468_chg);
int pca9468_set_switching_charger(bool enable, unsigned int input_current,
				  unsigned int charging_current,
				  unsigned int vfloat);
int pca9468_get_swc_property(enum power_supply_property prop,
			     union power_supply_propval *val);

int pca9468_request_pdo(struct pca9468_charger *pca9468);
int pca9468_usbpd_setup(struct pca9468_charger *pca9468);
int pca9468_send_pd_message(struct pca9468_charger *pca9468, unsigned int msg_type);
int pca9468_get_apdo_max_power(struct pca9468_charger *pca9468);
struct power_supply *pca9468_get_rx_psy(struct pca9468_charger *pca9468);
int pca9468_send_rx_voltage(struct pca9468_charger *pca9468, unsigned int msg_type);
int pca9468_get_rx_max_power(struct pca9468_charger *pca9468);
int pca9468_set_ta_type(struct pca9468_charger *pca9468);

/* GBMS integration */
int pca9468_get_chg_chgr_state(struct pca9468_charger *pca9468,
			       union gbms_charger_state *chg_state);
int pca9468_is_present(struct pca9468_charger *pca9468);
int pca9468_get_status(struct pca9468_charger *pca9468);
int pca9468_get_charge_type(struct pca9468_charger *pca9468);

#endif
