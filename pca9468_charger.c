/*
 * Driver for the NXP PCA9468 battery charger.
 *
 * Copyright (C) 2018 NXP Semiconductor.
 * Copyright 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/rtc.h>

#include "pca9468_regs.h"
#include "pca9468_charger.h"

#if defined (CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */



/* adc_gain bit[7:4] of reg 0x31 - 2's complement */
static int adc_gain[16] = { 0,  1,  2,  3,  4,  5,  6,  7,
			   -8, -7, -6, -5, -4, -3, -2, -1};

/* Timer definition */
#define PCA9468_VBATMIN_CHECK_T	1000	/* 1000ms */
#define PCA9468_CCMODE_CHECK1_T	5000	/* 10000ms -> 500ms */
#define PCA9468_CCMODE_CHECK2_T	5000	/* 5000ms */
#define PCA9468_CVMODE_CHECK_T 	10000	/* 10000ms */
#define PCA4968_ENABLE_DELAY_T	150	/* 150ms */
#define PCA9468_CVMODE_CHECK2_T	1000	/* 1000ms */

/* Battery Threshold */
#define PCA9468_DC_VBAT_MIN		3500000 /* uV */
/* Input Current Limit default value */
#define PCA9468_IIN_CFG_DFT		2500000 /* uA*/
/* Charging Float Voltage default value */
#define PCA9468_VFLOAT_DFT		4350000	/* uV */
/* Charging Sub Float Voltage default value */
#define PCA9468_VFLOAT_SUB_DFT		5000000	/* 5000000uV */
/* Charging Float Voltage max voltage for comp */
#define PCA9468_COMP_VFLOAT_MAX		4400000	/* uV */

/* Sense Resistance default value */
#define PCA9468_SENSE_R_DFT		1	/* 10mOhm */
/* Switching Frequency default value */
#define PCA9468_FSW_CFG_DFT		3	/* 980KHz */
/* NTC threshold voltage default value */
#define PCA9468_NTC_TH_DFT		0	/* uV*/

/* Charging Done Condition */
#define PCA9468_IIN_DONE_DFT	500000		/* uA */
/* parallel charging done conditoin */
#define PCA9468_IIN_P_DONE	1000000		/* uA */
/* Parallel charging default threshold */
#define PCA9468_IIN_P_TH_DFT	4000000		/* uA */
/* Single charging default threshold */
#define PCA9468_IIN_S_TH_DFT	10000000	/* uA */

/* Maximum TA voltage threshold */
#define PCA9468_TA_MAX_VOL		9800000 /* uV */
/* Maximum TA current threshold, set to max(cc_max) / 2 */
#define PCA9468_TA_MAX_CUR		2500000	 /* uA */
/* Minimum TA current threshold */
#define PCA9468_TA_MIN_CUR		1000000	/* uA - PPS minimum current */

/* Minimum TA voltage threshold in Preset mode */
#define PCA9468_TA_MIN_VOL_PRESET	8000000	/* uV */
/* TA voltage threshold starting Adjust CC mode */
#define PCA9468_TA_MIN_VOL_CCADJ	8500000	/* 8000000uV --> 8500000uV */

#define PCA9468_TA_VOL_PRE_OFFSET	500000	 /* uV */
/* Adjust CC mode TA voltage step */
#define PCA9468_TA_VOL_STEP_ADJ_CC	40000	/* uV */
/* Pre CV mode TA voltage step */
#define PCA9468_TA_VOL_STEP_PRE_CV	20000	/* uV */

/* IIN_CC adc offset for accuracy */
#define PCA9468_IIN_ADC_OFFSET		20000	/* uA */
/* IIN_CC compensation offset */
#define PCA9468_IIN_CC_COMP_OFFSET	75000	/* uA */
/* IIN_CC compensation offset in Power Limit Mode(Constant Power) TA */
#define PCA9468_IIN_CC_COMP_OFFSET_CP	20000	/* uA */
/* TA maximum voltage that can support CC in Constant Power Mode */
#define PCA9468_TA_MAX_VOL_CP		9800000	/* 9760000uV --> 9800000uV */
/* Offset for cc_max / 2 */
#define PCA9468_IIN_MAX_OFFSET		125000


/* maximum retry counter for restarting charging */
#define PCA9468_MAX_RETRY_CNT		3	/* retries */
/* TA IIN tolerance */
#define PCA9468_TA_IIN_OFFSET		100000	/* uA */
/* IIN_CC upper protection offset in Power Limit Mode TA */
#define PCA9468_IIN_CC_UPPER_OFFSET	50000	/* 50mA */

/* PD Message Voltage and Current Step */
#define PD_MSG_TA_VOL_STEP		20000	/* uV */
#define PD_MSG_TA_CUR_STEP		50000	/* uA */

/* Maximum WCRX voltage threshold */
#define PCA9468_WCRX_MAX_VOL		9750000 /* uV */
/* WCRX voltage Step */
#define WCRX_VOL_STEP			100000	/* uV */

#define PCA9468_OTV_MARGIN		12000	/* uV */

/* INT1 Register Buffer */
enum {
	REG_INT1,
	REG_INT1_MSK,
	REG_INT1_STS,
	REG_INT1_MAX
};

/* STS Register Buffer */
enum {
	REG_STS_A,
	REG_STS_B,
	REG_STS_C,
	REG_STS_D,
	REG_STS_MAX
};

/* CC Mode Status */
enum {
	CCMODE_CHG_LOOP,	/* TODO: There is no such thing */
	CCMODE_VFLT_LOOP,
	CCMODE_IIN_LOOP,
	CCMODE_LOOP_INACTIVE,
	CCMODE_VIN_UVLO,
};

/* CV Mode Status */
enum {
	CVMODE_CHG_LOOP,	/* TODO: There is no such thing */
	CVMODE_VFLT_LOOP,
	CVMODE_IIN_LOOP,
	CVMODE_LOOP_INACTIVE,
	CVMODE_CHG_DONE,
	CVMODE_VIN_UVLO,
};

/* Timer ID */
enum {
	TIMER_ID_NONE,
	TIMER_VBATMIN_CHECK,
	TIMER_PRESET_DC,
	TIMER_PRESET_CONFIG,
	TIMER_CHECK_ACTIVE,
	TIMER_ADJUST_CCMODE,
	TIMER_CHECK_CCMODE,
	TIMER_ENTER_CVMODE,
	TIMER_CHECK_CVMODE,
	TIMER_PDMSG_SEND,
	TIMER_ADJUST_TAVOL,
	TIMER_ADJUST_TACUR,
};


/* TA increment Type */
enum {
	INC_NONE,	/* No increment */
	INC_TA_VOL,	/* TA voltage increment */
	INC_TA_CUR,	/* TA current increment */
};

/* BATT info Type */
enum {
	BATT_CURRENT,
	BATT_VOLTAGE,
};

/* IIN offset as the switching frequency in uA*/
static int iin_fsw_cfg[16] = { 9990, 10540, 11010, 11520, 12000, 12520, 12990,
			      13470, 5460, 6050, 6580, 7150, 7670, 8230, 8720,
			      9260};

/* ------------------------------------------------------------------------ */

/* ADC Read function, return uV or uA */
int pca9468_read_adc(struct pca9468_charger *pca9468, u8 adc_ch)
{
	u8 reg_data[2];
	u16 raw_adc = 0;
	int conv_adc = -1;
	int ret;

	switch (adc_ch) {
	case ADCCH_VOUT:
		/* ~PCA9468_BIT_CH1_EN, PCA9468_REG_ADC_CFG, udelay(120) us */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_4,
				       reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VOUT9_2) << 2) |
			  ((reg_data[0] & PCA9468_BIT_ADC_VOUT1_0) >> 6);
		conv_adc = raw_adc * VOUT_STEP;	/* unit - uV */
		break;

	case ADCCH_VIN:
		/* ~PCA9468_BIT_CH2_EN, PCA9468_REG_ADC_CFG, udelay(120) us */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_3,
				       reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VIN9_4) << 4) |
			  ((reg_data[0] & PCA9468_BIT_ADC_VIN3_0) >> 4);
		conv_adc = raw_adc * VIN_STEP;	/* unit - uV */
		break;

	case ADCCH_VBAT:
		/* ~PCA9468_BIT_CH3_EN, PCA9468_REG_ADC_CFG, udelay(120) us */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_6,
				       reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VBAT9_8) << 8) |
			  ((reg_data[0] & PCA9468_BIT_ADC_VBAT7_0) >> 0);
		conv_adc = raw_adc * VBAT_STEP; /* unit - uV */
		break;

	case ADCCH_IIN:
		/* ~PCA9468_BIT_CH5_EN, PCA9468_REG_ADC_CFG, udelay(120) us */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_1,
				       reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_IIN9_8) << 8) |
			  ((reg_data[0] & PCA9468_BIT_ADC_IIN7_0) >> 0);

		/*
		 * iin = rawadc*4.89 + (rawadc*4.89 - 900) *
		 * 	 adc_comp_gain/100
		 */
		conv_adc = raw_adc * IIN_STEP + (raw_adc * IIN_STEP -
			   ADC_IIN_OFFSET) * pca9468->adc_comp_gain /
			   100; /* unit - uA */
		/*
		 * If ADC raw value is 0, convert value will be minus value
		 * because of compensation gain, so in this case conv_adc
		 * is 0
		 */
		if (conv_adc < 0)
			conv_adc = 0;
		break;

	case ADCCH_DIETEMP:
		/* ~PCA9468_BIT_CH6_EN, PCA9468_REG_ADC_CFG, udelay(120) us */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_7,
				       reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_DIETEMP9_6) << 6) |
			  ((reg_data[0] & PCA9468_BIT_ADC_DIETEMP5_0) >> 2);

		/* Temp = (935-rawadc)*0.435, unit - C */
		conv_adc = (935 - raw_adc) * DIETEMP_STEP / DIETEMP_DENOM;
		if (conv_adc > DIETEMP_MAX)
			conv_adc = DIETEMP_MAX;
		else if (conv_adc < DIETEMP_MIN)
			conv_adc = DIETEMP_MIN;
		break;

	case ADCCH_NTC:
		/* ~PCA9468_BIT_CH7_EN, PCA9468_REG_ADC_CFG, udelay(120) us */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_8,
				       reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_NTCV9_4) << 4) |
			  ((reg_data[0] & PCA9468_BIT_ADC_NTCV3_0) >> 4);

		/* Temp = (rawadc < 185)? (960-rawadc/4) : (730-rawadc/8) */
		/* unit: 0.1 degree C */
		if (raw_adc < NTC_CURVE_THRESHOLD)
			conv_adc = NTC_CURVE_1_BASE - ((raw_adc * 10) >> NTC_CURVE_1_SHIFT);
		else
			conv_adc = NTC_CURVE_2_BASE - ((raw_adc * 10) >> NTC_CURVE_2_SHIFT);
		break;

	default:
		conv_adc = -EINVAL;
		break;
	}

error:
	/* if disabled a channel, re-enable it in -> PCA9468_REG_ADC_CFG */

	pr_debug("%s: adc_ch=%u, raw_adc=%x convert_val=%d\n", __func__,
		 adc_ch, raw_adc, conv_adc);

	return conv_adc;
}

/* v float voltage (5 mV) resolution */
static int pca9468_set_vfloat(struct pca9468_charger *pca9468,
			      unsigned int v_float)
{
	const int val = PCA9468_V_FLOAT(v_float);
	int ret;

	ret = regmap_write(pca9468->regmap, PCA9468_REG_V_FLOAT, val);
	pr_debug("%s: v_float=%u (%d)\n", __func__, v_float, ret);

	return ret;
}

static int pca9468_set_input_current(struct pca9468_charger *pca9468,
				     unsigned int iin)
{
	int ret, val;

	/* round-up and increase one step */
	iin = iin + PD_MSG_TA_CUR_STEP;
	val = PCA9468_IIN_CFG(iin);

	/* Set IIN_CFG to one step higher */
	val = val + 1;
	if (val > 0x32)
		val = 0x32; /* maximum value is 5A */

	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_IIN_CTRL,
				 PCA9468_BIT_IIN_CFG, val);

	pr_debug("%s: iin=%d real iin_cfg=%d (%d)\n", __func__, iin,
		 val * PCA9468_IIN_CFG_STEP, ret);

	return ret;
}

/* Returns the enable or disable value. into 1 or 0. */
static int pca9468_get_charging_enabled(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_START_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_STANDBY_EN) ? 0 : 1;

	return intval;
}

static int pca9468_set_charging(struct pca9468_charger *pca9468, bool enable)
{
	int ret, val;

	if (enable && pca9468_get_charging_enabled(pca9468) == enable) {
		pr_debug("%s: no op, already enabled\n", __func__);
		return 0;
	}

	if (enable) {
		/* Improve adc */
		val = 0x5B;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS,
				   val);
		if (ret < 0)
			goto error;
		ret = regmap_update_bits(pca9468->regmap,
					 PCA9468_REG_ADC_IMPROVE,
					 PCA9468_BIT_ADC_IIN_IMP, 0);
		if (ret < 0)
			goto error;

		/* For fixing input current error */
		/* Overwrite 0x00 in 0x41 register */
		val = 0x00;
		ret = regmap_write(pca9468->regmap, 0x41, val);
		if (ret < 0)
			goto error;
		/* Overwrite 0x01 in 0x43 register */
		val = 0x01;
		ret = regmap_write(pca9468->regmap, 0x43, val);
		if (ret < 0)
			goto error;
		/* Overwrite 0x00 in 0x4B register */
		val = 0x00;
		ret = regmap_write(pca9468->regmap, 0x4B, val);
		if (ret < 0)
			goto error;
		/* End for fixing input current error */

	} else {
		/* Disable NTC_PROTECTION_EN */
		ret = regmap_update_bits(pca9468->regmap,
					 PCA9468_REG_TEMP_CTRL,
					 PCA9468_BIT_NTC_PROTECTION_EN, 0);
	}

	/* Enable PCA9468 */
	val = enable ? PCA9468_STANDBY_DONOT : PCA9468_STANDBY_FORCED;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
				 PCA9468_BIT_STANDBY_EN, val);
	if (ret < 0)
		goto error;

	if (enable) {
		/* Wait 50ms, first to keep the start-up sequence */
		mdelay(50);
		/* Wait 150ms */
		msleep(150);

		/* Improve ADC */
		ret = regmap_update_bits(pca9468->regmap,
					 PCA9468_REG_ADC_IMPROVE,
					 PCA9468_BIT_ADC_IIN_IMP,
					 PCA9468_BIT_ADC_IIN_IMP);
		if (ret  < 0)
			goto error;

		val = 0x00;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS,
				   val);

		/* Enable NTC_PROTECTION_EN TODO: disable */
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_TEMP_CTRL,
					 PCA9468_BIT_NTC_PROTECTION_EN,
					 PCA9468_BIT_NTC_PROTECTION_EN);
	} else {
		/* Wait 5ms to keep the shutdown sequence */
		mdelay(5);
	}

error:
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int pca9468_check_state(u8 val[8], struct pca9468_charger *pca9468)
{
	int ret;

	/* Dump register */
	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1,
			       &val[PCA9468_REG_INT1], 7);
	if (ret < 0)
		return ret;

	pr_err("%s: Error reg[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,[5]=0x%x,[6]=0x%x,[7]=0x%x\n",
	       __func__, val[1], val[2], val[3], val[4], val[5], val[6],
	       val[7]);

	return 0;
}

static void pca9468_dump_test_debug(struct pca9468_charger *pca9468)
{
	u8 test_val[16];
	int ret;

	/* Read test register for debugging */
	ret = regmap_bulk_read(pca9468->regmap, 0x40, test_val, 16);
	if (ret < 0) {
		pr_err("%s: cannot read test registers (%d)\n", __func__, ret);
	} else {

		pr_err("%s: Error reg[0x40]=0x%x,[0x41]=0x%x,[0x42]=0x%x,[0x43]=0x%x,[0x44]=0x%x,[0x45]=0x%x,[0x46]=0x%x,[0x47]=0x%x\n",
			__func__, test_val[0], test_val[1], test_val[2], test_val[3],
			test_val[4], test_val[5], test_val[6], test_val[7]);
		pr_err("%s: Error reg[0x48]=0x%x,[0x49]=0x%x,[0x4A]=0x%x,[0x4B]=0x%x,[0x4C]=0x%x,[0x4D]=0x%x,[0x4E]=0x%x,[0x4F]=0x%x\n",
			__func__, test_val[8], test_val[9], test_val[10], test_val[11],
			test_val[12], test_val[13], test_val[14], test_val[15]);
	}
}

/* PCA9468 is not active state  - standby or shutdown */
/* Stop charging in timer_work */
/* return 0 when no error is detected */
static int pca9468_check_not_active(struct pca9468_charger *pca9468)
{
	u8 val[8];
	int ret;

	ret = pca9468_check_state(val, pca9468);
	if (ret < 0) {
		pr_err("%s: cannot read state\n", __func__);
		return ret;
	}

	pca9468_dump_test_debug(pca9468);

	/* Check INT1_STS first */
	if ((val[PCA9468_REG_INT1_STS] & PCA9468_BIT_V_OK_STS) != PCA9468_BIT_V_OK_STS) {
		/* VBUS is invalid */
		pr_err("%s: VOK is invalid", __func__);

		/* Check STS_A. NOTE: V_OV_TRACKING is with VIN OV */
		if (val[PCA9468_REG_STS_A] & PCA9468_BIT_CFLY_SHORT_STS)
			pr_err("%s: Flying Cap is shorted to GND", __func__);
		else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VOUT_UV_STS)
			pr_err("%s: VOUT UV", __func__); /* VOUT < VOUT_OK */
		else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VBAT_OV_STS)
			pr_err("%s: VBAT OV", __func__); /* VBAT > VBAT_OV */
		else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VIN_OV_STS)
			pr_err("%s: VIN OV", __func__); /* VIN > V_OV_FIXED */
		else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VIN_UV_STS)
			pr_err("%s: VIN UV", __func__); /* VIN < V_UVTH */
		else
			pr_err("%s: Invalid VIN or VOUT", __func__);

		return  -EINVAL;
	}

	if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_NTC_TEMP_STS) {
		int ntc_adc, ntc_th; /* NTC protection */
		u8 reg_data[2]; /* NTC threshold */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_NTC_TH_1,
				       reg_data, sizeof(reg_data));
		if (ret < 0)
			return -EIO;

		ntc_th = ((reg_data[1] & PCA9468_BIT_NTC_THRESHOLD9_8) << 8) |
			 reg_data[0];	/* uV unit */

		/* Read NTC ADC */
		ntc_adc = pca9468_read_adc(pca9468, ADCCH_NTC);	/* uV unit */
		pr_err("%s: NTC Protection, NTC_TH=%d(uV), NTC_ADC=%d(uV)",
		       __func__, ntc_th, ntc_adc);

		return -EINVAL;
	}

	if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_CTRL_LIMIT_STS) {
		/* OCP event happens */

		if (val[PCA9468_REG_STS_B] & PCA9468_BIT_OCP_FAST_STS)
			pr_err("%s: IIN is over OCP_FAST", __func__);
		else if (val[PCA9468_REG_STS_B] & PCA9468_BIT_OCP_AVG_STS)
			pr_err("%s: IIN is over OCP_AVG", __func__);
		else
			pr_err("%s: No Loop active", __func__);

		return -EINVAL;
	}

	if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_TEMP_REG_STS) {
		/* Over temperature protection */
		pr_err("%s: Device is in temperature regulation", __func__);
		return -EINVAL;
	}

	if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_TIMER_STS) {
		const u8 sts_b = val[PCA9468_REG_STS_B];

		if (sts_b & PCA9468_BIT_CHARGE_TIMER_STS)
			pr_err("%s: Charger timer is expired", __func__);
		else if (sts_b & PCA9468_BIT_WATCHDOG_TIMER_STS)
			pr_err("%s: Watchdog timer is expired", __func__);
		else
			pr_err("%s: Timer INT, but no timer STS", __func__);

		return -EINVAL;
	}

	if (val[PCA9468_REG_STS_A] & PCA9468_BIT_CFLY_SHORT_STS) {
		/* Flying cap is short to GND */
		pr_err("%s: Flying Cap is shorted to GND", __func__);
		return -EINVAL;
	}

	return 0;
}

/* Keep the current charging state, check STS_B again */
/* return 0 if VIN is still present, -EAGAIN if needs to retry, -EINVAL oth */
static int pca9468_check_standby(struct pca9468_charger *pca9468)
{
	unsigned int reg_val;
	int ret;
	u8 val[8];

	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_B, &reg_val);
	if (ret < 0)
		return -EIO;

	pr_debug("%s: RCP check, STS_B=0x%x\n",	__func__, reg_val);

	/* Check Active status */
	if (reg_val & PCA9468_BIT_ACTIVE_STATE_STS) {
		/* RCP condition happened, but VIN is still valid */

		/* If VIN is increased, input current will increased over
		 * IIN_LOW level
		 */
		/* Normal charging */
		return 0;
	}

	/* It is not RCP condition */
	if (reg_val & PCA9468_BIT_STANDBY_STATE_STS) {
		/* Need to retry if DC is in starting state */
		pr_err("%s: Any abnormal condition, retry\n", __func__);
		ret = -EAGAIN;
	}  else {
		pr_err("%s: Shutdown state\n", __func__);
		ret = -EINVAL;
	}

	/* Dump registers again */
	pca9468_check_state(val, pca9468);
	ret = regmap_bulk_read(pca9468->regmap, 0x48, val, 3);
	pr_err("%s: Error reg[0x48]=0x%x,[0x49]=0x%x,[0x4a]=0x%x\n",
		__func__, val[0], val[1], val[2]);

	return ret;
}

/* Check Active status */
static int pca9468_check_error(struct pca9468_charger *pca9468)
{
	unsigned int reg_val;
	int ret;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_B, &reg_val);
	if (ret < 0)
		goto error;

	/* PCA9468 is active state */
	if (reg_val & PCA9468_BIT_ACTIVE_STATE_STS) {
		int vbatt;

		/* PCA9468 is charging */

		/* Check whether the battery voltage is over the minimum */
		vbatt = pca9468_read_adc(pca9468, ADCCH_VBAT);
		if (vbatt > PCA9468_DC_VBAT_MIN) {
			/* Normal charging battery level */
			/* Check temperature regulation loop */
			/* Read INT1_STS register */
			ret = regmap_read(pca9468->regmap, PCA9468_REG_INT1_STS,
					  &reg_val);
			if (ret < 0) {
				pr_err("%s: cannot read status (%d)\n",
				       __func__, ret);
			} else if (reg_val & PCA9468_BIT_TEMP_REG_STS) {
				/* Over temperature protection */
				pr_err("%s: Device is in temperature regulation",
					__func__);
				ret = -EINVAL;
			}
		} else {
			/* Abnormal battery level */
			pr_err("%s: Error abnormal battery voltage=%d\n",
				__func__, vbatt);
			ret = -EINVAL;
		}

		pr_debug("%s: Active Status=%d\n", __func__, ret);
		return ret;
	}

	/* not in error but in standby or shutdown */
	/* will stop charging in timer_work */

	ret = pca9468_check_not_active(pca9468);
	if (ret < 0) {
		/* There was an error, done... */
	} else if ((reg_val & PCA9468_BIT_STANDBY_STATE_STS) == 0) {
		/* PCA9468 is in shutdown state */
		pr_err("%s: PCA9468 is in shutdown\n", __func__);
		ret = -EINVAL;
	} else {
		/* Standby? state */

		/* Check the RCP condition, T_REVI_DET is 300ms */
		/* Wait 200ms */
		msleep(200);

		/*
		 * Sometimes battery driver might call set_property function
		 * to stop charging during msleep. At this case, charging
		 * state would change DC_STATE_NO_CHARGING. PCA9468 should
		 * stop checking RCP condition and exit timer_work
		 */
		if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
			pr_err("%s: other driver forced to stop direct charging\n",
				__func__);
			ret = -EINVAL;
		} else {

			ret = pca9468_check_standby(pca9468);
			if (ret == 0) {
				pr_info("%s: RCP happened, but VIN is valid\n",
					__func__);
			}
		}
	}

error:
	pr_debug("%s: Not Active Status=%d\n", __func__, ret);
	return -EINVAL;
}

static int pca9468_get_iin(struct pca9468_charger *pca9468, int *iin)
{

	if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
		*iin = 0;
	} else {
		const int offset = iin_fsw_cfg[pca9468->pdata->fsw_cfg];
		int temp;

		temp = pca9468_read_adc(pca9468, ADCCH_IIN);
		if (temp < 0)
			return temp;

		if (temp < offset)
			temp = offset;
		*iin = (temp - offset) * 2;
	}

	return 0;
}

/* only needed for irdrop compensation ane maybe not even that... */
static int pca9468_get_batt_info(struct pca9468_charger *pca9468, int info_type, int *info)
{
	union power_supply_propval val;
	enum power_supply_property psp;
	int ret;

	if (!pca9468->batt_psy)
		pca9468->batt_psy = power_supply_get_by_name("battery");
	if (!pca9468->batt_psy)
		return -EINVAL;

	if (info_type == BATT_CURRENT)
		psp = POWER_SUPPLY_PROP_CURRENT_NOW;
	else
		psp = POWER_SUPPLY_PROP_VOLTAGE_NOW;

	ret = power_supply_get_property(pca9468->batt_psy, psp, &val);
	if (ret == 0)
		*info = val.intval;

	return ret;
}

/* only needed for irdrop compensation ane maybe not even that... */
static int pca9468_get_ibatt(struct pca9468_charger *pca9468, int *info)
{
	return pca9468_get_batt_info(pca9468, BATT_CURRENT, info);
}


static int pca9468_read_status(struct pca9468_charger *pca9468)
{
	unsigned int reg_val;
	int ret;

	/* Read STS_A */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_A, &reg_val);
	if (ret < 0)
		return ret;

	/* Check STS_A */
	if (reg_val & PCA9468_BIT_VIN_UV_STS) {
		ret = CCMODE_VIN_UVLO;
	} else if (reg_val & PCA9468_BIT_CHG_LOOP_STS) {
		ret = CCMODE_CHG_LOOP; /* never */
	} else if (reg_val & PCA9468_BIT_VFLT_LOOP_STS) {
		ret = CCMODE_VFLT_LOOP;
	} else if (reg_val & PCA9468_BIT_IIN_LOOP_STS) {
		ret = CCMODE_IIN_LOOP;
	} else {
		ret = CCMODE_LOOP_INACTIVE; /* lower IIN or TA to enter CC? */
	}

	return ret;
}

/* TODO: tune offset and limit */
static int pca9468_apply_irdrop(struct pca9468_charger *pca9468, int fv_uv)
{
	const int delta_offset = 20000;
	const int delta_limit = 85000; /* reduce at high voltage? */
	int ret, vbat, pca_vbat = 0, delta = 0;

	ret = pca9468_get_batt_info(pca9468, BATT_VOLTAGE, &vbat);
	if (ret < 0)
		goto error_done;

	pca_vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
	if (pca_vbat < 0 || pca_vbat < vbat)
		goto error_done;

	delta = pca_vbat - vbat;
	if (delta > delta_limit)
		delta = delta_limit;

	delta += delta_offset;
	if (fv_uv + delta > PCA9468_COMP_VFLOAT_MAX)
		delta = PCA9468_COMP_VFLOAT_MAX - fv_uv;

error_done:
	pr_debug("%s: fv_uv=%d->%d pca_vbat=%d, vbat=%d delta_v=%d\n",
		 __func__, fv_uv, fv_uv + delta, pca_vbat,
		 ret < 0 ? ret : vbat, delta);

	if (fv_uv + delta < pca_vbat) {
		pr_err("%s: fv_uv=%d, comp_fv_uv=%d is lower than VBAT=%d\n",
		       __func__, fv_uv, fv_uv + delta, pca_vbat);
		return -EINVAL;
	}

	return fv_uv + delta;
}

static int pca9468_const_charge_voltage(struct pca9468_charger *pca9468);

/* irdrop compensation for the pca9468 V_FLOAT, will only raise it */
static int pca9468_comp_irdrop(struct pca9468_charger *pca9468)
{
	int ret = 0, v_float, fv_uv;

	v_float = pca9468_const_charge_voltage(pca9468);
	if (v_float < 0)
		return -EIO;

	fv_uv = pca9468_apply_irdrop(pca9468, pca9468->fv_uv);
	if (fv_uv < 0)
		return -EIO;

	/* do not back down */
	if (fv_uv > v_float) {
		ret = pca9468_set_vfloat(pca9468, fv_uv);
		pr_debug("%s: v_float=%u->%u (%d)\n", __func__,
			 v_float, fv_uv, ret);
	}

	return ret;
}

/* TODO: remove this, just call  pca9468_read_status() */
static int pca9468_check_ccmode_status(struct pca9468_charger *pca9468)
{
	int icn = -EINVAL, ibat = -EINVAL, vbat = -EINVAL;
	int rc, status;

	status = pca9468_read_status(pca9468);
	if (status < 0)
		goto error;

	rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		rc = pca9468_get_batt_info(pca9468, BATT_CURRENT, &ibat);
	if (rc == 0)
		rc = pca9468_get_batt_info(pca9468, BATT_VOLTAGE, &vbat);

	pr_debug("%s: status=%d icn:%d ibat:%d delta_c=%d, vbat:%d, fv:%d, cc_max:%d\n",
		 __func__, status, icn, ibat, icn - ibat, vbat,
		 pca9468->fv_uv, pca9468->cc_max);

error:
	pr_debug("%s: CCMODE Status=%d\n", __func__, status);
	return status;
}

/* TODO: pca9468_check_ccmode_status() is similar */
static int pca9468_check_cvmode_status(struct pca9468_charger *pca9468)
{
	int ibat = -EINVAL, vbat = -EINVAL, rc;
	int status;

	status = pca9468_read_status(pca9468);
	if (status < 0)
		goto error;

	rc = pca9468_get_batt_info(pca9468, BATT_CURRENT, &ibat);
	if (rc == 0)
		rc = pca9468_get_batt_info(pca9468, BATT_VOLTAGE, &vbat);

	pr_debug("%s: status=%d ibat:%d, cc_max:%d , vbat:%d, fv:%d\n",
		 __func__, status, ibat, pca9468->cc_max, vbat, pca9468->fv_uv);

error:
	pr_debug("%s: CVMODE Status=%d\n", __func__, status);
	return status;
}

/* hold mutex_lock(&pca9468->lock); */
static int pca9468_recover_ta(struct pca9468_charger *pca9468)
{
	int ret;

	if (pca9468->ta_type == TA_TYPE_WIRELESS) {
		/* TODO: recover TA to value before handoff, or use DT */
		pca9468->ta_vol = 18000000;
		ret = pca9468_send_rx_voltage(pca9468, WCRX_REQUEST_VOLTAGE);
	} else {
		/* TODO: recover TA to value before handoff, or use DT */
		pca9468->ta_vol = 5000000;
		pca9468->ta_cur = 3000000;
		pca9468->ta_objpos = 1; /* PDO1 - fixed 5V */
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_FIXED_PDO);
	}

	/* will not be able to recover if TA is offline */
	if (ret < 0)
		pr_err("%s: cannot recover TA (%d)\n", __func__, ret);

	return 0;
}

/* Stop Charging */
static int pca9468_stop_charging(struct pca9468_charger *pca9468)
{
	int ret = 0;

	mutex_lock(&pca9468->lock);

	/* Check the current state */
	if (pca9468->charging_state == DC_STATE_NO_CHARGING)
		goto done;

	/* Stop Direct charging  */
	cancel_delayed_work(&pca9468->timer_work);
	cancel_delayed_work(&pca9468->pps_work);
	pca9468->timer_id = TIMER_ID_NONE;
	pca9468->timer_period = 0;

	/* Clear parameter */
	pca9468->charging_state = DC_STATE_NO_CHARGING;
	pca9468->ret_state = DC_STATE_NO_CHARGING;
	pca9468->prev_iin = 0;
	pca9468->prev_inc = INC_NONE;
	pca9468->chg_mode = CHG_NO_DC_MODE;

	/*
	 * TODO: these values come from the device tree and not from
	 * PCA9468_*_CFG_DFT: do not change them while running the algo.
	 */
	pca9468->pdata->iin_cfg = PCA9468_IIN_CFG_DFT;
	//pca9468->pdata->v_float = PCA9468_VFLOAT_DFT;

	/*
	 * Clear charging configuration
	 * TODO: use defaults when these are negative or zero at startup
	 * NOTE: cc_max is twice of IIN + headroom
	 */
	pca9468->cc_max = -1;
	pca9468->fv_uv = -1;

	/* Clear requests for new Vfloat and new IIN */
	pca9468->new_vfloat = 0;
	pca9468->new_iin = 0;

	/* used to start DC and during errors */
	pca9468->retry_cnt = 0;

	ret = pca9468_set_charging(pca9468, false);
	if (ret < 0) {
		pr_err("%s: Error-set_charging(main)\n", __func__);
		goto error;
	}

	/* stop charging and recover TA voltage */
	if (pca9468->mains_online == true)
		pca9468_recover_ta(pca9468);

	power_supply_changed(pca9468->mains);

done:
error:
	mutex_unlock(&pca9468->lock);
	__pm_relax(pca9468->monitor_wake_lock);
	pr_debug("%s: END, ret=%d\n", __func__, ret);
	return ret;
}

#define FCC_TOLERANCE_RATIO		99
#define FCC_POWER_INCREASE_THRESHOLD	99

/*
 * Compensate TA current for the target input current called from
 * pca9468_charge_ccmode() when loop becomes not active.
 *
 * pca9468_charge_ccmode() ->
 * 	-> pca9468_set_rx_voltage_comp()
 * 	-> pca9468_set_ta_voltage_comp()
 * 	-> pca9468_set_ta_current_comp2()
 *
 * NOTE: call holding mutex_lock(&pca9468->lock);
 */
static int pca9468_set_ta_current_comp(struct pca9468_charger *pca9468)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;

	/* IIN = IBAT+SYSLOAD */
	rc = pca9468_get_ibatt(pca9468, &ibat);
	if (rc == 0)
		rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);

	pr_debug("%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d\n",
		 __func__, iin,
		 pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET,
		 pca9468->iin_cc,
		 pca9468->iin_cc + PCA9468_IIN_CC_COMP_OFFSET,
		 icn, ibat, pca9468->cc_max, rc);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > (pca9468->iin_cc + PCA9468_IIN_CC_COMP_OFFSET)) {

		/* TA current is higher than the target input current */
		if (pca9468->ta_cur > pca9468->iin_cc) {
			/* TA current is over than IIN_CC */
			/* Decrease TA current (50mA) */
			pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
			pr_debug("%s: Comp. Cont1: ta_cur=%u\n",
				 __func__, pca9468->ta_cur);

		/* TA current is already less than IIN_CC */
		/* Compara IIN_ADC with the previous IIN_ADC */
		} else if (iin < (pca9468->prev_iin - PCA9468_IIN_ADC_OFFSET)) {
			/* Assume that TA operation mode is CV mode */
			/* Decrease TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
			pr_debug("%s: Comp. Cont2-1: ta_vol=%u\n", __func__,
				 pca9468->ta_vol);
		} else {
			/* Assume TA operation mode is CL mode */
			/* Decrease TA current (50mA) */
			pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
			pr_debug("%s: Comp. Cont2-2: ta_cur=%u\n", __func__,
				 pca9468->ta_cur);
		}

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;

	} else if (iin < (pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET)) {

		/* compare IIN ADC with previous IIN ADC + 20mA */
		if (iin > (pca9468->prev_iin + PCA9468_IIN_ADC_OFFSET)) {
			/*
			 * TA voltage is not enough to supply the operating
			 * current of RDO: increase TA voltage
			 */

			/* Compare TA max voltage */
			if (pca9468->ta_vol == pca9468->ta_max_vol) {
				/* TA voltage is already the maximum voltage */
				/* Compare TA max current */
				if (pca9468->ta_cur == pca9468->ta_max_cur) {
					/* TA voltage and current are at max */
					pr_debug("%s: Comp. End1: ta_vol=%u, ta_cur=%u\n",
						 __func__, pca9468->ta_vol,
						 pca9468->ta_cur);

					/* Set timer */
					pca9468->timer_id = TIMER_CHECK_CCMODE;
					pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
				} else {
					/* Increase TA current (50mA) */
					pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;

					pr_debug("%s: Comp. Cont3: ta_cur=%u\n",
						 __func__, pca9468->ta_cur);

					/* Send PD Message */
					pca9468->timer_id = TIMER_PDMSG_SEND;
					pca9468->timer_period = 0;

					/* Set TA increment flag */
					pca9468->prev_inc = INC_TA_CUR;
				}
			} else {
				/* Increase TA voltage (20mV) */
				pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;
				pr_debug("%s: Comp. Cont4: ta_vol=%u\n",
					 __func__, pca9468->ta_vol);

				/* Send PD Message */
				pca9468->timer_id = TIMER_PDMSG_SEND;
				pca9468->timer_period = 0;

				/* Set TA increment flag */
				pca9468->prev_inc = INC_TA_VOL;
			}

		/* TA current is lower than the target input current */
		/* Check the previous TA increment */
		} else if (pca9468->prev_inc == INC_TA_VOL) {
			/*
			 * The previous increment is TA voltage, but
			 * input current does not increase.
			 */

			/* Try to increase TA current */
			/* Compare TA max current */
			if (pca9468->ta_cur == pca9468->ta_max_cur) {

				/* TA current is already the maximum current */
				/* Compare TA max voltage */
				if (pca9468->ta_vol == pca9468->ta_max_vol) {
					/*
					 * TA voltage and current are already
					 * the maximum values
					 */
					pr_debug("%s: Comp. End2: ta_vol=%u, ta_cur=%u\n",
						 __func__, pca9468->ta_vol,
						 pca9468->ta_cur);

					/* Set timer */
					pca9468->timer_id = TIMER_CHECK_CCMODE;
					pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
				} else {
					/* Increase TA voltage (20mV) */
					pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;
					pr_debug("%s: Comp. Cont5: ta_vol=%u\n",
						 __func__, pca9468->ta_vol);

					/* Send PD Message */
					pca9468->timer_id = TIMER_PDMSG_SEND;
					pca9468->timer_period = 0;

					/* Set TA increment flag */
					pca9468->prev_inc = INC_TA_VOL;
				}
			} else {
				/* Increase TA current (50mA) */
				pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;

				pr_debug("%s: Comp. Cont6: ta_cur=%u\n",
					 __func__, pca9468->ta_cur);

				/* Send PD Message */
				pca9468->timer_id = TIMER_PDMSG_SEND;
				pca9468->timer_period = 0;

				/* Set TA increment flag */
				pca9468->prev_inc = INC_TA_CUR;
			}

		/*
		 * The previous increment was TA current, but input current
		 * did not increase. Try to increase TA voltage.
		 */
		} else if (pca9468->ta_vol == pca9468->ta_max_vol) {
			/* TA voltage is already the maximum voltage */

			/* Compare TA maximum current */
			if (pca9468->ta_cur == pca9468->ta_max_cur) {
				/*
				* TA voltage and current are already at the
				 * maximum values
				 */
				pr_debug("%s: Comp. End3: ta_vol=%u, ta_cur=%u\n",
					 __func__, pca9468->ta_vol, pca9468->ta_cur);

				pca9468->timer_id = TIMER_CHECK_CCMODE;
				pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
			} else {
				/* Increase TA current (50mA) */
				pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;
				pr_debug("%s: Comp. Cont7: ta_cur=%u\n", __func__,
					 pca9468->ta_cur);

				/* Send PD Message */
				pca9468->timer_id = TIMER_PDMSG_SEND;
				pca9468->timer_period = 0;

				/* Set TA increment flag */
				pca9468->prev_inc = INC_TA_CUR;
			}
		} else {
			/* Increase TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;


			pr_debug("%s: Comp. Cont8: ta_vol=%u->%u\n", __func__,
				 pca9468->ta_vol, pca9468->ta_vol);

			/* Send PD Message */
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;

			/* Set TA increment flag */
			pca9468->prev_inc = INC_TA_VOL;
		}

	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		pr_debug("%s: Comp. End4(valid): ta_vol=%u, ta_cur=%u\n",
			 __func__, pca9468->ta_vol, pca9468->ta_cur);
		/* Set timer */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
	}

	/* Save previous iin adc */
	pca9468->prev_iin = iin;
	return 0;
}

/*
 * max iin for 2:1 mode given cc_max and iin_cfg.
 * TODO: maybe use pdata->iin_cfg if cc_max is zero or negative.
 */
static int pca9468_get_iin_max(struct pca9468_charger *pca9468, int cc_max)
{
	const int cc_limit = PCA9468_IIN_MAX_OFFSET + cc_max / 2;
	int iin_max;

	iin_max = min(pca9468->pdata->iin_cfg,  (unsigned int)cc_limit);

	pr_debug("%s: iin_max=%d iin_cfg=%u cc_max=%d cc_limit=%d\n", __func__,
		 iin_max,  pca9468->pdata->iin_cfg, cc_max, cc_limit);

	return iin_max;
}

/* Compensate TA current for constant power mode */
/* hold mutex_lock(&pca9468->lock), schedule on return 0 */
static int pca9468_set_ta_current_comp2(struct pca9468_charger *pca9468)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;

	/* IIN = IBAT+SYSLOAD */
	rc = pca9468_get_ibatt(pca9468, &ibat);
	if (rc == 0)
		rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
	pr_debug("%s: iin=%d, iin_cc=[%d,%d,%d], iin_cfg=%d icn=%d ibat=%d, cc_max=%d rc=%d\n",
		 __func__, iin,
		 pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET_CP,
		 pca9468->iin_cc,
		 pca9468->iin_cc + PCA9468_IIN_CC_COMP_OFFSET_CP,
		 pca9468->pdata->iin_cfg,
		 icn, ibat, pca9468->cc_max, rc);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > (pca9468->pdata->iin_cfg + PCA9468_IIN_CC_COMP_OFFSET)) {
		/* TA current is higher than the target input current limit */
		pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;

		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	} else if (iin < (pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET_CP)) {

		/* TA current is lower than the target input current */
		/* IIN_ADC < IIN_CC -20mA */
		if (pca9468->ta_vol == pca9468->ta_max_vol) {
			const int iin_cc_lb = pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET;

			/* Check IIN_ADC < IIN_CC - 50mA */
			if (iin < iin_cc_lb) {
				unsigned int iin_apdo;
				unsigned int val;

				/* Set new IIN_CC to IIN_CC - 50mA */
				pca9468->iin_cc = pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET;

				/* Set new TA_MAX_VOL to TA_MAX_PWR/IIN_CC */
				/* Adjust new IIN_CC with APDO resolution */
				iin_apdo = pca9468->iin_cc / PD_MSG_TA_CUR_STEP;
				iin_apdo = iin_apdo * PD_MSG_TA_CUR_STEP;
				/* in mV */
				val = pca9468->ta_max_pwr / (iin_apdo / pca9468->chg_mode / 1000);
				/* Adjust values with APDO resolution(20mV) */
				val = val * 1000 / PD_MSG_TA_VOL_STEP;
				val = val * PD_MSG_TA_VOL_STEP; /* uV */

				/* Set new TA_MAX_VOL */
				pca9468->ta_max_vol = min(val, (unsigned)PCA9468_TA_MAX_VOL * pca9468->chg_mode);

				/* Increase TA voltage(40mV) */
				pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP * 2;

				pr_debug("%s: Comp. Cont1: ta_vol=%u\n", __func__, pca9468->ta_vol);

				/* Send PD Message */
				pca9468->timer_id = TIMER_PDMSG_SEND;
				pca9468->timer_period = 0;
			} else {
				/* Wait for next current step compensation */
				/* IIN_CC - 50mA < IIN ADC < IIN_CC - 20mA */
				pr_debug("%s: Comp.(wait): ta_vol=%u\n",
					 __func__, pca9468->ta_vol);

				/* Set timer */
				pca9468->timer_id = TIMER_CHECK_CCMODE;
				pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;
			}
		} else {
			/* Increase TA voltage(40mV) */
			pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP * 2;
			if (pca9468->ta_vol > pca9468->ta_max_vol)
				pca9468->ta_vol = pca9468->ta_max_vol;

			pr_debug("%s: Comp. Cont2: ta_vol=%u\n",
				 __func__, pca9468->ta_vol);

			/* Send PD Message */
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CFG + 50mA */
		pr_debug("%s: Comp. End(valid): ta_vol=%u\n", __func__,
			 pca9468->ta_vol);

		/* Set timer */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;
	}

	/* Save previous iin adc */
	pca9468->prev_iin = iin;
	return 0;
}

/* Compensate TA voltage for the target input current */
/* hold mutex_lock(&pca9468->lock), schedule on return 0 */
static int pca9468_set_ta_voltage_comp(struct pca9468_charger *pca9468)
{
	const ibat_limit = (pca9468->cc_max * FCC_POWER_INCREASE_THRESHOLD) / 100;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;

	pr_debug("%s: ======START=======\n", __func__);
	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	/* IIN = IBAT+SYSLOAD */
	rc = pca9468_get_ibatt(pca9468, &ibat);
	if (rc == 0)
		rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);

	pr_debug("%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d\n",
		 __func__, iin,
		 pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET,
		 pca9468->iin_cc,
		 pca9468->iin_cc + PCA9468_IIN_CC_COMP_OFFSET,
		 icn, ibat, pca9468->cc_max, rc);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > (pca9468->iin_cc + PCA9468_IIN_CC_COMP_OFFSET)) {
		/* TA current is higher than the target input current */
		/* Decrease TA voltage (20mV) */
		pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
		pr_debug("%s: Comp. Cont1: ta_vol=%u\n", __func__,
			 pca9468->ta_vol);

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	} else if ((iin < (pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET)) ||
		   ibat < ibat_limit) {

		if (ibat < ibat_limit)
			pr_debug("%s: ibatt=%d limit=%d\n",
				__func__, ibat, ibat_limit);

		/* TA current is lower than the target input current */
		/* Compare TA max voltage */
		if (pca9468->ta_vol == pca9468->ta_max_vol) {
			/* TA is already at maximum voltage */
			pr_debug("%s: Comp. End1(max TA vol): ta_vol=%u\n",
				 __func__, pca9468->ta_vol);

			/* Set timer */
			/* Check the current charging state */
			if (pca9468->charging_state == DC_STATE_CC_MODE) {
				/* CC mode */
				pca9468->timer_id = TIMER_CHECK_CCMODE;
				pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
			} else {
				/* CV mode */
				pca9468->timer_id = TIMER_CHECK_CVMODE;
				pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
			}
		} else {
			/* Increase TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;
			pr_debug("%s: Comp. Cont2: ta_vol=%u\n", __func__,
				 pca9468->ta_vol);
			/* Send PD Message */
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		pr_debug("%s: Comp. End(valid): ta_vol=%u\n", __func__,
			 pca9468->ta_vol);

		/* Check the current charging state */
		if (pca9468->charging_state == DC_STATE_CC_MODE) {
			pca9468->timer_id = TIMER_CHECK_CCMODE;
			pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
		} else {
			pca9468->timer_id = TIMER_CHECK_CVMODE;
			pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
		}
	}

	return 0;
}

/* hold mutex_lock(&pca9468->lock), schedule on return 0 */
static int pca9468_set_rx_voltage_comp(struct pca9468_charger *pca9468)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;

	pr_debug("%s: ======START=======\n", __func__);

	rc = pca9468_get_ibatt(pca9468, &ibat);
	if (rc == 0)
		rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);

	pr_debug("%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d\n",
		 __func__, iin,
		 pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET,
		 pca9468->iin_cc,
		 pca9468->iin_cc + PCA9468_IIN_CC_COMP_OFFSET,
		 icn, ibat, pca9468->cc_max, rc);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > (pca9468->iin_cc + PCA9468_IIN_CC_COMP_OFFSET)) {

		/* RX current is higher than the target input current */
		pca9468->ta_vol = pca9468->ta_vol - WCRX_VOL_STEP;
		pr_debug("%s: Comp. Cont1: rx_vol=%u\n", __func__, pca9468->ta_vol);

		/* Set RX Voltage */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;

	} else if (iin < (pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET)) {

		/* RX current is lower than the target input current */
		/* Compare RX max voltage */
		if (pca9468->ta_vol == pca9468->ta_max_vol) {

			/* TA current is already the maximum voltage */
			pr_debug("%s: Comp. End1(max RX vol): rx_vol=%u\n",
				 __func__, pca9468->ta_vol);

			/* Check the current charging state */
			if (pca9468->charging_state == DC_STATE_CC_MODE) {
				/* CC mode */
				pca9468->timer_id = TIMER_CHECK_CCMODE;
				pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
			} else {
				/* CV mode */
				pca9468->timer_id = TIMER_CHECK_CVMODE;
				pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
			}
		} else {
			/* Increase RX voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol + WCRX_VOL_STEP;
			pr_debug("%s: Comp. Cont2: rx_vol=%u\n", __func__,
				 pca9468->ta_vol);

			/* Set RX Voltage */
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		pr_debug("%s: Comp. End(valid): rx_vol=%u\n", __func__,
			 pca9468->ta_vol);

		if (pca9468->charging_state == DC_STATE_CC_MODE) {
			/* CC mode */
			pca9468->timer_id = TIMER_CHECK_CCMODE;
			pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
		} else {
			/* CV mode */
			pca9468->timer_id = TIMER_CHECK_CVMODE;
			pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
		}
	}

	return 0;
}

/*
 * iin limit for 2:1 for the adapter and chg_mode
 * Miminm between the confguration, cc_max (scaled with offset) and the
 * adapter capabilities.
 */
static int pca9468_get_iin_limit(struct pca9468_charger *pca9468)
{
	int iin_cc;

	iin_cc = pca9468_get_iin_max(pca9468, pca9468->cc_max);
	if (pca9468->ta_max_cur * pca9468->chg_mode < iin_cc)
		iin_cc = pca9468->ta_max_cur * pca9468->chg_mode;

	pr_debug("%s: iin_cc=%d ta_max_cur=%u, chg_mode=%d\n", __func__,
		 iin_cc, pca9468->ta_max_cur, pca9468->chg_mode);

	return iin_cc;
}

/* recalculate ->ta_vol looking at demand (cc_max) */
static int pca9468_set_wireless_dc(struct pca9468_charger *pca9468, int vbat)
{
	unsigned long val;

	pca9468->iin_cc = pca9468_get_iin_limit(pca9468);

	/* RX_vol = MAX[(2*VBAT_ADC*CHG_mode + 500mV), 8.0V*CHG_mode] */
	pca9468->ta_vol = max(PCA9468_TA_MIN_VOL_PRESET * pca9468->chg_mode,
				(2 * vbat *  pca9468->chg_mode +
				PCA9468_TA_VOL_PRE_OFFSET));

	/* RX voltage resolution is 100mV */
	val = pca9468->ta_vol / WCRX_VOL_STEP;
	pca9468->ta_vol = val * WCRX_VOL_STEP;
	/* Set RX voltage to MIN[RX voltage, RX_MAX_VOL*chg_mode] */
	pca9468->ta_vol = min(pca9468->ta_vol, pca9468->ta_max_vol);

	/* ta_cur is ignored */

	pr_debug("%s: iin_cc=%d, ta_vol=%d ta_max_vol=%d\n", __func__,
		pca9468->iin_cc, pca9468->ta_vol, pca9468->ta_max_vol);

	return 0;
}

/* recalculate ->ta_vol and ->ta_cur looking at demand (cc_max) */
static int pca9468_set_wired_dc(struct pca9468_charger *pca9468, int vbat)
{
	unsigned long val;
	int iin_cc;

	pca9468->iin_cc = pca9468_get_iin_limit(pca9468);

	/* Calculate new TA max voltage, current */
	val = pca9468->iin_cc / PD_MSG_TA_CUR_STEP;
	iin_cc = val * PD_MSG_TA_CUR_STEP;

	val = pca9468->ta_max_pwr / (iin_cc / pca9468->chg_mode  / 1000); /* mV */

	/* Adjust values with APDO resolution(20mV) */
	val = val * 1000 / PD_MSG_TA_VOL_STEP;
	val = val * PD_MSG_TA_VOL_STEP; /* uV */
	pca9468->ta_max_vol = min(val, (unsigned long)PCA9468_TA_MAX_VOL *
				  pca9468->chg_mode);

	/* MAX[8000mV*chg_mode, 2*VBAT_ADC*chg_mode+500 mV] */
	pca9468->ta_vol = max(PCA9468_TA_MIN_VOL_PRESET * pca9468->chg_mode,
				2 * vbat * pca9468->chg_mode + PCA9468_TA_VOL_PRE_OFFSET);

	/* PPS voltage resolution is 20mV */
	val = pca9468->ta_vol / PD_MSG_TA_VOL_STEP;
	pca9468->ta_vol = val * PD_MSG_TA_VOL_STEP;
	pca9468->ta_vol = min(pca9468->ta_vol, pca9468->ta_max_vol);
	/* Set TA current to IIN_CC */
	pca9468->ta_cur = iin_cc / pca9468->chg_mode;

	pr_debug("%s: iin_cc=%d, ta_vol=%d ta_cur=%d ta_max_vol=%d\n", __func__,
		pca9468->iin_cc, pca9468->ta_vol, pca9468->ta_cur,
		pca9468->ta_max_vol);

	return 0;
}

/*
 * like pca9468_preset_dcmode() but will not query the TA.
 * Called from timer:
 * [pca9468_charge_ccmode | pca9468_charge_cvmode] ->
 * 	pca9468_apply_new_iin() ->
 * 		pca9468_adjust_ta_current() ->
 * 			pca9468_reset_dcmode()
 * 	pca9468_apply_new_vfloat() ->
 * 		pca9468_reset_dcmode()
 *
 * NOTE: caller holds mutex_lock(&pca9468->lock);
 */
static int pca9468_reset_dcmode(struct pca9468_charger *pca9468)
{
	int ret = -EINVAL, vbat;

	pr_debug("%s: ======START=======\n", __func__);
	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	if (pca9468->cc_max < 0) {
		pr_err("%s: invalid cc_max=%d\n", __func__, pca9468->cc_max);
		goto error;
	}

	/*
	 * VBAT is over threshold but it might be "bouncy" due to transitory
	 * used to determine ta_vout.
	 */
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
	if (vbat < 0)
		return vbat;

	/* Check the TA type and set the charging mode */
	if (pca9468->ta_type == TA_TYPE_WIRELESS) {
		ret = pca9468_set_wireless_dc(pca9468, vbat);
	} else {
		ret = pca9468_set_wired_dc(pca9468, vbat);
	}

	/* Clear previous IIN ADC, TA increment flag */
	pca9468->prev_inc = INC_NONE;
	pca9468->prev_iin = 0;
error:
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/*
 * The caller was triggered from pca9468_apply_new_iin(), return to the
 * calling CC or CV loop.
 */
static void pca9468_return_to_loop(struct pca9468_charger *pca9468)
{
	pca9468->new_iin = 0;

	/* Go to return state  */
	pca9468->charging_state = pca9468->ret_state;
	if (pca9468->charging_state == DC_STATE_CC_MODE)
		pca9468->timer_id = TIMER_CHECK_CCMODE;
	else
		pca9468->timer_id = TIMER_CHECK_CVMODE;

	pca9468->timer_period = 1000;	/* Wait 1000ms */
}

/*
 * Kicked from pca9468_apply_new_iin() when pca9468->new_iin!=0 and completed
 * off the timer. Never called on WLC_DC.
 * NOTE: Will return to the calling loop in ->ret_state
 */
static int pca9468_adjust_ta_current(struct pca9468_charger *pca9468)
{
	const int ta_limit = pca9468->iin_cc / pca9468->chg_mode;
	int ret = 0;

	pca9468->charging_state = DC_STATE_ADJUST_TACUR;

	if (pca9468->ta_cur == ta_limit) {

		pr_debug("%s: adj. End, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u\n",
			 __func__, pca9468->ta_cur, pca9468->ta_vol,
			 pca9468->iin_cc, pca9468->chg_mode);

		/* "Recover" IIN_CC to the original value (new_iin) */
		pca9468->iin_cc = pca9468->new_iin;
		pca9468_return_to_loop(pca9468);

	} else if (pca9468->iin_cc > pca9468->pdata->iin_cfg) {
		const int old_iin_cfg = pca9468->pdata->iin_cfg;

		/* Raise iin_cfg to the new iin_cc value (why??!?!?) */
		pca9468->pdata->iin_cfg = pca9468->iin_cc;

		ret = pca9468_set_input_current(pca9468, pca9468->iin_cc);
		if (ret == 0)
			ret = pca9468_reset_dcmode(pca9468);
		if (ret < 0)
			goto error;

		pr_debug("%s: New IIN, ta_max_vol=%u, ta_max_cur=%u, ta_max_pwr=%lu, iin_cc=%u, iin_cfg=%d->%d chg_mode=%u\n",
			 __func__, pca9468->ta_max_vol, pca9468->ta_max_cur,
			 pca9468->ta_max_pwr, pca9468->iin_cc,
			 old_iin_cfg, pca9468->iin_cc,
			 pca9468->chg_mode);

		pca9468->new_iin = 0;

		/* Send PD Message and go to Adjust CC mode */
		pca9468->charging_state = DC_STATE_ADJUST_CC;
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	} else {
		unsigned int val;

		/*
		 * Adjust IIN_CC with APDO resolution(50mA)
		 * pca9468->iin_cc will be reset to pca9468->new_iin when
		 * ->ta_cur reaches the ta_limit at the beginning of the
		 * function
		 */
		val = pca9468->iin_cc / PD_MSG_TA_CUR_STEP;
		pca9468->iin_cc = val * PD_MSG_TA_CUR_STEP;
		pca9468->ta_cur = pca9468->iin_cc / pca9468->chg_mode;

		pr_debug("%s: adjust iin=%u ta_cur=%d chg_mode=%d\n", __func__,
			 pca9468->iin_cc, pca9468->ta_cur, pca9468->chg_mode);

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	}

	/* reschedule on ret == 0 */
error:
	return ret;
}

/* Kicked from apply_new_iin() then run off the timer
 * call holding mutex_lock(&pca9468->lock);
 */
static int pca9468_adjust_ta_voltage(struct pca9468_charger *pca9468)
{
	int iin;

	pca9468->charging_state = DC_STATE_ADJUST_TAVOL;

	iin = pca9468_read_adc(pca9468, ADCCH_IIN);
	pr_debug("%s: iin=%d, iin_cc=[%d%d%d], cc_max=%d\n", __func__,
		 iin,
		 pca9468->iin_cc - PD_MSG_TA_CUR_STEP,
		 pca9468->iin_cc,
		 pca9468->iin_cc + PD_MSG_TA_CUR_STEP,
		 pca9468->cc_max);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with targer input current */
	if (iin > (pca9468->iin_cc + PD_MSG_TA_CUR_STEP)) {
		/* TA current is higher than the target input current */
		/* Decrease TA voltage (20mV) */
		pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;

		pr_debug("%s: adj. Cont1, ta_vol=%u\n",
			 __func__, pca9468->ta_vol);

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	} else if (iin < (pca9468->iin_cc - PD_MSG_TA_CUR_STEP)) {
		/* TA current is lower than the target input current */
		/* Compare TA max voltage */
		if (pca9468->ta_vol == pca9468->ta_max_vol) {

			/* TA current is already the maximum voltage */

			pr_debug("%s: adj. End1, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u\n",
				 __func__, pca9468->ta_cur, pca9468->ta_vol,
				 pca9468->iin_cc, pca9468->chg_mode);

			/* Return charging state to the previous state */
			pca9468_return_to_loop(pca9468);
		} else {
			/* Increase TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;

			pr_debug("%s: adj. Cont2, ta_vol=%u\n", __func__,
				 pca9468->ta_vol);

			/* Send PD Message */
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */

		pr_debug("%s: adj. End2, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u\n",
			 __func__, pca9468->ta_cur, pca9468->ta_vol,
			 pca9468->iin_cc, pca9468->chg_mode);

		/* Return charging state to the previous state */
		pca9468_return_to_loop(pca9468);
	}

	return 0;
}

/*
 * Kicked from apply_new_iin() then run off the timer
 * * NOTE: caller must hold mutex_lock(&pca9468->lock)
 */
static int pca9468_adjust_rx_voltage(struct pca9468_charger *pca9468)
{
	int iin;

	pca9468->charging_state = DC_STATE_ADJUST_TAVOL;

	/* Read IIN ADC */
	iin = pca9468_read_adc(pca9468, ADCCH_IIN);
	pr_debug("%s: iin=%d, iin_cc=%d, cc_max=%d\n", __func__,
		 iin, pca9468->iin_cc, pca9468->cc_max);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with targer input current */
	if (iin > (pca9468->iin_cc + PCA9468_IIN_CC_COMP_OFFSET)) {
		/* RX current is higher than the target input current */

		/* Decrease RX voltage (100mV) */
		pca9468->ta_vol = pca9468->ta_vol - WCRX_VOL_STEP;

		pr_debug("%s: adj. Cont1, rx_vol=%u\n", __func__, pca9468->ta_vol);

		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	} else if (iin < (pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET)) {
		/* RX current is lower than the target input current */

		if (pca9468->ta_vol == pca9468->ta_max_vol) {
			/* RX current is already the maximum voltage */
			pr_debug("%s: adj. End1, rx_vol=%u, iin_cc=%u, chg_mode=%u\n",
				 __func__, pca9468->ta_vol, pca9468->iin_cc,
				 pca9468->chg_mode);

			/* Return charging state to the previous state */
			pca9468_return_to_loop(pca9468);
		} else {
			/* Increase RX voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol + WCRX_VOL_STEP;

			pr_debug("%s: adj. Cont2, rx_vol=%u\n",
				 __func__, pca9468->ta_vol);

			/* Set RX voltage */
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */

		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		pr_debug("%s: adj. End2, rx_vol=%u, iin_cc=%u, chg_mode=%u\n",
			 __func__, pca9468->ta_vol, pca9468->iin_cc,
			 pca9468->chg_mode);

		/* Return charging state to the previous state */
		pca9468_return_to_loop(pca9468);
	}

	return 0;
}

/*
 * Called from CC and CV loops to set a new IIN (i.e. a new cc_max charging
 * current).
 * NOTE: caller must hold mutex_lock(&pca9468->lock)
 */
static int pca9468_apply_new_iin(struct pca9468_charger *pca9468)
{
	int ret;

	pr_debug("%s: new_iin=%d (cc_max), charging_state=%d\n", __func__,
		 pca9468->new_iin, pca9468->charging_state);

	/*
	 * new_iin is used as a flag to trigger the process which might span
	 * one or more timer ticks. The flag will be cleared once the target
	 * is reached.
	 */
	pca9468->iin_cc = pca9468->new_iin;

	 /*
	  * ->ret_state is used to go back to the loop (CC or CV) that called
	  * this function.
	  */
	pca9468->ret_state = pca9468->charging_state;

	if (pca9468->ta_type == TA_TYPE_WIRELESS) {
		ret = pca9468_adjust_rx_voltage(pca9468);
	} else if (pca9468->iin_cc < (PCA9468_TA_MIN_CUR * pca9468->chg_mode)) {
		/* TA current = PCA9468_TA_MIN_CUR(1.0A) */
		pca9468->ta_cur = PCA9468_TA_MIN_CUR;
		ret = pca9468_adjust_ta_voltage(pca9468);
	} else {
		ret = pca9468_adjust_ta_current(pca9468);
	}

	/* need reschedule on ret != 0 */

	pr_debug("%s: ret=%d\n", __func__, ret);
	return ret;
}

/* also called from pca9468_set_new_cc_max() */
static int pca9468_set_new_iin(struct pca9468_charger *pca9468, int iin)
{
	int ret = 0;

	if (iin < 0) {
		pr_debug("%s: ignore negative iin=%d\n", __func__, iin);
		return 0;
	}

	pr_debug("%s: iin=%d, new_iin=%d state=%d\n", __func__,
		 iin, pca9468->new_iin, pca9468->charging_state);

	/* same as previous request nevermind */
	if (pca9468->iin_cc == pca9468->new_iin)
		return 0;

	/* apply iin_cc in pca9468_preset_config() at start */
	if (pca9468->charging_state == DC_STATE_NO_CHARGING ||
	    pca9468->charging_state == DC_STATE_CHECK_VBAT) {
		pca9468->iin_cc = iin;
	} else {
		/*
		 * applied in pca9468_apply_new_iin() from CC or in CV loop
		 * will ultimately update pca9468->iin_cc.
		 */
		pca9468->new_iin = iin;

		/* might want to tickle the cycle now */
	}

	pr_debug("%s: ret=%d\n", __func__, ret);
	return ret;
}

/*
 * The is no CC loop in this part: current must be controlled on TA side
 * adjusting output power. cc_max (the charging current) is scaled to iin
 *
 */
static int pca9468_set_new_cc_max(struct pca9468_charger *pca9468, int cc_max)
{
	int iin_max, ret = 0;

	if (cc_max < 0) {
		pr_debug("%s: ignore negative cc_max=%d\n", __func__, cc_max);
		return 0;
	}

	mutex_lock(&pca9468->lock);

	pr_debug("%s: cc_max=%d->%d\n", __func__, pca9468->cc_max, cc_max);

	/* same as previous request nevermind */
	if (cc_max == pca9468->cc_max)
		goto done;

	/* iin will be capped by the adapter capabilities in reset_dcmode() */
	iin_max = pca9468_get_iin_max(pca9468, cc_max);
	if (iin_max <= 0) {
		pr_debug("%s: ignore negative iin_max=%d\n", __func__, iin_max);
		goto done;
	}

	ret = pca9468_set_new_iin(pca9468, iin_max);
	if (ret == 0)
		pca9468->cc_max = cc_max;

done:
	pr_debug("%s: ret=%d\n", __func__, ret);
	mutex_unlock(&pca9468->lock);
	return ret;
}

/*
 * Apply pca9468->new_vfloat to the charging voltage.
 * Called from CC and CV loops, needs mutex_lock(&pca9468->lock)
 */
static int pca9468_apply_new_vfloat(struct pca9468_charger *pca9468)
{
	int fv_uv, ret = 0;

	/* compensated float voltage, -EINVAL if under pca_vbat */
	fv_uv = pca9468_apply_irdrop(pca9468, pca9468->new_vfloat);
	if (fv_uv < 0)
		return fv_uv;

	/* actually change the hardware */
	ret = pca9468_set_vfloat(pca9468, fv_uv);
	if (ret < 0)
		goto error_done;

	/* Restart the process (TODO: optimize this) */
	ret = pca9468_reset_dcmode(pca9468);
	if (ret < 0) {
		pr_err("%s: cannot reset dcmode (%d)\n", __func__, ret);
	} else {
		pca9468->charging_state = DC_STATE_ADJUST_CC;
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	}

error_done:
	pr_debug("%s: new_vfloat=%d, fv_uv=%d ret=%d\n", __func__,
		 pca9468->new_vfloat, fv_uv, ret);

	if (ret == 0)
		pca9468->new_vfloat = 0;

	return ret;
}

static int pca9468_set_new_vfloat(struct pca9468_charger *pca9468, int vfloat)
{
	int ret = 0;

	if (vfloat < 0) {
		pr_debug("%s: ignore negative vfloat %d\n", __func__, vfloat);
		return 0;
	}

	mutex_lock(&pca9468->lock);
	if (pca9468->fv_uv == vfloat)
		goto done;

	/* this is what is requested */
	pca9468->fv_uv = vfloat;

	/* use fv_uv at start in pca9468_preset_config() */
	if (pca9468->charging_state == DC_STATE_NO_CHARGING ||
	    pca9468->charging_state == DC_STATE_CHECK_VBAT) {
		// pca9468->pdata->v_float = vfloat;
	} else {
		/* applied in pca9468_apply_new_vfloat() from CC or in CV loop */
		pca9468->new_vfloat = vfloat;

		/* might want to tickle the cycle */
	}

done:
	mutex_unlock(&pca9468->lock);
	return ret;
}

/* called on loop inactive */
static int pca9468_ajdust_ccmode_wireless(struct pca9468_charger *pca9468, int iin)
{
	/* IIN_ADC > IIN_CC -20mA ? */
	if (iin > (pca9468->iin_cc - PCA9468_IIN_ADC_OFFSET)) {
		/* Input current is already over IIN_CC */
		/* End RX voltage adjustment */
		/* change charging state to CC mode */
		pca9468->charging_state = DC_STATE_CC_MODE;

		pr_debug("%s: CC adjust End: IIN_ADC=%d, rx_vol=%u\n",
				__func__, iin, pca9468->ta_vol);

		/* Clear TA increment flag */
		pca9468->prev_inc = INC_NONE;
		/* Go to CC mode */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = 0;

	/* Check RX voltage */
	} else if (pca9468->ta_vol == pca9468->ta_max_vol) {
		/* RX voltage is already max value */
		pr_debug("%s: CC adjust End: MAX value, rx_vol=%u\n",
				__func__, pca9468->ta_vol);

		/* Clear TA increment flag */
		pca9468->prev_inc = INC_NONE;
		/* Go to CC mode */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = 0;
	} else {
		/* Try to increase RX voltage(100mV) */
		pca9468->ta_vol = pca9468->ta_vol + WCRX_VOL_STEP;
		if (pca9468->ta_vol > pca9468->ta_max_vol)
			pca9468->ta_vol = pca9468->ta_max_vol;

		pr_debug("%s: CC adjust. Cont: rx_vol=%u\n",
				__func__, pca9468->ta_vol);
		/* Set RX voltage */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	}

	return 0;
}

/* called on loop inactive */
static int pca9468_ajdust_ccmode_wired(struct pca9468_charger *pca9468, int iin)
{
	/* USBPD TA is connected */
	if (iin > (pca9468->iin_cc - PCA9468_IIN_ADC_OFFSET)) {
		/* IIN_ADC > IIN_CC -20mA ? */
		/* Input current is already over IIN_CC */
		/* End TA voltage and current adjustment */
		/* change charging state to CC mode */
		pca9468->charging_state = DC_STATE_CC_MODE;

		pr_debug("%s: CC adjust End: IIN_ADC=%d, ta_vol=%u, ta_cur=%u\n",
				__func__, iin, pca9468->ta_vol,
				pca9468->ta_cur);

		/* Clear TA increment flag */
		pca9468->prev_inc = INC_NONE;
		/* Go to CC mode */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = 0;

	/* Check TA voltage */
	} else if (pca9468->ta_vol == pca9468->ta_max_vol) {
		/* TA voltage is already max value */
		pr_debug("%s: CC adjust End: MAX value, ta_vol=%u, ta_cur=%u\n",
				__func__, pca9468->ta_vol, pca9468->ta_cur);

		/* Clear TA increment flag */
		pca9468->prev_inc = INC_NONE;
		/* Go to CC mode */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = 0;

		/* Check TA tolerance
		 * The current input current compares the final input
		 * current(IIN_CC) with 100mA offset PPS current tolerance
		 * has +/-150mA, so offset defined 100mA(tolerance +50mA)
		 */
	} else if (iin < (pca9468->iin_cc - PCA9468_TA_IIN_OFFSET)) {
		/*
		 * TA voltage too low to enter TA CC mode, so we
		 * should increase TA voltage
		 */
		pca9468->ta_vol = pca9468->ta_vol + PCA9468_TA_VOL_STEP_ADJ_CC *
					pca9468->chg_mode;

		if (pca9468->ta_vol > pca9468->ta_max_vol)
			pca9468->ta_vol = pca9468->ta_max_vol;

		pr_debug("%s: CC adjust Cont: ta_vol=%u\n",
				__func__, pca9468->ta_vol);
		/* Set TA increment flag */
		pca9468->prev_inc = INC_TA_VOL;
		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;

	/* compare IIN ADC with previous IIN ADC + 20mA */
	} else if (iin > (pca9468->prev_iin + PCA9468_IIN_ADC_OFFSET)) {
		/* TA can supply more current if TA voltage is high */
		/* TA voltage too low for TA CC mode: increase it */
		pca9468->ta_vol = pca9468->ta_vol +
					PCA9468_TA_VOL_STEP_ADJ_CC *
					pca9468->chg_mode;
		if (pca9468->ta_vol > pca9468->ta_max_vol)
			pca9468->ta_vol = pca9468->ta_max_vol;

		pr_debug("%s: CC adjust Cont: ta_vol=%u\n",
				__func__, pca9468->ta_vol);
		/* Set TA increment flag */
		pca9468->prev_inc = INC_TA_VOL;

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;

	/* Check the previous increment */
	} else if (pca9468->prev_inc == INC_TA_CUR) {
		/*
		 * The previous increment is TA current, but input
		 * current does not increase
		 */

		/* Try to increase TA voltage(40mV) */
		pca9468->ta_vol = pca9468->ta_vol +
					PCA9468_TA_VOL_STEP_ADJ_CC *
					pca9468->chg_mode;
		if (pca9468->ta_vol > pca9468->ta_max_vol)
			pca9468->ta_vol = pca9468->ta_max_vol;

		pr_debug("%s: CC adjust(flag) Cont: ta_vol=%u\n", __func__,
			 pca9468->ta_vol);

		/* Set TA increment flag */
		pca9468->prev_inc = INC_TA_VOL;
		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;

		/*
		 * The previous increment is TA voltage, but input
		 * current does not increase
		 */

		/* Try to increase TA current */
		/* Check APDO max current */
	} else if (pca9468->ta_cur == pca9468->ta_max_cur) {
		/* TA current is maximum current */

		pr_debug("%s: CC adjust End(MAX_CUR): IIN_ADC=%d, ta_vol=%u, ta_cur=%u\n",
			 __func__, iin, pca9468->ta_vol, pca9468->ta_cur);

		/* Clear TA increment flag */
		pca9468->prev_inc = INC_NONE;
		/* Go to CC mode */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = 0;
	} else {
		/* TA has tolerance and compensate it as real current */
		/* Increase TA current(50mA) */
		pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;
		if (pca9468->ta_cur > pca9468->ta_max_cur)
			pca9468->ta_cur = pca9468->ta_max_cur;

		pr_debug("%s: CC adjust Cont: ta_cur=%u\n", __func__,
			 pca9468->ta_cur);

		/* Set TA increment flag */
		pca9468->prev_inc = INC_TA_CUR;
		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	}

	return 0;
}

/* 2:1 Direct Charging Adjust CC MODE control
 * called at the beginnig of CC mode charging. Will be followed by
 * pca9468_charge_ccmode with wich share some of the adjustments.
 */
static int pca9468_charge_adjust_ccmode(struct pca9468_charger *pca9468)
{
	int iin, ccmode, vbatt, vin_vol;
	bool apply_ircomp = false;
	int ret = 0;

	pr_debug("%s: ======START=======\n", __func__);

	mutex_lock(&pca9468->lock);

	pca9468->charging_state = DC_STATE_ADJUST_CC;

	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error; // This is not active mode.

	ccmode = pca9468_check_ccmode_status(pca9468);
	if (ccmode < 0) {
		ret = ccmode;
		goto error;
	}

	switch(ccmode) {
	case CCMODE_IIN_LOOP:
	case CCMODE_CHG_LOOP:	/* CHG_LOOP does't exist */
		apply_ircomp = true;

		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol - WCRX_VOL_STEP;
			pr_debug("%s: CC adjust End(LOOP): rx_vol=%u\n",
				 __func__, pca9468->ta_vol);
		} else if (pca9468->ta_cur > PCA9468_TA_MIN_CUR) {
			/* TA current is higher than 1.0A */
			/* Decrease TA current (50mA) */
			pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;

			pr_debug("%s: CC adjust End(LOOP): ta_cur=%u, ta_vol=%u\n",
				 __func__, pca9468->ta_cur, pca9468->ta_vol);
		} else {
			/* Decrease TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;

			pr_debug("%s: CC adjust End(LOOP): ta_cur=%u, ta_vol=%u\n",
				 __func__, pca9468->ta_cur, pca9468->ta_vol);
		}

		pca9468->prev_inc = INC_NONE;

		/* Send PD Message and then go to CC mode */
		pca9468->charging_state = DC_STATE_CC_MODE;
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		break;

	case CCMODE_VFLT_LOOP:
		vbatt = pca9468_read_adc(pca9468, ADCCH_VBAT);

		pr_debug("%s: CC adjust End(VFLOAT): vbatt=%d, ta_vol=%u\n",
			 __func__, vbatt, pca9468->ta_vol);

		/* Clear TA increment flag */
		pca9468->prev_inc = INC_NONE;
		/* Go to Pre-CV mode */
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		pca9468->timer_period = 0;
		break;

	case CCMODE_LOOP_INACTIVE:

		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
		pr_debug("%s: iin=%d, iin_cc=%d, cc_max=%d\n", __func__,
			 iin, pca9468->iin_cc, pca9468->cc_max);
		if (iin < 0)
			break;

		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			ret = pca9468_ajdust_ccmode_wireless(pca9468, iin);
		} else {
			ret = pca9468_ajdust_ccmode_wired(pca9468, iin);
		}

		if (ret < 0) {
			pr_err("%s: %d", __func__, ret);
		} else {
			pca9468->prev_iin = iin;
			apply_ircomp = true;
		}

		break;

	case CCMODE_VIN_UVLO:
		/* VIN UVLO - just notification , it works by hardware */
		vin_vol = pca9468_read_adc(pca9468, ADCCH_VIN);

		pr_debug("%s: CC adjust VIN_UVLO: ta_vol=%u, vin_vol=%d\n",
			 __func__, pca9468->ta_cur, vin_vol);

		/* Check VIN after 1sec */
		pca9468->timer_id = TIMER_ADJUST_CCMODE;
		pca9468->timer_period = 1000;
		break;

	default:
		goto error;
	}

	if (!pca9468->irdrop_comp_ok && apply_ircomp) {
		int rc;

		rc = pca9468_comp_irdrop(pca9468);
		if (rc < 0)
			pr_err("%s: cannot apply ircomp (%d)\n",
			       __func__, rc);
	}

	mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
			 msecs_to_jiffies(pca9468->timer_period));
error:
	mutex_unlock(&pca9468->lock);
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* 2:1 Direct Charging CC MODE control */
static int pca9468_charge_ccmode(struct pca9468_charger *pca9468)
{
	int rc, ccmode, vin_vol, iin, ibat, ret = 0;
	bool apply_ircomp = false;

	pr_debug("%s: ======START======= \n", __func__);

	mutex_lock(&pca9468->lock);
	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	pca9468->charging_state = DC_STATE_CC_MODE;

	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error_exit;

	/*
	 * A change in VFLOAT here means that we have busted the tier, a
	 * change in iin means that the thermal engine had changed cc_max
	 */
	if (pca9468->new_vfloat) {
		ret = pca9468_apply_new_vfloat(pca9468);
		if (ret < 0)
			goto error_exit;

		goto done;
	}

	if (pca9468->new_iin) {
		ret = pca9468_apply_new_iin(pca9468);
		if (ret < 0)
			goto error_exit;

		goto done;
	}

	/* Check the charging type */
	ccmode = pca9468_check_ccmode_status(pca9468);
	if (ccmode < 0) {
		ret = ccmode;
		goto error_exit;
	}

	switch(ccmode) {
	case CCMODE_LOOP_INACTIVE:

		rc = pca9468_get_iin(pca9468, &ibat);
		if (rc < 0)
			ibat = pca9468->cc_max;

		/* Set input current compensation */
		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Need RX voltage compensation */
			ret = pca9468_set_rx_voltage_comp(pca9468);

			pr_debug("%s: CC INACTIVE: rx_vol=%u\n", __func__,
				 pca9468->ta_vol);
		} else {
			const int ta_max_vol = pca9468->ta_max_vol;

			/* Check TA current with TA_MIN_CUR */
			if (pca9468->ta_cur <= PCA9468_TA_MIN_CUR) {
				pca9468->ta_cur = PCA9468_TA_MIN_CUR;

				ret = pca9468_set_ta_voltage_comp(pca9468);
			} else if (ta_max_vol >= PCA9468_TA_MAX_VOL_CP) {
				ret = pca9468_set_ta_current_comp(pca9468);
			} else {
				/* constant power mode */
				ret = pca9468_set_ta_current_comp2(pca9468);
			}

			pr_debug("%s: CC INACTIVE: ta_cur=%u, ta_vol=%u\n",
				 __func__, pca9468->ta_cur,
				 pca9468->ta_vol);
		}

		if (ret == 0)
			apply_ircomp = true;
		break;

	case CCMODE_VFLT_LOOP:
		/* TODO: adjust fv_uv here based on real vbatt */

		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
		pr_debug("%s: CC VFLOAT: iin=%d\n", __func__, iin);

		/* go to Pre-CV mode */
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		pca9468->timer_period = 0;
		break;

	case CCMODE_IIN_LOOP:
	case CCMODE_CHG_LOOP:
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
		pr_debug("%s: iin=%d, iin_cc=%d, cc_max=%d\n", __func__,
			 iin, pca9468->iin_cc, pca9468->cc_max);
		if (iin < 0)
			break;

		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol - WCRX_VOL_STEP;
			pr_debug("%s: CC LOOP:iin=%d, next_rx_vol=%u\n",
				 __func__, iin, pca9468->ta_vol);
		} else if (pca9468->ta_cur <= PCA9468_TA_MIN_CUR) {
			/* Decrease TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
			pr_debug("%s: CC LOOP:iin=%d, next_ta_vol=%u\n",
				 __func__, iin, pca9468->ta_vol);
		} else {
			/* Decrease TA current (50mA) */
			pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
			pr_debug("%s: CC LOOP:iin=%d, next_ta_cur=%u\n",
				 __func__, iin, pca9468->ta_cur);
		}

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		break;

	case CCMODE_VIN_UVLO:
		/* VIN UVLO - just notification, it works by hardware */
		vin_vol = pca9468_read_adc(pca9468, ADCCH_VIN);

		pr_debug("%s: CC VIN_UVLO: ta_cur=%u ta_vol=%u, vin_vol=%d\n",
			  __func__, pca9468->ta_cur, pca9468->ta_vol, vin_vol);

		/* Check VIN after 1sec */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = 1000;
		break;

	default:
		break;
	}

	if (!pca9468->irdrop_comp_ok && apply_ircomp) {
		int rc;

		rc = pca9468_comp_irdrop(pca9468);
		if (rc < 0)
			pr_err("%s: cannot apply ircomp (%d)\n",
			       __func__, rc);
	}

done:
	mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
			 msecs_to_jiffies(pca9468->timer_period));

error_exit:
	mutex_unlock(&pca9468->lock);
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* 2:1 Direct Charging Start CV MODE control - Pre CV MODE */
static int pca9468_charge_start_cvmode(struct pca9468_charger *pca9468)
{
	int ret = 0;
	int cvmode;
	int vin_vol;

	pr_debug("%s: ======START=======\n", __func__);

	mutex_lock(&pca9468->lock);
	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	pca9468->charging_state = DC_STATE_START_CV;

	/* Check the charging type */
	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error_exit;

	/* Check the status */
	cvmode = pca9468_check_cvmode_status(pca9468);
	if (cvmode < 0) {
		ret = cvmode;
		goto error_exit;
	}

	switch(cvmode) {
	case CVMODE_CHG_LOOP:
	case CVMODE_IIN_LOOP:

		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol - WCRX_VOL_STEP;
			pr_debug("%s: PreCV Cont(IIN_LOOP): rx_vol=%u\n",
				 __func__, pca9468->ta_vol);
		} else {
			/* Check TA current */
			if (pca9468->ta_cur > PCA9468_TA_MIN_CUR) {
				/* TA current is higher than 1.0A */

				/* Decrease TA current (50mA) */
				pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
				pr_debug("%s: PreCV Cont: ta_cur=%u\n",
					 __func__, pca9468->ta_cur);
			} else {
				/* TA current is less than 1.0A */
				/* Decrease TA voltage (20mV) */
				pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
				pr_debug("%s: PreCV Cont(IIN_LOOP): ta_vol=%u\n",
					 __func__, pca9468->ta_vol);
			}
		}

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		break;

	case CVMODE_VFLT_LOOP:
		/* Check the TA type */
		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol - WCRX_VOL_STEP;
			pr_debug("%s: PreCV Cont: rx_vol=%u\n",
				 __func__, pca9468->ta_vol);
		} else {
			/* Decrease TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol -
					  PCA9468_TA_VOL_STEP_PRE_CV *
					  pca9468->chg_mode;
			pr_debug("%s: PreCV Cont: ta_vol=%u\n",
				 __func__, pca9468->ta_vol);
		}

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		break;

	case CVMODE_LOOP_INACTIVE:
		/* Exit Pre CV mode */
		pr_debug("%s: PreCV End: ta_vol=%u, ta_cur=%u\n", __func__,
			 pca9468->ta_vol, pca9468->ta_cur);

		/* Need to implement notification to other driver */
		/* To do here */

		/* Go to CV mode */
		pca9468->timer_id = TIMER_CHECK_CVMODE;
		pca9468->timer_period = 0;
		break;

	case CVMODE_VIN_UVLO:
		/* VIN UVLO - just notification , it works by hardware */
		vin_vol = pca9468_read_adc(pca9468, ADCCH_VIN);

		pr_debug("%s: PreCV VIN_UVLO: ta_vol=%u, vin_vol=%u\n",
			 __func__, pca9468->ta_cur, vin_vol);

		/* Check VIN after 1sec */
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		pca9468->timer_period = 1000;
		break;

	default:
		break;
	}

	mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
			 msecs_to_jiffies(pca9468->timer_period));
error_exit:
	mutex_unlock(&pca9468->lock);
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* 2:1 Direct Charging CV MODE control */
static int pca9468_charge_cvmode(struct pca9468_charger *pca9468)
{
	int ret = 0;
	int cvmode;
	int vin_vol;
	int iin = 0;

	pr_debug("%s: ======START=======\n", __func__);

	mutex_lock(&pca9468->lock);
	pr_debug("%s: = charging_state=%u -> %u== \n", __func__,
		 pca9468->charging_state, DC_STATE_CV_MODE);

	pca9468->charging_state = DC_STATE_CV_MODE;

	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error_exit;

	/*
	 * A change in vfloat and cc_max here is a normal tier transition, a
	 * change in iin  means that the thermal engine has changed cc_max.
	 */
	if (pca9468->new_vfloat) {
		ret = pca9468_apply_new_vfloat(pca9468);
		if (ret < 0)
			goto error_exit;

		goto done;
	}

	if (pca9468->new_iin) {
		ret = pca9468_apply_new_iin(pca9468);
		if (ret < 0)
			goto error_exit;

		goto done;
	}

	cvmode = pca9468_check_cvmode_status(pca9468);
	if (cvmode < 0) {
		ret = cvmode;
		goto error_exit;
	}

	/* TODO: move to check cvmode_status */
	if (cvmode == CVMODE_LOOP_INACTIVE) {
		/* Read IIN_ADC */
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
		pr_debug("%s: iin=%d, iin_topoff=%u\n", __func__, iin,
			 pca9468->pdata->iin_topoff);
		if (iin < 0)
			goto error_exit;

		/* Compare iin with input topoff current */
		if (iin < pca9468->pdata->iin_topoff) {
			/* Change cvmode status to charging done */
			cvmode = CVMODE_CHG_DONE;
			pr_debug("%s: CVMODE Status=%d\n", __func__, cvmode);
		}
	}

	switch(cvmode) {
	case CVMODE_CHG_DONE:
		/* Charging Done */
		/* Keep CV mode until driver send stop charging */
		pca9468->charging_state = DC_STATE_CHARGING_DONE;
		power_supply_changed(pca9468->mains);

		/* Need to implement notification function */
		/* TODO: notify DONE charger here */

		pr_debug("%s: CV Done\n", __func__);

		if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
			/* notification function called stop */
			pr_debug("%s: Already stop DC\n", __func__);
			break;
		}

		/* Notification function does not stop timer work yet */
		/* Keep the charging done state */
		/* Set timer */
		pca9468->timer_id = TIMER_CHECK_CVMODE;
		pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
		break;

	case CVMODE_CHG_LOOP:
	case CVMODE_IIN_LOOP:
		/* Check the TA type */
		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX Voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol -
						WCRX_VOL_STEP;
			pr_debug("%s: CV LOOP, Cont: rx_vol=%u\n",
					__func__, pca9468->ta_vol);

		/* Check TA current */
		} else if (pca9468->ta_cur > PCA9468_TA_MIN_CUR) {
			/* TA current is higher than (1.0A*chg_mode) */
			/* Decrease TA current (50mA) */
			pca9468->ta_cur = pca9468->ta_cur -
						PD_MSG_TA_CUR_STEP;
			pr_debug("%s: CV LOOP, Cont: ta_cur=%u\n",
					__func__, pca9468->ta_cur);
		} else {
			/* TA current is less than (1.0A*chg_mode) */
			/* Decrease TA Voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol -
						PD_MSG_TA_VOL_STEP;
			pr_debug("%s: CV LOOP, Cont: ta_vol=%u\n",
					__func__, pca9468->ta_vol);
		}

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		break;

	case CVMODE_VFLT_LOOP:
		/* Check the TA type */
		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage */
			pca9468->ta_vol = pca9468->ta_vol -
						WCRX_VOL_STEP;
			pr_debug("%s: CV VFLOAT, Cont: rx_vol=%u\n",
					__func__, pca9468->ta_vol);
		} else {
			/* Decrease TA voltage */
			pca9468->ta_vol = pca9468->ta_vol -
						PD_MSG_TA_VOL_STEP;
			pr_debug("%s: CV VFLOAT, Cont: ta_vol=%u\n",
					__func__, pca9468->ta_vol);
		}

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		break;

	case CVMODE_LOOP_INACTIVE:
		pca9468->timer_id = TIMER_CHECK_CVMODE;
		pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
		break;

	case CVMODE_VIN_UVLO:
		/* VIN UVLO - just notification, it works by hardware */
		vin_vol = pca9468_read_adc(pca9468, ADCCH_VIN);
		pr_debug("%s: CC VIN_UVLO: ta_cur=%u ta_vol=%u, vin_vol=%d\n",
				__func__, pca9468->ta_cur, pca9468->ta_vol,
				vin_vol);

		/* Check VIN after 1sec */
		pca9468->timer_id = TIMER_CHECK_CVMODE;
		pca9468->timer_period = 1000;
		break;

	default:
		break;
	}

done:
	pr_debug("%s: reschedule next id=%d period=%ld chg_state=%d\n",
		 __func__, pca9468->timer_id, pca9468->timer_period,
		pca9468->charging_state);

	mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
			 msecs_to_jiffies(pca9468->timer_period));
error_exit:
	mutex_unlock(&pca9468->lock);
	pr_debug("%s: End, ret=%d next\n", __func__, ret);
	return ret;
}

/*
 * Preset TA voltage and current for Direct Charging Mode using
 * the configured cc_max and fv_uv limits.
 */
static int pca9468_preset_dcmode(struct pca9468_charger *pca9468)
{
	int vbat;
	int ret = 0;

	pr_debug("%s: ======START=======\n", __func__);
	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	/* gcpm must set this */
	if (pca9468->cc_max < 0 || pca9468->fv_uv < 0) {
		pr_err("%s: cc_max=%d fv_uv=%d invalid\n", __func__,
		       pca9468->cc_max, pca9468->fv_uv);
		return -EINVAL;
	}

	pca9468->charging_state = DC_STATE_PRESET_DC;

	/* VBAT is over threshold but it might be "bouncy" due to transitory */
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
	if (vbat < 0) {
		ret = vbat;
		goto error;
	}

	/* v_float is set on start from GCPM */
	if (vbat > pca9468->fv_uv) {
		pr_err("%s: vbat adc=%d is higher than VFLOAT=%d\n", __func__,
			vbat, pca9468->fv_uv);
		ret = -EINVAL;
		goto error;
	}

	/* determined by ->cfg_iin and cc_max */
	pca9468->ta_max_cur = pca9468_get_iin_max(pca9468, pca9468->cc_max);
	pr_debug("%s: ta_max_cur=%u, iin_cfg=%u, pca9468->ta_type=%d\n",
		 __func__, pca9468->ta_max_cur, pca9468->pdata->iin_cfg,
		 pca9468->ta_type);

	/* Check the TA type and set the charging mode */
	if (pca9468->ta_type == TA_TYPE_WIRELESS) {
		/*
		 * Set the RX max voltage to enough high value to find RX
		 * maximum voltage initially
		 */
		pca9468->ta_max_vol = PCA9468_WCRX_MAX_VOL * pca9468->chg_mode;

		/* Get the RX max current/voltage(RX_MAX_CUR/VOL) */
		ret = pca9468_get_rx_max_power(pca9468);
		if (ret < 0) {
			pr_err("%s: no RX voltage to support 4:1 (%d)\n", __func__, ret);
			pca9468->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		ret = pca9468_set_wireless_dc(pca9468, vbat);
		if (ret < 0) {
			pr_err("%s: set wired failed (%d)\n", __func__, ret);
			pca9468->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		pr_debug("%s: Preset DC, rx_max_vol=%u, rx_max_cur=%u, rx_max_pwr=%lu, iin_cc=%u, chg_mode=%u\n",
			 __func__, pca9468->ta_max_vol, pca9468->ta_max_cur,
			 pca9468->ta_max_pwr, pca9468->iin_cc, pca9468->chg_mode);
	} else {
		/*
		 * ->ta_max_cur is too high for startup, needs to target
		 * CC before hitting max current AND work to ta_max_cur
		 * from there.
		 */

		pca9468->ta_max_vol = PCA9468_TA_MAX_VOL * pca9468->chg_mode;

		/* Get the APDO max for 2:1 mode */
		ret = pca9468_get_apdo_max_power(pca9468);
		if (ret < 0) {
			pr_err("%s: No APDO to support 2:1\n", __func__);
			pca9468->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		ret = pca9468_set_wired_dc(pca9468, vbat);
		if (ret < 0) {
			pr_err("%s: set wired failed (%d)\n", __func__, ret);
			pca9468->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		pr_debug("%s: Preset DC, objpos=%d ta_max_vol=%u, ta_max_cur=%u, ta_max_pwr=%lu, iin_cc=%u, chg_mode=%u\n",
			 __func__, pca9468->ta_objpos, pca9468->ta_max_vol, pca9468->ta_max_cur,
			 pca9468->ta_max_pwr, pca9468->iin_cc, pca9468->chg_mode);
	}

error:
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* Preset direct charging configuration */
static int pca9468_preset_config(struct pca9468_charger *pca9468)
{
	int ret = 0;

	pr_debug("%s: ======START=======\n", __func__);

	mutex_lock(&pca9468->lock);

	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	pca9468->charging_state = DC_STATE_PRESET_DC;

	/* ->iin_cc and ->fv_uv are configured externally */
	ret = pca9468_set_input_current(pca9468, pca9468->iin_cc);
	if (ret < 0)
		goto error;

	ret = pca9468_set_vfloat(pca9468, pca9468->fv_uv);
	if (ret < 0)
		goto error;

	/* Enable PCA9468 unless aready enabled */
	ret = pca9468_set_charging(pca9468, true);
	if (ret < 0)
		goto error;

	/* Clear previous iin adc */
	pca9468->prev_iin = 0;
	pca9468->prev_inc = INC_NONE;

	/* Go to CHECK_ACTIVE state after 150ms*/
	pca9468->timer_id = TIMER_CHECK_ACTIVE;
	pca9468->timer_period = PCA4968_ENABLE_DELAY_T;
	mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
			   msecs_to_jiffies(pca9468->timer_period));
error:
	mutex_unlock(&pca9468->lock);
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* Check the charging status before entering the adjust cc mode */
static int pca9468_check_active_state(struct pca9468_charger *pca9468)
{
	int ret = 0;

	pr_debug("%s: ======START=======\n", __func__);
	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	mutex_lock(&pca9468->lock);
	pca9468->charging_state = DC_STATE_CHECK_ACTIVE;

	ret = pca9468_check_error(pca9468);
	if (ret == 0) {
		/* PCA9468 is active state */

		pca9468->retry_cnt = 0;
		pca9468->timer_id = TIMER_ADJUST_CCMODE;
		pca9468->timer_period = 0;
	} else if (ret == -EAGAIN) {
		int rc;

		/* Check the retry counter */
		if (pca9468->retry_cnt >= PCA9468_MAX_RETRY_CNT) {
			pr_err("%s: retry fail\n", __func__);
			ret = -EINVAL;
			goto exit_done;
		}

		/* Disable charging */
		rc = pca9468_set_charging(pca9468, false);
		pr_err("%s: retry charging start - retry_cnt=%d, (%d)\n",
			__func__, pca9468->retry_cnt, rc);

		/* Go to DC_STATE_PRESET_DC */
		pca9468->timer_id = TIMER_PRESET_DC;
		pca9468->timer_period = 0;
		pca9468->retry_cnt++;
	} else {
		/* Implement error handler function if it is needed */
		pca9468->timer_id = TIMER_ID_NONE;
		pca9468->timer_period = 0;
	}

	mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
			 msecs_to_jiffies(pca9468->timer_period));
exit_done:
	mutex_unlock(&pca9468->lock);
	return ret;
}

/* Enter direct charging algorithm */
static int pca9468_start_direct_charging(struct pca9468_charger *pca9468)
{
	int ret;
	unsigned int val;

	pr_debug("%s: =========START=========\n", __func__);
	mutex_lock(&pca9468->lock);

	/* Set OV_DELTA to 40% */
	val = OV_DELTA_40P << MASK2SHIFT(PCA9468_BIT_OV_DELTA);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_SAFETY_CTRL,
				 PCA9468_BIT_OV_DELTA, val);
	if (ret < 0)
		goto error_done;

	/* Set Switching Frequency */
	val = pca9468->pdata->fsw_cfg;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
				 PCA9468_BIT_FSW_CFG, val);
	if (ret < 0)
		goto error_done;

	/* Set EN_CFG to active HIGH */
	val =  PCA9468_EN_ACTIVE_H;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
				 PCA9468_BIT_EN_CFG, val);
	if (ret < 0)
		goto error_done;

	/* Set NTC voltage threshold */
	val = pca9468->pdata->ntc_th / PCA9468_NTC_TH_STEP;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_NTC_TH_1, (val & 0xFF));
	if (ret < 0)
		goto error_done;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_NTC_TH_2,
				 PCA9468_BIT_NTC_THRESHOLD9_8, (val >> 8));
	if (ret < 0)
		goto error_done;

	/* configure DC charging type for the requested index */
	ret = pca9468_set_ta_type(pca9468, pca9468->pps_index);
	pr_info("%s: Current ta_type=%d, chg_mode=%d\n", __func__,
		pca9468->ta_type, pca9468->chg_mode);
	if (ret < 0)
		goto error_done;

	/* wake lock */
	__pm_stay_awake(pca9468->monitor_wake_lock);

	/* Preset charging configuration and TA condition */
	ret = pca9468_preset_dcmode(pca9468);
	if (ret == 0) {
		/* Configure the TA  and start charging */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;

		mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
				 msecs_to_jiffies(pca9468->timer_period));
	}

error_done:
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	mutex_unlock(&pca9468->lock);
	return ret;
}

/* Check Vbat minimum level to start direct charging */
static int pca9468_check_vbatmin(struct pca9468_charger *pca9468)
{
	int ret = 0, vbat;

	pr_debug("%s: =========START=========\n", __func__);
	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	mutex_lock(&pca9468->lock);

	pca9468->charging_state = DC_STATE_CHECK_VBAT;

	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
	if (vbat < 0) {
		ret = vbat;
		goto error;
	}

	if (pca9468->cc_max < 0 || pca9468->fv_uv < 0 || vbat < PCA9468_DC_VBAT_MIN) {
		pr_debug("%s: not yet fv_uv=%d, cc_max=%d vbat=%d\n", __func__,
			pca9468->fv_uv, pca9468->cc_max, vbat);

		/* retry again after 1sec */
		pca9468->timer_id = TIMER_VBATMIN_CHECK;
		pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
		pca9468->retry_cnt += 1;
	} else {
		pca9468->timer_id = TIMER_PRESET_DC;
		pca9468->timer_period = 0;
		pca9468->retry_cnt = 0; /* start charging */
	}

	/* timeout for VBATMIN or charging parameters */
	if (pca9468->retry_cnt > PCA9468_MAX_RETRY_CNT) {
		pr_debug("%s: not yet fv_uv=%d, cc_max=%d vbat=%d\n", __func__,
			pca9468->fv_uv, pca9468->cc_max, vbat);
		ret = -ETIMEDOUT;
	} else {
		mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
				 msecs_to_jiffies(pca9468->timer_period));
	}


error:
	mutex_unlock(&pca9468->lock);
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int pca9468_send_message(struct pca9468_charger *pca9468)
{
	int val, ret;

	/* Go to the next state */
	mutex_lock(&pca9468->lock);

	/* Adjust TA current and voltage step */
	if (pca9468->ta_type == TA_TYPE_WIRELESS) {
		/* RX voltage resolution is 100mV */
		val = pca9468->ta_vol / WCRX_VOL_STEP;
		pca9468->ta_vol = val * WCRX_VOL_STEP;

		/* Set RX voltage */
		ret = pca9468_send_rx_voltage(pca9468, WCRX_REQUEST_VOLTAGE);
	} else {
		/* PPS voltage resolution is 20mV */
		val = pca9468->ta_vol / PD_MSG_TA_VOL_STEP;
		pca9468->ta_vol = val * PD_MSG_TA_VOL_STEP;
		/* PPS current resolution is 50mA */
		val = pca9468->ta_cur / PD_MSG_TA_CUR_STEP;
		pca9468->ta_cur = val * PD_MSG_TA_CUR_STEP;
		/* PPS minimum current is 1000mA */
		if (pca9468->ta_cur < PCA9468_TA_MIN_CUR)
			pca9468->ta_cur = PCA9468_TA_MIN_CUR;

		/* Send PD Message */
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
	}

	switch (pca9468->charging_state) {
	case DC_STATE_PRESET_DC:
		pca9468->timer_id = TIMER_PRESET_CONFIG;
		break;
	case DC_STATE_ADJUST_CC:
		pca9468->timer_id = TIMER_ADJUST_CCMODE;
		break;
	case DC_STATE_CC_MODE:
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		break;
	case DC_STATE_START_CV:
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		break;
	case DC_STATE_CV_MODE:
		pca9468->timer_id = TIMER_CHECK_CVMODE;
		break;
	case DC_STATE_ADJUST_TAVOL:
		pca9468->timer_id = TIMER_ADJUST_TAVOL;
		break;
	case DC_STATE_ADJUST_TACUR:
		pca9468->timer_id = TIMER_ADJUST_TACUR;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret < 0) {
		pr_err("%s: Error-send_pd_message to %d (%d)\n",
			__func__, pca9468->ta_type, ret);
		pca9468->timer_id = TIMER_CHECK_ACTIVE;
	}

	if (pca9468->ta_type == TA_TYPE_WIRELESS)
		pca9468->timer_period = PCA9468_PDMSG_WLC_WAIT_T;
	else
		pca9468->timer_period = PCA9468_PDMSG_WAIT_T;

	pr_debug("%s: charging_state=%u next_time_id => pca9468->timer_id=%d ret=%d\n",
		__func__, pca9468->charging_state, pca9468->ta_type, ret);

	mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
			 msecs_to_jiffies(pca9468->timer_period));
	mutex_unlock(&pca9468->lock);

	return ret;
}

/* delayed work function for charging timer */
static void pca9468_timer_work(struct work_struct *work)
{
	struct pca9468_charger *pca9468 =
		container_of(work, struct pca9468_charger, timer_work.work);
	int timer_id;
	int ret = 0;

	pr_debug("%s: ========= START =========\n", __func__);
	pr_debug("%s: timer id=%d, charging_state=%u\n", __func__,
		 pca9468->timer_id, pca9468->charging_state);

	mutex_lock(&pca9468->lock);
	timer_id = pca9468->timer_id;
	mutex_unlock(&pca9468->lock);

	switch (timer_id) {

	/* charging_state <- DC_STATE_CHECK_VBAT */
	case TIMER_VBATMIN_CHECK:
		ret = pca9468_check_vbatmin(pca9468);
		if (ret < 0)
			goto error;
		break;

	/* charging_state <- DC_STATE_PRESET_DC */
	case TIMER_PRESET_DC:
		ret = pca9468_start_direct_charging(pca9468);
		if (ret < 0)
			goto error;
		break;

	/*
	 * charging_state <- DC_STATE_PRESET_DC
	 *	preset configuration, start charging
	 */
	case TIMER_PRESET_CONFIG:
		ret = pca9468_preset_config(pca9468);
		if (ret < 0)
			goto error;
		break;

	/*
	 * charging_state <- DC_STATE_PRESET_DC
	 *	150 ms after preset_config
	 */
	case TIMER_CHECK_ACTIVE:
		ret = pca9468_check_active_state(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ADJUST_CCMODE:
		ret = pca9468_charge_adjust_ccmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CHECK_CCMODE:
		ret = pca9468_charge_ccmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ENTER_CVMODE:
		/* Enter Pre-CV mode */
		ret = pca9468_charge_start_cvmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CHECK_CVMODE:
		ret = pca9468_charge_cvmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_PDMSG_SEND:
		ret = pca9468_send_message(pca9468);
		if (ret < 0)
			goto error;
		break;

	/* called from 2 contexts */
	case TIMER_ADJUST_TAVOL:
		mutex_lock(&pca9468->lock);

		ret = pca9468_adjust_ta_voltage(pca9468);
		if (ret < 0) {
			mutex_unlock(&pca9468->lock);
			goto error;
		}

		mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
				 msecs_to_jiffies(pca9468->timer_period));
		mutex_unlock(&pca9468->lock);
		break;

	/* called from 2 contexts */
	case TIMER_ADJUST_TACUR:
		mutex_lock(&pca9468->lock);
		ret = pca9468_adjust_ta_current(pca9468);
		if (ret < 0) {
			mutex_unlock(&pca9468->lock);
			goto error;
		}

		mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
				 msecs_to_jiffies(pca9468->timer_period));
		mutex_unlock(&pca9468->lock);
		break;

	case TIMER_ID_NONE:
		ret = pca9468_stop_charging(pca9468);
		if (ret < 0)
			goto error;
		break;

	default:
		break;
	}


	/* Check the charging state again */
	if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
		cancel_delayed_work(&pca9468->timer_work);
		cancel_delayed_work(&pca9468->pps_work);
	}

	pr_debug("%s: NEXT timer_id=%d, period=%ld, charging_state=%u\n",
		 __func__, pca9468->timer_id, pca9468->timer_period,
		 pca9468->charging_state);

	return;

error:
	pr_debug("%s: ========= ERROR =========\n", __func__);
	pca9468_stop_charging(pca9468);
}

/* delayed work function for pps periodic timer */
static void pca9468_pps_request_work(struct work_struct *work)
{
	struct pca9468_charger *pca9468 = container_of(work,
					struct pca9468_charger, pps_work.work);
	int ret;

	pr_debug("%s: =========START=========\n", __func__);
	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
	if (ret < 0)
		pr_err("%s: Error-send_pd_message\n", __func__);

	/* TODO: do other background stuff */

	pr_debug("%s: ret=%d\n", __func__, ret);
}

static int pca9468_hw_ping(struct pca9468_charger *pca9468)
{
	unsigned int val = 0;
	int ret;

	/* Read Device info register to check the incomplete I2C operation */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_DEVICE_INFO, &val);
	if ((ret < 0) || (val != PCA9468_DEVICE_ID))
		ret = regmap_read(pca9468->regmap, PCA9468_REG_DEVICE_INFO, &val);
	if ((ret < 0) || (val != PCA9468_DEVICE_ID)) {
		dev_err(pca9468->dev, "reading DEVICE_INFO failed, val=0x%x ret=%d\n",
			val, ret);
		return -EINVAL;
	}

	return 0;
}

/* we can have multiple device drivers using this piece of HW
 * TODO: execute this in ONLINE
 */
static int pca9468_hw_init(struct pca9468_charger *pca9468)
{
	unsigned int val;
	int ret;

	/* Set OV_DELTA to 40% */
	val = OV_DELTA_40P << MASK2SHIFT(PCA9468_BIT_OV_DELTA);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_SAFETY_CTRL,
				 PCA9468_BIT_OV_DELTA, val);
	if (ret < 0)
		return ret;

	/* Set Switching Frequency */
	val = pca9468->pdata->fsw_cfg << MASK2SHIFT(PCA9468_BIT_FSW_CFG);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
				 PCA9468_BIT_FSW_CFG, val);
	if (ret < 0)
		return ret;

	/* Set Reverse Current Detection and standby mode */
	val = PCA9468_BIT_REV_IIN_DET | PCA9468_EN_ACTIVE_L |
	      PCA9468_STANDBY_FORCED;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
				 (PCA9468_BIT_REV_IIN_DET |
				  PCA9468_BIT_EN_CFG |
				  PCA9468_BIT_STANDBY_EN),
				 val);
	if (ret < 0)
		return ret;

	/* clear LIMIT_INCREMENT_EN */
	val = 0;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_IIN_CTRL,
				 PCA9468_BIT_LIMIT_INCREMENT_EN, val);
	if (ret < 0)
		return ret;

	/* Set the ADC channels, NTC is invalid if Bias is not enabled */
	val = PCA9468_BIT_CH7_EN |	/* NTC voltage ADC */
	      PCA9468_BIT_CH6_EN |	/* DIETEMP ADC */
	      PCA9468_BIT_CH5_EN |	/* IIN ADC */
	      PCA9468_BIT_CH3_EN |	/* VBAT ADC */
	      PCA9468_BIT_CH2_EN |	/* VIN ADC */
	      PCA9468_BIT_CH1_EN;	/* VOUT ADC */

	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_CFG, val);
	if (ret < 0)
		return ret;

	/* ADC Mode change */
	val = 0x5B;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);
	if (ret < 0)
		return ret;
	val = 0x10;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_MODE, val);
	if (ret < 0)
		return ret;
	val = 0x00;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);
	if (ret < 0)
		return ret;

	/* Read ADC compensation gain */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_ADC_ADJUST, &val);
	if (ret < 0)
		return ret;
	pca9468->adc_comp_gain = adc_gain[val >> MASK2SHIFT(PCA9468_BIT_ADC_GAIN)];

	/* input current - uA*/
	ret = pca9468_set_input_current(pca9468, pca9468->pdata->iin_cfg);
	if (ret < 0)
		return ret;

	/* v float voltage */
	ret = pca9468_set_vfloat(pca9468, pca9468->pdata->v_float);
	if (ret < 0)
		return ret;

	return ret;
}

static irqreturn_t pca9468_interrupt_handler(int irq, void *data)
{
	struct pca9468_charger *pca9468 = data;
	/* INT1, INT1_MSK, INT1_STS, STS_A, B, C, D */
	u8 int1[REG_INT1_MAX], sts[REG_STS_MAX];
	u8 masked_int;	/* masked int */
	bool handled = false;
	int ret;

	/* Read INT1, INT1_MSK, INT1_STS */
	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1, int1, 3);
	if (ret < 0) {
		dev_warn(pca9468->dev, "reading INT1_X failed\n");
		return IRQ_NONE;
	}

	/* Read STS_A, B, C, D */
	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_A, sts, 4);
	if (ret < 0) {
		dev_warn(pca9468->dev, "reading STS_X failed\n");
		return IRQ_NONE;
	}

	pr_debug("%s: int1=0x%2x, int1_sts=0x%2x, sts_a=0x%2x\n", __func__,
			int1[REG_INT1], int1[REG_INT1_STS], sts[REG_STS_A]);

	/* Check Interrupt */
	masked_int = int1[REG_INT1] & !int1[REG_INT1_MSK];
	if (masked_int & PCA9468_BIT_V_OK_INT) {
		/* V_OK interrupt happened */
		mutex_lock(&pca9468->lock);
		pca9468->mains_online = !!(int1[REG_INT1_STS] &
					PCA9468_BIT_V_OK_STS);

		/* TODO: alex perform a clean shutdown */
		mutex_unlock(&pca9468->lock);
		power_supply_changed(pca9468->mains);
		handled = true;
	}

	if (masked_int & PCA9468_BIT_NTC_TEMP_INT) {
		/* NTC_TEMP interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_NTC_TEMP_STS) {
			/* above NTC_THRESHOLD */
			dev_err(pca9468->dev, "charging stopped due to NTC threshold voltage\n");
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_CHG_PHASE_INT) {
		/* CHG_PHASE interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_CHG_PHASE_STS) {
			/* Any of loops is active*/
			if (sts[REG_STS_A] & PCA9468_BIT_VFLT_LOOP_STS)	{
				/* V_FLOAT loop is in regulation */
				pr_debug("%s: V_FLOAT loop interrupt\n",
					__func__);
				/* Disable CHG_PHASE_M */
				ret = regmap_update_bits(pca9468->regmap,
						PCA9468_REG_INT1_MSK,
						PCA9468_BIT_CHG_PHASE_M,
						PCA9468_BIT_CHG_PHASE_M);
				if (ret < 0) {
					handled = false;
					return handled;
				}

				/* Go to Pre CV Mode */
				pca9468->timer_id = TIMER_ENTER_CVMODE;
				pca9468->timer_period = 10;
				mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
					msecs_to_jiffies(pca9468->timer_period));

			} else if (sts[REG_STS_A] & PCA9468_BIT_IIN_LOOP_STS) {
				/* IIN loop or ICHG loop is in regulation */
				pr_debug("%s: IIN loop interrupt\n", __func__);
			} else if (sts[REG_STS_A] & PCA9468_BIT_CHG_LOOP_STS) {
				/* ICHG loop is in regulation */
				pr_debug("%s: ICHG loop interrupt\n", __func__);
			}
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_CTRL_LIMIT_INT) {
		/* CTRL_LIMIT interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_CTRL_LIMIT_STS) {
			/* No Loop is active or OCP */
			if (sts[REG_STS_B] & PCA9468_BIT_OCP_FAST_STS) {
				/* Input fast over current */
				dev_err(pca9468->dev, "IIN > 50A instantaneously\n");
			}
			if (sts[REG_STS_B] & PCA9468_BIT_OCP_AVG_STS) {
				/* Input average over current */
				dev_err(pca9468->dev, "IIN > IIN_CFG*150percent\n");
			}
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_TEMP_REG_INT) {
		/* TEMP_REG interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_TEMP_REG_STS) {
			/* Device is in temperature regulation */
			dev_err(pca9468->dev, "Device is in temperature regulation\n");
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_ADC_DONE_INT) {
		/* ADC complete interrupt happened */
		dev_dbg(pca9468->dev, "ADC has been completed\n");
		handled = true;
	}

	if (masked_int & PCA9468_BIT_TIMER_INT) {
		/* Timer falut interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_TIMER_STS) {
			if (sts[REG_STS_B] & PCA9468_BIT_CHARGE_TIMER_STS) {
				/* Charger timer is expired */
				dev_err(pca9468->dev, "Charger timer is expired\n");
			}
			if (sts[REG_STS_B] & PCA9468_BIT_WATCHDOG_TIMER_STS) {
				/* Watchdog timer is expired */
				dev_err(pca9468->dev, "Watchdog timer is expired\n");
			}
		}
		handled = true;
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int pca9468_irq_init(struct pca9468_charger *pca9468,
			    struct i2c_client *client)
{
	const struct pca9468_platform_data *pdata = pca9468->pdata;
	int ret, msk, irq;

	irq = gpio_to_irq(pdata->irq_gpio);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, pca9468_interrupt_handler,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   client->name, pca9468);
	if (ret < 0)
		goto fail_gpio;

	/* disable all interrupts by default. */
	msk = PCA9468_BIT_V_OK_M |
	      PCA9468_BIT_NTC_TEMP_M |
	      PCA9468_BIT_CHG_PHASE_M |
	      PCA9468_BIT_RESERVED_M |
	      PCA9468_BIT_CTRL_LIMIT_M |
	      PCA9468_BIT_TEMP_REG_M |
	      PCA9468_BIT_ADC_DONE_M |
	      PCA9468_BIT_TIMER_M;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_INT1_MSK, msk);
	if (ret < 0)
		goto fail_write;

	client->irq = irq;
	return 0;

fail_write:
	free_irq(irq, pca9468);
fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	client->irq = 0;
	return ret;
}

/* Returns the input current limit programmed into the charger in uA. */
int pca9468_input_current_limit(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	if (!pca9468->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_IIN_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_IIN_CFG) * 100000;

	if (intval < 500000)
		intval = 500000;

	return intval;
}

/* Returns the constant charge current requested from GCPM */
static int get_const_charge_current(struct pca9468_charger *pca9468)
{
	/* Charging current cannot be controlled directly */
	return pca9468->cc_max;
}

/* Return the constant charge voltage programmed into the charger in uV. */
static int pca9468_const_charge_voltage(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	if (!pca9468->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_V_FLOAT, &val);
	if (ret < 0)
		return ret;

	intval = (val * 5 + 3725) * 1000;

	return intval;
}

#define get_boot_sec() div_u64(ktime_to_ns(ktime_get_boottime()), NSEC_PER_SEC)

/* index is the PPS source to use */
static int pca9468_set_charging_enabled(struct pca9468_charger *pca9468, int index)
{
	if (index < 0 || index >= PPS_INDEX_MAX)
		return -EINVAL;

	if (index == 0) {
		/* this is the same as stop charging */
		pca9468->pps_index = 0;

		cancel_delayed_work(&pca9468->timer_work);
		cancel_delayed_work(&pca9468->pps_work);

		/* will call pca9468_stop_charging() in timer_work() */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_ID_NONE;
		pca9468->timer_period = 0;
		mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
				 msecs_to_jiffies(pca9468->timer_period));
		mutex_unlock(&pca9468->lock);
	} else if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
		/* Start Direct Charging on Index */
		pca9468->dc_start_time = get_boot_sec();
		pca9468->irdrop_comp_ok = false;
		pca9468->pps_index = index;

		/* Start battery check */
		mutex_lock(&pca9468->lock);
		pca9468->charging_state = DC_STATE_CHECK_VBAT;
		pca9468->timer_id = TIMER_VBATMIN_CHECK;

		/* PD is already in PE_SNK_STATE */
		pca9468->timer_period = 0;
		mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
				 msecs_to_jiffies(pca9468->timer_period));
		mutex_unlock(&pca9468->lock);

		/* Set the initial charging step */
		power_supply_changed(pca9468->mains);
	} else {
		pr_debug("%s: ping pps_idx=%d charging_state=%d timer_id=%d\n",
			 __func__, pca9468->pps_index, pca9468->charging_state,
			 pca9468->timer_id);
	}

	return 0;
}

static int pca9468_mains_set_property(struct power_supply *psy,
				      enum power_supply_property prop,
				      const union power_supply_propval *val)
{
	struct pca9468_charger *pca9468 = power_supply_get_drvdata(psy);
	int ret = 0;

	pr_debug("%s: =========START=========\n", __func__);
	pr_debug("%s: prop=%d, val=%d\n", __func__, prop, val->intval);
	if (!pca9468->init_done)
		return -EAGAIN;

	switch (prop) {

	case POWER_SUPPLY_PROP_ONLINE:
		if (val->intval == 0) {
			ret = pca9468_stop_charging(pca9468);
			if (ret < 0)
				pr_err("%s: cannot stop charging (%d)\n",
				       __func__, ret);

			pca9468->mains_online = false;
		} else if (pca9468->mains_online == false) {
			ret = pca9468_hw_init(pca9468);
			if (ret < 0)
				pr_err("%s: cannot online (%d)\n",
				       __func__, ret);
			else
				pca9468->mains_online = true;
		}

		break;

	/* TODO: locking is wrong */
	case GBMS_PROP_CHARGING_ENABLED:
		ret = pca9468_set_charging_enabled(pca9468, val->intval);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = pca9468_set_new_vfloat(pca9468, val->intval);
		break;

	/*
	 * pcaA9468 cannot control charging current directly so need to control
	 * current on TA side resolving cc_max for TA_VOL*TA_CUT on vbat.
	 * NOTE: iin should be equivalent to iin = cc_max /2
	 */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = pca9468_set_new_cc_max(pca9468, val->intval);
		break;

	/* CURRENT MAX, same as  is really only set by the algo */
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		pr_debug("%s: set iin %d, ignore\n", __func__, val->intval);
		break;

	/* allow direct setting, not used */
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		mutex_lock(&pca9468->lock);
		ret = pca9468_set_new_iin(pca9468, val->intval);
		mutex_unlock(&pca9468->lock);
		break;

	case GBMS_PROP_CHARGE_DISABLE:
		break;

	default:
		ret = -EINVAL;
		break;
	}

	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int pca9468_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct pca9468_charger *pca9468 = power_supply_get_drvdata(psy);
	union gbms_charger_state chg_state;
	int intval, rc, ret = 0;

	if (!pca9468->init_done)
		return -EAGAIN;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pca9468->mains_online;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = pca9468_is_present(pca9468);
		if (val->intval < 0)
			val->intval = 0;
		break;

	case GBMS_PROP_CHARGE_DISABLE:
		ret = pca9468_get_charging_enabled(pca9468);
		if (ret < 0)
			return ret;
		val->intval = !ret;
		break;

	case GBMS_PROP_CHARGING_ENABLED:
		ret = pca9468_get_charging_enabled(pca9468);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = pca9468_const_charge_voltage(pca9468);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = get_const_charge_current(pca9468);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = pca9468_input_current_limit(pca9468);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* return the output current - uA unit */
		rc = pca9468_get_iin(pca9468, &val->intval);
		if (rc < 0)
			dev_err(pca9468->dev, "Invalid IIN ADC (%d)\n", rc);
		break;

	case GBMS_PROP_CHARGE_CHARGER_STATE:
		ret = pca9468_get_chg_chgr_state(pca9468, &chg_state);
		if (ret < 0)
			return ret;
		if (pca9468->irdrop_comp_ok)
			chg_state.f.flags &= ~GBMS_CS_FLAG_NOCOMP;
		gbms_propval_int64val(val) = chg_state.v;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		intval = pca9468_read_adc(pca9468, ADCCH_VOUT);
		if (intval < 0)
			return intval;
		val->intval = intval;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		intval = pca9468_read_adc(pca9468, ADCCH_VBAT);
		if (intval < 0)
			return intval;
		val->intval = intval;
		break;

	/* TODO: read NTC temperature? */
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = pca9468_read_adc(pca9468, ADCCH_DIETEMP);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = pca9468_get_charge_type(pca9468);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = pca9468_get_status(pca9468);
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = pca9468_input_current_limit(pca9468);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * GBMS not visible
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
 */
static enum power_supply_property pca9468_mains_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_TEMP,
	/* same as POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT */
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
};

static int pca9468_mains_is_writeable(struct power_supply *psy,
				      enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGE_DISABLE:
		return 1;
	default:
		break;
	}

	return 0;
}

static bool pca9468_is_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case PCA9468_REG_DEVICE_INFO ... PCA9468_REG_STS_ADC_9:
	case PCA9468_REG_IIN_CTRL ... PCA9468_REG_NTC_TH_2:
	case PCA9468_REG_ADC_ACCESS:
	case PCA9468_REG_ADC_ADJUST:
	case PCA9468_REG_ADC_IMPROVE:
	case PCA9468_REG_ADC_MODE:
	case 0x40 ... 0x4f: /* debug */
		return true;
	default:
		break;
	}

	return false;
}

static const struct regmap_config pca9468_regmap = {
	.name		= "pca9468-mains",
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= PCA9468_MAX_REGISTER,
	.readable_reg = pca9468_is_reg,
	.volatile_reg = pca9468_is_reg,
};

static const struct power_supply_desc pca9468_mains_desc = {
	.name		= "pca9468-mains",
	/* b/179246019 will not look online to Android */
	.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property	= pca9468_mains_get_property,
	.set_property 	= pca9468_mains_set_property,
	.properties	= pca9468_mains_properties,
	.property_is_writeable = pca9468_mains_is_writeable,
	.num_properties	= ARRAY_SIZE(pca9468_mains_properties),
};

#if defined(CONFIG_OF)
static int of_pca9468_dt(struct device *dev,
			 struct pca9468_platform_data *pdata)
{
	struct device_node *np_pca9468 = dev->of_node;
	int ret;
	if(!np_pca9468)
		return -EINVAL;

	/* irq gpio */
	pdata->irq_gpio = of_get_named_gpio(np_pca9468, "pca9468,irq-gpio", 0);
	pr_info("%s: irq-gpio: %d \n", __func__, pdata->irq_gpio);

	/* input current limit */
	ret = of_property_read_u32(np_pca9468, "pca9468,input-current-limit",
				   &pdata->iin_cfg);
	if (ret) {
		pr_warn("%s: pca9468,input-current-limit is Empty\n", __func__);
		pdata->iin_cfg = PCA9468_IIN_CFG_DFT;
	}
	pr_info("%s: pca9468,iin_cfg is %u\n", __func__, pdata->iin_cfg);

	/* charging float voltage */
	ret = of_property_read_u32(np_pca9468, "pca9468,float-voltage",
				   &pdata->v_float);
	if (ret) {
		pr_warn("%s: pca9468,float-voltage is Empty\n", __func__);
		pdata->v_float = PCA9468_VFLOAT_DFT;
	}
	pr_info("%s: pca9468,v_float is %u\n", __func__, pdata->v_float);

	/* input topoff current */
	ret = of_property_read_u32(np_pca9468, "pca9468,input-itopoff",
				   &pdata->iin_topoff);
	if (ret) {
		pr_warn("%s: pca9468,input-itopoff is Empty\n", __func__);
		pdata->iin_topoff = PCA9468_IIN_DONE_DFT;
	}
	pr_info("%s: pca9468,iin_topoff is %u\n", __func__, pdata->iin_topoff);

	/* switching frequency */
	ret = of_property_read_u32(np_pca9468, "pca9468,switching-frequency",
				   &pdata->fsw_cfg);
	if (ret) {
		pr_warn("%s: pca9468,switching frequency is Empty\n", __func__);
		pdata->fsw_cfg = PCA9468_FSW_CFG_DFT;
	}
	pr_info("%s: pca9468,fsw_cfg is %u\n", __func__, pdata->fsw_cfg);

	/* NTC threshold voltage */
	ret = of_property_read_u32(np_pca9468, "pca9468,ntc-threshold",
				   &pdata->ntc_th);
	if (ret) {
		pr_warn("%s: pca9468,ntc threshold voltage is Empty\n",
			__func__);
		pdata->ntc_th = PCA9468_NTC_TH_DFT;
	}
	pr_info("%s: pca9468,ntc_th is %u\n", __func__, pdata->ntc_th);

#ifdef CONFIG_THERMAL
	/* USBC thermal zone */
	ret = of_property_read_string(np_pca9468, "google,usb-port-tz-name",
				      &pdata->usb_tz_name);
	if (ret) {
		pr_info("%s: google,usb-port-tz-name is Empty\n", __func__);
		pdata->usb_tz_name = NULL;
	} else {
		pr_info("%s: google,usb-port-tz-name is %s\n", __func__,
			pdata->usb_tz_name);
	}
#endif

	return 0;
}
#else
static int of_pca9468_dt(struct device *dev,
			 struct pca9468_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

#ifdef CONFIG_THERMAL
static int pca9468_usb_tz_read_temp(struct thermal_zone_device *tzd, int *temp)
{
	struct pca9468_charger *pca9468 = tzd->devdata;

	if (!pca9468)
		return -ENODEV;
	*temp = pca9468_read_adc(pca9468, ADCCH_NTC);

	return 0;
}

static struct thermal_zone_device_ops pca9468_usb_tzd_ops = {
	.get_temp = pca9468_usb_tz_read_temp,
};
#endif

static int read_reg(void *data, u64 *val)
{
	struct pca9468_charger *chip = data;
	int rc;
	unsigned int temp;

	rc = regmap_read(chip->regmap, chip->debug_address, &temp);
	if (rc) {
		pr_err("Couldn't read reg %x rc = %d\n",
			chip->debug_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int write_reg(void *data, u64 val)
{
	struct pca9468_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = regmap_write(chip->regmap, chip->debug_address, temp);
	if (rc) {
		pr_err("Couldn't write 0x%02x to 0x%02x rc = %d\n",
			temp, chip->debug_address, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(register_debug_ops, read_reg, write_reg, "0x%02llx\n");

static int debug_adc_chan_get(void *data, u64 *val)
{
	struct pca9468_charger *pca9468 = data;

	*val = pca9468_read_adc(data, pca9468->debug_adc_channel);
	return 0;
}

static int debug_adc_chan_set(void *data, u64 val)
{
	struct pca9468_charger *pca9468 = data;

	if (val < ADCCH_VOUT || val >= ADCCH_MAX)
		return -EINVAL;
	pca9468->debug_adc_channel = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_adc_chan_ops, debug_adc_chan_get,
			debug_adc_chan_set, "%llu\n");


static int debug_pps_index_get(void *data, u64 *val)
{
	struct pca9468_charger *pca9468 = data;

	*val = pca9468->pps_index;
	return 0;
}

static int debug_pps_index_set(void *data, u64 val)
{
	struct pca9468_charger *pca9468 = data;

	return pca9468_set_charging_enabled(pca9468, (int)val);
}

DEFINE_SIMPLE_ATTRIBUTE(debug_pps_index_ops, debug_pps_index_get,
			debug_pps_index_set, "%llu\n");

static ssize_t show_sts_ab(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pca9468_charger *pca9468 = dev_get_drvdata(dev);
	u8 tmp[2];
	int ret;

	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_A, &tmp, sizeof(tmp));
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%02x%02x\n", tmp[0], tmp[1]);
}

static DEVICE_ATTR(sts_ab, 0444, show_sts_ab, NULL);


static int pca9468_create_fs_entries(struct pca9468_charger *chip)
{

	device_create_file(chip->dev, &dev_attr_sts_ab);


	chip->debug_root = debugfs_create_dir("charger-pca9468", NULL);
	if (IS_ERR_OR_NULL(chip->debug_root)) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		return -ENOENT;
	}

	debugfs_create_file("data", 0644, chip->debug_root, chip, &register_debug_ops);
	debugfs_create_x32("address", 0644, chip->debug_root, &chip->debug_address);

	chip->debug_adc_channel = ADCCH_VOUT;
	debugfs_create_file("adc_chan", 0644, chip->debug_root, chip,
			    &debug_adc_chan_ops);
	debugfs_create_file("pps_index", 0644, chip->debug_root, chip,
			    &debug_pps_index_ops);
	debugfs_create_bool("irdrop_comp", 0644, chip->debug_root,
			    &chip->irdrop_comp_ok);

	return 0;
}


static int pca9468_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	static char *battery[] = { "pca9468-battery" };
	struct power_supply_config mains_cfg = {};
	struct pca9468_platform_data *pdata;
	struct pca9468_charger *pca9468_chg;
	struct device *dev = &client->dev;
	int ret;

	pr_debug("%s: =========START=========\n", __func__);

	pca9468_chg = devm_kzalloc(dev, sizeof(*pca9468_chg), GFP_KERNEL);
	if (!pca9468_chg)
		return -ENOMEM;

#if defined(CONFIG_OF)
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct pca9468_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory \n");
			return -ENOMEM;
		}

		ret = of_pca9468_dt(&client->dev, pdata);
		if (ret < 0){
			dev_err(&client->dev, "Failed to get device of_node \n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
	} else {
		pdata = client->dev.platform_data;
	}
#else
	pdata = dev->platform_data;
#endif
	if (!pdata)
		return -EINVAL;

	i2c_set_clientdata(client, pca9468_chg);

	mutex_init(&pca9468_chg->lock);
	pca9468_chg->dev = &client->dev;
	pca9468_chg->pdata = pdata;
	pca9468_chg->charging_state = DC_STATE_NO_CHARGING;

	/* Create a work queue for the direct charger */
	pca9468_chg->dc_wq = alloc_ordered_workqueue("pca9468_dc_wq", WQ_MEM_RECLAIM);
	if (pca9468_chg->dc_wq == NULL) {
		dev_err(pca9468_chg->dev, "failed to create work queue\n");
		return -ENOMEM;
	}

	pca9468_chg->monitor_wake_lock =
		wakeup_source_register(NULL, "pca9468-charger-monitor");
	if (!pca9468_chg->monitor_wake_lock) {
		pr_err("Failed to register wakeup source\n");
		return -ENODEV;
	}

	/* initialize work */
	INIT_DELAYED_WORK(&pca9468_chg->timer_work, pca9468_timer_work);
	pca9468_chg->timer_id = TIMER_ID_NONE;
	pca9468_chg->timer_period = 0;

	INIT_DELAYED_WORK(&pca9468_chg->pps_work, pca9468_pps_request_work);

	pca9468_chg->regmap = devm_regmap_init_i2c(client, &pca9468_regmap);
	if (IS_ERR(pca9468_chg->regmap)) {
		ret = -EINVAL;
		goto error;
	}

	ret = pca9468_probe_pps(pca9468_chg);
	if (ret < 0) {
		pr_warn("pca9468: PPS not available (%d)\n", ret);
	} else {
		const char *logname = "pca9468";

		pca9468_chg->log = logbuffer_register(logname);
		if (IS_ERR(pca9468_chg->log)) {
			pr_err("%s: no logbuffer (%ld)\n", __func__,
			       PTR_ERR(pca9468_chg->log));
			pca9468_chg->log = NULL;
		}
	}

	ret = pca9468_hw_ping(pca9468_chg);
	if (ret < 0)
		goto error;

	/* TODO: only enable ADC if usb_tz_name is defined */
	ret = pca9468_hw_init(pca9468_chg);
	if (ret < 0)
		goto error;


	mains_cfg.supplied_to = battery;
	mains_cfg.num_supplicants = ARRAY_SIZE(battery);
	mains_cfg.drv_data = pca9468_chg;
	pca9468_chg->mains = devm_power_supply_register(dev,
							&pca9468_mains_desc,
							&mains_cfg);
	if (IS_ERR(pca9468_chg->mains)) {
		ret = -ENODEV;
		goto error;
	}

	/* Interrupt pin is optional. */
	if (pdata->irq_gpio >= 0) {
		ret = pca9468_irq_init(pca9468_chg, client);
		if (ret < 0) {
			dev_warn(dev, "failed to initialize IRQ: %d\n", ret);
			dev_warn(dev, "disabling IRQ support\n");
		}

		/* disable interrupt */
		disable_irq(client->irq);
	}

	ret = pca9468_create_fs_entries(pca9468_chg);
	if (ret < 0)
		dev_err(dev, "error while registering debugfs %d\n", ret);

#ifdef CONFIG_THERMAL
	if (pdata->usb_tz_name) {
		pca9468_chg->usb_tzd =
			thermal_zone_device_register(pdata->usb_tz_name, 0, 0,
						     pca9468_chg,
						     &pca9468_usb_tzd_ops,
						     NULL, 0, 0);
		if (IS_ERR(pca9468_chg->usb_tzd)) {
			pca9468_chg->usb_tzd = NULL;
			ret = PTR_ERR(pca9468_chg->usb_tzd);
			dev_err(dev, "Couldn't register usb connector thermal zone ret=%d\n",
				ret);
		}
	}
#endif
	pca9468_chg->init_done = true;
	pr_info("pca9468: probe_done\n");
	pr_debug("%s: =========END=========\n", __func__);
	return 0;

error:
	destroy_workqueue(pca9468_chg->dc_wq);
	mutex_destroy(&pca9468_chg->lock);
	wakeup_source_unregister(pca9468_chg->monitor_wake_lock);
	return ret;
}

static int pca9468_remove(struct i2c_client *client)
{
	struct pca9468_charger *pca9468_chg = i2c_get_clientdata(client);

	/* stop charging if it is active */
	pca9468_stop_charging(pca9468_chg);

	if (client->irq) {
		free_irq(client->irq, pca9468_chg);
		gpio_free(pca9468_chg->pdata->irq_gpio);
	}

	destroy_workqueue(pca9468_chg->dc_wq);

	wakeup_source_unregister(pca9468_chg->monitor_wake_lock);

#ifdef CONFIG_THERMAL
	if (pca9468_chg->usb_tzd)
		thermal_zone_device_unregister(pca9468_chg->usb_tzd);
#endif
	if (pca9468_chg->log)
		logbuffer_unregister(pca9468_chg->log);
	pps_free(&pca9468_chg->pps_data);

	return 0;
}

static const struct i2c_device_id pca9468_id[] = {
	{ "pca9468", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca9468_id);

#if defined(CONFIG_OF)
static struct of_device_id pca9468_i2c_dt_ids[] = {
	{ .compatible = "nxp,pca9468" },
	{ },
};
MODULE_DEVICE_TABLE(of, pca9468_i2c_dt_ids);
#endif /* CONFIG_OF */

#if defined(CONFIG_PM)
#ifdef CONFIG_RTC_HCTOSYS
static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	*now_tm_sec = rtc_tm_to_time64(&tm);

close_time:
	rtc_class_close(rtc);
	return rc;
}

static void
pca9468_check_and_update_charging_timer(struct pca9468_charger *pca9468)
{
	unsigned long current_time = 0, next_update_time, time_left;

	get_current_time(&current_time);

	if (pca9468->timer_id != TIMER_ID_NONE)	{
		next_update_time = pca9468->last_update_time +
				(pca9468->timer_period / 1000); /* seconds */

		pr_debug("%s: current_time=%ld, next_update_time=%ld\n",
			__func__, current_time, next_update_time);

		if (next_update_time > current_time)
			time_left = next_update_time - current_time;
		else
			time_left = 0;

		mutex_lock(&pca9468->lock);
		pca9468->timer_period = time_left * 1000; /* ms unit */
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->timer_work,
				msecs_to_jiffies(pca9468->timer_period));

		pr_debug("%s: timer_id=%d, time_period=%ld\n", __func__,
			 pca9468->timer_id, pca9468->timer_period);
	}
	pca9468->last_update_time = current_time;
}
#endif

static int pca9468_suspend(struct device *dev)
{
	struct pca9468_charger *pca9468 = dev_get_drvdata(dev);

	pr_debug("%s: cancel delayed work\n", __func__);

	/* cancel delayed_work */
	cancel_delayed_work(&pca9468->timer_work);
	return 0;
}

static int pca9468_resume(struct device *dev)
{
	struct pca9468_charger *pca9468 = dev_get_drvdata(dev);

	pr_debug("%s: update_timer\n", __func__);

	/* Update the current timer */
#ifdef CONFIG_RTC_HCTOSYS
	pca9468_check_and_update_charging_timer(pca9468);
#else
	if (pca9468->timer_id != TIMER_ID_NONE) {
		mutex_lock(&pca9468->lock);
		pca9468->timer_period = 0;	/* ms unit */
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->timer_work,
				      msecs_to_jiffies(pca9468->timer_period));
	}
#endif
	return 0;
}
#else
#define pca9468_suspend		NULL
#define pca9468_resume		NULL
#endif

const struct dev_pm_ops pca9468_pm_ops = {
	.suspend = pca9468_suspend,
	.resume = pca9468_resume,
};

static struct i2c_driver pca9468_driver = {
	.driver = {
		.name = "pca9468",
#if defined(CONFIG_OF)
		.of_match_table = pca9468_i2c_dt_ids,
#endif /* CONFIG_OF */
#if defined(CONFIG_PM)
		.pm = &pca9468_pm_ops,
#endif
	},
	.probe        = pca9468_probe,
	.remove       = pca9468_remove,
	.id_table     = pca9468_id,
};

module_i2c_driver(pca9468_driver);

MODULE_AUTHOR("Clark Kim <clark.kim@nxp.com>");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_AUTHOR("Wasb Liu <wasbliu@google.com>");
MODULE_DESCRIPTION("PCA9468 gcharger driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("3.7.0");
