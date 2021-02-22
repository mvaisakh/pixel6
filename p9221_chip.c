/* SPDX-License-Identifier: GPL-2.0 */
/*
 * P9221 Wireless Charger Driver chip revision specific functions.
 *
 * Copyright (C) 2020 Google, LLC
 *
 */

#include <linux/device.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/alarmtimer.h>
#include <misc/logbuffer.h>
#include "p9221_charger.h"

/* Simple Chip Specific Accessors */
/*
 * chip_get_rx_ilim
 *
 *   Get the receive current limit (mA).
 */
static int p9221_chip_get_rx_ilim(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u8 val;

	ret = chgr->reg_read_8(chgr, P9221R5_ILIM_SET_REG, &val);
	if (ret)
		return ret;

	*ma = ((val * 100) + 200);
	return 0;
}

static int p9412_chip_get_rx_ilim(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u8 val;

	ret = chgr->reg_read_8(chgr, P9221R5_ILIM_SET_REG, &val);
	if (ret)
		return ret;

	*ma = (val + 1) * 100;
	return 0;
}

static int p9222_chip_get_rx_ilim(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u8 val;

	ret = chgr->reg_read_8(chgr, P9222RE_ILIM_SET_REG, &val);
	if (ret)
		return ret;

	*ma = ((val * 100) + 100);
	return 0;
}

/*
 * chip_set_rx_ilim
 *
 *   Set the receive current limit (mA).
 */
static int p9221_chip_set_rx_ilim(struct p9221_charger_data *chgr, u32 ma)
{
	u8 val;

	/* uA -> 0.1A, offset 0.2A */
	if (ma < P9221_RX_ILIM_MIN_MA)
		return -EINVAL;

	if (ma > P9221_RX_ILIM_MAX_MA)
		ma = P9221_RX_ILIM_MAX_MA;

	val = (ma / 100) - 2;
	return chgr->reg_write_8(chgr, P9221R5_ILIM_SET_REG, val);
}

static int p9412_chip_set_rx_ilim(struct p9221_charger_data *chgr, u32 ma)
{
	u8 val;

	if (ma > P9412_RX_ILIM_MAX_MA)
		return -EINVAL;

	val = (ma / 100) - 1;
	return chgr->reg_write_8(chgr, P9221R5_ILIM_SET_REG, val);
}

static int p9222_chip_set_rx_ilim(struct p9221_charger_data *chgr, u32 ma)
{
	u8 val;

	if (ma < P9222_RX_ILIM_MIN_MA)
		return -EINVAL;

	if (ma > P9222_RX_ILIM_MAX_MA)
		ma = P9222_RX_ILIM_MAX_MA;

	val = (ma / 100) - 1;
	return chgr->reg_write_8(chgr, P9222RE_ILIM_SET_REG, val);
}

/*
 * chip_get_tx_ilim
 *
 *   Get the transmit current limit (mA).
 */
static int p9221_chip_get_tx_ilim(struct p9221_charger_data *chgr, u32 *ma)
{
	return -ENOTSUPP;
}

static int p9382_chip_get_tx_ilim(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9382A_ILIM_SET_REG, &val);
	if (ret)
		return ret;

	*ma = (u32) val;
	return 0;
}

static int p9412_chip_get_tx_ilim(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9412_TX_I_API_LIM_REG, &val);
	if (ret)
		return ret;

	*ma = (u32) val;
	return 0;
}

/*
 * chip_set_tx_ilim
 *
 *   Set the transmit current limit (mA).
 */
static int p9221_chip_set_tx_ilim(struct p9221_charger_data *chgr, u32 ma)
{
	return -ENOTSUPP;
}

static int p9382_chip_set_tx_ilim(struct p9221_charger_data *chgr, u32 ma)
{
	if ((ma < 0) || (ma > P9382A_RTX_ICL_MAX_MA))
		return -EINVAL;

	return chgr->reg_write_16(chgr, P9382A_ILIM_SET_REG, (u16) ma);
}

static int p9412_chip_set_tx_ilim(struct p9221_charger_data *chgr, u32 ma)
{
	if ((ma < 0) || (ma > P9382A_RTX_ICL_MAX_MA))
		return -EINVAL;

	return chgr->reg_write_16(chgr, P9412_TX_I_API_LIM_REG, (u16) ma);
}

/*
 * chip_get_die_temp
 *
 *   Get the chip temperature (milli-C).
 */
static int p9221_chip_get_die_temp(struct p9221_charger_data *chgr, int *mc)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9221R5_DIE_TEMP_ADC_REG, &val);
	if (ret)
		return ret;

	*mc = (val * 10 / 107 - 247) * 10;
	return 0;
}

static int p9412_chip_get_die_temp(struct p9221_charger_data *chgr, int *mc)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9412_DIE_TEMP_REG, &val);
	if (ret)
		return ret;

	*mc = P9221_C_TO_MILLIC(val);
	return 0;
}

static int p9222_chip_get_die_temp(struct p9221_charger_data *chgr, int *mc)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9222RE_DIE_TEMP_REG, &val);
	if (ret)
		return ret;

	*mc = P9221_C_TO_MILLIC(val);
	return 0;
}

/*
 * chip_get_iout
 *
 * get measure of current out (mA).
 */
static int p9xxx_chip_get_iout(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9221R5_IOUT_REG, &val);
	if (ret)
		return ret;

	*ma = val;
	return 0;
}

static int p9222_chip_get_iout(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9222RE_IOUT_REG, &val);
	if (ret)
		return ret;

	*ma = val;
	return 0;
}

/*
 * chip_get_vout
 *
 *   get voltage out (mV).
 */
static int p9xxx_chip_get_vout(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9221R5_VOUT_REG, &val);
	if (ret)
		return ret;

	*mv = val;
	return 0;
}

static int p9222_chip_get_vout(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9222RE_VOUT_REG, &val);
	if (ret)
		return ret;

	*mv = val;
	return 0;
}

/*
 * chip_get_vrect
 *
 *   Get rectified voltage out (mV).
 */
static int p9xxx_chip_get_vrect(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9221R5_VRECT_REG, &val);
	if (ret)
		return ret;

	*mv = val;
	return 0;
}

static int p9222_chip_get_vrect(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9222RE_VRECT_REG, &val);
	if (ret)
		return ret;

	*mv = val;
	return 0;
}

/*
 * chip_get_op_freq
 *
 *   Get operating frequency (KHz).
 */
static int p9xxx_chip_get_op_freq(struct p9221_charger_data *chgr, u32 *khz)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9221R5_OP_FREQ_REG, &val);
	if (ret)
		return ret;

	*khz = (u32) val;
	return 0;
}

static int p9222_chip_get_op_freq(struct p9221_charger_data *chgr, u32 *khz)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9222RE_OP_FREQ_REG, &val);
	if (ret)
		return ret;

	*khz = (u32) val;
	return 0;
}

/*
 * chip_get_vout_max
 *
 *   Get the maximum output voltage (mV).
 */
static int p9221_chip_get_vout_max(struct p9221_charger_data *chgr, u32 *mv)
{
	return -ENOTSUPP;
}

static int p9832_chip_get_vout_max(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u8 val;

	ret = chgr->reg_read_8(chgr, P9221R5_VOUT_SET_REG, &val);
	if (ret)
		return ret;

	*mv = val * 100; /* 100 mV units */
	return 0;
}

static int p9412_chip_get_vout_max(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9412_VOUT_SET_REG, &val);
	if (ret)
		return ret;

	*mv = val * 10; /* 10 mV units */
	return 0;
}

/*
 * chip_set_vout_max
 *
 *   Set the maximum output voltage (mV).
 */
static int p9221_chip_set_vout_max(struct p9221_charger_data *chgr, u32 mv)
{
	return -ENOTSUPP;
}

static int p9832_chip_set_vout_max(struct p9221_charger_data *chgr, u32 mv)
{
	int ret;

	if (mv < P9221_VOUT_SET_MIN_MV || mv > chgr->pdata->max_vout_mv)
		return -EINVAL;

	ret = chgr->reg_write_8(chgr, P9221R5_VOUT_SET_REG, mv / 100);
	return ret;
}

static int p9412_chip_set_vout_max(struct p9221_charger_data *chgr, u32 mv)
{
	int ret;

	if (mv < P9412_VOUT_SET_MIN_MV || mv > chgr->pdata->max_vout_mv)
		return -EINVAL;

	ret = chgr->reg_write_16(chgr, P9412_VOUT_SET_REG, mv / 10);
	return ret;
}

/* system mode register */
static int p9221_chip_get_sys_mode(struct p9221_charger_data *chgr, u8 *mode)
{
	int ret;
	u8 val8;

	ret = chgr->reg_read_8(chgr, P9221R5_SYSTEM_MODE_REG, &val8);
	if (ret)
		return ret;

	/* map to p9412 values */
	if (val8 & P9221R5_MODE_WPCMODE)
		*mode = P9412_SYS_OP_MODE_WPC_BASIC;
	else if (val8 & P9382A_MODE_TXMODE)
		*mode = P9412_SYS_OP_MODE_TX_MODE;
	else if (val8 & P9221R5_MODE_EXTENDED)
		*mode = P9412_SYS_OP_MODE_WPC_EXTD;
	else
		*mode = P9412_SYS_OP_MODE_AC_MISSING;
	return 0;
}

static int p9222_chip_get_sys_mode(struct p9221_charger_data *chgr, u8 *mode)
{
	int ret;
	u8 val8;

	ret = chgr->reg_read_8(chgr, P9222RE_SYSTEM_MODE_REG, &val8);
	if (ret)
		return ret;

	/* map to p9412 values */
	if (val8 & P9222_SYS_OP_MODE_WPC_BASIC)
		*mode = P9412_SYS_OP_MODE_WPC_BASIC;
	else if (val8 & P9222_SYS_OP_MODE_WPC_EXTD)
		*mode = P9412_SYS_OP_MODE_WPC_EXTD;
	else
		*mode = P9412_SYS_OP_MODE_AC_MISSING;

	return 0;
}

static int p9412_chip_get_sys_mode(struct p9221_charger_data *chgr, u8 *mode)
{
	return chgr->reg_read_8(chgr, P9221R5_SYSTEM_MODE_REG, mode);
}

/* These are more involved than just chip access */

static int p9382_wait_for_mode(struct p9221_charger_data *chgr, int mode)
{
	int loops, ret;
	uint8_t sys_mode;

	/* 30 * 100 = 3 sec */
	for (loops = 30 ; loops ; loops--) {
		ret = chgr->reg_read_8(chgr, P9221R5_SYSTEM_MODE_REG,
					&sys_mode);
		if (ret < 0) {
			dev_err(&chgr->client->dev,
				"cannot read system_mode (%d)", ret);
			return -EIO;
		}

		if (sys_mode == mode)
			return 0;

		msleep(100);
	}

	return -ETIMEDOUT;
}

/* get the data buf for receive */
static int p9221_get_data_buf(struct p9221_charger_data *chgr,
			      u8 data[], size_t len)
{
	if (!len || len > P9221R5_DATA_RECV_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_read_n(chgr, P9221R5_DATA_RECV_BUF_START, data, len);
}

static int p9382_get_data_buf(struct p9221_charger_data *chgr,
			      u8 data[], size_t len)
{
	if (!len || len > P9221R5_DATA_RECV_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_read_n(chgr, P9382A_DATA_RECV_BUF_START, data, len);
}

static int p9412_get_data_buf(struct p9221_charger_data *chgr,
			      u8 data[], size_t len)
{
	if (!len || len > P9412_DATA_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_read_n(chgr, P9412_DATA_BUF_START, data, len);
}

/* set the data buf for send */
static int p9221_set_data_buf(struct p9221_charger_data *chgr,
			      const u8 data[], size_t len)
{
	if (!len || len > P9221R5_DATA_SEND_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_write_n(chgr, P9221R5_DATA_SEND_BUF_START, data, len);
}

static int p9382_set_data_buf(struct p9221_charger_data *chgr,
			      const u8 data[], size_t len)
{
	if (!len || len > P9221R5_DATA_SEND_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_write_n(chgr, P9382A_DATA_SEND_BUF_START, data, len);
}

static int p9412_set_data_buf(struct p9221_charger_data *chgr,
				   const u8 data[], size_t len)
{
	if (!len || len > P9412_DATA_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_write_n(chgr, P9412_DATA_BUF_START, data, len);
}

/* receive size */
static int p9221_get_cc_recv_size(struct p9221_charger_data *chgr,
				  size_t *len)
{
	int ret;
	u8 len8;

	ret = chgr->reg_read_8(chgr, P9221R5_COM_CHAN_RECV_SIZE_REG,
			       &len8);
	if (!ret)
		*len = len8;
	return ret;
}

static int p9412_get_cc_recv_size(struct p9221_charger_data *chgr,
				  size_t *len)
{
	int ret;
	u16 len16;

	ret = chgr->reg_read_16(chgr, P9412_COM_CHAN_RECV_SIZE_REG,
				&len16);
	if (!ret)
		*len = len16;
	return ret;
}

/* send size */
static int p9221_set_cc_send_size(struct p9221_charger_data *chgr,
				  size_t len)
{
	return chgr->reg_write_8(chgr, P9221R5_COM_CHAN_SEND_SIZE_REG,
				len);
}

static int p9382_set_cc_send_size(struct p9221_charger_data *chgr,
				  size_t len)
{
	int ret;

	/* set packet type to 0x100 */
	ret = chgr->reg_write_8(chgr, P9382A_COM_PACKET_TYPE_ADDR,
				BIDI_COM_PACKET_TYPE);
	if (ret) {
		dev_err(&chgr->client->dev,
			"Failed to write packet type %d\n", ret);
		return ret;
	}

	return chgr->reg_write_16(chgr, P9382A_COM_CHAN_SEND_SIZE_REG,
				 len);
}

static int p9412_set_cc_send_size(struct p9221_charger_data *chgr,
				  size_t len)
{
	int ret;

	/* set packet type to 0x100 */
	ret = chgr->reg_write_8(chgr, P9382A_COM_PACKET_TYPE_ADDR,
				BIDI_COM_PACKET_TYPE);
	if (ret) {
		dev_err(&chgr->client->dev,
			"Failed to write packet type %d\n", ret);
		return ret;
	}

	return chgr->reg_write_16(chgr, P9412_COM_CHAN_SEND_SIZE_REG,
				 len);
}

/* get align x */
static int p9221_get_align_x(struct p9221_charger_data *chgr, u8 *x)
{
	return chgr->reg_read_8(chgr, P9221R5_ALIGN_X_ADC_REG, x);
}

static int p9412_get_align_x(struct p9221_charger_data *chgr, u8 *x)
{
	return chgr->reg_read_8(chgr, P9412_ALIGN_X_REG, x);
}

/* get align y */
static int p9221_get_align_y(struct p9221_charger_data *chgr, u8 *y)
{
	return chgr->reg_read_8(chgr, P9221R5_ALIGN_Y_ADC_REG, y);
}

static int p9412_get_align_y(struct p9221_charger_data *chgr, u8 *y)
{
	return chgr->reg_read_8(chgr, P9412_ALIGN_Y_REG, y);
}

/* Simple Chip Specific Logic Functions */

/* Disable/Enable transmit mode. */
static int p9221_chip_tx_mode(struct p9221_charger_data *chgr, bool enable)
{
	logbuffer_log(chgr->rtx_log, "%s(%d)", __func__, enable);
	return -ENOTSUPP;
}

static int p9382_chip_tx_mode(struct p9221_charger_data *chgr, bool enable)
{
	int ret;
	u16 rev;

	logbuffer_log(chgr->rtx_log, "%s(%d)", __func__, enable);
	if (enable) {
		/* check FW revision */
		ret = chgr->reg_read_16(chgr,
					P9221_OTP_FW_MINOR_REV_REG, &rev);
		if (ret)
			return ret;

		if (rev >= P9382A_FW_REV_25) {
			/* write 0x0003 to 0x69 after rev 25 */
			ret = chgr->reg_write_16(chgr,
						 P9382A_TRX_ENABLE_REG,
						 P9382A_TX_INHIBIT);
		} else {
			/* write 0x0000 to 0x34 */
			ret = chgr->reg_write_16(chgr,
						 P9382A_STATUS_REG, 0);
		}

		/* check 0x4C reads back as 0x04 */
		ret = p9382_wait_for_mode(chgr, P9382A_MODE_TXMODE);
	} else {
		/* Write 0x80 to 0x4E, check 0x4C reads back as 0x0000 */
		ret = chgr->chip_set_cmd(chgr, P9221R5_COM_RENEGOTIATE);
		if (ret == 0) {
			ret = p9382_wait_for_mode(chgr, 0);
			if (ret < 0)
				pr_err("cannot exit rTX mode (%d)\n", ret);
		}
	}

	return ret;
}

static int p9412_chip_tx_mode(struct p9221_charger_data *chgr, bool enable)
{
	int ret;

	logbuffer_log(chgr->rtx_log, "%s(%d)", __func__, enable);
	if (enable) {
		ret = chgr->reg_write_8(chgr, P9412_TX_CMD_REG,
					P9412_TX_CMD_TX_MODE_EN);
		if (ret) {
			logbuffer_log(chgr->rtx_log,
				 "tx_cmd_reg write failed (%d)\n", ret);
			return ret;
		}

		ret = p9382_wait_for_mode(chgr, P9412_SYS_OP_MODE_TX_MODE);
		if (ret)
			logbuffer_log(chgr->rtx_log,
				      "error waiting for tx_mode (%d)", ret);
	} else {
		ret = chgr->chip_set_cmd(chgr, P9221R5_COM_RENEGOTIATE);
		if (ret == 0) {
			ret = p9382_wait_for_mode(chgr, 0);
			if (ret < 0)
				pr_err("cannot exit rTX mode (%d)\n", ret);
		}
	}
	return ret;
}

static int p9221_chip_set_cmd_reg(struct p9221_charger_data *chgr, u8 cmd)
{
	u8 cur_cmd = 0;
	int retry;
	int ret;

	for (retry = 0; retry < P9221_COM_CHAN_RETRIES; retry++) {
		ret = chgr->reg_read_8(chgr, P9221_COM_REG, &cur_cmd);
		if (ret == 0 && cur_cmd == 0)
			break;
		msleep(25);
	}

	if (retry >= P9221_COM_CHAN_RETRIES) {
		dev_err(&chgr->client->dev,
			"Failed to wait for cmd free %02x\n", cur_cmd);
		return -EBUSY;
	}

	ret = chgr->reg_write_8(chgr, P9221_COM_REG, cmd);
	if (ret)
		dev_err(&chgr->client->dev,
			"Failed to set cmd reg %02x: %d\n", cmd, ret);

	return ret;
}

static int p9222_chip_set_cmd_reg(struct p9221_charger_data *chgr, u8 cmd)
{
	u16 cur_cmd = 0;
	int retry;
	int ret;

	for (retry = 0; retry < P9221_COM_CHAN_RETRIES; retry++) {
		ret = chgr->reg_read_16(chgr, P9222_COM_REG, &cur_cmd);
		if (ret == 0 && cur_cmd == 0)
			break;
		msleep(25);
	}

	if (retry >= P9221_COM_CHAN_RETRIES) {
		dev_err(&chgr->client->dev,
			"Failed to wait for cmd free %02x\n", cur_cmd);
		return -EBUSY;
	}

	ret = chgr->reg_write_16(chgr, P9222_COM_REG, (u16)cmd);
	if (ret)
		dev_err(&chgr->client->dev,
			"Failed to set cmd reg %02x: %d\n", (u16)cmd, ret);

	return ret;
}

static int p9412_chip_set_cmd_reg(struct p9221_charger_data *chgr, u16 cmd)
{
	u16 cur_cmd = 0;
	int retry;
	int ret;

	for (retry = 0; retry < P9221_COM_CHAN_RETRIES; retry++) {
		ret = chgr->reg_read_16(chgr, P9221_COM_REG, &cur_cmd);
		if (ret == 0 && cur_cmd == 0)
			break;
		msleep(25);
	}

	if (retry >= P9221_COM_CHAN_RETRIES) {
		dev_err(&chgr->client->dev,
			"Failed to wait for cmd free %02x\n", cur_cmd);
		return -EBUSY;
	}

	ret = chgr->reg_write_16(chgr, P9221_COM_REG, cmd);
	if (ret)
		dev_err(&chgr->client->dev,
			"Failed to set cmd reg %02x: %d\n", cmd, ret);

	return ret;
}

/* ccreset */
static int p9221_send_ccreset(struct p9221_charger_data *chgr)
{
	int ret;

	dev_info(&chgr->client->dev, "Send CC reset\n");

	mutex_lock(&chgr->cmd_lock);

	ret = chgr->reg_write_8(chgr, P9221R5_COM_CHAN_RESET_REG,
				P9221R5_COM_CHAN_CCRESET);
	if (ret == 0)
		ret = chgr->chip_set_cmd(chgr, P9221R5_COM_CCACTIVATE);

	mutex_unlock(&chgr->cmd_lock);
	return ret;
}

static int p9412_send_ccreset(struct p9221_charger_data *chgr)
{
	return -ENOTSUPP;
}

/* send eop */
static int p9221_send_eop(struct p9221_charger_data *chgr, u8 reason)
{
	int ret;

	dev_info(&chgr->client->dev, "Send EOP reason=%d\n", reason);

	mutex_lock(&chgr->cmd_lock);

	ret = chgr->reg_write_8(chgr, P9221R5_EPT_REG, reason);
	if (ret == 0)
		ret = chgr->chip_set_cmd(chgr, P9221R5_COM_SENDEPT);

	mutex_unlock(&chgr->cmd_lock);
	return ret;
}

static int p9412_send_eop(struct p9221_charger_data *chgr, u8 reason)
{
	int ret = 0;

	dev_info(&chgr->client->dev, "Send EOP reason=%d\n", reason);

	mutex_lock(&chgr->cmd_lock);

	/* Command P9412 to send EPT */
	ret = chgr->reg_write_8(chgr, P9221R5_EPT_REG, reason);
	if (ret == 0)
		ret = chgr->chip_set_cmd(chgr, P9221R5_COM_SENDEPT);

	/* Change P9412 mode to Disable Mode */
	ret = chgr->reg_write_8(chgr, P9412_CDMODE_REQ_REG, 0);
	if (ret < 0)
		dev_err(&chgr->client->dev,
			"fail to switch cap to disable mode\n");

	mutex_unlock(&chgr->cmd_lock);

	return ret;
}

/* renegotiate power from charger->pdata->epp_rp_value */
static int p9221_chip_renegotiate_pwr(struct p9221_charger_data *chgr)
{
	int ret;
	int val8 = P9412_MW_TO_HW(chgr->pdata->epp_rp_value);

	/* units 0.5 W*/
	ret = chgr->reg_write_8(chgr,
				P9221R5_EPP_REQ_NEGOTIATED_POWER_REG, val8);
	if (ret < 0)
		dev_err(&chgr->client->dev,
			"cannot write to EPP_NEG_POWER=%d (%d)\n",
			val8, ret);
	return ret;
}

static int p9222_chip_renegotiate_pwr(struct p9221_charger_data *chgr)
{
	int ret;
	int val8 = P9412_MW_TO_HW(chgr->pdata->epp_rp_value);

	/* units 0.5 W*/
	ret = chgr->reg_write_8(chgr,
				P9222RE_EPP_REQ_NEGOTIATED_POWER_REG, val8);
	if (ret < 0)
		dev_err(&chgr->client->dev,
			"cannot write to EPP_NEG_POWER=%d (%d)\n",
			val8, ret);
	return ret;
}

static int p9412_chip_renegotiate_pwr(struct p9221_charger_data *chgr)
{
	int ret;
	u8 val8;
	int try;
	int guar_pwr_mw; /* power provided by charger */
	int cur_pwr_mw;  /* power currently supplied */
	int tgt_pwr_mw;  /* power to be requested */
	int cfg_pwr_mw = chgr->pdata->epp_rp_value;

	ret = chgr->chip_get_sys_mode(chgr, &val8);
	if (ret)
		goto out;

	if (val8 != P9412_SYS_OP_MODE_WPC_EXTD)
		return 0;

	logbuffer_log(chgr->log, "%s: WPC renegotiation", __func__);

	/*
	 * Compare the current power to the available power o
	 * determine if renegotiation is worthwhile.
	 */
	ret = chgr->reg_read_8(chgr, P9221R5_EPP_TX_GUARANTEED_POWER_REG,
			       &val8);
	if (ret)
		goto out;
	guar_pwr_mw = P9412_HW_TO_MW(val8);

	ret = chgr->reg_read_8(chgr, P9221R5_EPP_CUR_NEGOTIATED_POWER_REG,
			       &val8);
	if (ret)
		goto out;
	cur_pwr_mw = P9412_HW_TO_MW(val8);

	tgt_pwr_mw = min(cfg_pwr_mw, guar_pwr_mw);
	logbuffer_log(chgr->log, "%s: tgt pwr = %d cur pwr = %d mW",
		      __func__, tgt_pwr_mw, cur_pwr_mw);

	if (cur_pwr_mw >= tgt_pwr_mw) {
		ret = -EAGAIN;
		logbuffer_log(chgr->log, "%s: no extra power available",
			      __func__);
		goto out;
	}

	/* Set the voltage to maximum value as defined in device-tree. */
	ret = chgr->chip_set_vout_max(chgr, chgr->pdata->max_vout_mv);

	val8 = P9412_MW_TO_HW(tgt_pwr_mw);
	ret = chgr->reg_write_8(chgr,
				P9221R5_EPP_REQ_NEGOTIATED_POWER_REG,
				val8);
	if (ret) {
		dev_err(&chgr->client->dev,
			"cannot write to EPP_NEG_POWER=%d (%d)\n", val8, ret);
		goto out;
	}

	val8 = P9412_MW_TO_HW(tgt_pwr_mw);
	ret = chgr->reg_write_8(chgr,
				P9221R5_EPP_REQ_MAXIMUM_POWER_REG,
				val8);
	if (ret < 0)
		dev_err(&chgr->client->dev,
			"cannot write to EPP_MAX_POWER=%d (%d)\n",
			 chgr->pdata->epp_rp_value, ret);

	ret = chgr->chip_set_cmd(chgr, P9221_COM_RENEGOTIATE);
	if (ret < 0)
		dev_err(&chgr->client->dev,
			"cannot write to sys_cmd =%d (%d)\n",
			 chgr->pdata->epp_rp_value, ret);

	/* Wait for renegotiation to complete. */
	ret = -ETIME;
	for (try = 0; try < P9412_RN_MAX_POLL_ATTEMPTS; try++) {
		msleep(P9412_RN_DELAY_MS);
		ret = chgr->reg_read_8(chgr,
				       P9221R5_EPP_RENEGOTIATION_REG,
				       &val8);
		if (val8 & P9412_RN_STATUS_DONE) {
			ret = 0;
			break;
		} else if (val8 & P9412_RN_STATUS_ERROR) {
			ret = -EIO;
			break;
		}
	}
	logbuffer_log(chgr->log, "%s: status = 0x%02x (tries = %d)",
		      __func__, val8, try);
out:
	return ret;
}

/* For high power mode */
static bool p9221_prop_mode_enable(struct p9221_charger_data *chgr, int req_pwr)
{
	return -ENOTSUPP;
}

static bool p9412_prop_mode_enable(struct p9221_charger_data *chgr, int req_pwr)
{
	int ret, loops;
	u8 val8, cdmode, txpwr, pwr_stp, mode_sts, err_sts, prop_cur_pwr, prop_req_pwr;

	ret = chgr->chip_get_sys_mode(chgr, &val8);
	if (ret) {
		dev_err(&chgr->client->dev,
			"PROP_MODE: cannot get sys mode\n");
		return 0;
	}

	if (val8 == P9412_SYS_OP_MODE_PROPRIETARY) {
		chgr->prop_mode_en = true;
		goto err_exit;
	}

	ret = p9xxx_chip_get_tx_mfg_code(chgr, &chgr->mfg);
	if (chgr->mfg != WLC_MFG_GOOGLE) {
		dev_err(&chgr->client->dev,
			"PROP_MODE: mfg code =%02x\n", chgr->mfg);
                return 0;
	}

	/*
	 * Step 1: clear all interrupts:
	 * write 0xFFFF to 0x3A then write 0x20 to 0x4E
	 */
	mutex_lock(&chgr->cmd_lock);

	ret = chgr->reg_write_16(chgr,
				 P9221R5_INT_CLEAR_REG, P9XXX_INT_CLEAR_MASK);
	if (ret == 0)
		ret = chgr->chip_set_cmd(chgr, P9221_COM_CLEAR_INT_MASK);
	if (ret) {
		dev_err(&chgr->client->dev, "Failed to reset INT: %d\n", ret);
		mutex_unlock(&chgr->cmd_lock);
		goto err_exit;
	}

	mutex_unlock(&chgr->cmd_lock);

	msleep(50);

	/*
	 * Step 2: Enable Proprietary Mode: write 0x01 to 0x4F (0x4E bit8)
	 */
	ret = p9412_chip_set_cmd_reg(chgr, PROP_MODE_EN_CMD);
	if (ret) {
		dev_err(&chgr->client->dev,
			"PROP_MODE: fail to send PROP_MODE_EN_CMD\n");
		goto err_exit;
	}

	msleep(50);

	/*
	 * Step 3: wait for PropModeStat interrupt, register 0x37[4]
	 */
	/* 60 * 50 = 3 secs */
	for (loops = 60 ; loops ; loops--) {
		if (chgr->prop_mode_en) {
			dev_info(&chgr->client->dev,
				 "PROP_MODE: Proprietary Mode Enabled\n");
			break;
		}
		msleep(50);
	}
	if (!chgr->prop_mode_en)
		goto err_exit;

	/*
	 * Step 4: enable Cap Divider configuration:
	 * write 0x02 to 0x101 then write 0x40 to 0x4E
	 */
	ret = chgr->reg_write_8(chgr, P9412_CDMODE_REQ_REG, CDMODE_CAP_DIV_MODE);
	if (ret) {
		dev_err(&chgr->client->dev,
                        "PROP_MODE: fail to enable Cap Div mode\n");
		goto err_exit;
	}
	ret = p9412_chip_set_cmd_reg(chgr, INIT_CAP_DIV_CMD);
	if (ret) {
		dev_err(&chgr->client->dev,
			"PROP_MODE: fail to send INIT_CAP_DIV_CMD\n");
		goto err_exit;
	}

	msleep(50);

	/* verify the change to Cap Divider mode */
	for (loops = 30 ; loops ; loops--) {
		ret = chgr->reg_read_8(chgr, P9412_CDMODE_STS_REG, &cdmode);
		if ((ret == 0) && (cdmode & CDMODE_CAP_DIV_MODE))
			break;
		if (ret < 0)
			goto err_exit;
		msleep(100);
	}
	if (!(cdmode & CDMODE_CAP_DIV_MODE))
		goto err_exit;

	/*
	 * Step 5: Read TX potential power register (0xC4)
	 * [TX max power capability] in 0.5W units
	 */
	ret = chgr->reg_read_8(chgr, P9412_PROP_TX_POTEN_PWR_REG, &txpwr);
	if (ret == 0)
		dev_info(&chgr->client->dev,
			 "PROP_MODE: Tx potential power=%02x\n", txpwr);
	else
		goto err_exit;

	/*
	 * Step 6: Request xx W Neg power by writing 0xC5,
	 * then write 0x02 to 0x4F
	 */
	ret = chgr->reg_write_8(chgr, P9412_PROP_REQ_PWR_REG, req_pwr * 2);
	if (ret) {
		dev_err(&chgr->client->dev,
			"PROP_MODE: fail to write pwr req register\n");
		goto err_exit;
	} else {
		dev_info(&chgr->client->dev, "request power=%dW\n", req_pwr);
	}
	/* Request power from TX based on PropReqPwr (0xC5) */
	ret = p9412_chip_set_cmd_reg(chgr, PROP_REQ_PWR_CMD);
	if (ret) {
		dev_err(&chgr->client->dev,
			"PROP_MODE: fail to send PROP_REQ_PWR_CMD\n");
		goto err_exit;
	}

	msleep(50);

err_exit:
        /* check status */
	ret = chgr->chip_get_sys_mode(chgr, &val8);
	ret |= chgr->reg_read_8(chgr, P9412_PROP_CURR_PWR_REG, &prop_cur_pwr);
	ret |= chgr->reg_read_8(chgr, P9412_PROP_MODE_PWR_STEP_REG, &pwr_stp);
	ret |= chgr->reg_read_8(chgr, P9412_PROP_MODE_STATUS_REG, &mode_sts);
	ret |= chgr->reg_read_8(chgr, P9412_PROP_MODE_ERR_STS_REG, &err_sts);
	ret |= chgr->reg_read_8(chgr, P9412_CDMODE_STS_REG, &cdmode);
	ret |= chgr->reg_read_8(chgr, P9412_PROP_REQ_PWR_REG, &prop_req_pwr);

	pr_debug("%s PROP_MODE: en=%d,sys_mode=%02x,mode_sts=%02x,err_sts=%02x,"
		 "cdmode=%02x,pwr_stp=%02x,req_pwr=%02x,prop_cur_pwr=%02x",
		 __func__, chgr->prop_mode_en, val8, mode_sts, err_sts,
		 cdmode, pwr_stp, prop_req_pwr, prop_cur_pwr);

	if (!ret) {
		dev_info(&chgr->client->dev,
			 "PROP_MODE: en=%d,sys_mode=%02x,mode_sts=%02x,"
			 "err_sts=%02x,cdmode=%02x,pwr_stp=%02x,"
			 "req_pwr=%02x,prop_cur_pwr=%02x",
			 chgr->prop_mode_en, val8, mode_sts, err_sts,
			 cdmode, pwr_stp, prop_req_pwr, prop_cur_pwr);
	}

	return chgr->prop_mode_en;
}

void p9221_chip_init_params(struct p9221_charger_data *chgr, u16 chip_id)
{
	switch (chip_id) {
	case P9412_CHIP_ID:
		chgr->reg_tx_id_addr = P9412_PROP_TX_ID_REG;
		chgr->reg_tx_mfg_code_addr = P9382_EPP_TX_MFG_CODE_REG;
		break;
	case P9382A_CHIP_ID:
		chgr->reg_tx_id_addr = P9382_PROP_TX_ID_REG;
		chgr->reg_tx_mfg_code_addr = P9382_EPP_TX_MFG_CODE_REG;
		break;
	case P9222_CHIP_ID:
		chgr->reg_tx_id_addr = P9221R5_PROP_TX_ID_REG;
		chgr->reg_tx_mfg_code_addr = P9222RE_TX_MFG_CODE_REG;
		break;
	default:
		chgr->reg_tx_id_addr = P9221R5_PROP_TX_ID_REG;
		chgr->reg_tx_mfg_code_addr = P9221R5_EPP_TX_MFG_CODE_REG;
		break;
	}
}

int p9221_chip_init_funcs(struct p9221_charger_data *chgr, u16 chip_id)
{
	chgr->chip_get_iout = p9xxx_chip_get_iout;
	chgr->chip_get_vout = p9xxx_chip_get_vout;
	chgr->chip_set_cmd = p9221_chip_set_cmd_reg;
	chgr->chip_get_op_freq = p9xxx_chip_get_op_freq;
	chgr->chip_get_vrect = p9xxx_chip_get_vrect;

	switch (chip_id) {
	case P9412_CHIP_ID:
		chgr->rtx_state = RTX_AVAILABLE;
		chgr->rx_buf_size = P9412_DATA_BUF_SIZE;
		chgr->tx_buf_size = P9412_DATA_BUF_SIZE;

		chgr->chip_get_rx_ilim = p9412_chip_get_rx_ilim;
		chgr->chip_set_rx_ilim = p9412_chip_set_rx_ilim;
		chgr->chip_get_tx_ilim = p9412_chip_get_tx_ilim;
		chgr->chip_set_tx_ilim = p9412_chip_set_tx_ilim;
		chgr->chip_get_die_temp = p9412_chip_get_die_temp;
		chgr->chip_get_vout_max = p9412_chip_get_vout_max;
		chgr->chip_set_vout_max = p9412_chip_set_vout_max;
		chgr->chip_tx_mode_en = p9412_chip_tx_mode;
		chgr->chip_get_data_buf = p9412_get_data_buf;
		chgr->chip_set_data_buf = p9412_set_data_buf;
		chgr->chip_get_cc_recv_size = p9412_get_cc_recv_size;
		chgr->chip_set_cc_send_size = p9412_set_cc_send_size;
		chgr->chip_get_align_x = p9412_get_align_x;
		chgr->chip_get_align_y = p9412_get_align_y;
		chgr->chip_send_ccreset = p9412_send_ccreset;
		chgr->chip_send_eop = p9412_send_eop;
		chgr->chip_get_sys_mode = p9412_chip_get_sys_mode;
		chgr->chip_renegotiate_pwr = p9412_chip_renegotiate_pwr;
		chgr->chip_prop_mode_en = p9412_prop_mode_enable;
		break;
	case P9382A_CHIP_ID:
		chgr->rtx_state = RTX_AVAILABLE;
		chgr->rx_buf_size = P9221R5_DATA_RECV_BUF_SIZE;
		chgr->tx_buf_size = P9221R5_DATA_SEND_BUF_SIZE;

		chgr->chip_get_rx_ilim = p9221_chip_get_rx_ilim;
		chgr->chip_set_rx_ilim = p9221_chip_set_rx_ilim;
		chgr->chip_get_tx_ilim = p9382_chip_get_tx_ilim;
		chgr->chip_set_tx_ilim = p9382_chip_set_tx_ilim;
		chgr->chip_get_die_temp = p9221_chip_get_die_temp;
		chgr->chip_get_vout_max = p9832_chip_get_vout_max;
		chgr->chip_set_vout_max = p9832_chip_set_vout_max;
		chgr->chip_tx_mode_en = p9382_chip_tx_mode;
		chgr->chip_get_data_buf = p9382_get_data_buf;
		chgr->chip_set_data_buf = p9382_set_data_buf;
		chgr->chip_get_cc_recv_size = p9221_get_cc_recv_size;
		chgr->chip_set_cc_send_size = p9382_set_cc_send_size;
		chgr->chip_get_align_x = p9221_get_align_x;
		chgr->chip_get_align_y = p9221_get_align_y;
		chgr->chip_send_ccreset = p9221_send_ccreset;
		chgr->chip_send_eop = p9221_send_eop;
		chgr->chip_get_sys_mode = p9221_chip_get_sys_mode;
		chgr->chip_renegotiate_pwr = p9221_chip_renegotiate_pwr;
		chgr->chip_prop_mode_en = p9221_prop_mode_enable;
		break;
	case P9222_CHIP_ID:
		chgr->chip_get_iout = p9222_chip_get_iout;
		chgr->chip_get_vout = p9222_chip_get_vout;
		chgr->chip_get_op_freq = p9222_chip_get_op_freq;
		chgr->chip_get_vrect = p9222_chip_get_vrect;
		chgr->chip_set_cmd = p9222_chip_set_cmd_reg;

		chgr->rtx_state = RTX_NOTSUPPORTED;
		chgr->rx_buf_size = P9221R5_DATA_RECV_BUF_SIZE;
		chgr->tx_buf_size = P9221R5_DATA_SEND_BUF_SIZE;

		chgr->chip_get_rx_ilim = p9222_chip_get_rx_ilim;
		chgr->chip_set_rx_ilim = p9222_chip_set_rx_ilim;
		chgr->chip_get_tx_ilim = p9221_chip_get_tx_ilim;
		chgr->chip_set_tx_ilim = p9221_chip_set_tx_ilim;
		chgr->chip_get_die_temp = p9222_chip_get_die_temp;
		chgr->chip_get_vout_max = p9221_chip_get_vout_max;
		chgr->chip_set_vout_max = p9221_chip_set_vout_max;
		chgr->chip_tx_mode_en = p9221_chip_tx_mode;
		chgr->chip_set_data_buf = p9221_set_data_buf;
		chgr->chip_get_data_buf = p9221_get_data_buf;
		chgr->chip_get_cc_recv_size = p9221_get_cc_recv_size;
		chgr->chip_set_cc_send_size = p9221_set_cc_send_size;
		chgr->chip_get_align_x = p9221_get_align_x;
		chgr->chip_get_align_y = p9221_get_align_y;
		chgr->chip_send_ccreset = p9221_send_ccreset;
		chgr->chip_send_eop = p9221_send_eop;
		chgr->chip_get_sys_mode = p9222_chip_get_sys_mode;
		chgr->chip_renegotiate_pwr = p9222_chip_renegotiate_pwr;
		chgr->chip_prop_mode_en = p9221_prop_mode_enable;
		break;
	default:
		chgr->rtx_state = RTX_NOTSUPPORTED;
		chgr->rx_buf_size = P9221R5_DATA_RECV_BUF_SIZE;
		chgr->tx_buf_size = P9221R5_DATA_SEND_BUF_SIZE;

		chgr->chip_get_rx_ilim = p9221_chip_get_rx_ilim;
		chgr->chip_set_rx_ilim = p9221_chip_set_rx_ilim;
		chgr->chip_get_tx_ilim = p9221_chip_get_tx_ilim;
		chgr->chip_set_tx_ilim = p9221_chip_set_tx_ilim;
		chgr->chip_get_die_temp = p9221_chip_get_die_temp;
		chgr->chip_get_vout_max = p9221_chip_get_vout_max;
		chgr->chip_set_vout_max = p9221_chip_set_vout_max;
		chgr->chip_tx_mode_en = p9221_chip_tx_mode;
		chgr->chip_set_data_buf = p9221_set_data_buf;
		chgr->chip_get_data_buf = p9221_get_data_buf;
		chgr->chip_get_cc_recv_size = p9221_get_cc_recv_size;
		chgr->chip_set_cc_send_size = p9221_set_cc_send_size;
		chgr->chip_get_align_x = p9221_get_align_x;
		chgr->chip_get_align_y = p9221_get_align_y;
		chgr->chip_send_ccreset = p9221_send_ccreset;
		chgr->chip_send_eop = p9221_send_eop;
		chgr->chip_get_sys_mode = p9221_chip_get_sys_mode;
		chgr->chip_renegotiate_pwr = p9221_chip_renegotiate_pwr;
		chgr->chip_prop_mode_en = p9221_prop_mode_enable;
		break;
	}

	chgr->rx_buf = devm_kzalloc(chgr->dev, chgr->rx_buf_size, GFP_KERNEL);
	if (chgr->rx_buf == NULL)
		return -ENOMEM;

	chgr->tx_buf = devm_kzalloc(chgr->dev, chgr->tx_buf_size, GFP_KERNEL);
	if (chgr->tx_buf == NULL)
		return -ENOMEM;

	return 0;
}
