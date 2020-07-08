/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Fuel gauge driver for Maxim Fuel Gauges with M5 Algo
 *
 * Copyright (C) 2018 Google Inc.
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include "google_bms.h"
#include "google_psy.h"
#include "logbuffer.h"

#include "max_m5_reg.h"
#include "max_m5.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

static void dump_model(struct device *dev, u16 *data, int count)
{
	int i, j, len;
	char buff[16 * 5 + 1] = {};

	for (i = 0; i < count; i += 16) {

		for (len = 0, j = 0; j < 16; j++)
			len += scnprintf(&buff[len], sizeof(buff) - len,
					 "%04x ", data[i + j]);

		dev_info(dev, "%x: %s\n", i + MAX_M5_FG_MODEL_START, buff);
	}
}

/* input current is in the fuel gauge */
int max_m5_read_actual_input_current_ua(struct i2c_client *client, int *iic)
{
	struct max_m5_data *m5_data = max1720x_get_model_data(client);
	unsigned int tmp;
	int rtn;

	if (!m5_data || !m5_data->regmap)
		return -ENODEV;

	rtn = regmap_read(m5_data->regmap->regmap, MAX_M5_IIN, &tmp);
	if (rtn)
		pr_err("Failed to read %x\n", MAX_M5_IIN);
	else
		*iic  = tmp * 125;

	return rtn;
}
EXPORT_SYMBOL_GPL(max_m5_read_actual_input_current_ua);

int max_m5_reg_read(struct i2c_client *client, unsigned int reg,
		    unsigned int *val)
{
	struct max_m5_data *m5_data = max1720x_get_model_data(client);

	if (!m5_data || !m5_data->regmap)
		return -ENODEV;

	return regmap_read(m5_data->regmap->regmap, reg, val);
}
EXPORT_SYMBOL_GPL(max_m5_reg_read);

int max_m5_reg_write(struct i2c_client *client, unsigned int reg,
		    unsigned int val)
{
	struct max_m5_data *m5_data = max1720x_get_model_data(client);

	if (!m5_data || !m5_data->regmap)
		return -ENODEV;

	return regmap_write(m5_data->regmap->regmap, reg, val);
}
EXPORT_SYMBOL_GPL(max_m5_reg_write);


static int max_m5_read_custom_model(struct regmap *regmap, u16 *model_data,
				    int count)
{
	return regmap_raw_read(regmap, MAX_M5_FG_MODEL_START, model_data,
			       count * 2);
}

static int max_m5_write_custom_model(struct regmap *regmap, u16 *model_data,
				     int count)
{
	return regmap_raw_write(regmap, MAX_M5_FG_MODEL_START, model_data,
				count * 2);
}

static int max_m5_model_lock(struct regmap *regmap, bool enabled)
{
	u16 code[2] = {0x59, 0xC4};

	if (enabled) {
		code[0] = 0;
		code[1] = 0;
	}

	return regmap_raw_write(regmap, MAX_M5_UNLOCK_MODEL_ACCESS, code,
				sizeof(code));
}

static int mem16test(u16 *data, u16 code, int count)
{
	int same, i;

	for (i = 0, same = 1; same && i < count; i++)
		same = data[i] == code;

	return same;
}

/* load custom model b/137037210 */
static int max_m5_update_custom_model(struct max_m5_data *m5_data)
{
	int retries, ret;
	bool success;
	u16 *data;

	data = kzalloc(m5_data->custom_model_size * 2, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* un lock, update the model */
	for (success = false, retries = 3; !success && retries > 0; retries--) {

		ret = max_m5_model_lock(m5_data->regmap->regmap, false);
		if (ret < 0) {
			dev_err(m5_data->dev, "cannot unlock model access (%d)\n",
				ret);
			continue;
		}

		ret = max_m5_write_custom_model(m5_data->regmap->regmap,
						m5_data->custom_model,
						m5_data->custom_model_size);
		if (ret < 0) {
			dev_err(m5_data->dev, "cannot write custom model (%d)\n",
				ret);
			continue;
		}

		ret = max_m5_read_custom_model(m5_data->regmap->regmap,
					       data,
					       m5_data->custom_model_size);
		if (ret < 0) {
			dev_err(m5_data->dev, "cannot write custom model (%d)\n",
				ret);
			continue;
		}

		ret = memcmp(m5_data->custom_model, data,
			     m5_data->custom_model_size * 2);
		success = ret == 0;
		if (!success) {
			dump_model(m5_data->dev, m5_data->custom_model,
				   m5_data->custom_model_size);
			dump_model(m5_data->dev, data,
				   m5_data->custom_model_size);
		}

	}

	if (!success) {
		dev_err(m5_data->dev, "cannot write custom model (%d)\n", ret);
		kfree(data);
		return -EIO;
	}

	/* lock and verify lock */
	for (retries = 3; retries > 0; retries--) {
		int same;

		ret = max_m5_model_lock(m5_data->regmap->regmap, true);
		if (ret < 0) {
			dev_err(m5_data->dev, "cannot lock model access (%d)\n",
				ret);
			continue;
		}

		ret = max_m5_read_custom_model(m5_data->regmap->regmap, data,
					       m5_data->custom_model_size);
		if (ret < 0) {
			dev_err(m5_data->dev, "cannot read custom model (%d)\n",
				ret);
			continue;
		}

		/* model is locked when read retuns all 0xffff */
		same = mem16test(data, 0xffff, m5_data->custom_model_size);
		if (same)
			break;
	}

	kfree(data);
	return 0;
}

/* Step 7: Write custom parameters */
static int max_m5_update_custom_parameters(struct max_m5_data *m5_data)
{
	struct max_m5_custom_parameters *cp = &m5_data->parameters;
	struct max17x0x_regmap *regmap = m5_data->regmap;
	int tmp, ret;
	u16 vfsoc;

	ret = REGMAP_WRITE(regmap, MAX_M5_TEMPNOM, 0x1403);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_COFF, 0x1);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_REPCAP, 0x0);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_IAVGEMPTY,
					  cp->iavg_empty);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_RELAXCFG,
					  cp->relaxcfg);
	if (ret < 0)
		return -EIO;

	ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_UNLOCK_EXTRA_CONFIG,
				  MAX_M5_UNLOCK_EXTRA_CONFIG_UNLOCK_CODE);
	if (ret < 0) {
		dev_err(m5_data->dev, "cannot unlock extra config (%d)\n", ret);
		return -EIO;
	}

	ret = REGMAP_READ(regmap, MAX_M5_VFSOC, &vfsoc);
	if (ret < 0)
		return ret;

	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_VFSOC0, vfsoc);

	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_LEARNCFG, cp->learncfg);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_CONFIG, cp->config);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_CONFIG2, cp->config2);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_FULLSOCTHR, cp->fullsocthr);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_FULLCAPREP,
					  cp->fullcaprep);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_DESIGNCAP,
					  cp->designcap);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_DPACC, cp->dpacc);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_DQACC, cp->dqacc);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_FULLCAPNOM,
					  cp->fullcapnom);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_VEMPTY, cp->v_empty);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_QRTABLE00,
					  cp->qresidual00);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_QRTABLE10,
					  cp->qresidual10);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_QRTABLE20,
					  cp->qresidual20);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_QRTABLE30,
					  cp->qresidual30);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_RCOMP0,
					  cp->rcomp0);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_RCOMP0, cp->tempco);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_ICHGTERM, cp->ichgterm);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_TGAIN, cp->tgain);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_TOFF, cp->toff);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_MISCCFG, cp->misccfg);
	if (ret < 0)
		goto exit_done;

	ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_UNLOCK_EXTRA_CONFIG,
				  MAX_M5_UNLOCK_EXTRA_CONFIG_UNLOCK_CODE);
	if (ret < 0) {
		dev_err(m5_data->dev, "cannot unlock extra config (%d)\n", ret);
		goto exit_done;
	}

	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_ATRATE, cp->atrate);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_CV_MIXCAP,
					  (cp->fullcapnom * 75) / 100);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_CV_HALFTIME, 0x600);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_CONVGCFG, cp->convgcfg);

exit_done:
	tmp = REGMAP_WRITE_VERIFY(regmap, MAX_M5_UNLOCK_EXTRA_CONFIG,
				  MAX_M5_UNLOCK_EXTRA_CONFIG_LOCK_CODE);
	if (tmp < 0) {
		dev_err(m5_data->dev, "cannot lock extra config (%d)\n", tmp);
		return tmp;
	}

	return ret;
}

int max_m5_load_gauge_model(struct max_m5_data *m5_data)
{
	struct max17x0x_regmap *regmap = m5_data->regmap;
	int ret, retries;
	u16 data;

	if (!regmap)
		return -EIO;

	if (!m5_data || !m5_data->custom_model || !m5_data->custom_model_size)
		return -ENODATA;

	ret = max_m5_update_custom_model(m5_data);
	if (ret < 0) {
		dev_err(m5_data->dev, "cannot update custom model (%d)\n", ret);
		return ret;
	}

	ret = max_m5_update_custom_parameters(m5_data);
	if (ret < 0) {
		dev_err(m5_data->dev, "cannot update custom parameters (%d)\n",
			ret);
		return ret;
	}

	/* tcurve, filterconfig are not part of model state */
	ret = REGMAP_WRITE(regmap, MAX_M5_TCURVE, m5_data->parameters.tcurve);
	if (ret < 0) {
		dev_err(m5_data->dev, "cannot update tcurve (%d)\n",
			ret);
		return ret;
	}

	ret = REGMAP_WRITE(regmap, MAX_M5_FILTERCFG,
			   m5_data->parameters.filtercfg);
	if (ret < 0) {
		dev_err(m5_data->dev, "cannot update filter config (%d)\n",
			ret);
		return ret;
	}

	/* load model now */
	ret = REGMAP_READ(regmap, MAX_M5_CONFIG2, &data);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_CONFIG2, data | 0x20);
	if (ret < 0) {
		dev_err(m5_data->dev, "failed start model loading (%d)\n", ret);
		return ret;
	}

	/* around 400ms for this */
	for (retries = 10; retries > 0; retries--) {

		mdelay(50);

		ret = REGMAP_READ(regmap, MAX_M5_CONFIG2, &data);
		if (ret == 0 && !(data & 0x20))
			return 0;
	}

	return -ETIMEDOUT;
}

/* load parameters and model state from permanent storage */
int max_m5_load_state_data(struct max_m5_data *m5_data)
{
	int ret = 0;

	if (!m5_data)
		return -EINVAL;

	/* might return -EAGAIN during init */
	ret = gbms_storage_read(GBMS_TAG_GMSR, &m5_data->model_save,
				sizeof(m5_data->model_save));
	if (ret < 0) {
		dev_info(m5_data->dev, "Load Model Data Failed ret=%d\n", ret);
		return ret;
	}

	if (m5_data->model_save.rcomp0 == 0xFF) {
		dev_info(m5_data->dev, "Model Data Empty\n");
		return -EINVAL;
	}

	m5_data->parameters.rcomp0 = m5_data->model_save.rcomp0;
	m5_data->parameters.tempco = m5_data->model_save.tempco;
	m5_data->parameters.fullcaprep = m5_data->model_save.fullcaprep;
	m5_data->cycles = m5_data->model_save.cycles;
	m5_data->parameters.fullcapnom = m5_data->model_save.fullcapnom;
	m5_data->parameters.qresidual00 = m5_data->model_save.qresidual00;
	m5_data->parameters.qresidual10 = m5_data->model_save.qresidual10;
	m5_data->parameters.qresidual20 = m5_data->model_save.qresidual20;
	m5_data->parameters.qresidual30 = m5_data->model_save.qresidual30;
	m5_data->mixcap = m5_data->model_save.mixcap;
	m5_data->halftime = m5_data->model_save.halftime;

	return ret;
}

/* save/commit parameters and model state to permanent storage */
int max_m5_save_state_data(struct max_m5_data *m5_data)
{
	int ret = 0;

	m5_data->model_save.rcomp0 = m5_data->parameters.rcomp0;
	m5_data->model_save.tempco = m5_data->parameters.tempco;
	m5_data->model_save.fullcaprep = m5_data->parameters.fullcaprep;
	m5_data->model_save.cycles = m5_data->cycles;
	m5_data->model_save.fullcapnom = m5_data->parameters.fullcapnom;
	m5_data->model_save.qresidual00 = m5_data->parameters.qresidual00;
	m5_data->model_save.qresidual10 = m5_data->parameters.qresidual10;
	m5_data->model_save.qresidual20 = m5_data->parameters.qresidual20;
	m5_data->model_save.qresidual30 = m5_data->parameters.qresidual30;
	m5_data->model_save.mixcap = m5_data->mixcap;
	m5_data->model_save.halftime = m5_data->halftime;

	ret = gbms_storage_write(GBMS_TAG_GMSR,
				 (const void *)&m5_data->model_save,
				 sizeof(m5_data->model_save));
	if (ret < 0)
		return ret;

	return ret == sizeof(m5_data->model_save) ? 0 : -ERANGE;
}

/* read fuel gauge state to parameters/model state */
int max_m5_model_read_state(struct max_m5_data *m5_data)
{
	int rc;
	struct max17x0x_regmap *regmap = m5_data->regmap;

	rc= REGMAP_READ(regmap, MAX_M5_RCOMP0, &m5_data->parameters.rcomp0);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_TEMPCO,
				 &m5_data->parameters.tempco);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_FULLCAPREP,
				 &m5_data->parameters.fullcaprep);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_CYCLES, &m5_data->cycles);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_FULLCAPNOM,
				 &m5_data->parameters.fullcapnom);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_QRTABLE00,
				 &m5_data->parameters.qresidual00);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_QRTABLE10,
				 &m5_data->parameters.qresidual10);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_QRTABLE20,
				 &m5_data->parameters.qresidual20);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_QRTABLE30,
				 &m5_data->parameters.qresidual30);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_MIXCAP, &m5_data->mixcap);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_CV_HALFTIME,
				 &m5_data->halftime);

	return rc;
}

ssize_t max_m5_model_state_cstr(char *buf, int max,
				struct max_m5_data *m5_data)
{
	int len = 0;

	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_RCOMP0,
			 m5_data->parameters.rcomp0);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_TEMPCO,
			 m5_data->parameters.tempco);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_FULLCAPREP,
			 m5_data->parameters.fullcaprep);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_CYCLES,
			 m5_data->cycles);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_FULLCAPNOM,
			 m5_data->parameters.fullcapnom);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_QRTABLE00,
			 m5_data->parameters.qresidual00);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_QRTABLE10,
			 m5_data->parameters.qresidual10);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_QRTABLE20,
			 m5_data->parameters.qresidual20);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_QRTABLE30,
			 m5_data->parameters.qresidual30);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_MIXCAP,
			 m5_data->mixcap);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_CV_HALFTIME,
			 m5_data->halftime);

	return len;
}

/* can be use to restore parametes and model state after POR */
int max_m5_model_state_sscan(struct max_m5_data *m5_data, const char *buf,
			     int max)
{
	int ret, index, reg, val;

	for (index = 0; index < max ; index += 1) {
		ret = sscanf(&buf[index], "%x:%x", &reg, &val);
		if (ret != 2) {
			dev_err(m5_data->dev, "@%d: sscan error %d\n",
				index, ret);
			return -EINVAL;
		}

		dev_info(m5_data->dev, "@%d: reg=%x val=%x\n", index, reg, val);

		switch (reg) {
		/* fg-params */
		case MAX_M5_IAVGEMPTY:
			m5_data->parameters.iavg_empty = val;
			break;
		case MAX_M5_RELAXCFG:
			m5_data->parameters.relaxcfg = val;
			break;
		case MAX_M5_LEARNCFG:
			m5_data->parameters.learncfg = val;
			break;
		case MAX_M5_CONFIG:
			m5_data->parameters.config = val;
			break;
		case MAX_M5_CONFIG2:
			m5_data->parameters.config2 = val;
			break;
		case MAX_M5_FULLSOCTHR:
			m5_data->parameters.fullsocthr = val;
			break;
		case MAX_M5_DESIGNCAP:
			m5_data->parameters.designcap = val;
			break;
		case MAX_M5_DPACC:
			m5_data->parameters.dpacc = val;
			break;
		case MAX_M5_DQACC:
			m5_data->parameters.dqacc = val;
			break;
		case MAX_M5_VEMPTY:
			m5_data->parameters.v_empty = val;
			break;
		case MAX_M5_TGAIN:
			m5_data->parameters.tgain = val;
			break;
		case MAX_M5_TOFF:
			m5_data->parameters.toff = val;
			break;
		case MAX_M5_TCURVE:
			m5_data->parameters.tcurve = val;
			break;
		case MAX_M5_MISCCFG:
			m5_data->parameters.misccfg = val;
			break;
		case MAX_M5_ATRATE:
			m5_data->parameters.atrate = val;
			break;
		case MAX_M5_CONVGCFG:
			m5_data->parameters.convgcfg = val;
			break;
		case MAX_M5_FILTERCFG:
			m5_data->parameters.filtercfg = val;
			break;

		/* model state */
		case MAX_M5_RCOMP0:
			m5_data->parameters.rcomp0 = val;
			break;
		case MAX_M5_TEMPCO:
			m5_data->parameters.tempco = val;
			break;
		case MAX_M5_FULLCAPREP:
			m5_data->parameters.fullcaprep = val;
			break;
		case MAX_M5_CYCLES:
			m5_data->cycles = val;
			break;
		case MAX_M5_FULLCAPNOM:
			m5_data->parameters.fullcapnom = val;
			break;
		case MAX_M5_QRTABLE00:
			m5_data->parameters.qresidual00 = val;
			break;
		case MAX_M5_QRTABLE10:
			m5_data->parameters.qresidual10 = val;
			break;
		case MAX_M5_QRTABLE20:
			m5_data->parameters.qresidual20 = val;
			break;
		case MAX_M5_QRTABLE30:
			m5_data->parameters.qresidual30 = val;
			break;
		case MAX_M5_MIXCAP:
			m5_data->mixcap = val;
			break;
		case MAX_M5_CV_HALFTIME:
			m5_data->halftime = val;
			break;
		default:
			dev_err(m5_data->dev, "@%d: reg=%x out of range\n",
				index, reg);
			return -EINVAL;
		}

		for ( ; index < max && buf[index] != '\n'; index++)
			;
	}

	return 0;
}

/* custom model parameters */
int max_m5_fg_model_cstr(char *buf, int max, const struct max_m5_data *m5_data)
{
	int i, len;

	if (!m5_data->custom_model || !m5_data->custom_model_size)
		return -EINVAL;

	for (len = 0, i = 0; i < m5_data->custom_model_size; i += 1)
		len += scnprintf(&buf[len], max - len, "%x: %04x\n",
				 MAX_M5_FG_MODEL_START + i,
				 m5_data->custom_model[i]);

	return len;
}

/* custom model parameters */
int max_m5_fg_model_sscan(struct max_m5_data *m5_data, const char *buf, int max)
{
	int ret, index, reg, val, fg_model_end;

	if (!m5_data->custom_model)
		return -EINVAL;

	/* use the default size */
	if (!m5_data->custom_model_size)
		m5_data->custom_model_size = MAX_M5_FG_MODEL_SIZE;

	fg_model_end = MAX_M5_FG_MODEL_START + m5_data->custom_model_size;
	for (index = 0; index < max ; index += 1) {
		ret = sscanf(&buf[index], "%x:%x", &reg, &val);
		if (ret != 2) {
			dev_err(m5_data->dev, "@%d: sscan error %d\n",
				index, ret);
			return -EINVAL;
		}

		dev_info(m5_data->dev, "@%d: reg=%x val=%x\n", index, reg, val);

		if (reg >= MAX_M5_FG_MODEL_START && reg < fg_model_end) {
			const int offset = reg - MAX_M5_FG_MODEL_START;

			m5_data->custom_model[offset] = val;
		}

		for ( ; index < max && buf[index] != '\n'; index++)
			;
	}

	return 0;
}

/* Initial values??? */
static int m5_init_custom_parameters(struct device *dev,
				     struct max_m5_custom_parameters *cp,
				     struct device_node *node)
{
	const char *propname = "maxim,fg-params";
	int ret, cnt;

	memset(cp, 0, sizeof(*cp));

	cnt = of_property_count_elems_of_size(node, propname, sizeof(u16));
	if (cnt < 0)
		return -ENODATA;

	if (cnt != sizeof(*cp) / 2) {
		dev_err(dev, "fg-params: %s has %d elements, need %d\n",
			propname, cnt, sizeof(*cp) / 2);
		return -ERANGE;
	}

	ret = of_property_read_u16_array(node, propname, (u16 *)cp, cnt);
	if (ret < 0) {
		dev_err(dev, "fg-params: failed to read %s %s: %d\n",
			node->name, propname, ret);
		return -EINVAL;
	}

	return 0;
}

void max_m5_free_data(void *data)
{

}

void *max_m5_init_data(struct device *dev, struct device_node *node,
		       struct max17x0x_regmap *regmap)
{
	const char *propname = "maxim,fg-model";
	struct max_m5_data *m5_data;
	int cnt, ret;
	u16 *model;

	m5_data = devm_kzalloc(dev, sizeof(*m5_data), GFP_KERNEL);
	if (!m5_data) {
		dev_err(dev, "fg-model: %s not found\n", propname);
		return ERR_PTR(-ENOMEM);
	}

	model = devm_kmalloc_array(dev, MAX_M5_FG_MODEL_SIZE, sizeof(u16),
				   GFP_KERNEL);
	if (!model) {
		dev_err(dev, "fg-model: out of memory\n");
		return ERR_PTR(-ENOMEM);
	}

	cnt = of_property_count_elems_of_size(node, propname, sizeof(u16));
	if (cnt != MAX_M5_FG_MODEL_SIZE) {
		dev_err(dev, "fg-model: not found, or invalid %d\n", cnt);
	} else {
		ret = of_property_read_u16_array(node, propname, model, cnt);
		if (ret < 0)
			dev_err(dev, "fg-model: no data cnt=%d %s %s: %d\n",
				cnt, node->name, propname, ret);
		else
			m5_data->custom_model_size = cnt;
	}

	/*
	 * Initial values: check max_m5_model_read_state() for the registers
	 * updated from max1720x_model_work()
	 */
	ret = m5_init_custom_parameters(dev, &m5_data->parameters, node);
	if (ret < 0)
		dev_err(dev, "fg-params: %s not found\n", propname);

	m5_data->custom_model = model;
	m5_data->regmap = regmap;
	m5_data->dev = dev;

	return m5_data;
}

static bool max_m5_is_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x00 ... 0x4F:
	case 0xB0 ... 0xBF:
	case 0xD0:		/* IIC */
	case 0xDC ... 0xDF:
	case 0xFB:
	case 0xFF:		/* VFSOC */
		return true;
	case 0x60:		/* Model unlock */
	case 0x62:		/* Unlock Model Access */
	case 0x63:		/* Unlock Model Access */
	case 0x80 ... 0xAF:	/* FG Model */
		/* TODO: add a check on unlock */
		return true;
	}

	return false;
}

const struct regmap_config max_m5_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX_M5_VFSOC,
	.readable_reg = max_m5_is_reg,
	.volatile_reg = max_m5_is_reg,
};
const struct max17x0x_reg max_m5[] = {
	[MAX17X0X_TAG_avgc] = { ATOM_INIT_REG16(MAX_M5_AVGCURRENT)},
	[MAX17X0X_TAG_cnfg] = { ATOM_INIT_REG16(MAX_M5_CONFIG)},
	[MAX17X0X_TAG_mmdv] = { ATOM_INIT_REG16(MAX_M5_MAXMINVOLT)},
	[MAX17X0X_TAG_vcel] = { ATOM_INIT_REG16(MAX_M5_VCELL)},
	[MAX17X0X_TAG_temp] = { ATOM_INIT_REG16(MAX_M5_TEMP)},
	[MAX17X0X_TAG_curr] = { ATOM_INIT_REG16(MAX_M5_CURRENT)},
	[MAX17X0X_TAG_mcap] = { ATOM_INIT_REG16(MAX_M5_MIXCAP)},
	[MAX17X0X_TAG_vfsoc] = { ATOM_INIT_REG16(MAX_M5_VFSOC)},
};

int max_m5_regmap_init(struct max17x0x_regmap *regmap, struct i2c_client *clnt)
{
	struct regmap *map;

	map = devm_regmap_init_i2c(clnt, &max_m5_regmap_cfg);
	if (IS_ERR(map))
		return IS_ERR_VALUE(map);

	regmap->regtags.max = ARRAY_SIZE(max_m5);
	regmap->regtags.map = max_m5;
	regmap->regmap = map;
	return 0;
}

int batt_hist_data_collect(void *h, int idx, int cycle_cnt,
			   struct power_supply *fg_psy)
{
	return -ENODEV;
}
void *batt_hist_init_data(struct device *dev)
{
	return NULL;
}

void batt_hist_free_data(void *p)
{

}
