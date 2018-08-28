/*
 * Google LWIS Sensor Interface
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-sensor: " fmt

#include <linux/kernel.h>
#include <linux/slab.h>

#include "lwis_sensor.h"
#include "lwis_dt.h"

/*
 *  Sensor Interface Functions
 */

int lwis_sensor_init(struct lwis_sensor *psensor)
{
	if (!psensor || !psensor->ops || !psensor->ops->init) {
		return -ENODEV;
	}

	return psensor->ops->init();
}

/*
 *  Sensor Helper Functions
 */

struct lwis_sensor *lwis_sensor_get_ptr(struct i2c_client *pclient)
{
	struct lwis_sensor *psensor = NULL;

#ifdef CONFIG_OF
	struct device_node *pdnode;

	pdnode = pclient->dev.of_node;
	if (!pdnode) {
		pr_err("Unable to obtain sensor device node\n");
		goto error_exit;
	}

	psensor = (struct lwis_sensor *) pdnode->data;
	if (!psensor) {
		pr_err("Unable to obtain sensor data struct\n");
		goto error_exit;
	}

#else
	// Implementation reserved for non-device-tree platforms
#endif

error_exit:
	return psensor;
}

int lwis_sensor_parse_config(struct device *pdev, struct lwis_sensor *psensor)
{
#ifdef CONFIG_OF
	return lwis_sensor_parse_config_dt(pdev, psensor);
#else
	// Not implemented for non-device-tree parsing yet
	return -ENOSYS;
#endif
}

int lwis_sensor_initialize_i2c(struct i2c_client *pclient,
			       struct lwis_sensor *psensor)
{
	struct lwis_i2c *pi2c;

	pi2c = kzalloc(sizeof(struct lwis_i2c), GFP_KERNEL);
	if (!pi2c) {
		pr_err("Cannot allocate i2c structure\n");
		return -ENOMEM;
	}

	psensor->i2c = pi2c;
	pi2c->pclient = pclient;

	return lwis_i2c_initialize(pi2c);
}

int lwis_sensor_initialize_mclk_ctrl(struct lwis_sensor *psensor)
{
	struct device *pdev;
	struct pinctrl *clkctrl;

	if (!psensor || !psensor->i2c) {
		pr_err("Cannot find sensor/i2c instance\n");
		return -ENODEV;
	}

	pdev = &psensor->i2c->pclient->dev;

	clkctrl = devm_pinctrl_get(pdev);
	if (IS_ERR(clkctrl)) {
		pr_err("Cannot allocate mclk pinctrl\n");
		return PTR_ERR(clkctrl);
	}

	psensor->mclk_ctrl = clkctrl;

	return 0;
}

int lwis_sensor_set_mclk_state(struct lwis_sensor *psensor, const char *state)
{
	int ret;
	struct pinctrl_state *pstate;

	if (!psensor || !psensor->mclk_ctrl) {
		pr_err("Cannot find sensor/pinctrl instance\n");
		return -ENODEV;
	}

	pstate = pinctrl_lookup_state(psensor->mclk_ctrl, state);
	if (IS_ERR(pstate)) {
		pr_err("Cannot find mclk state %s\n", state);
		return PTR_ERR(pstate);
	}

	ret = pinctrl_select_state(psensor->mclk_ctrl, pstate);
	if (ret) {
		pr_err("Cannot select state %s\n", state);
		return ret;
	}

	return 0;
}
