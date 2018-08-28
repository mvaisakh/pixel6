/*
 * Google LWIS I2C Interface
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-i2c: " fmt

#include "lwis_i2c.h"

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>

int lwis_i2c_initialize(struct lwis_i2c *pi2c)
{
	struct device *pdev;
	struct pinctrl *ppc;

	if (!pi2c || !pi2c->pclient) {
		pr_err("Cannot find i2c instance\n");
		return -ENODEV;
	}

	pdev = &pi2c->pclient->dev;

	/* Parent of the client is the i2c block, which is where the pinctrl's
	   are defined */
	/* TODO: Need to figure out why this is parent's parent */
	ppc = devm_pinctrl_get(pdev->parent->parent);
	if (IS_ERR(ppc)) {
		pr_err("Cannot instantiate pinctrl instance (%d)\n",
			(int) PTR_ERR(ppc));
		return PTR_ERR(ppc);
	}
	pi2c->ppc = ppc;

	return 0;
}

int lwis_i2c_set_state(struct lwis_i2c *pi2c, const char *state_str)
{
	int ret;
	struct pinctrl_state *pstate;

	if (!pi2c || !pi2c->ppc) {
		pr_err("Cannot find i2c instance\n");
		return -ENODEV;
	}

	pstate = pinctrl_lookup_state(pi2c->ppc, state_str);
	if (IS_ERR(pstate)) {
		pr_err("State %s not found (%ld)\n", state_str,
			PTR_ERR(pstate));
		return PTR_ERR(pstate);
	}

	ret = pinctrl_select_state(pi2c->ppc, pstate);
	if (ret) {
		pr_err("Error selecting state %s (%d)\n", state_str, ret);
		return ret;
	}

	return 0;
}

int lwis_i2c_read(struct lwis_i2c *pi2c, u16 addr, u8 *rbuf, u32 bufsize)
{
	int ret;
	struct i2c_client *pclient = pi2c->pclient;
	struct i2c_msg msg[2];
	u8 buf[2];

	msg[0].addr = pclient->addr;
	msg[0].flags = 0;
	msg[0].len = sizeof(addr);
	msg[0].buf = buf;

	buf[0] = (addr & 0xFF00) >> 8;
	buf[1] = addr & 0xFF;

	msg[1].addr = pclient->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = bufsize;
	msg[1].buf = rbuf;

	ret = i2c_transfer(pclient->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		pr_err("Read failed (%d): addr 0x%x\n", ret, addr);
		return ret;
	}

	pr_info("Read: addr 0x%04x val 0x%04x\n", addr, rbuf[0]);

	return 0;
}

int lwis_i2c_write(struct lwis_i2c *pi2c, u16 addr, u8 *wbuf, u32 bufsize)
{
	int ret;
	struct i2c_client *pclient = pi2c->pclient;
	struct i2c_msg msg;
	u8 *buf;

	buf = kzalloc(sizeof(addr) + bufsize, GFP_KERNEL);
	if (!buf) {
		pr_err("Failed to allocate memory for i2c buffer\n");
		return -ENOMEM;
	}

	msg.addr = pclient->addr;
	msg.flags = 0;
	msg.len = sizeof(addr) + bufsize;
	msg.buf = buf;

	buf[0] = (addr & 0xFF00) >> 8;
	buf[1] = addr & 0xFF;
	memcpy(buf + sizeof(addr), wbuf, bufsize);

	ret = i2c_transfer(pclient->adapter, &msg, 1);
	if (ret < 0) {
		pr_err("Write failed (%d): addr 0x%x\n", ret, addr);
	}

	kfree(buf);

	return 0;
}
