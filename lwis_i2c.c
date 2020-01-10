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
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>

#define I2C_DEVICE_NAME "LWIS_I2C"

/* Max bit width for register and data that is supported by this
   driver currently */
#define MIN_OFFSET_BITS 8
#define MAX_OFFSET_BITS 16
#define MIN_DATA_BITS 8
#define MAX_DATA_BITS 32

static inline bool check_bitwidth(const int bitwidth, const int min,
				  const int max)
{
	return (bitwidth >= min) && (bitwidth <= max) && ((bitwidth % 8) == 0);
}

static int perform_read_transfer(struct i2c_client *client, struct i2c_msg *msg,
				 uint64_t offset, int offset_bits,
				 int value_bits, uint64_t *value)
{
	int ret = 0;
	u8 *wbuf = msg[0].buf;
	u8 *rbuf = msg[1].buf;

	const int num_msg = 2;

	if (offset_bits == 8) {
		wbuf[0] = offset;
	} else if (offset_bits == 16) {
		wbuf[0] = (offset >> 8) & 0xFF;
		wbuf[1] = offset & 0xFF;
	} else {
		pr_err("%s: Read with unsupported offset bits %d\n",
		       client->name, offset);
		return -EPERM;
	}

	ret = i2c_transfer(client->adapter, msg, num_msg);
	if (ret != num_msg) {
		pr_err("Read failed (%d): offset 0x%llx\n", ret, offset);
		return ret;
	}

	if (value_bits == 8) {
		*value = rbuf[0];
	} else if (value_bits == 16) {
		*value = (rbuf[0] << 8) | rbuf[1];
	} else if (value_bits == 32) {
		*value = (rbuf[0] << 24) | (rbuf[1] << 16) | (rbuf[2] << 8) |
			 rbuf[3];
	} else {
		pr_err("%s: Read with unsupported value bits %d\n",
		       client->name, value_bits);
		return -EPERM;
	}

	// pr_info("Read: offset 0x%04llx val 0x%04llx\n", offset, *value);

	return ret;
}

static int perform_write_transfer(struct i2c_client *client,
				  struct i2c_msg *msg, uint64_t offset,
				  int offset_bits, int value_bits,
				  uint64_t value)
{
	int i = 0;
	int ret = 0;
	u8 *buf = msg->buf;

	const int num_msg = 1;

	if (offset_bits == 8) {
		buf[i++] = offset;
	} else if (offset_bits == 16) {
		buf[i++] = (offset >> 8) & 0xFF;
		buf[i++] = offset & 0xFF;
	} else {
		pr_err("%s: Write with unsupported offset bits %d\n",
		       client->name, offset);
		return -EPERM;
	}

	if (value_bits == 8) {
		buf[i++] = value;
	} else if (value_bits == 16) {
		buf[i++] = (value >> 8) & 0xFF;
		buf[i++] = value & 0xFF;
	} else if (value_bits == 32) {
		buf[i++] = (value >> 24) & 0xFF;
		buf[i++] = (value >> 16) & 0xFF;
		buf[i++] = (value >> 8) & 0xFF;
		buf[i++] = value & 0xFF;
	} else {
		pr_err("%s: Write with unsupported value bits %d\n",
		       client->name, value_bits);
		return -EPERM;
	}

	ret = i2c_transfer(client->adapter, msg, num_msg);
	if (ret != num_msg) {
		pr_err("Write failed (%d): offset 0x%llx\n", ret, offset);
	}

	return ret;
}

int lwis_i2c_set_state(struct lwis_i2c_device *i2c, const char *state_str)
{
	int ret;
	struct pinctrl_state *state;

	if (!i2c || !i2c->state_pinctrl) {
		pr_err("Cannot find i2c instance\n");
		return -ENODEV;
	}

	state = pinctrl_lookup_state(i2c->state_pinctrl, state_str);
	if (IS_ERR(state)) {
		pr_err("State %s not found (%ld)\n", state_str, PTR_ERR(state));
		return PTR_ERR(state);
	}

	ret = pinctrl_select_state(i2c->state_pinctrl, state);
	if (ret) {
		pr_err("Error selecting state %s (%d)\n", state_str, ret);
		return ret;
	}

	return 0;
}

int lwis_i2c_read_batch(struct lwis_i2c_device *i2c,
			struct lwis_io_entry *entries, int num_entries)
{
	int i;
	int ret;
	u8 *wbuf;
	u8 *rbuf;
	struct i2c_client *client = i2c->client;
	struct i2c_msg i2c_msg[2];
	unsigned int offset_bits;
	unsigned int value_bits;

	const int num_i2c_msg = 2;

	if (!i2c || !i2c->client) {
		pr_err("Cannot find i2c instance\n");
		return -ENODEV;
	}

	if (num_entries <= 0) {
		pr_err("Invalid number of entries %d\n", num_entries);
		return -EINVAL;
	}

	offset_bits = i2c->base_dev.reg_addr_bitwidth;
	value_bits = i2c->base_dev.reg_value_bitwidth;

	if (!check_bitwidth(offset_bits, MIN_OFFSET_BITS, MAX_OFFSET_BITS)) {
		pr_err("Invalid offset bitwidth %d\n", offset_bits);
		return -EINVAL;
	}

	wbuf = kzalloc(offset_bits / 8, GFP_KERNEL);
	if (!wbuf) {
		pr_err("Failed to allocate memory for i2c write buffer\n");
		return -ENOMEM;
	}

	// Allocating read buffer to be max data bits allowed, as the data
	// bitwidth in the list could potentially be diffent throughout the list
	rbuf = kzalloc(MAX_DATA_BITS / 8, GFP_KERNEL);
	if (!rbuf) {
		pr_err("Failed to allocate memory for i2c read buffer\n");
		ret = -ENOMEM;
		goto error_rbuf_alloc;
	}

	i2c_msg[0].addr = client->addr;
	i2c_msg[0].flags = 0;
	i2c_msg[0].len = offset_bits / 8;
	i2c_msg[0].buf = wbuf;

	i2c_msg[1].addr = client->addr;
	i2c_msg[1].flags = I2C_M_RD;
	i2c_msg[1].buf = rbuf;

	mutex_lock(&i2c->base_dev.reg_rw_lock);
	for (i = 0; i < num_entries; ++i) {
		i2c_msg[1].len = value_bits / 8;
		ret = perform_read_transfer(client, i2c_msg,
					    entries[i].rw.offset, offset_bits,
					    value_bits, &entries[i].rw.val);
		if (ret != num_i2c_msg) {
			break;
		}
	}
	mutex_unlock(&i2c->base_dev.reg_rw_lock);

	kfree(rbuf);
error_rbuf_alloc:
	kfree(wbuf);

	return (ret == num_i2c_msg) ? 0 : ret;
}

int lwis_i2c_write_batch(struct lwis_i2c_device *i2c,
			 struct lwis_io_entry *entries, int num_entries)
{
	int i;
	int ret;
	u8 *buf;
	struct i2c_client *client = i2c->client;
	struct i2c_msg i2c_msg;
	unsigned int offset_bits;
	unsigned int value_bits;

	const int num_i2c_msg = 1;

	if (!i2c || !i2c->client) {
		pr_err("Cannot find i2c instance\n");
		return -ENODEV;
	}

	if (num_entries <= 0) {
		pr_err("Invalid number of entries %d\n", num_entries);
		return -EINVAL;
	}

	offset_bits = i2c->base_dev.reg_addr_bitwidth;
	value_bits = i2c->base_dev.reg_value_bitwidth;

	if (!check_bitwidth(offset_bits, MIN_OFFSET_BITS, MAX_OFFSET_BITS)) {
		pr_err("Invalid offset bitwidth %d\n", offset_bits);
		return -EINVAL;
	}

	buf = kzalloc((offset_bits + MAX_DATA_BITS) / 8, GFP_KERNEL);
	if (!buf) {
		pr_err("Failed to allocate memory for i2c buffer\n");
		return -ENOMEM;
	}

	i2c_msg.addr = client->addr;
	i2c_msg.flags = 0;
	i2c_msg.buf = buf;

	mutex_lock(&i2c->base_dev.reg_rw_lock);
	for (i = 0; i < num_entries; ++i) {
		i2c_msg.len = (offset_bits + value_bits) / 8;
		ret = perform_write_transfer(client, &i2c_msg,
					     entries[i].rw.offset, offset_bits,
					     value_bits, entries[i].rw.val);
		if (ret != num_i2c_msg) {
			break;
		}
	}
	mutex_unlock(&i2c->base_dev.reg_rw_lock);

	kfree(buf);

	return (ret == num_i2c_msg) ? 0 : ret;
}

int lwis_i2c_read(struct lwis_i2c_device *i2c, uint64_t offset, uint64_t *value)
{
	int ret;
	u8 *wbuf;
	u8 *rbuf;
	struct i2c_client *client = i2c->client;
	struct i2c_msg msg[2];

	const int num_msg = ARRAY_SIZE(msg);
	const unsigned int offset_bits = i2c->base_dev.reg_addr_bitwidth;
	const unsigned int value_bits = i2c->base_dev.reg_value_bitwidth;

	if (!i2c || !i2c->client) {
		pr_err("Cannot find i2c instance\n");
		return -ENODEV;
	}

	if (!check_bitwidth(offset_bits, MIN_OFFSET_BITS, MAX_OFFSET_BITS)) {
		pr_err("Invalid offset bitwidth %d\n", offset_bits);
		return -EINVAL;
	}

	if (!check_bitwidth(value_bits, MIN_DATA_BITS, MAX_DATA_BITS)) {
		pr_err("Invalid value bitwidth %d\n", value_bits);
		return -EINVAL;
	}

	wbuf = kzalloc(offset_bits / 8, GFP_KERNEL);
	if (!wbuf) {
		pr_err("Failed to allocate memory for i2c write buffer\n");
		return -ENOMEM;
	}

	rbuf = kzalloc(value_bits / 8, GFP_KERNEL);
	if (!rbuf) {
		pr_err("Failed to allocate memory for i2c read buffer\n");
		ret = -ENOMEM;
		goto error_rbuf_alloc;
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = offset_bits / 8;
	msg[0].buf = wbuf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = value_bits / 8;
	msg[1].buf = rbuf;

	mutex_lock(&i2c->base_dev.reg_rw_lock);
	ret = perform_read_transfer(client, msg, offset, offset_bits,
				    value_bits, value);
	mutex_unlock(&i2c->base_dev.reg_rw_lock);

	kfree(rbuf);
error_rbuf_alloc:
	kfree(wbuf);

	return (ret == num_msg) ? 0 : ret;
}

int lwis_i2c_write(struct lwis_i2c_device *i2c, uint64_t offset, uint64_t value)
{
	int ret;
	u8 *buf;
	struct i2c_client *client = i2c->client;
	struct i2c_msg msg;

	const int num_msg = 1;
	const unsigned int offset_bits = i2c->base_dev.reg_addr_bitwidth;
	const unsigned int value_bits = i2c->base_dev.reg_value_bitwidth;
	const int msg_bytes = (offset_bits + value_bits) / 8;

	if (!i2c || !i2c->client) {
		pr_err("Cannot find i2c instance\n");
		return -ENODEV;
	}

	if (!check_bitwidth(offset_bits, MIN_OFFSET_BITS, MAX_OFFSET_BITS)) {
		pr_err("Invalid offset bitwidth %d\n", offset_bits);
		return -EINVAL;
	}

	if (!check_bitwidth(value_bits, MIN_DATA_BITS, MAX_DATA_BITS)) {
		pr_err("Invalid value bitwidth %d\n", value_bits);
		return -EINVAL;
	}

	buf = kzalloc(msg_bytes, GFP_KERNEL);
	if (!buf) {
		pr_err("Failed to allocate memory for i2c buffer\n");
		return -ENOMEM;
	}

	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = buf;
	msg.len = msg_bytes;

	mutex_lock(&i2c->base_dev.reg_rw_lock);
	ret = perform_write_transfer(client, &msg, offset, offset_bits,
				     value_bits, value);
	mutex_unlock(&i2c->base_dev.reg_rw_lock);

	kfree(buf);

	return (ret == num_msg) ? 0 : ret;
}
