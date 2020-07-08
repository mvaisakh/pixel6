/*
 * Copyright 2019 Google, Inc
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

#ifndef MAX_M5_H_
#define MAX_M5_H_

#include "max1720x_battery.h"

#define MAX_M5_I2C_ADDR 0x6C

/** ------------------------------------------------------------------------ */

/* change to 1 or 0 to load FG model with default parameters on startup */
#define MAX_M5_LOAD_MODEL_DISABLED	-1
#define MAX_M5_LOAD_MODEL_IDLE		0
#define MAX_M5_LOAD_MODEL_REQUEST	1

#define MAX_M5_FG_MODEL_START		0x80
#define MAX_M5_FG_MODEL_SIZE		48

#define MAX_M5_UNLOCK_EXTRA_CONFIG	0x60
#define MAX_M5_UNLOCK_EXTRA_CONFIG_UNLOCK_CODE	0x80
#define MAX_M5_UNLOCK_EXTRA_CONFIG_LOCK_CODE	0x00

#define MAX_M5_UNLOCK_MODEL_ACCESS	0x62
#define MAX_M5_MODEL_ACCESS_UNLOCK_CODE	0xc459
#define MAX_M5_MODEL_ACCESS_LOCK_CODE	0x0000
#define MAX_M5_MODEL_ACCESS_LOCK_OK	0xFFFF

#define MAX_M5_TCURVE	0xB9
#define MAX_M5_VFSOC	0xFF

/** ------------------------------------------------------------------------ */

/*
 * Custom parameters are updated while the device is runnig. These are saved
 * to permanent storage and restored from userspace (on load).
 */
struct max_m5_custom_parameters {
	u16 iavg_empty; /* WV */
	u16 relaxcfg;
	u16 learncfg;
	u16 config;
	u16 config2;
	u16 fullsocthr;
	u16 fullcaprep; /* WV */
	u16 designcap;
	u16 dpacc;	/* WV */
	u16 dqacc;	/* WV */
	u16 fullcapnom;	/* WV */
	u16 v_empty;
	u16 qresidual00;	/* WV */
	u16 qresidual10;	/* WV */
	u16 qresidual20;	/* WV */
	u16 qresidual30;	/* WV */
	u16 rcomp0;	/* WV */
	u16 tempco;	/* WV */
	u16 ichgterm;
	u16 tgain;
	u16 toff;
	u16 tcurve; 	/* write to 0x00B9 */
	u16 misccfg;	/* 0x9d0 for internal current sense, 0x8d0 external */

	u16 atrate;
	u16 convgcfg;
	u16 filtercfg; 	/* write to 0x0029 */
} __attribute__((packed));

struct model_state_save {
	u16 rcomp0;
	u16 tempco;
	u16 fullcaprep;
	u16 cycles;
	u16 fullcapnom;
	u16 qresidual00;
	u16 qresidual10;
	u16 qresidual20;
	u16 qresidual30;
	u16 mixcap;
	u16 halftime;
};

struct max_m5_data {
	struct device *dev;

	struct max_m5_custom_parameters parameters;
	u16 cycles;
	u16 mixcap;
	u16 halftime;

	struct max17x0x_regmap *regmap;

	int custom_model_size;
	u16 *custom_model;

	struct model_state_save model_save;
};

int max_m5_regmap_init(struct max17x0x_regmap *regmap,
		       struct i2c_client *primary);

void *max_m5_init_data(struct device *dev, struct device_node *batt_node,
		       struct max17x0x_regmap *regmap);
void max_m5_free_data(void *data);

/* load state from storage */
int max_m5_load_state_data(struct max_m5_data *m5_data);
/* save state to storage */
int max_m5_save_state_data(struct max_m5_data *m5_data);

/* read state from the gauge */
int max_m5_model_read_state(struct max_m5_data *m5_data);

/* load model to gauge */
int max_m5_load_gauge_model(struct max_m5_data *m5_data);

ssize_t max_m5_model_state_cstr(char *buf, int max,
				struct max_m5_data *m5_data);
int max_m5_model_state_sscan(struct max_m5_data *m5_data, const char *buf,
			     int max);
int max_m5_fg_model_sscan(struct max_m5_data *m5_data, const char *buf,
			  int max);
int max_m5_fg_model_cstr(char *buf, int max, const struct max_m5_data *m5_data);

int max_m5_read_actual_input_current_ua(struct i2c_client *client, int *iic);

int max_m5_reg_read(struct i2c_client *client, unsigned int reg,
		    unsigned int *val);
int max_m5_reg_write(struct i2c_client *client, unsigned int reg,
		     unsigned int val);

void *max1720x_get_model_data(struct i2c_client *client);

#endif