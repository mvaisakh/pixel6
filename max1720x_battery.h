/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google Battery Management System
 *
 * Copyright 2020 Google, Inc
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

#ifndef MAX1720X_BATTERY_H_
#define MAX1720X_BATTERY_H_

#include <linux/device.h>
#include <linux/regmap.h>

#define MAX1720X_GAUGE_TYPE	0
#define MAX1730X_GAUGE_TYPE	1
#define MAX_M5_GAUGE_TYPE	2

static inline int reg_to_micro_amp_h(s16 val, u16 rsense)
{
	/* LSB: 5.0μVh/RSENSE ; Rsense LSB is 10μΩ */
	return div_s64((s64) val * 500000, rsense);
}

static inline int reg_to_micro_volt(u16 val)
{
	/* LSB: 0.078125mV */
	return div_u64((u64) val * 78125, 1000);
}


enum max17x0x_reg_tags {
	MAX17X0X_TAG_avgc,
	MAX17X0X_TAG_cnfg,
	MAX17X0X_TAG_mmdv,
	MAX17X0X_TAG_vcel,
	MAX17X0X_TAG_temp,
	MAX17X0X_TAG_curr,
	MAX17X0X_TAG_mcap,
	MAX17X0X_TAG_avgr,
	MAX17X0X_TAG_vfsoc,
	MAX17X0X_TAG_vfocv,

	MAX17X0X_TAG_BCNT,
	MAX17X0X_TAG_SNUM,
	MAX17X0X_TAG_HSTY,
	MAX17X0X_TAG_BCEA,
	MAX17X0X_TAG_rset,
	MAX17X0X_TAG_BRES,
};

enum max17x0x_reg_types {
	GBMS_ATOM_TYPE_MAP = 0,
	GBMS_ATOM_TYPE_REG = 1,
	GBMS_ATOM_TYPE_ZONE = 2,
	GBMS_ATOM_TYPE_SET = 3,
};

/* this is a map for u16 registers */
#define ATOM_INIT_MAP(...)			\
	.type = GBMS_ATOM_TYPE_MAP,		\
	.size = 2 * sizeof((u8[]){__VA_ARGS__}),\
	.map = (u8[]){__VA_ARGS__}

#define ATOM_INIT_REG16(r)		\
	.type = GBMS_ATOM_TYPE_REG,	\
	.size = 2,			\
	.reg = r

#define ATOM_INIT_ZONE(start, sz)	\
	.type = GBMS_ATOM_TYPE_ZONE,	\
	.size = sz,			\
	.base = start

/* a set has no storage and cannot be used in load/store */
#define ATOM_INIT_SET(...)		\
	.type = GBMS_ATOM_TYPE_SET,	\
	.size = 0,			\
	.map = (u8[]){__VA_ARGS__}

#define ATOM_INIT_SET16(...)		\
	.type = GBMS_ATOM_TYPE_SET,	\
	.size = 0,			\
	.map16 = (u16[]){__VA_ARGS__}

struct max17x0x_reg {
	int type;
	int size;
	union {
		unsigned int base;
		unsigned int reg;
		const u16 *map16;
		const u8 *map;
	};
};

struct max17x0x_cache_data {
	struct max17x0x_reg atom;
	u16 *cache_data;
};

#define NB_REGMAP_MAX 256

struct max17x0x_reglog {
	u16 data[NB_REGMAP_MAX];
	DECLARE_BITMAP(valid, NB_REGMAP_MAX);
	int errors[NB_REGMAP_MAX];
	int count[NB_REGMAP_MAX];
};

struct max17x0x_regtags {
	const struct max17x0x_reg *map;
	unsigned int max;
};

struct max17x0x_regmap {
	struct regmap *regmap;
	struct max17x0x_regtags regtags;
	struct max17x0x_reglog *reglog;
};

/* */
#ifdef CONFIG_MAX1720X_REGLOG_LOG
static inline void max17x0x_reglog_log(struct max17x0x_reglog *reglog,
				       unsigned int reg, u16 data, int rtn)
{
	if (!reglog)
		return;

	reglog->count[reg] += 1;
	if (rtn != 0) {
		reglog->errors[reg] += 1;
	} else {
		__set_bit(reg, reglog->valid);
		reglog->data[reg] = data;
	}

}

#else
static inline void max17x0x_reglog_log(struct max17x0x_reglog *reglog,
				       unsigned int reg, u16 data, int rtn)
{

}
#endif

static inline int max17x0x_regmap_read(const struct max17x0x_regmap *map,
				       unsigned int reg,
				       u16 *val,
				       const char *name)
{
	int rtn;
	unsigned int tmp;

	if (!map->regmap) {
		pr_err("Failed to read %s, no regmap\n", name);
		return -EIO;
	}

	rtn = regmap_read(map->regmap, reg, &tmp);
	if (rtn)
		pr_err("Failed to read %s\n", name);
	else
		*val = tmp;

	return rtn;
}

#define REGMAP_READ(regmap, what, dst) \
	max17x0x_regmap_read(regmap, what, dst, #what)

static inline int max17x0x_regmap_write(const struct max17x0x_regmap *map,
				       unsigned int reg,
				       u16 data,
				       const char *name)
{
	int rtn;

	if (!map->regmap) {
		pr_err("Failed to write %s, no regmap\n", name);
		return -EIO;
	}

	rtn = regmap_write(map->regmap, reg, data);
	if (rtn)
		pr_err("Failed to write %s\n", name);

	max17x0x_reglog_log(map->reglog, reg, data, rtn);

	return rtn;
}

#define REGMAP_WRITE(regmap, what, value) \
	max17x0x_regmap_write(regmap, what, value, #what)

static inline int max1720x_regmap_writeverify(const struct max17x0x_regmap *map,
					unsigned int reg,
					u16 data,
					const char *name)
{
	int tmp, ret;

	if (!map->regmap) {
		pr_err("Failed to write %s, no regmap\n", name);
		return -EINVAL;
	}

	ret = regmap_write(map->regmap, reg, data);
	if (ret < 0) {
		pr_err("Failed to write %s\n", name);
		return -EIO;
	}

	ret = regmap_read(map->regmap, reg, &tmp);
	if (ret < 0) {
		pr_err("Failed to read %s\n", name);
		return -EIO;
	}

	return (tmp == data) ? 0 : -EIO;
}

#define REGMAP_WRITE_VERIFY(regmap, what, value) \
	max1720x_regmap_writeverify(regmap, what, value, #what)

#endif