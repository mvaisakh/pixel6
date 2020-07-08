/*
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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/ctype.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include "max77759.h"
#include "max77759_maxq.h"
#include "max_m5.h"

enum max77729_pmic_register {
	MAX77729_PMIC_ID         = 0x00,
	MAX77729_PMIC_REVISION   = 0x01,
	MAX77729_PMIC_MAINCTRL   = 0x02,
	MAX77729_PMIC_INTSRC     = 0x22,
	MAX77729_PMIC_INTSRCMASK = 0x23,
	MAX77729_PMIC_TOPSYS_INT = 0x24,
	MAX77729_PMIC_TOPSYS_INT_MASK = 0x26,
};

#define MUSBC 0x08
#define MFUEL 0x04
#define MTOPS 0x02
#define MCHGR 0x01

#define MAX77759_GPIO_DIR_IN  0
#define MAX77759_GPIO_DIR_OUT 1

#define MAX77759_GPIO5_DIR_MASK (1 << 2)
#define MAX77759_GPIO5_DIR(x) ((x) << 2)
#define MAX77759_GPIO5_VAL_MASK (1 << 3)
#define MAX77759_GPIO5_VAL(x) ((x) << 3)
#define MAX77759_GPIO6_DIR_MASK (1 << 4)
#define MAX77759_GPIO6_DIR(x) ((x) << 4)
#define MAX77759_GPIO6_VAL_MASK (1 << 5)
#define MAX77759_GPIO6_VAL(x) ((x) << 5)

#define MAX77759_GPIO5_OFF 4
#define MAX77759_GPIO6_OFF 5
#define MAX77759_MIN_GPIO_OFF 4
#define MAX77759_MAX_GPIO_OFF 5
#define MAX77759_NUM_GPIOS 6

#define MAX77759_GPIO_CONTROL_READ 0x23
#define MAX77759_GPIO_CONTROL_WRITE 0x24

#define MDEFAULT (0xF0 | ~(MUSBC | MFUEL | MCHGR))


#define MAX77729_PMIC_INTSRCMASK_DEFAULT	MDEFAULT
#define MAX77729_PMIC_TOPSYS_INT_MASK_DEFAULT	0xff

#define MAX77759_PMIC_INTSRCMASK_DEFAULT \
		~(MAX77759_PMIC_INTSRCMASK_MAXQ_INT_M | \
		  MAX77759_PMIC_INTSRCMASK_CHGR_INT_M)

/* b/156527175: *_PMIC_TOPSYS_INT_MASK_SPR_7 is reserved */
#define MAX77759_PMIC_TOPSYS_INT_MASK_MASK \
		(MAX77759_PMIC_TOPSYS_INT_MASK_TSHDN_INT_M | \
		 MAX77759_PMIC_TOPSYS_INT_MASK_SYSOVLO_INT_M | \
		 MAX77759_PMIC_TOPSYS_INT_MASK_SYSUVLO_INT_M)

#define MAX77759_PMIC_TOPSYS_INT_MASK_DEFAULT \
		(MAX77759_PMIC_TOPSYS_INT_MASK_TSHDN_INT_M)

struct max77729_pmic_data {
	struct device        *dev;
	struct regmap        *regmap;
	uint8_t pmic_id;

#ifdef CONFIG_GPIOLIB
	struct gpio_chip     gpio;
#endif
	struct max77759_maxq *maxq;

	struct i2c_client *fg_i2c_client;
	struct mutex io_lock;
	int batt_id;

	atomic_t sysuvlo_cnt;
	atomic_t sysovlo_cnt;
	struct dentry *de;
};

bool max77729_pmic_is_reg(struct device *dev, unsigned int reg)
{
	int ret;

	switch (reg) {
	case MAX77729_PMIC_ID:
	case MAX77729_PMIC_REVISION:
	case MAX77729_PMIC_MAINCTRL:
	case MAX77729_PMIC_INTSRC:
	case MAX77729_PMIC_INTSRCMASK:
	case MAX77729_PMIC_TOPSYS_INT:
	case MAX77729_PMIC_TOPSYS_INT_MASK:
		ret = true;
		break;
	case MAX77759_PMIC_I2C_CNFG:
	case MAX77759_PMIC_SWRESET:
	case MAX77759_PMIC_CONTROL_FG:
		ret = true;
		break;
	case MAX77759_PMIC_DEVICE_ID:
	case MAX77759_PMIC_DEVICE_REV:
	case MAX77759_PMIC_UIC_INT1...MAX77759_PMIC_UIC_INT4_M:
	case MAX77759_PMIC_AP_DATAOUT0...MAX77759_PMIC_AP_DATAIN32:
	case MAX77759_PMIC_UIC_SWRST:
		ret = true;
		break;
	default:
		ret = false;
		break;
	}

	return ret;
}

static struct regmap_config max777x9_pmic_regmap_cfg = {
	.name = "max777x9_pmic",
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX77729_PMIC_INTSRCMASK,
	.readable_reg = max77729_pmic_is_reg,
	.volatile_reg = max77729_pmic_is_reg,
};

static inline int max77729_pmic_readn(struct max77729_pmic_data *data,
				      int addr, u8 *val, int len)
{
	int rc;

	rc = regmap_bulk_read(data->regmap, addr, val, len);
	if (rc < 0)
		pr_err("regmap_read failed for address %04x rc=%d\n",
			addr, rc);

	return rc;
}

static inline int max77729_pmic_writen(struct max77729_pmic_data *data,
				       int addr, const u8 *val, int len)
{
	int rc;

	rc = regmap_bulk_write(data->regmap, addr, val, len);
	if (rc < 0)
		pr_err("regmap_write failed for address %04x rc=%d\n",
			addr, rc);

	return 0;
}


#define max77729_pmic_rd8(data, addr, val) \
		max77729_pmic_readn(data, addr, val, 1)
#define max77729_pmic_wr8(data, addr, val) \
		max77729_pmic_writen(data, addr, (const u8[]){ val }, 1)

/* no need for caching */
static inline int max77729_pmic_rmw8(struct max77729_pmic_data *data,
				     int reg, u8 mask, u8 value)
{
	return regmap_write_bits(data->regmap, reg, mask, value);
}

int max777x9_pmic_reg_read(struct i2c_client *client,
			   u8 addr, u8 *val, int len)
{
	struct max77729_pmic_data *data = i2c_get_clientdata(client);

	if (!data || !data->regmap)
		return -ENXIO;

	return max77729_pmic_readn(data, addr, val, len);
}
EXPORT_SYMBOL_GPL(max777x9_pmic_reg_read);

int max777x9_pmic_reg_write(struct i2c_client *client,
			    u8 addr, const u8 *val, int len)
{
	struct max77729_pmic_data *data = i2c_get_clientdata(client);

	if (!data || !data->regmap)
		return -ENXIO;

	return max77729_pmic_writen(data, addr, val, len);
}
EXPORT_SYMBOL_GPL(max777x9_pmic_reg_write);

int max777x9_pmic_reg_update(struct i2c_client *client,
			     u8 reg, u8 mask, u8 value)
{
	struct max77729_pmic_data *data = i2c_get_clientdata(client);

	if (!data || !data->regmap)
		return -ENXIO;

	return max77729_pmic_rmw8(data, reg, mask, value);
}
EXPORT_SYMBOL_GPL(max777x9_pmic_reg_update);


/* this interrupt is read to clear, in max77759 it should be write to clear */
static irqreturn_t max777x9_pmic_irq(int irq, void *ptr)
{
	struct max77729_pmic_data *data = ptr;
	uint8_t intsrc = 0, uic_int[4];
	int ret;

	/* INTSRC is read to clear on MW and max77729f */
	ret = max77729_pmic_rd8(data, MAX77729_PMIC_INTSRC, &intsrc);
	if (ret < 0) {
		dev_err_ratelimited(data->dev, "INTSRC: read error %d\n", ret);
		return IRQ_NONE;
	}

	if (intsrc == 0)
		return IRQ_NONE;

	/* just clear for max77729f */
	pr_debug("INTSRC:%x\n", intsrc);
	if (data->pmic_id != MAX77759_PMIC_PMIC_ID_MW)
		return IRQ_HANDLED;

	/* UIC_INT are write to clear */
	if (intsrc & MAX77759_PMIC_INTSRC_MAXQ_INT) {
		ret = max77729_pmic_readn(data, MAX77759_PMIC_UIC_INT1,
					  uic_int, sizeof(uic_int));
		if (ret < 0) {
			dev_err_ratelimited(data->dev,
				"UIC_INT1: read error %d\n", ret);
			return IRQ_NONE;
		}

		/* TODO: implement / handle comms with maxq */
		if (uic_int[0] & MAX77759_PMIC_UIC_INT1_APCMDRESI) {
			maxq_irq(data->maxq);
			max77729_pmic_wr8(data, MAX77759_PMIC_UIC_INT1,
					  MAX77759_PMIC_UIC_INT1_APCMDRESI);
		}
	}

	if (intsrc & MAX77759_PMIC_TOPSYS_INT_SYSUVLO_INT)
		atomic_inc(&data->sysuvlo_cnt);

	if (intsrc & MAX77759_PMIC_TOPSYS_INT_SYSOVLO_INT)
		atomic_inc(&data->sysovlo_cnt);

	if (intsrc & MAX77759_PMIC_INTSRC_TOPSYS_INT) {
		uint8_t tsi;

		ret = max77729_pmic_rd8(data, MAX77729_PMIC_TOPSYS_INT, &tsi);
		if (ret < 0) {
			dev_err_ratelimited(data->dev,
					"TOPSYS_INT: read error %d\n", ret);
			return IRQ_NONE;
		}

		ret = max77729_pmic_wr8(data, MAX77729_PMIC_TOPSYS_INT, tsi);
		if (ret < 0) {
			dev_err_ratelimited(data->dev,
				"TOPSYS_INT:%x clr error %d\n", tsi, ret);
			return IRQ_NONE;
		}

		pr_info("TOPSYS_INT:%x\n", tsi);

		/* TODO: handle TSHDN_INT, SYSOVLO_INT, SYSUVLO_INT */
	}

	/* just clear CHG_INT, no FG intr for MW */

	return IRQ_HANDLED;
}

/* Bootloader has everything masked clear this on boot */
static int max777x9_pmic_set_irqmask(struct max77729_pmic_data *data)
{
	int ret;

	if (data->pmic_id == MAX77759_PMIC_PMIC_ID_MW) {
		const uint8_t uic_mask[] = {0x7f, 0xff, 0xff, 0xff};
		uint8_t reg;

		ret = max77729_pmic_rd8(data, MAX77759_PMIC_INTSRC,
					&reg);
		if (ret < 0 || reg)
			dev_info(data->dev, "INTSRC :%x (%d)\n", reg, ret);
		ret = max77729_pmic_rd8(data, MAX77759_PMIC_TOPSYS_INT,
					&reg);
		if (ret < 0 || reg)
			dev_info(data->dev, "TOPSYS_INT :%x (%d)\n", reg, ret);

		ret = max77729_pmic_wr8(data, MAX77759_PMIC_INTSRCMASK,
					MAX77759_PMIC_INTSRCMASK_DEFAULT);

		/* b/156527175, *_PMIC_TOPSYS_INT_MASK_SPR_7 is reserved */
		ret |= max77729_pmic_rmw8(data, MAX77759_PMIC_TOPSYS_INT_MASK,
					MAX77759_PMIC_TOPSYS_INT_MASK_MASK,
					MAX77759_PMIC_TOPSYS_INT_MASK_DEFAULT);
		ret |= max77729_pmic_wr8(data, MAX77759_PMIC_UIC_INT1,
					 MAX77759_PMIC_UIC_INT1_APCMDRESI);
		ret |= max77729_pmic_writen(data, MAX77759_PMIC_UIC_INT1_M,
					    uic_mask, sizeof(uic_mask));
	} else {
		ret = max77729_pmic_wr8(data, MAX77729_PMIC_INTSRCMASK,
					MAX77729_PMIC_INTSRCMASK_DEFAULT);
		ret |= max77729_pmic_wr8(data, MAX77729_PMIC_TOPSYS_INT_MASK,
					 MAX77729_PMIC_TOPSYS_INT_MASK_DEFAULT);
	}

	return ret ? -EIO : 0;
}

static int max77759_find_fg(struct max77729_pmic_data *data)
{
	struct device_node *dn;

	if (data->fg_i2c_client)
		return 0;

	dn = of_parse_phandle(data->dev->of_node, "max77759,max_m5", 0);
	if (!dn)
		return -ENXIO;

	data->fg_i2c_client = of_find_i2c_device_by_node(dn);
	if (!data->fg_i2c_client)
		return -EAGAIN;

	return 0;
}

static int max77759_read_thm(struct max77729_pmic_data *data, int mux,
			     unsigned int *value)
{
	unsigned int ain0, config;
	int tmp, ret;
	u8 pmic_ctrl;

	if (!data->fg_i2c_client)
		return -EINVAL;

	/* set TEX=1 in Config 0x1D */
	ret = max_m5_reg_read(data->fg_i2c_client, MAX77759_FG_CONFIG, &config);
	if (ret == 0) {
		const u16 val = _fg_config_tex_set(config, 1);

		ret = max_m5_reg_write(data->fg_i2c_client, MAX77759_FG_CONFIG,
				       val);
	}
	if (ret < 0) {
		pr_err("%s: cannot change FG config (%d)\n", __func__, ret);
		return -EIO;
	}

	/* set THMIO_MUX */
	ret = max77729_pmic_rd8(data, MAX77759_PMIC_CONTROL_FG, &pmic_ctrl);
	if (ret == 0) {
		const u8 val = _pmic_control_fg_thmio_mux_set(pmic_ctrl, mux);

		ret = max77729_pmic_wr8(data, MAX77759_PMIC_CONTROL_FG, val);
	}

	if (ret < 0) {
		pr_err("%s: cannot change MUX config (%d)\n", __func__, ret);
		goto restore_fg;
	}

	/* this is not good */
	msleep(1500);

	ret = max_m5_reg_read(data->fg_i2c_client, MAX77759_FG_AIN0, &ain0);
	pr_debug("%s: AIN0=%d (%d)\n", __func__, ain0, ret);

	/* AIN0 is ratiometric on THM, 0xffff = 100%, lsb is 2^-16 */
	*value = (100000 * (unsigned long)(ain0)) / (0x10000 - ain0);

	/* restore THMIO_MUX */
	tmp = max77729_pmic_wr8(data, MAX77759_PMIC_CONTROL_FG, pmic_ctrl);
	WARN_ON(tmp != 0);

restore_fg:
	/* set TEX=0 in Config 0x1D */
	tmp = max_m5_reg_write(data->fg_i2c_client, MAX77759_FG_CONFIG, config);
	WARN_ON(tmp != 0);

	return ret;
}


/* THMIO_MUX=1 in CONTROL_FG (0x51) */
int max77759_read_usb_temp(struct i2c_client *client, int *temp)
{
	struct max77729_pmic_data *data = i2c_get_clientdata(client);
	unsigned int val;
	int ret;

	mutex_lock(&data->io_lock);
	ret = max77759_find_fg(data);
	if (ret == 0)
		ret = max77759_read_thm(data, THMIO_MUX_USB_TEMP, &val);
	mutex_unlock(&data->io_lock);

	if (ret == 0) {
		/* TODO: b/160737498 convert voltage to temperature */
		*temp = val;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(max77759_read_usb_temp);

static int read_batt_id(struct max77729_pmic_data *data, unsigned int *id)
{
	if  (data->batt_id == -1) {
		unsigned int val;
		int ret;

		mutex_lock(&data->io_lock);
		ret = max77759_find_fg(data);
		if (ret == 0)
			ret = max77759_read_thm(data, THMIO_MUX_BATT_ID, &val);
		mutex_unlock(&data->io_lock);

		if (ret < 0)
			return ret;

		data->batt_id = val;
	}

	*id = data->batt_id;
	return 0;
}

/* THMIO_MUX=2 in CONTROL_FG (0x51) */
int max77759_read_batt_id(struct i2c_client *client, unsigned int *id)
{
	struct max77729_pmic_data *data = i2c_get_clientdata(client);

	return read_batt_id(data, id);
}
EXPORT_SYMBOL_GPL(max77759_read_batt_id);


/* must use repeated starts b/152373060 */
static int max77729_pmic_read_id(struct i2c_client *i2c)
{
	struct i2c_msg xfer[2];
	u8 reg = MAX77729_PMIC_ID;
	u8 pmic_id;
	int ret;

	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &reg;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 1;
	xfer[1].buf = &pmic_id;

	ret = i2c_transfer(i2c->adapter, xfer, 2);
	if (ret == 2)
		return pmic_id;
	return -EIO;
}

static int debug_batt_id_get(void *d, u64 *val)
{
	struct max77729_pmic_data *data = d;
	unsigned int batt_id;
	int ret;

	ret = read_batt_id(data, &batt_id);
	if (ret == 0)
		*val = batt_id;

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(debug_batt_id_fops, debug_batt_id_get, NULL, "%llu\n");

static int dbg_init_fs(struct max77729_pmic_data *data)
{
	data->de = debugfs_create_dir("max77729_pmic", 0);
	if (IS_ERR_OR_NULL(data->de))
		return -EINVAL;

	debugfs_create_atomic_t("sysuvlo_cnt", 0644, data->de,
				&data->sysuvlo_cnt);
	debugfs_create_atomic_t("sysovlo_cnt", 0644, data->de,
				&data->sysovlo_cnt);

	debugfs_create_file("batt_id", 0400, data->de, data,
			    &debug_batt_id_fops);

	return 0;
}

#ifdef CONFIG_GPIOLIB

static int max77759_gpio_get_direction(struct gpio_chip *chip,
				       unsigned int offset)
{
	struct max77729_pmic_data *data = gpiochip_get_data(chip);
	int rc;
	uint8_t val;

	if ((offset < MAX77759_MIN_GPIO_OFF) || (offset > MAX77759_MAX_GPIO_OFF))
		return -EINVAL;

	rc = maxq_gpio_control_read(data->maxq, &val);
	if (rc < 0) {
		dev_err(data->dev, "opcode read 0x23 failed\n");
		return rc;
	}

	if (offset == MAX77759_GPIO5_OFF)
		return !(val & MAX77759_GPIO5_DIR_MASK);

	return !(val & MAX77759_GPIO6_DIR_MASK);
}

static int max77759_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct max77729_pmic_data *data = gpiochip_get_data(chip);
	int rc;
	uint8_t val;

	if ((offset < MAX77759_MIN_GPIO_OFF) || (offset > MAX77759_MAX_GPIO_OFF))
		return -EINVAL;

	rc = maxq_gpio_control_read(data->maxq, &val);
	if (rc < 0) {
		dev_err(data->dev, "opcode read 0x23 failed\n");
		return rc;
	}

	if (offset == MAX77759_GPIO5_OFF)
		return !!(val & MAX77759_GPIO5_VAL_MASK);

	return !!(val & MAX77759_GPIO6_VAL_MASK);
}

static void max77759_gpio_set(struct gpio_chip *chip,
			      unsigned int offset, int value)
{
	struct max77729_pmic_data *data = gpiochip_get_data(chip);
	int rc;
	uint8_t val;
	uint8_t new_val;
	uint8_t dir;

	if ((offset < MAX77759_MIN_GPIO_OFF) || (offset > MAX77759_MAX_GPIO_OFF))
		return;

	rc = maxq_gpio_control_read(data->maxq, &val);
	if (rc < 0) {
		dev_err(data->dev, "opcode read 0x23 failed\n");
		return;
	}

	if (offset == MAX77759_GPIO5_OFF) {
		dir = !(val & MAX77759_GPIO5_DIR_MASK);
		if (dir != GPIOF_DIR_OUT)  {
			dev_err(data->dev, "not output\n");
			return;
		}
		new_val = val & ~MAX77759_GPIO5_VAL_MASK;
		new_val |= MAX77759_GPIO5_VAL(value);
	} else {  /* MAX77759_GPIO6_OFF */
		dir = !(val & MAX77759_GPIO6_DIR_MASK);
		if (dir != GPIOF_DIR_OUT)  {
			dev_err(data->dev, "not output\n");
			return;
		}
		new_val = val & ~MAX77759_GPIO6_VAL_MASK;
		new_val |= MAX77759_GPIO6_VAL(value);
	}

	if (new_val != val) {
		rc = maxq_gpio_control_write(data->maxq, new_val);
		if (rc < 0) {
			dev_err(data->dev, "opcode write 0x24 failed\n");
			return;
		}
	}

}

static int max77759_gpio_direction_input(struct gpio_chip *chip,
					 unsigned int offset)
{
	struct max77729_pmic_data *data = gpiochip_get_data(chip);
	int rc;
	uint8_t val;
	uint8_t new_val;

	if ((offset < MAX77759_MIN_GPIO_OFF) || (offset > MAX77759_MAX_GPIO_OFF))
		return -EINVAL;

	rc = maxq_gpio_control_read(data->maxq, &val);
	if (rc < 0) {
		dev_err(data->dev, "opcode read 0x23 failed\n");
		return rc;
	}

	if (offset == MAX77759_GPIO5_OFF) {
		new_val = val & ~MAX77759_GPIO5_DIR_MASK;
		new_val |= MAX77759_GPIO5_DIR(MAX77759_GPIO_DIR_IN);
	} else { /* MAX77759_GPIO6_OFF */
		new_val = val & ~MAX77759_GPIO6_DIR_MASK;
		new_val |= MAX77759_GPIO6_DIR(MAX77759_GPIO_DIR_IN);
	}

	if (new_val != val) {
		rc = maxq_gpio_control_write(data->maxq, new_val);
		if (rc < 0) {
			dev_err(data->dev, "opcode write 0x24 failed\n");
			return rc;
		}
	}

	return 0;
}

static int max77759_gpio_direction_output(struct gpio_chip *chip,
					 unsigned int offset, int value)
{
	struct max77729_pmic_data *data = gpiochip_get_data(chip);
	int rc;
	uint8_t val;
	uint8_t new_val;

	if ((offset < MAX77759_MIN_GPIO_OFF) || (offset > MAX77759_MAX_GPIO_OFF))
		return -EINVAL;

	rc = maxq_gpio_control_read(data->maxq, &val);
	if (rc < 0) {
		dev_err(data->dev, "opcode read 0x23 failed\n");
		return rc;
	}

	if (offset == MAX77759_GPIO5_OFF) {
		new_val = val & ~MAX77759_GPIO5_DIR_MASK;
		new_val |= MAX77759_GPIO5_DIR(MAX77759_GPIO_DIR_OUT);
	} else { /* MAX77759_GPIO6_OFF */
		new_val = val & ~MAX77759_GPIO6_DIR_MASK;
		new_val |= MAX77759_GPIO6_DIR(MAX77759_GPIO_DIR_OUT);
	}

	if (new_val != val) {
		rc = maxq_gpio_control_write(data->maxq, new_val);
		if (rc < 0) {
			dev_err(data->dev, "opcode write 0x24 failed\n");
			return rc;
		}
	}

	return 0;
}
#endif

static int max77729_pmic_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max77729_pmic_data *data;
	int irq_gpio, pmic_id, ret =0;

	pmic_id = max77729_pmic_read_id(client);
	if (pmic_id < 0)
		return -ENODEV;
	if (pmic_id == MAX77759_PMIC_PMIC_ID_MW)
		max777x9_pmic_regmap_cfg.max_register = 0xe0;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->pmic_id = pmic_id;
	data->batt_id = -1;
	mutex_init(&data->io_lock);
	atomic_set(&data->sysuvlo_cnt, 0);
	atomic_set(&data->sysovlo_cnt, 0);
	i2c_set_clientdata(client, data);

	data->regmap = devm_regmap_init_i2c(client, &max777x9_pmic_regmap_cfg);
	if (IS_ERR(data->regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	irq_gpio = of_get_named_gpio(dev->of_node, "max777x9,irq-gpio", 0);
	if (irq_gpio < 0) {
		dev_err(dev, "irq is not defined\n");
	} else {
		client->irq = gpio_to_irq(irq_gpio);

		ret = devm_request_threaded_irq(data->dev, client->irq, NULL,
						max777x9_pmic_irq,
						IRQF_TRIGGER_LOW |
						IRQF_SHARED |
						IRQF_ONESHOT,
						"max777x9_pmic",
						data);
		if (ret < 0) {
			dev_err(dev, "failed get irq thread\n");
		} else {
			/* force clear pending before unmasking */
			max777x9_pmic_irq(0, data);

			ret = max777x9_pmic_set_irqmask(data);
			if (ret < 0)
				dev_err(dev, "failed to apply irq mask\n");
		}
	}

#ifdef CONFIG_GPIOLIB
	if (pmic_id == MAX77759_PMIC_PMIC_ID_MW) {
		/* Setup GPIO cotroller */
		data->gpio.owner = THIS_MODULE;
		data->gpio.parent = dev;
		data->gpio.label = "max777x9_gpio";
		data->gpio.get_direction = max77759_gpio_get_direction;
		data->gpio.direction_input = max77759_gpio_direction_input;
		data->gpio.direction_output = max77759_gpio_direction_output;
		data->gpio.get = max77759_gpio_get;
		data->gpio.set = max77759_gpio_set;
		data->gpio.base	= -1;
		data->gpio.ngpio = MAX77759_NUM_GPIOS;
		data->gpio.can_sleep = true;
		data->gpio.of_node = of_find_node_by_name(dev->of_node,
				data->gpio.label);
		if (!data->gpio.of_node)
			dev_err(dev, "Failed to find %s DT node\n",
				data->gpio.label);

		ret = devm_gpiochip_add_data(dev, &data->gpio, data);
		if (ret)
			dev_err(dev, "Failed to initialize gpio chip\n");
	}
#endif

	ret = dbg_init_fs(data);
	if (ret < 0)
		dev_err(dev, "Failed to initialize debug fs\n");

	if (pmic_id == MAX77759_PMIC_PMIC_ID_MW) {
		const int poll_en = of_property_read_bool(dev->of_node,
							  "goog,maxq-poll");
		data->maxq = maxq_init(dev, data->regmap, poll_en);
		if (IS_ERR_OR_NULL(data->maxq)) {
			dev_err(dev, "Maxq init failed!\n");
			ret = PTR_ERR(data->maxq);
		}
	}

	dev_info(dev, "probe_done pmic_id = %x\n", pmic_id);
	return ret;
}

static int max77729_pmic_remove(struct i2c_client *client)
{
	struct max77729_pmic_data *data = i2c_get_clientdata(client);

	maxq_remove(data->maxq);
	return 0;
}

static const struct of_device_id max77729_pmic_of_match_table[] = {
	{ .compatible = "maxim,max77729pmic" },
	{ .compatible = "maxim,max77759pmic" },
	{},
};
MODULE_DEVICE_TABLE(of, max77729_pmic_of_match_table);

static const struct i2c_device_id max77729_pmic_id[] = {
	{"max77729_pmic", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77729_pmic_id);

static struct i2c_driver max77729_pmic_i2c_driver = {
	.driver = {
		.name = "max777x9-pmic",
		.owner = THIS_MODULE,
		.of_match_table = max77729_pmic_of_match_table,
	},
	.id_table = max77729_pmic_id,
	.probe = max77729_pmic_probe,
	.remove = max77729_pmic_remove,
};

module_i2c_driver(max77729_pmic_i2c_driver);
MODULE_DESCRIPTION("Maxim 77729 PMIC driver");
MODULE_LICENSE("GPL");
