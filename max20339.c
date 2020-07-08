// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020, Google Inc
 *
 * MAX20339 OVP and LS driver
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#define MAX20339_OVLOSEL			0x11
#define MAX20339_OVLOSEL_INOVLOSEL_14_5		0x2
#define MAX20339_IN_CTR				0x10

#define MAX20339_POLL_ATTEMPTS			10
#define MAX20339_INT2_REG			0x5
#define MAX20339_INT2_LSW1CLOSEDI		(1 << 0)
#define MAX20339_INT3_REG			0x6
#define MAX20339_INT3_LSW2CLOSEDI		(1 << 0)

#define MAX20339_SW_CNTL_REG			0xA
#define MAX20339_SW_CNTL_LSW1_EN_SHIFT		0
#define MAX20339_SW_CNTL_LSW1_EN_MASK		0x1
#define MAX20339_SW_CNTL_LSW1_OV_SHIFT		1
#define MAX20338_SW_CNTL_LSW1_OV_EN_MASK	0x2
#define MAX20339_SW_CNTL_LSW2_EN_SHIFT		4
#define MAX20339_SW_CNTL_LSW2_EN_MASK		0x10
#define MAX20338_SW_CNTL_LSW2_OV_EN_SHIFT	5
#define MAX20338_SW_CNTL_LSW2_OV_EN_MASK	0x20

#define MAX20339_MIN_GPIO			0
#define MAX20339_MAX_GPIO			1
#define MAX20339_NUM_GPIOS			2
#define MAX20339_LSW1_OFF			0
#define MAX20339_LSW2_OFF			1

struct max20339_ovp {
	struct i2c_client *client;
	struct regmap *regmap;
#ifdef CONFIG_GPIOLIB
	struct gpio_chip gpio;
#endif
};

static const struct regmap_range max20339_ovp_range[] = {
	regmap_reg_range(0x0, 0x2f)
};

const struct regmap_access_table max20339_ovp_write_table = {
	.yes_ranges = max20339_ovp_range,
	.n_yes_ranges = ARRAY_SIZE(max20339_ovp_range),
};

static const struct regmap_config max20339_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x2f,
	.wr_table = &max20339_ovp_write_table,
};

static int max20339_init_regs(struct regmap *regmap, struct device *dev)
{
	int ret;
	unsigned int val;

	ret = regmap_read(regmap, MAX20339_OVLOSEL, &val);
	if (ret < 0) {
		dev_err(dev, "OVLSEL read error: ret %d\n", ret);
		return ret;
	}

	dev_info(dev, "OVLOSEL default: %#x\n", val);

	ret = regmap_write(regmap, MAX20339_OVLOSEL,
			 MAX20339_OVLOSEL_INOVLOSEL_14_5);
	if (ret < 0) {
		dev_err(dev, "OVLSEL write error: ret %d\n", ret);
		return ret;
	}

	ret = regmap_read(regmap, MAX20339_IN_CTR, &val);
	if (ret < 0) {
		dev_err(dev, "IN_CTR read error: ret %d\n", ret);
		return ret;
	}

	dev_info(dev, "IN_CTR default: %#x\n", val);

	/* Disable & enable to make OVLOSEL reflect */
	ret = regmap_write(regmap, MAX20339_IN_CTR, 0);
	if (ret < 0) {
		dev_err(dev, "IN_CTR write error: ret %d\n", ret);
		return ret;
	}

	ret = regmap_write(regmap, MAX20339_IN_CTR, val);
	if (ret < 0) {
		dev_err(dev, "IN_CTR write error: ret %d\n", ret);
		return ret;
	}

	return ret;
}

#ifdef CONFIG_GPIOLIB
static int max20339_gpio_get_direction(struct gpio_chip *chip,
				       unsigned int offset)
{
	return GPIOF_DIR_OUT;
}

static int max20339_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	int ret;
	unsigned int val;
	u8 mask;
	u8 shift;
	struct max20339_ovp *ovp = gpiochip_get_data(chip);

	switch (offset) {
	case MAX20339_LSW1_OFF:
		mask = MAX20339_SW_CNTL_LSW1_EN_MASK;
		shift = MAX20339_SW_CNTL_LSW1_EN_SHIFT;
		break;
	case MAX20339_LSW2_OFF:
		mask = MAX20339_SW_CNTL_LSW2_EN_MASK;
		shift = MAX20339_SW_CNTL_LSW2_EN_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_read(ovp->regmap, MAX20339_SW_CNTL_REG, &val);
	if (ret < 0) {
		dev_err(&ovp->client->dev, "SW_CNTL read error: ret %d\n", ret);
		return ret;
	}

	return (val & mask) >> shift;
}

static void max20339_gpio_set(struct gpio_chip *chip,
			      unsigned int offset, int value)
{
	int ret;
	unsigned int tmp;
	bool change;
	u8 mask;
	u8 shift;
	u8 int_reg; /* regulator to poll for update */
	u8 closed_fld; /* field to read for closed change */
	int i;
	struct max20339_ovp *ovp = gpiochip_get_data(chip);

	switch (offset) {
	case MAX20339_LSW1_OFF:
		mask = MAX20339_SW_CNTL_LSW1_EN_MASK;
		shift = MAX20339_SW_CNTL_LSW1_EN_SHIFT;
		int_reg = MAX20339_INT2_REG;
		closed_fld = MAX20339_INT2_LSW1CLOSEDI;
		break;
	case MAX20339_LSW2_OFF:
		mask = MAX20339_SW_CNTL_LSW2_EN_MASK;
		shift = MAX20339_SW_CNTL_LSW2_EN_SHIFT;
		int_reg = MAX20339_INT3_REG;
		closed_fld = MAX20339_INT3_LSW2CLOSEDI;
		break;
	default:
		return;
	}

	tmp = (!!value << shift);
	ret = regmap_update_bits_base(ovp->regmap, MAX20339_SW_CNTL_REG, mask,  tmp,
				      &change, false, false);
	if (ret < 0)
		dev_err(&ovp->client->dev, "SW_CNTL update error: ret %d\n", ret);

	/* poll until update seen */
	for (i = 0; i < MAX20339_POLL_ATTEMPTS; i++) {
		ret = regmap_read(ovp->regmap, int_reg, &tmp);
		if (tmp & closed_fld)
			break;
		mdelay(20);
	}

}
#endif

static int max20339_probe(struct i2c_client *client,
			  const struct i2c_device_id *i2c_id)
{
	struct max20339_ovp *ovp;
	int ret = 0;

	ovp = devm_kzalloc(&client->dev, sizeof(*ovp), GFP_KERNEL);
	if (!ovp)
		return -ENOMEM;

	ovp->client = client;
	ovp->regmap = devm_regmap_init_i2c(client,
					   &max20339_regmap_config);
	if (IS_ERR(ovp->regmap)) {
		dev_err(&client->dev, "Regmap init failed\n");
		return PTR_ERR(ovp->regmap);
	}

	max20339_init_regs(ovp->regmap, &client->dev);

#ifdef CONFIG_GPIOLIB
	/* Setup GPIO cotroller */
	ovp->gpio.owner = THIS_MODULE;
	ovp->gpio.parent = &client->dev;
	ovp->gpio.label = "max20339_gpio";
	ovp->gpio.get_direction = max20339_gpio_get_direction;
	ovp->gpio.get = max20339_gpio_get;
	ovp->gpio.set = max20339_gpio_set;
	ovp->gpio.base = -1;
	ovp->gpio.ngpio = MAX20339_NUM_GPIOS;
	ovp->gpio.can_sleep = true;
	ovp->gpio.of_node = of_find_node_by_name(client->dev.of_node,
						 ovp->gpio.label);
	if (!ovp->gpio.of_node)
		dev_err(&client->dev, "Failed to find %s DT node\n",
			ovp->gpio.label);

	ret = devm_gpiochip_add_data(&client->dev, &ovp->gpio, ovp);
	if (ret)
		dev_err(&client->dev, "Failed to initialize gpio chip\n");
#endif

	return ret;
}

static int max20339_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id max20339_id[] = {
	{ "max20339ovp", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max20339_id);

#ifdef CONFIG_OF
static const struct of_device_id max20339_of_match[] = {
	{ .compatible = "max20339ovp", },
	{},
};
MODULE_DEVICE_TABLE(of, max20339_of_match);
#endif

static struct i2c_driver max20339_i2c_driver = {
	.driver = {
		.name = "max20339ovp",
		.of_match_table = of_match_ptr(max20339_of_match),
	},
	.probe = max20339_probe,
	.remove = max20339_remove,
	.id_table = max20339_id,
};
module_i2c_driver(max20339_i2c_driver);

MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
