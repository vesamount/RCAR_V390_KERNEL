/*
 * Driver for TI TUSB8041 USB SSIC 4-port 3.0 hub controller driver
 * Based on tusb8041 and usb3503 drivers
 *
 * Copyright (c) 2012-2013 Dongjin Kim (tobetter@gmail.com)
 * Copyright (c) 2016 Linaro Ltd.
 * Copyright (c) 2018 Andrey Gusakov (andrey.gusakov@cogentembedded.com)
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

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>

enum tusb8041_mode {
	TUSB8041_MODE_UNKNOWN,
	TUSB8041_MODE_HUB,
	TUSB8041_MODE_STANDBY,
};

struct tusb8041 {
	enum tusb8041_mode	mode;
	u8			*regs;
	int			regs_sz;
	struct device		*dev;
	struct gpio_desc	*gpio_reset;
};

static void tusb8041_reset(struct tusb8041 *hub, int state)
{
	gpiod_set_value_cansleep(hub->gpio_reset, state);
}

static int tusb8041_connect(struct tusb8041 *hub)
{
	struct device *dev = hub->dev;
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	int retry = 100;

	tusb8041_reset(hub, 1);

	/* reset and load defaults */
	ret = i2c_smbus_write_byte_data(client, 0xf8, 0x03);
	if (ret < 0)
		goto err;

	/* wait reset finished */
	do {
		/* check reset bit cleared */
		ret = i2c_smbus_read_byte_data(client, 0xf8);
		if (ret >= 0) {
			if (ret & 0x02)
				ret = -EIO;
			else
				ret = 0;
		}
	} while ((retry--) && (ret < 0));
	if (ret < 0)
		goto err;

	/* write settings */
	if ((hub->regs) && (hub->regs_sz)) {
		int i;

		for (i = 0; i < hub->regs_sz; i += 2) {
			do {
				ret = i2c_master_send(client, &(hub->regs[i]), 2);
			} while ((retry--) && (ret < 0));
		}
		if (ret < 0)
			goto err;
	}

	/* attach */
	do {
		ret = i2c_smbus_write_byte_data(client, 0xf8, 0x01);
		if (ret >= 0) {
			/* check bit cleared */
			ret = i2c_smbus_read_byte_data(client, 0xf8);
			if (ret >= 0) {
				if (ret & 0x01)
					ret = -EIO;
				else
					ret = 0;
			}
		}
	} while ((retry--) && (ret < 0));

	if (ret < 0)
		goto err;

	hub->mode = TUSB8041_MODE_HUB;
	dev_dbg(dev, "switched to HUB mode\n");
	return 0;

err:
	dev_err(dev, "Failed to write settings: %d\n", ret);
	tusb8041_reset(hub, 0);
	return ret;
}

static int tusb8041_switch_mode(struct tusb8041 *hub, enum tusb8041_mode mode)
{
	struct device *dev = hub->dev;
	int err = 0;

	switch (mode) {
	case TUSB8041_MODE_HUB:
		err = tusb8041_connect(hub);
		break;

	case TUSB8041_MODE_STANDBY:
		tusb8041_reset(hub, 0);
		dev_dbg(dev, "switched to STANDBY mode\n");
		break;

	default:
		dev_err(dev, "unknown mode is requested\n");
		err = -EINVAL;
		break;
	}

	return err;
}

static int tusb8041_probe(struct tusb8041 *hub)
{
	struct device *dev = hub->dev;
	struct device_node *np = dev->of_node;
	struct gpio_desc *gpio;
	int ret;
	u32 mode = TUSB8041_MODE_HUB;
	int sz;

	gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);
	hub->gpio_reset = gpio;

	if (of_property_read_u32(np, "initial-mode", &hub->mode))
		hub->mode = mode;

	if (of_find_property(np, "ti,registers", &sz) && ((sz & 0x01) == 0)) {
		hub->regs = devm_kzalloc(hub->dev, sz, GFP_KERNEL);
		ret = of_property_read_u8_array(np, "ti,registers", hub->regs, sz);
		if (ret == 0)
			hub->regs_sz = sz;
	}

	/* reset */
	tusb8041_reset(hub, 0);
	msleep(250);

	return tusb8041_switch_mode(hub, hub->mode);
}

static int tusb8041_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct tusb8041 *hub;

	hub = devm_kzalloc(&i2c->dev, sizeof(*hub), GFP_KERNEL);
	if (!hub)
		return -ENOMEM;

	i2c_set_clientdata(i2c, hub);
	hub->dev = &i2c->dev;

	return tusb8041_probe(hub);
}

#ifdef CONFIG_PM_SLEEP
static int tusb8041_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tusb8041 *hub = i2c_get_clientdata(client);

	tusb8041_switch_mode(hub, TUSB8041_MODE_STANDBY);

	return 0;
}

static int tusb8041_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tusb8041 *hub = i2c_get_clientdata(client);

	tusb8041_switch_mode(hub, hub->mode);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(tusb8041_i2c_pm_ops, tusb8041_i2c_suspend,
		tusb8041_i2c_resume);

static const struct i2c_device_id tusb8041_id[] = {
	{ "tusb8041", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tusb8041_id);

#ifdef CONFIG_OF
static const struct of_device_id tusb8041_of_match[] = {
	{ .compatible = "ti,tusb8041" },
	{}
};
MODULE_DEVICE_TABLE(of, tusb8041_of_match);
#endif

static struct i2c_driver tusb8041_i2c_driver = {
	.driver = {
		.name = "tusb8041",
		.pm = &tusb8041_i2c_pm_ops,
		.of_match_table = of_match_ptr(tusb8041_of_match),
	},
	.probe		= tusb8041_i2c_probe,
	.id_table	= tusb8041_id,
};
module_i2c_driver(tusb8041_i2c_driver);

MODULE_DESCRIPTION("TUSB8041 USB HUB driver");
MODULE_LICENSE("GPL v2");
