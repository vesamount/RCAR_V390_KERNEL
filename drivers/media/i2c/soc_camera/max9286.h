/*
 * MAXIM max9286-max9271 GMSL driver include file
 *
 * Copyright (C) 2015-2017 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _MAX9286_MAX9271_H
#define _MAX9286_MAX9271_H

//#define DEBUG
#ifdef DEBUG
//#define WRITE_VERIFY
#define MAXIM_DUMP
#undef dev_dbg
#define dev_dbg dev_info
#endif

#define REG8_NUM_RETRIES	1 /* number of read/write retries */
#define REG16_NUM_RETRIES	10 /* number of read/write retries */
#define MAX9271_ID		0x9
#define MAX96705_ID		0x41
#define MAX96707_ID		0x45 /* MAX96715: there is no HS pin */
#define MAX9286_ID		0x40
#define BROADCAST		0x6f

static inline int reg8_read(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret, retries;

	for (retries = REG8_NUM_RETRIES; retries; retries--) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (!(ret < 0))
			break;
	}

	if (ret < 0) {
		dev_dbg(&client->dev,
			"read fail: chip 0x%x register 0x%x: %d\n",
			client->addr, reg, ret);
	} else {
		*val = ret;
	}

	return ret < 0 ? ret : 0;
}

static inline int reg8_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret, retries;

	for (retries = REG8_NUM_RETRIES; retries; retries--) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (!(ret < 0))
			break;
	}

	if (ret < 0) {
		dev_dbg(&client->dev,
			"write fail: chip 0x%x register 0x%x: %d\n",
			client->addr, reg, ret);
	} else {
#ifdef WRITE_VERIFY
		u8 val2;
		reg8_read(client, reg, &val2);
		if (val != val2)
			dev_err(&client->dev,
				"write verify mismatch: chip 0x%x reg=0x%x "
				"0x%x->0x%x\n", client->addr, reg, val, val2);
#endif
	}

	return ret < 0 ? ret : 0;
}

static inline int reg16_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret, retries;
	u8 buf[2] = {reg >> 8, reg & 0xff};

	for (retries = REG16_NUM_RETRIES; retries; retries--) {
		ret = i2c_master_send(client, buf, 2);
		if (ret == 2) {
			ret = i2c_master_recv(client, buf, 1);
			if (ret == 1)
				break;
		}
	}

	if (ret < 0) {
		dev_dbg(&client->dev,
			"read fail: chip 0x%x register 0x%x: %d\n",
			client->addr, reg, ret);
	} else {
		*val = buf[0];
	}

	return ret < 0 ? ret : 0;
}

static inline int reg16_write(struct i2c_client *client, u16 reg, u8 val)
{
	int ret, retries;
	u8 buf[3] = {reg >> 8, reg & 0xff, val};

	for (retries = REG16_NUM_RETRIES; retries; retries--) {
		ret = i2c_master_send(client, buf, 3);
		if (ret == 3)
			break;
	}

	if (ret < 0) {
		dev_dbg(&client->dev,
			"write fail: chip 0x%x register 0x%x: %d\n",
			client->addr, reg, ret);
	} else {
#ifdef WRITE_VERIFY
		u8 val2;
		reg16_read(client, reg, &val2);
		if (val != val2)
			dev_err(&client->dev,
				"write verify mismatch: chip 0x%x reg=0x%x "
				"0x%x->0x%x\n", client->addr, reg, val, val2);
#endif
	}

	return ret < 0 ? ret : 0;
}


static inline int reg16_read16(struct i2c_client *client, u16 reg, u16 *val)
{
	int ret, retries;
	u8 buf[2] = {reg >> 8, reg & 0xff};

	for (retries = REG8_NUM_RETRIES; retries; retries--) {
		ret = i2c_master_send(client, buf, 2);
		if (ret == 2) {
			ret = i2c_master_recv(client, buf, 2);
			if (ret == 2)
				break;
		}
	}

	if (ret < 0) {
		dev_err(&client->dev,
			"read fail: chip 0x%x register 0x%x: %d\n",
			client->addr, reg, ret);
	} else {
		*val = ((u16)buf[0] << 8) | buf[1];
	}

	return ret < 0 ? ret : 0;
}

static inline int reg16_write16(struct i2c_client *client, u16 reg, u16 val)
{
	int ret, retries;
	u8 buf[4] = {reg >> 8, reg & 0xff, val >> 8, val & 0xff};

	for (retries = REG8_NUM_RETRIES; retries; retries--) {
		ret = i2c_master_send(client, buf, 4);
		if (ret == 4)
			break;
	}

	if (ret < 0) {
		dev_err(&client->dev,
			"write fail: chip 0x%x register 0x%x: %d\n",
			client->addr, reg, ret);
	}

	return ret < 0 ? ret : 0;
}


#ifdef MAXIM_DUMP
static void maxim_ovsensor_dump_regs(struct i2c_client *client)
{
	int ret, i;
	u8 val = 0;
	u16 regs[] = {0x300a, 0x300b, 0x300c};

	dev_dbg(&client->dev, "dump regs 0x%x\n", client->addr);

	for (i = 0; i < sizeof(regs) / 2; i++) {
		ret = reg16_read(client, regs[i], &val);
		if (ret < 0)
			dev_err(&client->dev,
				"read fail: chip 0x%x register 0x%02x: %d\n",
				client->addr, regs[i], ret);
		printk("0x%02x -> 0x%x\n", regs[i], val);
	}
}

static void maxim_ov10635_dump_format_regs(struct i2c_client *client)
{
	int ret, i;
	u8 val;
	u16 regs[] = {0x3003, 0x3004, 0x4300,
		      0x4605, 0x3621, 0x3702, 0x3703, 0x3704,
		      0x3802, 0x3803, 0x3806, 0x3807, 0x3808, 0x3809, 0x380a,
		      0x380b, 0x380c, 0x380d, 0x380e, 0x380f,
		      0x4606, 0x4607, 0x460a, 0x460b,
		      0xc488, 0xc489, 0xc48a, 0xc48b,
		      0xc4cc, 0xc4cd, 0xc4ce, 0xc4cf, 0xc512, 0xc513,
		      0xc518, 0xc519, 0xc51a, 0xc51b,
	};

	dev_dbg(&client->dev, "dump regs 0x%x\n", client->addr);

	for (i = 0; i < sizeof(regs) / 2; i++) {
		ret = reg16_read(client, regs[i], &val);
		if (ret < 0)
			dev_err(&client->dev,
				"read fail: chip 0x%x register 0x%02x: %d\n",
				client->addr, regs[i], ret);
		printk("0x%02x -> 0x%x\n", regs[i], val);
	}
}

static void maxim_max927x_dump_regs(struct i2c_client *client)
{
	int ret;
	u8 reg;

	dev_dbg(&client->dev, "dump regs 0x%x\n", client->addr);

	for (reg = 0; reg < 0x20; reg++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0)
			dev_err(&client->dev,
				"read fail: chip 0x%x register 0x%x: %d\n",
				client->addr, reg, ret);
		printk("0x%02x ", ret);
		if (((reg + 1) % 0x10) == 0)
			printk("\n");
	}
}
#endif /* MAXIM_DUMP */
#endif /* _MAX9286_MAX9271_H */
