/*
 * MAXIM max9286/max9288 GMSL driver include file
 *
 * Copyright (C) 2015-2019 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _MAX92XX_H
#define _MAX92XX_H

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
#define MAX9288_ID		0x2A
#define MAX9290_ID		0x2C
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

static inline int reg16_read_n(struct i2c_client *client, u16 reg, u8 *val, int n)
{
	int ret, retries;
	u8 buf[2] = {reg >> 8, reg & 0xff};

	for (retries = REG16_NUM_RETRIES; retries; retries--) {
		ret = i2c_master_send(client, buf, 2);
		if (ret == 2) {
			ret = i2c_master_recv(client, val, n);
			if (ret == n)
				break;
		}
	}

	if (ret < 0) {
		dev_dbg(&client->dev,
			"read fail: chip 0x%x registers 0x%x-0x%x: %d\n",
			client->addr, reg, reg + n, ret);
	}

	return ret < 0 ? ret : 0;
}

static inline int reg16_write_n(struct i2c_client *client, u16 reg, const u8* val, int n)
{
	int ret, retries;
	u8 buf[2 + n];

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	memcpy(&buf[2], val, n);

	for (retries = REG16_NUM_RETRIES; retries; retries--) {
		ret = i2c_master_send(client, buf, 2 + n);
		if (ret == 2 + n)
			break;
	}

	if (ret < 0) {
		dev_dbg(&client->dev,
			"write fail: chip 0x%x register 0x%x-0x%x: %d\n",
			client->addr, reg, reg + n, ret);
	} else {
#ifndef WRITE_VERIFY
		u8 val2[n];
		ret = reg16_read_n(client, reg, val2, n);
		if (ret < 0)
			return ret;

		if (memcmp(val, val2, n)) {
			dev_err(&client->dev,
				"write verify mismatch: chip 0x%x reg=0x%x-0x%x "
				"'%*phN'->'%*phN'\n", client->addr, reg, reg + n,
				n, val, n, val2);
				ret = -EBADE;
		}
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
#endif /* _MAX92XX_H */
