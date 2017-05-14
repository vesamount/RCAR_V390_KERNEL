/*
 * TI FPDLinkIII driver include file
 *
 * Copyright (C) 2017 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _TI9X4_H
#define _TI9X4_H

//#define DEBUG
#ifdef DEBUG
#undef dev_dbg
#define dev_dbg dev_info
#endif

#define MAXIM_NUM_RETRIES	1 /* number of read/write retries */
#define TI913_ID		0x58
#define TI953_ID		0x30 /* or starapped to 0x32 */
#define TI9X4_ID		0x00 /* strapped */
#define BROADCAST		0x6f

static inline int reg8_read(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret, retries;

	for (retries = MAXIM_NUM_RETRIES; retries; retries--) {
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

	for (retries = MAXIM_NUM_RETRIES; retries; retries--) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (!(ret < 0))
			break;
	}

	if (ret < 0) {
		dev_dbg(&client->dev,
			"write fail: chip 0x%x register 0x%x: %d\n",
			client->addr, reg, ret);
	}

	return ret < 0 ? ret : 0;
}

static inline int reg16_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret, retries;
	u8 buf[2] = {reg >> 8, reg & 0xff};

	for (retries = MAXIM_NUM_RETRIES; retries; retries--) {
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

	for (retries = MAXIM_NUM_RETRIES; retries; retries--) {
		ret = i2c_master_send(client, buf, 3);
		if (ret == 3)
			break;
	}

	if (ret < 0) {
		dev_dbg(&client->dev,
			"write fail: chip 0x%x register 0x%x: %d\n",
			client->addr, reg, ret);
	}

	return ret < 0 ? ret : 0;
}

static inline int reg16_read16(struct i2c_client *client, u16 reg, u16 *val)
{
	int ret, retries;
	u8 buf[2] = {reg >> 8, reg & 0xff};

	for (retries = MAXIM_NUM_RETRIES; retries; retries--) {
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

	for (retries = MAXIM_NUM_RETRIES; retries; retries--) {
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
#endif /* _TI9X4_H */
