// SPDX-License-Identifier: GPL-2.0-only
/******************** (C) COPYRIGHT 2010-2026 STMicroelectronics ***************
*
* File Name          : lis2dw12_i2c.c
* Authors            : AMG - Application Team
*		     : Mario Tesi <mario.tesi@st.com>
*		     : Giuseppe Barba <giuseppe.barba@st.com>
*		     : Author is willing to be considered the contact and update
*		     : point for the driver.
* Version            : V.1.0
* Date               : 2016/Oct/18
* Description        : LIS2DW12 driver
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************/

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/version.h>

#include "lis2dw12_core.h"

static int lis2dw12_i2c_read(struct lis2dw12_data *cdata,
			     u8 reg_addr, int len,
			     u8 *data, bool b_lock)
{
	int err;
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(cdata->dev);

	if (!data) {
		dev_err(cdata->dev, "Invalid I2C read: NULL data buffer\n");
		return -EINVAL;
	}

	if (len <= 0 || len > LIS2DW12_RX_MAX_LENGTH) {
		dev_err(cdata->dev, "Invalid I2C read length: %d\n", len);
		return -EINVAL;
	}

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = cdata->tb.tx_buf;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = cdata->tb.rx_buf;

	if (b_lock)
		mutex_lock(&cdata->bank_registers_lock);

	mutex_lock(&cdata->tb.buf_lock);

	cdata->tb.tx_buf[0] = reg_addr;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err == 2)
		memcpy(data, cdata->tb.rx_buf, len);

	mutex_unlock(&cdata->tb.buf_lock);

	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	if (err != 2)
		return err < 0 ? err : -EIO;

	return 0;
}

static int lis2dw12_i2c_write(struct lis2dw12_data *cdata,
			      u8 reg_addr, int len,
			      const u8 *data, bool b_lock)
{
	int err;
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(cdata->dev);

	if (!data) {
		dev_err(cdata->dev, "Invalid I2C write: NULL data buffer\n");
		return -EINVAL;
	}

	if (len <= 0 || len >= LIS2DW12_TX_MAX_LENGTH) {
		dev_err(cdata->dev, "Invalid I2C write length: %d\n", len);
		return -EINVAL;
	}

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len + 1;
	msg.buf = cdata->tb.tx_buf;

	if (b_lock)
		mutex_lock(&cdata->bank_registers_lock);

	mutex_lock(&cdata->tb.buf_lock);

	cdata->tb.tx_buf[0] = reg_addr;
	memcpy(&cdata->tb.tx_buf[1], data, len);

	err = i2c_transfer(client->adapter, &msg, 1);

	mutex_unlock(&cdata->tb.buf_lock);

	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	if (err != 1)
		return err < 0 ? err : -EIO;

	return 0;
}

static const struct lis2dw12_transfer_function lis2dw12_tf_i2c = {
	.write = lis2dw12_i2c_write,
	.read = lis2dw12_i2c_read,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static int lis2dw12_i2c_probe(struct i2c_client *client)
#else /* LINUX_VERSION_CODE */
static int lis2dw12_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
#endif /* LINUX_VERSION_CODE */
{
	int err;
	struct lis2dw12_data *cdata;

	cdata = devm_kzalloc(&client->dev, sizeof(struct lis2dw12_data),
			     GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->tf = &lis2dw12_tf_i2c;
	i2c_set_clientdata(client, cdata);

	err = lis2dw12_common_probe(cdata, client->irq, BUS_I2C);
	if (err < 0)
		return err;

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void lis2dw12_i2c_remove(struct i2c_client *client)
{
	struct lis2dw12_data *cdata = i2c_get_clientdata(client);

	lis2dw12_common_remove(cdata, client->irq);
}
#else
static int lis2dw12_i2c_remove(struct i2c_client *client)
{
	struct lis2dw12_data *cdata = i2c_get_clientdata(client);

	lis2dw12_common_remove(cdata, client->irq);

	return 0;
}
#endif /* LINUX_VERSION_CODE */

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int lis2dw12_suspend(struct device *dev)
{
	struct lis2dw12_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lis2dw12_common_suspend(cdata);
}

static int lis2dw12_resume(struct device *dev)
{
	struct lis2dw12_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lis2dw12_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(lis2dw12_pm_ops, lis2dw12_suspend, lis2dw12_resume);

#define LIS2DW12_PM_OPS	(&lis2dw12_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LIS2DW12_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */


static const struct i2c_device_id lis2dw12_ids[] = {
	{ LIS2DW12_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lis2dw12_ids);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id lis2dw12_id_table[] = {
	{ .compatible = "st,lis2dw12", },
	{ },
};
MODULE_DEVICE_TABLE(of, lis2dw12_id_table);
#endif	/* CONFIG_OF */

static struct i2c_driver lis2dw12_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS2DW12_DEV_NAME,
		.pm = LIS2DW12_PM_OPS,
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = lis2dw12_id_table,
#endif
	},
	.probe    = lis2dw12_i2c_probe,
	.remove   = lis2dw12_i2c_remove,
	.id_table = lis2dw12_ids,
};

module_i2c_driver(lis2dw12_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lis2dw12 i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
