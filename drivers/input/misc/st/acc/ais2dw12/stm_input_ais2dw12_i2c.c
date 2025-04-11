// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics ais2dw12 i2c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2025 STMicroelectronics Inc.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/version.h>

#include "stm_input_ais2dw12_core.h"

static int ais2dw12_input_i2c_read(struct ais2dw12_input_data *cdata,
				   u8 reg_addr, int len, u8 *data, bool b_lock)
{
	int err = 0;
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(cdata->dev);

	if (len > AIS2DW12_I2C_MAX_BUFFER_SIZE) {
		dev_err(cdata->dev, "I2C buffer size exceeded\n");
		return -EINVAL;
	}

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	if (b_lock) {
		mutex_lock(&cdata->bank_registers_lock);
		err = i2c_transfer(client->adapter, msg, 2);
		mutex_unlock(&cdata->bank_registers_lock);
	} else {
		err = i2c_transfer(client->adapter, msg, 2);
	}

	return err;
}

static int ais2dw12_input_i2c_write(struct ais2dw12_input_data *cdata,
				    u8 reg_addr, int len, u8 *data,
				    bool b_lock)
{
	int err = 0;
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(cdata->dev);

	if (len > AIS2DW12_I2C_MAX_BUFFER_SIZE) {
		dev_err(cdata->dev, "I2C buffer size exceeded\n");
		return -EINVAL;
	}

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	if (b_lock) {
		mutex_lock(&cdata->bank_registers_lock);
		err = i2c_transfer(client->adapter, &msg, 1);
		mutex_unlock(&cdata->bank_registers_lock);
	} else {
		err = i2c_transfer(client->adapter, &msg, 1);
	}

	return err;
}

static const struct ais2dw12_input_transfer_function ais2dw12_input_tf_i2c = {
	.write = ais2dw12_input_i2c_write,
	.read = ais2dw12_input_i2c_read,
};

#if KERNEL_VERSION(6, 2, 0) <= LINUX_VERSION_CODE
static int ais2dw12_input_i2c_probe(struct i2c_client *client)
#else	/* LINUX_VERSION_CODE */
static int ais2dw12_input_i2c_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
#endif	/* LINUX_VERSION_CODE */
{
	int err;
	struct ais2dw12_input_data *cdata;

	cdata = devm_kzalloc(&client->dev, sizeof(struct ais2dw12_input_data),
			     GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->tf = &ais2dw12_input_tf_i2c;
	i2c_set_clientdata(client, cdata);

	err = ais2dw12_input_common_probe(cdata, client->irq, BUS_I2C);
	if (err < 0)
		return err;

	return 0;
}

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
static void ais2dw12_input_i2c_remove(struct i2c_client *client)
{
	struct ais2dw12_input_data *cdata = i2c_get_clientdata(client);

	ais2dw12_input_common_remove(cdata, client->irq);
}
#else
static int ais2dw12_input_i2c_remove(struct i2c_client *client)
{
	struct ais2dw12_input_data *cdata = i2c_get_clientdata(client);

	ais2dw12_input_common_remove(cdata, client->irq);

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int ais2dw12_input_suspend(struct device *dev)
{
	struct ais2dw12_input_data *cdata =
		i2c_get_clientdata(to_i2c_client(dev));

	return ais2dw12_input_common_suspend(cdata);
}

static int ais2dw12_input_resume(struct device *dev)
{
	struct ais2dw12_input_data *cdata =
		i2c_get_clientdata(to_i2c_client(dev));

	return ais2dw12_input_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(ais2dw12_input_pm_ops, ais2dw12_input_suspend,
			 ais2dw12_input_resume);

#define AIS2DW12_PM_OPS	(&ais2dw12_input_pm_ops)
#else	/* CONFIG_PM_SLEEP */
#define AIS2DW12_PM_OPS	NULL
#endif	/* CONFIG_PM_SLEEP */


static const struct i2c_device_id ais2dw12_input_ids[] = {
	{AIS2DW12_DEV_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ais2dw12_input_ids);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ais2dw12_input_id_table[] = {
	{.compatible = "st,ais2dw12_input",},
	{},
};
MODULE_DEVICE_TABLE(of, ais2dw12_input_id_table);
#endif

static struct i2c_driver ais2dw12_input_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = AIS2DW12_DEV_NAME,
		   .pm = AIS2DW12_PM_OPS,
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = ais2dw12_input_id_table,
#endif
	},
	.probe = ais2dw12_input_i2c_probe,
	.remove = ais2dw12_input_i2c_remove,
	.id_table = ais2dw12_input_ids,
};

module_i2c_driver(ais2dw12_input_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics ais2dw12 i2c for input driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
