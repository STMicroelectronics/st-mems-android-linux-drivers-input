// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics ais2ih i2c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2010-2026 STMicroelectronics Inc.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/version.h>

#include "stm_input_ais2ih_core.h"

static int ais2ih_input_i2c_read(struct ais2ih_input_data *cdata,
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

	if (len <= 0 || len > AIS2IH_RX_MAX_LENGTH) {
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

static int ais2ih_input_i2c_write(struct ais2ih_input_data *cdata,
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

	if (len <= 0 || len >= AIS2IH_TX_MAX_LENGTH) {
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

static const struct ais2ih_input_transfer_function ais2ih_input_tf_i2c = {
	.write = ais2ih_input_i2c_write,
	.read = ais2ih_input_i2c_read,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static int ais2ih_input_i2c_probe(struct i2c_client *client)
#else	/* LINUX_VERSION_CODE */
static int ais2ih_input_i2c_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
#endif	/* LINUX_VERSION_CODE */
{
	int err;
	struct ais2ih_input_data *cdata;

	cdata = devm_kzalloc(&client->dev, sizeof(struct ais2ih_input_data),
			     GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->tf = &ais2ih_input_tf_i2c;
	i2c_set_clientdata(client, cdata);

	err = ais2ih_input_common_probe(cdata, client->irq, BUS_I2C);
	if (err < 0)
		return err;

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void ais2ih_input_i2c_remove(struct i2c_client *client)
{
	struct ais2ih_input_data *cdata = i2c_get_clientdata(client);

	ais2ih_input_common_remove(cdata, client->irq);
}
#else
static int ais2ih_input_i2c_remove(struct i2c_client *client)
{
	struct ais2ih_input_data *cdata = i2c_get_clientdata(client);

	ais2ih_input_common_remove(cdata, client->irq);

	return 0;
}
#endif /* LINUX_VERSION_CODE */

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int ais2ih_input_suspend(struct device *dev)
{
	struct ais2ih_input_data *cdata =
		i2c_get_clientdata(to_i2c_client(dev));

	return ais2ih_input_common_suspend(cdata);
}

static int ais2ih_input_resume(struct device *dev)
{
	struct ais2ih_input_data *cdata =
		i2c_get_clientdata(to_i2c_client(dev));

	return ais2ih_input_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(ais2ih_input_pm_ops, ais2ih_input_suspend,
			 ais2ih_input_resume);

#define AIS2IH_PM_OPS	(&ais2ih_input_pm_ops)
#else	/* CONFIG_PM_SLEEP */
#define AIS2IH_PM_OPS	NULL
#endif	/* CONFIG_PM_SLEEP */

static const struct i2c_device_id ais2ih_input_ids[] = {
	{AIS2IH_DEV_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ais2ih_input_ids);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ais2ih_input_id_table[] = {
	{.compatible = "st,ais2ih_input",},
	{},
};
MODULE_DEVICE_TABLE(of, ais2ih_input_id_table);
#endif	/* CONFIG_OF */

static struct i2c_driver ais2ih_input_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = AIS2IH_DEV_NAME,
		   .pm = AIS2IH_PM_OPS,
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = ais2ih_input_id_table,
#endif
	},
	.probe = ais2ih_input_i2c_probe,
	.remove = ais2ih_input_i2c_remove,
	.id_table = ais2ih_input_ids,
};

module_i2c_driver(ais2ih_input_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics ais2ih i2c for input driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
