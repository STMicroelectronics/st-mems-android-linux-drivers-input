/*
 * STMicroelectronics lsm9ds1_acc_gyr_i2c.c driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/err.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>

#include "lsm9ds1.h"

/* XXX: caller must hold dev->lock */
static int lsm9ds1_acc_gyr_i2c_read(struct device *dev, u8 addr, int len,
				    u8 *data)
{
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(dev);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	return i2c_transfer(client->adapter, msg, 2);
}

/* XXX: caller must hold dev->lock */
static int lsm9ds1_acc_gyr_i2c_write(struct device *dev, u8 addr, int len,
				     u8 *data)
{
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(dev);

	send[0] = addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	return i2c_transfer(client->adapter, &msg, 1);
}

static struct lsm9ds1_transfer_function lsm9ds1_acc_gyr_i2c_tf = {
	.write = lsm9ds1_acc_gyr_i2c_write,
	.read = lsm9ds1_acc_gyr_i2c_read,
};

#ifdef CONFIG_PM_SLEEP
static int lsm9ds1_acc_gyr_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct lsm9ds1_acc_gyr_dev *dev = i2c_get_clientdata(client);

	return lsm9ds1_acc_gyr_enable(dev);
}

static int lsm9ds1_acc_gyr_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct lsm9ds1_acc_gyr_dev *dev = i2c_get_clientdata(client);

	return lsm9ds1_acc_gyr_disable(dev);
}

static SIMPLE_DEV_PM_OPS(lsm9ds1_acc_gyr_pm_ops,
				lsm9ds1_acc_gyr_suspend,
				lsm9ds1_acc_gyr_resume);

#define LSM9DS1_ACC_GYR_PM_OPS		(&lsm9ds1_acc_gyr_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LSM9DS1_ACC_GYR_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_OF
static const struct of_device_id lsm9ds1_acc_gyr_i2c_id_table[] = {
	{ .compatible = "st,lsm9ds1_acc_gyr", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm9ds1_acc_gyr_i2c_id_table);
#endif /* CONFIG_OF */

static int lsm9ds1_acc_gyr_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	int err;
	struct lsm9ds1_acc_gyr_dev *dev;

#ifdef LSM303D_DEBUG
	dev_info(&client->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	dev = kzalloc(sizeof(struct lsm9ds1_acc_gyr_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	dev->dev = &client->dev;
	dev->name = client->name;
	dev->bus_type = BUS_I2C;
	dev->tf = &lsm9ds1_acc_gyr_i2c_tf;
#ifdef CONFIG_OF
	dev->acc_gyr_dt_id = lsm9ds1_acc_gyr_i2c_id_table;
#endif /* CONFIG_OF */

	i2c_set_clientdata(client, dev);

	mutex_init(&dev->lock);

	err = lsm9ds1_acc_gyr_probe(dev, client->irq);
	if (err < 0) {
		kfree(dev);

		return err;
	}

	return 0;
}

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
static void lsm9ds1_acc_gyr_i2c_remove(struct i2c_client *client)
{
	struct lsm9ds1_acc_gyr_dev *dev = i2c_get_clientdata(client);

#ifdef LSM303D_DEBUG
	dev_info(dev->dev, "driver removing\n");
#endif

	lsm9ds1_acc_gyr_remove(dev);
	kfree(dev);
}
#else
static int lsm9ds1_acc_gyr_i2c_remove(struct i2c_client *client)
{
	struct lsm9ds1_acc_gyr_dev *dev = i2c_get_clientdata(client);

#ifdef LSM303D_DEBUG
	dev_info(dev->dev, "driver removing\n");
#endif

	lsm9ds1_acc_gyr_remove(dev);
	kfree(dev);

	return 0;
}
#endif

static const struct i2c_device_id lsm9ds1_acc_gyr_i2c_id[] = {
	{ "lsm9ds1_acc_gyr", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, lsm9ds1_acc_gyr_i2c_id);

static struct i2c_driver lsm9ds1_acc_gyr_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lsm9ds1_acc_gyr_i2c",
		.pm = LSM9DS1_ACC_GYR_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lsm9ds1_acc_gyr_i2c_id_table,
#endif /* CONFIG_OF */
	},
	.probe = lsm9ds1_acc_gyr_i2c_probe,
	.remove = lsm9ds1_acc_gyr_i2c_remove,
	.id_table = lsm9ds1_acc_gyr_i2c_id,
};

module_i2c_driver(lsm9ds1_acc_gyr_i2c_driver);

MODULE_DESCRIPTION("lsm9ds1 acc-gyro i2c driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

