/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: a3g4250d_i2c.c
* Authors	: AMS - Motion Mems Division - Application Team - Application Team
*		: Giuseppe Barba <giuseppe.barba@st.com>
*		: Mario Tesi <mario.tesi@st.com>
*		: Author is willing to be considered the contact and update
* Version	: V.1.0.0
* Date		: 2016/May/23
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
*******************************************************************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/delay.h>

#include "a3g4250d.h"

#define	I2C_AUTO_INCREMENT	0x80

static int a3g4250d_i2c_read(struct a3g4250d_data *cdata, u8 reg_addr,
			     int len, u8 *data)
{
	int err = 0;
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(cdata->dev);

	reg_addr |= ((len > 1) ? I2C_AUTO_INCREMENT : 0);
	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);

	return err;
}

static int a3g4250d_i2c_write(struct a3g4250d_data *cdata, u8 reg_addr,
			      int len, u8 *data)
{
	int err = 0;
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(cdata->dev);

	reg_addr |= ((len > 1) ? I2C_AUTO_INCREMENT : 0);
	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	err = i2c_transfer(client->adapter, &msg, 1);

	return err;
}

static struct a3g4250d_transfer_function a3g4250d_tf_i2c = {
	.write = a3g4250d_i2c_write,
	.read = a3g4250d_i2c_read,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static int a3g4250d_i2c_probe(struct i2c_client *client)
#else /* LINUX_VERSION_CODE */
static int a3g4250d_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
#endif /* LINUX_VERSION_CODE */
{
	int err;
	struct a3g4250d_data *cdata;

	cdata = kzalloc(sizeof(struct a3g4250d_data), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->bustype = BUS_I2C;
	cdata->tf = &a3g4250d_tf_i2c;
	i2c_set_clientdata(client, cdata);

	err = a3g4250d_common_probe(cdata);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);
	return err;
}

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
static void a3g4250d_i2c_remove(struct i2c_client *client)
{
	struct a3g4250d_data *cdata = i2c_get_clientdata(client);

	a3g4250d_common_remove(cdata);
	kfree(cdata);
}
#else
static int a3g4250d_i2c_remove(struct i2c_client *client)
{
	struct a3g4250d_data *cdata = i2c_get_clientdata(client);

	a3g4250d_common_remove(cdata);
	kfree(cdata);

	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int a3g4250d_suspend(struct device *dev)
{
	struct a3g4250d_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return a3g4250d_common_suspend(cdata);
}

static int a3g4250d_resume(struct device *dev)
{
	struct a3g4250d_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return a3g4250d_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(a3g4250d_pm_ops, a3g4250d_suspend, a3g4250d_resume);

#define A3G4250D_PM_OPS		(&a3g4250d_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define A3G4250D_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id a3g4250d_ids[] = {
	{ A3G4250D_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, a3g4250d_ids);

#ifdef CONFIG_OF
static const struct of_device_id a3g4250d_id_table[] = {
	{ .compatible = "st,a3g4250d", },
	{ },
};
MODULE_DEVICE_TABLE(of, a3g4250d_id_table);
#endif

static struct i2c_driver a3g4250d_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = A3G4250D_DEV_NAME,
		.pm = A3G4250D_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = a3g4250d_id_table,
#endif
	},
	.remove = a3g4250d_i2c_remove,
	.probe = a3g4250d_i2c_probe,
	.id_table = a3g4250d_ids,
};

module_i2c_driver(a3g4250d_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics a3g4250d i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
