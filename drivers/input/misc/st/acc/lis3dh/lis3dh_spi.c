/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: lis3dh_spi.c
* Authors	: AMS - Motion Mems Division - Application Team - Application Team
*		: Giuseppe Barba <giuseppe.barba@st.com>
*		: Mario Tesi <mario.tesi@st.com>
*		: Author is willing to be considered the contact and update
* Version	: V.1.0.14
* Date		: 2016/Apr/26
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
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/input.h>

#include "lis3dh.h"

#define SENSORS_SPI_READ	0x80
#define SPI_AUTO_INCREMENT	0x40

static int lis3dh_acc_spi_read(struct lis3dh_acc_status *stat, u8 reg_addr, int len,
			       u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(stat->dev);

	struct spi_transfer xfers[] = {
		{
			.tx_buf = stat->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = stat->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	mutex_lock(&stat->tb.buf_lock);
	stat->tb.tx_buf[0] = reg_addr | SENSORS_SPI_READ | ((len > 1) ? SPI_AUTO_INCREMENT : 0);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(spi, &msg);
	if (err)
		goto acc_spi_read_error;

	memcpy(data, stat->tb.rx_buf, len*sizeof(u8));
	mutex_unlock(&stat->tb.buf_lock);

	return len;

acc_spi_read_error:
	mutex_unlock(&stat->tb.buf_lock);

	return err;
}

static int lis3dh_acc_spi_write(struct lis3dh_acc_status *stat, u8 reg_addr, int len,
			       u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(stat->dev);

	struct spi_transfer xfers = {
		.tx_buf = stat->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= BUFF_RX_MAX_LENGTH)
		return -ENOMEM;

	mutex_lock(&stat->tb.buf_lock);
	stat->tb.tx_buf[0] = reg_addr | ((len > 1) ? SPI_AUTO_INCREMENT : 0);

	memcpy(&stat->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	err = spi_sync(spi, &msg);
	mutex_unlock(&stat->tb.buf_lock);

	return err;
}

static struct lis3dh_acc_transfer_function lis3dh_acc_tf_spi = {
	.write = lis3dh_acc_spi_write,
	.read = lis3dh_acc_spi_read,
};

static int lis3dh_acc_spi_probe(struct spi_device *spi)
{
	int err;
	struct lis3dh_acc_status *stat;

	stat = kmalloc(sizeof(struct lis3dh_acc_status), GFP_KERNEL);
	if (!stat)
		return -ENOMEM;

	stat->dev = &spi->dev;
	stat->name = spi->modalias;
	stat->bustype = BUS_SPI;
	stat->tf = &lis3dh_acc_tf_spi;
	spi_set_drvdata(spi, stat);

	err = lis3dh_acc_probe(stat, spi->irq);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(stat);

	return err;
}

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
static void lis3dh_acc_spi_remove(struct spi_device *spi)
{
	struct lis3dh_acc_status *stat = spi_get_drvdata(spi);

	lis3dh_acc_remove(stat);
	kfree(stat);
}
#else
static int lis3dh_acc_spi_remove(struct spi_device *spi)
{
	struct lis3dh_acc_status *stat = spi_get_drvdata(spi);

	lis3dh_acc_remove(stat);
	kfree(stat);

	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int lis3dh_acc_suspend(struct device *dev)
{
	struct lis3dh_acc_status *stat= spi_get_drvdata(to_spi_device(dev));

	return lis3dh_acc_common_suspend(stat);
}

static int lis3dh_acc_resume(struct device *dev)
{
	struct lis3dh_acc_status *stat = spi_get_drvdata(to_spi_device(dev));

	return lis3dh_acc_common_resume(stat);
}

static SIMPLE_DEV_PM_OPS(lis3dh_acc_pm_ops,
				lis3dh_acc_suspend,
				lis3dh_acc_resume);

#define LIS3DH_PM_OPS		(&lis3dh_acc_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LIS3DH_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static const struct spi_device_id lis3dh_acc_ids[] = {
	{ LIS3DH_ACC_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, lis3dh_acc_ids);

#ifdef CONFIG_OF
static const struct of_device_id lis3dh_acc_id_table[] = {
	{ .compatible = "st,lis3dh", },
	{ },
};
MODULE_DEVICE_TABLE(of, lis3dh_acc_id_table);
#endif

static struct spi_driver lis3dh_acc_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS3DH_ACC_DEV_NAME,
		.pm = LIS3DH_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lis3dh_acc_id_table,
#endif
	},
	.remove = lis3dh_acc_spi_remove,
	.probe = lis3dh_acc_spi_probe,
	.id_table = lis3dh_acc_ids,
};

module_spi_driver(lis3dh_acc_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lis3dh spi driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
