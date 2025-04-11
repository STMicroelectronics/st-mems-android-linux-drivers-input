// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics ais2ih input driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2025 STMicroelectronics Inc.
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/version.h>

#include "stm_input_ais2ih_core.h"

#define SENSORS_SPI_READ	0x80

static int ais2ih_input_spi_read(struct ais2ih_input_data *cdata,
				 u8 reg_addr, int len, u8 *data, bool b_lock)
{
	int err;
	struct spi_message msg;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = cdata->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = cdata->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	if (len >= AIS2IH_RX_MAX_LENGTH) {
		dev_err(cdata->dev, "SPI read len size exceeded:%d\n", len);
		return -EINVAL;
	}

	if (b_lock)
		mutex_lock(&cdata->bank_registers_lock);

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr | SENSORS_SPI_READ;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(to_spi_device(cdata->dev), &msg);
	if (err)
		goto acc_spi_read_error;

	memcpy(data, cdata->tb.rx_buf, len * sizeof(u8));
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return len;

acc_spi_read_error:
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return err;
}

static int ais2ih_input_spi_write(struct ais2ih_input_data *cdata,
				  u8 reg_addr, int len, u8 *data,
				  bool b_lock)
{
	if (len >= AIS2IH_TX_MAX_LENGTH) {
		dev_err(cdata->dev, "SPI write len size exceeded:%d\n", len);
		return -EINVAL;
	}

	int err;
	struct spi_message msg;
	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (b_lock)
		mutex_lock(&cdata->bank_registers_lock);

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr;

	memcpy(&cdata->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	err = spi_sync(to_spi_device(cdata->dev), &msg);

	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return err;
}


static const struct ais2ih_input_transfer_function ais2ih_input_tf_spi = {
	.write = ais2ih_input_spi_write,
	.read = ais2ih_input_spi_read,
};

static int ais2ih_input_spi_probe(struct spi_device *spi)
{
	int err;
	struct ais2ih_input_data *cdata;

	cdata =
		devm_kzalloc(&spi->dev, sizeof(struct ais2ih_input_data),
				GFP_KERNEL);

	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->tf = &ais2ih_input_tf_spi;
	spi_set_drvdata(spi, cdata);

	err = ais2ih_input_common_probe(cdata, spi->irq, BUS_SPI);
	if (err < 0)
		return err;

	return 0;
}

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
static void ais2ih_input_spi_remove(struct spi_device *spi)
{
	struct ais2ih_input_data *cdata = spi_get_drvdata(spi);

	ais2ih_input_common_remove(cdata, spi->irq);
}
#else
static int ais2ih_input_spi_remove(struct spi_device *spi)
{
	struct ais2ih_input_data *cdata = spi_get_drvdata(spi);

	ais2ih_input_common_remove(cdata, spi->irq);

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int ais2ih_input_suspend(struct device *dev)
{
	struct ais2ih_input_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return ais2ih_input_common_suspend(cdata);
}

static int ais2ih_input_resume(struct device *dev)
{
	struct ais2ih_input_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return ais2ih_input_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(ais2ih_input_pm_ops, ais2ih_input_suspend,
			 ais2ih_input_resume);

#define AIS2IH_PM_OPS		(&ais2ih_input_pm_ops)
#else	/* CONFIG_PM_SLEEP */
#define AIS2IH_PM_OPS		NULL
#endif	/* CONFIG_PM_SLEEP */

static const struct spi_device_id ais2ih_input_ids[] = {
	{AIS2IH_DEV_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(spi, ais2ih_input_ids);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ais2ih_input_id_table[] = {
	{.compatible = "st,ais2ih_input",},
	{},
};
MODULE_DEVICE_TABLE(of, ais2ih_input_id_table);
#endif	/* CONFIG_OF */

static struct spi_driver ais2ih_input_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = AIS2IH_DEV_NAME,
		.pm = AIS2IH_PM_OPS,
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = ais2ih_input_id_table,
#endif
	},
	.probe = ais2ih_input_spi_probe,
	.remove = ais2ih_input_spi_remove,
	.id_table = ais2ih_input_ids,
};

module_spi_driver(ais2ih_input_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics ais2ih spi for input driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
