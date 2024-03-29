/*
 * STMicroelectronics lsm9ds1_mag_spi driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/version.h>

#include "lsm9ds1.h"

#define SENSORS_SPI_READ	0x80
#define SPI_AUTO_INCREMENT	0x40

/* XXX: caller must hold dev->lock */
static int lsm9ds1_mag_spi_read(struct device *device, u8 addr, int len,
				u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct lsm9ds1_mag_dev *dev = spi_get_drvdata(spi);

	struct spi_transfer xfers[] = {
		{
			.tx_buf = dev->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = dev->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	if (len > 1)
		addr |= SPI_AUTO_INCREMENT;

	dev->tb.tx_buf[0] = addr | SENSORS_SPI_READ;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(spi, &msg);
	if (err)
		return err;

	memcpy(data, dev->tb.rx_buf, len * sizeof(u8));

	return len;
}

/* XXX: caller must hold dev->lock */
static int lsm9ds1_mag_spi_write(struct device *device, u8 addr, int len,
				 u8 *data)
{
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct lsm9ds1_mag_dev *dev = spi_get_drvdata(spi);

	struct spi_transfer xfers = {
		.tx_buf = dev->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LSM9DS1_TX_MAX_LENGTH)
		return -ENOMEM;

	if (len > 1)
		addr |= SPI_AUTO_INCREMENT;

	dev->tb.tx_buf[0] = addr;

	memcpy(&dev->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	return spi_sync(spi, &msg);
}

static struct lsm9ds1_transfer_function lsm9ds1_mag_spi_tf = {
	.write = lsm9ds1_mag_spi_write,
	.read = lsm9ds1_mag_spi_read,
};

#ifdef CONFIG_PM_SLEEP
static int lsm9ds1_mag_resume(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct lsm9ds1_mag_dev *dev = spi_get_drvdata(spi);

	return lsm9ds1_mag_enable(dev);
}

static int lsm9ds1_mag_suspend(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct lsm9ds1_mag_dev *dev = spi_get_drvdata(spi);

	return lsm9ds1_mag_disable(dev);
}

static SIMPLE_DEV_PM_OPS(lsm9ds1_mag_pm_ops,
				lsm9ds1_mag_suspend,
				lsm9ds1_mag_resume);

#define LSM9DS1_MAG_PM_OPS		(&lsm9ds1_mag_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LSM9DS1_MAG_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_OF
static const struct of_device_id lsm9ds1_mag_spi_id_table[] = {
	{ .compatible = "st,lsm9ds1_mag" },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm9ds1_mag_spi_id_table);
#endif /* CONFIG_OF */

static int lsm9ds1_mag_spi_probe(struct spi_device *spi)
{
	int err;
	struct lsm9ds1_mag_dev *dev;

#ifdef LSM9DS1_DEBUG
	dev_info(&spi->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	dev = kzalloc(sizeof(struct lsm9ds1_mag_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&spi->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	dev->name = spi->modalias;
	dev->bus_type = BUS_SPI;
	dev->tf = &lsm9ds1_mag_spi_tf;
	dev->dev = &spi->dev;
#ifdef CONFIG_OF
	dev->mag_dt_id = lsm9ds1_mag_spi_id_table;
#endif

	spi_set_drvdata(spi, dev);

	mutex_init(&dev->lock);

	err = lsm9ds1_mag_probe(dev);
	if (err < 0) {
		kfree(dev);
		return err;
	}

	return 0;
}

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
static void lsm9ds1_mag_spi_remove(struct spi_device *spi)
{
	struct lsm9ds1_mag_dev *dev = spi_get_drvdata(spi);

#ifdef LSM9DS1_DEBUG
	dev_info(dev->dev, "driver removing\n");
#endif

	lsm9ds1_mag_remove(dev);
	kfree(dev);
}
#else
static int lsm9ds1_mag_spi_remove(struct spi_device *spi)
{
	struct lsm9ds1_mag_dev *dev = spi_get_drvdata(spi);

#ifdef LSM9DS1_DEBUG
	dev_info(dev->dev, "driver removing\n");
#endif

	lsm9ds1_mag_remove(dev);
	kfree(dev);

	return 0;
}
#endif

static const struct spi_device_id lsm9ds1_mag_spi_id[] = {
	{ "lsm9ds1_mag", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, lsm9ds1_mag_spi_id);

static struct spi_driver lsm9ds1_mag_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lsm9ds1_mag_spi",
		.pm = LSM9DS1_MAG_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lsm9ds1_mag_spi_id_table,
#endif /* CONFIG_OF */
	},
	.probe = lsm9ds1_mag_spi_probe,
	.remove = lsm9ds1_mag_spi_remove,
	.id_table = lsm9ds1_mag_spi_id,
};

module_spi_driver(lsm9ds1_mag_spi_driver);

MODULE_DESCRIPTION("lsm9ds1 mag spi driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

