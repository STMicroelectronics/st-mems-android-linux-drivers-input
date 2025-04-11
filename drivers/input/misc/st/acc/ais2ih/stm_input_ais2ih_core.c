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
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include <linux/platform_data/st/ais2ih.h>
#include "stm_input_ais2ih_core.h"

#define AIS2IH_WHO_AM_I_ADDR		0x0f
#define AIS2IH_WHO_AM_I_DEF		0x44

#define AIS2IH_CTRL1_ADDR		0x20
#define AIS2IH_CTRL2_ADDR		0x21
#define AIS2IH_CTRL3_ADDR		0x22
#define AIS2IH_CTRL4_INT1_PAD_ADDR	0x23
#define AIS2IH_CTRL5_INT2_PAD_ADDR	0x24
#define AIS2IH_CTRL6_ADDR		0x25
#define AIS2IH_OUT_T_ADDR		0x26
#define AIS2IH_STATUS_ADDR		0x27
#define AIS2IH_OUTX_L_ADDR		0x28

#define AIS2IH_TAP_THS_X_ADDR		0x30
#define AIS2IH_TAP_THS_Y_ADDR		0x31
#define AIS2IH_TAP_THS_Z_ADDR		0x32
#define AIS2IH_INT_DUR_ADDR		0x33
#define AIS2IH_WAKE_UP_THS_ADDR		0x34
#define AIS2IH_WAKE_UP_DUR_ADDR		0x35
#define AIS2IH_FREE_FALL_ADDR		0x36
#define AIS2IH_STATUS_DUP_ADDR		0x37
#define AIS2IH_WAKE_UP_SRC_ADDR		0x38
#define AIS2IH_TAP_SRC_ADDR		0x39
#define AIS2IH_6D_SRC_ADDR		0x3a
#define AIS2IH_ALL_INT_ADDR		0x3b

#define AIS2IH_WAKE_UP_IA_MASK		0x40
#define AIS2IH_DOUBLE_TAP_MASK		0x10
#define AIS2IH_SINGLE_TAP_MASK		0x08
#define AIS2IH_6D_IA_MASK		0x04
#define AIS2IH_FF_IA_MASK		0x02
#define AIS2IH_DRDY_MASK		0x01
#define AIS2IH_EVENT_MASK		(AIS2IH_WAKE_UP_IA_MASK | \
					 AIS2IH_DOUBLE_TAP_MASK | \
					 AIS2IH_SINGLE_TAP_MASK | \
					 AIS2IH_6D_IA_MASK | \
					 AIS2IH_FF_IA_MASK)

#define AIS2IH_ODR_MASK			0xf0
#define AIS2IH_ODR_POWER_OFF_VAL	0x00
#define AIS2IH_ODR_1HZ_LP_VAL		0x01
#define AIS2IH_ODR_12HZ_LP_VAL		0x02
#define AIS2IH_ODR_25HZ_LP_VAL		0x03
#define AIS2IH_ODR_50HZ_LP_VAL		0x04
#define AIS2IH_ODR_100HZ_LP_VAL		0x05
#define AIS2IH_ODR_200HZ_LP_VAL		0x06
#define AIS2IH_ODR_400HZ_LP_VAL		0x06
#define AIS2IH_ODR_800HZ_LP_VAL		0x06
#define AIS2IH_ODR_LP_LIST_NUM		9

#define AIS2IH_ODR_12_5HZ_HR_VAL	0x02
#define AIS2IH_ODR_25HZ_HR_VAL		0x03
#define AIS2IH_ODR_50HZ_HR_VAL		0x04
#define AIS2IH_ODR_100HZ_HR_VAL		0x05
#define AIS2IH_ODR_200HZ_HR_VAL		0x06
#define AIS2IH_ODR_400HZ_HR_VAL		0x07
#define AIS2IH_ODR_800HZ_HR_VAL		0x08
#define AIS2IH_ODR_HR_LIST_NUM		8

#define __MAX(a, b) ((a) > (b) ? (a) : (b))
#define AIS2IH_ODR_LIST_NUM_MAX __MAX(AIS2IH_ODR_LP_LIST_NUM, \
				      AIS2IH_ODR_HR_LIST_NUM)

#define AIS2IH_LP_MODE_MASK		0x03
#define AIS2IH_PWR_OPER_MODE_MASK	0x0c

#define AIS2IH_FS_MASK			0x30
#define AIS2IH_FS_2G_VAL		0x00
#define AIS2IH_FS_4G_VAL		0x01
#define AIS2IH_FS_8G_VAL		0x02
#define AIS2IH_FS_16G_VAL		0x03

/*
 * Sensitivity sets in LP mode [ug]
 */
#define AIS2IH_FS_2G_GAIN_LP		976
#define AIS2IH_FS_4G_GAIN_LP		1952
#define AIS2IH_FS_8G_GAIN_LP		3904
#define AIS2IH_FS_16G_GAIN_LP		7808

/*
 * Sensitivity sets in HR mode [ug]
 */
#define AIS2IH_FS_2G_GAIN_HR		244
#define AIS2IH_FS_4G_GAIN_HR		488
#define AIS2IH_FS_8G_GAIN_HR		976
#define AIS2IH_FS_16G_GAIN_HR		1952

#define AIS2IH_FS_LIST_NUM		4

#define AIS2IH_INT1_6D_MASK		0x80
#define AIS2IH_INT1_S_TAP_MASK		0x40
#define AIS2IH_INT1_WAKEUP_MASK		0x20
#define AIS2IH_INT1_FREE_FALL_MASK	0x10
#define AIS2IH_INT1_TAP_MASK		0x08
#define AIS2IH_INT1_DRDY_MASK		0x01
#define AIS2IH_INT2_SLEEP_MASK		0x40
#define AIS2IH_INT1_EVENTS_MASK		(AIS2IH_INT1_S_TAP_MASK | \
					 AIS2IH_INT1_WAKEUP_MASK | \
					 AIS2IH_INT1_FREE_FALL_MASK | \
					 AIS2IH_INT1_TAP_MASK | \
					 AIS2IH_INT1_6D_MASK)

#define AIS2IH_INT_DUR_SHOCK_MASK	0x03
#define AIS2IH_INT_DUR_QUIET_MASK	0x0c
#define AIS2IH_INT_DUR_LAT_MASK		0xf0
#define AIS2IH_INT_DUR_MASK		(AIS2IH_INT_DUR_SHOCK_MASK | \
					 AIS2IH_INT_DUR_QUIET_MASK | \
					 AIS2IH_INT_DUR_LAT_MASK)

#define AIS2IH_INT_DUR_STAP_DEFAULT	0x06
#define AIS2IH_INT_DUR_DTAP_DEFAULT	0x7f

#define AIS2IH_WAKE_UP_THS_S_D_TAP_MASK	0x80
#define AIS2IH_WAKE_UP_THS_SLEEP_MASK	0x40
#define AIS2IH_WAKE_UP_THS_WU_MASK	0x3f
#define AIS2IH_WAKE_UP_THS_WU_DEFAULT	0x02

#define AIS2IH_FREE_FALL_THS_MASK	0x07
#define AIS2IH_FREE_FALL_DUR_MASK	0xf8
#define AIS2IH_FREE_FALL_THS_DEFAULT	0x01
#define AIS2IH_FREE_FALL_DUR_DEFAULT	0x01

#define AIS2IH_BDU_MASK			0x08
#define AIS2IH_SOFT_RESET_MASK		0x40
#define AIS2IH_LIR_MASK			0x10

#define AIS2IH_TAP_AXIS_MASK		0xe0
#define AIS2IH_TAP_AXIS_ENABLE_ALL	0xe0
#define AIS2IH_TAP_THS_MASK		0x1f
#define AIS2IH_TAP_THS_DEFAULT		0x09
#define AIS2IH_INT2_ON_INT1_MASK	0x20

#define AIS2IH_OUT_XYZ_SIZE		6
#define AIS2IH_EN_BIT			0x01
#define AIS2IH_DIS_BIT			0x00
#define AIS2IH_EN_LP_MODE_02		0x01

#define AIS2IH_ACCEL_FS			2
#define AIS2IH_FF_ODR			25
#define AIS2IH_TAP_ODR			400
#define AIS2IH_WAKEUP_ODR		25

#define AIS2IH_MIN_EVENT_ODR		25

#define AIS2IH_POLLRATE_CLAMP_USEC	1000000 // 1 sec,	1Hz
#define AIS2IH_POLLRATE_MIN_USEC	1250	// 0.125 ms,	800Hz

enum {
	AIS2IH_LP_MODE = 0,
	AIS2IH_HR_MODE,
	AIS2IH_MODE_COUNT,
};

static struct workqueue_struct *ais2ih_input_workqueue;

static const struct ais2ih_input_sensor_name {
	const char *name;
	const char *description;
} ais2ih_input_sensor_name[AIS2IH_SENSORS_NUMB] = {
	[AIS2IH_ACCEL] = {
		.name = AIS2IH_DEV_NAME"_accel",
		.description = "ST AIS2IH Accelerometer Sensor",
	},
	[AIS2IH_FF] = {
		.name = AIS2IH_DEV_NAME"_free_fall",
		.description = "ST AIS2IH Free Fall Sensor",
	},
	[AIS2IH_TAP] = {
		.name = AIS2IH_DEV_NAME"_tap",
		.description = "ST AIS2IH Tap Sensor",
	},
	[AIS2IH_DOUBLE_TAP] = {
		.name = AIS2IH_DEV_NAME"_double_tap",
		.description = "ST AIS2IH Double Tap Sensor",
	},
	[AIS2IH_WAKEUP] = {
		.name = AIS2IH_DEV_NAME"_wake_up",
		.description = "ST AIS2IH Wake Up Sensor",
	},
};

struct ais2ih_input_odr_reg {
	u32 hz;
	u8 value;
};

static const struct ais2ih_input_odr_table_t {
	struct ais2ih_input_odr_reg
			odr_avl[AIS2IH_MODE_COUNT][AIS2IH_ODR_LIST_NUM_MAX];
} ais2ih_input_odr_table = {
	.odr_avl[AIS2IH_LP_MODE][0] = {
		.hz = 0,
		.value = AIS2IH_ODR_POWER_OFF_VAL
	},
	.odr_avl[AIS2IH_LP_MODE][1] = {
		.hz = 1,
		.value = AIS2IH_ODR_1HZ_LP_VAL
	},
	.odr_avl[AIS2IH_LP_MODE][2] = {
		.hz = 12,
		.value = AIS2IH_ODR_12HZ_LP_VAL
	},
	.odr_avl[AIS2IH_LP_MODE][3] = {
		.hz = 25,
		.value = AIS2IH_ODR_25HZ_LP_VAL
	},
	.odr_avl[AIS2IH_LP_MODE][4] = {
		.hz = 50,
		.value = AIS2IH_ODR_50HZ_LP_VAL
	},
	.odr_avl[AIS2IH_LP_MODE][5] = {
		.hz = 100,
		.value = AIS2IH_ODR_100HZ_LP_VAL
	},
	.odr_avl[AIS2IH_LP_MODE][6] = {
		.hz = 200,
		.value = AIS2IH_ODR_200HZ_LP_VAL
	},
	.odr_avl[AIS2IH_LP_MODE][7] = {
		.hz = 400,
		.value = AIS2IH_ODR_400HZ_LP_VAL
	},
	.odr_avl[AIS2IH_LP_MODE][8] = {
		.hz = 800,
		.value = AIS2IH_ODR_800HZ_LP_VAL
	},

	.odr_avl[AIS2IH_HR_MODE][0] = {
		.hz = 0,
		.value = AIS2IH_ODR_POWER_OFF_VAL
	},
	.odr_avl[AIS2IH_HR_MODE][1] = {
		.hz = 12,
		.value = AIS2IH_ODR_12_5HZ_HR_VAL
	},
	.odr_avl[AIS2IH_HR_MODE][2] = {
		.hz = 25,
		.value = AIS2IH_ODR_25HZ_HR_VAL
	},
	.odr_avl[AIS2IH_HR_MODE][3] = {
		.hz = 50,
		.value = AIS2IH_ODR_50HZ_HR_VAL
	},
	.odr_avl[AIS2IH_HR_MODE][4] = {
		.hz = 100,
		.value = AIS2IH_ODR_100HZ_HR_VAL
	},
	.odr_avl[AIS2IH_HR_MODE][5] = {
		.hz = 200,
		.value = AIS2IH_ODR_200HZ_HR_VAL
	},
	.odr_avl[AIS2IH_HR_MODE][6] = {
		.hz = 400,
		.value = AIS2IH_ODR_400HZ_HR_VAL
	},
	.odr_avl[AIS2IH_HR_MODE][7] = {
		.hz = 800,
		.value = AIS2IH_ODR_800HZ_HR_VAL
	},
};

struct ais2ih_input_fs_reg {
	unsigned int gain[AIS2IH_MODE_COUNT];
	u8 value;
	int urv;
};

static struct ais2ih_input_fs_table {
	u8 addr;
	u8 mask;
	struct ais2ih_input_fs_reg fs_avl[AIS2IH_FS_LIST_NUM];
} ais2ih_input_fs_table = {
	.addr = AIS2IH_CTRL6_ADDR,
	.mask = AIS2IH_FS_MASK,
	.fs_avl[0] = {
		.gain = {
			AIS2IH_FS_2G_GAIN_LP,
			AIS2IH_FS_2G_GAIN_HR,
		},
		.value = AIS2IH_FS_2G_VAL,
		.urv = 2,
	},
	.fs_avl[1] = {
		.gain = {
			AIS2IH_FS_4G_GAIN_LP,
			AIS2IH_FS_4G_GAIN_HR,
		},
		.value = AIS2IH_FS_4G_VAL,
		.urv = 4,
	},
	.fs_avl[2] = {
		.gain = {
			AIS2IH_FS_8G_GAIN_LP,
			AIS2IH_FS_8G_GAIN_HR,
		},
		.value = AIS2IH_FS_8G_VAL,
		.urv = 8,
	},
	.fs_avl[3] = {
		.gain = {
			AIS2IH_FS_16G_GAIN_LP,
			AIS2IH_FS_16G_GAIN_HR,
		},
		.value = AIS2IH_FS_16G_VAL,
		.urv = 16,
	},
};

static int
ais2ih_input_write_data_with_mask(struct ais2ih_input_data *cdata,
				  u8 reg_addr, u8 mask, u8 data,
				  bool b_lock)
{
	int err;
	u8 new_data = 0x00, old_data = 0x00;

	err = cdata->tf->read(cdata, reg_addr, 1, &old_data, b_lock);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));

	if (new_data == old_data)
		return 1;

	return cdata->tf->write(cdata, reg_addr, 1, &new_data, b_lock);
}

static int ais2ih_input_input_init(struct ais2ih_input_sensor_data *sdata,
				   u16 bustype)
{
	int err = 0;

	sdata->input_dev = devm_input_allocate_device(sdata->cdata->dev);
	if (!sdata->input_dev) {
		dev_err(sdata->cdata->dev, "failed to allocate input device");
		return -ENOMEM;
	}

	sdata->input_dev->name =
			ais2ih_input_sensor_name[sdata->sindex].description;
	sdata->input_dev->id.bustype = bustype;
	sdata->input_dev->dev.parent = sdata->cdata->dev;
	input_set_drvdata(sdata->input_dev, sdata);

	__set_bit(INPUT_EVENT_TYPE, sdata->input_dev->evbit);
	__set_bit(INPUT_EVENT_TIME_MSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_X, sdata->input_dev->mscbit);

	if (sdata->sindex == AIS2IH_ACCEL) {
		__set_bit(INPUT_EVENT_Y, sdata->input_dev->mscbit);
		__set_bit(INPUT_EVENT_Z, sdata->input_dev->mscbit);
	}

	err = input_register_device(sdata->input_dev);
	if (err) {
		dev_err(sdata->cdata->dev, "unable to register sensor %s\n",
			sdata->name);
		return err;
	}

	return err;
}

static void
ais2ih_input_input_cleanup(struct ais2ih_input_sensor_data *sdata)
{
	if (!sdata->input_dev) {
		pr_warn("Input device is already cleaned up.\n");
		return;
	}
	input_unregister_device(sdata->input_dev);
	sdata->input_dev = NULL; // Prevent further use
}

static void
ais2ih_input_report_3axes_event(struct ais2ih_input_sensor_data *sdata,
				s32 *xyz, s64 timestamp)
{
	struct input_dev *input = sdata->input_dev;

	if (!sdata->enabled)
		return;

	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    timestamp >> 32);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    timestamp & 0xffffffff);
	input_sync(input);
}

static void
ais2ih_input_report_single_event(struct ais2ih_input_sensor_data *sdata,
				 s32 data)
{
	struct input_dev  *input = sdata->input_dev;

	if (!sdata->enabled)
		return;

	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, data);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    sdata->timestamp >> 32);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    sdata->timestamp & 0xffffffff);
	input_sync(input);
}

static inline s32 ais2ih_input_data_align_bit(u8 ms, u8 ls, u8 power_mode)
{
	if (power_mode == AIS2IH_LP_MODE)
		return (s32)(((s16)(ls | ms << 8)) >> 4);

	return (s32)(((s16)(ls | ms << 8)) >> 2);
}

static u8 ais2ih_input_event_irq1_value(struct ais2ih_input_data *cdata)
{
	u8 value = 0x0;

	if (cdata->sensors[AIS2IH_FF].enabled)
		value |= AIS2IH_INT1_FREE_FALL_MASK;

	if (cdata->sensors[AIS2IH_DOUBLE_TAP].enabled)
		value |= AIS2IH_INT1_TAP_MASK;

	if (cdata->sensors[AIS2IH_TAP].enabled)
		value |= AIS2IH_INT1_S_TAP_MASK | AIS2IH_INT1_TAP_MASK;

	if (cdata->sensors[AIS2IH_WAKEUP].enabled)
		value |= AIS2IH_INT1_WAKEUP_MASK;

	return value;
}

static int
ais2ih_input_update_drdy_irq(struct ais2ih_input_sensor_data *sdata)
{
	u8 reg_addr = AIS2IH_CTRL4_INT1_PAD_ADDR, reg_val, reg_mask;

	switch (sdata->sindex) {
	case AIS2IH_FF:
	case AIS2IH_TAP:
	case AIS2IH_DOUBLE_TAP:
	case AIS2IH_WAKEUP:
		reg_val = ais2ih_input_event_irq1_value(sdata->cdata);
		reg_mask = AIS2IH_INT1_EVENTS_MASK;
		break;
	case AIS2IH_ACCEL:
		return 0;
	default:
		return -EINVAL;
	}

	return ais2ih_input_write_data_with_mask(sdata->cdata, reg_addr,
						 reg_mask,
						 reg_val >> __ffs(reg_mask),
						 true);
}

static int
ais2ih_input_set_fs(struct ais2ih_input_sensor_data *sdata, unsigned int fs)
{
	int err, i;

	for (i = 0; i < AIS2IH_FS_LIST_NUM; i++) {
		if (ais2ih_input_fs_table.fs_avl[i].urv == fs)
			break;
	}

	if (i == AIS2IH_FS_LIST_NUM)
		return -EINVAL;

	err = ais2ih_input_write_data_with_mask(sdata->cdata,
						ais2ih_input_fs_table.addr,
						ais2ih_input_fs_table.mask,
						ais2ih_input_fs_table.fs_avl[i].value,
						true);
	if (err < 0)
		return err;

	sdata->c_gain =
	       ais2ih_input_fs_table.fs_avl[i].gain[sdata->cdata->power_mode];

	return 0;
}

static inline int64_t ais2ih_input_get_time_ns(void)
{
	return ktime_to_ns(ktime_get_boottime());
}

/* Acc data */
static int ais2ih_input_get_acc_data(struct ais2ih_input_data *cdata)
{
	u8 data[AIS2IH_OUT_XYZ_SIZE];
	int err, xyz[3];
	struct ais2ih_input_sensor_data
				*sdata = &cdata->sensors[AIS2IH_ACCEL];

	err = cdata->tf->read(cdata,
			      AIS2IH_OUTX_L_ADDR,
			      AIS2IH_OUT_XYZ_SIZE,
			      data, true);
	if (err < 0) {
		dev_err(cdata->dev, "get acc data failed %d\n", err);
		return err;
	}

	xyz[0] = ais2ih_input_data_align_bit(data[1],
					     data[0],
					     cdata->power_mode);
	xyz[1] = ais2ih_input_data_align_bit(data[3],
					     data[2],
					     cdata->power_mode);
	xyz[2] = ais2ih_input_data_align_bit(data[5],
					     data[4],
					     cdata->power_mode);

	xyz[0] *= sdata->c_gain;
	xyz[1] *= sdata->c_gain;
	xyz[2] *= sdata->c_gain;

	ais2ih_input_report_3axes_event(sdata, xyz, sdata->timestamp);

	return 0;
}

static void
ais2ih_input_acc_poll_function_work(struct work_struct *input_work)
{
	struct ais2ih_input_sensor_data *sdata;

	sdata = container_of((struct work_struct *)input_work,
			     struct ais2ih_input_sensor_data, input_work);
	ais2ih_input_get_acc_data(sdata->cdata);
}

static enum
hrtimer_restart ais2ih_input_hrtimer_acc_callback(struct hrtimer *timer)
{
	struct ais2ih_input_sensor_data *sdata;

	sdata = container_of((struct hrtimer *)timer,
			     struct ais2ih_input_sensor_data,
			     hr_timer);

	sdata->timestamp = ais2ih_input_get_time_ns();

	queue_work(ais2ih_input_workqueue, &sdata->input_work);

	hrtimer_forward(timer, ktime_get(), sdata->oldktime);

	return HRTIMER_RESTART;
}

/* Events data */
static irqreturn_t ais2ih_input_thread_fn(int irq, void *private)
{
	u8 status;
	struct ais2ih_input_data *cdata = private;

	cdata->tf->read(cdata, AIS2IH_STATUS_ADDR, 1, &status, true);

	if (status & AIS2IH_EVENT_MASK) {
		if ((cdata->sensors[AIS2IH_TAP].enabled) &&
			(status & AIS2IH_SINGLE_TAP_MASK)) {
			cdata->sensors[AIS2IH_TAP].timestamp = cdata->timestamp;
			ais2ih_input_report_single_event(&cdata->sensors[AIS2IH_TAP], 1);
		}

		if ((cdata->sensors[AIS2IH_DOUBLE_TAP].enabled) &&
			(status & AIS2IH_DOUBLE_TAP_MASK)) {
			cdata->sensors[AIS2IH_DOUBLE_TAP].timestamp = cdata->timestamp;
			ais2ih_input_report_single_event(&cdata->sensors[AIS2IH_DOUBLE_TAP], 1);
		}

		if ((cdata->sensors[AIS2IH_FF].enabled) &&
			(status & AIS2IH_FF_IA_MASK)) {
			cdata->sensors[AIS2IH_FF].timestamp = cdata->timestamp;
			ais2ih_input_report_single_event(&cdata->sensors[AIS2IH_FF], 1);
		}

		if ((cdata->sensors[AIS2IH_WAKEUP].enabled) &&
			(status & AIS2IH_WAKE_UP_IA_MASK)) {
			cdata->sensors[AIS2IH_WAKEUP].timestamp = cdata->timestamp;
			ais2ih_input_report_single_event(&cdata->sensors[AIS2IH_WAKEUP], 1);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t ais2ih_input_save_timestamp(int irq, void *private)
{
	struct ais2ih_input_data *cdata = (struct ais2ih_input_data *)private;

	cdata->timestamp = ais2ih_input_get_time_ns();

	return IRQ_WAKE_THREAD;
}

static int ais2ih_input_update_hw_odr(struct ais2ih_input_sensor_data *sdata,
				      unsigned int odr_hz)
{
	int err, i;
	struct ais2ih_input_data *cdata = sdata->cdata;
	u8 num_aval;

	/* odr num. may differs for LP and HR mode */
	num_aval = (cdata->power_mode == AIS2IH_LP_MODE) ?
			AIS2IH_ODR_LP_LIST_NUM : AIS2IH_ODR_HR_LIST_NUM;

	for (i = 0; i < num_aval; i++) {
		if (ais2ih_input_odr_table.odr_avl[cdata->power_mode][i].hz >= odr_hz)
			break;
	}

	if (i == num_aval)
		return -EINVAL;

	if (sdata->c_odr ==
		    ais2ih_input_odr_table.odr_avl[cdata->power_mode][i].hz)
		return 0;

	err = ais2ih_input_write_data_with_mask(sdata->cdata,
					  AIS2IH_CTRL1_ADDR,
					  AIS2IH_ODR_MASK,
					  ais2ih_input_odr_table.odr_avl[cdata->power_mode][i].value,
					  true);
	if (err < 0)
		return err;

	sdata->c_odr =
		    ais2ih_input_odr_table.odr_avl[cdata->power_mode][i].hz;

	return 0;
}

static int
ais2ih_input_configure_tap_event(struct ais2ih_input_sensor_data *sdata,
				 bool single_tap)
{
	u8 err = 0;

	if (single_tap) {
		err = ais2ih_input_write_data_with_mask(sdata->cdata,
						AIS2IH_INT_DUR_ADDR,
						AIS2IH_INT_DUR_MASK,
						AIS2IH_INT_DUR_STAP_DEFAULT,
						true);
		if (err < 0)
			return err;

		err = ais2ih_input_write_data_with_mask(sdata->cdata,
						AIS2IH_WAKE_UP_THS_ADDR,
						AIS2IH_WAKE_UP_THS_S_D_TAP_MASK,
						AIS2IH_DIS_BIT, true);
		if (err < 0)
			return err;
	} else {
		err = ais2ih_input_write_data_with_mask(sdata->cdata,
						AIS2IH_INT_DUR_ADDR,
						AIS2IH_INT_DUR_MASK,
						AIS2IH_INT_DUR_DTAP_DEFAULT,
						true);
		if (err < 0)
			return err;

		err = ais2ih_input_write_data_with_mask(sdata->cdata,
						AIS2IH_WAKE_UP_THS_ADDR,
						AIS2IH_WAKE_UP_THS_S_D_TAP_MASK,
						AIS2IH_EN_BIT, true);
		if (err < 0)
			return err;
	}

	return err;
}

static int
_ais2ih_input_enable_sensors(struct ais2ih_input_sensor_data *sdata)
{
	int err = 0, i;
	struct ais2ih_input_data *cdata = sdata->cdata;
	u8 num_aval;
	int64_t newTime_us;

	err = ais2ih_input_update_drdy_irq(sdata);
	if (err < 0)
		return err;

	switch (sdata->sindex) {
	case AIS2IH_ACCEL:
		num_aval = (cdata->power_mode == AIS2IH_LP_MODE) ?
			AIS2IH_ODR_LP_LIST_NUM : AIS2IH_ODR_HR_LIST_NUM;

		for (i = 0; i < num_aval; i++) {
			if (ais2ih_input_odr_table.odr_avl[cdata->power_mode][i].hz >= sdata->c_odr)
				break;
		}

		if (i == num_aval)
			return -EINVAL;

		err = ais2ih_input_write_data_with_mask(sdata->cdata,
						AIS2IH_CTRL1_ADDR,
						AIS2IH_ODR_MASK,
						ais2ih_input_odr_table.odr_avl[cdata->power_mode][i].value,
						true);
		if (err < 0)
			return err;

		err = ais2ih_input_write_data_with_mask(sdata->cdata,
						AIS2IH_CTRL1_ADDR,
						AIS2IH_PWR_OPER_MODE_MASK,
						cdata->power_mode,
						true);
		if (err < 0)
			return err;

		sdata->c_odr =
		      ais2ih_input_odr_table.odr_avl[cdata->power_mode][i].hz;
		newTime_us = US_TO_NS(sdata->poll_us);
		sdata->oldktime = ktime_set(0, newTime_us);
		hrtimer_start(&sdata->hr_timer, sdata->oldktime,
			      HRTIMER_MODE_REL);
		break;
	case AIS2IH_TAP:
		ais2ih_input_configure_tap_event(sdata, 1);
		break;
	case AIS2IH_DOUBLE_TAP:
		ais2ih_input_configure_tap_event(sdata, 0);
		break;
	case AIS2IH_FF:
	case AIS2IH_WAKEUP:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int
ais2ih_input_enable_sensors(struct ais2ih_input_sensor_data *sdata)
{
	int err = 0;

	if (sdata->enabled)
		return 0;

	sdata->enabled = true;
	err = _ais2ih_input_enable_sensors(sdata);
	if (err < 0)
		sdata->enabled = false;

	return err;
}

static int
_ais2ih_input_disable_sensors(struct ais2ih_input_sensor_data *sdata)
{
	int err;

	switch (sdata->sindex) {
	case AIS2IH_ACCEL:
		err = ais2ih_input_write_data_with_mask(sdata->cdata,
					AIS2IH_CTRL1_ADDR,
					AIS2IH_ODR_MASK,
					ais2ih_input_odr_table.odr_avl[sdata->cdata->power_mode][0].value,
					true);
		if (err < 0)
			return err;

		hrtimer_cancel(&sdata->hr_timer);
		cancel_work_sync(&sdata->input_work);
		break;
	case AIS2IH_FF:
	case AIS2IH_TAP:
	case AIS2IH_DOUBLE_TAP:
	case AIS2IH_WAKEUP:
		break;
	default:
		return -EINVAL;
	}

	err = ais2ih_input_update_drdy_irq(sdata);
	if (err < 0)
		return err;

	return 0;
}

static int
ais2ih_input_disable_sensors(struct ais2ih_input_sensor_data *sdata)
{
	int err;

	if (!sdata->enabled)
		return 0;

	sdata->enabled = false;
	err = _ais2ih_input_disable_sensors(sdata);
	if (err < 0)
		sdata->enabled = true;

	return err;
}

static int
ais2ih_input_allocate_workqueue_and_interrupt(struct ais2ih_input_data *cdata)
{
	int err = 0;

	if (!ais2ih_input_workqueue)
		ais2ih_input_workqueue =
			alloc_workqueue("ais2ih_input_workqueue",
					WQ_UNBOUND | WQ_HIGHPRI, 0);

	if (!ais2ih_input_workqueue) {
		dev_err(cdata->dev, "Failed to allocate workqueue\n");
		return -ENOMEM;
	}

	if (cdata->irq) {
		err = devm_request_threaded_irq(cdata->dev, cdata->irq,
					 ais2ih_input_save_timestamp,
					 ais2ih_input_thread_fn,
					 IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					 cdata->name, cdata);

		if (err < 0) {
			dev_err(cdata->dev, "Failed to request IRQ. irq=%d\n",
				cdata->irq);
			return err;
		}
	} else {
		dev_info(cdata->dev, "IRQ not requested. irq=%d\n", cdata->irq);
	}
	return err;
}

static int ais2ih_input_init_sensors(struct ais2ih_input_data *cdata)
{
	int err, i;
	struct ais2ih_input_sensor_data *sdata;

	for (i = 0; i < AIS2IH_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];

		err = ais2ih_input_disable_sensors(sdata);
		if (err < 0)
			return err;

		if (sdata->sindex == AIS2IH_ACCEL) {
			err = ais2ih_input_set_fs(sdata, AIS2IH_ACCEL_FS);
			if (err < 0)
				return err;
		}
	}

	hrtimer_init(&cdata->sensors[AIS2IH_ACCEL].hr_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	cdata->sensors[AIS2IH_ACCEL].hr_timer.function =
					&ais2ih_input_hrtimer_acc_callback;
	cdata->selftest_status = 0;
	err = ais2ih_input_write_data_with_mask(cdata,
					  AIS2IH_CTRL2_ADDR,
					  AIS2IH_SOFT_RESET_MASK,
					  AIS2IH_EN_BIT, true);
	if (err < 0)
		return err;

	/* Wait for the device to reset */
	usleep_range(5, 10);

	err = ais2ih_input_write_data_with_mask(cdata,
					  AIS2IH_CTRL1_ADDR,
					  AIS2IH_LP_MODE_MASK,
					  AIS2IH_EN_LP_MODE_02, true);
	if (err < 0)
		return err;

	err = ais2ih_input_write_data_with_mask(cdata,
					  AIS2IH_CTRL3_ADDR,
					  AIS2IH_LIR_MASK,
					  AIS2IH_EN_BIT, true);
	if (err < 0)
		return err;

	err = ais2ih_input_write_data_with_mask(cdata,
					  AIS2IH_CTRL2_ADDR,
					  AIS2IH_BDU_MASK,
					  AIS2IH_EN_BIT, true);
	if (err < 0)
		return err;

	err = ais2ih_input_write_data_with_mask(sdata->cdata,
					  AIS2IH_FREE_FALL_ADDR,
					  AIS2IH_FREE_FALL_THS_MASK,
					  AIS2IH_FREE_FALL_THS_DEFAULT, true);
	if (err < 0)
		return err;

	err = ais2ih_input_write_data_with_mask(sdata->cdata,
					  AIS2IH_FREE_FALL_ADDR,
					  AIS2IH_FREE_FALL_DUR_MASK,
					  AIS2IH_FREE_FALL_DUR_DEFAULT, true);
	if (err < 0)
		return err;

	err = ais2ih_input_write_data_with_mask(sdata->cdata,
					  AIS2IH_TAP_THS_Z_ADDR,
					  AIS2IH_TAP_AXIS_MASK,
					  AIS2IH_TAP_AXIS_ENABLE_ALL, true);
	if (err < 0)
		return err;

	err = ais2ih_input_write_data_with_mask(sdata->cdata,
					  AIS2IH_TAP_THS_X_ADDR,
					  AIS2IH_TAP_THS_MASK,
					  AIS2IH_TAP_THS_DEFAULT, true);
	if (err < 0)
		return err;

	err = ais2ih_input_write_data_with_mask(sdata->cdata,
					  AIS2IH_WAKE_UP_THS_ADDR,
					  AIS2IH_WAKE_UP_THS_WU_MASK,
					  AIS2IH_WAKE_UP_THS_WU_DEFAULT, true);
	if (err < 0)
		return err;

	cdata->sensors[AIS2IH_ACCEL].oldktime = ktime_set(0,
			US_TO_NS(cdata->sensors[AIS2IH_ACCEL].poll_us));
	INIT_WORK(&cdata->sensors[AIS2IH_ACCEL].input_work,
		  ais2ih_input_acc_poll_function_work);

	return 0;
}

static ssize_t show_enable(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct ais2ih_input_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->enabled);
}

static ssize_t store_enable(struct device *dev, struct device_attribute *attr,
			    const char *buf,
			    size_t count)
{
	struct ais2ih_input_sensor_data *sdata = dev_get_drvdata(dev);
	unsigned long enable;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		ais2ih_input_enable_sensors(sdata);
	else
		ais2ih_input_disable_sensors(sdata);

	return count;
}

static ssize_t show_resolution_mode(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct ais2ih_input_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n",
		(sdata->cdata->power_mode == AIS2IH_LP_MODE) ? "low" : "high");
}

static ssize_t store_resolution_mode(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	int i;
	struct ais2ih_input_sensor_data *sdata = dev_get_drvdata(dev);

	for (i = 0; i < AIS2IH_FS_LIST_NUM; i++) {
		if (sdata->c_gain ==
			ais2ih_input_fs_table.fs_avl[i].gain[sdata->cdata->power_mode])
			break;
	}

	if (!strncmp(buf, "low", count - 1))
		sdata->cdata->power_mode = AIS2IH_LP_MODE;
	else if (!strncmp(buf, "high", count - 1))
		sdata->cdata->power_mode = AIS2IH_HR_MODE;
	else
		return -EINVAL;

	sdata->c_gain =
		ais2ih_input_fs_table.fs_avl[i].gain[sdata->cdata->power_mode];

	return count;
}

static ssize_t show_polling_rate(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct ais2ih_input_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->poll_us);
}

static ssize_t store_polling_rate(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int err;
	unsigned int polling_rate_us;
	struct ais2ih_input_sensor_data *sdata = dev_get_drvdata(dev);
	int64_t newTime_ns;

	err = kstrtoint(buf, 10, &polling_rate_us);
	if (err < 0)
		return err;

	if (polling_rate_us < AIS2IH_POLLRATE_MIN_USEC || \
			polling_rate_us > AIS2IH_POLLRATE_CLAMP_USEC) {

		dev_err(sdata->cdata->dev, "Invalid polling rate: %u\n",
			polling_rate_us);
		return -EINVAL;
	}

	mutex_lock(&sdata->input_dev->mutex);
	err = ais2ih_input_update_hw_odr(sdata, 1000000 / polling_rate_us);
	if (err < 0)
		goto err_poll;

	sdata->poll_us = polling_rate_us;
	newTime_ns = US_TO_NS(sdata->poll_us);
	sdata->oldktime = ktime_set(0, newTime_ns);
	mutex_unlock(&sdata->input_dev->mutex);

	return count;

err_poll:
	mutex_unlock(&sdata->input_dev->mutex);

	return err;
}

static ssize_t show_scale_avail(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int i, len = 0;

	for (i = 0; i < AIS2IH_FS_LIST_NUM; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
			ais2ih_input_fs_table.fs_avl[i].urv);

	buf[len - 1] = '\n';

	return len;
}

static ssize_t show_current_scale(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	int i;
	struct ais2ih_input_sensor_data *sdata = dev_get_drvdata(dev);

	for (i = 0; i < AIS2IH_FS_LIST_NUM; i++)
		if (sdata->c_gain ==
			ais2ih_input_fs_table.fs_avl[i].gain[sdata->cdata->power_mode])
			break;

	return sprintf(buf, "%d\n", ais2ih_input_fs_table.fs_avl[i].urv);
}

static ssize_t store_current_scale(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int urv, err;
	struct ais2ih_input_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &urv);
	if (err < 0)
		return err;

	err = ais2ih_input_set_fs(sdata, urv);
	if (err < 0)
		return err;

	return count;
}

static ssize_t show_internal_current_odr(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct ais2ih_input_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->c_odr);
}

static DEVICE_ATTR(enable,
		   0644,
		   show_enable,
		   store_enable);

static DEVICE_ATTR(resolution,
		   0644,
		   show_resolution_mode,
		   store_resolution_mode);

static DEVICE_ATTR(polling_rate_usec,
		   0644,
		   show_polling_rate,
		   store_polling_rate);

/* The scale_avail attribute is intentionally read-only as it only provides
 * available scale values.*/
static DEVICE_ATTR(scale_avail,
		   0444,
		   show_scale_avail,
		   NULL);

static DEVICE_ATTR(scale,
		   0644,
		   show_current_scale,
		   store_current_scale);

static DEVICE_ATTR(internal_odr_hz,
		    0444,
		    show_internal_current_odr,
		    NULL);

static struct attribute *ais2ih_input_accel_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_resolution.attr,
	&dev_attr_polling_rate_usec.attr,
	&dev_attr_internal_odr_hz.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *ais2ih_input_step_ff_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *ais2ih_input_tap_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *ais2ih_input_double_tap_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *ais2ih_input_wakeup_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group ais2ih_input_attribute_groups[] = {
	[AIS2IH_ACCEL] = {
		.name = AIS2IH_DEV_NAME"_accel",
		.attrs = ais2ih_input_accel_attribute,
	},
	[AIS2IH_FF] = {
		.name = AIS2IH_DEV_NAME"_free_fall",
		.attrs = ais2ih_input_step_ff_attribute,
	},
	[AIS2IH_TAP] = {
		.name = AIS2IH_DEV_NAME"_tap",
		.attrs = ais2ih_input_tap_attribute,
	},
	[AIS2IH_DOUBLE_TAP] = {
		.name = AIS2IH_DEV_NAME"_double_tap",
		.attrs = ais2ih_input_double_tap_attribute,
	},
	[AIS2IH_WAKEUP] = {
		.name = AIS2IH_DEV_NAME"_wake_up",
		.attrs = ais2ih_input_wakeup_attribute,
	},
};

#if IS_ENABLED(CONFIG_OF)
static u32 ais2ih_input_parse_dt(struct ais2ih_input_data *cdata)
{
	u32 val;
	struct device_node *np;

	np = cdata->dev->of_node;
	if (!np)
		return -EINVAL;

	if (!of_property_read_u32(np, "st,drdy-int-pin", &val) &&
	    (val <= 2) && (val > 0))
		cdata->drdy_int_pin = (u8)val;
	else
		cdata->drdy_int_pin = 1;

	return 0;
}
#endif /* CONFIG_OF */

int ais2ih_input_common_probe(struct ais2ih_input_data *cdata, int irq,
			      u16 bustype)
{
	int32_t err, i;
	u8 wai = 0;
	struct ais2ih_input_sensor_data *sdata;

	mutex_init(&cdata->bank_registers_lock);
	mutex_init(&cdata->tb.buf_lock);

	err = cdata->tf->read(cdata, AIS2IH_WHO_AM_I_ADDR, 1, &wai, true);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register.\n");
		return err;
	}
	if (wai != AIS2IH_WHO_AM_I_DEF) {
		dev_err(cdata->dev, "Who-Am-I value not valid.\n");
		return -ENODEV;
	}

	if (irq > 0) {

		cdata->irq = irq;

#if IS_ENABLED(CONFIG_OF)
		dev_info(cdata->dev, "dt enabled: parsing dt info");
		err = ais2ih_input_parse_dt(cdata);
		if (err < 0) {
			dev_err(cdata->dev, "error parsing dt:%d", err);
			return err;
		}
#else /* CONFIG_OF */
		dev_info(cdata->dev, "dt not enabled: using platform data info");
		if (cdata->dev->platform_data) {
			cdata->drdy_int_pin = ((struct ais2ih_input_platform_data *)
				cdata->dev->platform_data)->drdy_int_pin;

			if ((cdata->drdy_int_pin > 2) ||
						      (cdata->drdy_int_pin < 1))
				cdata->drdy_int_pin = 1;
		} else {
			cdata->drdy_int_pin = 1;
			dev_info(cdata->dev,
				"platform_data not found, assigning default drdy_int_pin");
		}
#endif /* CONFIG_OF */

		dev_info(cdata->dev, "driver use DRDY int pin %d\n",
			 cdata->drdy_int_pin);
	} else {

		cdata->irq = 0;
		dev_warn(cdata->dev,
			"IRQ not available! driver enables limited features");
	}

	cdata->power_mode = AIS2IH_LP_MODE;

	for (i = 0; i < AIS2IH_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];
		sdata->enabled = false;
		sdata->cdata = cdata;
		sdata->sindex = i;
		sdata->name = ais2ih_input_sensor_name[i].name;

		if (i == AIS2IH_ACCEL) {
			sdata->c_odr =
			    ais2ih_input_odr_table.odr_avl[cdata->power_mode][1].hz;
			sdata->poll_us = 1000 * 1000 / sdata->c_odr;
		}

		ais2ih_input_input_init(sdata, bustype);

		if (sysfs_create_group(&sdata->input_dev->dev.kobj,
				       &ais2ih_input_attribute_groups[i])) {
			dev_err(cdata->dev,
				"failed to create sysfs group for sensor %s",
				sdata->name);

			input_unregister_device(sdata->input_dev);
			sdata->input_dev = NULL;
		}
	}

	err = ais2ih_input_init_sensors(cdata);
	if (err < 0)
		return err;

	err = ais2ih_input_allocate_workqueue_and_interrupt(cdata);
	if (err)
		return err;

	dev_info(cdata->dev, "%s: probed\n", AIS2IH_DEV_NAME);

	return 0;
}
EXPORT_SYMBOL(ais2ih_input_common_probe);

void ais2ih_input_common_remove(struct ais2ih_input_data *cdata, int irq)
{
	u8 i;

	for (i = 0; i < AIS2IH_SENSORS_NUMB; i++) {
		ais2ih_input_disable_sensors(&cdata->sensors[i]);
		ais2ih_input_input_cleanup(&cdata->sensors[i]);
	}

	if (ais2ih_input_workqueue) {
		flush_workqueue(ais2ih_input_workqueue);
		destroy_workqueue(ais2ih_input_workqueue);
		ais2ih_input_workqueue = NULL;
	}
}
EXPORT_SYMBOL(ais2ih_input_common_remove);

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int ais2ih_input_resume_sensors(struct ais2ih_input_sensor_data *sdata)
{
	if (!sdata->enabled)
		return 0;

	return _ais2ih_input_enable_sensors(sdata);
}

static int ais2ih_input_suspend_sensors(struct ais2ih_input_sensor_data *sdata)
{
	if (!sdata->enabled)
		return 0;

	return _ais2ih_input_disable_sensors(sdata);
}

int ais2ih_input_common_suspend(struct ais2ih_input_data *cdata)
{
	ais2ih_input_suspend_sensors(&cdata->sensors[AIS2IH_ACCEL]);

	return 0;
}
EXPORT_SYMBOL(ais2ih_input_common_suspend);

int ais2ih_input_common_resume(struct ais2ih_input_data *cdata)
{
	ais2ih_input_resume_sensors(&cdata->sensors[AIS2IH_ACCEL]);

	return 0;
}
EXPORT_SYMBOL(ais2ih_input_common_resume);

#endif	/* CONFIG_PM_SLEEP */

MODULE_DESCRIPTION("STMicroelectronics ais2ih input driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
