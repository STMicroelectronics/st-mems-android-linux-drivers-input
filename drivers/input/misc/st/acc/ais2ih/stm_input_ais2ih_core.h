/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * STMicroelectronics ais2ih input driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2025 STMicroelectronics Inc.
 */

#ifndef DRIVERS_INPUT_MISC_AIS2IH_CORE_H_
#define DRIVERS_INPUT_MISC_AIS2IH_CORE_H_

#define AIS2IH_DEV_NAME		"ais2ih"

#define HZ_TO_PERIOD_NSEC(hz)	(1000 * 1000 * 1000 / ((u32)(hz)))
#define MS_TO_US(x)		({ typeof(x) _x = (x); ((_x) * \
				 ((typeof(x)) 1000)); })
#define US_TO_NS(x)		(MS_TO_US(x))
#define MS_TO_NS(x)		(US_TO_NS(MS_TO_US(x)))
#define US_TO_MS(x)		({ typeof(x) _x = (x); ((_x) / \
				 ((typeof(x)) 1000)); })
#define NS_TO_US(x)		(US_TO_MS(x))
#define NS_TO_MS(x)		(US_TO_MS(NS_TO_US(x)))

enum {
	AIS2IH_ACCEL = 0,
	AIS2IH_FF,
	AIS2IH_TAP,
	AIS2IH_DOUBLE_TAP,
	AIS2IH_WAKEUP,
	AIS2IH_SENSORS_NUMB,
};

#define INPUT_EVENT_TYPE	EV_MSC
#define INPUT_EVENT_X		MSC_SERIAL
#define INPUT_EVENT_Y		MSC_PULSELED
#define INPUT_EVENT_Z		MSC_GESTURE
#define INPUT_EVENT_TIME_MSB	MSC_SCAN
#define INPUT_EVENT_TIME_LSB	MSC_MAX

#define AIS2IH_RX_MAX_LENGTH	16
#define AIS2IH_TX_MAX_LENGTH	16

#define AIS2IH_I2C_MAX_BUFFER_SIZE	32

#define to_dev(obj) container_of(obj, struct device, kobj)

struct reg_rw {
	u8 const address;
	u8 const init_val;
	u8 resume_val;
};

struct reg_r {
	const u8 address;
	const u8 init_val;
};

struct ais2ih_input_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[AIS2IH_RX_MAX_LENGTH];
	u8 tx_buf[AIS2IH_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct ais2ih_input_data;

struct ais2ih_input_transfer_function {
	int (*write)(struct ais2ih_input_data *cdata, u8 reg_addr, int len,
		     u8 *data, bool b_lock);
	int (*read)(struct ais2ih_input_data *cdata, u8 reg_addr, int len,
		    u8 *data, bool b_lock);
};

struct ais2ih_input_sensor_data {
	struct ais2ih_input_data *cdata;
	const char *name;
	s64 timestamp;
	u8 enabled;
	u32 c_odr;
	u32 c_gain;
	u8 sindex;
	unsigned int poll_us;
	struct input_dev *input_dev;
	struct hrtimer hr_timer;
	struct work_struct input_work;
	ktime_t oldktime;
};

struct ais2ih_input_data {
	const char *name;
	u8 drdy_int_pin;
	u8 selftest_status;
	u8 power_mode;
	int irq;
	s64 timestamp;
	struct device *dev;
	struct ais2ih_input_sensor_data sensors[AIS2IH_SENSORS_NUMB];
	struct mutex bank_registers_lock;
	const struct ais2ih_input_transfer_function *tf;
	struct ais2ih_input_transfer_buffer tb;
};

int ais2ih_input_common_probe(struct ais2ih_input_data *cdata, int irq,
			      u16 bustype);
void ais2ih_input_common_remove(struct ais2ih_input_data *cdata, int irq);

#ifdef CONFIG_PM_SLEEP
int ais2ih_input_common_suspend(struct ais2ih_input_data *cdata);
int ais2ih_input_common_resume(struct ais2ih_input_data *cdata);
#endif /* CONFIG_PM_SLEEP */

#endif /* DRIVERS_INPUT_MISC_AIS2IH_CORE_H_ */
