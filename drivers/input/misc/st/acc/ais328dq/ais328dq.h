/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2010-2026 STMicroelectronics
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
 * File Name		: ais328dq.h
 * Authors		: VMA - Volume Mems & Analog Division
 *			: Matteo Dameno (matteo.dameno@st.com)
 *			: Author is willing to be considered the contact
 *			: and update point for the driver.
 * Version		: V 1.0.1
 * Date			: 2012/10/07
 * Description		: AIS328DQ 3D accelerometer sensor LDD
 *
 *****************************************************************************/

#ifndef __AIS328DQ_H__
#define __AIS328DQ_H__

#include <linux/kconfig.h>

#define AIS328DQ_ACC_I2C_SAD_L		(0x18)
#define AIS328DQ_ACC_I2C_SAD_H		(0x19)
#define	AIS328DQ_ACC_DEV_NAME	"ais328dq_acc"



/************************************************/
/* 	Accelerometer section defines	 	*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define AIS328DQ_ACC_FS_MASK		0x30
#define AIS328DQ_ACC_G_2G 		0x00
#define AIS328DQ_ACC_G_4G 		0x10
#define AIS328DQ_ACC_G_8G 		0x30

/* Accelerometer Sensor Operating Mode */
#define AIS328DQ_ACC_ENABLE		0x01
#define AIS328DQ_ACC_DISABLE		0x00
#define AIS328DQ_ACC_PM_NORMAL		0x20
#define AIS328DQ_ACC_PM_OFF		AIS328DQ_ACC_DISABLE

#ifdef __KERNEL__

#if IS_ENABLED(CONFIG_INPUT_AIS328DQ_SPI)
#define AIS328DQ_RX_MAX_LENGTH	500
#define AIS328DQ_TX_MAX_LENGTH	500

struct ais328dq_transfer_buffer {
	u8 rx_buf[AIS328DQ_RX_MAX_LENGTH];
	u8 tx_buf[AIS328DQ_TX_MAX_LENGTH] ____cacheline_aligned;
};
#endif

struct ais328dq_transfer_function {
	int (*write)(struct device *dev, u8 reg_addr, int len, u8 *data);
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
};
struct ais328dq_acc_platform_data {

	int poll_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
};

#define	RESUME_ENTRIES		12

struct ais328dq_acc_data {
	const char *name;
	struct device *dev;
	u16 bus_type;
	struct ais328dq_acc_platform_data *pdata;

	struct mutex lock;
	s64 timestamp, delta_ts;
	struct hrtimer hr_timer;
	struct workqueue_struct *work_queue;
	struct work_struct poll_work;
	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	int selftest_enabled;

	atomic_t enabled;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

#ifdef AIS328DQ_DEBUG
	u8 reg_addr;
#endif
#if IS_ENABLED(CONFIG_INPUT_AIS328DQ_SPI)
	struct ais328dq_transfer_buffer tb;
#endif
	struct ais328dq_transfer_function *tf;
};

int ais328dq_acc_probe(struct ais328dq_acc_data *acc);
int ais328dq_acc_remove(struct ais328dq_acc_data *acc);
int ais328dq_acc_enable(struct ais328dq_acc_data *acc);
int ais328dq_acc_disable(struct ais328dq_acc_data *acc);

#endif /* __KERNEL__ */

#endif  /* __AIS328DQ_H__ */
