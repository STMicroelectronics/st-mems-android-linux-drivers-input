/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * STMicroelectronics lsm6dsm driver
 *
 * Copyright 2010-2026 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.2.2
 * Licensed under the GPL-2.
 */

#ifndef __LSM6DSM_PLATFORM_DATA_
#define __LSM6DSM_PLATFORM_DATA_
struct lsm6dsm_platform_data {
	int drdy_int_pin;
};
#endif /* __LSM6DSM_PLATFORM_DATA_ */

