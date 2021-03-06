/*
 * Copyright (C) 2015 Atmark Techno, Inc. All Rights Reserved.
 *
 * Based on:
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <asm/sizes.h>

#include "../hardware.h"
#include "devices-common.h"

struct platform_device *__init imx_add_w1_gpio(int id,
		const struct w1_gpio_platform_data *pdata)
{
	return imx_add_platform_device("w1-gpio", id, NULL,
		 0, pdata, sizeof(*pdata));
}
