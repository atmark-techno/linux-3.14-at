/*
 * Copyright (C) 2015 Atmark Techno .Inc
 * Hiroaki OHSAWA <ohsawa@atmark-techo.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include "../hardware.h"
#include "devices-common.h"

#define imx_adc_data_entry_single(soc, _size)		\
	{						\
		.iobase = soc ## _TSC_BASE_ADDR,	\
		.iosize = _size,			\
		.irq    = soc ## _INT_TSC		\
	}

#ifdef CONFIG_SOC_IMX25
const struct imx_adc_data imx25_adc_data __initconst =
	imx_adc_data_entry_single(MX25, SZ_16K);
#endif /* ifdef CONFIG_SOC_IMX25 */

struct platform_device *__init imx_add_adc(
	const struct imx_adc_data *data)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + data->iosize - 1,
			.flags = IORESOURCE_MEM,
		},
		{
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},

	};
	return imx_add_platform_device("imx_adc", -1,
				       res, ARRAY_SIZE(res), NULL, 0);
}
