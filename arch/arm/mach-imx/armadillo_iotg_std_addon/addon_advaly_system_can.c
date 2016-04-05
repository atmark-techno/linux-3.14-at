/*
 * Copyright (C) 2015 Atmark Techno, Inc. All Rights Reserved.
 * Copyright (C) 2015 ADVALY SYSTEM CO., LTD. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pinctrl/machine.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/memory.h>
#include <asm/mach/map.h>

#include "../common.h"
#include "../mx25.h"

#include "armadillo_iotg_std_addon.h"
#include "../imx25-named_gpio.h"
#include "../iomux-mx25.h"
#include "../hardware.h"
#include "../devices-imx25.h"

static unsigned long pin_cfgs_100kup[] = {
	PAD_CTL_PUS_100K_UP,
};

static unsigned long pin_cfgs_none[] = {
	0,
};



static iomux_v3_cfg_t addon_pinctrl_pads_con1[] = {
	MX25_PAD_GPIO_F__GPIO_F,
};

static iomux_v3_cfg_t addon_pinctrl_pads_con2[] = {
	MX25_PAD_RTCK__GPIO_3_14,
};


static struct {
	iomux_v3_cfg_t *pads;
	size_t nr_pads;
	struct addon_gpio power;
} addon_data[NR_ADDON_INTERFACES] = {
	[ADDON_INTERFACE1] = {
		.pads		= addon_pinctrl_pads_con1,
		.nr_pads	= ARRAY_SIZE(addon_pinctrl_pads_con1),
		.power		= ADDON_GPIO(IMX_GPIO_NR(1, 5), "CAN_POWER_CON1"),
	},
	[ADDON_INTERFACE2] = {
		.pads		= addon_pinctrl_pads_con2,
		.nr_pads	= ARRAY_SIZE(addon_pinctrl_pads_con2),
		.power		= ADDON_GPIO(IMX_GPIO_NR(3, 14), "CAN_POWER_CON2"),
	},
};


static struct regulator_consumer_supply flexcan0_dummy_supplies[] = {
	REGULATOR_SUPPLY("xceiver", "flexcan.0"),
};

static struct regulator_consumer_supply flexcan1_dummy_supplies[] = {
	REGULATOR_SUPPLY("xceiver", "flexcan.1"),
};


static const struct pinctrl_map advaly_system_can_map[] = {
	/* can1 */
	PIN_MAP_MUX_GROUP_DEFAULT("flexcan.0", "imx25-pinctrl.0",
				  "gpio_a__can1_tx", "can1"),
	PIN_MAP_MUX_GROUP_DEFAULT("flexcan.0", "imx25-pinctrl.0",
				  "gpio_b__can1_rx", "can1"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("flexcan.0", "imx25-pinctrl.0",
				    "MX25_PAD_GPIO_A", pin_cfgs_none),
	PIN_MAP_CONFIGS_PIN_DEFAULT("flexcan.0", "imx25-pinctrl.0",
				    "MX25_PAD_GPIO_B", pin_cfgs_100kup),
	/* can2 */
	PIN_MAP_MUX_GROUP_DEFAULT("flexcan.1", "imx25-pinctrl.0",
				  "gpio_c__can2_tx", "can2"),
	PIN_MAP_MUX_GROUP_DEFAULT("flexcan.1", "imx25-pinctrl.0",
				  "gpio_d__can2_rx", "can2"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("flexcan.1", "imx25-pinctrl.0",
				    "MX25_PAD_GPIO_C", pin_cfgs_none),
	PIN_MAP_CONFIGS_PIN_DEFAULT("flexcan.1", "imx25-pinctrl.0",
				    "MX25_PAD_GPIO_D", pin_cfgs_100kup),
};

int __init addon_setup_advaly_system_can(struct addon_device_descriptor *desc,
					 enum addon_interface intf)
{
	mxc_iomux_v3_setup_multiple_pads(addon_data[intf].pads,
				 addon_data[intf].nr_pads);

	addon_gpio_request(addon_data[intf].power);

	pinctrl_register_mappings(advaly_system_can_map,
				  ARRAY_SIZE(advaly_system_can_map));

	if (intf == ADDON_INTERFACE1) {
		regulator_register_fixed(PLATFORM_DEVID_AUTO,
					 flexcan0_dummy_supplies,
					 ARRAY_SIZE(flexcan0_dummy_supplies));
		imx25_add_flexcan0();
	}

	if (intf == ADDON_INTERFACE2) {
		regulator_register_fixed(PLATFORM_DEVID_AUTO,
					 flexcan1_dummy_supplies,
					 ARRAY_SIZE(flexcan1_dummy_supplies));
		imx25_add_flexcan1();
	}

	/* refered: reference circuit */
	addon_gpio_direction_output(addon_data[intf].power, 1); /* always high */

	return 0;
}
