/*
 * Copyright (C) 2015 Atmark Techno, Inc. All Rights Reserved.
 *
 * Based on mach-mx25_3ds.c which is :
 *   Copyright 2009 Sascha Hauer, <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/pinctrl/machine.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <asm/system_info.h>

#include "common.h"
#include "devices-imx25.h"
#include "hardware.h"
#include "iomux-mx25.h"
#include "mx25.h"
#include "imx25-named_gpio.h"
#include "armadillo4x0_extif.h"

static const struct imxuart_platform_data uart2_pdata __initconst = {
#if defined(CONFIG_SERIAL_MXC_HW_FLOW_ENABLED3)
	.flags = IMXUART_HAVE_RTSCTS,
#endif
};

static const struct imxuart_platform_data uart4_pdata __initconst = {
#if defined(CONFIG_SERIAL_MXC_HW_FLOW_ENABLED5)
	.flags = IMXUART_HAVE_RTSCTS,
#endif
};

static unsigned long __maybe_unused pin_cfgs_100kup[] = {
	PAD_CTL_PUS_100K_UP,
};

static unsigned long __maybe_unused pin_cfgs_none[] = {
	NO_PAD_CTRL,
};

static unsigned long __maybe_unused pin_cfgs_22kup_ode[] = {
	PAD_CTL_PUS_22K_UP | PAD_CTL_ODE,
};

static unsigned long __maybe_unused pin_cfgs_sre_fast[] = {
	PAD_CTL_SRE_FAST,
};

static const struct pinctrl_map armadillo4x0_extif_pinctrl_map[] = {
	/* uart3 */
#if defined(CONFIG_ARMADILLO4X0_UART3_CON9)
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.2", "imx25-pinctrl.0",
				  "cspi1_mosi__uart3_rxd_mux", "uart3"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.2", "imx25-pinctrl.0",
				  "cspi1_miso__uart3_txd_mux", "uart3"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.2", "imx25-pinctrl.0",
				    "MX25_PAD_CSPI1_MOSI", pin_cfgs_100kup),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.2", "imx25-pinctrl.0",
				    "MX25_PAD_CSPI1_MISO", pin_cfgs_none),
#endif
#if defined(CONFIG_ARMADILLO4X0_UART3_HW_FLOW_CON9)
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.2", "imx25-pinctrl.0",
				  "cspi1_ss1__uart3_rts", "uart3"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.2", "imx25-pinctrl.0",
				  "cspi1_sclk__uart3_cts", "uart3"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.2", "imx25-pinctrl.0",
				    "MX25_PAD_CSPI1_SS1", pin_cfgs_100kup),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.2", "imx25-pinctrl.0",
				    "MX25_PAD_CSPI1_SCLK", pin_cfgs_none),
#endif

	/* uart5 */
#if defined(CONFIG_ARMADILLO4X0_UART5_CON9)
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.4", "imx25-pinctrl.0",
				  "csi_d2__uart5_rxd_mux", "uart5"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.4", "imx25-pinctrl.0",
				  "csi_d3__uart5_txd_mux", "uart5"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.4", "imx25-pinctrl.0",
				    "MX25_PAD_CSI_D2", pin_cfgs_100kup),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.4", "imx25-pinctrl.0",
				    "MX25_PAD_CSI_D3", pin_cfgs_none),
#endif
#if defined(CONFIG_ARMADILLO4X0_UART5_HW_FLOW_CON9)
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.4", "imx25-pinctrl.0",
				  "csi_d4__uart5_rts", "uart5"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.4", "imx25-pinctrl.0",
				  "csi_d5__uart5_cts", "uart5"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.4", "imx25-pinctrl.0",
				    "MX25_PAD_CSI_D4", pin_cfgs_100kup),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.4", "imx25-pinctrl.0",
				    "MX25_PAD_CSI_D5", pin_cfgs_none),
#endif

	/* i2c2 */
#if defined(CONFIG_ARMADILLO4X0_I2C2_CON14)
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-i2c.1", "imx25-pinctrl.0",
				  "gpio_c__i2c2_clk", "i2c2"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-i2c.1", "imx25-pinctrl.0",
				  "gpio_d__i2c2_dat", "i2c2"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-i2c.1", "imx25-pinctrl.0",
				    "MX25_PAD_GPIO_C", pin_cfgs_22kup_ode),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-i2c.1", "imx25-pinctrl.0",
				    "MX25_PAD_GPIO_D", pin_cfgs_22kup_ode),
#endif

	/* CON9 GPIO */
#if defined(CONFIG_ARMADILLO4X0_CON9_1_GPIO3_17)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "vstby_req__gpio_3_17", "gpio3"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_VSTBY_REQ", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_2_GPIO3_14)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "rtck__gpio_3_14", "gpio3"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_RTCK", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_3_GPIO1_14)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "cspi1_mosi__gpio_1_14", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSPI1_MOSI", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_4_GPIO1_27)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "csi_d2__gpio_1_27", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSI_D2", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_5_GPIO1_15)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "cspi1_miso__gpio_1_15", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSPI1_MISO", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_6_GPIO1_28)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "csi_d3__gpio_1_28", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSI_D3", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_11_GPIO1_17)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "cspi1_ss1__gpio_1_17", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSPI1_SS1", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_12_GPIO1_29)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "csi_d4__gpio_1_29", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSI_D4", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_13_GPIO1_18)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "cspi1_sclk__gpio_1_18", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSPI1_SCLK", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_14_GPIO1_30)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "csi_d5__gpio_1_30", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSI_D5", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_15_GPIO1_7)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "csi_d8__gpio_1_7", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSI_D8", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_16_GPIO1_31)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "csi_d6__gpio_1_31", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSI_D6", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_17_GPIO4_21)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "csi_d9__gpio_4_21", "gpio4"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSI_D9", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_18_GPIO1_6)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "csi_d7__gpio_1_6", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSI_D7", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_21_GPIO1_8)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "csi_mclk__gpio_1_8", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSI_MCLK", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_22_GPIO1_9)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "csi_vsync__gpio_1_9", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSI_VSYNC", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_23_GPIO1_10)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "csi_hsync__gpio_1_10", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSI_HSYNC", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_24_GPIO1_11)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "csi_pixclk__gpio_1_11", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSI_PIXCLK", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_25_GPIO1_16)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "cspi1_ss0__gpio_1_16", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSPI1_SS0", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_26_GPIO2_22)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "cspi1_rdy__gpio_2_22", "gpio2"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CSPI1_RDY", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_27_GPIO2_21)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "clko__gpio_2_21", "gpio2"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_CLKO", pin_cfgs_sre_fast),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON9_28_GPIO3_15)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "ext_armclk__gpio_3_15", "gpio3"),
#endif

	/* CON14 GPIO */
#if defined(CONFIG_ARMADILLO4X0_CON14_3_GPIO1_2)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "gpio_c__gpio_c", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_GPIO_C", pin_cfgs_100kup),
#endif
#if defined(CONFIG_ARMADILLO4X0_CON14_4_GPIO1_3)
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("imx25-pinctrl.0",
				      "gpio_d__gpio_d", "gpio1"),
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_GPIO_D", pin_cfgs_100kup),
#endif

};

static const struct imxi2c_platform_data mx25_i2c1_data __initconst = {
	.bitrate = 40000,
};

static struct i2c_board_info armadillo4x0_i2c1[] = {
};

enum armadillo4x0_extif_gpio_direction {
	EXTIF_GPIO_DIRECTION_OUTPUT,
	EXTIF_GPIO_DIRECTION_INPUT,
};

struct armadillo4x0_extif_gpio {
	int gpio;
	const char *name;
	enum armadillo4x0_extif_gpio_direction dir;
	int val;
};

static const struct armadillo4x0_extif_gpio armadillo4x0_extif_gpios[] = {
	/* CON9_1 */
#if defined(CONFIG_ARMADILLO4X0_CON9_1_GPIO3_17)
	{ IMX_GPIO_NR(3, 17), "CON9_1", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_2 */
#if defined(CONFIG_ARMADILLO4X0_CON9_2_GPIO3_14)
	{ IMX_GPIO_NR(3, 14), "CON9_2", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_3 */
#if defined(CONFIG_ARMADILLO4X0_CON9_3_GPIO1_14)
	{ IMX_GPIO_NR(1, 14), "CON9_3", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_4 */
#if defined(CONFIG_ARMADILLO4X0_CON9_4_GPIO1_27)
	{ IMX_GPIO_NR(1, 27), "CON9_4", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_5 */
#if defined(CONFIG_ARMADILLO4X0_CON9_5_GPIO1_15)
	{ IMX_GPIO_NR(1, 15), "CON9_5", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_6 */
#if defined(CONFIG_ARMADILLO4X0_CON9_6_GPIO1_28)
	{ IMX_GPIO_NR(1, 28), "CON9_6", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_11 */
#if defined(CONFIG_ARMADILLO4X0_CON9_11_GPIO1_17)
	{ IMX_GPIO_NR(1, 17), "CON9_11", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_12 */
#if defined(CONFIG_ARMADILLO4X0_CON9_12_GPIO1_29)
	{ IMX_GPIO_NR(1, 29), "CON9_12", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_13 */
#if defined(CONFIG_ARMADILLO4X0_CON9_13_GPIO1_18)
	{ IMX_GPIO_NR(1, 18), "CON9_13", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_14 */
#if defined(CONFIG_ARMADILLO4X0_CON9_14_GPIO1_30)
	{ IMX_GPIO_NR(1, 30), "CON9_14", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_15 */
#if defined(CONFIG_ARMADILLO4X0_CON9_15_GPIO1_7)
	{ IMX_GPIO_NR(1, 7), "CON9_15", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_16 */
#if defined(CONFIG_ARMADILLO4X0_CON9_16_GPIO1_31)
	{ IMX_GPIO_NR(1, 31), "CON9_16", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_17 */
#if defined(CONFIG_ARMADILLO4X0_CON9_17_GPIO4_21)
	{ IMX_GPIO_NR(4, 21), "CON9_17", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_18 */
#if defined(CONFIG_ARMADILLO4X0_CON9_18_GPIO1_6)
	{ IMX_GPIO_NR(1, 6), "CON9_18", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_21 */
#if defined(CONFIG_ARMADILLO4X0_CON9_21_GPIO1_8)
	{ IMX_GPIO_NR(1, 8), "CON9_21", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_22 */
#if defined(CONFIG_ARMADILLO4X0_CON9_22_GPIO1_9)
	{ IMX_GPIO_NR(1, 9), "CON9_22", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_23 */
#if defined(CONFIG_ARMADILLO4X0_CON9_23_GPIO1_10)
	{ IMX_GPIO_NR(1, 10), "CON9_23", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_24 */
#if defined(CONFIG_ARMADILLO4X0_CON9_24_GPIO1_11)
	{ IMX_GPIO_NR(1, 11), "CON9_24", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_25 */
#if defined(CONFIG_ARMADILLO4X0_CON9_25_GPIO1_16)
	{ IMX_GPIO_NR(1, 16), "CON9_25", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_26 */
#if defined(CONFIG_ARMADILLO4X0_CON9_26_GPIO2_22)
	{ IMX_GPIO_NR(2, 22), "CON9_26", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON9_27 */
#if defined(CONFIG_ARMADILLO4X0_CON9_27_GPIO2_21)
	{ IMX_GPIO_NR(2, 21), "CON9_27" , EXTIF_GPIO_DIRECTION_OUTPUT, 0 },
#endif
	/* CON9_28 */
#if defined(CONFIG_ARMADILLO4X0_CON9_28_GPIO3_15)
	{ IMX_GPIO_NR(3, 15), "CON9_28" , EXTIF_GPIO_DIRECTION_OUTPUT, 0 },
#endif
	/* CON14_3 */
#if defined(CONFIG_ARMADILLO4X0_CON14_3_GPIO1_2)
	{ IMX_GPIO_NR(1, 2), "CON14_3", EXTIF_GPIO_DIRECTION_INPUT },
#endif
	/* CON14_4 */
#if defined(CONFIG_ARMADILLO4X0_CON14_4_GPIO1_3)
	{ IMX_GPIO_NR(1, 3), "CON14_4", EXTIF_GPIO_DIRECTION_INPUT },
#endif
};

static void armadillo4x0_set_extif_gpio(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(armadillo4x0_extif_gpios); i++) {
		imx25_named_gpio_request(armadillo4x0_extif_gpios[i].gpio,
					 armadillo4x0_extif_gpios[i].name);
		if (armadillo4x0_extif_gpios[i].dir == EXTIF_GPIO_DIRECTION_INPUT)
			gpio_direction_input(armadillo4x0_extif_gpios[i].gpio);
		else
			gpio_direction_output(armadillo4x0_extif_gpios[i].gpio,
					      armadillo4x0_extif_gpios[i].val);
		gpio_export(armadillo4x0_extif_gpios[i].gpio, true);
	}
}

void __init armadillo4x0_extif_init(void)
{
	pinctrl_register_mappings(armadillo4x0_extif_pinctrl_map,
				  ARRAY_SIZE(armadillo4x0_extif_pinctrl_map));

	if (IS_ENABLED(CONFIG_SERIAL_MXC_SELECT3))
		imx25_add_imx_uart2(&uart2_pdata);

	if (IS_ENABLED(CONFIG_SERIAL_MXC_SELECT5))
		imx25_add_imx_uart4(&uart4_pdata);

	if (IS_ENABLED(CONFIG_I2C_MXC_SELECT2)) {
		imx25_add_imx_i2c1(&mx25_i2c1_data);
		i2c_register_board_info(1, armadillo4x0_i2c1,
					ARRAY_SIZE(armadillo4x0_i2c1));
	}

	armadillo4x0_set_extif_gpio();
}
