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
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/pinctrl/machine.h>
#include <linux/platform_device.h>
#include <linux/usb/otg.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/leds.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/usb/imx_usb.h>

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

static const struct imxuart_platform_data uart0_pdata __initconst = {
#if defined(CONFIG_SERIAL_MXC_HW_FLOW_ENABLED1)
	.flags = IMXUART_HAVE_RTSCTS,
#endif
};

#define UART2_FORCEOFF_GPIO	IMX_GPIO_NR(4, 31)
static int armadillo4x0_uart1_activate(struct platform_device *pdev)
{
	int ret = 0;

	/* uart2 rs-232c transceiver power on */
	ret = imx25_named_gpio_request(UART2_FORCEOFF_GPIO, "UART2_FORCEOFF");
	if (ret)
		return ret;
	ret = gpio_direction_output(UART2_FORCEOFF_GPIO, 1);

	return ret;
}

#define UART2_DSR_GPIO		IMX_GPIO_NR(4, 23)
static const struct imxuart_platform_data uart1_pdata __initconst = {
	.init = armadillo4x0_uart1_activate,
#if defined(CONFIG_SERIAL_MXC_HW_FLOW_ENABLED2)
	.flags = IMXUART_HAVE_RTSCTS,
#endif
	.use_gpio_for_dsr = 1,
	.gpio_dsr = UART2_DSR_GPIO,
};

static unsigned long __maybe_unused pin_cfgs_100kup[] = {
	PAD_CTL_PUS_100K_UP,
};

static unsigned long pin_cfgs_none[] = {
	0,
};

static unsigned long pin_cfgs_100kdown_sre_fast[] = {
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SRE_FAST,
};

static unsigned long pin_cfgs_sre_fast[] = {
	PAD_CTL_SRE_FAST,
};

static unsigned long pin_cfgs_ode[] = {
	PAD_CTL_ODE,
};

static unsigned long pin_cfgs_dse_low[] = {
	PAD_CTL_DSE_LOW,
};

static unsigned long __maybe_unused pin_cfgs_hys[] = {
	PAD_CTL_HYS,
};

static unsigned long  __maybe_unused pin_cfgs_100kup_hys[] = {
	PAD_CTL_PUS_100K_UP | PAD_CTL_HYS,
};

static unsigned long __maybe_unused pin_cfgs_47kup[] = {
	PAD_CTL_PUS_47K_UP,
};

static const struct pinctrl_map armadillo4x0_pinctrl_map[] = {
	/* uart all */
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("imx25-pinctrl.0",
					"MX25_PAD_GRP_DSE_UART", pin_cfgs_dse_low),

	/* uart2 */
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				  "uart2_rxd__uart2_rxd", "uart2"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				  "uart2_txd__uart2_txd", "uart2"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				    "MX25_PAD_UART2_RXD", pin_cfgs_100kup),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				    "MX25_PAD_UART2_TXD", pin_cfgs_none),

	/* uart2 hardware flow control */
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				  "uart2_rts__uart2_rts", "uart2"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				  "uart2_cts__uart2_cts", "uart2"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				    "MX25_PAD_UART2_RTS", pin_cfgs_100kup),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				    "MX25_PAD_UART2_CTS", pin_cfgs_none),

	/* uart2 modem control */
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				  "uart1_rxd__uart2_dtr", "uart2"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				  "uart1_txd__gpio_4_23", "gpio4"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				  "uart1_rts__uart2_dcd", "uart2"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				  "uart1_cts__uart2_ri", "uart2"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				    "MX25_PAD_UART1_RXD", pin_cfgs_none),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				    "MX25_PAD_UART1_TXD", pin_cfgs_100kup),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				    "MX25_PAD_UART1_RTS", pin_cfgs_100kup),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				    "MX25_PAD_UART1_CTS", pin_cfgs_100kup),

	/* uart2 rs-232c transceiver forceoff  */
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-uart.1", "imx25-pinctrl.0",
				  "boot_mode1__gpio_4_31", "gpio4"),

	/* FEC */
	PIN_MAP_MUX_GROUP_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				  "fec_mdc__fec_mdc", "fec"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				  "fec_mdio__fec_mdio", "fec"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				  "fec_tdata0__fec_tdata0", "fec"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				  "fec_tdata1__fec_tdata1", "fec"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				  "fec_tx_en__fec_tx_en", "fec"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				  "fec_rdata0__fec_rdata0", "fec"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				  "fec_rdata1__fec_rdata1", "fec"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				  "fec_rx_dv__fec_rx_dv", "fec"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				  "fec_tx_clk__fec_tx_clk", "fec"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				  "upll_bypclk__gpio_3_16", "gpio3"),

	PIN_MAP_CONFIGS_PIN_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				    "MX25_PAD_FEC_MDC", pin_cfgs_none),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				    "MX25_PAD_FEC_MDIO", pin_cfgs_hys),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				    "MX25_PAD_FEC_TDATA0", pin_cfgs_sre_fast),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				    "MX25_PAD_FEC_TDATA1", pin_cfgs_sre_fast),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				    "MX25_PAD_FEC_TX_EN", pin_cfgs_sre_fast),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				    "MX25_PAD_FEC_RDATA0", pin_cfgs_none),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				    "MX25_PAD_FEC_RDATA1", pin_cfgs_none),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				    "MX25_PAD_FEC_RX_DV", pin_cfgs_none),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx25-fec.0", "imx25-pinctrl.0",
				    "MX25_PAD_FEC_TX_CLK", pin_cfgs_100kup_hys),

	/* eSDHC1 */
	PIN_MAP_MUX_GROUP_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				  "sd1_cmd__sd1_cmd", "sd1"),
	PIN_MAP_MUX_GROUP_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				  "sd1_clk__sd1_clk", "sd1"),
	PIN_MAP_MUX_GROUP_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				  "sd1_data0__sd1_data0", "sd1"),
	PIN_MAP_MUX_GROUP_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				  "sd1_data1__sd1_data1", "sd1"),
	PIN_MAP_MUX_GROUP_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				  "sd1_data2__sd1_data2", "sd1"),
	PIN_MAP_MUX_GROUP_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				  "sd1_data3__sd1_data3", "sd1"),
	PIN_MAP_MUX_GROUP_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				  "nfrb__gpio_3_31", "gpio3"),

	PIN_MAP_CONFIGS_PIN_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				    "MX25_PAD_SD1_CMD",
				    pin_cfgs_sre_fast),
	PIN_MAP_CONFIGS_PIN_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				    "MX25_PAD_SD1_CLK",
				    pin_cfgs_sre_fast),
	PIN_MAP_CONFIGS_PIN_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				    "MX25_PAD_SD1_DATA0",
				    pin_cfgs_sre_fast),
	PIN_MAP_CONFIGS_PIN_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				    "MX25_PAD_SD1_DATA1",
				    pin_cfgs_sre_fast),
	PIN_MAP_CONFIGS_PIN_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				    "MX25_PAD_SD1_DATA2",
				    pin_cfgs_sre_fast),
	PIN_MAP_CONFIGS_PIN_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				    "MX25_PAD_SD1_DATA3",
				    pin_cfgs_sre_fast),
	PIN_MAP_CONFIGS_PIN_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				    "MX25_PAD_NFRB", pin_cfgs_none),

	PIN_MAP_CONFIGS_PIN_DEFAULT("sdhci-esdhc-imx25.0", "imx25-pinctrl.0",
				    "MX25_PAD_GRP_DSE_SDHC1", pin_cfgs_dse_low),

	PIN_MAP_MUX_GROUP("sdhci-esdhc-imx25.0",
			  "state_engcm02759_workaround",
			  "imx25-pinctrl.0", "sd1_clk__gpio_2_24", "gpio2"),
	PIN_MAP_CONFIGS_PIN("sdhci-esdhc-imx25.0",
			    "state_engcm02759_workaround",
			    "imx25-pinctrl.0", "MX25_PAD_SD1_CLK",
			    pin_cfgs_100kdown_sre_fast),

	/* Fixed voltage regulator */
	PIN_MAP_MUX_GROUP_DEFAULT("reg-fixed-voltage.1", "imx25-pinctrl.0",
				  "nfre_b__gpio_3_27", "gpio3"),
	PIN_MAP_MUX_GROUP_DEFAULT("reg-fixed-voltage.2", "imx25-pinctrl.0",
				  "nfwe_b__gpio_3_26", "gpio3"),

	/* I2C1 */
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-i2c.0", "imx25-pinctrl.0",
				  "i2c1_clk__i2c1_clk", "i2c1"),
	PIN_MAP_MUX_GROUP_DEFAULT("imx21-i2c.0", "imx25-pinctrl.0",
				  "i2c1_dat__i2c1_dat", "i2c1"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-i2c.0", "imx25-pinctrl.0",
				    "MX25_PAD_I2C1_CLK", pin_cfgs_ode),
	PIN_MAP_CONFIGS_PIN_DEFAULT("imx21-i2c.0", "imx25-pinctrl.0",
				    "MX25_PAD_I2C1_DAT", pin_cfgs_ode),

	/* GPIO_KEY */
	PIN_MAP_MUX_GROUP_DEFAULT("gpio-keys", "imx25-pinctrl.0",
				  "nfwp_b__gpio_3_30", "gpio3"),
#if defined(CONFIG_ARMADILLO4X0_GPIOKEYS_CON11)
	PIN_MAP_MUX_GROUP_DEFAULT("gpio-keys", "imx25-pinctrl.0",
				  "de_b__gpio_2_20", "gpio2"),
	PIN_MAP_MUX_GROUP_DEFAULT("gpio-keys", "imx25-pinctrl.0",
				  "kpp_row0__gpio_2_29", "gpio2"),
	PIN_MAP_MUX_GROUP_DEFAULT("gpio-keys", "imx25-pinctrl.0",
				  "kpp_row1__gpio_2_30", "gpio2"),

	PIN_MAP_CONFIGS_PIN_DEFAULT("gpio-keys", "imx25-pinctrl.0",
				    "MX25_PAD_DE_B", pin_cfgs_none),
	PIN_MAP_CONFIGS_PIN_DEFAULT("gpio-keys", "imx25-pinctrl.0",
				    "MX25_PAD_KPP_ROW0", pin_cfgs_47kup),
	PIN_MAP_CONFIGS_PIN_DEFAULT("gpio-keys", "imx25-pinctrl.0",
				    "MX25_PAD_KPP_ROW1", pin_cfgs_47kup),
#endif

	/* leds */
	PIN_MAP_MUX_GROUP_DEFAULT("leds-gpio", "imx25-pinctrl.0",
				  "nfale__gpio_3_28", "gpio3"),
	PIN_MAP_MUX_GROUP_DEFAULT("leds-gpio", "imx25-pinctrl.0",
				  "nfcle__gpio_3_29", "gpio3"),
	PIN_MAP_MUX_GROUP_DEFAULT("leds-gpio", "imx25-pinctrl.0",
				  "boot_mode0__gpio_4_30", "gpio4"),
};

static const struct fec_platform_data mx25_fec_pdata __initconst = {
	.phy    = PHY_INTERFACE_MODE_RMII,
	.link_led_gpio	= IMX_GPIO_NR(3, 16),
	.link_led_active_low	= true,
};

#define FEC_PHY_RST		IMX_GPIO_NR(3, 18)
static void __init armadillo4x0_fec_reset(void)
{
	mxc_iomux_v3_setup_pad(MX25_PAD_VSTBY_ACK__GPIO_3_18);

	imx25_named_gpio_request(FEC_PHY_RST, "FEC PHY reset");
	gpio_direction_output(FEC_PHY_RST, 0); /* assert reset */
	/* PHY spec says 100us min and Armadillo-400 has low pass filter
	 * for PHY_RST pin (300us). */
	udelay(1000);
	gpio_set_value(FEC_PHY_RST, 1); /* deassert reset */
}

static struct imx_usb_platform_data usbh2_pdata __initdata = {
	.phy_mode = USBPHY_INTERFACE_MODE_SERIAL,
	.dr_mode = USB_DR_MODE_HOST,
};

static struct imx_usb_platform_data otg_pdata __initdata = {
	.phy_mode = USBPHY_INTERFACE_MODE_UTMI,
	.dr_mode = USB_DR_MODE_HOST,
};

static const struct imxi2c_platform_data mx25_i2c0_data __initconst = {
	.bitrate = 100000,
};

static const struct esdhc_platform_data
armadillo4x0_esdhc1_pdata __initconst = {
	.cd_gpio = IMX_GPIO_NR(3, 31),
	.clk_gpio = IMX_GPIO_NR(2, 24),
	.wp_type = ESDHC_WP_NONE,
	.cd_type = ESDHC_CD_GPIO,
	.max_bus_width = 4,
	.f_max = 52000000,
	.support_vsel = false,
};

static struct gpio_keys_button armadillo4x0_gpio_key_buttons[] = {
	{KEY_ENTER, IMX_GPIO_NR(3, 30), 1, "SW1", EV_KEY, 0},
#if defined(CONFIG_ARMADILLO4X0_GPIOKEYS_CON11)
	{KEY_BACK,  IMX_GPIO_NR(2, 20), 1, "LCD_SW1", EV_KEY, 0},
	{KEY_MENU,  IMX_GPIO_NR(2, 29), 1, "LCD_SW2", EV_KEY, 0},
	{KEY_HOME,  IMX_GPIO_NR(2, 30), 1, "LCD_SW3", EV_KEY, 0},
#endif
};

static struct gpio_keys_platform_data armadillo4x0_gpio_key_data = {
	.buttons = armadillo4x0_gpio_key_buttons,
	.nbuttons = ARRAY_SIZE(armadillo4x0_gpio_key_buttons),
};

static struct gpio_led armadillo4x0_led_pins[] = {
	{"green",  "default-on", IMX_GPIO_NR(3, 29), 0},
	{"red",  "default-on", IMX_GPIO_NR(3, 28), 0},
	{"yellow", NULL,         IMX_GPIO_NR(4, 30), 0},
};

static struct gpio_led_platform_data armadillo4x0_led_data = {
	.leds = armadillo4x0_led_pins,
	.num_leds = ARRAY_SIZE(armadillo4x0_led_pins),
};

/*
 * MTD NOR Flash
 */
static struct mtd_partition armadillo4x0_nor_flash_partitions_16m[] = {
	{
		.name		= "nor.bootloader",
		.offset		= 0x00000000,
		.size		= 4 * SZ_32K,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "nor.kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 32 * SZ_128K,
		.mask_flags	= 0,
	}, {
		.name		= "nor.userland",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 87 * SZ_128K,
		.mask_flags	= 0,
	}, {
		.name		= "nor.config",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 8 * SZ_128K,
		.mask_flags	= 0,
	},
};

static struct mtd_partition armadillo4x0_nor_flash_partitions_32m[] = {
	{
		.name		= "nor.bootloader",
		.offset		= 0x00000000,
		.size		= 4 * SZ_32K,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "nor.kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 32 * SZ_128K,
		.mask_flags	= 0,
	}, {
		.name		= "nor.userland",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 215 * SZ_128K,
		.mask_flags	= 0,
	}, {
		.name		= "nor.config",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 8 * SZ_128K,
		.mask_flags	= 0,
	},
};

static struct mtd_partition armadillo4x1_nor_flash_partitions_32m[] = {
	{
		.name		= "nor.bootloader",
		.offset		= 0x00000000,
		.size		= 2 * SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "nor.kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 32 * SZ_128K,
		.mask_flags	= 0,
	}, {
		.name		= "nor.userland",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 214 * SZ_128K,
		.mask_flags	= 0,
	}, {
		.name		= "nor.config",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 8 * SZ_128K,
		.mask_flags	= 0,
	},
};

static const struct physmap_flash_data
		armadillo4x0_nor_flash_pdata_16m __initconst = {
	.width		= 2,
	.parts		= armadillo4x0_nor_flash_partitions_16m,
	.nr_parts	= ARRAY_SIZE(armadillo4x0_nor_flash_partitions_16m),
};

static const struct physmap_flash_data
		armadillo4x0_nor_flash_pdata_32m __initconst = {
	.width		= 2,
	.parts		= armadillo4x0_nor_flash_partitions_32m,
	.nr_parts	= ARRAY_SIZE(armadillo4x0_nor_flash_partitions_32m),
};

static const struct physmap_flash_data
		armadillo4x1_nor_flash_pdata_32m __initconst = {
	.width		= 2,
	.parts		= armadillo4x1_nor_flash_partitions_32m,
	.nr_parts	= ARRAY_SIZE(armadillo4x1_nor_flash_partitions_32m),
};

static const struct resource
armadillo4x0_nor_flash_resource __initconst = {
	.flags		= IORESOURCE_MEM,
	.start		= MX25_CS0_BASE_ADDR,
	.end		= MX25_CS0_BASE_ADDR + SZ_32M - 1,
};

/*
 * Regulator
 */
static struct regulator_consumer_supply esdhc1_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx25.0"),
};

static struct regulator_init_data armadillo4x0_esdhc1_regulator_data = {
	.constraints	= {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(esdhc1_consumers),
	.consumer_supplies	= esdhc1_consumers,
};

static struct fixed_voltage_config
armadillo4x0_esdhc1_regulator_config = {
	.supply_name		= "eSDHC1 Vcc",
	.microvolts		= 3300000,
	.gpio			= IMX_GPIO_NR(3, 27),
	.enable_high		= 0,
	.enabled_at_boot	= 0,
	.init_data		= &armadillo4x0_esdhc1_regulator_data,
};

static struct regulator_consumer_supply usb_consumers[] = {
	REGULATOR_SUPPLY("vbus", "imx27-usb.0"),
	REGULATOR_SUPPLY("vbus", "imx27-usb.1"),
};

static struct regulator_init_data armadillo4x0_usb_regulator_data = {
	.constraints	= {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(usb_consumers),
	.consumer_supplies	= usb_consumers,
};

static struct fixed_voltage_config
armadillo4x0_usb_regulator_config = {
	.supply_name		= "USB VBUS",
	.microvolts		= 5000000,
	.gpio			= IMX_GPIO_NR(3, 26),
	.enable_high		= 0,
	.enabled_at_boot	= 1,
	.init_data		= &armadillo4x0_usb_regulator_data,
};

inline static void __init armadillo4x0_init_mtd(const struct physmap_flash_data data)
{
	platform_device_register_resndata(NULL, "physmap-flash", -1,
					  &armadillo4x0_nor_flash_resource, 1,
					  &data, sizeof(data));
}
#if defined(CONFIG_MACH_ARMADILLO420)
static void __init armadillo420_init_mtd(void)
{
	armadillo4x0_init_mtd(armadillo4x0_nor_flash_pdata_16m);
}
#endif

#if defined(CONFIG_MACH_ARMADILLO440) || defined(CONFIG_MACH_ARMADILLO410)
static void __init armadillo440_init_mtd(void)
{
	if (machine_is_armadillo441() || machine_is_armadillo411())
		armadillo4x0_init_mtd(armadillo4x1_nor_flash_pdata_32m);
	else
		armadillo4x0_init_mtd(armadillo4x0_nor_flash_pdata_32m);
}
#endif

static void __init armadillo4x0_init(void)
{
	imx25_soc_init();

	imx25_add_imx_pinctrl("imx25-pinctrl");
	pinctrl_register_mappings(armadillo4x0_pinctrl_map,
				  ARRAY_SIZE(armadillo4x0_pinctrl_map));

	if (IS_ENABLED(CONFIG_SERIAL_MXC_SELECT1))
		imx25_add_imx_uart0(&uart0_pdata);

	if (IS_ENABLED(CONFIG_SERIAL_MXC_SELECT2))
		imx25_add_imx_uart1(&uart1_pdata);

	imx25_add_usbmisc_imx();
	imx25_add_usb_phy_gen_xceiv_hs();
	imx25_add_usb_phy_gen_xceiv_otg();
	imx25_add_imx_usb_hs(&usbh2_pdata);
	imx25_add_imx_usb_otg(&otg_pdata);
	imx25_add_imx2_wdt();

	imx25_named_gpio_init();

	armadillo4x0_fec_reset();
	imx25_add_fec(&mx25_fec_pdata);

	imx25_add_imx_i2c0(&mx25_i2c0_data);

	platform_device_register_data(NULL, "reg-fixed-voltage", 1,
				&armadillo4x0_esdhc1_regulator_config,
				sizeof(armadillo4x0_esdhc1_regulator_config));
	platform_device_register_data(NULL, "reg-fixed-voltage", 2,
				&armadillo4x0_usb_regulator_config,
				sizeof(armadillo4x0_usb_regulator_config));

	imx25_add_sdhci_esdhc_imx(0, &armadillo4x0_esdhc1_pdata);

	imx_add_gpio_keys(&armadillo4x0_gpio_key_data);

	gpio_led_register_device(-1, &armadillo4x0_led_data);
}

static void __init armadillo4x0_timer_init(void)
{
	mx25_clocks_init();
}

#if defined(CONFIG_MACH_ARMADILLO420)
static void __init armadillo420_init(void)
{
	armadillo4x0_init();
	armadillo420_init_mtd();
	armadillo4x0_con9_con14_init();
}
#endif

#if defined(CONFIG_MACH_ARMADILLO440) || defined(CONFIG_MACH_ARMADILLO410)
static void __init armadillo440_init(void)
{
	armadillo4x0_init();
	armadillo440_init_mtd();
	armadillo4x0_con9_con14_init();
	armadillo4x0_con11_init();
}
#endif

#if defined(CONFIG_MACH_ARMADILLO410)
MACHINE_START(ARMADILLO410, "Armadillo-410")
	/* Maintainer: Atmark Techno, Inc.  */
	.atag_offset	= 0x100,
	.map_io		= mx25_map_io,
	.init_early	= imx25_init_early,
	.init_irq	= mx25_init_irq,
	.handle_irq	= imx25_handle_irq,
	.init_time	= armadillo4x0_timer_init,
	.init_machine	= armadillo440_init,
	.restart	= mxc_restart,
MACHINE_END

MACHINE_START(ARMADILLO411, "Armadillo-410")
	/* Maintainer: Atmark Techno, Inc.  */
	.atag_offset	= 0x100,
	.map_io		= mx25_map_io,
	.init_early	= imx25_init_early,
	.init_irq	= mx25_init_irq,
	.handle_irq	= imx25_handle_irq,
	.init_time	= armadillo4x0_timer_init,
	.init_machine	= armadillo440_init,
	.restart	= mxc_restart,
MACHINE_END
#endif

#if defined(CONFIG_MACH_ARMADILLO420)
MACHINE_START(ARMADILLO420, "Armadillo-420")
	/* Maintainer: Atmark Techno, Inc.  */
	.atag_offset	= 0x100,
	.map_io		= mx25_map_io,
	.init_early	= imx25_init_early,
	.init_irq	= mx25_init_irq,
	.handle_irq	= imx25_handle_irq,
	.init_time	= armadillo4x0_timer_init,
	.init_machine	= armadillo420_init,
	.restart	= mxc_restart,
MACHINE_END
#endif

#if defined(CONFIG_MACH_ARMADILLO440)
MACHINE_START(ARMADILLO440, "Armadillo-440")
	/* Maintainer: Atmark Techno, Inc.  */
	.atag_offset	= 0x100,
	.map_io		= mx25_map_io,
	.init_early	= imx25_init_early,
	.init_irq	= mx25_init_irq,
	.handle_irq	= imx25_handle_irq,
	.init_time	= armadillo4x0_timer_init,
	.init_machine	= armadillo440_init,
	.restart	= mxc_restart,
MACHINE_END

MACHINE_START(ARMADILLO441, "Armadillo-440")
	/* Maintainer: Atmark Techno, Inc.  */
	.atag_offset	= 0x100,
	.map_io		= mx25_map_io,
	.init_early	= imx25_init_early,
	.init_irq	= mx25_init_irq,
	.handle_irq	= imx25_handle_irq,
	.init_time	= armadillo4x0_timer_init,
	.init_machine	= armadillo440_init,
	.restart	= mxc_restart,
MACHINE_END
#endif
