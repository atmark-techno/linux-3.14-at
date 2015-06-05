/*
 * Copyright (C) 2015 Atmark Techno, Inc. All Rights Reserved.
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
#include <linux/i2c.h>

#include "armadillo_iotg_std_addon.h"

/* EEPROM I2C slave address */
#define ADDON_EEPROM_ADDR1	(0x50) /* CON1(Ext.I/F1) */
#define ADDON_EEPROM_ADDR2	(0x51) /* CON2(Ext.I/F2) */

struct addon_vendor_name
{
        u16 vendor;
        const char *name;
};
#define VENDOR_NAME(v, n) { ADDON_VENDOR_ID_##v, n }

static struct addon_vendor_name vendor_names[] = {
	VENDOR_NAME(ATMARK_TECHNO, "Atmark Techno"),
};

static const char *unknownvendorname = "Unknown Vendor";

struct addon_product_name
{
        u16 vendor;
        u16 product;
        const char *name;
};
#define PRODUCT_NAME(v, p, n) { ADDON_VENDOR_ID_##v, \
				ADDON_PRODUCT_ID_##v##_##p, n }

static struct addon_product_name product_names[] = {
	PRODUCT_NAME(ATMARK_TECHNO, WI_SUN, "Wi-SUN"),
	PRODUCT_NAME(ATMARK_TECHNO, EN_OCEAN, "EnOcean"),
	PRODUCT_NAME(ATMARK_TECHNO, SERIAL, "RS485/RS422/RS232C"),
	PRODUCT_NAME(ATMARK_TECHNO, DIDOAD, "DI/DO/AD"),
	PRODUCT_NAME(ATMARK_TECHNO, BLE, "Bluetooth Low Energy"),
	PRODUCT_NAME(ATMARK_TECHNO, CAN, "Can"),
	PRODUCT_NAME(ATMARK_TECHNO, ZIGBEE, "ZigBee"),
	PRODUCT_NAME(ATMARK_TECHNO, RS232C, "RS232C"),
	PRODUCT_NAME(ATMARK_TECHNO, RS485, "RS485"),
};

static const char *unknownproductname = "Unknown Product";

static const char *addon_get_vendor_name(u16 vendor)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vendor_names); i++)
                if (vendor_names[i].vendor == vendor) 
                        return vendor_names[i].name;

        return unknownvendorname;
}

static const char *addon_get_product_name(u16 vendor, u16 product)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(product_names); i++) {
                if ((product_names[i].vendor == vendor) &&
		    (product_names[i].product == product)) 
                        return product_names[i].name;
	}

        return unknownproductname;
}

static int addon_get_descriptor(struct addon_device_descriptor *desc, u16 addr)
{
	struct i2c_adapter *adap;
	union i2c_smbus_data data;
	unsigned char *p = (unsigned char *)desc;
	int i;
	int ret;

	adap = i2c_get_adapter(4);
	if (!adap) {
		pr_err("failed to get i2c adapter\n");
		return -EINVAL;
	}

	for (i = 0; i < sizeof(struct addon_device_descriptor); i++) {
		ret = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ,
				     i, I2C_SMBUS_BYTE_DATA, &data);
		if (ret)
			goto out;
		*(p + i) = data.byte;
	}

out:
	i2c_put_adapter(adap);

	return ret;
}

/*
 * Setup
 */
static void __init addon_setup(struct addon_device_descriptor *desc,
			       enum addon_interface intf)
{
	u16 vendor_id = be16_to_cpu(desc->vendor_id);
	u16 product_id = be16_to_cpu(desc->product_id);
	u16 revision = be16_to_cpu(desc->revision);
	u32 serial_no = be32_to_cpu(desc->serial_no);
	int ret = -ENODEV;

	pr_info("%s %s board detected at CON%d(Rev %d, SerialNumber=%d).\n",
		addon_get_vendor_name(vendor_id),
		addon_get_product_name(vendor_id, product_id),
		intf + 1, revision, serial_no);

	switch (vendor_id) {
	case ADDON_VENDOR_ID_ATMARK_TECHNO:
		switch (product_id) {
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_WI_SUN:
			ret = addon_setup_atmark_techno_wi_sun(desc, intf);
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_EN_OCEAN:
			ret = addon_setup_atmark_techno_en_ocean(desc, intf);
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_SERIAL:
			ret = addon_setup_atmark_techno_serial(desc, intf);
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_DIDOAD:
			ret = addon_setup_atmark_techno_didoad(desc, intf);
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_BLE:
			ret = addon_setup_atmark_techno_ble(desc, intf);
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_CAN:
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_ZIGBEE:
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_RS232C:
			ret = addon_setup_atmark_techno_rs232c(desc, intf);
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_RS485:
			ret = addon_setup_atmark_techno_rs485(desc, intf);
			break;
		default:
			break;
		}
	default:
		break;
	}

	if (ret) {
		pr_err("Failed to initialize Add-On.I/F at CON%d.\n",
		       intf + 1);
	}
}

static int __init armadillo_iotg_std_addon_init(void)
{
	struct addon_device_descriptor desc;
	int ret;

	ret = addon_get_descriptor(&desc, ADDON_EEPROM_ADDR1);
	if (!ret)
		addon_setup(&desc, ADDON_INTERFACE1);
	else
		pr_info("No add-on expansion board detected at CON1.\n");

	ret = addon_get_descriptor(&desc, ADDON_EEPROM_ADDR2);
	if (!ret)
		addon_setup(&desc, ADDON_INTERFACE2);
	else
		pr_info("No add-on expansion board detected at CON2.\n");

	return 0;
}
subsys_initcall_sync(armadillo_iotg_std_addon_init);
