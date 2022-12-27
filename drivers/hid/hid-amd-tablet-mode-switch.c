
// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  HID driver for Speedlink Vicious and Divine Cezanne (USB mouse).
 *  Fixes "jumpy" cursor and removes nonexistent keyboard LEDS from
 *  the HID descriptor.
 *
 *  Copyright (c) 2022-2023 Adrian Freund <adrian@freund.io>
 */

#include <linux/hid.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/hid-sensor-hub.h>
#include <linux/iio/iio.h>

#include "./amd-sfh-hid/amd_sfh_hid.h"

enum amd_tablet_mode_switch_channel {
	CHANNEL_ANGLE,
	CHANNEL_KEYBOARD_UNAVAILABLE,
	CHANNEL_POSE,
	AMD_TABLET_MODE_SWITCH_CHANNEL_MAX,
};

struct amd_tablet_mode_switch_report {
};

struct amd_tablet_mode_switch_state {
	struct hid_sensor_hub_callbacks callbacks;
	struct hid_sensor_common common_attributes;
	struct hid_sensor_hub_attribute_info attr_info[AMD_TABLET_MODE_SWITCH_CHANNEL_MAX];
	struct input_dev *idev;
	/* Ensure timestamp is naturally aligned */
	struct {
		u16 hinge_angle;
		bool keyboard_unavailable;
		u8 pose;
	} scan;
};

static ssize_t hinge_angle_show(struct device *dev,
				struct device_attribute
				*attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct amd_tablet_mode_switch_state *tms_state = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d", tms_state->scan.hinge_angle);
}
static DEVICE_ATTR_RO(hinge_angle);

static ssize_t keyboard_unavailable_show(struct device *dev,
				struct device_attribute
				*attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct amd_tablet_mode_switch_state *tms_state = platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%d", tms_state->scan.keyboard_unavailable);
}
static DEVICE_ATTR_RO(keyboard_unavailable);

static ssize_t pose_show(struct device *dev,
				struct device_attribute
				*attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct amd_tablet_mode_switch_state *tms_state = platform_get_drvdata(pdev);

	char* pose;

	switch (tms_state->scan.pose) {
	case 0:
		pose = "laptop";
		break;
	case 1:
		pose = "flat";
		break;
	case 4:
		pose = "screen";
		break;
	case 6:
		pose = "tablet";
		break;
	case 7:
		pose = "tent";
		break;
	default:
		pose = "unknown";
	}

	return sysfs_emit(buf, "%s", pose);
}
static DEVICE_ATTR_RO(pose);

static struct attribute *amd_tablet_mode_switch_attrs[] = {
	&dev_attr_hinge_angle.attr,
	&dev_attr_keyboard_unavailable.attr,
	&dev_attr_pose.attr,
	NULL
};
ATTRIBUTE_GROUPS(amd_tablet_mode_switch);

static int amd_tablet_mode_switch_proc_event(struct hid_sensor_hub_device *hsdev,
				unsigned usage_id,
				void *priv)
{
	struct amd_tablet_mode_switch_state *tms_state = platform_get_drvdata(priv);
	struct input_dev *idev = tms_state->idev;

	input_report_switch(idev, SW_TABLET_MODE, tms_state->scan.keyboard_unavailable);
	input_sync(idev);

	return 0;
}

static int amd_tablet_mode_switch_capture_sample(struct hid_sensor_hub_device *hsdev,
				unsigned usage_id,
				size_t raw_len, char *raw_data,
				void *priv)
{
	struct amd_tablet_mode_switch_state *tms_state = platform_get_drvdata(priv);

	switch (usage_id) {
	case HID_USAGE_SENSOR_DATA_FIELD_CUSTOM_VALUE(1):
		pr_err("%d", *raw_data);
		tms_state->scan.hinge_angle = *(u16 *)raw_data;
		break;
	case HID_USAGE_SENSOR_DATA_FIELD_CUSTOM_VALUE(2):
		tms_state->scan.keyboard_unavailable = *(bool *)raw_data;
		break;
	case HID_USAGE_SENSOR_DATA_FIELD_CUSTOM_VALUE(3):
		tms_state->scan.pose = *(u8 *)raw_data;
		break;
	default:
		return -EINVAL;
		
	}
	return 0;
}

static int amd_tablet_mode_switch_parse_report(struct platform_device *pdev,
		struct hid_sensor_hub_device *hsdev,
		unsigned usage_id,
		struct amd_tablet_mode_switch_state *st)
{
	int ret;
	int i;

	for (i = 0; i < AMD_TABLET_MODE_SWITCH_CHANNEL_MAX; i++) {
		ret = sensor_hub_input_get_attribute_info(hsdev,
				HID_INPUT_REPORT,
				usage_id,
				HID_USAGE_SENSOR_DATA_FIELD_CUSTOM_VALUE(i + 1),
				&st->attr_info[i]);

		if (ret < 0)
			break;
	}

	return ret;
}

static int amd_tablet_mode_switch_probe(struct platform_device *pdev) {
	int ret;
	struct hid_sensor_hub_device *hsdev = pdev->dev.platform_data;
	struct amd_tablet_mode_switch_state *tms_state;
	struct input_dev *idev;

	if (hsdev->vendor_id != AMD_SFH_HID_VENDOR || hsdev->product_id != AMD_SFH_HID_PRODUCT) 
		return -ENODEV;

	tms_state = devm_kzalloc(&pdev->dev,
			sizeof(*tms_state), GFP_KERNEL);
	if (!tms_state)
		return -ENOMEM;

	platform_set_drvdata(pdev, tms_state);

	tms_state->common_attributes.hsdev = hsdev;
	tms_state->common_attributes.pdev = pdev;
	ret = hid_sensor_parse_common_attributes(hsdev,
				hsdev->usage,
				&tms_state->common_attributes,
				NULL,
				0);

	if (ret) {
		dev_err(&pdev->dev, "failed to setup common attributes!\n");
		return ret;
	}

	ret = amd_tablet_mode_switch_parse_report(pdev, hsdev, hsdev->usage, tms_state);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to setup attributes!\n");
		return ret;
	}

	tms_state->callbacks.send_event = amd_tablet_mode_switch_proc_event;
	tms_state->callbacks.capture_sample = amd_tablet_mode_switch_capture_sample;
	tms_state->callbacks.pdev = pdev;
	ret = sensor_hub_register_callback(hsdev, hsdev->usage, &tms_state->callbacks);
	if (ret < 0) {
		dev_err(&pdev->dev, "register callback failed!\n");
		goto err_remove_callback;
	}

	idev = input_allocate_device();
	if (!idev) {
		ret = -ENOMEM;
		goto err_remove_callback;
	}

	idev->name = "AMD Sensor Fusion Hub - Tablet Mode Switch";
	idev->dev.parent = &pdev->dev;
	input_set_capability(idev, EV_SW, SW_TABLET_MODE);

	ret = input_register_device(idev);
	if (ret) {
		goto err_free_dev;
	}

	tms_state->idev = idev;


	return 0;

err_free_dev:
	input_free_device(idev);
err_remove_callback:
	sensor_hub_remove_callback(hsdev, hsdev->usage);
	return ret;
}

static int amd_tablet_mode_switch_remove(struct platform_device *pdev) {
	struct hid_sensor_hub_device *hsdev = pdev->dev.platform_data;
	struct amd_tablet_mode_switch_state *tms_state;

	tms_state = platform_get_drvdata(pdev);

	sensor_hub_remove_callback(hsdev, hsdev->usage);
	input_unregister_device(tms_state->idev);

	return 0;
}

static const struct platform_device_id hid_amd_tablet_mode_switch[] = {
	{
		/* Format: HID-SENSOR-usage_id_in_hex_lowercase
		 * Vendor Defined 1 */
		.name = "HID-SENSOR-2000f0",
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(platform, hid_amd_tablet_mode_switch);

static struct platform_driver amd_tablet_mode_switch_driver = {
	.id_table = hid_amd_tablet_mode_switch,
	.driver = {
		.name 	= KBUILD_MODNAME,
		.dev_groups = amd_tablet_mode_switch_groups,

	},
	.probe 		= amd_tablet_mode_switch_probe,
	.remove 	= amd_tablet_mode_switch_remove,
};
module_platform_driver(amd_tablet_mode_switch_driver);

MODULE_DESCRIPTION("HID Tablet Mode Switch Sensor");
MODULE_AUTHOR("Adrian Freund <adrian@freund.io>");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_HID);
