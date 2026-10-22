// SPDX-License-Identifier: GPL-2.0-only
/*
 * USB HID fan cooling device.
 */

#include <linux/hid.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/thermal.h>

#define FAN_USBHID_VENDOR_ID	0x2341
#define FAN_USBHID_PRODUCT_ID	0x0069

/* Same interface, three reports:
 *   1 input  uint16  hall angle, left for hidraw / sensors.hall
 *   3 output uint8   fan duty, SET_REPORT from this driver
 *   4 input  uint16  consumer array, AC Zoom In/Out become keys
 */
#define FAN_REPORT_ID_ANGLE	0x01
#define FAN_REPORT_ID_DUTY	0x03
#define FAN_REPORT_ID_CONSUMER	0x04
#define FAN_DUTY_LEN		2	/* report id + pwm level */
#define FAN_SPEED_MAX		255
#define CONSUMER_AC_ZOOM_IN	0x022d
#define CONSUMER_AC_ZOOM_OUT	0x022e

struct usbhid_fan {
	struct hid_device *hdev;
	struct thermal_cooling_device *cdev;
	struct mutex lock;
	unsigned int duty;
	bool dead;
};

static unsigned short vendor = FAN_USBHID_VENDOR_ID;
static unsigned short product = FAN_USBHID_PRODUCT_ID;
module_param(vendor, ushort, 0444);
MODULE_PARM_DESC(vendor, "USB vendor id of the fan (default 0x2341)");
module_param(product, ushort, 0444);
MODULE_PARM_DESC(product, "USB product id of the fan (default 0x0069)");

static int usbhid_fan_send(struct usbhid_fan *fan, unsigned int duty)
{
	u8 buf[FAN_DUTY_LEN] = { FAN_REPORT_ID_DUTY, duty };
	int ret;

	if (fan->dead)
		return -ENODEV;

	ret = hid_hw_raw_request(fan->hdev, FAN_REPORT_ID_DUTY, buf,
				 sizeof(buf), HID_OUTPUT_REPORT,
				 HID_REQ_SET_REPORT);
	if (ret < 0) {
		hid_err(fan->hdev, "SET_REPORT duty %u failed: %d\n", duty, ret);
		return ret;
	}

	return 0;
}

static int usbhid_fan_set_duty(struct usbhid_fan *fan, unsigned int duty)
{
	int ret;

	ret = usbhid_fan_send(fan, duty);
	if (!ret)
		fan->duty = duty;
	return ret;
}

static int usbhid_fan_get_max_state(struct thermal_cooling_device *cdev,
				    unsigned long *state)
{
	*state = FAN_SPEED_MAX;
	return 0;
}

static int usbhid_fan_get_cur_state(struct thermal_cooling_device *cdev,
				    unsigned long *state)
{
	struct usbhid_fan *fan = cdev->devdata;

	mutex_lock(&fan->lock);
	*state = fan->duty;
	mutex_unlock(&fan->lock);
	return 0;
}

static int usbhid_fan_set_cur_state(struct thermal_cooling_device *cdev,
				    unsigned long state)
{
	struct usbhid_fan *fan = cdev->devdata;
	int ret;

	if (state > FAN_SPEED_MAX)
		return -EINVAL;

	mutex_lock(&fan->lock);
	ret = usbhid_fan_set_duty(fan, state);
	mutex_unlock(&fan->lock);
	return ret;
}

static const struct thermal_cooling_device_ops usbhid_fan_cooling_ops = {
	.get_max_state = usbhid_fan_get_max_state,
	.get_cur_state = usbhid_fan_get_cur_state,
	.set_cur_state = usbhid_fan_set_cur_state,
};

static ssize_t speed_control_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct usbhid_fan *fan = dev_get_drvdata(dev);
	unsigned int duty;

	mutex_lock(&fan->lock);
	duty = fan->duty;
	mutex_unlock(&fan->lock);

	return sysfs_emit(buf, "%u\n", duty);
}

static ssize_t speed_control_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct usbhid_fan *fan = dev_get_drvdata(dev);
	unsigned int duty;
	int ret;

	if (kstrtouint(buf, 0, &duty) || duty > FAN_SPEED_MAX)
		return -EINVAL;

	mutex_lock(&fan->lock);
	ret = usbhid_fan_set_duty(fan, duty);
	mutex_unlock(&fan->lock);
	return ret ? ret : count;
}

static DEVICE_ATTR_RW(speed_control);

static struct attribute *usbhid_fan_attrs[] = {
	&dev_attr_speed_control.attr,
	NULL,
};

static const struct attribute_group usbhid_fan_group = {
	.attrs = usbhid_fan_attrs,
};

static int usbhid_fan_input_mapping(struct hid_device *hdev,
				    struct hid_input *hi,
				    struct hid_field *field,
				    struct hid_usage *usage,
				    unsigned long **bit, int *max)
{
	unsigned int usage_code = usage->hid & HID_USAGE;

	if (field->report->id == FAN_REPORT_ID_ANGLE ||
	    field->report->id == FAN_REPORT_ID_DUTY)
		return -1;

	if (field->report->id != FAN_REPORT_ID_CONSUMER)
		return 0;

	switch (usage_code) {
		case CONSUMER_AC_ZOOM_IN:
			hid_map_usage_clear(hi, usage, bit, max, EV_KEY, KEY_ZOOMIN);
			return 1;
		case CONSUMER_AC_ZOOM_OUT:
			hid_map_usage_clear(hi, usage, bit, max, EV_KEY, KEY_ZOOMOUT);
			return 1;
		default:
			return -1;
	}
}

static int usbhid_fan_probe(struct hid_device *hdev,
			    const struct hid_device_id *id)
{
	struct usbhid_fan *fan;
	char name[THERMAL_NAME_LENGTH];
	int ret;

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed: %d\n", ret);
		return ret;
	}

	/*
	 * hidraw delivers report 1 to the Hall sub-HAL.
	 * hid-input delivers report 4 as KEY_ZOOMIN / KEY_ZOOMOUT.
	 * Report 3 is written below with SET_REPORT.
	 */
	ret = hid_hw_start(hdev, HID_CONNECT_HIDINPUT | HID_CONNECT_HIDRAW);
	if (ret) {
		hid_err(hdev, "hw start failed: %d\n", ret);
		return ret;
	}

	fan = devm_kzalloc(&hdev->dev, sizeof(*fan), GFP_KERNEL);
	if (!fan) {
		ret = -ENOMEM;
		goto stop;
	}

	fan->hdev = hdev;
	mutex_init(&fan->lock);
	hid_set_drvdata(hdev, fan);

	mutex_lock(&fan->lock);
	ret = usbhid_fan_set_duty(fan, 0);
	mutex_unlock(&fan->lock);
	if (ret)
		goto stop;

	ret = sysfs_create_group(&hdev->dev.kobj, &usbhid_fan_group);
	if (ret)
		goto stop;

	scnprintf(name, sizeof(name), "fan-usbhid");
	fan->cdev = thermal_cooling_device_register(name, fan,
						    &usbhid_fan_cooling_ops);
	if (IS_ERR(fan->cdev)) {
		ret = PTR_ERR(fan->cdev);
		fan->cdev = NULL;
		hid_err(hdev, "cooling device register failed: %d\n", ret);
		goto remove_sysfs;
	}

	hid_info(hdev, "fan cooling device %s ready\n", name);
	return 0;

remove_sysfs:
	sysfs_remove_group(&hdev->dev.kobj, &usbhid_fan_group);
stop:
	hid_hw_stop(hdev);
	return ret;
}

static void usbhid_fan_remove(struct hid_device *hdev)
{
	struct usbhid_fan *fan = hid_get_drvdata(hdev);

	if (!fan)
		return;

	if (fan->cdev)
		thermal_cooling_device_unregister(fan->cdev);

	sysfs_remove_group(&hdev->dev.kobj, &usbhid_fan_group);

	mutex_lock(&fan->lock);
	fan->dead = true;
	mutex_unlock(&fan->lock);

	hid_hw_stop(hdev);
}

static int usbhid_fan_suspend(struct hid_device *hdev, pm_message_t message)
{
	struct usbhid_fan *fan = hid_get_drvdata(hdev);

	(void)message;

	mutex_lock(&fan->lock);
	/* Stop the blades but keep the setpoint for resume. */
	usbhid_fan_send(fan, 0);
	mutex_unlock(&fan->lock);
	return 0;
}

static int usbhid_fan_resume(struct hid_device *hdev)
{
	struct usbhid_fan *fan = hid_get_drvdata(hdev);
	int ret;

	mutex_lock(&fan->lock);
	ret = usbhid_fan_send(fan, fan->duty);
	mutex_unlock(&fan->lock);
	return ret;
}

static struct hid_device_id usbhid_fan_ids[] = {
	{ HID_USB_DEVICE(FAN_USBHID_VENDOR_ID, FAN_USBHID_PRODUCT_ID) },
	{ }
};

static struct hid_driver usbhid_fan_driver = {
	.name = "usbhid_fan",
	.id_table = usbhid_fan_ids,
	.probe = usbhid_fan_probe,
	.remove = usbhid_fan_remove,
	.input_mapping = usbhid_fan_input_mapping,
	.suspend = usbhid_fan_suspend,
	.reset_resume = usbhid_fan_resume,
};

static int __init usbhid_fan_init(void)
{
	usbhid_fan_ids[0].vendor = vendor;
	usbhid_fan_ids[0].product = product;
	return hid_register_driver(&usbhid_fan_driver);
}

static void __exit usbhid_fan_exit(void)
{
	hid_unregister_driver(&usbhid_fan_driver);
}

module_init(usbhid_fan_init);
module_exit(usbhid_fan_exit);

MODULE_DEVICE_TABLE(hid, usbhid_fan_ids);
MODULE_DESCRIPTION("USB HID fan cooling device");
MODULE_LICENSE("GPL v2");
