// SPDX-License-Identifier: GPL-2.0-only
/*
 * USB HID fan cooling device.
 */

#include <linux/hid.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/thermal.h>

#define FAN_USBHID_VENDOR_ID	0x2341
#define FAN_USBHID_PRODUCT_ID	0x0074

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

/* Millidegree C. step_wise snaps to the duty when lower == upper.
 *
 * Cool as soon as a heavy task warms the CPU into the low-30s, while it
 * is still under 40C. Max duty stays gated on the last trip.
 */
#define FAN_MAX_TEMP		40000
#define FAN_MAX_HYST		3000
/* Consecutive 1s polls on one side of the max trip before it changes. */
#define FAN_STABLE_POLLS	3

struct usbhid_fan_trip {
	int temp;
	int hyst;
	unsigned long duty;
};

static const struct usbhid_fan_trip usbhid_fan_trips[] = {
	{ 33000, 2000, 0 },
	{ 37000, 2000, 160 },
	{ FAN_MAX_TEMP, FAN_MAX_HYST, FAN_SPEED_MAX },
};

#define FAN_TRIP_NUM		ARRAY_SIZE(usbhid_fan_trips)
#define FAN_MAX_TRIP		(FAN_TRIP_NUM - 1)
#define FAN_POLL_MS		1000

struct usbhid_fan {
	struct hid_device *hdev;
	struct thermal_cooling_device *cdev;
	struct thermal_zone_device *tz;
	struct thermal_zone_device *cpu_tz;
	struct mutex lock;
	struct delayed_work poll;
	unsigned int duty;
	int cpu_temp;
	bool trip_on[FAN_TRIP_NUM];
	bool max_on;
	u8 max_hot_polls;
	u8 max_cool_polls;
	bool dead;
};

static unsigned short vendor = FAN_USBHID_VENDOR_ID;
static unsigned short product = FAN_USBHID_PRODUCT_ID;
static char *cpu_zone = "cpu-1-0";
module_param(vendor, ushort, 0444);
MODULE_PARM_DESC(vendor, "USB vendor id of the fan (default 0x2341)");
module_param(product, ushort, 0444);
MODULE_PARM_DESC(product, "USB product id of the fan (default 0x0074)");
module_param(cpu_zone, charp, 0444);
MODULE_PARM_DESC(cpu_zone, "CPU thermal zone to follow (default cpu-1-0)");

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

static int usbhid_fan_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct usbhid_fan *fan = tz->devdata;

	/*
	 * The thermal core calls this with tz->lock held. Reading the CPU
	 * zone here would take a second thermal-zone lock of the same class.
	 */
	if (fan->cpu_temp == THERMAL_TEMP_INVALID)
		return -EAGAIN;

	*temp = READ_ONCE(fan->cpu_temp);
	return 0;
}

/*
 * Max duty only after the CPU has stayed at or above 40C. A few samples
 * on either side of that line, or anywhere inside the hysteresis band,
 * leave the latch where it is so the Peltier does not chatter.
 */
static void usbhid_fan_update_max(struct usbhid_fan *fan, int temp)
{
	if (temp >= FAN_MAX_TEMP) {
		fan->max_cool_polls = 0;
		if (fan->max_hot_polls < FAN_STABLE_POLLS)
			fan->max_hot_polls++;
		if (fan->max_hot_polls >= FAN_STABLE_POLLS)
			WRITE_ONCE(fan->max_on, true);
		return;
	}

	fan->max_hot_polls = 0;
	if (!READ_ONCE(fan->max_on))
		return;

	if (temp < FAN_MAX_TEMP - FAN_MAX_HYST) {
		if (fan->max_cool_polls < FAN_STABLE_POLLS)
			fan->max_cool_polls++;
		if (fan->max_cool_polls >= FAN_STABLE_POLLS)
			WRITE_ONCE(fan->max_on, false);
	} else {
		fan->max_cool_polls = 0;
	}
}

static void usbhid_fan_poll(struct work_struct *work)
{
	struct usbhid_fan *fan = container_of(work, struct usbhid_fan, poll.work);
	int temp;

	if (fan->dead || !fan->tz)
		return;

	if (!thermal_zone_get_temp(fan->cpu_tz, &temp)) {
		WRITE_ONCE(fan->cpu_temp, temp);
		usbhid_fan_update_max(fan, temp);
	}

	thermal_zone_device_update(fan->tz, THERMAL_EVENT_UNSPECIFIED);

	if (!fan->dead)
		schedule_delayed_work(&fan->poll, msecs_to_jiffies(FAN_POLL_MS));
}

/*
 * step_wise throttles while temperature >= trip. Report trip-hyst once the
 * trip has fired so the fan stays on until the temperature falls through
 * the clear point. RAISE_FULL snaps an active trip to its duty. DROPPING,
 * not DROP_FULL, clears an inactive trip: DROP_FULL would request that
 * trip's duty even while the temperature is below it, and the highest
 * request would win.
 */
static int usbhid_fan_get_trip_temp(struct thermal_zone_device *tz, int trip,
				    int *temp)
{
	struct usbhid_fan *fan = tz->devdata;
	const struct usbhid_fan_trip *point;
	int cpu_temp;

	if (trip < 0 || trip >= FAN_TRIP_NUM)
		return -EINVAL;

	point = &usbhid_fan_trips[trip];
	cpu_temp = READ_ONCE(fan->cpu_temp);

	/*
	 * The max trip is latched in the poll. Reporting the real 40C point
	 * while a single sample is already hot would let step_wise throttle
	 * before the temperature has stayed there. While the latch is held,
	 * report the clear point (or the live temperature if it is lower) so
	 * a dip inside the stable window cannot drop the duty.
	 */
	if (trip == FAN_MAX_TRIP) {
		fan->trip_on[trip] = READ_ONCE(fan->max_on);
		if (fan->trip_on[trip])
			*temp = cpu_temp < point->temp - point->hyst ?
				cpu_temp : point->temp - point->hyst;
		else if (cpu_temp >= point->temp)
			*temp = cpu_temp + 1;
		else
			*temp = point->temp;
		return 0;
	}

	if (fan->trip_on[trip]) {
		if (cpu_temp < point->temp - point->hyst)
			fan->trip_on[trip] = false;
	} else if (cpu_temp >= point->temp) {
		fan->trip_on[trip] = true;
	}

	*temp = fan->trip_on[trip] ? point->temp - point->hyst : point->temp;
	return 0;
}

static int usbhid_fan_get_trip_hyst(struct thermal_zone_device *tz, int trip,
				    int *hyst)
{
	if (trip < 0 || trip >= FAN_TRIP_NUM)
		return -EINVAL;

	*hyst = usbhid_fan_trips[trip].hyst;
	return 0;
}

static int usbhid_fan_get_trip_type(struct thermal_zone_device *tz, int trip,
				    enum thermal_trip_type *type)
{
	if (trip < 0 || trip >= FAN_TRIP_NUM)
		return -EINVAL;

	*type = THERMAL_TRIP_PASSIVE;
	return 0;
}

static int usbhid_fan_get_trend(struct thermal_zone_device *tz, int trip,
				enum thermal_trend *trend)
{
	struct usbhid_fan *fan = tz->devdata;

	if (trip < 0 || trip >= FAN_TRIP_NUM)
		return -EINVAL;

	if (trip == FAN_MAX_TRIP)
		fan->trip_on[trip] = READ_ONCE(fan->max_on);

	*trend = fan->trip_on[trip] ? THERMAL_TREND_RAISE_FULL :
				      THERMAL_TREND_DROPPING;
	return 0;
}

static int usbhid_fan_bind(struct thermal_zone_device *tz,
			   struct thermal_cooling_device *cdev)
{
	struct usbhid_fan *fan = tz->devdata;
	int i, ret;

	if (cdev != fan->cdev)
		return 0;

	for (i = 0; i < FAN_TRIP_NUM; i++) {
		ret = thermal_zone_bind_cooling_device(tz, i, cdev,
						       usbhid_fan_trips[i].duty,
						       usbhid_fan_trips[i].duty,
						       THERMAL_WEIGHT_DEFAULT);
		if (ret) {
			hid_err(fan->hdev, "bind trip %d failed: %d\n", i, ret);
			while (i--)
				thermal_zone_unbind_cooling_device(tz, i, cdev);
			return ret;
		}
	}

	return 0;
}

static int usbhid_fan_unbind(struct thermal_zone_device *tz,
			     struct thermal_cooling_device *cdev)
{
	struct usbhid_fan *fan = tz->devdata;
	int i;

	if (cdev != fan->cdev)
		return 0;

	for (i = 0; i < FAN_TRIP_NUM; i++)
		thermal_zone_unbind_cooling_device(tz, i, cdev);

	return 0;
}

static struct thermal_zone_device_ops usbhid_fan_tz_ops = {
	.bind = usbhid_fan_bind,
	.unbind = usbhid_fan_unbind,
	.get_temp = usbhid_fan_get_temp,
	.get_trip_type = usbhid_fan_get_trip_type,
	.get_trip_temp = usbhid_fan_get_trip_temp,
	.get_trip_hyst = usbhid_fan_get_trip_hyst,
	.get_trend = usbhid_fan_get_trend,
};

static struct thermal_zone_params usbhid_fan_tzp = {
	.governor_name = "step_wise",
	.no_hwmon = true,
};

static int usbhid_fan_register_zone(struct usbhid_fan *fan)
{
	struct thermal_zone_device *cpu_tz;
	int temp, ret;

	if (!cpu_zone || strlen(cpu_zone) >= THERMAL_NAME_LENGTH) {
		hid_err(fan->hdev, "cpu thermal zone name invalid\n");
		return -EINVAL;
	}

	cpu_tz = thermal_zone_get_zone_by_name(cpu_zone);
	if (IS_ERR(cpu_tz)) {
		hid_err(fan->hdev, "cpu thermal zone %s missing: %ld\n",
			cpu_zone, PTR_ERR(cpu_tz));
		return PTR_ERR(cpu_tz);
	}

	fan->cpu_tz = cpu_tz;
	fan->cpu_temp = THERMAL_TEMP_INVALID;
	if (!thermal_zone_get_temp(cpu_tz, &temp))
		fan->cpu_temp = temp;

	/*
	 * Both delays stay 0. usbhid_fan_poll samples the CPU zone and kicks
	 * this zone; polling here would nest thermal-zone locks in get_temp.
	 * Registration runs step_wise once before it returns.
	 */
	fan->tz = thermal_zone_device_register("usbhid-fan", FAN_TRIP_NUM, 0,
					       fan, &usbhid_fan_tz_ops,
					       &usbhid_fan_tzp, 0, 0);
	if (IS_ERR(fan->tz)) {
		ret = PTR_ERR(fan->tz);

		fan->tz = NULL;
		fan->cpu_tz = NULL;
		hid_err(fan->hdev, "thermal zone register failed: %d\n", ret);
		return ret;
	}

	schedule_delayed_work(&fan->poll, msecs_to_jiffies(FAN_POLL_MS));
	return 0;
}

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
	fan->cpu_temp = THERMAL_TEMP_INVALID;
	mutex_init(&fan->lock);
	INIT_DELAYED_WORK(&fan->poll, usbhid_fan_poll);
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

	ret = usbhid_fan_register_zone(fan);
	if (ret)
		hid_err(hdev, "fan follows no cpu zone, speed_control still works\n");

	hid_info(hdev, "fan cooling device %s ready, tracking %s\n",
		 name, fan->tz ? cpu_zone : "nothing");
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

	mutex_lock(&fan->lock);
	fan->dead = true;
	mutex_unlock(&fan->lock);

	cancel_delayed_work_sync(&fan->poll);

	if (fan->tz)
		thermal_zone_device_unregister(fan->tz);

	if (fan->cdev)
		thermal_cooling_device_unregister(fan->cdev);

	sysfs_remove_group(&hdev->dev.kobj, &usbhid_fan_group);

	hid_hw_stop(hdev);
}

#ifdef CONFIG_PM
static int usbhid_fan_suspend(struct hid_device *hdev, pm_message_t message)
{
	struct usbhid_fan *fan = hid_get_drvdata(hdev);

	(void)message;

	if (!fan)
		return 0;

	cancel_delayed_work_sync(&fan->poll);

	mutex_lock(&fan->lock);
	/* Stop the blades but keep the setpoint for resume. */
	usbhid_fan_send(fan, 0);
	mutex_unlock(&fan->lock);
	return 0;
}

static int usbhid_fan_resume(struct hid_device *hdev)
{
	struct usbhid_fan *fan = hid_get_drvdata(hdev);
	int ret = 0;

	if (!fan)
		return 0;

	mutex_lock(&fan->lock);
	ret = usbhid_fan_send(fan, fan->duty);
	mutex_unlock(&fan->lock);

	if (fan->tz)
		schedule_delayed_work(&fan->poll, 0);
	return ret;
}
#endif

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
#ifdef CONFIG_PM
	.suspend = usbhid_fan_suspend,
	.resume = usbhid_fan_resume,
	.reset_resume = usbhid_fan_resume,
#endif
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
