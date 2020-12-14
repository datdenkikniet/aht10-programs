// SPDX-License-Identifier: GPL-2.0-only

/*
 * aht10.c - Linux hwmon driver for AHT10 I2C Temperature and Humidity sensor
 * Copyright (C) 2020 Johannes Cornelis Draaijer
 */

#include <asm/div64.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>

#define AHT10_ADDR 0x38
#define AHT10_MEAS_SIZE 6

/*
 * Delays
 */
#define AHT10_POWERON_USEC_DELAY 40000
#define AHT10_MEAS_USEC_DELAY 80000
#define AHT10_CMD_USEC_DELAY 350000
#define AHT10_USEC_DELAY_OFFSET 100000

/*
 * Command bytes
 */
#define AHT10_CMD_INIT 0b11100001
#define AHT10_CMD_MEAS 0b10101100
#define AHT10_CMD_RST  0b10111010

/*
 * Flags in the answer byte/command
 */
#define AHT10_RESP_ERROR	0xFF

#define AHT10_CAL_ENABLED	(1u << 3u)
#define AHT10_BUSY	(1u << 7u)
#define AHT10_MODE_NOR	(0b11u << 5u)
#define AHT10_MODE_CYC	(0b01u << 5u)
#define AHT10_MODE_CMD	(0b10u << 5u)

#define AHT10_MAX_POLL_INTERVAL_LEN	30

/**
 *   struct aht10_data - All the data required to operate an AHT10 chip
 *   @client: the i2c client associated with the AHT10
 *   @lock: a mutex that is used to prevent parallel access to the
 *          i2c client
 *   @current_measurement: the last-measured values of the AHT10
 *   @poll_interval: the minimum poll interval
 *                   While the poll rate is not 100% necessary,
 *                   the datasheet recommends that a measurement
 *                   is not performed more too often to prevent
 *                   the chip from "heating up". If it's
 *                   unwanted, it can be ignored by setting
 *                   it to 0.
 *   @previous_poll_time: the previous time that the AHT10
 *                        was polled
 *   @temperature: the latest temperature value received from
 *                 the AHT10
 *   @humidity: the latest humidity value received from the 
 *              AHT10
 */

struct aht10_data {
	struct i2c_client *client;
	struct mutex lock;
	struct aht10_measurement current_measurement;
	ktime_t poll_interval;
	ktime_t previous_poll_time;
	int temperature;
	int humidity;
};

/**
 * aht10_init() - Initialize an AHT10 chip
 * @client: the i2c client associated with the AHT10
 * @data: the data associated with this AHT10 chip
 * Return: 0 if succesfull, 1 if not
 */
static int aht10_init(struct i2c_client *client, struct aht10_data *data)
{
	const u8 cmd_init[] = {AHT10_CMD_INIT, AHT10_CAL_ENABLED | AHT10_MODE_CYC,
		0x00};
	int res;
	u8 status;

	res = i2c_master_send(client, cmd_init, 3);
	if (res < 0)
		return res;
	
	usleep_range(AHT10_CMD_USEC_DELAY, AHT10_CMD_USEC_DELAY +
		AHT10_USEC_DELAY_OFFSET);

	res = i2c_master_recv(client, &status, 1);
	if (res != 1)
		return -ENODATA;

	if (status & AHT10_BUSY)
		return -EBUSY;

	return 0;
}

/**
 * aht10_read_data() - read and parse the raw data from the AHT10
 * @client: the i2c client associated with the AHT10
 * @aht10_data: the struct aht10_data to use for the lock
 * Return: 0 if succesfull, 1 if not
 */
static int aht10_read_data(struct i2c_client *client,
			struct aht10_data *aht10_data)
{
	const u8 cmd_meas[] = {AHT10_CMD_MEAS, 0x33, 0x00};
	u32 temp, hum;
	int hum_i, temp_i;
	int res;
	struct mutex *mutex = &aht10_data->lock;
	int was_locked = mutex_is_locked(mutex);
	u8 *raw_data[AHT10_MEAS_SIZE];

	mutex_lock(mutex);
	if (!was_locked) {
		res = i2c_master_send(client, cmd_meas, sizeof(cmd_meas));
		if (res < 0)
			return res;
		
		usleep_range(AHT10_MEAS_USEC_DELAY,
			AHT10_MEAS_USEC_DELAY + AHT10_USEC_DELAY_OFFSET);

		res = i2c_master_recv(client, raw_data, AHT10_MEAS_SIZE);
		if (res != 6) {
			mutex_unlock(mutex);
			if (res >= 0)
				return -ENODATA;
			else
				return res;
		}

		temp = ((u32) (raw_data[3] & 0x0Fu) << 16u) | ((u32) raw_data[4] << 8u) | raw_data[5];
		hum = ((u32) raw_data[1] << 12u) | ((u32) raw_data[2] << 4u) | (raw_data[3] & 0xF0u >> 4u);

		/*
		 * Avoid doing float arithmetic, while trying to preserve
		 * precision. There must be a better way to do this (or
		 * by using 64 bit values)
		 */

		temp = temp * 200;
		temp = temp >> 10u;
		temp = temp * 100;
		temp = temp >> 10u;

		hum = hum * 100;
		hum = hum >> 10u;
		hum = hum * 100;
		hum = hum >> 10u;

		temp_i = temp - 5000;
		hum_i = hum;

		data->temperature = temp_i;
		data->humidity = hum_i;
	}
	mutex_unlock(mutex);
	return 0;
}

/**
 * aht10_check_and_set_polltime() - check if the minimum poll interval has
 *                                  expired, and if so set the previous
 *                                  poll time
 * @data: what time to compare (and possibly set)
 * Return: 1 if the minimum poll interval has expired, 0 if not
 */
static int aht10_check_and_set_polltime(struct aht10_data *data)
{
	ktime_t current_time = ktime_get_boottime();
	ktime_t difference = ktime_sub(current_time,
				data->previous_poll_time);
	if (ktime_to_us(difference) >=
	ktime_to_us(data->poll_interval)) {
		data->previous_poll_time = current_time;
		return 1;
	}
	return 0;
}

/**
 * temperature_show() - show the temperature in Celcius
 */
static ssize_t temperature_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int res;
	struct aht10_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	if (aht10_check_and_set_polltime(data)) {
		res = aht10_read_data(client, data);
		if (res < 0)
			return res;
	}
	res = sprintf(buf, "%d", data->temperature * 10);
	return res;
}


/**
 * humidity_show() - show the relative humidity in %H
 */
static ssize_t humidity_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int res;
	struct aht10_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	if (aht10_check_and_set_polltime(data)) {
		res = aht10_read_data(client, data);
		if (res < 0)
			return res;
	}

	res = sprintf(buf, "%d", data->humidity * 10);
	return res;
}

/**
 * reset_store() - reset the ATH10
 */
static ssize_t reset_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	// TODO
	const u8 cmd_rst[] = {AHT10_CMD_RST, 0x00, 0x00};
	return count;
}

/**
 * min_poll_interval_show() - show the minimum poll interval
 *                            in milliseconds
 */
static ssize_t min_poll_interval_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct aht10_data *data = dev_get_drvdata(dev);
	int res;
	u64 usec = ktime_to_us(data->poll_interval);

	do_div(usec, USEC_PER_MSEC);
	res = sprintf(buf, "%lld", usec);
	return res;
}

/**
 * min_poll_interval_store() - store the given minimum poll interval.
 * Input in milliseconds
 */
static ssize_t min_poll_interval_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct aht10_data *data = dev_get_drvdata(dev);
	u64 msecs;
	int res;

	res = kstrtoull(buf, 10, &msecs);
	if (res < 0)
		return res;

	data->poll_interval = ms_to_ktime(msecs);
	return count;
}

static SENSOR_DEVICE_ATTR(reset, 0200, NULL, reset_store, 0);
static SENSOR_DEVICE_ATTR(temp1_input, 0444, temperature_show, NULL, 0);
static SENSOR_DEVICE_ATTR(humidity1_input, 0444, humidity_show, NULL, 0);
static SENSOR_DEVICE_ATTR(min_poll_interval, 0644, min_poll_interval_show,
						  min_poll_interval_store, 0);

static struct attribute *aht10_attrs[] = {
	&sensor_dev_attr_reset.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_humidity1_input.dev_attr.attr,
	&sensor_dev_attr_min_poll_interval.dev_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(aht10);

static int aht10_probe(struct i2c_client *client,
		const struct i2c_device_id *aht10_id)
{
	struct device *device = &client->dev;
	struct device *hwmon_dev;
	struct i2c_adapter *adapter = client->adapter;
	struct aht10_data *data;
	const struct attribute_group **attribute_groups = aht10_groups;
	int res;

	if (client->addr != AHT10_ADDR)
		return 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENOSYS;

	data = devm_kzalloc(device, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->poll_interval = ns_to_ktime((u64) 10000 * NSEC_PER_MSEC);
	data->previous_poll_time = ns_to_ktime(0);
	data->client = client;

	i2c_set_clientdata(client, data);

	mutex_init(&data->lock);

	res = aht10_init(client, data);
	if (res < 0)
		return res;

	hwmon_dev = devm_hwmon_device_register_with_groups(device,
							client->name,
							data,
							attribute_groups);

	pr_info("AHT10 was detected and registered\n");
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id aht10_id[] = {
	{ "aht10", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, aht10_id);

static const struct of_device_id aht10_of_match[] = {
	{ .compatible = "aosong,aht10", },
};

static struct i2c_driver aht10_driver = {
	.driver = {
		.name = "aht10",
		.of_match_table = aht10_of_match,
	},
	.probe      = aht10_probe,
	.remove     = aht10_remove,
	.id_table   = aht10_id,
};

module_i2c_driver(aht10_driver);

MODULE_AUTHOR("Johannes Draaijer <jcdra1@gmail.com>");
MODULE_DESCRIPTION("AHT10 Temperature and Humidity sensor driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
