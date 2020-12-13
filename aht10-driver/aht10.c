/*
 AHT10 Temperature and Humidity sensor sysfs driver
 Copyright (C) 2020 Johannes Cornelis Draaijer

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2
 of the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/err.h>
#include <asm/div64.h>

#define AHT10_ADDR 0x38

// Delays

#define AHT10_POWERON_USEC_DELAY 40000
#define AHT10_MEAS_USEC_DELAY 80000
#define AHT10_CMD_USEC_DELAY 350000
#define AHT10_USEC_DELAY_OFFSET 100000

// Command bytes

#define AHT10_CMD_INIT 0b11100001
#define AHT10_CMD_MEAS 0b10101100
#define AHT10_CMD_RST  0b10111010

// Flags in the answer byte/command

#define AHT10_RESP_ERROR 0xFF

#define AHT10_CAL_ENABLED (1u << 3u)
#define AHT10_BUSY        (1u << 7u)
#define AHT10_MODE_NOR    (0b11u << 5u)
#define AHT10_MODE_CYC    (0b01u << 5u)
#define AHT10_MODE_CMD    (0b10u << 5u)

#define AHT10_MAX_POLL_INTERVAL_LEN 30

// Full commands

const u8 cmd_init[] = {AHT10_CMD_INIT, AHT10_CAL_ENABLED | AHT10_MODE_CYC, 0x00};
const u8 cmd_meas[] = {AHT10_CMD_MEAS, 0x33, 0x00};
const u8 cmd_rst[] = {AHT10_CMD_RST, 0x00, 0x00};

struct aht10_measurement {
    u8 data[6];
    u8 status;
    int temperature;
    int humidity;
};

struct aht10_data {
    struct i2c_client *client;
    int initialized;
    int number;
    struct aht10_measurement current_measurement;
    struct mutex lock;
    ktime_t poll_interval;
    ktime_t previous_poll_time;
};

static int aht10_init(struct i2c_client *client)
{
    struct aht10_data *data = i2c_get_clientdata(client);
    struct mutex *mutex = &data->lock;

    int res;
    u8 status;

    mutex_lock(mutex);

    usleep_range(AHT10_POWERON_USEC_DELAY, AHT10_POWERON_USEC_DELAY + AHT10_USEC_DELAY_OFFSET);
    i2c_master_send(client, cmd_init, 3);
    usleep_range(AHT10_CMD_USEC_DELAY, AHT10_CMD_USEC_DELAY + AHT10_USEC_DELAY_OFFSET);

    res = i2c_master_recv(client, &status, 1);

    if (res != 1) {
        mutex_unlock(mutex);
        return 1;
    }

    data->initialized = 1;

    if (status & AHT10_BUSY) {
        pr_warn("AHT10 busy flag is enabled! Is another program already using the AHT10?\n");
    }

    mutex_unlock(mutex);
    return 0;
}

static int aht10_read_data(struct i2c_client *client, struct aht10_data *aht10_data, struct aht10_measurement *measurement){
    u32 temp, hum;
    int hum_i, temp_i;
    int res;
    struct mutex *mutex = &aht10_data->lock;
    int was_locked = mutex_is_locked(mutex);

    u8 *raw_data = measurement->data;
    mutex_lock(mutex);
    if (!was_locked){
        i2c_master_send(client, cmd_meas, 3);
        usleep_range(AHT10_MEAS_USEC_DELAY, AHT10_MEAS_USEC_DELAY + AHT10_USEC_DELAY_OFFSET);

        res = i2c_master_recv(client, raw_data, 6);

        if (res != 6) {
            mutex_unlock(mutex);
            pr_warn("Did not receive 6 bytes from AHT10!\n");
            return 1;
        }

        temp = ((u32) (raw_data[3] & 0x0Fu) << 16u) | ((u32) raw_data[4] << 8u) | raw_data[5];
        hum = ((u32) raw_data[1] << 12u) | ((u32) raw_data[2] << 4u) | (raw_data[3] & 0xF0u >> 4u);

        // Avoid doing float arithmetic, while trying to preserve precision
        // There must be a better way to do this (or by using 64 bit values)
        // temp = (temp * 200)/2^20 - 50

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

        measurement->temperature = temp_i;
        measurement->humidity = hum_i;
    }
    mutex_unlock(mutex);
    return 0;
}

static int aht10_check_and_set_polltime(struct aht10_data *data)
{
    ktime_t current_time = ktime_get_boottime();
    ktime_t difference = ktime_sub(current_time, data->previous_poll_time);
    if (ktime_to_us(difference) >= ktime_to_us(data->poll_interval)) {
        data->previous_poll_time = current_time;
        return 1;
    }
    return 0;
}

static ssize_t temperature_show(struct device *dev,
                                    struct device_attribute *attr, char* buf)
{
    int bytes_written;
    struct i2c_client *client = dev_get_drvdata(dev);
    struct aht10_data *data = i2c_get_clientdata(client);
    struct aht10_measurement *measurement = &data->current_measurement;

    if (aht10_check_and_set_polltime(data)) {
        aht10_read_data(client, data, measurement);
    }

    bytes_written = sprintf(buf, "%d.%d", measurement->temperature / 100, measurement->temperature % 100);
    return bytes_written;
}

static ssize_t humidity_show(struct device *dev,
                                    struct device_attribute *attr, char* buf)
{
    int bytes_written;
    struct i2c_client *client = dev_get_drvdata(dev);
    struct aht10_data *data = i2c_get_clientdata(client);
    struct aht10_measurement *measurement = &data->current_measurement;

    if (aht10_check_and_set_polltime(data)) {
        aht10_read_data(client, data, measurement);
    }

    bytes_written = sprintf(buf, "%d.%d", measurement->humidity / 100, measurement->humidity % 100);
    return bytes_written;
}

static ssize_t reset_store(struct device *dev,
                        struct device_attribute *attr, const char *buf, size_t count)
{
    // struct i2c_client *client = dev_get_drvdata(dev);
    // struct aht10_data *data = i2c_get_clientdata(client);
    // TODO
    return count;
}

static ssize_t min_poll_interval_show(struct device *dev,
                                    struct device_attribute *attr, char* buf)
{
    struct i2c_client *client = dev_get_drvdata(dev);
    struct aht10_data *data = i2c_get_clientdata(client);
    int bytes_written;

    u64 usec = ktime_to_us(data->poll_interval);
    do_div(usec, USEC_PER_MSEC);
    bytes_written = sprintf(buf, "%lld", usec);
    return bytes_written;
}

static ssize_t min_poll_interval_store(struct device *dev,
                        struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = dev_get_drvdata(dev);
    struct aht10_data *data = i2c_get_clientdata(client);
    int i;
    u64 msecs;
    int res;

    char null_terminated[AHT10_MAX_POLL_INTERVAL_LEN + 1];

    if (count > AHT10_MAX_POLL_INTERVAL_LEN) {
        pr_warn("AHT10: Warning! Input too long. Max characters: %d\n", AHT10_MAX_POLL_INTERVAL_LEN);
        return count;
    }

    for (i = 0; i < count && i < AHT10_MAX_POLL_INTERVAL_LEN; i++) {
        null_terminated[i] = buf[i];
    }
    null_terminated[i] = 0;

    res = kstrtoull(null_terminated, 10, &msecs);

    if (res) {
        pr_warn("AHT10: Warning! Invalid input.\n");
        return count;
    }

    data->poll_interval = ms_to_ktime(msecs);
    return count;
}

static DEVICE_ATTR_WO(reset);
static DEVICE_ATTR_RO(temperature);
static DEVICE_ATTR_RO(humidity);
static DEVICE_ATTR_RW(min_poll_interval);

static struct attribute *aht10_attributes[] = {
    &dev_attr_reset.attr,
    &dev_attr_temperature.attr,
    &dev_attr_humidity.attr,
    &dev_attr_min_poll_interval.attr,
    NULL,
};

static const struct attribute_group aht10_attribute_group = {
    .attrs = aht10_attributes,
};

static struct device *dev;

static int aht10_probe(struct i2c_client *client,
                    const struct i2c_device_id *aht10_id)
{
    struct device *device = &client->dev;
    struct aht10_data *data;
    int res = 0;

    if (client->addr != AHT10_ADDR){
        return 0;
    }

    data = devm_kzalloc(device, sizeof(*data), GFP_KERNEL);

    if (data == NULL) {
        return 1;
    }

    data->number = 10;
    data->poll_interval = ns_to_ktime((u64) 10000 * NSEC_PER_MSEC);
    data->previous_poll_time = ns_to_ktime(0);
    i2c_set_clientdata(client, data);

    mutex_init(&data->lock);

    res = aht10_init(client);

    if (res) {
        return 2;
    }

    dev = root_device_register("aht10");
    if (IS_ERR(dev)) {
        return 3;
    }

    dev_set_drvdata(dev, client);
    res = sysfs_create_group(&dev->kobj, &aht10_attribute_group);

    if (res != 0) {
        return 4;
    }

    pr_info("AHT10 was detected and registered\n");
    return 0;
}

static int aht10_remove(struct i2c_client *client){
    if (client->addr != AHT10_ADDR){
        return 0;
    }

    if (dev) {
        root_device_unregister(dev);
    }

    pr_info("AHT10 was removed\n");
    return 0;
}

static const struct i2c_device_id aht10_id[] = {
    { "aht10", 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, aht10_id);

static const struct of_device_id aht10_of_match[] = {
    { .compatible = "aht10,aht20", },
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

static int __init aht10_init_driver(void)
{
    return i2c_add_driver(&aht10_driver);
}

static void __exit aht10_exit_driver(void)
{
    return i2c_del_driver(&aht10_driver);
}

module_init(aht10_init_driver);
module_exit(aht10_exit_driver);

MODULE_AUTHOR("Johannes Draaijer <jcdra1@gmail.com>");
MODULE_DESCRIPTION("AHT10 Temperature and Humidity sensor driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
