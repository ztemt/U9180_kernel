/*
 * Copyright (C) 2012 Sensirion AG, Switzerland
 * Author: Johannes Winkelmann <johannes.winkelmann@sensirion.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SHTC1_H_
#define __SHTC1_H_

struct shtc1_pdata {
	bool blocking_io;
	bool high_precision;
};

struct shtc1_data {

    struct mutex lock;

    struct i2c_client *client;
    struct shtc1_pdata pdata;

    struct device *temperature_dev;
    struct device *humidity_dev;

	struct input_dev *t_idev;
	struct input_dev *h_idev;

    const char *input_dev_t_name;
    const char *input_dev_h_name;


	struct delayed_work poll_work;

    unsigned int temperature_poll_time;
    unsigned int humidity_poll_time;
    unsigned int poll_time;
    unsigned int poll_time_minimum;


    bool valid;
    unsigned long last_updated; /* In jiffies */

    const unsigned char *command;
    unsigned int nonblocking_wait_time;


    int temperature_data;
    int humidity_data;

    bool temperature_state;
    bool humidity_state;
    
    bool temperature_debug;
    bool humidity_debug;

};

static int __devinit shtc1_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit shtc1_remove(struct i2c_client *client);
static void shtc1_input_humidity(struct shtc1_data *chip_data);
static void shtc1_input_temperature(struct shtc1_data *chip_data);

#endif
