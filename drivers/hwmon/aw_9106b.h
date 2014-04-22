/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __AW_9106B_H__
#define __AW_9106B_H__

#include <linux/types.h>
#include <linux/platform_device.h>
#include <mach/board.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>

#define AW_DEVICE_NAME "aw_9106b"

enum aw_outn{
	AW_OUT_0,
	AW_OUT_1,
	AW_OUT_2,
	AW_OUT_3,
	AW_OUT_4,
	AW_OUT_5,
};

struct aw9106b_plat_data {
	int led_in_breathing;
	struct i2c_client *i2c_client;
	struct i2c_driver *i2c_driver;
	struct work_struct work;
	enum aw_outn outn;
	struct hrtimer timer;
};

struct aw9106b_regs_data {
	int en_bre;    //enable or disable breath func
	int smart_blink;  //blink or smart-fade modle
	int in_out;  //gpio in and out 
	int out_val;
	int smart_fade;
	int led_gpio;  //led or gpio
	int fade_tmr; //fade time
	int full_tmr; //full light and black
	int delay_bre0; //delay breath time
	int dim0;
	int aw_reset;
	int ctl;  //set start blink
};

#endif /* __AW_9106B_H__ */


