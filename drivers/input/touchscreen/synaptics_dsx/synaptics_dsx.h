/*
 * Synaptics RMI4 touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
 */

#ifndef _SYNAPTICS_DSX_H_
#define _SYNAPTICS_DSX_H_
/*** ZTEMT Added by luochangyang, 2013/04/28 ***/
#ifdef CONFIG_ZTEMT_TOUCHSCREEN_SYNAPTICS_DSX
struct synaptics_dsx_cap_button_map {
	unsigned char nbuttons;
	unsigned char *map;
};

struct synaptics_dsx_platform_data {
	bool x_flip;
	bool y_flip;
	bool regulator_en;
	int irq_gpio;
	u32 irq_flags;
	int reset_gpio;
	int (*gpio_config)(unsigned gpio, bool configure);
	struct synaptics_dsx_cap_button_map *cap_button_map;
};
#endif
/***ZTEMT END***/

#endif
