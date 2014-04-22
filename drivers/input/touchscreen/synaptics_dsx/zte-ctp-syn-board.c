/**********************************************************************

  Copyright (C), 2001-2013, ZTE. Co., Ltd.

 **********************************************************************
  File Name     : zte-ctp-syn-board.c
  Version       : Initial Draft
  Author        : luochangyang
  Created       : 2013/04/19
  Last Modified :
  Description   : This is touchscreen driver board file for synaptics.
  Function List :
  History       :
  1.Date        : 2013/04/19
    Author      : luochangyang
    Modification: Created file

**********************************************************************/

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/ion.h>
#include <asm/mach-types.h>
#include <mach/msm_memtypes.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/ion.h>
#include <mach/msm_bus_board.h>
#include <mach/socinfo.h>

#include <linux/i2c.h>

#include "synaptics_dsx.h"

static int synaptics_gpio_setup(unsigned gpio, bool configure)
{
	int retval = 0;

	if (configure) {
		retval = gpio_request(gpio, "rmi4_attn");
		if (retval) {
			pr_err("%s: Failed to get attn gpio %d (code: %d)",
					 __func__, gpio, retval);
			return retval;
		}

		retval = gpio_direction_input(gpio);
		if (retval) {
			pr_err("%s: Failed to setup attn gpio %d (code: %d)",
					__func__, gpio, retval);
			gpio_free(gpio);
		}
	} else {
		pr_warn("%s: No way to deconfigure gpio %d",
				__func__, gpio);
	}

	return retval;
}

#define SYNAPTICS_I2C_ADDR	0x22
#define SYNAPTICS_GPIO_IRQ  17
#define SYNAPTICS_GPIO_RST  16

static unsigned char TM_f1a_button_codes[] = {KEY_MENU, KEY_MENU, KEY_HOME, KEY_BACK, KEY_BACK};

static struct synaptics_dsx_cap_button_map TM_cap_button_map = {
	.nbuttons = ARRAY_SIZE(TM_f1a_button_codes),
	.map = TM_f1a_button_codes,
};

static struct synaptics_dsx_platform_data dsx_platformdata = {
    .regulator_en = true,
	.irq_flags = IRQF_TRIGGER_FALLING,
	.irq_gpio = SYNAPTICS_GPIO_IRQ,
	.reset_gpio = SYNAPTICS_GPIO_RST,
	.gpio_config = synaptics_gpio_setup,
	.cap_button_map = &TM_cap_button_map,
};

static struct i2c_board_info synaptics_dsx_i2c_devices[] = {
 	{
 		I2C_BOARD_INFO("synaptics_dsx_i2c,rmi4", SYNAPTICS_I2C_ADDR),
 		.platform_data = &dsx_platformdata,
     	},	
};

static int  __init zte_ctp_synaptics_init(void)
{
	int rc = 0;
    
#ifndef CONFIG_OF
//	pr_info("TouchScreen I2C device setup");
	if (ARRAY_SIZE(synaptics_dsx_i2c_devices)) {
		rc = i2c_register_board_info(2, 
                synaptics_dsx_i2c_devices,
				ARRAY_SIZE(synaptics_dsx_i2c_devices));
	}
#endif

	return rc;
}

arch_initcall(zte_ctp_synaptics_init);

