/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
*
* File Name          : maxq616.c
* Authors            : Zhu Bing
* Version            : V.1.0.0
* Date               : 07/024/2013
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
********************************************************************************
Version History.
 
Revision 1-0-0 07/024/2013
 first revision

*******************************************************************************/
 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>



#define LOG_TAG "IR_REMOTE"
#define DEBUG_ON //DEBUG SWITCH

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/')+1) : __FILE__

#ifdef  CONFIG_FEATURE_ZTEMT_SENSORS_LOG_ON
#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR   "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
    #ifdef  DEBUG_ON
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO  "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
                                              
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
    #else
#define SENSOR_LOG_INFO(fmt, args...)
#define SENSOR_LOG_DEBUG(fmt, args...)
    #endif

#else
#define SENSOR_LOG_ERROR(fmt, args...)
#define SENSOR_LOG_INFO(fmt, args...)
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif


static int maxq616_power_on(struct device *dev)
{
	int rc;
    static struct regulator *vdd_chip;
    //static struct regulator *vdd_led;

    SENSOR_LOG_INFO("start\n");

	vdd_chip = regulator_get(dev, "vdd-chip");
	if (IS_ERR(vdd_chip))
    {
		rc = PTR_ERR(vdd_chip);
		SENSOR_LOG_ERROR("Regulator get failed vdd_chip rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(vdd_chip) > 0)
    {
		rc = regulator_set_voltage(vdd_chip, 1800000, 1800000);
		if (rc)
        {
			SENSOR_LOG_ERROR("Regulator set vdd_chip failed rc=%d\n", rc);
			goto error_set_vtg_vdd_chip;
		}
	}
    
    rc = regulator_set_optimum_mode(vdd_chip, 600000);
    if (rc < 0)
    {
        SENSOR_LOG_ERROR("Regulator vdd_chip set_opt failed rc=%d\n", rc);
        goto error_set_optimum_vdd_chip;

    }
    
    rc = regulator_enable(vdd_chip);
    if (rc)
    {
        SENSOR_LOG_ERROR("Regulator vdd_chip enable failed rc=%d\n", rc);
        goto error_reg_en_vdd_chip;
    }
    
    SENSOR_LOG_INFO("success\n");
    return 0;

#if 0
error_reg_en_vcc_i2c:
    reg_set_optimum_mode_check(vcc_i2c, 0);
//error_set_vtg_i2c:
//    regulator_put(vcc_i2c);
error_reg_opt_vcc_dig:
    regulator_disable(vcc_ana);
#endif
error_set_optimum_vdd_chip:
    regulator_set_voltage(vdd_chip, 0, 1800000);
    regulator_put(vdd_chip);
error_reg_en_vdd_chip:
    regulator_set_optimum_mode(vdd_chip, 0);
error_set_vtg_vdd_chip:
	regulator_put(vdd_chip);
	return rc;
}

 
 
static int __devinit maxq616_probe(struct i2c_client *client,
                  const struct i2c_device_id *id)
{

    SENSOR_LOG_INFO("prob start\n");

    maxq616_power_on(&client->dev);

    SENSOR_LOG_INFO("prob success\n");

    return 0;
}

 
 /**
  * maxq616_remove() - remove device
  * @client: I2C client device
  */
 static int __devexit maxq616_remove(struct i2c_client *client)
 {
     struct shtc1_data *chip_data = i2c_get_clientdata(client);
 
     SENSOR_LOG_INFO("maxq616_remove\n");
    
     kfree(chip_data);
     return 0;
 }


static const struct i2c_device_id maxq616_idtable_id[] = {
     { "uie,maxq616", 0 },
     { },
 };
 
static struct of_device_id of_maxq616_idtable[] = {
     { .compatible = "uei,maxq616",},
     {}
};
 
 MODULE_DEVICE_TABLE(i2c, maxq616_idtable);
 
 static struct i2c_driver maxq616_driver = {
     .driver = {
         .name = "maxq616",
         .of_match_table = of_maxq616_idtable,
         .pm = NULL,
     },
     .id_table = maxq616_idtable_id,
     .probe = maxq616_probe,
     .remove = __devexit_p(maxq616_remove),
 };



static int __init maxq616_init(void)
{
        SENSOR_LOG_INFO("driver: init\n");
        return i2c_add_driver(&maxq616_driver);
}
 
static void __exit maxq616_exit(void)
{
        SENSOR_LOG_INFO("driver: exit\n");
        i2c_del_driver(&maxq616_driver);
}

module_init(maxq616_init);
module_exit(maxq616_exit);
 
MODULE_DESCRIPTION("UEI maxq616 driver");
MODULE_AUTHOR("ZhuBing, ZTEMT");
MODULE_LICENSE("GPL");
