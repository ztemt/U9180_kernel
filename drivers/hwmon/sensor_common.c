/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
*
* File Name          : hts221.c
* Authors            : Motion MEMS
*                      Morris Chen (morris.chen@st.com)
* Version            : V.1.0.0
* Date               : 06/03/2013
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
 
Revision 1-0-0 06/03/2013
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
#include <linux/i2c/sensor_common.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/miscdevice.h>



#define LOG_TAG "SENSOR_COMMON"
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

static const struct i2c_device_id sensor_common_idtable_id[] = {
    { "zte,sensor_common", 0 },
    { },
};
 
static struct of_device_id of_sensor_common_idtable[] = {
    { .compatible = "zte,sensor_common",},
    {}
};
 
 MODULE_DEVICE_TABLE(i2c, sensor_common_idtable);
 
static struct i2c_driver sensor_common_driver = {
    .driver = {
        .name = "sensor_common",
        .of_match_table = of_sensor_common_idtable,
        .pm = NULL,
    },
    .id_table = sensor_common_idtable_id,
    .probe = sensor_common_probe,
    .remove = __devexit_p(sensor_common_remove),
};


struct class  *sensor_common_class;

static dev_t const sensor_compass_dev = MKDEV(MISC_MAJOR, 248);

static ssize_t compass_int_pin_get(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "gpio is %d\n",gpio_get_value(COMPASS_INT_PIN));
}
 

static ssize_t compass_int_pin_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    bool value;
    if (strtobool(buf, &value))
        return -EINVAL;

    if (value)
    { 
        SENSOR_LOG_INFO("set to be 1\n");
        gpio_set_value(COMPASS_INT_PIN, 1);
    }
    else
    {
        SENSOR_LOG_INFO("set to be 0\n");
        gpio_set_value(COMPASS_INT_PIN, 0);
    }

    return size;
}



static struct device_attribute attrs_sensor_compass[] = {
	__ATTR(int_pin,                  0640,   compass_int_pin_get,          compass_int_pin_set),
};



static int sensor_compass_create_sysfs_interfaces(struct device *dev)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(attrs_sensor_compass); i++)
     if (device_create_file(dev, attrs_sensor_compass + i))
         goto error;
    return 0;

error:
    for ( ; i >= 0; i--)
    device_remove_file(dev, attrs_sensor_compass + i);
    SENSOR_LOG_ERROR("Unable to create interface\n");
    return -1;
}


static int sensor_compass_int_pin_init(int pin_num)
{
    int ret = 0;

    ret = gpio_request(pin_num, "compass_int");
    if (ret)    
    {
        SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",COMPASS_INT_PIN);
        
        gpio_free(pin_num);
        ret = gpio_request(pin_num, "compass_int");
        if (ret) 
        {
            SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",COMPASS_INT_PIN);
            return ret;
        }
    }
    else
    {
        SENSOR_LOG_INFO("gpio %d get success\n",COMPASS_INT_PIN);
    }

    ret = gpio_tlmm_config(GPIO_CFG(pin_num, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (ret < 0)
    {
        SENSOR_LOG_ERROR("gpio_tlmm_config failed ret = %d",ret);
    }
    
    return ret;
}

static int __devinit sensor_common_probe(struct i2c_client *client,
                  const struct i2c_device_id *id)
{

    struct sensor_common_data *chip_data;
//	struct device *dev = &client->dev;

    int ret;

	chip_data = kzalloc(sizeof(struct sensor_common_data), GFP_KERNEL);

    SENSOR_LOG_INFO("prob start\n");

    i2c_set_clientdata(client, chip_data);

    sensor_common_class = class_create(THIS_MODULE, "sensor");

    chip_data->sensor_compass_dev = device_create(sensor_common_class, NULL, sensor_compass_dev, &sensor_common_driver ,"compass");
    if (IS_ERR(chip_data->sensor_compass_dev)) 
    {
        ret = PTR_ERR(chip_data->sensor_compass_dev);
        goto create_sensor_compass__failed;
    }

    dev_set_drvdata(chip_data->sensor_compass_dev, chip_data);

    sensor_compass_create_sysfs_interfaces(chip_data->sensor_compass_dev);

    sensor_compass_int_pin_init(COMPASS_INT_PIN);


    SENSOR_LOG_INFO("prob success\n");

    return 0;

create_sensor_compass__failed:
    chip_data->sensor_compass_dev = NULL;
    class_destroy(sensor_common_class); 

    return ret;
}

 
 /**
  * sensor_common_remove() - remove device
  * @client: I2C client device
  */
 static int __devexit sensor_common_remove(struct i2c_client *client)
 {
     struct sensor_common_data *chip_data = i2c_get_clientdata(client);
 
      SENSOR_LOG_INFO("sensor_common_remove\n");
     //hwmon_device_unregister(chip_data->hwmon_dev);
     //sysfs_remove_group(&client->dev.kobj, &sensor_common_attr_group);
    
     kfree(chip_data);
     return 0;
 }

static int __init sensor_common_init(void)
{
        SENSOR_LOG_INFO("driver: init\n");
        return i2c_add_driver(&sensor_common_driver);
}
 
static void __exit sensor_common_exit(void)
{
        SENSOR_LOG_INFO("driver: exit\n");
        i2c_del_driver(&sensor_common_driver);
}

module_init(sensor_common_init);
module_exit(sensor_common_exit);
 
MODULE_DESCRIPTION("sensor commom driver");
MODULE_AUTHOR("ZhuBing, ZTEMT");
MODULE_LICENSE("GPL");
