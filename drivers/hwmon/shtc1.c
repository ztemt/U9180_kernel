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
#include <linux/i2c/shtc1.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/miscdevice.h>



#define LOG_TAG "TEMP_HUMI"
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

 static const struct i2c_device_id shtc1_idtable_id[] = {
     { "sencirion,shtc1", 0 },
     { },
 };
 
 static struct of_device_id of_shtc1_idtable[] = {
     { .compatible = "sencirion,shtc1",},
     {}
 };
 
 MODULE_DEVICE_TABLE(i2c, shtc1_idtable);
 
 static struct i2c_driver shtc1_driver = {
     .driver = {
         .name = "shtc1",
         .of_match_table = of_shtc1_idtable,
         .pm = NULL,
     },
     .id_table = shtc1_idtable_id,
     .probe = shtc1_probe,
     .remove = __devexit_p(shtc1_remove),
 };


 struct class  *temperature_class;
 struct class  *humidity_class;

 
 static dev_t const shtc1_temperature_dev_t = MKDEV(MISC_MAJOR, 248);
 static dev_t const shtc1_humidity_dev_t    = MKDEV(MISC_MAJOR, 249);

 /* commands (high precision mode) */
 static const unsigned char shtc1_cmd_measure_blocking_hpm[]    = { 0x7C, 0xA2 };
 static const unsigned char shtc1_cmd_measure_nonblocking_hpm[] = { 0x78, 0x66 };
 
 /* commands (low precision mode) */
 static const unsigned char shtc1_cmd_measure_blocking_lpm[]    = { 0x64, 0x58 };
 static const unsigned char shtc1_cmd_measure_nonblocking_lpm[] = { 0x60, 0x9c };
 
 /* delays for non-blocking i2c commands */
 /* TODO: use max timings */
#define SHTC1_NONBLOCKING_WAIT_TIME_HPM  10
#define SHTC1_NONBLOCKING_WAIT_TIME_LPM   1
 
#define SHTC1_CMD_LENGTH      2
#define SHTC1_RESPONSE_LENGTH 6
 

static void shtc1_data_init(struct shtc1_data *chip_data)
{
    chip_data->input_dev_t_name = "temperature";
    chip_data->input_dev_h_name = "humidity";

    chip_data->temperature_poll_time = 1000;
    chip_data->humidity_poll_time    = 1000;
    chip_data->poll_time             = 1000;
    chip_data->poll_time_minimum     = 20;

    chip_data->temperature_debug     = false;
    chip_data->humidity_debug        = false;
}

static void shtc1_select_command(struct shtc1_data *chip_data)
{
    if (chip_data->pdata.high_precision) 
    {
        chip_data->command = chip_data->pdata.blocking_io ?
             shtc1_cmd_measure_blocking_hpm :
             shtc1_cmd_measure_nonblocking_hpm;
        chip_data->nonblocking_wait_time = SHTC1_NONBLOCKING_WAIT_TIME_HPM;

    } 
    else 
    {
        chip_data->command = chip_data->pdata.blocking_io ?
             shtc1_cmd_measure_blocking_lpm :
             shtc1_cmd_measure_nonblocking_lpm;
        chip_data->nonblocking_wait_time = SHTC1_NONBLOCKING_WAIT_TIME_LPM;
    }
}
 
 static int shtc1_update_values(struct i2c_client *client, struct shtc1_data *chip_data, char *buf, int bufsize)
 {
     int ret = i2c_master_send(client, chip_data->command, SHTC1_CMD_LENGTH);
     if (ret < 0) {
         dev_err(&client->dev, "failed to send command");
         return ret;
     }
 
     /*
      * in blocking mode (clock stretching mode) the I2C bus
      * is blocked for other traffic, thus the call to i2c_master_recv()
      * will wait until the data is ready. for non blocking mode, we
      * have to wait ourselves, thus the msleep()
      *
      * TODO: consider usleep_range
      */
     if (!chip_data->pdata.blocking_io)
         msleep(chip_data->nonblocking_wait_time);
 
     ret = i2c_master_recv(client, buf, bufsize);
     if (ret != bufsize) {
         dev_err(&client->dev, "failed to read values: %d", ret);
         return ret;
     }
 
     return 0;
 }
 
/* sysfs attributes */
static struct shtc1_data *shtc1_update_client(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct shtc1_data *chip_data = i2c_get_clientdata(client);

    unsigned char buf[SHTC1_RESPONSE_LENGTH];
    int val;
    int ret;

    //mutex_lock(&chip_data->lock);

    /*
    * initialize 'ret' in case we had a valid result before, but
    * read too quickly in which case we return the last values
    */
    ret = !chip_data->valid;

    if (time_after(jiffies, chip_data->last_updated + HZ / 10) || !chip_data->valid) 
    {
        ret = shtc1_update_values(client, chip_data, buf, sizeof(buf));

        if (ret)
        goto out;

        /*
        * From datasheet:
        *   T = -45 + 175 * ST / 2^16
        *   RH = -10 + 120 * SRH / 2^16
        *
        * Adapted for integer fixed point (3 digit) arithmetic
        */
        val = be16_to_cpup((__be16 *)buf);
        chip_data->temperature_data = ((21875 * val) >> 13) - 45000;
        val = be16_to_cpup((__be16 *)(buf+3));
        chip_data->humidity_data = ((15000 * val) >> 13) - 10000;

        chip_data->last_updated = jiffies;
        chip_data->valid = 1;
    }

    out:
    //mutex_unlock(&chip_data->lock);

    return ret == 0 ? chip_data : ERR_PTR(ret);
}
 

static void shtc1_updata_poll_time(struct shtc1_data *chip_data)
{
    unsigned mini_time = 0;
    
    if (!chip_data->humidity_state && chip_data->temperature_state)
    {
        mini_time = chip_data->temperature_poll_time;
    }
    else
    {
        if (chip_data->humidity_state && !chip_data->temperature_state)
        {
            mini_time = chip_data->humidity_poll_time;
        }
        else
        {
            mini_time = chip_data->humidity_poll_time < chip_data->temperature_poll_time ? 
                        chip_data->humidity_poll_time : chip_data->temperature_poll_time;
        }
    }

    chip_data->poll_time = mini_time > chip_data->poll_time_minimum ?
                           mini_time : chip_data->poll_time_minimum;
}


static ssize_t shtc1_poll_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct shtc1_data *chip_data = dev_get_drvdata(dev);
    SENSOR_LOG_INFO("poll_time = %d\n",chip_data->poll_time);
    return sprintf(buf, "%d\n", chip_data->poll_time);
}

static ssize_t shtc1_poll_time_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct shtc1_data *chip_data = dev_get_drvdata(dev);
	unsigned long time;
	int rc;

	rc = kstrtoul(buf, 10, &time);
	if (rc)
		return -EINVAL;

    mutex_lock(&chip_data->lock);

    chip_data->poll_time = time;

    mutex_unlock(&chip_data->lock);

    return size;
}




static void shtc1_temperature_enable(struct shtc1_data *chip_data, bool on)
{
    SENSOR_LOG_INFO("temperature %s\n",on? "enable" : "disable");
    chip_data->temperature_state = on;
    if (on)
    {
       if (false==chip_data->humidity_state)
       {            
           schedule_delayed_work(&chip_data->poll_work, msecs_to_jiffies(100));
       }
    }
    else
    {
       msleep(50);
       shtc1_input_temperature(chip_data);

       chip_data->temperature_poll_time = 1000;
       shtc1_updata_poll_time(chip_data);
    }
}


static ssize_t shtc1_temperature_poll_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct shtc1_data *chip_data = dev_get_drvdata(dev);
    SENSOR_LOG_INFO("temperature_poll_time = %d\n",chip_data->temperature_poll_time);
    return sprintf(buf, "%d\n", chip_data->temperature_poll_time);
}

static ssize_t shtc1_temperature_poll_time_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct shtc1_data *chip_data = dev_get_drvdata(dev);
	unsigned long time;
	int rc;

	rc = kstrtoul(buf, 10, &time);
	if (rc)
		return -EINVAL;

    mutex_lock(&chip_data->lock);
    chip_data->temperature_poll_time = time;
    shtc1_updata_poll_time(chip_data);
    mutex_unlock(&chip_data->lock);

    return size;
}

/*
static int shtc1_temperature_data_convert(int original_data)
{
    int temperature_data = 175 * original_data;
    temperature_data = temperature_data / 65536;
    temperature_data -= 45;
    SENSOR_LOG_INFO("temperature = %d\n",temperature_data);
    return temperature_data;
}
*/

static ssize_t shtc1_temperature_data_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct shtc1_data *chip_data = dev_get_drvdata(dev);
    shtc1_update_client(&chip_data->client->dev);
    shtc1_input_temperature(chip_data);
    SENSOR_LOG_INFO("temperature original = %d\n",chip_data->temperature_data % 1000000);
    return sprintf(buf, "%d\n", chip_data->temperature_data % 1000000);
}
 

static ssize_t shtc1_temperature_enable_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct shtc1_data *chip_data = dev_get_drvdata(dev);

    SENSOR_LOG_INFO("temperature_state = %d\n",chip_data->temperature_state);
    return sprintf(buf, "%d\n", chip_data->temperature_state);
}
 

static ssize_t shtc1_temperature_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct shtc1_data *chip_data = dev_get_drvdata(dev);
    bool value;

    if (strtobool(buf, &value))
        return -EINVAL;
    mutex_lock(&chip_data->lock);

    if (value)
    {
        shtc1_temperature_enable(chip_data, 1);
    }
    else
    {
        shtc1_temperature_enable(chip_data, 0);
    }

    mutex_unlock(&chip_data->lock);

    return size;
}

static ssize_t shtc1_temperature_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct shtc1_data *chip_data = dev_get_drvdata(dev);
    bool value;

    if (strtobool(buf, &value))
        return -EINVAL;

    mutex_lock(&chip_data->lock);
    if (value)
    {
        chip_data->temperature_debug = true;
    }
    else
    {
        chip_data->temperature_debug = false;
    }
    mutex_unlock(&chip_data->lock);

    return size;
}

static ssize_t shtc1_temperature_debug_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct shtc1_data *chip_data = dev_get_drvdata(dev);
    SENSOR_LOG_INFO("temperature_debug = %s\n",chip_data->temperature_debug? "true":"false");
    return sprintf(buf, "%s\n", chip_data->temperature_debug? "true":"false");
}


static void shtc1_humidity_enable(struct shtc1_data *chip_data, bool on)
{
    SENSOR_LOG_INFO("humidity %s\n",on? "enable" : "disable");
    chip_data->humidity_state = on;
    if (on)
    {
        if (false==chip_data->temperature_state)
        {            
            schedule_delayed_work(&chip_data->poll_work, msecs_to_jiffies(100));
        }
    }
    else
    {
        msleep(50);
        shtc1_input_humidity(chip_data);

        chip_data->humidity_poll_time = 1000;
        shtc1_updata_poll_time(chip_data);
    }
}

static ssize_t shtc1_humidity_poll_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct shtc1_data *chip_data = dev_get_drvdata(dev);
    SENSOR_LOG_INFO("humidity_poll_time = %d\n",chip_data->humidity_poll_time);
    return sprintf(buf, "%d\n", chip_data->humidity_poll_time);
}

static ssize_t shtc1_humidity_poll_time_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct shtc1_data *chip_data = dev_get_drvdata(dev);
	unsigned long time;
	int rc;

	rc = kstrtoul(buf, 10, &time);
	if (rc)
		return -EINVAL;

    mutex_lock(&chip_data->lock);

    chip_data->humidity_poll_time = time;
    shtc1_updata_poll_time(chip_data);

    mutex_unlock(&chip_data->lock);

    return size;
}


static ssize_t shtc1_humidity_data_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    struct shtc1_data *chip_data = dev_get_drvdata(dev);
    shtc1_update_client(&chip_data->client->dev);
    shtc1_input_humidity(chip_data);
    SENSOR_LOG_INFO("humidity original data = %d\n",chip_data->humidity_data % 1000000);
    return sprintf(buf, "%d\n", chip_data->humidity_data % 1000000);
}


static ssize_t shtc1_humidity_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct shtc1_data *chip_data = dev_get_drvdata(dev);
    bool value;

    if (strtobool(buf, &value))
        return -EINVAL;

    mutex_lock(&chip_data->lock);
    if (value)
    {
        shtc1_humidity_enable(chip_data, 1);
    }
    else
    {
        shtc1_humidity_enable(chip_data, 0);
    }
    mutex_unlock(&chip_data->lock);

    return size;
}


static ssize_t shtc1_humidity_enable_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct shtc1_data *chip_data = dev_get_drvdata(dev);
    SENSOR_LOG_INFO("humidity_state = %d\n",chip_data->humidity_state);
    return sprintf(buf, "%d\n", chip_data->humidity_state);
}


static ssize_t shtc1_humidity_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct shtc1_data *chip_data = dev_get_drvdata(dev);
    bool value;

    if (strtobool(buf, &value))
        return -EINVAL;

    mutex_lock(&chip_data->lock);
    if (value)
    {
        chip_data->humidity_debug = true;
    }
    else
    {
        chip_data->humidity_debug = false;
    }
    mutex_unlock(&chip_data->lock);

    return size;
}

static ssize_t shtc1_humidity_debug_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct shtc1_data *chip_data = dev_get_drvdata(dev);
    SENSOR_LOG_INFO("humidity_debug = %s\n",chip_data->humidity_debug? "true":"false");
    return sprintf(buf, "%s\n", chip_data->humidity_debug? "true":"false");
}


static struct device_attribute attrs_temperature[] = {
	__ATTR(enable,                  0640,   shtc1_temperature_enable_show,          shtc1_temperature_enable_store),
	__ATTR(temperature_data,        0640,   shtc1_temperature_data_show,            NULL),
    __ATTR(temperature_poll_time,   0640,   shtc1_temperature_poll_time_show,       shtc1_temperature_poll_time_store),
    __ATTR(poll_time,               0640,   shtc1_poll_time_show,                   shtc1_poll_time_store),
    __ATTR(temperature_debug,       0640,   shtc1_temperature_debug_show,           shtc1_temperature_debug_store),
};


static struct device_attribute attrs_humidity[] = {
	__ATTR(enable,                  0640,   shtc1_humidity_enable_show,             shtc1_humidity_enable_store),
	__ATTR(humidity_data,           0640,   shtc1_humidity_data_show,               NULL),
    __ATTR(humidity_poll_time,      0640,   shtc1_humidity_poll_time_show,          shtc1_humidity_poll_time_store),
    __ATTR(poll_time,               0640,   shtc1_poll_time_show,                   shtc1_poll_time_store),
    __ATTR(humidity_debug,          0640,   shtc1_humidity_debug_show,              shtc1_humidity_debug_store),

};


static int shtc1_create_sysfs_interfaces_temperature(struct device *dev)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(attrs_temperature); i++)
     if (device_create_file(dev, attrs_temperature + i))
         goto error;
    return 0;

error:
    for ( ; i >= 0; i--)
    device_remove_file(dev, attrs_temperature + i);
    SENSOR_LOG_ERROR("Unable to create interface\n");
    return -1;
}

static int shtc1_create_sysfs_interfaces_humidity(struct device *dev)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(attrs_humidity); i++)
    if (device_create_file(dev, attrs_humidity + i))
        goto error;
    return 0;

    error:
    for ( ; i >= 0; i--)
        device_remove_file(dev, attrs_humidity + i);
    SENSOR_LOG_ERROR("Unable to create interface\n");
    return -1;
}

static void shtc1_input_temperature(struct shtc1_data *chip_data)
{
    if (chip_data->temperature_debug)
    {
        SENSOR_LOG_INFO("temperature = %d\n",chip_data->temperature_data);
    }

    input_report_rel(chip_data->t_idev, REL_X, chip_data->temperature_data);
    input_sync(chip_data->t_idev);
}


static void shtc1_input_humidity(struct shtc1_data *chip_data)
{
    if (chip_data->humidity_debug)
    {
        SENSOR_LOG_INFO("humidity = %d\n",chip_data->humidity_data);
    }

    input_report_rel(chip_data->h_idev, REL_X, chip_data->humidity_data);
    input_sync(chip_data->h_idev);
}

static void shtc1_poll_work_func(struct work_struct *work)
{
    struct shtc1_data *chip_data = container_of((struct delayed_work *)work, struct shtc1_data, poll_work);
    
    mutex_lock(&chip_data->lock);

    shtc1_update_client(&chip_data->client->dev);

    if (chip_data->temperature_state)
    {
        shtc1_input_temperature(chip_data);
    }

    if (chip_data->humidity_state)
    {
        shtc1_input_humidity(chip_data);
    }

    if (chip_data->humidity_state || chip_data->temperature_state)
    {
        schedule_delayed_work(&chip_data->poll_work, msecs_to_jiffies(chip_data->poll_time));
    }

    mutex_unlock(&chip_data->lock);

}


static int __devinit shtc1_probe(struct i2c_client *client,
                  const struct i2c_device_id *id)
{

    struct shtc1_data *chip_data;
	struct device *dev = &client->dev;

    int ret;

	chip_data = kzalloc(sizeof(struct shtc1_data), GFP_KERNEL);

    if (!chip_data)
    {
        return -ENOMEM;
    }
    else
    {
        shtc1_data_init(chip_data);
    }

    SENSOR_LOG_INFO("prob start\n");

    dev->platform_data = &chip_data->pdata;


    /* defaults: blocking, high precision mode */
    chip_data->pdata.blocking_io    = 1;
    chip_data->pdata.high_precision = 1;


    shtc1_select_command(chip_data);

    chip_data->client = client;
    i2c_set_clientdata(client, chip_data);

    mutex_init(&chip_data->lock);

    temperature_class = class_create(THIS_MODULE, "temperature");
    humidity_class    = class_create(THIS_MODULE, "humidity");


    chip_data->temperature_dev = device_create(temperature_class, NULL, shtc1_temperature_dev_t, &shtc1_driver ,"temperature");
    if (IS_ERR(chip_data->temperature_dev)) 
    {
        ret = PTR_ERR(chip_data->temperature_dev);
        goto create_temperature_dev_failed;
    }

    chip_data->humidity_dev = device_create(humidity_class, NULL, shtc1_humidity_dev_t, &shtc1_driver ,"humidity");
    if (IS_ERR(chip_data->humidity_dev)) 
    {
        ret = PTR_ERR(chip_data->humidity_dev);
        goto create_humidity_dev_failed;
    }

    dev_set_drvdata(chip_data->temperature_dev, chip_data);
    dev_set_drvdata(chip_data->humidity_dev,   chip_data);

    shtc1_create_sysfs_interfaces_temperature(chip_data->temperature_dev);
    shtc1_create_sysfs_interfaces_humidity(chip_data->humidity_dev);

    chip_data->t_idev = input_allocate_device();
    if (!chip_data->t_idev) 
    {
        SENSOR_LOG_ERROR("input_allocate_device temperature failed\n");
        ret = -ENODEV;
        goto input_t_alloc_failed;
    }

    chip_data->t_idev->name = chip_data->input_dev_t_name;
    chip_data->t_idev->id.bustype = BUS_I2C; 
    set_bit(EV_REL, chip_data->t_idev->evbit);
    set_bit(REL_X,  chip_data->t_idev->relbit);
    dev_set_drvdata(&chip_data->t_idev->dev, chip_data);



    ret = input_register_device(chip_data->t_idev);
    if (ret) 
    {
        SENSOR_LOG_ERROR("cant register input '%s'\n",chip_data->input_dev_t_name);
        goto input_t_register_failed;
    }



    chip_data->h_idev = input_allocate_device();
    if (!chip_data->h_idev) 
    {
        SENSOR_LOG_ERROR("input_allocate_device humidity failed\n");
        ret = -ENODEV;
        goto input_h_alloc_failed;
    }

    chip_data->h_idev->name = chip_data->input_dev_h_name;
    chip_data->h_idev->id.bustype = BUS_I2C;
    set_bit(EV_REL, chip_data->h_idev->evbit);
    set_bit(REL_X,  chip_data->h_idev->relbit);
    dev_set_drvdata(&chip_data->h_idev->dev, chip_data);
    ret = input_register_device(chip_data->h_idev);
    if (ret) 
    {
        SENSOR_LOG_ERROR("cant register input '%s'\n",chip_data->input_dev_h_name);
        goto input_h_register_failed;
    }

    INIT_DELAYED_WORK(&chip_data->poll_work, shtc1_poll_work_func);

    SENSOR_LOG_INFO("prob success\n");

    return 0;

input_h_register_failed:
    input_free_device(chip_data->h_idev);
input_h_alloc_failed:

input_t_register_failed:
    input_free_device(chip_data->t_idev);
input_t_alloc_failed:

create_humidity_dev_failed:
    chip_data->humidity_dev = NULL;
    class_destroy(humidity_class); 

create_temperature_dev_failed:
    chip_data->temperature_dev = NULL;
    class_destroy(temperature_class); 

    return ret;
}

 
 /**
  * shtc1_remove() - remove device
  * @client: I2C client device
  */
 static int __devexit shtc1_remove(struct i2c_client *client)
 {
     struct shtc1_data *chip_data = i2c_get_clientdata(client);
 
      SENSOR_LOG_INFO("shtc1_remove\n");
     //hwmon_device_unregister(chip_data->hwmon_dev);
     //sysfs_remove_group(&client->dev.kobj, &shtc1_attr_group);
    
     kfree(chip_data);
     return 0;
 }

static int __init shtc1_init(void)
{
        SENSOR_LOG_INFO("driver: init\n");
        return i2c_add_driver(&shtc1_driver);
}
 
static void __exit shtc1_exit(void)
{
        SENSOR_LOG_INFO("driver: exit\n");
        i2c_del_driver(&shtc1_driver);
}

module_init(shtc1_init);
module_exit(shtc1_exit);
 
MODULE_DESCRIPTION("shtc1 humidity driver");
MODULE_AUTHOR("ZhuBing, ZTEMT");
MODULE_LICENSE("GPL");
