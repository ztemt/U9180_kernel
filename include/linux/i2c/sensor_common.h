
#ifndef __SENSOR_COMMON_H_
#define __SENSOR_COMMON_H_


struct sensor_common_data {
    struct i2c_client *client;
    struct device *sensor_compass_dev;
};

#define COMPASS_INT_PIN     64


static int __devinit sensor_common_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit sensor_common_remove(struct i2c_client *client);
#endif
