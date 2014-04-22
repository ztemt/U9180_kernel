/*
 * cyttsp4_i2c.c
 * Cypress TrueTouch(TM) Standard Product V4 I2C Driver module.
 * For use with Cypress Txx4xx parts.
 * Supported parts include:
 * TMA4XX
 * TMA1036
 *
 * Copyright (C) 2012 Cypress Semiconductor
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 * Modified by: Cypress Semiconductor for test with device
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include "cyttsp4_bus.h"
#include "cyttsp4_core.h"
#include "cyttsp4_i2c.h"

#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
/*** ZTEMT Added by luochangyang, 2013/03/27 ***/
#ifdef CONFIG_OF
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#endif

#define CY_I2C_DATA_SIZE  (3 * 256)

struct cyttsp4_i2c {
	struct i2c_client *client;
	u8 wr_buf[CY_I2C_DATA_SIZE];
	struct hrtimer timer;
	struct mutex lock;
	atomic_t timeout;
};

static int cyttsp4_i2c_read_block_data(struct cyttsp4_i2c *ts_i2c, u8 addr,
	size_t length, void *values)
{
	int rc;

	/* write addr */
	rc = i2c_master_send(ts_i2c->client, &addr, sizeof(addr));
	if (rc < 0)
		return rc;
	else if (rc != sizeof(addr))
		return -EIO;

	/* read data */
	rc = i2c_master_recv(ts_i2c->client, values, length);

	return (rc < 0) ? rc : rc != length ? -EIO : 0;
}

static int cyttsp4_i2c_write_block_data(struct cyttsp4_i2c *ts_i2c, u8 addr,
	size_t length, const void *values)
{
	int rc;

	if (sizeof(ts_i2c->wr_buf) < (length + 1))
		return -ENOMEM;

	ts_i2c->wr_buf[0] = addr;
	memcpy(&ts_i2c->wr_buf[1], values, length);
	length += 1;

	/* write data */
	rc = i2c_master_send(ts_i2c->client, ts_i2c->wr_buf, length);

	return (rc < 0) ? rc : rc != length ? -EIO : 0;
}

static int cyttsp4_i2c_write(struct cyttsp4_adapter *adap, u8 addr,
	const void *buf, int size)
{
	struct cyttsp4_i2c *ts = dev_get_drvdata(adap->dev);
	int rc;

	pm_runtime_get_noresume(adap->dev);
	mutex_lock(&ts->lock);
	rc = cyttsp4_i2c_write_block_data(ts, addr, size, buf);
	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(adap->dev);

	return rc;
}

static int cyttsp4_i2c_read(struct cyttsp4_adapter *adap, u8 addr,
	void *buf, int size)
{
	struct cyttsp4_i2c *ts = dev_get_drvdata(adap->dev);
	int rc;

	pm_runtime_get_noresume(adap->dev);
	mutex_lock(&ts->lock);
	rc = cyttsp4_i2c_read_block_data(ts, addr, size, buf);
	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(adap->dev);

	return rc;
}

/*** ZTEMT Added by luochangyang, 2013/03/27 ***/
#ifdef CONFIG_OF
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int cyttsp4_power_on(struct device *dev)
{
	int rc;
    static struct regulator *vcc_ana;
    static struct regulator *vcc_i2c;

	vcc_ana = regulator_get(dev, "vdd_ana");
	if (IS_ERR(vcc_ana))
    {
		rc = PTR_ERR(vcc_ana);
		dev_err(dev, "Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(vcc_ana) > 0)
    {
		rc = regulator_set_voltage(vcc_ana, 2850000, 2850000);
		if (rc)
        {
			dev_err(dev, "Regulator set ana vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}
    
    rc = reg_set_optimum_mode_check(vcc_ana, 15000);
    if (rc < 0)
    {
        dev_err(dev, "Regulator vcc_ana set_opt failed rc=%d\n", rc);
        return rc;
    }
    
    rc = regulator_enable(vcc_ana);
    if (rc)
    {
        dev_err(dev, "Regulator vcc_ana enable failed rc=%d\n", rc);
        goto error_reg_en_vcc_ana;
    }
    
	vcc_i2c = regulator_get(dev, "vcc_i2c");
	if (IS_ERR(vcc_i2c))
    {
		rc = PTR_ERR(vcc_i2c);
		dev_err(dev, "Regulator get failed rc=%d\n", rc);
		goto error_reg_opt_vcc_dig;
	}

    rc = regulator_enable(vcc_i2c);
    if (rc)
    {
        dev_err(dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
        goto error_reg_en_vcc_i2c;
    }

    msleep(100);
    
    return 0;

error_reg_en_vcc_i2c:
    reg_set_optimum_mode_check(vcc_i2c, 0);
error_reg_opt_vcc_dig:
    regulator_disable(vcc_ana);
error_reg_en_vcc_ana:
    reg_set_optimum_mode_check(vcc_ana, 0);
error_set_vtg_vcc_ana:
	regulator_put(vcc_ana);
	return rc;
}

static int cyttsp4_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;    
//    struct cyttsp4_core_platform_data *platform_data = dev->platform_data;
    int rst_gpio = 0;
    int irq_gpio = 0;

	/* reset, irq gpio info */
	rst_gpio = of_get_named_gpio(np, "cypress,reset-gpio", 0);
	irq_gpio = of_get_named_gpio(np, "cypress,irq-gpio", 0);
    
    return 0;
}
#endif

static struct cyttsp4_ops ops = {
	.write = cyttsp4_i2c_write,
	.read = cyttsp4_i2c_read,
};

static int __devinit cyttsp4_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *i2c_id)
{
	struct cyttsp4_i2c *ts_i2c;
	struct device *dev = &client->dev;
	char const *adap_id = dev_get_platdata(dev);
	char const *id;
	int rc;

	dev_dbg(dev, "%s: Starting %s probe...\n", __func__, CYTTSP4_I2C_NAME);

	dev_dbg(dev, "%s: debug on\n", __func__);
	dev_vdbg(dev, "%s: verbose debug on\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "%s: fail check I2C functionality\n", __func__);
		rc = -EIO;
		goto error_alloc_data_failed;
	}

	ts_i2c = kzalloc(sizeof(struct cyttsp4_i2c), GFP_KERNEL);
	if (ts_i2c == NULL) {
		dev_err(dev, "%s: Error, kzalloc.\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}
/*** ZTEMT Added by luochangyang, 2013/03/27 ***/
#ifdef CONFIG_OF
	if (client->dev.of_node) {
		rc = cyttsp4_parse_dt(&client->dev);
		if (rc)
			return rc;
	} else
		dev_err(dev, "%s: no client->dev.of_node.\n", __func__);
    
    cyttsp4_power_on(&client->dev);
#endif

	mutex_init(&ts_i2c->lock);
	ts_i2c->client = client;
	client->dev.bus = &i2c_bus_type;
	i2c_set_clientdata(client, ts_i2c);
	dev_set_drvdata(&client->dev, ts_i2c);

	if (adap_id)
		id = adap_id;
	else
		id = CYTTSP4_I2C_NAME;

	dev_dbg(dev, "%s: add adap='%s' (CYTTSP4_I2C_NAME=%s)\n", __func__, id,
		CYTTSP4_I2C_NAME);

	pm_runtime_enable(&client->dev);

	rc = cyttsp4_add_adapter(id, &ops, dev);
	if (rc) {
		dev_err(dev, "%s: Error on probe %s\n", __func__,
			CYTTSP4_I2C_NAME);
		goto add_adapter_err;
	}

	dev_dbg(dev, "%s: Successful probe %s\n", __func__, CYTTSP4_I2C_NAME);

	return 0;

add_adapter_err:
	pm_runtime_disable(&client->dev);
	dev_set_drvdata(&client->dev, NULL);
	i2c_set_clientdata(client, NULL);
	kfree(ts_i2c);
error_alloc_data_failed:
	return rc;
}

/* registered in driver struct */
static int __devexit cyttsp4_i2c_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cyttsp4_i2c *ts_i2c = dev_get_drvdata(dev);
	char const *adap_id = dev_get_platdata(dev);
	char const *id;

	if (adap_id)
		id = adap_id;
	else
		id = CYTTSP4_I2C_NAME;

	dev_info(dev, "%s\n", __func__);
	cyttsp4_del_adapter(id);
	pm_runtime_disable(&client->dev);
	dev_set_drvdata(&client->dev, NULL);
	i2c_set_clientdata(client, NULL);
	kfree(ts_i2c);
	return 0;
}

static const struct i2c_device_id cyttsp4_i2c_id[] = {
	{ CYTTSP4_I2C_NAME, 0 },
    { }
};

/*** ZTEMT modify by luochangyang, 2013/03/27 ***/
#ifdef CONFIG_OF //Open firmware must be defined for dts useage
static struct of_device_id cyttsp4_match_table[] = {
	{ .compatible = "cypress,cyttsp4_i2c_adapter",}, //Compatible node must match dts
    { },
};
#else
#define cyttsp4_match_table NULL
#endif

static struct i2c_driver cyttsp4_i2c_driver = {
	.driver = {
		.name = CYTTSP4_I2C_NAME,
		.owner = THIS_MODULE,
        .of_match_table = cyttsp4_match_table,
	},
	.probe = cyttsp4_i2c_probe,
	.remove = __devexit_p(cyttsp4_i2c_remove),
	.id_table = cyttsp4_i2c_id,
};

module_i2c_driver(cyttsp4_i2c_driver);

MODULE_ALIAS(CYTTSP4_I2C_NAME);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) I2C driver");
MODULE_AUTHOR("Cypress");
MODULE_DEVICE_TABLE(i2c, cyttsp4_i2c_id);
