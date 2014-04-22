/*
 * Device driver for monitoring ambient light intensity in (lux)
 * proximity detection (prox), and Gesture functionality within the
 * AMS-TAOS TMG3992/3.
 *
 * Copyright (c) 2013, AMS-TAOS USA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/i2c/tmg399x.h>
#include <mach/irqs.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>

#define LOG_TAG "SENSOR_ALS_PROX_GES"
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

#define CAL_THRESHOLD   "/persist/proxdata/threshold"


static u8 const tmg399x_ids[] = {
	0x9C,
	0x9E,
	0x9F,
    0XAA,
};

static char const *tmg399x_names[] = {
	"tmg399x",
	"tmg399x",
	"tmg399x",
    "tmg399x",
};

static u8 const restorable_regs[] = {
	TMG399X_ALS_TIME,
	TMG399X_WAIT_TIME,
	TMG399X_PERSISTENCE,
	TMG399X_CONFIG_1,
	TMG399X_PRX_PULSE,
	TMG399X_GAIN,
	TMG399X_CONFIG_2,
	TMG399X_PRX_OFFSET_NE,
	TMG399X_PRX_OFFSET_SW,
	TMG399X_CONFIG_3,
};

static u8 const prox_gains[] = {
	1,
	2,
	4,
	8
};

static u8 const als_gains[] = {
	1,
	4,
	16,
	64
};

static u8 const prox_pplens[] = {
	4,
	8,
	16,
	32
};

static u8 const led_drives[] = {
	100,
	50,
	25,
	12
};

static u16 const led_boosts[] = {
	100,
	150,
	200,
	300
};

static struct lux_segment segment_default[] = {
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
};

static struct tmg399x_parameters param_default = {
#if 0
	.als_time = 0xEE, /* 50ms */
	.als_gain = AGAIN_16,
	.als_deltaP = 10,
	.wait_time = 0xFF, /* 2.78ms */
	.prox_th_low = 50,
	.prox_th_high = 80,
	.persist = PRX_PERSIST(1) | ALS_PERSIST(2),
	.als_prox_cfg1 = 0x60,
	.prox_pulse = PPLEN_4US | PRX_PULSE_CNT(8),
	.prox_gain = PGAIN_1,
	.ldrive = PDRIVE_100MA,
	.als_prox_cfg2 = LEDBOOST_300 | 0x01,
	.prox_offset_ne = 0,
	.prox_offset_sw = 0,
	.als_prox_cfg3 = 0,

	.ges_entry_th = 0,
	.ges_exit_th = 0,
	.ges_cfg1 = FIFOTH_1 | GEXPERS_2,
	.ges_cfg2 = GGAIN_4 | GLDRIVE_100 | GWTIME_6,
	.ges_offset_n = 0,
	.ges_offset_s = 0,
	.ges_pulse = GPLEN_16US | GES_PULSE_CNT(2),
	.ges_offset_w = 0,
	.ges_offset_e = 0,
	.ges_dimension = GBOTH_PAIR,
#endif
};

static int __devinit tmg399x_probe(struct i2c_client *client, const struct i2c_device_id *idp);
static int __devexit tmg399x_remove(struct i2c_client *client);
static int tmg399x_suspend(struct device *dev);
static int tmg399x_resume(struct device *dev);
static int tmg399x_prox_calibrate(struct tmg399x_chip *chip);
int tmg399x_read_cal_value(char *file_path);
int tmg399x_write_cal_file(char *file_path,unsigned int value);
static int tmg399x_get_id(struct tmg399x_chip *chip, u8 *id, u8 *rev);


static const struct i2c_device_id tmg399x_idtable_id[] = {
	{ "ams,ams-sensor", 0 },
	{ },
};

static struct of_device_id of_tmg399x_idtable[] = {
	{ .compatible = "ams,ams-sensor",},
	{}
};

MODULE_DEVICE_TABLE(i2c, tmg399x_idtable);

static const struct dev_pm_ops tmg399x_pm_ops = {
	.suspend = tmg399x_suspend,
	.resume  = tmg399x_resume,
};


struct i2c_driver tmg399x_driver = {
	.driver = {
		.name = "ams-sensor",
        .of_match_table = of_tmg399x_idtable,
		.pm = &tmg399x_pm_ops,
	},
	.id_table = tmg399x_idtable_id,
	.probe = tmg399x_probe,
	.remove = __devexit_p(tmg399x_remove),
};



/**********************************  MARCO START ****************************************/

#define	AMS_ALS_POLL_DELAY_FAST	         500
#define	AMS_ALS_POLL_DELAY_SLOW	         1000
#define AMS_ALS_ATIME_LONG               0xEE
#define AMS_ALS_ATIME_SHORT              0xF8

#define PROX_FAR                     0
#define PROX_NEAR                    1
#define PROX_UNKNOW                 -1
/**********************************  MARCO  END  ****************************************/

/**********************************  VARIABLE START ****************************************/

struct tmg399x_chip *p_global_tmg399x_chip;

static struct class         *proximity_class;
static struct class         *light_class;
static struct class         *gesture_class;


static dev_t const tmg399x_proximity_dev_t = MKDEV(MISC_MAJOR, 245);
static dev_t const tmg399x_light_dev_t     = MKDEV(MISC_MAJOR, 246);
static dev_t const tmg399x_gesture_dev_t   = MKDEV(MISC_MAJOR, 247);


wait_queue_head_t   gesture_drdy_wq;
atomic_t            gesture_drdy;


static unsigned char reg_addr = 0;

/**********************************  VARIABLE END ****************************************/
static int tmg399x_ges_enable(struct tmg399x_chip *chip, int on);
static int tmg399x_get_prox(struct tmg399x_chip *chip);
static void tmg399x_report_prox(struct tmg399x_chip *chip);



static int board_tmg399x_init(void)
{
    SENSOR_LOG_INFO("\n");
	return 0;
}

static int board_tmg399x_power(struct device *dev, enum tmg399x_pwr_state state)
{
    SENSOR_LOG_INFO("\n");
	return 0;
}

static void board_tmg399x_teardown(struct device *dev)
{
    SENSOR_LOG_INFO("\n");
}

static const struct lux_segment tmg399x_segment[] = {
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
};

struct tmg399x_i2c_platform_data tmg399x_data = {
	.platform_power = board_tmg399x_power,
	.platform_init = board_tmg399x_init,
	.platform_teardown = board_tmg399x_teardown,
	.prox_name = "proximity",
	.als_name = "light",
    .ges_name = "gesture",
	.parameters = {
		.als_time = 0xEE, /* 50ms */
		.als_gain = AGAIN_16,
		.als_deltaP = 10,
		.wait_time = 0xF6, //10* 2.78ms 
		.prox_th_low = 120,
		.prox_th_high = 150,
		.persist = PRX_PERSIST(1) | ALS_PERSIST(2),
		.als_prox_cfg1 = 0x60,
		.prox_pulse = PPLEN_16US | PRX_PULSE_CNT(1),
		.prox_gain = PGAIN_4,
		.ldrive = PDRIVE_100MA,
        .als_prox_cfg2 = LEDBOOST_300 | 0x01,
		.prox_offset_ne = 0,
		.prox_offset_sw = 0,
		.als_prox_cfg3 = 0,
		
		.ges_entry_th = 0,
		.ges_exit_th = 0,
		.ges_cfg1 = FIFOTH_4| GEXPERS_1,
		.ges_cfg2 = GGAIN_4 | GLDRIVE_100 | GWTIME_3,
		.ges_offset_n = 30,
		.ges_offset_s = 0,
		.ges_pulse = GPLEN_32US | GES_PULSE_CNT(2),
		.ges_offset_w = 0,
		.ges_offset_e = 0,
		.ges_dimension = GBOTH_PAIR,
	},
	.als_can_wake = false,
	.proximity_can_wake = true,
	.segment = (struct lux_segment *) tmg399x_segment,
	.segment_num = ARRAY_SIZE(tmg399x_segment),

};

static int tmg399x_i2c_read(struct tmg399x_chip *chip, u8 reg, u8 *val)
{
	int ret;

	s32 read;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret = i2c_smbus_write_byte(client, reg);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte(client, reg);
		if (ret < 0) {
			dev_err(&client->dev, "%s: failed 2x to write register %x\n",
				__func__, reg);
		return ret;
		}
	}
	read = i2c_smbus_read_byte(client);
	if (read < 0) {
		mdelay(3);
		read = i2c_smbus_read_byte(client);
		if (read < 0) {
			dev_err(&client->dev, "%s: failed read from register %x\n",
				__func__, reg);
		}
		return ret;
	}

	*val = (u8)read;
	return 0;
}

static int tmg399x_i2c_write(struct tmg399x_chip *chip, u8 reg, u8 val)
{
	int ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0) {
			dev_err(&client->dev, "%s: failed to write register %x err= %d\n",
				__func__, reg, ret);
		}
	}

	return ret;
}

static int tmg399x_i2c_modify(struct tmg399x_chip *chip, u8 reg, u8 mask, u8 val)
{
	int ret;
	u8 temp;
    
	ret = tmg399x_i2c_read(chip, reg, &temp);
	temp &= ~mask;
	temp |= val;
	ret |= tmg399x_i2c_write(chip, reg, temp);

	return ret;
}

static int tmg399x_i2c_reg_blk_write(struct tmg399x_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret =  i2c_smbus_write_i2c_block_data(client,
			reg, size, val);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed 2X at address %x (%d bytes)\n",
				__func__, reg, size);
	}

	return ret;
}

static int tmg399x_i2c_ram_blk_read(struct tmg399x_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret =  i2c_smbus_read_i2c_block_data(client,
			reg, size, val);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed 2X at address %x (%d bytes)\n",
				__func__, reg, size);
	}

	return ret;
}

static int tmg399x_flush_regs(struct tmg399x_chip *chip)
{
	unsigned i;
	int ret;
	u8 reg;

	dev_info(&chip->client->dev, "%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		ret = tmg399x_i2c_write(chip, reg, chip->shadow[reg]);
		if (ret < 0) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}

	return ret;
}

static void tmg399x_wakelock_ops(struct tmg399x_wake_lock *wakelock, bool enable)
{
    if (enable == wakelock->locked)
    {
        SENSOR_LOG_INFO("doubule %s %s, retern here\n",enable? "lock" : "unlock",wakelock->name);
        return;
    }

    if (enable)
    {
        wake_lock(&wakelock->lock);
    }
    else
    {
        wake_unlock(&wakelock->lock);
    }

    wakelock->locked = enable;

    SENSOR_LOG_INFO("%s %s \n",enable? "lock" : "unlock",wakelock->name);
}


static void tmg399x_chip_data_init(struct tmg399x_chip *chip)
{
    chip->light_debug_enable      = false;
    chip->prox_debug_enable       = false;
    chip->ges_debug_enable        = false;
    chip->prx_enabled             = false;
    chip->als_enabled             = false;
    chip->ges_enabled             = false;
    chip->prox_calibrate_result   = false;
    chip->prox_data_max           = 255;
    chip->prox_manual_calibrate_threshold = 0;
    chip->prox_thres_hi_max       = 200;
    chip->prox_thres_lo_min       = 100;
    chip->prox_calibrate_times    = 0;
    chip->chip_name               = "tmg3993";
    chip->wakeup_from_sleep       = false;
    chip->wakelock_locked         = false;
    chip->irq_enabled             = true;
    chip->gesture_start           = false;
    chip->prox_calibrate_start    = false;
    chip->phone_is_sleep          = false;
    chip->irq_work_status         = false;
    chip->proximity_wakelock.name = "proximity-wakelock";
    chip->proximity_wakelock.locked = false;
    chip->light_poll_time         = AMS_ALS_POLL_DELAY_SLOW;
}

static void tmg399x_irq_enable(bool enable, bool flag_sync)
{
    if (enable == p_global_tmg399x_chip->irq_enabled)
    {
        SENSOR_LOG_INFO("doubule %s irq, retern here\n",enable? "enable" : "disable");
        return;
    }

    if (enable)
    {
        enable_irq(p_global_tmg399x_chip->client->irq);
    }
    else
    {
        if (flag_sync)
        {
            disable_irq(p_global_tmg399x_chip->client->irq);

        }
        else
        {
            disable_irq_nosync(p_global_tmg399x_chip->client->irq);
        }
    }

    p_global_tmg399x_chip->irq_enabled  = enable;
    //SENSOR_LOG_INFO("%s irq \n",enable? "enable" : "disable");
}


static int tmg399x_irq_clr(struct tmg399x_chip *chip, u8 int2clr)
{
	int ret, ret2;
    
	ret = i2c_smbus_write_byte(chip->client, int2clr);

	if (ret < 0) {
		mdelay(3);
		ret2 = i2c_smbus_write_byte(chip->client, int2clr);
		if (ret2 < 0) {
			dev_err(&chip->client->dev, "%s: failed 2x, int to clr %02x\n",
					__func__, int2clr);
		}
		return ret2;
	}
	return ret;
}

static int tmg399x_update_enable_reg(struct tmg399x_chip *chip)
{
	int ret;
	
    SENSOR_LOG_INFO("\n");

	//mutex_lock(&chip->lock);
	ret  = tmg399x_i2c_write(chip, TMG399X_CONTROL, chip->shadow[TMG399X_CONTROL]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_4, chip->shadow[TMG399X_GES_CFG_4]);
    SENSOR_LOG_INFO("TMG399X_CONTROL   = %X\n",chip->shadow[TMG399X_CONTROL]);
    SENSOR_LOG_INFO("TMG399X_GES_CFG_4 = %X\n",chip->shadow[TMG399X_GES_CFG_4]);

	//mutex_unlock(&chip->lock);
	
    SENSOR_LOG_INFO("\n");
	return ret;
}

static int tmg399x_set_als_gain(struct tmg399x_chip *chip, int gain)
{
	int ret;
	u8 ctrl_reg  = chip->shadow[TMG399X_GAIN] & ~TMG399X_ALS_GAIN_MASK;

	switch (gain) {
	case 1:
		ctrl_reg |= AGAIN_1;
		break;
	case 4:
		ctrl_reg |= AGAIN_4;
		break;
	case 16:
		ctrl_reg |= AGAIN_16;
		break;
	case 64:
		ctrl_reg |= AGAIN_64;
		break;
	default:
		break;
	}
    
    SENSOR_LOG_INFO("TMG399X_ALS_AGL_MASK = %d\n",TMG399X_ALS_AGL_MASK);


	//mutex_lock(&chip->lock);
	ret = tmg399x_i2c_write(chip, TMG399X_GAIN, ctrl_reg);
	if (!ret) {
		chip->shadow[TMG399X_GAIN] = ctrl_reg;
		chip->params.als_gain = ctrl_reg & TMG399X_ALS_GAIN_MASK;
	}
	return ret;
}

static void tmg399x_calc_cpl(struct tmg399x_chip *chip)
{
	u32 cpl;
	u32 sat;
	u8 atime = chip->shadow[TMG399X_ALS_TIME];

	cpl = 256 - chip->shadow[TMG399X_ALS_TIME];
	cpl *= TMG399X_ATIME_PER_100;
	cpl /= 100;
	cpl *= als_gains[chip->params.als_gain];

    //SENSOR_LOG_INFO("chip->params.als_gain = %d\n",chip->params.als_gain);

	sat = min_t(u32, MAX_ALS_VALUE, (u32)(256 - atime) << 10);
	sat = sat * 8 / 10;
	chip->als_inf.cpl = cpl;
	chip->als_inf.saturation = sat;
}

static int tmg399x_get_lux(struct tmg399x_chip *chip)
{
	u32 rp1, gp1, bp1, cp1;
	u32 lux = 0;
	u32 cct;
	u32 sat;
	u32 sf;
    bool lux_is_too_low = false;


	/* use time in ms get scaling factor */
	tmg399x_calc_cpl(chip);
	sat = chip->als_inf.saturation;

	if (!chip->als_gain_auto) 
    {
		if (chip->als_inf.clear_raw <= MIN_ALS_VALUE) 
        {
            SENSOR_LOG_INFO("darkness\n");
			lux = 0;
			goto exit;
		} 
        else
        {
            if (chip->als_inf.clear_raw >= sat) 
            {
                SENSOR_LOG_INFO("saturation, keep lux & cct, sat = %d\n",sat);
			    lux = chip->als_inf.lux;
			    goto exit;
		    }
        }
	} 
    else
    {
		u8 gain = als_gains[chip->params.als_gain];
		int ret = -EIO;


        //SENSOR_LOG_INFO("gain = %d, sat = %d, clear_raw = %d\n",gain,sat,chip->als_inf.clear_raw);

		if (gain == 16 && chip->als_inf.clear_raw >= sat) 
        {
			ret = tmg399x_set_als_gain(chip, 1);
		}   
        else
        { 
            if (gain == 16 && chip->als_inf.clear_raw < GAIN_SWITCH_LEVEL) 
            {
			    ret = tmg399x_set_als_gain(chip, 64);
		    } 
            else 
            {
                if ((gain == 64 &&
			        chip->als_inf.clear_raw >= (sat - GAIN_SWITCH_LEVEL)) ||
			        (gain == 1 && chip->als_inf.clear_raw < GAIN_SWITCH_LEVEL)) 
                {
    			    ret = tmg399x_set_als_gain(chip, 16);
    		    }
            }
        }

		if (!ret) 
        {
            SENSOR_LOG_INFO("gain adjusted, skip\n");
			tmg399x_calc_cpl(chip);
			ret = -EAGAIN;
			lux = chip->als_inf.lux;
			goto exit;
		}

		if (chip->als_inf.clear_raw <= MIN_ALS_VALUE)
        {
            //SENSOR_LOG_INFO("darkness\n");
			lux = 0;
			goto exit;
		} 
        else
        { 
            if (chip->als_inf.clear_raw >= sat)
            {
                SENSOR_LOG_INFO("saturation, keep lux, sat = %d\n",sat);
			    lux = chip->als_inf.lux;
			    goto exit;
		    }
        }
	}

	/* remove ir from counts*/
	rp1 = chip->als_inf.red_raw   - chip->als_inf.ir;
	gp1 = chip->als_inf.green_raw - chip->als_inf.ir;
	bp1 = chip->als_inf.blue_raw  - chip->als_inf.ir;
	cp1 = chip->als_inf.clear_raw - chip->als_inf.ir;

    
    if (0==rp1)
    {
        //SENSOR_LOG_INFO("!!!!!!!!! rp1 =0\n");
        rp1 = 1;
        lux_is_too_low = true;
    }
    else
    {        
        if (p_global_tmg399x_chip->light_debug_enable)
        {    
            SENSOR_LOG_INFO("rp1 = %d\n",rp1);
        }
    }
    
    /*
    SENSOR_LOG_INFO("chip->als_inf.red_raw   = %d\n",chip->als_inf.red_raw);
    SENSOR_LOG_INFO("chip->als_inf.green_raw = %d\n",chip->als_inf.green_raw);
    SENSOR_LOG_INFO("chip->als_inf.blue_raw  = %d\n",chip->als_inf.blue_raw);
    SENSOR_LOG_INFO("chip->als_inf.clear_raw = %d\n",chip->als_inf.clear_raw);
    SENSOR_LOG_INFO("lux = %d\n",lux);
    */

    //SENSOR_LOG_INFO("lux = %d\n",lux);


	if (!chip->als_inf.cpl) 
    {
    	SENSOR_LOG_ERROR("zero cpl. Setting to 1\n");

		chip->als_inf.cpl = 1;
	}

	if (chip->als_inf.red_raw > chip->als_inf.ir)
    {
		lux += chip->segment[chip->device_index].r_coef * rp1;
        if (p_global_tmg399x_chip->light_debug_enable)
        {  
            //SENSOR_LOG_INFO("lux = %d, r_coef = %d\n",lux,chip->segment[chip->device_index].r_coef);
        }
    }
	else
    {
    	//SENSOR_LOG_ERROR("lux rp1 = %d\n",(chip->segment[chip->device_index].r_coef * rp1));
    }

	if (chip->als_inf.green_raw > chip->als_inf.ir)
    {
		lux += chip->segment[chip->device_index].g_coef * gp1;
        if (p_global_tmg399x_chip->light_debug_enable)
        {  
            //SENSOR_LOG_INFO("lux = %d, g_coef = %d\n",lux,chip->segment[chip->device_index].g_coef);
        }
    }
	else
	{	
    	//SENSOR_LOG_ERROR("lux gp1 = %d\n",(chip->segment[chip->device_index].g_coef * rp1));
    }

	if (chip->als_inf.blue_raw > chip->als_inf.ir)
	{
    	lux -= chip->segment[chip->device_index].b_coef * bp1;
        if (p_global_tmg399x_chip->light_debug_enable)
        {  
            //SENSOR_LOG_INFO("lux = %d, b_coef = %d\n",lux,chip->segment[chip->device_index].b_coef);
	    }
    }
    else
	{
    	//SENSOR_LOG_ERROR("lux bp1 = %d\n",(chip->segment[chip->device_index].b_coef * rp1));
    }

    if (lux<0)
    {
        SENSOR_LOG_INFO("lux<0\n");
        lux = 0;
    }

	sf = chip->als_inf.cpl;

	if (sf > 131072)
		goto error;

    //SENSOR_LOG_INFO("sf = %d\n",sf);

	lux /= sf;
	lux *= chip->segment[chip->device_index].d_factor;
	lux += 500;
	lux /= 1000;
	chip->als_inf.lux = (u16) lux;


	cct = ((chip->segment[chip->device_index].ct_coef * bp1) / rp1) + chip->segment[chip->device_index].ct_offset;

	chip->als_inf.cct = (u16) cct;

    if (true == lux_is_too_low)
    {
        chip->als_inf.lux = 0;
        lux_is_too_low = false;
    }

exit:
return 0;

error:
	dev_err(&chip->client->dev, "ERROR Scale factor = %d", sf);

return 1;

}



static int tmg399x_resume(struct device *dev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int ret = 0;
	mutex_lock(&chip->lock);
    SENSOR_LOG_INFO("enter\n");
	if(true == chip->prx_enabled)
    {
        SENSOR_LOG_INFO( "disable irq wakeup\n");
		ret = disable_irq_wake(chip->client->irq);
	}
    if(ret < 0)
    {
		SENSOR_LOG_INFO("disable_irq_wake failed\n");
    }
    SENSOR_LOG_INFO("eixt\n");
	mutex_unlock(&chip->lock);
    return ret ;
}

//suspend  
static int tmg399x_suspend(struct device *dev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int ret = 0;
	mutex_lock(&chip->lock);
    SENSOR_LOG_INFO("enter\n");
	 tmg399x_wakelock_ops(&(chip->proximity_wakelock),false);
	if(true == chip->prx_enabled)
    {  
        SENSOR_LOG_INFO( "enable irq wakeup\n");
       	ret = enable_irq_wake(chip->client->irq);
        chip->wakeup_from_sleep = true;
    }
	if(ret < 0)
    {
		SENSOR_LOG_INFO("enable_irq_wake failed\n");
    }

    SENSOR_LOG_INFO("eixt\n");
	mutex_unlock(&chip->lock);
    return ret ;
}

static u8 tmg399x_get_ges(struct tmg399x_chip *chip)
{
	u8 gstatus = 0;
	u8 buffer[512];
    u8 gesflvl = 0;
    int i = 0;
    int j = 0;
    
	//mutex_lock(&chip->lock);

	tmg399x_i2c_read(chip, TMG399X_GES_FLVL, &gesflvl);

    
    if (chip->ges_debug_enable)
    {
        SENSOR_LOG_ERROR("gesflvl = %d\n",gesflvl);
    }

    if (gesflvl>0)
    {
        if (gesflvl>8)
        {
            gesflvl = 8;
        }

        tmg399x_i2c_read(chip, TMG399X_GES_STAT, &gstatus);
        
        //SENSOR_LOG_INFO("gstatus = %d\n",gstatus);

    	if ( (gstatus & TMG399X_GES_ST_MASK) == TMG399X_GES_VALID)
        {
            tmg399x_i2c_ram_blk_read(chip, TMG399X_GES_NFIFO, buffer, 4 * gesflvl);

            j += 4;

            for(i=0; i<(gesflvl-1); i++)
            {
                chip->gesture_data[i].north = buffer[j + 0];
                chip->gesture_data[i].south = buffer[j + 1];
                chip->gesture_data[i].west  = buffer[j + 2];
                chip->gesture_data[i].east  = buffer[j + 3];
                j += 4;

                if (chip->ges_debug_enable)
                {
                    SENSOR_LOG_ERROR("%4d, %4d, %4d, %4d\n",
                                                       chip->gesture_data[i].north,
                                                       chip->gesture_data[i].south,
                                                       chip->gesture_data[i].west,
                                                       chip->gesture_data[i].east);   
                }
            }

            if (true == chip->prx_enabled)
            {
                i--;
                chip->shadow[TMG399X_PRX_CHAN] = (chip->gesture_data[i].east  + 
                                                  chip->gesture_data[i].west  + 
                                                  chip->gesture_data[i].north +
                                                  chip->gesture_data[i].south)/4;


                //chip->shadow[TMG399X_PRX_CHAN] = (chip->shadow[TMG399X_PRX_CHAN] * 6) / 10;

                //SENSOR_LOG_INFO("chip->shadow[TMG399X_PRX_CHAN] = %d\n",chip->shadow[TMG399X_PRX_CHAN]);

                if (tmg399x_get_prox(chip))
                {
                    tmg399x_report_prox(chip);
                }
            }
        }
        else
        {
            SENSOR_LOG_ERROR("over flow !!!\n");
            gesflvl = 0;
        }
    }
    else
    {
        //SENSOR_LOG_INFO("fifi data overflow!\n");
  
        //tmg399x_i2c_modify(chip, TMG399X_GES_CFG_4, TMG399X_GES_INT_CLR, TMG399X_GES_INT_CLR);
        /*
        tmg399x_ges_enable(chip, 0);
        tmg399x_ges_enable(chip, 1);
        */

        gesflvl = 0;

    }

    return ((gesflvl-1) >= 0)? (gesflvl-1) : 0;

    //mutex_unlock(&chip->lock);

}

static int tmg399x_get_prox(struct tmg399x_chip *chip)
{
    int ret = 0;
	chip->prx_inf.raw = chip->shadow[TMG399X_PRX_CHAN];

    if (chip->prox_debug_enable || chip->prox_calibrate_start)
    {
        SENSOR_LOG_ERROR("state = %d, data = %d, hi = %d, low = %d\n",
                                                                     chip->prx_inf.detected,
                                                                     chip->prx_inf.raw,
                                                                     chip->params.prox_th_high,
                                                                     chip->params.prox_th_low);
        if (chip->prox_calibrate_start)
        {
            ret = 1;
        }
    }
    else
    {
        if (chip->prx_inf.detected == PROX_UNKNOW)
        {
            if (chip->prx_inf.raw >= chip->params.prox_th_high) 
            {
                SENSOR_LOG_INFO("NEAR!\n");
                chip->prx_inf.detected = PROX_NEAR;
                chip->shadow[TMG399X_PRX_THRES_LOW] = chip->params.prox_th_low;
                chip->shadow[TMG399X_PRX_THRES_HIGH] = 0xff;
                tmg399x_i2c_write(chip, TMG399X_PRX_THRES_LOW, chip->shadow[TMG399X_PRX_THRES_LOW]);
                tmg399x_i2c_write(chip, TMG399X_PRX_THRES_HIGH, chip->shadow[TMG399X_PRX_THRES_HIGH]);
                ret = 1;
            }
            else
            {
                SENSOR_LOG_INFO("FAR!\n");
                chip->prx_inf.detected = PROX_FAR;
                chip->shadow[TMG399X_PRX_THRES_LOW] = 0x00;
                chip->shadow[TMG399X_PRX_THRES_HIGH] = chip->params.prox_th_high;
                tmg399x_i2c_write(chip, TMG399X_PRX_THRES_LOW, chip->shadow[TMG399X_PRX_THRES_LOW]);
                tmg399x_i2c_write(chip, TMG399X_PRX_THRES_HIGH, chip->shadow[TMG399X_PRX_THRES_HIGH]);
                ret = 1;  
            }
         }
         else
         {
            if (chip->prx_inf.detected == PROX_FAR) 
            {
                if (chip->prx_inf.raw >= chip->shadow[TMG399X_PRX_THRES_HIGH]) 
                {
                    SENSOR_LOG_INFO("NEAR!\n");
                    chip->prx_inf.detected = PROX_NEAR;
                    chip->shadow[TMG399X_PRX_THRES_LOW] = chip->params.prox_th_low;
                    chip->shadow[TMG399X_PRX_THRES_HIGH] = 0xff;
                    tmg399x_i2c_write(chip, TMG399X_PRX_THRES_LOW, chip->shadow[TMG399X_PRX_THRES_LOW]);
                    tmg399x_i2c_write(chip, TMG399X_PRX_THRES_HIGH, chip->shadow[TMG399X_PRX_THRES_HIGH]);
                    ret = 1;
                }
            }
            else 
            {
                if (chip->prx_inf.raw <= chip->shadow[TMG399X_PRX_THRES_LOW]) 
                {
                    SENSOR_LOG_INFO("FAR!\n");
                    chip->prx_inf.detected = PROX_FAR;
                    chip->shadow[TMG399X_PRX_THRES_LOW] = 0x00;
                    chip->shadow[TMG399X_PRX_THRES_HIGH] = chip->params.prox_th_high;
                    tmg399x_i2c_write(chip, TMG399X_PRX_THRES_LOW, chip->shadow[TMG399X_PRX_THRES_LOW]);
                    tmg399x_i2c_write(chip, TMG399X_PRX_THRES_HIGH, chip->shadow[TMG399X_PRX_THRES_HIGH]); 
                    ret = 1;
                }
                /*
                else
                {
                    SENSOR_LOG_INFO("FAR!\n");
                    chip->prx_inf.detected = PROX_FAR;
                    chip->shadow[TMG399X_PRX_THRES_LOW] = 0x00;
                    chip->shadow[TMG399X_PRX_THRES_HIGH] = chip->params.prox_th_high;
                    tmg399x_i2c_write(chip, TMG399X_PRX_THRES_LOW, chip->shadow[TMG399X_PRX_THRES_LOW]);
                    tmg399x_i2c_write(chip, TMG399X_PRX_THRES_HIGH, chip->shadow[TMG399X_PRX_THRES_HIGH]); 
                }
                */
            }
         }
    }

    
   //SENSOR_LOG_INFO("HIGH = %d, LOW = %d\n",chip->shadow[TMG399X_PRX_THRES_HIGH],chip->shadow[TMG399X_PRX_THRES_LOW]);

    return ret;
}

static void tmg399x_get_als(struct tmg399x_chip *chip)
{
	u8 *buf = &chip->shadow[TMG399X_CLR_CHANLO];

	/* extract raw channel data */
	chip->als_inf.clear_raw = le16_to_cpup((const __le16 *)&buf[0]);
	chip->als_inf.red_raw   = le16_to_cpup((const __le16 *)&buf[2]);
	chip->als_inf.green_raw = le16_to_cpup((const __le16 *)&buf[4]);
	chip->als_inf.blue_raw  = le16_to_cpup((const __le16 *)&buf[6]);
	chip->als_inf.ir =
		(chip->als_inf.red_raw + chip->als_inf.green_raw +
		chip->als_inf.blue_raw - chip->als_inf.clear_raw + 1) >> 1;
	if (chip->als_inf.ir < 0)
		chip->als_inf.ir = 0;
}

static int tmg399x_read_all(struct tmg399x_chip *chip)
{
	int ret;

	//mutex_lock(&chip->lock);
	
	tmg399x_i2c_read(chip, TMG399X_STATUS,
			&chip->shadow[TMG399X_STATUS]);

	tmg399x_i2c_read(chip, TMG399X_CLR_CHANLO,
			&chip->shadow[TMG399X_CLR_CHANLO]);
	tmg399x_i2c_read(chip, TMG399X_CLR_CHANHI,
			&chip->shadow[TMG399X_CLR_CHANHI]);

	tmg399x_i2c_read(chip, TMG399X_RED_CHANLO,
			&chip->shadow[TMG399X_RED_CHANLO]);
	tmg399x_i2c_read(chip, TMG399X_RED_CHANHI,
			&chip->shadow[TMG399X_RED_CHANHI]);

	tmg399x_i2c_read(chip, TMG399X_GRN_CHANLO,
			&chip->shadow[TMG399X_GRN_CHANLO]);
	tmg399x_i2c_read(chip, TMG399X_GRN_CHANHI,
			&chip->shadow[TMG399X_GRN_CHANHI]);

	tmg399x_i2c_read(chip, TMG399X_BLU_CHANLO,
			&chip->shadow[TMG399X_BLU_CHANLO]);
	tmg399x_i2c_read(chip, TMG399X_BLU_CHANHI,
			&chip->shadow[TMG399X_BLU_CHANHI]);

	ret = tmg399x_i2c_read(chip, TMG399X_PRX_CHAN,
			&chip->shadow[TMG399X_PRX_CHAN]);

	//mutex_unlock(&chip->lock);
	return (ret < 0) ? ret : 0;
}

/*
static int tmg399x_read_status(struct tmg399x_chip *chip)
{
	int ret = 0;
    
	//mutex_lock(&chip->lock);

    ret = tmg399x_i2c_read(chip, TMG399X_STATUS, &chip->shadow[TMG399X_STATUS]);

	//mutex_unlock(&chip->lock);

	return (ret < 0) ? ret : 0;
}
*/

static int tmg399x_read_prox_data(struct tmg399x_chip *chip)
{
	int ret = 0;
    ret = tmg399x_i2c_read(chip, TMG399X_PRX_CHAN, &chip->shadow[TMG399X_PRX_CHAN]);
	return (ret < 0) ? ret : 0;
}

static int tmg399x_read_rgb_data(struct tmg399x_chip *chip)
{
    
	//mutex_lock(&chip->lock);

	tmg399x_i2c_read(chip, TMG399X_CLR_CHANLO, &chip->shadow[TMG399X_CLR_CHANLO]);
	tmg399x_i2c_read(chip, TMG399X_CLR_CHANHI, &chip->shadow[TMG399X_CLR_CHANHI]);

	tmg399x_i2c_read(chip, TMG399X_RED_CHANLO, &chip->shadow[TMG399X_RED_CHANLO]);
	tmg399x_i2c_read(chip, TMG399X_RED_CHANHI, &chip->shadow[TMG399X_RED_CHANHI]);

	tmg399x_i2c_read(chip, TMG399X_GRN_CHANLO, &chip->shadow[TMG399X_GRN_CHANLO]);
	tmg399x_i2c_read(chip, TMG399X_GRN_CHANHI, &chip->shadow[TMG399X_GRN_CHANHI]);

	tmg399x_i2c_read(chip, TMG399X_BLU_CHANLO, &chip->shadow[TMG399X_BLU_CHANLO]);
	tmg399x_i2c_read(chip, TMG399X_BLU_CHANHI, &chip->shadow[TMG399X_BLU_CHANHI]);

	//mutex_unlock(&chip->lock);

	return 0;
}

static void tmg399x_als_atime_set(u8 time)
{
	p_global_tmg399x_chip->shadow[TMG399X_ALS_TIME] = time;
	p_global_tmg399x_chip->params.als_time = time;
	tmg399x_i2c_write(p_global_tmg399x_chip, TMG399X_ALS_TIME, p_global_tmg399x_chip->shadow[TMG399X_ALS_TIME]);
}

static int tmg399x_update_als_thres(struct tmg399x_chip *chip, bool on_enable)
{
	s32 ret;
	u8 *buf = &chip->shadow[TMG399X_ALS_MINTHRESHLO];
	u16 deltaP = chip->params.als_deltaP;
	u16 from, to, cur;
	u16 saturation = chip->als_inf.saturation;

	cur = chip->als_inf.clear_raw;

	if (on_enable) {
		/* move deltaP far away from current position to force an irq */
		from = to = cur > saturation / 2 ? 0 : saturation;
	} else {
		deltaP = cur * deltaP / 100;
		if (!deltaP)
			deltaP = 1;

		if (cur > deltaP)
			from = cur - deltaP;
		else
			from = 0;

		if (cur < (saturation - deltaP))
			to = cur + deltaP;
		else
			to = saturation;

	}

	*buf++ = from & 0xff;
	*buf++ = from >> 8;
	*buf++ = to & 0xff;
	*buf++ = to >> 8;
	ret = tmg399x_i2c_reg_blk_write(chip, TMG399X_ALS_MINTHRESHLO,
			&chip->shadow[TMG399X_ALS_MINTHRESHLO],
			TMG399X_ALS_MAXTHRESHHI - TMG399X_ALS_MINTHRESHLO + 1);

	return (ret < 0) ? ret : 0;
}

static int tmg399x_ges_init(struct tmg399x_chip *chip, int on)
{
	int ret;
	
	if (on) 
    {
		if (chip->pdata) 
        {
			chip->params.ges_entry_th = chip->pdata->parameters.ges_entry_th;
			chip->params.ges_exit_th = chip->pdata->parameters.ges_exit_th;
			chip->params.ges_cfg1 = chip->pdata->parameters.ges_cfg1;
			chip->params.ges_cfg2 = chip->pdata->parameters.ges_cfg2;
			chip->params.ges_offset_n = chip->pdata->parameters.ges_offset_n;
			chip->params.ges_offset_s = chip->pdata->parameters.ges_offset_s;
			chip->params.ges_pulse = chip->pdata->parameters.ges_pulse;
			chip->params.ges_offset_w = chip->pdata->parameters.ges_offset_w;
			chip->params.ges_offset_e = chip->pdata->parameters.ges_offset_e;
			chip->params.ges_dimension = chip->pdata->parameters.ges_dimension;
		} 
        else 
        {
			chip->params.ges_entry_th = param_default.ges_entry_th;
			chip->params.ges_exit_th = param_default.ges_exit_th;
			chip->params.ges_cfg1 = param_default.ges_cfg1;
			chip->params.ges_cfg2 = param_default.ges_cfg2;
			chip->params.ges_offset_n = param_default.ges_offset_n;
			chip->params.ges_offset_s = param_default.ges_offset_s;
			chip->params.ges_pulse = param_default.ges_pulse;
			chip->params.ges_offset_w = param_default.ges_offset_w;
			chip->params.ges_offset_e = param_default.ges_offset_e;
			chip->params.ges_dimension = param_default.ges_dimension;
		}
	}
    else 
    {
		chip->params.ges_entry_th = 0;
		chip->params.ges_exit_th = 0;
		chip->params.ges_cfg1 = 0;
		chip->params.ges_cfg2 = 0;
		chip->params.ges_offset_n = 0;
		chip->params.ges_offset_s = 0;
		chip->params.ges_pulse = 0;
		chip->params.ges_offset_w = 0;
		chip->params.ges_offset_e = 0;
		chip->params.ges_dimension = 0;	
	}

	/* Initial gesture registers */
	chip->shadow[TMG399X_GES_ENTH]  = chip->params.ges_entry_th;
	chip->shadow[TMG399X_GES_EXTH]  = chip->params.ges_exit_th;
	chip->shadow[TMG399X_GES_CFG_1] = chip->params.ges_cfg1;
	chip->shadow[TMG399X_GES_CFG_2] = chip->params.ges_cfg2;
	chip->shadow[TMG399X_GES_OFFSET_N] = chip->params.ges_offset_n;
	chip->shadow[TMG399X_GES_OFFSET_S] = chip->params.ges_offset_s;
	chip->shadow[TMG399X_GES_PULSE] = chip->params.ges_pulse;
	chip->shadow[TMG399X_GES_OFFSET_W] = chip->params.ges_offset_w;
	chip->shadow[TMG399X_GES_OFFSET_E] = chip->params.ges_offset_e;
	chip->shadow[TMG399X_GES_CFG_3] = chip->params.ges_dimension;
	
	ret  = tmg399x_i2c_write(chip, TMG399X_GES_ENTH,     chip->shadow[TMG399X_GES_ENTH]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_EXTH,     chip->shadow[TMG399X_GES_EXTH]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_1,    chip->shadow[TMG399X_GES_CFG_1]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_2,    chip->shadow[TMG399X_GES_CFG_2]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_N, chip->shadow[TMG399X_GES_OFFSET_N]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_S, chip->shadow[TMG399X_GES_OFFSET_S]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_PULSE,    chip->shadow[TMG399X_GES_PULSE]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_W, chip->shadow[TMG399X_GES_OFFSET_W]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_E, chip->shadow[TMG399X_GES_OFFSET_E]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_3,    chip->shadow[TMG399X_GES_CFG_3]);

	return ret;
}

static int tmg399x_ges_enable(struct tmg399x_chip *chip, int on)
{
	int ret = 0;

    SENSOR_LOG_INFO("on = %d\n",on);
	if (on) 
    {
		/* initialize */		
		ret |= tmg399x_ges_init(chip, 1);		
		if (ret < 0)
        {
            SENSOR_LOG_INFO("tmg399x_ges_init failed!\n");
			return ret;
        }
        
        chip->shadow[TMG399X_CONTROL] |= (TMG399X_EN_PWR_ON | TMG399X_EN_PRX | TMG399X_EN_GES);
	    chip->shadow[TMG399X_CONTROL] &=(~(TMG399X_EN_WAIT));
		chip->shadow[TMG399X_GES_CFG_4] |= (TMG399X_GES_EN_IRQ | TMG399X_GES_MODE);
		ret |= tmg399x_update_enable_reg(chip);
		
		mdelay(3);

        chip->light_poll_time = AMS_ALS_POLL_DELAY_SLOW;
        tmg399x_als_atime_set(AMS_ALS_ATIME_SHORT);
        tmg399x_irq_enable(true, true);

	} 
    else 
    {
        if (false == chip->prx_enabled)
        {                        
            chip->light_poll_time = AMS_ALS_POLL_DELAY_FAST;
            tmg399x_als_atime_set(AMS_ALS_ATIME_LONG);
            tmg399x_irq_enable(false, true);
        }

		chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_GES;

        if ((false == chip->prx_enabled) && (false == chip->als_enabled))
        {
			chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PWR_ON;
        }
		chip->shadow[TMG399X_GES_CFG_4] &= ~(TMG399X_GES_EN_IRQ | TMG399X_GES_MODE);

		ret = tmg399x_update_enable_reg(chip);
        
		/* deinitialize */		
		ret |= tmg399x_ges_init(chip, 0);
		if (ret)
        {
            SENSOR_LOG_INFO("tmg399x_ges_init failed!\n");
			return ret;
        }
	}

    SENSOR_LOG_INFO("chip->shadow[TMG399X_CONTROL] = %X\n",chip->shadow[TMG399X_CONTROL]);

	return ret;
}

static int tmg399x_prox_enable(struct tmg399x_chip *chip, int on)
{
	int ret;

    SENSOR_LOG_INFO("on = %d\n",on);

	if (on) 
    {
		tmg399x_irq_clr(chip, TMG399X_CMD_PROX_INT_CLR);

        tmg399x_i2c_write(chip, TMG399X_PRX_THRES_LOW,  0x01);
        tmg399x_i2c_write(chip, TMG399X_PRX_THRES_HIGH, 0x01);

		chip->shadow[TMG399X_CONTROL] |= (TMG399X_EN_PWR_ON | TMG399X_EN_PRX | TMG399X_EN_PRX_IRQ | TMG399X_EN_WAIT);

		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		mdelay(3);
    
        chip->light_poll_time = AMS_ALS_POLL_DELAY_SLOW;
        tmg399x_als_atime_set(AMS_ALS_ATIME_SHORT);

        chip->prx_inf.detected = PROX_UNKNOW;
        tmg399x_irq_enable(true, true);
	} 
    else 
    {
        if (true == (chip->proximity_wakelock).locked)
        {
            tmg399x_wakelock_ops(&(chip->proximity_wakelock),false);
        }


        if (false == chip->ges_enabled)
        {
            chip->light_poll_time = AMS_ALS_POLL_DELAY_FAST;
            tmg399x_als_atime_set(AMS_ALS_ATIME_LONG);
            tmg399x_irq_enable(false, true);
        }

        chip->shadow[TMG399X_CONTROL] &= ~(TMG399X_EN_PRX_IRQ | TMG399X_EN_PRX);

        if ((false == chip->als_enabled) && (false == chip->ges_enabled))
        {            
            chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PWR_ON;
        }

		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		tmg399x_irq_clr(chip, TMG399X_CMD_PROX_INT_CLR);
        chip->prx_inf.detected = PROX_UNKNOW;
	}

    SENSOR_LOG_INFO("chip->shadow[TMG399X_CONTROL] = %X\n",chip->shadow[TMG399X_CONTROL]);

	return ret;
}

static int tmg399x_als_enable(struct tmg399x_chip *chip, int on)
{
	int ret;

    SENSOR_LOG_INFO("on = %d\n",on);

	if (on) 
    {
        chip->als_enabled = true;
		tmg399x_irq_clr(chip, TMG399X_CMD_ALS_INT_CLR);
		tmg399x_update_als_thres(chip, 1);
		chip->shadow[TMG399X_CONTROL] |= (TMG399X_EN_PWR_ON | TMG399X_EN_ALS /*| TMG399X_EN_ALS_IRQ*/);
		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		mdelay(3);
        schedule_delayed_work(&p_global_tmg399x_chip->als_poll_work, msecs_to_jiffies(chip->light_poll_time));
	}
    else
    {
        chip->als_enabled = false;
		//chip->shadow[TMG399X_CONTROL] &= ~(TMG399X_EN_ALS_IRQ);

        if ((false == chip->prx_enabled) && (false == chip->ges_enabled))
        {
             chip->shadow[TMG399X_CONTROL] &= ~ (TMG399X_EN_ALS | TMG399X_EN_PWR_ON);
        }     

		ret = tmg399x_update_enable_reg(chip);

		if (ret < 0)
			return ret;
		tmg399x_irq_clr(chip, TMG399X_CMD_ALS_INT_CLR);
        //cancel_delayed_work_sync(&p_global_tmg399x_chip->als_poll_work);
	}

    SENSOR_LOG_INFO("chip->shadow[TMG399X_CONTROL] = %X\n",chip->shadow[TMG399X_CONTROL]);

	if (!ret)
		chip->als_enabled = on;

	return ret;
}

static int tmg399x_wait_enable(struct tmg399x_chip *chip, int on)
{
	int ret;

	dev_info(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
		chip->shadow[TMG399X_CONTROL] |= TMG399X_EN_WAIT;

		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		mdelay(3);
	} else {
		chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_WAIT;

		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
	}
	if (!ret)
	        chip->wait_enabled = on;

	return ret;
}

static ssize_t tmg399x_ges_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ges_enabled);
}

static ssize_t tmg399x_ges_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
    mutex_lock(&chip->lock);

    chip->ges_enabled = (value>0) ? true : false;

	if (value)
    {
        if (false==chip->prx_enabled)
        {
		    tmg399x_ges_enable(chip, true);
        }
	}
    else
    {   
		tmg399x_ges_enable(chip, false);
        chip->gesture_start = false;
    }
    mutex_unlock(&chip->lock);

	return size;
}

static ssize_t tmg399x_prox_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_enabled);
}

static ssize_t tmg399x_prox_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
    mutex_lock(&chip->lock);

    chip->prx_enabled = (value>0) ? true : false;

	if (value)
    {
        if (true == chip->ges_enabled)
        {
            tmg399x_ges_enable(chip, false);
        }

		tmg399x_prox_enable(chip, true);
    }
	else
    {
		tmg399x_prox_enable(chip, false);

        if (true == chip->ges_enabled)
        {
            tmg399x_ges_enable(chip, true);
        }
    }
    mutex_unlock(&chip->lock);

	return size;
}

static ssize_t tmg399x_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tmg399x_als_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

    mutex_lock(&chip->lock);

	if (value)
    {
		tmg399x_als_enable(chip, true);
	}
    else
    {
		tmg399x_als_enable(chip, false);
    }
    mutex_unlock(&chip->lock);

	return size;
}

static ssize_t tmg399x_light_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "light_debug_enable is %s\n", chip->light_debug_enable? "true":"false");
}

static ssize_t tmg399x_light_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
    mutex_lock(&chip->lock);

	if (value)
    {
        chip->light_debug_enable = true;
	}
    else
    {
        chip->light_debug_enable = false;
    }
    mutex_unlock(&chip->lock);

	return size;
}

static ssize_t tmg399x_prox_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "prox_debug_enable is %s\n", chip->prox_debug_enable? "true":"false");
}

static ssize_t tmg399x_prox_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
    mutex_lock(&chip->lock);

	if (value)
    {
        chip->prox_debug_enable = true;
	}
    else
    {
        chip->prox_debug_enable = false;
    }
    mutex_unlock(&chip->lock);

	return size;
}


static ssize_t tmg399x_prox_calibrate_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prox_calibrate_result);
}

static ssize_t tmg399x_chip_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", chip->chip_name);
}

static ssize_t tmg399x_prox_thres_hi_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prox_thres_hi_max);
}

static ssize_t tmg399x_prox_thres_lo_min(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prox_thres_lo_min);
}

static ssize_t tmg399x_prox_data_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prox_data_max);
}
static ssize_t tmg399x_manual_calibrate_threshold(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prox_manual_calibrate_threshold);
}


static ssize_t tmg399x_ges_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "ges_debug_enable is %s\n", chip->ges_debug_enable? "true":"false");
}

static ssize_t tmg399x_ges_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
    mutex_lock(&chip->lock);

	if (value)
    {
        chip->ges_debug_enable = true;
	}
    else
    {
        chip->ges_debug_enable = false;
    }
    mutex_unlock(&chip->lock);

	return size;
}


static ssize_t tmg399x_wait_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->wait_enabled);
}

static ssize_t tmg399x_wait_enable_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t size)
{
    struct tmg399x_chip *chip = dev_get_drvdata(dev);
    bool value;

    if (strtobool(buf, &value))
            return -EINVAL;
	mutex_lock(&chip->lock);

    if (value)
            tmg399x_wait_enable(chip, 1);
    else
            tmg399x_wait_enable(chip, 0);

	mutex_unlock(&chip->lock);
    return size;
}

static ssize_t tmg399x_als_itime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int t;
	t = 256 - chip->params.als_time;
	t *= TMG399X_ATIME_PER_100;
	t /= 100;
	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tmg399x_als_itime_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long itime;
	int rc;

	rc = kstrtoul(buf, 10, &itime);
	if (rc)
		return -EINVAL;
	if (itime > 712 || itime < 3) {
		dev_err(&chip->client->dev,
			"als integration time range [3,712]\n");
		return -EINVAL;
	}

	itime *= 100;
	itime /= TMG399X_ATIME_PER_100;
	itime = (256 - itime);
   
	mutex_lock(&chip->lock);
	/*
    chip->shadow[TMG399X_ALS_TIME] = (u8)itime;
	chip->params.als_time = (u8)itime;
	tmg399x_i2c_write(chip, TMG399X_ALS_TIME, chip->shadow[TMG399X_ALS_TIME]);
	*/

    tmg399x_als_atime_set((u8)itime);

    mutex_unlock(&chip->lock);
	
    return size;
}

static ssize_t tmg399x_als_poll_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "delay is %d ms\n", chip->light_poll_time);
}

static ssize_t tmg399x_als_poll_time_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long time;
	int rc;
    struct tmg399x_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &time);
	if (rc)
		return -EINVAL;
    SENSOR_LOG_INFO("set delay to %d ms\n",(int)time);
	chip->light_poll_time = (int)time;
	return size;
}


static ssize_t tmg399x_wait_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int t;
	t = 256 - chip->params.wait_time;
	t *= TMG399X_ATIME_PER_100;
	t /= 100;
	if (chip->params.als_prox_cfg1 & WLONG)
		t *= 12;
	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tmg399x_wait_time_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long time;
	int ret;
	u8 cfg1;

	ret = kstrtoul(buf, 10, &time);
	if (ret) {
		return -EINVAL;
	}
	if (time > 8540 || time < 3) {
		dev_err(&chip->client->dev,
			"wait time range [3,8540]\n");
		return -EINVAL;
	}
	
	cfg1 = chip->shadow[TMG399X_CONFIG_1] & ~0x02;
	if (time > 712) {
		cfg1 |= WLONG;
		time /= 12;
	}

	time *= 100;
	time /= TMG399X_ATIME_PER_100;
	time = (256 - time);
	mutex_lock(&chip->lock);
	chip->shadow[TMG399X_WAIT_TIME] = (u8)time;
	chip->params.wait_time = (u8)time;
	chip->shadow[TMG399X_CONFIG_1] = cfg1;
	chip->params.als_prox_cfg1 = cfg1;
	tmg399x_i2c_write(chip, TMG399X_WAIT_TIME,
		chip->shadow[TMG399X_WAIT_TIME]);
	tmg399x_i2c_write(chip, TMG399X_CONFIG_1,
		chip->shadow[TMG399X_CONFIG_1]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_persist_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.persist & 0xF0) >> 4);
}

static ssize_t tmg399x_prox_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long persist;
	int ret;

	ret = kstrtoul(buf, 10, &persist);
	if (ret) {
		return -EINVAL;
	}
	
	if (persist > 15) {
		dev_err(&chip->client->dev,
			"prox persistence range [0,15]\n");
		return -EINVAL;
	}
	
	mutex_lock(&chip->lock);
	chip->shadow[TMG399X_PERSISTENCE] &= 0x0F;
	chip->shadow[TMG399X_PERSISTENCE] |= (((u8)persist << 4) & 0xF0);
	chip->params.persist = chip->shadow[TMG399X_PERSISTENCE];
	tmg399x_i2c_write(chip, TMG399X_PERSISTENCE,
		chip->shadow[TMG399X_PERSISTENCE]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_persist_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.persist & 0x0F));
}

static ssize_t tmg399x_als_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long persist;
	int ret;

	ret = kstrtoul(buf, 10, &persist);
	if (ret) {
		return -EINVAL;
	}

	if (persist > 15) {
		dev_err(&chip->client->dev,
			"als persistence range [0,15]\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	chip->shadow[TMG399X_PERSISTENCE] &= 0xF0;
	chip->shadow[TMG399X_PERSISTENCE] |= ((u8)persist & 0x0F);
	chip->params.persist = chip->shadow[TMG399X_PERSISTENCE];
	tmg399x_i2c_write(chip, TMG399X_PERSISTENCE,
		chip->shadow[TMG399X_PERSISTENCE]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_pulse_len_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = (chip->params.prox_pulse & 0xC0) >> 6;
	return snprintf(buf, PAGE_SIZE, "%duS\n", prox_pplens[i]);
}

static ssize_t tmg399x_prox_pulse_len_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long length;
	int ret;
	u8 ppulse;

	ret = kstrtoul(buf, 10, &length);
	if (ret) {
		return -EINVAL;
	}

	if (length != 4 && length != 8 &&
		length != 16 &&	length != 32) {
		dev_err(&chip->client->dev, 
			"pulse length set: {4, 8, 16, 32}\n");
		return -EINVAL;
	}
	
	mutex_lock(&chip->lock);
	ppulse = chip->shadow[TMG399X_PRX_PULSE] & 0x3F;
	switch (length){
	case 4:
		ppulse |= PPLEN_4US;
		break;
	case 8:
		ppulse |= PPLEN_8US;
		break;
	case 16:
		ppulse |= PPLEN_16US;
		break;
	case 32:
		ppulse |= PPLEN_32US;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_PRX_PULSE] = ppulse;
	chip->params.prox_pulse = ppulse;
	tmg399x_i2c_write(chip, TMG399X_PRX_PULSE,
		chip->shadow[TMG399X_PRX_PULSE]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_pulse_cnt_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.prox_pulse & 0x3F) + 1);
}

static ssize_t tmg399x_prox_pulse_cnt_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long count;
	int ret;

	ret = kstrtoul(buf, 10, &count);
	if (ret) {
		return -EINVAL;
	}

	if (count > 32 || count == 0) {
		dev_err(&chip->client->dev,
			"prox pulse count range [1,32]\n");
		return -EINVAL;
	}
	count -= 1;

	mutex_lock(&chip->lock);
	chip->shadow[TMG399X_PRX_PULSE] &= 0xC0;
	chip->shadow[TMG399X_PRX_PULSE] |= ((u8)count & 0x3F);
	chip->params.prox_pulse = chip->shadow[TMG399X_PRX_PULSE];
	tmg399x_i2c_write(chip, TMG399X_PRX_PULSE,
		chip->shadow[TMG399X_PRX_PULSE]);
	mutex_unlock(&chip->lock);
	return size;
}


static ssize_t tmg399x_ges_pulse_cnt_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.ges_pulse & 0x3F) + 1);
}

static ssize_t tmg399x_ges_pulse_cnt_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long count;
	int ret;

	ret = kstrtoul(buf, 10, &count);
	if (ret) {
		return -EINVAL;
	}

	if (count > 32 || count == 0) {
		dev_err(&chip->client->dev,
			"prox pulse count range [1,32]\n");
		return -EINVAL;
	}
	count -= 1;

	mutex_lock(&chip->lock);
	chip->shadow[TMG399X_GES_PULSE] &= 0xC0;
	chip->shadow[TMG399X_GES_PULSE] |= ((u8)count & 0x3F);
	chip->params.ges_pulse = chip->shadow[TMG399X_GES_PULSE];
	tmg399x_i2c_write(chip, TMG399X_GES_PULSE, chip->shadow[TMG399X_GES_PULSE]);
	mutex_unlock(&chip->lock);
	return size;
}



static ssize_t tmg399x_ges_pulse_len_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = (chip->params.ges_pulse & 0xC0) >> 6;
	return snprintf(buf, PAGE_SIZE, "%duS\n", prox_pplens[i]);
}

static ssize_t tmg399x_ges_pulse_len_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long length;
	int ret;
	u8 ppulse;

	ret = kstrtoul(buf, 10, &length);
	if (ret) {
		return -EINVAL;
	}

	if (length != 4 && length != 8 &&
		length != 16 &&	length != 32) {
		dev_err(&chip->client->dev, 
			"pulse length set: {4, 8, 16, 32}\n");
		return -EINVAL;
	}
	
	mutex_lock(&chip->lock);
	ppulse = chip->shadow[TMG399X_GES_PULSE] & 0x3F;
	switch (length){
	case 4:
		ppulse |= GPLEN_4US;
		break;
	case 8:
		ppulse |= GPLEN_8US;
		break;
	case 16:
		ppulse |= GPLEN_16US;
		break;
	case 32:
		ppulse |= GPLEN_32US;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_GES_PULSE] = ppulse;
	chip->params.ges_pulse = ppulse;
	tmg399x_i2c_write(chip, TMG399X_GES_PULSE,
		chip->shadow[TMG399X_GES_PULSE]);
	mutex_unlock(&chip->lock);
	return size;
}


static ssize_t tmg399x_ges_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = chip->params.ges_cfg2 >> 2;
	return snprintf(buf, PAGE_SIZE, "%d\n", prox_gains[i]);
}

static ssize_t tmg399x_ges_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long gain;
	int ret;
	u8 ctrl_reg;

	ret = kstrtoul(buf, 10, &gain);
	if (ret)
    {
		return -EINVAL;
    }
	if (gain != 1 && gain != 2 && gain != 4 && gain != 8) {
		dev_err(&chip->client->dev,
			"prox gain set: {1, 2, 4, 8}\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	ctrl_reg = chip->shadow[TMG399X_GES_CFG_2] & ~TMG399X_GES_GAIN_MASK;
	switch (gain){
	case 1:
		ctrl_reg |= GGAIN_1;
		break;
	case 2:
		ctrl_reg |= GGAIN_2;
		break;
	case 4:
		ctrl_reg |= GGAIN_4;
		break;
	case 8:
		ctrl_reg |= GGAIN_8;
		break;
	default:
		break;
	}

	ret = tmg399x_i2c_write(chip, TMG399X_GES_CFG_2, ctrl_reg);
	if (!ret) {
		chip->shadow[TMG399X_GES_CFG_2] = ctrl_reg;
		chip->params.ges_cfg2 = ctrl_reg & TMG399X_GES_GAIN_MASK;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}




static ssize_t tmg399x_prox_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = chip->params.prox_gain >> 2;
	return snprintf(buf, PAGE_SIZE, "%d\n", prox_gains[i]);
}

static ssize_t tmg399x_prox_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long gain;
	int ret;
	u8 ctrl_reg;

	ret = kstrtoul(buf, 10, &gain);
	if (ret)
    {
		return -EINVAL;
    }
	if (gain != 1 && gain != 2 && gain != 4 && gain != 8) {
		dev_err(&chip->client->dev,
			"prox gain set: {1, 2, 4, 8}\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	ctrl_reg = chip->shadow[TMG399X_GAIN] & ~TMG399X_PRX_GAIN_MASK;
	//cfg1_reg = chip->shadow[TMG399X_CONFIG_1] & ~0x80;
	switch (gain){
	case 1:
		ctrl_reg |= PGAIN_1;
		break;
	case 2:
		ctrl_reg |= PGAIN_2;
		break;
	case 4:
		ctrl_reg |= PGAIN_4;
		break;
	case 8:
		ctrl_reg |= PGAIN_8;
		break;
	default:
		break;
	}

	ret = tmg399x_i2c_write(chip, TMG399X_GAIN, ctrl_reg);
//	ret |= tmg399x_i2c_write(chip, TMG399X_CONFIG_1, cfg1_reg);
	if (!ret) {
		chip->shadow[TMG399X_GAIN] = ctrl_reg;
		chip->params.prox_gain = ctrl_reg & TMG399X_PRX_GAIN_MASK;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_led_drive_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%dmA\n",
			led_drives[chip->params.ldrive]);
}

static ssize_t tmg399x_prox_led_drive_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long ldrive;
	int ret;
	u8 ctrl_reg;

	ret = kstrtoul(buf, 10, &ldrive);
	if (ret) 
    {
		return -EINVAL;
    }
	if (ldrive != 100 && ldrive != 50 &&
		ldrive != 25 && ldrive != 12) {
		dev_err(&chip->client->dev,
			"led drive set: {100, 50, 25, 12}\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	ctrl_reg = chip->shadow[TMG399X_GAIN] & ~TMG399X_LDRIVE_MASK;
	switch (ldrive){
	case 100:
		ctrl_reg |= PDRIVE_100MA;
		chip->params.ldrive = 0;
		break;
	case 50:
		ctrl_reg |= PDRIVE_50MA;
		chip->params.ldrive = 1;
		break;
	case 25:
		ctrl_reg |= PDRIVE_25MA;
		chip->params.ldrive = 2;
		break;
	case 12:
		ctrl_reg |= PDRIVE_12MA;
		chip->params.ldrive = 3;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_GAIN] = ctrl_reg;
	tmg399x_i2c_write(chip, TMG399X_GAIN, chip->shadow[TMG399X_GAIN]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (%s)\n",
			als_gains[chip->params.als_gain],
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmg399x_als_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long gain;
	int i = 0;
	int ret;

	ret = kstrtoul(buf, 10, &gain);
	if (ret) 
    {
		return -EINVAL;
	}	
	if (gain != 0 && gain != 1 && gain != 4 &&
		gain != 16 && gain != 64) {
		dev_err(&chip->client->dev,
			"als gain set: {0(auto), 1, 4, 16, 64}\n");
		return -EINVAL;
	}

	while (i < sizeof(als_gains)) {
		if (gain == als_gains[i])
			break;
		i++;
	}

	if (gain) {
		chip->als_gain_auto = false;
		ret = tmg399x_set_als_gain(chip, als_gains[i]);
		if (!ret)
			tmg399x_calc_cpl(chip);
	} else {
		chip->als_gain_auto = true;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_led_boost_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = (chip->params.als_prox_cfg2 & 0x30) >> 4;
	return snprintf(buf, PAGE_SIZE, "%d percents\n", led_boosts[i]);
}

static ssize_t tmg399x_led_boost_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long lboost;
	int ret;
	u8 cfg2;

	ret = kstrtoul(buf, 10, &lboost);
	if (ret) {
		return -EINVAL;
    }
	if (lboost != 100 && lboost != 150 &&
		lboost != 200 && lboost != 300) {
		dev_err(&chip->client->dev,
			"led boost set: {100, 150, 200, 300}\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	cfg2 = chip->shadow[TMG399X_CONFIG_2] & ~0x30;
	switch (lboost){
	case 100:
		cfg2 |= LEDBOOST_100;
		break;
	case 150:
		cfg2 |= LEDBOOST_150;
		break;
	case 200:
		cfg2 |= LEDBOOST_200;
		break;
	case 300:
		cfg2 |= LEDBOOST_300;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_CONFIG_2] = cfg2;
	chip->params.als_prox_cfg2 = cfg2;
	tmg399x_i2c_write(chip, TMG399X_CONFIG_2,
		chip->shadow[TMG399X_CONFIG_2]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_sat_irq_en_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.als_prox_cfg2 & 0x80) >> 7);
}

static ssize_t tmg399x_sat_irq_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool psien;
	int rc;

	rc = strtobool(buf, &psien);
	if (rc)
		return -EINVAL;

	mutex_lock(&chip->lock);
	chip->shadow[TMG399X_CONFIG_2] &= 0x7F;
	if (psien)
		chip->shadow[TMG399X_CONFIG_2] |= PSIEN;
	chip->params.als_prox_cfg2 = chip->shadow[TMG399X_CONFIG_2];
	tmg399x_i2c_write(chip, TMG399X_CONFIG_2, chip->shadow[TMG399X_CONFIG_2]);
	mutex_unlock(&chip->lock);
	return size;
}
static ssize_t tmg399x_prox_init_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	static long value;
	int ret,err;
	ret = kstrtol(buf, 10, &value);
	if(ret)
		return -EINVAL;
	if (value==1)
    {
		mutex_lock(&chip->lock);
    		if((ret=tmg399x_read_cal_value(CAL_THRESHOLD))<0)
		{
		SENSOR_LOG_ERROR("tmg399x_prox_init<0\n");
		err=tmg399x_write_cal_file(CAL_THRESHOLD,0);
			if(err<0)
			{
				SENSOR_LOG_ERROR("ERROR=%s\n",CAL_THRESHOLD);
				mutex_unlock(&chip->lock);
				return -EINVAL;
			}
		chip->prox_calibrate_times = 5;	
	       schedule_delayed_work(&p_global_tmg399x_chip->prox_calibrate_work, msecs_to_jiffies(1000));
		}
		else if (ret==0){
		chip->prox_calibrate_times = 5;	
	       schedule_delayed_work(&p_global_tmg399x_chip->prox_calibrate_work, msecs_to_jiffies(1000));
		SENSOR_LOG_ERROR("tmg399x_prox_calibrate==1\n");
		}
		else if(ret > 0){
	      chip->prox_manual_calibrate_threshold = ret;
            chip->params.prox_th_high = ret;
            chip->params.prox_th_low  = ret - 20;
		input_report_rel(chip->p_idev, REL_Y, chip->params.prox_th_high);
		input_report_rel(chip->p_idev, REL_Z, chip->params.prox_th_low);
		input_sync(chip->p_idev);
		SENSOR_LOG_ERROR("tmg399x_prox_init> 0\n");
		}
		mutex_unlock(&chip->lock);
	}
	else {
		SENSOR_LOG_ERROR("ERROR=tmg399x_prox_init_store\n");
		return -EINVAL;
	}
    SENSOR_LOG_ERROR("prox_th_high  = %d\n",chip->params.prox_th_high);
    SENSOR_LOG_ERROR("prox_th_low   = %d\n",chip->params.prox_th_low);
	return size;
}

static ssize_t tmg399x_prox_offset_ne_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			chip->params.prox_offset_ne);
}

static ssize_t tmg399x_prox_offset_ne_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	long offset_ne;
	int ret;
	u8 offset = 0;

	ret = kstrtol(buf, 10, &offset_ne);
	if (ret) {
		return -EINVAL;
    }
	if (offset_ne > 127 || offset_ne < -127) {
		dev_err(&chip->client->dev, "prox offset range [-127, 127]\n");
		return -EINVAL;
	}
	if (offset_ne < 0)
		offset = 128 - offset_ne;
	else
		offset = offset_ne;
	mutex_lock(&chip->lock);
	ret = tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_NE, offset);
	if (!ret) {
		chip->params.prox_offset_ne = (s8)offset_ne;
		chip->shadow[TMG399X_PRX_OFFSET_NE] = offset;
	}
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_offset_sw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			chip->params.prox_offset_sw);
}

static ssize_t tmg399x_prox_offset_sw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	long offset_sw;
	int ret;
	u8 offset = 0;

	ret = kstrtol(buf, 10, &offset_sw);
	if (ret) {
		return -EINVAL;
    }
	if (offset_sw > 127 || offset_sw < -127) {
		dev_err(&chip->client->dev, "prox offset range [-127, 127]\n");
		return -EINVAL;
	}
	if (offset_sw < 0)
		offset = 128 - offset_sw;
	else
		offset = offset_sw;
	mutex_lock(&chip->lock);
	ret = tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_SW, offset);
	if (!ret) {
		chip->params.prox_offset_sw = (s8)offset_sw;
		chip->shadow[TMG399X_PRX_OFFSET_SW] = offset;
	}
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_thres_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	static long value;
	int rc;

	rc = kstrtol(buf, 10, &value);
	if (rc)
		return -EINVAL;
	if (value==1){
   		if( (rc=tmg399x_read_cal_value(CAL_THRESHOLD))<0)
			return -EINVAL;
		else{
			mutex_lock(&chip->lock);
				if(rc > 30){
					chip->prox_manual_calibrate_threshold = rc;
			             chip->params.prox_th_high = rc;
			             chip->params.prox_th_low  = rc - 20;
					input_report_rel(chip->p_idev, REL_Y, chip->params.prox_th_high);
					input_report_rel(chip->p_idev, REL_Z, chip->params.prox_th_low);
					input_sync(chip->p_idev);
					mutex_unlock(&chip->lock);
					SENSOR_LOG_ERROR("prox_th_high  = %d\n",chip->params.prox_th_high);
			    		SENSOR_LOG_ERROR("prox_th_low   = %d\n",chip->params.prox_th_low);
					}
	 
			}
	}
	else{		
		return -EINVAL;
	}
	return size;
}


static ssize_t tmg399x_prox_thres_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

    SENSOR_LOG_ERROR("prox_th_high  = %d\n",chip->params.prox_th_high);
    SENSOR_LOG_ERROR("prox_th_low   = %d\n",chip->params.prox_th_low);

	return snprintf(buf, PAGE_SIZE, "prox_th_high  = %d, prox_th_low = %d\n", 
                                                   chip->params.prox_th_high,
                                                   chip->params.prox_th_low);
}



static ssize_t tmg399x_gesture_offset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

    u8 ges_offset_N = 0;
    u8 ges_offset_S = 0;
    u8 ges_offset_W = 0;
    u8 ges_offset_E = 0;

	mutex_lock(&chip->lock);    


        
	tmg399x_i2c_read(chip, TMG399X_GES_OFFSET_N, &ges_offset_N);
	tmg399x_i2c_read(chip, TMG399X_GES_OFFSET_S, &ges_offset_S);
	tmg399x_i2c_read(chip, TMG399X_GES_OFFSET_W, &ges_offset_W);
	tmg399x_i2c_read(chip, TMG399X_GES_OFFSET_E, &ges_offset_E);

    SENSOR_LOG_INFO("N = %X\n", (unsigned int)ges_offset_N);
    SENSOR_LOG_INFO("S = %X\n", (unsigned int)ges_offset_S);
    SENSOR_LOG_INFO("W = %X\n", (unsigned int)ges_offset_W);
    SENSOR_LOG_INFO("E = %X\n", (unsigned int)ges_offset_E);

	mutex_unlock(&chip->lock);  

	return sizeof(buf);

//	return snprintf(buf, PAGE_SIZE, "%d\n", chip->params.prox_offset_sw);
}

//add by zhubing
static ssize_t tmg399x_gesture_offset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	long offset;
	int rc;
    mutex_lock(&chip->lock);
    SENSOR_LOG_ERROR("Enter!\n");
	rc = kstrtol(buf, 10, &offset);
	if (rc)
		return -EINVAL;

    SENSOR_LOG_ERROR("offset = %X\n", (unsigned int)offset);

    switch( (offset>>8) & 0x0f)
    {
        case 1<<0: // E
        {            
            SENSOR_LOG_ERROR("Set E offset = %X\n", (u8)(offset & 0xff));
            rc = tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_E, (u8)(offset & 0xff));
            break;
        }

        case 1<<1: // W
        {
            SENSOR_LOG_ERROR("Set W offset = %X\n", (u8)(offset & 0xff));
            rc = tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_W, (u8)(offset & 0xff));
            break;
        }

        case 1<<2: // S
        {
            SENSOR_LOG_ERROR("Set S offset = %X\n", (u8)(offset & 0xff));
            rc = tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_S, (u8)(offset & 0xff));
            break;
        }

        case 1<<3: // N
        {
            SENSOR_LOG_ERROR("Set N offset = %X\n", (u8)(offset & 0xff));
            rc = tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_N, (u8)(offset & 0xff));
            break;
        }

        default:
        {
            
            SENSOR_LOG_ERROR("error!\n");
            break;
        }
    }

    SENSOR_LOG_ERROR("Exit!\n");
    mutex_unlock(&chip->lock);

	return size;
}

static ssize_t tmg399x_gesture_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);    
    u8 ret = 0;

    wait_event_interruptible(gesture_drdy_wq, (atomic_read(&gesture_drdy) == 1));

    mutex_lock(&chip->lock);

    ret = tmg399x_get_ges(chip);
    if (ret>0)
    {
        memcpy(buf, &(chip->gesture_data[0]), ret * sizeof(struct tmg399x_ges_raw_data));
    }

    tmg399x_i2c_modify(chip, TMG399X_GES_CFG_4, TMG399X_GES_INT_CLR, TMG399X_GES_INT_CLR);
    
    atomic_set(&gesture_drdy, 0);

    mutex_unlock(&chip->lock);

	return (sizeof(struct tmg399x_ges_raw_data) * ret);
}


static ssize_t tmg399x_gesture_data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
    struct sGesture_Result Gesture_Result;

    memcpy(&Gesture_Result, buf, sizeof(struct sGesture_Result));

    input_report_rel(chip->g_idev, REL_RX,      Gesture_Result.gesture_style);
    input_report_rel(chip->g_idev, REL_RY,      Gesture_Result.enter_time);
    input_report_rel(chip->g_idev, REL_RZ,      Gesture_Result.exit_time);
    input_report_rel(chip->g_idev, REL_HWHEEL,  Gesture_Result.enter_angle);
    input_report_rel(chip->g_idev, REL_DIAL,    Gesture_Result.exit_angle);
    input_sync(chip->g_idev);

    if (true==chip->ges_debug_enable)
    {
        SENSOR_LOG_INFO("gesture_style = %d, enter_time = %ld, exit_time = %ld, enter_angle = %d, exit_angle = %d",
                                                                                    Gesture_Result.gesture_style,
                                                                                    Gesture_Result.enter_time,
                                                                                    Gesture_Result.exit_time,
                                                                                    Gesture_Result.enter_angle,
                                                                                    Gesture_Result.exit_angle);
    }
	return size;
}



static ssize_t tmg399x_prox_mask_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%.2x\n",
			chip->params.als_prox_cfg3 & 0x0F);
}

static ssize_t tmg399x_prox_mask_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long prx_mask;
	int ret;

	ret = kstrtol(buf, 10, &prx_mask);
	if (ret) {
		return -EINVAL;
    }
	if (prx_mask > 15) {
		dev_err(&chip->client->dev, "prox mask range [0, 15]\n");
		return -EINVAL;
	}
	if ((prx_mask >> 3) ^ ((prx_mask >> 2) & 0x01) ||
		((prx_mask >> 1) & 0x01) ^ (prx_mask & 0x01))
		prx_mask |= PCMP;

	mutex_unlock(&chip->lock);
	chip->shadow[TMG399X_CONFIG_3] &= 0xD0;
	chip->shadow[TMG399X_CONFIG_3] |= (u8)prx_mask;
	chip->params.als_prox_cfg3 = chip->shadow[TMG399X_CONFIG_3];
	tmg399x_i2c_write(chip, TMG399X_CONFIG_3,
		chip->shadow[TMG399X_CONFIG_3]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_device_prx_raw(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
    mutex_lock(&chip->lock);
    tmg399x_get_prox(chip);
    mutex_unlock(&chip->lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.raw);
}

static ssize_t tmg399x_device_prx_detected(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
    mutex_lock(&chip->lock);
    tmg399x_get_prox(chip);
    mutex_unlock(&chip->lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
}


static ssize_t tmg399x_prox_calibrate_store(struct device *dev, struct device_attribute *attr, 
                                            const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
    int ret = kstrtouint(buf, 10, &(chip->prox_calibrate_times));
    if (ret) 
    {
        SENSOR_LOG_ERROR("input error\n");
        return -EINVAL;
    }

    mutex_lock(&chip->lock);
    tmg399x_prox_calibrate(chip);
    mutex_unlock(&chip->lock);
	return size;
}



static ssize_t tmg399x_prox_calibrate_start_store(struct device *dev, struct device_attribute *attr, 
                                            const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned int recv;
    int ret = kstrtouint(buf, 10, &recv);
    if (ret) 
    {
        SENSOR_LOG_ERROR("input error\n");
        return -EINVAL;
    }

    mutex_lock(&chip->lock);
    if (recv)
    {
        chip->prox_calibrate_start = true;
        SENSOR_LOG_INFO("enable prox calibrate\n");
    }
    else
    {
        chip->prox_calibrate_start = false;
        SENSOR_LOG_INFO("disable prox calibrate\n");
    }
    mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_calibrate_start_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
    SENSOR_LOG_INFO("prox calibrate is %s\n",chip->prox_calibrate_start ? "enable" : "disable");
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prox_calibrate_start);
}


static ssize_t tmg399x_phone_is_sleep_store(struct device *dev, struct device_attribute *attr, 
                                            const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned int recv;
    int ret = kstrtouint(buf, 10, &recv);
    if (ret) 
    {
        SENSOR_LOG_ERROR("input error\n");
        return -EINVAL;
    }

    mutex_lock(&chip->lock);
    if (recv==chip->phone_is_sleep)
    {
        SENSOR_LOG_INFO("double %s phone_is_sleep\n",recv? "enable" : "false");
    }
    else
    {        
        chip->phone_is_sleep = recv;
        SENSOR_LOG_INFO("success %s phone_is_sleep\n",recv? "enable" : "false");
    }
    mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_phone_is_sleep_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
    SENSOR_LOG_INFO("phone_is_sleep is %s\n",chip->phone_is_sleep? "true" : "false");
	return snprintf(buf, PAGE_SIZE, "phone_is_sleep %s\n\n", chip->phone_is_sleep? "true" : "false");
}

static ssize_t tmg399x_prox_wakelock_store(struct device *dev, struct device_attribute *attr, 
                                            const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned int recv;
    int ret = kstrtouint(buf, 10, &recv);
    if (ret) 
    {
        SENSOR_LOG_ERROR("input error\n");
        return -EINVAL;
    }

    mutex_lock(&chip->lock);
    if (recv)
    {
        tmg399x_wakelock_ops(&(chip->proximity_wakelock),true);
    }
    else
    {
        //cancel_delayed_work_sync(&p_global_tmg399x_chip->prox_unwakelock_work);
        hrtimer_cancel(&p_global_tmg399x_chip->prox_unwakelock_timer);
        tmg399x_wakelock_ops(&(chip->proximity_wakelock),false);
    }
    mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_wakelock_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
    SENSOR_LOG_INFO("proximity_wakelock is %s\n",chip->proximity_wakelock.locked ? "true" : "false");
	return snprintf(buf, PAGE_SIZE, "proximity_wakelock is %s\n",chip->proximity_wakelock.locked ? "true" : "false");
}


static ssize_t tmg399x_lux_table_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	struct lux_segment *s = chip->segment;
	int i, k;

	for (i = k = 0; i < chip->segment_num; i++)
		k += snprintf(buf + k, PAGE_SIZE - k,
				"%d:%d,%d,%d,%d,%d,%d\n", i,
				s[i].d_factor,
				s[i].r_coef,
				s[i].g_coef,
				s[i].b_coef,
				s[i].ct_coef,
				s[i].ct_offset
				);
	return k;
}

static ssize_t tmg399x_lux_table_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i;
	u32 d_factor, r_coef, g_coef, b_coef, ct_coef, ct_offset;

	if (7 != sscanf(buf, "%10d:%10d,%10d,%10d,%10d,%10d,%10d",
		&i, &d_factor, &r_coef, &g_coef, &b_coef, &ct_coef, &ct_offset))
		return -EINVAL;
	if (i >= chip->segment_num)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->segment[i].d_factor = d_factor;
	chip->segment[i].r_coef = r_coef;
	chip->segment[i].g_coef = g_coef;
	chip->segment[i].b_coef = b_coef;
	chip->segment[i].ct_coef = ct_coef;
	chip->segment[i].ct_offset = ct_offset;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_deltaP_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE,
			"%d (in %%)\n", chip->params.als_deltaP);
}

static ssize_t tmg399x_als_deltaP_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long deltaP;
	int ret;
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 10, &deltaP);
	if (ret || deltaP > 100) 
    {
		return -EINVAL;
    }
	mutex_lock(&chip->lock);
	chip->params.als_deltaP = deltaP;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_auto_gain_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
				chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmg399x_auto_gain_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
	mutex_lock(&chip->lock);

	if (value)
		chip->als_gain_auto = true;
	else
		chip->als_gain_auto = false;

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_device_als_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	mutex_lock(&chip->lock);
	tmg399x_get_als(chip);
	tmg399x_get_lux(chip);
	mutex_unlock(&chip->lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}

static ssize_t tmg399x_als_red_show(struct device *dev,
	struct device_attribute *attr, char *buf)
		{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.red_raw);
}

static ssize_t tmg399x_als_green_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.green_raw);
}

static ssize_t tmg399x_als_blue_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.blue_raw);
}

static ssize_t tmg399x_als_clear_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.clear_raw);
}

static ssize_t tmg399x_als_cct_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	mutex_lock(&chip->lock);
	tmg399x_read_all(chip);
	tmg399x_get_als(chip);
	tmg399x_get_lux(chip);
	mutex_unlock(&chip->lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cct);
}



static ssize_t tmg399x_set_reg_addr(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val;
    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }

    reg_addr = val;

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t tmg399x_get_reg_addr(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
   
    SENSOR_LOG_ERROR("enter\n");
    SENSOR_LOG_ERROR("reg_addr = 0x%02X\n",reg_addr);
	return strlen(buf);
    SENSOR_LOG_ERROR("exit\n");

}


static ssize_t tmg399x_set_reg_data(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val;
    int ret;
    mutex_lock(&p_global_tmg399x_chip->lock);

    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }
    
    if (100==reg_addr)
    {
        SENSOR_LOG_ERROR("reg addr error!\n");
    }
    else
    {
        if ((ret = (i2c_smbus_write_byte_data(p_global_tmg399x_chip->client, reg_addr, val))) < 0)
        {
            SENSOR_LOG_ERROR("failed write reg\n");
        }   
    }

    SENSOR_LOG_ERROR("exit\n");
    mutex_unlock(&p_global_tmg399x_chip->lock);

	return size;
}


static ssize_t tmg399x_get_reg_data(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    int i;

    mutex_lock(&p_global_tmg399x_chip->lock);

    if (1 == reg_addr)
    {
        for (i=0x80; i<=0xFF; i++)
        {
            i2c_smbus_write_byte(p_global_tmg399x_chip->client, i);
            SENSOR_LOG_ERROR("reg[0x%02X] = 0x%02X",i,i2c_smbus_read_byte(p_global_tmg399x_chip->client));
        }
    }   
    else
    {
        i2c_smbus_write_byte(p_global_tmg399x_chip->client, reg_addr);
        SENSOR_LOG_ERROR("reg[0x%02X] = 0x%02X",reg_addr,i2c_smbus_read_byte(p_global_tmg399x_chip->client)); 
    }

    mutex_unlock(&p_global_tmg399x_chip->lock);

	return strlen(buf);
}


static ssize_t tmg399x_irq_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    SENSOR_LOG_ERROR("irq_enabled  = %d\n",p_global_tmg399x_chip->irq_enabled );
	return snprintf(buf, PAGE_SIZE, "irq_enabled  = %d\n",p_global_tmg399x_chip->irq_enabled );
}

static ssize_t tmg399x_irq_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val;

    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }
    
    if (val)
    {
        tmg399x_irq_enable(true, true);
    }
    else
    {
        tmg399x_irq_enable(false, true);
    }
 
	return size;
}


static ssize_t tmg399x_irq_clear(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val;

    SENSOR_LOG_ERROR("enter\n");

    mutex_lock(&p_global_tmg399x_chip->lock);

    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }
    
    if (1==val)
    {
        SENSOR_LOG_ERROR("CLEAR ALS INT!\n");
        tmg399x_irq_clr(p_global_tmg399x_chip, TMG399X_CMD_ALS_INT_CLR);
    }
    else
    {
        if (2==val)
        {
            SENSOR_LOG_ERROR("CLEAR PROX INT!\n");
            tmg399x_irq_clr(p_global_tmg399x_chip, TMG399X_CMD_PROX_INT_CLR);
        }
        else
        {
            if (3==val)
            {
                SENSOR_LOG_ERROR("CLEAR GESTURE INT!\n");
                tmg399x_i2c_modify(p_global_tmg399x_chip, TMG399X_GES_CFG_4, TMG399X_GES_INT_CLR, TMG399X_GES_INT_CLR);
            }
            else
            {
                if (4==val)
                {
                    SENSOR_LOG_ERROR("CLEAR ALL INT!\n");
                    tmg399x_irq_clr(p_global_tmg399x_chip, TMG399X_CMD_NON_GES_INT_CLR);
                }
            }
        }
    }

    mutex_unlock(&p_global_tmg399x_chip->lock);

    SENSOR_LOG_ERROR("exit\n");

	return size;
}


static ssize_t tmg399x_set_ges_enter_thres(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val;

    SENSOR_LOG_ERROR("enter\n");

    mutex_lock(&p_global_tmg399x_chip->lock);

    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }
    
    tmg399x_data.parameters.ges_entry_th = (val & 0xff);
	p_global_tmg399x_chip->shadow[TMG399X_GES_ENTH]  = p_global_tmg399x_chip->params.ges_entry_th;
	tmg399x_i2c_write(p_global_tmg399x_chip, TMG399X_GES_ENTH, p_global_tmg399x_chip->shadow[TMG399X_GES_ENTH]);

    mutex_unlock(&p_global_tmg399x_chip->lock);

    SENSOR_LOG_ERROR("exit\n");

	return size;
}


static ssize_t tmg399x_get_ges_enter_thres(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    SENSOR_LOG_ERROR("enter\n");

    mutex_lock(&p_global_tmg399x_chip->lock);
    
    SENSOR_LOG_ERROR("ges_entry_th = %d\n",tmg399x_data.parameters.ges_entry_th);

    mutex_unlock(&p_global_tmg399x_chip->lock);

    SENSOR_LOG_ERROR("exit\n");

	return strlen(buf);
}

static ssize_t tmg399x_set_ges_exit_thres(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val;

    SENSOR_LOG_ERROR("enter\n");

    mutex_lock(&p_global_tmg399x_chip->lock);

    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }
    
    tmg399x_data.parameters.ges_exit_th = (val & 0xff);
	p_global_tmg399x_chip->shadow[TMG399X_GES_EXTH]  = p_global_tmg399x_chip->params.ges_exit_th;
	tmg399x_i2c_write(p_global_tmg399x_chip, TMG399X_GES_EXTH, p_global_tmg399x_chip->shadow[TMG399X_GES_EXTH]);

    mutex_unlock(&p_global_tmg399x_chip->lock);

    SENSOR_LOG_ERROR("exit\n");

	return size;
}


static ssize_t tmg399x_get_ges_exit_thres(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    SENSOR_LOG_ERROR("enter\n");

    mutex_lock(&p_global_tmg399x_chip->lock);
    
    SENSOR_LOG_ERROR("ges_exit_th = %d\n",tmg399x_data.parameters.ges_exit_th);

    mutex_unlock(&p_global_tmg399x_chip->lock);

    SENSOR_LOG_ERROR("exit\n");

	return strlen(buf);
}

static ssize_t tmg399x_set_ges_start(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val;

    mutex_lock(&p_global_tmg399x_chip->lock);

    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }
    
    if (val)
    {
        p_global_tmg399x_chip->gesture_start = true;
    }
    else
    {
        p_global_tmg399x_chip->gesture_start = false;
    }
    mutex_unlock(&p_global_tmg399x_chip->lock);

	return size;
}

static ssize_t tmg399x_prox_threshold_high_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev); 

	if (NULL!=chip)
    {
        return sprintf(buf, "%d", chip->params.prox_th_high);
    }
    else
    {       
        sprintf(buf, "chip->params.prox_th_high is NULL\n");
    }
	return strlen(buf);
}

static ssize_t tmg399x_prox_threshold_high_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	static long value;
	int rc;

	rc = kstrtol(buf, 10, &value);
	if (rc)
		return -EINVAL;

    chip->params.prox_th_high = (u8)(value&0xff);

	mutex_lock(&chip->lock);
	chip->shadow[TMG399X_PRX_THRES_HIGH] = chip->params.prox_th_high;
    /*
	tmg399x_i2c_write(chip, TMG399X_PRX_THRES_HIGH, chip->shadow[TMG399X_PRX_THRES_HIGH]);
	*/
    mutex_unlock(&chip->lock);

    SENSOR_LOG_ERROR("prox_th_high = %d\n",chip->params.prox_th_high);

	return size;
}



static ssize_t tmg399x_prox_threshold_low_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev); 

	if (NULL!=chip)
    {
        return sprintf(buf, "%d", chip->params.prox_th_low);
    }
    else
    {       
        sprintf(buf, "chip->params.prox_th_low is NULL\n");
    }
	return strlen(buf);
}

static ssize_t tmg399x_prox_threshold_low_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	static long value;
	int rc;

	rc = kstrtol(buf, 10, &value);
	if (rc)
		return -EINVAL;

    chip->params.prox_th_low = (u8)(value&0xff);

	mutex_lock(&chip->lock);
	chip->shadow[TMG399X_PRX_THRES_LOW] = chip->params.prox_th_low;
    /*
	tmg399x_i2c_write(chip, TMG399X_PRX_THRES_LOW, chip->shadow[TMG399X_PRX_THRES_HIGH]);
	*/
    mutex_unlock(&chip->lock);

    SENSOR_LOG_ERROR("prox_th_low = %d\n",chip->params.prox_th_low);

	return size;
}



static struct device_attribute attrs_prox[] = {
    __ATTR(chip_name,                       0640,   tmg399x_chip_name_show,              NULL), 
	__ATTR(enable,                          0640,   tmg399x_prox_enable_show,            tmg399x_prox_enable_store),
	__ATTR(prox_init,                       0640,   NULL,                                tmg399x_prox_init_store),
	__ATTR(prox_persist,                    0640,   tmg399x_prox_persist_show,           tmg399x_prox_persist_store),
	__ATTR(prx_pulse_length,                0640,   tmg399x_prox_pulse_len_show,         tmg399x_prox_pulse_len_store),
	__ATTR(prox_pulse_count,                0640,   tmg399x_prox_pulse_cnt_show,	     tmg399x_prox_pulse_cnt_store),
	__ATTR(prox_gain,                       0640,   tmg399x_prox_gain_show,	             tmg399x_prox_gain_store),
	__ATTR(prox_led_drive,                  0640,   tmg399x_prox_led_drive_show,         tmg399x_prox_led_drive_store),
	__ATTR(prox_led_boost,                  0640,   tmg399x_led_boost_show,	             tmg399x_led_boost_store),
	__ATTR(prox_sat_irq_en,                 0640,   tmg399x_sat_irq_en_show,             tmg399x_sat_irq_en_store),
	__ATTR(prox_offset_ne,                  0640,   tmg399x_prox_offset_ne_show,         tmg399x_prox_offset_ne_store),
	__ATTR(prox_offset_sw,                  0640,   tmg399x_prox_offset_sw_show,         tmg399x_prox_offset_sw_store),
	__ATTR(prox_mask,                       0640,   tmg399x_prox_mask_show,              tmg399x_prox_mask_store),
	__ATTR(prox_raw,                        0640,   tmg399x_device_prx_raw,              NULL),
	__ATTR(prox_detect,                     0640,   tmg399x_device_prx_detected,         NULL),
    __ATTR(prox_calibrate,                  0640,   NULL,                                tmg399x_prox_calibrate_store), 
    __ATTR(prox_calibrate_start,            0640,   tmg399x_prox_calibrate_start_show,   tmg399x_prox_calibrate_start_store),
    __ATTR(prox_calibrate_result,           0640,   tmg399x_prox_calibrate_result_show,  NULL), 
    __ATTR(prox_thres,                      0640,   tmg399x_prox_thres_show,             tmg399x_prox_thres_store),
    __ATTR(prox_debug,                      0640,   tmg399x_prox_debug_show,             tmg399x_prox_debug_store),
    __ATTR(prox_phone_is_sleep,             0640,   tmg399x_phone_is_sleep_show,         tmg399x_phone_is_sleep_store),
    __ATTR(prox_wakelock,                   0640,   tmg399x_prox_wakelock_show,          tmg399x_prox_wakelock_store),
    __ATTR(prox_thres_max,                  0644,   tmg399x_prox_thres_hi_max,           NULL), 
    __ATTR(prox_thres_min,                  0644,   tmg399x_prox_thres_lo_min,           NULL), 
    __ATTR(prox_data_max,                   0640,   tmg399x_prox_data_max,               NULL),
    __ATTR(prox_manual_calibrate_threshold, 0644,   tmg399x_manual_calibrate_threshold,               NULL), 
    __ATTR(tmg_irq,                         0640,   tmg399x_irq_show,                    tmg399x_irq_store),
    __ATTR(tmg_reg_addr,                    0640,   tmg399x_get_reg_addr,                tmg399x_set_reg_addr),
    __ATTR(tmg_reg_data,                    0640,   tmg399x_get_reg_data,                tmg399x_set_reg_data),
    __ATTR(tmg_clear_irq,                   0640,   NULL,                                tmg399x_irq_clear),
    __ATTR(prox_threshold_high,             0644,   tmg399x_prox_threshold_high_show,    tmg399x_prox_threshold_high_store),
    __ATTR(prox_threshold_low,              0644,   tmg399x_prox_threshold_low_show,     tmg399x_prox_threshold_low_store),
};


static struct device_attribute attrs_gesture[] = {
    __ATTR(chip_name,                   0640,   tmg399x_chip_name_show,              NULL),
	__ATTR(enable,                      0640,   tmg399x_ges_enable_show,             tmg399x_ges_enable_store),
	__ATTR(gesture_pulse_length,        0640,   tmg399x_ges_pulse_len_show,          tmg399x_ges_pulse_len_store),
	__ATTR(gesture_pulse_count,         0640,   tmg399x_ges_pulse_cnt_show,	         tmg399x_ges_pulse_cnt_store),
	__ATTR(gesture_gain,                0640,   tmg399x_ges_gain_show,	             tmg399x_ges_gain_store),
    __ATTR(gesture_data,                0640,   tmg399x_gesture_data_show,           tmg399x_gesture_data_store),
    __ATTR(gesture_offset,              0640,   tmg399x_gesture_offset_show,         tmg399x_gesture_offset_store),
    __ATTR(gesture_enter_thres,         0640,   tmg399x_get_ges_enter_thres,         tmg399x_set_ges_enter_thres),
    __ATTR(gesture_exit_thres,          0640,   tmg399x_get_ges_exit_thres,          tmg399x_set_ges_exit_thres),
    __ATTR(gesture_start_flag,          0640,   NULL,                                tmg399x_set_ges_start),
    __ATTR(gesture_debug,               0640,   tmg399x_ges_debug_show,              tmg399x_ges_debug_store),
    __ATTR(led_boost,                   0640,   tmg399x_led_boost_show,              tmg399x_led_boost_store),
    __ATTR(reg_addr,                    0640,   tmg399x_get_reg_addr,                tmg399x_set_reg_addr),
    __ATTR(reg_data,                    0640,   tmg399x_get_reg_data,                tmg399x_set_reg_data),
    __ATTR(clear_irq,                   0640,   NULL,                                tmg399x_irq_clear),
};


static struct device_attribute attrs_light[] = {
    __ATTR(chip_name,               0640,   tmg399x_chip_name_show,              NULL), 
	__ATTR(enable,                  0640,   tmg399x_als_enable_show,             tmg399x_als_enable_store),
    __ATTR(delay,                   0640,   tmg399x_als_poll_time_show,          tmg399x_als_poll_time_store),
	__ATTR(light_wait_time_en,      0640,   tmg399x_wait_enable_show,            tmg399x_wait_enable_store),
	__ATTR(light_Itime,             0640,   tmg399x_als_itime_show,              tmg399x_als_itime_store),
	__ATTR(light_wait_time,         0640,   tmg399x_wait_time_show,              tmg399x_wait_time_store),
	__ATTR(light_persist,           0640,   tmg399x_als_persist_show,            tmg399x_als_persist_store),
	__ATTR(light_light_gain,        0640,   tmg399x_als_gain_show,               tmg399x_als_gain_store),
	__ATTR(light_lux_table,         0640,   tmg399x_lux_table_show,              tmg399x_lux_table_store),
	__ATTR(light_thresh_deltaP,     0640,   tmg399x_als_deltaP_show,             tmg399x_als_deltaP_store),
	__ATTR(light_auto_gain,         0640,   tmg399x_auto_gain_enable_show,       tmg399x_auto_gain_enable_store),
	__ATTR(light_lux,               0640,   tmg399x_device_als_lux,              NULL),
	__ATTR(light_red,               0640,   tmg399x_als_red_show,                NULL),
	__ATTR(light_green,             0640,   tmg399x_als_green_show,              NULL),
	__ATTR(light_blue,              0640,   tmg399x_als_blue_show,               NULL),
	__ATTR(light_clear,             0640,   tmg399x_als_clear_show,              NULL),
	__ATTR(light_cct,               0640,   tmg399x_als_cct_show,                NULL),
    __ATTR(light_debug,             0640,   tmg399x_light_debug_show,            tmg399x_light_debug_store),
};

int tmg399x_read_cal_value(char *file_path)
{
    struct file *file_p;
    int vfs_read_retval = 0;
    mm_segment_t old_fs; 
    char read_buf[32];
    unsigned short read_value;

    if (NULL==file_path)
    {
        SENSOR_LOG_ERROR("file_path is NULL\n");
        goto error;
    }

    memset(read_buf, 0, 32);

    file_p = filp_open(file_path, O_RDONLY , 0);
    if (IS_ERR(file_p))
    {
        SENSOR_LOG_ERROR("[open file <%s>failed]\n",file_path);
        goto error;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    
    vfs_read_retval = vfs_read(file_p, (char*)read_buf, 16, &file_p->f_pos);
    if (vfs_read_retval < 0)
    {
        SENSOR_LOG_ERROR("[read file <%s>failed]\n",file_path);
        goto error;
    }

    set_fs(old_fs);
    filp_close(file_p, NULL);

    if (kstrtou16(read_buf, 10, &read_value) < 0)
    {
        SENSOR_LOG_ERROR("[kstrtou16 %s failed]\n",read_buf);
        goto error;
    }
    
    SENSOR_LOG_ERROR("[the content of %s is %s]\n", file_path, read_buf);

    return read_value;

error:
    return -1;
}

int tmg399x_write_cal_file(char *file_path,unsigned int value)
{
    struct file *file_p;
    char write_buf[10];
	 mm_segment_t old_fs; 
    int vfs_write_retval=0;
    if (NULL==file_path)
    {
        SENSOR_LOG_ERROR("file_path is NULL\n");
      
    }
       memset(write_buf, 0, sizeof(write_buf));
      sprintf(write_buf, "%d\n", value);
    file_p = filp_open(file_path, O_CREAT|O_RDWR , 0665);
    if (IS_ERR(file_p))
    {
        SENSOR_LOG_ERROR("[open file <%s>failed]\n",file_path);
        goto error;
    }
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    
    vfs_write_retval = vfs_write(file_p, (char*)write_buf, sizeof(write_buf), &file_p->f_pos);
    if (vfs_write_retval < 0)
    {
        SENSOR_LOG_ERROR("[write file <%s>failed]\n",file_path);
        goto error;
    }

    set_fs(old_fs);
    filp_close(file_p, NULL);


    return 1;

error:
    return -1;
}

static int create_sysfs_interfaces_prox(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attrs_prox); i++)
		if (device_create_file(dev, attrs_prox + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attrs_prox + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int create_sysfs_interfaces_light(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attrs_light); i++)
		if (device_create_file(dev, attrs_light + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attrs_light + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int create_sysfs_interfaces_gesture(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attrs_gesture); i++)
		if (device_create_file(dev, attrs_gesture + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attrs_gesture + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int tmg399x_check_chip_ready(struct tmg399x_chip *chip)
{
	u8 id, rev;
    return tmg399x_get_id(chip, &id, &rev);
}

static int tmg399x_wait_chip_ready(struct tmg399x_chip *chip)
{
    int i = 0;
    msleep(50);
    for (i=1; i<100; i++)
    {
        if (tmg399x_check_chip_ready(chip) < 0)
        {
            msleep(10);
        }
        else
        {
            SENSOR_LOG_INFO("retry %d times\n",(i-1));
            return 0;
        }
    }

    SENSOR_LOG_INFO("retry times out \n");
    return -1;
}


static void tmg399x_input_far_event(struct tmg399x_chip *chip)
{
    input_report_rel(chip->p_idev, REL_X, 250);
    input_sync(chip->p_idev);
}

static void tmg399x_report_prox(struct tmg399x_chip *chip)
{
	if (chip->p_idev) 
    {
        SENSOR_LOG_INFO("data = %d, high = %d, low = %d",chip->prx_inf.raw,chip->params.prox_th_high, chip->params.prox_th_low);
        if (chip->prox_calibrate_start)
        {
            input_report_rel(chip->p_idev, REL_MISC, chip->prx_inf.raw);
        }
        else
        {
            input_report_rel(chip->p_idev, REL_X, chip->prx_inf.raw);
        }
		input_sync(chip->p_idev);
	}
}

static int tmg399x_check_and_report(struct tmg399x_chip *chip)
{   
	u8 status;
	
    tmg399x_i2c_read(chip, TMG399X_STATUS, &chip->shadow[TMG399X_STATUS]);
	status = chip->shadow[TMG399X_STATUS];

    if (true == chip->prx_enabled)
    {
        tmg399x_read_prox_data(chip);
        if (tmg399x_get_prox(chip))
        {
            tmg399x_report_prox(chip);
        }
        tmg399x_irq_clr(chip, TMG399X_CMD_PROX_INT_CLR);
    }

    if ((status & (TMG399X_ST_GES_IRQ)) == (TMG399X_ST_GES_IRQ)) 
    {
        atomic_set(&gesture_drdy, 1);
        wake_up(&gesture_drdy_wq);
    }
    
	return 0;
}

static void tmg399x_irq_work(struct work_struct *work)
{
	struct tmg399x_chip *chip = container_of(work, struct tmg399x_chip, irq_work);
    mutex_lock(&chip->lock);
    if (true == chip->wakeup_from_sleep)
    {        
        chip->wakeup_from_sleep = false;
        SENSOR_LOG_INFO(" wakeup_from_sleep = true\n");
        if (tmg399x_wait_chip_ready(chip) < 0)
        {
            tmg399x_input_far_event(chip);
        }
    }
	tmg399x_check_and_report(chip);
	hrtimer_cancel(&p_global_tmg399x_chip->prox_unwakelock_timer);
	p_global_tmg399x_chip->irq_work_status = false;
	//SENSOR_LOG_INFO("########  tmg399x_irq_work enter   hrtimer_start #########\n");

	hrtimer_start(&p_global_tmg399x_chip->prox_unwakelock_timer, ktime_set(3, 0), HRTIMER_MODE_REL);

	//schedule_delayed_work(&p_global_tmg399x_chip->prox_unwakelock_work, msecs_to_jiffies(1000));
	
    tmg399x_irq_enable(true, true);
    mutex_unlock(&chip->lock);
};

static irqreturn_t tmg399x_irq(int irq, void *handle)
{
	struct tmg399x_chip *chip = handle;
    chip->irq_work_status =true;
    tmg399x_irq_enable(false, false);
    if (true == chip->prx_enabled)
    {
        tmg399x_wakelock_ops(&(chip->proximity_wakelock),true);
	}
    if (0==schedule_work(&chip->irq_work))
    {
        SENSOR_LOG_INFO("schedule_work failed!\n");
    }
	return IRQ_HANDLED;
}

static int tmg399x_set_segment_table(struct tmg399x_chip *chip,
		struct lux_segment *segment, int seg_num)
{
	int i;
	struct device *dev = &chip->client->dev;

	chip->seg_num_max = chip->pdata->segment_num ?
			chip->pdata->segment_num : ARRAY_SIZE(segment_default);

	if (!chip->segment) 
    {
		dev_info(dev, "%s: allocating segment table\n", __func__);
		chip->segment = kzalloc(sizeof(*chip->segment) *
				chip->seg_num_max, GFP_KERNEL);
		if (!chip->segment) {
			dev_err(dev, "%s: no memory!\n", __func__);
			return -ENOMEM;
		}
	}
	if (seg_num > chip->seg_num_max) {
		dev_warn(dev, "%s: %d segment requested, %d applied\n",
				__func__, seg_num, chip->seg_num_max);
		chip->segment_num = chip->seg_num_max;
	} else {
		chip->segment_num = seg_num;
	}
	memcpy(chip->segment, segment,
			chip->segment_num * sizeof(*chip->segment));
	dev_info(dev, "%s: %d segment requested, %d applied\n", __func__,
			seg_num, chip->seg_num_max);
	for (i = 0; i < chip->segment_num; i++)
		dev_info(dev,
		"seg %d: d_factor %d, r_coef %d, g_coef %d, b_coef %d, ct_coef %d ct_offset %d\n",
		i, chip->segment[i].d_factor, chip->segment[i].r_coef,
		chip->segment[i].g_coef, chip->segment[i].b_coef,
		chip->segment[i].ct_coef, chip->segment[i].ct_offset);
	return 0;
}

static void tmg399x_set_defaults(struct tmg399x_chip *chip)
{
	struct device *dev = &chip->client->dev;

	if (chip->pdata) {
		dev_info(dev, "%s: Loading pltform data\n", __func__);
		chip->params.als_time = chip->pdata->parameters.als_time;
		chip->params.als_gain = chip->pdata->parameters.als_gain;
		chip->params.als_deltaP = chip->pdata->parameters.als_deltaP;
		chip->params.wait_time = chip->pdata->parameters.wait_time;
		chip->params.prox_th_low = chip->pdata->parameters.prox_th_low;
		chip->params.prox_th_high = chip->pdata->parameters.prox_th_high;
		chip->params.persist = chip->pdata->parameters.persist;
		chip->params.als_prox_cfg1 = chip->pdata->parameters.als_prox_cfg1;
		chip->params.prox_pulse = chip->pdata->parameters.prox_pulse;
		chip->params.prox_gain = chip->pdata->parameters.prox_gain;
		chip->params.ldrive = chip->pdata->parameters.ldrive;
		chip->params.als_prox_cfg2 = chip->pdata->parameters.als_prox_cfg2;
		chip->params.prox_offset_ne = chip->pdata->parameters.prox_offset_ne;
		chip->params.prox_offset_sw = chip->pdata->parameters.prox_offset_sw;
		chip->params.als_prox_cfg3 = chip->pdata->parameters.als_prox_cfg3;
	} else {
		dev_info(dev, "%s: use defaults\n", __func__);
		chip->params.als_time = param_default.als_time;
		chip->params.als_gain = param_default.als_gain;
		chip->params.als_deltaP = param_default.als_deltaP;
		chip->params.wait_time = param_default.wait_time;
		chip->params.prox_th_low = param_default.prox_th_low;
		chip->params.prox_th_high = param_default.prox_th_high;
		chip->params.persist = param_default.persist;
		chip->params.als_prox_cfg1 = param_default.als_prox_cfg1;
		chip->params.prox_pulse = param_default.prox_pulse;
		chip->params.prox_gain = param_default.prox_gain;
		chip->params.ldrive = param_default.ldrive;
		chip->params.als_prox_cfg2 = param_default.als_prox_cfg2;
		chip->params.prox_offset_ne = param_default.prox_offset_ne;
		chip->params.prox_offset_sw = param_default.prox_offset_sw;
		chip->params.als_prox_cfg3 = param_default.als_prox_cfg3;
	}

	chip->als_gain_auto = true;

	/* Initial proximity threshold */
	chip->shadow[TMG399X_PRX_THRES_LOW] = 0;//chip->params.prox_th_low;
	chip->shadow[TMG399X_PRX_THRES_HIGH] = chip->params.prox_th_high;
	tmg399x_i2c_write(chip, TMG399X_PRX_THRES_LOW, chip->shadow[TMG399X_PRX_THRES_LOW]);
	tmg399x_i2c_write(chip, TMG399X_PRX_THRES_HIGH, chip->shadow[TMG399X_PRX_THRES_HIGH]);
			
	chip->shadow[TMG399X_ALS_TIME]      = chip->params.als_time;
	chip->shadow[TMG399X_WAIT_TIME]     = chip->params.wait_time;
	chip->shadow[TMG399X_PERSISTENCE]   = chip->params.persist;
	chip->shadow[TMG399X_CONFIG_1]      = chip->params.als_prox_cfg1;	
	chip->shadow[TMG399X_PRX_PULSE]     = chip->params.prox_pulse;
	chip->shadow[TMG399X_GAIN]          = chip->params.als_gain | chip->params.prox_gain | chip->params.ldrive;
	chip->shadow[TMG399X_CONFIG_2]      = chip->params.als_prox_cfg2;
	chip->shadow[TMG399X_PRX_OFFSET_NE] = chip->params.prox_offset_ne;
	chip->shadow[TMG399X_PRX_OFFSET_SW] = chip->params.prox_offset_sw;
	chip->shadow[TMG399X_CONFIG_3]      = chip->params.als_prox_cfg3;
}

static int tmg399x_get_id(struct tmg399x_chip *chip, u8 *id, u8 *rev)
{
	int ret;
	ret = tmg399x_i2c_read(chip, TMG399X_REVID, rev);
	ret |= tmg399x_i2c_read(chip, TMG399X_CHIPID, id);
	return ret;
}

static int tmg399x_add_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void tmg399x_remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}

static void tmg399x_als_poll_work_func(struct work_struct *work)
{
    int ret = 0;
    u8 temp = 0;

    if (p_global_tmg399x_chip->gesture_start)
    {
        if (true == p_global_tmg399x_chip->als_enabled)
        {
    	    schedule_delayed_work(&p_global_tmg399x_chip->als_poll_work, msecs_to_jiffies(p_global_tmg399x_chip->light_poll_time));
        }
        return;
    }

    mutex_lock(&p_global_tmg399x_chip->lock);

    if ((true==p_global_tmg399x_chip->ges_enabled) && (false==p_global_tmg399x_chip->prx_enabled))
    {
    
        ret = tmg399x_i2c_read(p_global_tmg399x_chip, TMG399X_CONTROL, &temp);

        temp &= ~TMG399X_EN_GES;
	    ret = tmg399x_i2c_write(p_global_tmg399x_chip, TMG399X_CONTROL, temp);

        msleep(10);
        
        ret = tmg399x_i2c_read(p_global_tmg399x_chip, TMG399X_CONTROL, &temp);

        temp |= TMG399X_EN_GES;
	    ret = tmg399x_i2c_write(p_global_tmg399x_chip, TMG399X_CONTROL, temp);


	    ret = tmg399x_i2c_read(p_global_tmg399x_chip, TMG399X_GES_CFG_4, &temp);
	    temp |= 0x01;
	    ret = tmg399x_i2c_write(p_global_tmg399x_chip, TMG399X_GES_CFG_4, temp);
    }

	tmg399x_read_rgb_data(p_global_tmg399x_chip);
    tmg399x_get_als(p_global_tmg399x_chip);
	tmg399x_get_lux(p_global_tmg399x_chip);
    if (p_global_tmg399x_chip->light_debug_enable)
    {
        SENSOR_LOG_INFO("R = %d, G = %d, B = %d, C = %d, Lux = %d cct = %d",
                                                                p_global_tmg399x_chip->als_inf.red_raw,
                                                                p_global_tmg399x_chip->als_inf.green_raw,
                                                                p_global_tmg399x_chip->als_inf.blue_raw,
                                                                p_global_tmg399x_chip->als_inf.clear_raw,
                                                                p_global_tmg399x_chip->als_inf.lux,
                                                                p_global_tmg399x_chip->als_inf.cct);
    }

    if (p_global_tmg399x_chip->als_inf.lux>10000)
    {
        p_global_tmg399x_chip->als_inf.lux = 10000;
    }
    input_report_rel(p_global_tmg399x_chip->a_idev, REL_X, (p_global_tmg399x_chip->als_inf.lux + 1));
    input_report_rel(p_global_tmg399x_chip->a_idev, REL_Y, p_global_tmg399x_chip->als_inf.cct);
    input_sync(p_global_tmg399x_chip->a_idev);

    if (true == p_global_tmg399x_chip->als_enabled)
    {
	    schedule_delayed_work(&p_global_tmg399x_chip->als_poll_work, msecs_to_jiffies(p_global_tmg399x_chip->light_poll_time));
    }

    mutex_unlock(&p_global_tmg399x_chip->lock);
}

static void tmg399x_prox_calibrate_work_func(struct work_struct *work)
{
    tmg399x_prox_calibrate(p_global_tmg399x_chip);
}

static enum hrtimer_restart tmg399x_unwakelock_work_func(struct hrtimer *timer)
{ 

   SENSOR_LOG_INFO("########  in  tmg399x_prox_unwakelock_timer_func #########\n");
   if (false == p_global_tmg399x_chip->irq_work_status )
   tmg399x_wakelock_ops(&(p_global_tmg399x_chip->proximity_wakelock),false);

   return HRTIMER_NORESTART;
}
static int __devinit tmg399x_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	int i, ret;
	u8 id, rev;
	struct device *dev = &client->dev;
	static struct tmg399x_chip *chip;
	struct tmg399x_i2c_platform_data *pdata = &tmg399x_data;
	bool powered = 0;
    SENSOR_LOG_INFO("Prob Start\n");

    dev->platform_data = &tmg399x_data;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}
	if (!pdata) {
		dev_err(dev, "%s: platform data required\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}

	if (!(pdata->prox_name || pdata->als_name) || client->irq < 0) {
		dev_err(dev, "%s: no reason to run.\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}

	if (pdata->platform_init) {
		ret = pdata->platform_init();
		if (ret)
			goto init_failed;
	}
	if (pdata->platform_power) {
		ret = pdata->platform_power(dev, POWER_ON);
		if (ret) {
			dev_err(dev, "%s: pltf power on failed\n", __func__);
			goto pon_failed;
		}
		powered = true;
		mdelay(10);
	}
	chip = kzalloc(sizeof(struct tmg399x_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}

    tmg399x_chip_data_init(chip);

    p_global_tmg399x_chip = chip;

	chip->client = client;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);

	chip->seg_num_max = chip->pdata->segment_num ?
			chip->pdata->segment_num : ARRAY_SIZE(segment_default);
	if (chip->pdata->segment)
		ret = tmg399x_set_segment_table(chip, chip->pdata->segment,
			chip->pdata->segment_num);
	else
		ret =  tmg399x_set_segment_table(chip, segment_default,
			ARRAY_SIZE(segment_default));
	if (ret)
		goto set_segment_failed;

	ret = tmg399x_get_id(chip, &id, &rev);
	if (ret < 0)
		dev_err(&chip->client->dev,
			"%s: failed to get tmg399x id\n",
			__func__);

	dev_info(dev, "%s: device id:%02x device rev:%02x\n", __func__,
				id, rev);

	for (i = 0; i < ARRAY_SIZE(tmg399x_ids); i++) 
    {
		if (id == tmg399x_ids[i])
        {
            SENSOR_LOG_INFO("id = %d\n",id);
            if (i>=1)
            {
                i = 1;
            }
			break;
        }
	}

	if (i < ARRAY_SIZE(tmg399x_names)) 
    {
		dev_info(dev, "%s: '%s rev. %d' detected\n", __func__, tmg399x_names[i], rev);
		chip->device_index = i;
	} 
    else 
    {
		dev_err(dev, "%s: not supported chip id\n", __func__);
		ret = -EOPNOTSUPP;
		goto id_failed;
	}

	mutex_init(&chip->lock);
    wake_lock_init(&chip->proximity_wakelock.lock, WAKE_LOCK_SUSPEND, (chip->proximity_wakelock).name);

	tmg399x_set_defaults(chip);
	ret = tmg399x_flush_regs(chip);
	if (ret)
		goto flush_regs_failed;
	if (pdata->platform_power) {
		pdata->platform_power(dev, POWER_OFF);
		powered = false;
		chip->unpowered = true;
	}

	if (!pdata->prox_name)
		goto bypass_prox_idev;
	chip->p_idev = input_allocate_device();
	if (!chip->p_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",
				__func__, pdata->prox_name);
		ret = -ENODEV;
		goto input_p_alloc_failed;
	}
	chip->p_idev->name = pdata->prox_name;
	chip->p_idev->id.bustype = BUS_I2C;


    // for prox
    set_bit(EV_REL,     chip->p_idev->evbit);
    set_bit(REL_X,      chip->p_idev->relbit);
    set_bit(REL_Y,      chip->p_idev->relbit);
    set_bit(REL_Z,      chip->p_idev->relbit);
    set_bit(REL_MISC,   chip->p_idev->relbit);

    // for ges
    set_bit(REL_RX,     chip->p_idev->relbit);  //gesture style
    set_bit(REL_RY,     chip->p_idev->relbit);  //gesture enter time
    set_bit(REL_RZ,     chip->p_idev->relbit);  //gesture exit time
    set_bit(REL_HWHEEL, chip->p_idev->relbit);  //gesture enter angle
    set_bit(REL_DIAL,   chip->p_idev->relbit);  //gesture exit angle


  
	dev_set_drvdata(&chip->p_idev->dev, chip);
	ret = input_register_device(chip->p_idev);
	if (ret) {
		input_free_device(chip->p_idev);
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, pdata->prox_name);
		goto input_p_register_failed;
	}
   
	ret = tmg399x_add_sysfs_interfaces(&chip->p_idev->dev,
			attrs_prox, ARRAY_SIZE(attrs_prox));
    if (ret)
		goto input_p_sysfs_failed;

bypass_prox_idev:

	if (!pdata->als_name)
		goto bypass_als_idev;
	chip->a_idev = input_allocate_device();
	if (!chip->a_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",
				__func__, pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->a_idev->name = pdata->als_name;
	chip->a_idev->id.bustype = BUS_I2C;

    set_bit(EV_REL, chip->a_idev->evbit);
    set_bit(REL_X,  chip->a_idev->relbit);
    set_bit(REL_Y,  chip->a_idev->relbit);

	dev_set_drvdata(&chip->a_idev->dev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		input_free_device(chip->a_idev);
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, pdata->prox_name);
		goto input_a_register_failed;
	}
	ret = tmg399x_add_sysfs_interfaces(&chip->a_idev->dev,
			attrs_light, ARRAY_SIZE(attrs_light));
	if (ret)
		goto input_a_sysfs_failed;

bypass_als_idev:

	if (!pdata->ges_name)
		goto bypass_ges_idev;
	chip->g_idev = input_allocate_device();
	if (!chip->g_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",
				__func__, pdata->ges_name);
		ret = -ENODEV;
		goto input_g_alloc_failed;
	}
	chip->g_idev->name = pdata->ges_name;
	chip->g_idev->id.bustype = BUS_I2C;

    // for ges
    set_bit(EV_REL,     chip->g_idev->evbit);
    set_bit(REL_RX,     chip->g_idev->relbit);  //gesture style
    set_bit(REL_RY,     chip->g_idev->relbit);  //gesture enter time
    set_bit(REL_RZ,     chip->g_idev->relbit);  //gesture exit  time
    set_bit(REL_HWHEEL, chip->g_idev->relbit);  //gesture enter angle
    set_bit(REL_DIAL,   chip->g_idev->relbit);  //gesture exit  angle


	dev_set_drvdata(&chip->g_idev->dev, chip);
	ret = input_register_device(chip->g_idev);
	if (ret) {
		input_free_device(chip->g_idev);
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, pdata->ges_name);
		goto input_g_register_failed;
	}
	ret = tmg399x_add_sysfs_interfaces(&chip->g_idev->dev,
			attrs_gesture, ARRAY_SIZE(attrs_gesture));
	if (ret)
		goto input_g_sysfs_failed;
bypass_ges_idev:

    ret = gpio_request(TMG399X_INT_PIN, "tmg3993");
	if (ret)    
    {
        SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",TMG399X_INT_PIN);
        
        gpio_free(TMG399X_INT_PIN);
        ret = gpio_request(TMG399X_INT_PIN, "tmg3993");
        if (ret) 
        {
            SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",TMG399X_INT_PIN);
            return ret;
        }
	}
    
    ret = gpio_tlmm_config(GPIO_CFG(TMG399X_INT_PIN, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

    client->irq = gpio_to_irq(TMG399X_INT_PIN);///ztemt
    SENSOR_LOG_INFO("client->irq = %d\n",client->irq);
	INIT_WORK(&chip->irq_work, tmg399x_irq_work);
	ret = request_threaded_irq(client->irq, NULL, &tmg399x_irq, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "tmg399x", chip);
	if (ret) {
		dev_info(dev, "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}

    INIT_DELAYED_WORK(&chip->als_poll_work, tmg399x_als_poll_work_func);
    INIT_DELAYED_WORK(&chip->prox_calibrate_work, tmg399x_prox_calibrate_work_func);
  //  INIT_DELAYED_WORK(&chip->prox_unwakelock_work, tmg399x_unwakelock_work_func);
    hrtimer_init(&chip->prox_unwakelock_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    chip->prox_unwakelock_timer.function = tmg399x_unwakelock_work_func;
  //  hrtimer_start(&chip->prox_unwakelock_timer, ktime_set(0, 0), HRTIMER_MODE_REL);


	init_waitqueue_head(&gesture_drdy_wq);
	atomic_set(&gesture_drdy, 0);
    chip->light_poll_time = AMS_ALS_POLL_DELAY_FAST;

    proximity_class = class_create(THIS_MODULE, "proximity");
    light_class     = class_create(THIS_MODULE, "light");
    gesture_class   = class_create(THIS_MODULE, "gesture");

 
    chip->proximity_dev = device_create(proximity_class, NULL, tmg399x_proximity_dev_t, &tmg399x_driver ,"proximity");
    if (IS_ERR(chip->proximity_dev)) 
    {
       ret = PTR_ERR(chip->proximity_dev);
       goto create_proximity_dev_failed;
    }

    chip->light_dev= device_create(light_class, NULL, tmg399x_light_dev_t, &tmg399x_driver ,"light");
    if (IS_ERR(chip->light_dev)) 
    {
       ret = PTR_ERR(chip->light_dev);
       goto create_light_dev_failed;
    }

    chip->gesture_dev = device_create(gesture_class, NULL, tmg399x_gesture_dev_t, &tmg399x_driver ,"gesture");
    if (IS_ERR(chip->gesture_dev)) 
    {
       ret = PTR_ERR(chip->gesture_dev);
       goto create_gesture_dev_failed;
    }

	dev_set_drvdata(chip->proximity_dev, chip);
	dev_set_drvdata(chip->light_dev, chip);
	dev_set_drvdata(chip->gesture_dev, chip);

    create_sysfs_interfaces_prox(chip->proximity_dev);
    create_sysfs_interfaces_light(chip->light_dev);
    create_sysfs_interfaces_gesture(chip->gesture_dev);

    tmg399x_irq_enable(false, true);

	SENSOR_LOG_INFO("Probe ok.\n");
	return 0;

create_proximity_dev_failed:
    chip->proximity_dev = NULL;
    class_destroy(proximity_class);

create_light_dev_failed:
    chip->proximity_dev = NULL;
    chip->light_dev     = NULL;
    class_destroy(proximity_class);
    class_destroy(proximity_class);

create_gesture_dev_failed:
    chip->proximity_dev = NULL;
    chip->light_dev     = NULL;
    chip->gesture_dev     = NULL;
    class_destroy(proximity_class);
    class_destroy(light_class);
    class_destroy(gesture_class);


irq_register_fail:
    if (chip->a_idev) {
            tmg399x_remove_sysfs_interfaces(&chip->g_idev->dev,attrs_gesture, ARRAY_SIZE(attrs_gesture));
input_g_sysfs_failed:
            input_unregister_device(chip->g_idev);
input_g_register_failed:
            input_free_device(chip->g_idev);
        }
input_g_alloc_failed:

	if (chip->a_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->a_idev->dev,attrs_light, ARRAY_SIZE(attrs_light));
input_a_sysfs_failed:
		input_unregister_device(chip->a_idev);
input_a_register_failed:
		input_free_device(chip->a_idev);
	}
input_a_alloc_failed:
	if (chip->p_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->p_idev->dev, attrs_prox, ARRAY_SIZE(attrs_prox));
input_p_sysfs_failed:
		input_unregister_device(chip->p_idev);
input_p_register_failed:
		input_free_device(chip->p_idev);
	}	
input_p_alloc_failed:
flush_regs_failed:
id_failed:
	kfree(chip->segment);
set_segment_failed:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
malloc_failed:
	if (powered && pdata->platform_power)
		pdata->platform_power(dev, POWER_OFF);
pon_failed:
	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
init_failed:
	SENSOR_LOG_INFO("Probe failed.\n");
	return ret;
}

static int tmg399x_prox_calibrate(struct tmg399x_chip *chip)
{
    int prox_sum = 0, prox_mean = 0, prox_max = 0,prox_min = 0;
    u8  i = 0 ;
    int ret = 0;
    struct tmg399x_prox_info *prox_cal_info = NULL;
    chip->prox_calibrate_result = false;

    prox_cal_info = kmalloc(sizeof(struct tmg399x_prox_info) * (chip->prox_calibrate_times), GFP_KERNEL); 
    if (NULL == prox_cal_info)
    {
        SENSOR_LOG_ERROR("malloc prox_cal_info failed\n");
        ret = -1;
        goto prox_calibrate_failed;
    }

    memset(prox_cal_info, 0, sizeof(struct tmg399x_prox_info) * (chip->prox_calibrate_times));

    if (false == (chip->prx_enabled))
    {
        chip->shadow[TMG399X_CONTROL] |= (TMG399X_EN_PWR_ON | TMG399X_EN_PRX|TMG399X_EN_WAIT);
        ret = tmg399x_update_enable_reg(chip);
        if (ret < 0)
        {
            
            SENSOR_LOG_ERROR("enable prox failed\n");
            kfree(prox_cal_info);
            goto prox_calibrate_failed;
        }    
        chip->light_poll_time = AMS_ALS_POLL_DELAY_SLOW;
        tmg399x_als_atime_set(AMS_ALS_ATIME_SHORT);
    }
	
    mdelay(20);

	for (i = 0; i < chip->prox_calibrate_times; i++) 
    {
        tmg399x_read_prox_data(chip);   
        prox_cal_info[i].raw = chip->shadow[TMG399X_PRX_CHAN];
		prox_sum += prox_cal_info[i].raw;

		if (prox_cal_info[i].raw> prox_max)
        {
		    	prox_max = prox_cal_info[i].raw;
		}
        if (prox_cal_info[i].raw < prox_min)
		{
   	        prox_min = prox_cal_info[i].raw;
        }

        SENSOR_LOG_ERROR("prox data = %d\n",prox_cal_info[i].raw);

	    mdelay(30);
	}
             
    prox_mean = prox_sum/chip->prox_calibrate_times;
				
    if (prox_max <= 50)
    {

		chip->params.prox_th_high = prox_mean + 110;
		chip->params.prox_th_low  = prox_mean + 30;
    }
	  else
	 {
        if (prox_max > 170)
        {
		chip->params.prox_th_high = 250;
		chip->params.prox_th_low  = 230;
        }
        else
        {
           
		chip->params.prox_th_high = prox_mean +50;
		chip->params.prox_th_low  = prox_mean +30;
            
        }        
	}

	SENSOR_LOG_ERROR("chip->params.prox_th_high = %d\n",chip->params.prox_th_high );
	SENSOR_LOG_ERROR("chip->params.prox_th_low  = %d\n",chip->params.prox_th_low);

    chip->shadow[TMG399X_PRX_THRES_HIGH] = chip->params.prox_th_high;
    chip->shadow[TMG399X_PRX_THRES_LOW]  = chip->params.prox_th_low;

    input_report_rel(chip->p_idev, REL_Y, chip->params.prox_th_high);
    input_report_rel(chip->p_idev, REL_Z, chip->params.prox_th_low);
	input_sync(chip->p_idev);
    
    if (true == (chip->prx_enabled))
    {
        tmg399x_prox_enable(chip, true);
    }
    else
    {
        tmg399x_prox_enable(chip, false);
    }
    
    kfree(prox_cal_info);
    chip->prox_calibrate_result = true;
    return 0;

prox_calibrate_failed:
    return ret;
}



static int __devexit tmg399x_remove(struct i2c_client *client)
{
	struct tmg399x_chip *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	free_irq(client->irq, chip);
	if (chip->a_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->a_idev->dev,
			attrs_light, ARRAY_SIZE(attrs_light));
		input_unregister_device(chip->a_idev);
	}
	if (chip->p_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->p_idev->dev,
			attrs_prox, ARRAY_SIZE(attrs_prox));
		input_unregister_device(chip->p_idev);
	}
	if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);
	i2c_set_clientdata(client, NULL);
	kfree(chip->segment);
	kfree(chip);
	mutex_unlock(&chip->lock);
	return 0;
}

static int __init tmg399x_init(void)
{
#ifdef CONFIG_ZTEMT_SENSORS_ALS_PS_AUTO_DETECT
    return 0;
#else
    return i2c_add_driver(&tmg399x_driver);
#endif
}

static void __exit tmg399x_exit(void)
{
	i2c_del_driver(&tmg399x_driver);
}

module_init(tmg399x_init);
module_exit(tmg399x_exit);

MODULE_AUTHOR("J. August Brenner<jon.brenner@ams.com>");
MODULE_AUTHOR("Byron Shi<byron.shi@ams.com>");
MODULE_DESCRIPTION("AMS-TAOS tmg3992/3 Ambient, Proximity, Gesture sensor driver");
MODULE_LICENSE("GPL");
