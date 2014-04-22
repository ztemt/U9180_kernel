/***********************************************************************************/
/* File Name: aw_9106b.c */
/* File Description: this file is used to make aw9106b driver to be added in kernel or module. */

/*  Copyright (c) 2002-2012, ZTEMT, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: ZTEMT, Inc.,            */
/***********************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/io.h>
#include <linux/kthread.h>

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock_types.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>

#include <linux/i2c.h>
#include "aw_9106b.h"

#define GPIO_15 15
#define DELAY_256MS_UNIT 1
#define AW_DRIVER_NAME "aw9106bdrv"
//#define AW_GPIO_CONFIG


enum aw_fade_time {
	FADE_0_MS = 0x00,
	FADE_256_MS = 0x09,
	FADE_512_MS = 0x12,
	FADE_1024_MS = 0x1b,
	FADE_2048_MS = 0x24,
	FADE_4096_MS = 0x2d,
};
enum aw_fullon_time {
	FULLON_0_MS,
	FULLON_256_MS,
	FULLON_512_MS,
	FULLON_1024_MS,
	FULLON_2048_MS,
	FULLON_4096_MS,
	FULLON_8192_MS,
	FULLON_16384_MS,
};

enum aw_fulloff_time {
	FULLOFF_0_MS = 0x00,
	FULLOFF_256_MS = 0x08,
	FULLOFF_512_MS = 0x10,
	FULLOFF_1024_MS = 0x18,
	FULLOFF_2048_MS = 0x20,
	FULLOFF_4096_MS = 0x28,
	FULLOFF_8192_MS = 0x30,
	FULLOFF_16384_MS = 0x38,
};

enum aw_max_current {
	MAX_37_MA,
	MAX_27_8__MA,
	MAX_18_5__MA,
	MAX_9_25__MA,
};

enum aw_ctl {
	AW_CTL_DISABLE,
	AW_CTL_ENABLE,
};

enum aw_reg_ctl {
	REG_BIT_CLEAR,
	REG_BIT_SET,
};

enum aw_gpio_led {
	AW_LED_MODE,
	AW_GPIO_MODE,
};

enum aw_smart_blink{
	AW_SMART_MODE,
	AW_BLINK_MODE,
};

enum aw_smart_fade{
	AW_FADE_OFF,
	AW_FADE_ON,
};

enum aw_led_close_mode{
	AW_CLOSE_NOW,
	AW_CLOSE_DELAY,
};

enum aw_power_state{
	AW_POWER_DOWN,
	AW_POWER_ON,
};

enum aw_out_val{
	AW_OUT_LOW,
	AW_OUT_HIGH,
};

enum aw_outn_mode{
	AW_POWER_OFF,// 0
	AW_CONST_ON,  // 1
	AW_LOW_BATT_ON, // 2 
	AW_CHG_ON, // 3
	AW_NORMAL_ON, // 4
	AW_FADE_ON_STEP,  // 5
	AW_FADE_OFF_STEP, // 6
	AW_FADE_CYCLE, // 7
};

typedef struct {
	enum aw_fade_time fade_t;
	enum aw_fullon_time fullon_t;
	enum aw_fulloff_time fulloff_t;
	enum aw_max_current max_ma;
} breath_config;

typedef struct {
	enum aw_fade_time fade_t;
	enum aw_max_current max_ma;
} smart_config;

typedef struct  {
	breath_config breath_t[3];
	smart_config  smart_t[2];
} aw9106b_config_param;

#define AW_LED_DELAY_MS 650
#define TIME_MS_UNIT  1000000ULL
#define CONST_MIN_GRADE  10
#define CONST_MAX_GRADE  200

static int min_grade = CONST_MIN_GRADE;
module_param(min_grade, int, 0644);

static int max_grade = CONST_MAX_GRADE;
module_param(max_grade, int, 0644);

static int start_grade = CONST_MIN_GRADE;

struct aw9106b_plat_data aw9106b_data;
static struct aw9106b_regs_data aw9106b_regs = 
{
	.en_bre = 0x14 ,
	.smart_blink = 0x05,
	.in_out = 0x05,
	.out_val = 0x03,
	.smart_fade = 0x03,
	.led_gpio = 0x13,
	.fade_tmr = 0x15,
	.full_tmr = 0x16,
	.delay_bre0 = 0x17,
	.dim0 = 0x20,
	.aw_reset = 0x7f,
	.ctl = 0x11,
};

aw9106b_config_param aw_config = {
	.breath_t = {
		{FADE_512_MS,FULLON_0_MS,FULLOFF_4096_MS,MAX_9_25__MA},
		{FADE_2048_MS,FULLON_0_MS,FULLOFF_2048_MS,MAX_9_25__MA},
		{FADE_1024_MS,FULLON_0_MS,FULLOFF_256_MS,MAX_9_25__MA}
	},
	.smart_t ={
		{FADE_4096_MS,MAX_9_25__MA},
		{FADE_4096_MS,MAX_9_25__MA}
	},
};

extern int ztemt_get_hw_id(void);
void enable_outn_fade_cycle(enum aw_outn out,enum aw_fade_time fade_t);

//+++ added by duguowei,2013.12.18,for breath light dts
#ifdef CONFIG_OF
static struct of_device_id aw_9106b_match_table[] = {
	{ .compatible = "aw,9106b", },
	{}
};
#endif
//--- added by duguowei,2013.12.18,for breath light dts

/******************************************************** 
*					 I2C I/O function 				              *
*********************************************************/

//read aw9106b i2c function
static int aw9106b_i2c_rx_byte_data(
	  struct i2c_client *i2c,
      unsigned char  reg,
      unsigned char* buf)
{

	struct i2c_msg msgs[2];

	//write message: this is the sub address that we are reading from
	msgs[0].addr = i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;
	
	//read message: now we read into the buffer
	msgs[1].addr = i2c->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	if (i2c_transfer(i2c->adapter, msgs, 2) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.\n", __func__);
		return -4;
	}
    pr_debug("return  buf[0]=0x%x!\n",buf[0]);
	
	return 0;
}

//write aw9106b i2c function
static int aw9106b_i2c_tx_byte_data(
	struct i2c_client *i2c,
	unsigned char reg, 
	unsigned char buf)
{
	struct i2c_msg msgs;
	char bufwr[2];
	
	bufwr[0] = reg;
	bufwr[1] = buf;
	
	//write message: this is the sub address that we are reading from
	msgs.addr = i2c->addr;
	msgs.flags = 0;
	msgs.len = 2;
	msgs.buf = bufwr;
	
	if (i2c_transfer(i2c->adapter, &msgs, 1) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.\n", __func__);
		return -4;
	}

	return 0;
}

//modigy aw9106b i2c function
static int aw9106b_modify_regs(int reg,char bitn,enum aw_reg_ctl set)
{
	char buf = 0;
	int ret;
	
	ret = aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,reg,&buf);
    if(ret < 0)
		pr_err("%s: read reg[0x%x] fail!\n",__func__,reg);
	
	if(set == REG_BIT_SET)
		buf |= (0x01 << bitn);
	else
		buf &= ~(0x01 << bitn);

	ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,reg,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);

	return ret;

}

static void aw9106b_power_set(enum aw_power_state power_set)
{
	 gpio_set_value(GPIO_15, power_set);
}

static int set_fade_time(enum aw_fade_time fade_t)
{
    char buf;
	int ret;

    buf = fade_t;

    ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.fade_tmr,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw9106b_regs.fade_tmr);
    return ret;
}

static int set_full_onoff_time(enum aw_fullon_time full_on,enum aw_fulloff_time full_off)
{
    char buf;
	int ret;

    buf = full_on | full_off;

    ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.full_tmr,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw9106b_regs.full_tmr);
    return ret;
}
/******************************************************** 
*                                Config AW Outn mode                                 *
*********************************************************/

static int set_out_led_gpio(enum aw_outn out,enum aw_gpio_led gpmd_mode)
{
    enum aw_reg_ctl set;
	int ret;
	int reg;
	int shift;

    if( out <= AW_OUT_3 ){
		reg = aw9106b_regs.led_gpio;
		shift = out;
    }
	else{
		reg = 0x12;
        shift = out - AW_OUT_4;
	}
	
    if(gpmd_mode == AW_LED_MODE)
		set = REG_BIT_CLEAR;
	else
		set = REG_BIT_SET;
	
	ret = aw9106b_modify_regs(reg,shift,set);

	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);
    return ret;
}

static int set_out_breath(enum aw_outn out,enum aw_ctl enable)
{
	enum aw_reg_ctl set;
	int ret;
	int reg;

    reg = aw9106b_regs.en_bre;
    if( enable == AW_CTL_DISABLE )
		set = REG_BIT_CLEAR;
	else
		set = REG_BIT_SET;

    ret = aw9106b_modify_regs(reg,out,set);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);
    return ret;
}

static int set_out_smart_blink(enum aw_outn out,enum aw_smart_blink blink_cfg)
{
	enum aw_reg_ctl set;
	int ret;
	int reg;
	int shift;

    if( out <= AW_OUT_3 ){
		reg = aw9106b_regs.smart_blink;
		shift = out;
    }
	else{
		reg = 0x04;
        shift = out - AW_OUT_4;
	}
	
    if(blink_cfg == AW_SMART_MODE)
		set = REG_BIT_CLEAR;
	else
		set = REG_BIT_SET;
	
    ret = aw9106b_modify_regs(reg,shift,set);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);
    return ret;
}

static int set_out_smart_fade(enum aw_outn out,enum aw_smart_fade fade_onoff)
{
	enum aw_reg_ctl set;
	int ret;
	int reg;
	int shift;

    if( out <= AW_OUT_3 ){
		reg = aw9106b_regs.smart_fade;
		shift = out;
    }
	else{
		reg = 0x02;
        shift = out - AW_OUT_4;
	}
	
    if(fade_onoff == AW_FADE_OFF)
		set = REG_BIT_CLEAR;
	else
		set = REG_BIT_SET;
	
    ret = aw9106b_modify_regs(reg,shift,set);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);
    return ret;
}

static int set_out_delay_time(enum aw_outn out,int delay_unit)
{
    char buf;
	int ret;
	int reg;

	reg = aw9106b_regs.delay_bre0 + out;
	
    buf = delay_unit;

    ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,reg,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);
    return ret;
}

static int set_out_dim_grade(enum aw_outn out,char grade)
{
    char buf;
	int ret;
	int reg;

	reg = aw9106b_regs.dim0 + out;

    buf = grade;

    ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,reg,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);
    return ret;
}

static int set_const_current(enum aw_max_current max_i)
{
    char buf;
	int ret;

    buf = max_i;

    ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.ctl,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw9106b_regs.ctl);
    return ret;
}

static int start_blink_led(enum aw_max_current max_i)
{
    char buf;
	int ret;

    buf = max_i | 0x80;

    ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.ctl,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw9106b_regs.ctl);
    return ret;
}

static int close_out_blink_led(enum aw_led_close_mode close_mode)
{
    char buf;
	int ret;

	buf = 0;

    if(close_mode == AW_CLOSE_NOW)
	    ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.en_bre,buf);
	else
		ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.smart_blink,buf);

	
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw9106b_regs.ctl);
    return ret;
}

int aw_full_fade_time_confg(enum aw_fade_time fade_t,
	        enum aw_fullon_time full_on,
	        enum aw_fulloff_time full_off)
{
	int ret;

	ret = set_fade_time(fade_t);
	if(ret < 0)
		pr_err("%s: config fade time  fail!\n",__func__);
	
	ret = set_full_onoff_time(full_on,full_off);
	if(ret < 0)
		pr_err("%s: config full time fail!\n",__func__);

	return ret;
}

static int aw9106b_solft_reset(void)
{
    char buf;
	int ret;

    buf = 0x00;

    ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.aw_reset,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw9106b_regs.aw_reset);
    return ret;
}

#ifdef AW_GPIO_CONFIG
static int set_out0_gpio_value(enum aw_out_val out_val)
{
    char buf;
	int ret;

    buf = out_val;

    ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.out_val,buf);
	if(ret < 0)
		pr_err("%s: write out_val[0x%x] fail!\n",__func__,aw9106b_regs.out_val);
    return ret;
}
int enable_out0_gpio(enum aw_out_val out_val)
{

	int ret;
	//power on aw9106b
	aw9106b_power_set(AW_POWER_ON);

    ret = set_out_led_gpio(aw9106b_data.outn ,AW_GPIO_MODE);
	if(ret < 0)
		pr_err("%s: config fade time  fail!\n",__func__);
	
	ret = set_out0_gpio_value(out_val);
	if(ret < 0)
		pr_err("%s: config fade time  fail!\n",__func__);

	return ret;
}

#endif 

void enable_outn_fade_onoff(enum aw_outn out,smart_config smart_parm,
								enum aw_smart_fade fade_onoff)
{
    int rc = 0;
	
	aw9106b_power_set(AW_POWER_ON);

	if(fade_onoff == AW_FADE_ON){
        rc = set_out_breath(out,AW_CTL_DISABLE);
		if(rc < 0)
			pr_err("%s: diable out[%d] breath mode fail!\n",__func__,out);

	}

    //set fade,fullon,fulloff time
	rc = aw_full_fade_time_confg(smart_parm.fade_t,FULLOFF_0_MS,FULLOFF_0_MS);
	if(rc < 0)
		pr_err("%s: config fade full on/off time fail!\n",__func__);
 
    //set outn led mode
	rc = set_out_led_gpio(out,AW_LED_MODE);
	if(rc < 0)
		pr_err("%s: set out[%d] LED mode fail!\n",__func__,out);

	//enable outn breath func
    rc = set_out_breath(out,AW_CTL_ENABLE);
	if(rc < 0)
		pr_err("%s: set out[%d] breath mode fail!\n",__func__,out);

	//set outn blink
	rc = set_out_smart_blink(out,AW_SMART_MODE);
	if(rc < 0)
		pr_err("%s: set out smart-fade mode fail!\n",__func__);

	//set Imax
	rc = set_const_current(smart_parm.max_ma);
	if(rc < 0)
		pr_err("%s: set imax fail!\n",__func__);

	//set outn delay time
	rc = set_out_delay_time(out,0*DELAY_256MS_UNIT);
	if(rc < 0)
		pr_err("%s: set out delay time fail!\n",__func__);

	 //set outn smart fade
	rc = set_out_smart_fade(out,fade_onoff);
	if(rc < 0)
		pr_err("%s: start samrt-fade mode fail!\n",__func__);

}


void enable_outn_blink_led(enum aw_outn out,breath_config breath_parm)
{
    int rc = 0;

	aw9106b_power_set(AW_POWER_ON);

	rc = aw9106b_solft_reset();
	if(rc < 0)
		pr_err("%s: solft reset fail!\n",__func__);

    //set fade,fullon,fulloff time
	rc = aw_full_fade_time_confg(breath_parm.fade_t,breath_parm.fullon_t,breath_parm.fulloff_t);
	if(rc < 0)
		pr_err("%s: config fade full on/off time fail!\n",__func__);

    //set outn led mode
	rc = set_out_led_gpio(out,AW_LED_MODE);
	if(rc < 0)
		pr_err("%s: set out[%d] LED mode fail!\n",__func__,out);

	//enable outn breath func
    rc = set_out_breath(out,AW_CTL_ENABLE);
	if(rc < 0)
		pr_err("%s: set out[%d] breath mode fail!\n",__func__,out);

	//set outn blink
	rc = set_out_smart_blink(out,AW_BLINK_MODE);
	if(rc < 0)
		pr_err("%s: set out blink mode fail!\n",__func__);

	//set outn delay time
	rc = set_out_delay_time(out,0*DELAY_256MS_UNIT);
	if(rc < 0)
		pr_err("%s: set out delay time fail!\n",__func__);

	//start outn blink led
	rc = start_blink_led(breath_parm.max_ma);
	if(rc < 0)
		pr_err("%s: start blink fail!\n",__func__);
	
}

static void aw9106b_work_func(struct work_struct *work)
{
    int rc;
	int stop_work = 0;
	static int dim_grade = 0;
	static int grade_updown = 1;

	if(dim_grade == 0)
		dim_grade = start_grade;		

	if(grade_updown == 1 && dim_grade<max_grade)
		dim_grade += 5;
	else if(grade_updown == 2 && dim_grade>min_grade)
		dim_grade -= 5;   

	rc = set_out_dim_grade(aw9106b_data.outn,dim_grade);
	if(rc < 0){
		stop_work = 1;
		pr_err("%s: set out dim grade fail!\n",__func__);
	}
	
	if(grade_updown == 1 && dim_grade>=max_grade)
		grade_updown = 2;
    else if(grade_updown == 2 && dim_grade <= min_grade){
		stop_work = 1;
		pr_debug("%s: end breath!\n",__func__);	
    }

	if(stop_work){
		dim_grade = 0;
		grade_updown = 1;
		aw9106b_data.led_in_breathing = 0;
		return;
	}
	
    hrtimer_start(&aw9106b_data.timer,ktime_set(0,5*TIME_MS_UNIT),HRTIMER_MODE_REL);	
}

static enum hrtimer_restart aw9106b_timer(struct hrtimer *timer)
{
	schedule_work(&aw9106b_data.work);
	return HRTIMER_NORESTART;
}


void enable_outn_const_led(enum aw_outn out,enum aw_max_current imax,int grade)
{
    int rc = 0;

	aw9106b_power_set(AW_POWER_ON);

	rc = aw9106b_solft_reset();
	if(rc < 0)
		pr_err("%s: solft reset fail!\n",__func__);

    //set outn led mode
	rc = set_out_led_gpio(out,AW_LED_MODE);
	if(rc < 0)
		pr_err("%s: set out[%d] LED mode fail!\n",__func__,out);
	
    //set Imax
	rc = set_const_current(imax);
	if(rc < 0)
		pr_err("%s: set imax fail!\n",__func__);

	//set const current = (grade/256)*imax  
	rc = set_out_dim_grade(out,grade);
	if(rc < 0)
		pr_err("%s: set out dim grade fail!\n",__func__);

}

/*********************       aw9106b_breath_mode_set    *********************/

void aw9106b_breath_mode_set(struct led_classdev *led_cdev,
					  enum led_brightness brightness)
{
	static int last_val;
	int val = brightness;
	
	if((val == last_val) && 
		(val==AW_CONST_ON || val==AW_LOW_BATT_ON || val==AW_CHG_ON || val==AW_NORMAL_ON ))
	    return;

	aw9106b_data.outn = AW_OUT_4;

	pr_info("%s: out=%d val=%d\n",__func__,aw9106b_data.outn,val);
	//gpio_tlmm_config(GPIO_CFG(GPIO_15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);

	switch (val) {
	case AW_POWER_OFF:
		aw9106b_power_set(AW_POWER_DOWN);
		break;
	case AW_CONST_ON: 
		enable_outn_const_led(aw9106b_data.outn ,MAX_9_25__MA,min_grade);
		break;
    //blink breath mode
	case AW_LOW_BATT_ON:  
	case AW_CHG_ON:  
	case AW_NORMAL_ON: 
	    enable_outn_blink_led(aw9106b_data.outn ,aw_config.breath_t[val-AW_LOW_BATT_ON]);
		break;
    //smart breath mode
	case AW_FADE_ON_STEP:  
		enable_outn_fade_onoff(aw9106b_data.outn ,aw_config.smart_t[0],AW_FADE_ON);
		break;
	case AW_FADE_OFF_STEP:  
	    enable_outn_fade_onoff(aw9106b_data.outn ,aw_config.smart_t[1],AW_FADE_OFF);
		break;
	//fade 1 cycle for press home key	
	case AW_FADE_CYCLE:  
	    if( aw9106b_data.led_in_breathing == 0 ){
			aw9106b_data.led_in_breathing = 1;
			
			start_grade = last_val? min_grade:0;
			enable_outn_const_led(aw9106b_data.outn ,MAX_9_25__MA,start_grade);
		
			pr_debug("%s: start breath!\n",__func__);
			hrtimer_start(&aw9106b_data.timer,ktime_set(0,0),HRTIMER_MODE_REL);
		}
		break;
	default:
		  break;
	}
	last_val = val;
}
EXPORT_SYMBOL_GPL(aw9106b_breath_mode_set);
static void aw9106b_show_regs(void)
{
	char buf[1];

	//read EN_BRE
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.en_bre,buf);
    printk("read  en_bre[0x%x]= 0x%x\n",aw9106b_regs.en_bre,buf[0]);

	//set led_gpio 
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.led_gpio,buf);
	printk("read  led_gpio[0x%x] = 0x%x\n",aw9106b_regs.led_gpio,buf[0]);
	
    //set BLINK 
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.smart_blink,buf);
	printk("read  smart_blink[0x%x] = 0x%x\n",aw9106b_regs.smart_blink,buf[0]);

	//set delay time 
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.delay_bre0,buf);
	printk("read  delay_bre0[0x%x] = 0x%x\n",aw9106b_regs.delay_bre0,buf[0]);
	   
    //set flade time 
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.fade_tmr,buf);
	printk("read  fade_tmr[0x%x] = 0x%x\n",aw9106b_regs.fade_tmr,buf[0]);

	//set full on and off  time 
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.full_tmr,buf);
	printk("read  full_tmr[0x%x] = 0x%x\n",aw9106b_regs.full_tmr,buf[0]);

	//set gpio in and out 
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.in_out,buf);
	printk("read  in_out[0x%x] = 0x%x\n",aw9106b_regs.in_out,buf[0]);

	//start
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.ctl,buf);
    printk("read  ctl[0x%x] = 0x%x\n",aw9106b_regs.ctl,buf[0]);
}

static int led_config;
static int set_led_mode(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	printk("__%s: led_config=%d!\n",__func__,led_config);
	switch(led_config){
    case 0:
		aw9106b_power_set(AW_POWER_DOWN);
		break;
	case 1:
		aw9106b_data.outn = AW_OUT_0;
		break;
	case 2:
		aw9106b_data.outn = AW_OUT_1;
		break;
	case 3:
		aw9106b_show_regs();
		break;
	case 4:
		enable_outn_const_led(aw9106b_data.outn ,MAX_18_5__MA,min_grade);
		break;
	case 5:
		enable_outn_blink_led(aw9106b_data.outn ,aw_config.breath_t[2]);
		break;
	case 6:
		close_out_blink_led(AW_CLOSE_NOW);
		break;
	case 7:
		close_out_blink_led(AW_CLOSE_DELAY);
		break;
	default:
		break;
	};
	
	return 0;
}
module_param_call(led_config, set_led_mode, param_get_uint,
					&led_config, 0644);

static struct led_classdev breath_led = {
	.name		= "breath-led",
	.brightness_set	= aw9106b_breath_mode_set,
};

static int  aw9106b_probe(struct i2c_client *client,
			const struct i2c_device_id *dev_id)
{
	int ret = 0;
	
	printk("%s: start probe:\n",__func__);
	aw9106b_data.i2c_client = client;
	aw9106b_data.led_in_breathing = 0;
	
	ret = led_classdev_register(NULL, &breath_led);
	if (ret) {
		pr_err("unable to register breath_led ret=%d\n",ret);
		goto init_fail;
	}

	INIT_WORK(&aw9106b_data.work, aw9106b_work_func);

	hrtimer_init(&aw9106b_data.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw9106b_data.timer.function = aw9106b_timer;
		
	ret = gpio_request(GPIO_15, "aw9106b_shdn");
	if (ret) {
		pr_err("%s: fail gpio_request(%d)=%d\n", __func__,GPIO_15, ret);
		goto init_fail;
	}
	gpio_tlmm_config(GPIO_CFG(GPIO_15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);

	return 0;

	init_fail:
		return ret;
		
}

static int aw9106b_remove(struct i2c_client *client)
{	
    led_classdev_unregister(&breath_led);
	gpio_free(GPIO_15);
	return 0;
}

static int aw9106b_suspend(struct i2c_client *cl, pm_message_t mesg)
{

	return 0;
};

static int aw9106b_resume(struct i2c_client *cl)
{
	
	return 0;
};


static const struct i2c_device_id aw9106b_id[] = {
	{ "aw9106b", 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, BQ27520_id);

static struct i2c_driver aw9106b_driver = {
	.driver = {
		.name = "aw9106b_Driver",
//+++ added by duguowei,2013.12.18,for breath light dts
#ifdef CONFIG_OF
		.of_match_table = aw_9106b_match_table,
#endif
//--- added by duguowei,2013.12.18,for breath light dts
	},
	.id_table 	= aw9106b_id,
	.probe 		= aw9106b_probe,
	.remove 	= aw9106b_remove,

	.suspend	= aw9106b_suspend,
	.resume 	= aw9106b_resume,
};


static int __init aw9106b_init(void)
{
	printk(KERN_INFO "%s:enter...\n", __func__);

	return i2c_add_driver(&aw9106b_driver);
}

static void __exit aw9106b_exit(void)
{
	printk(KERN_INFO "%s:%d:aw9106b is exiting\n", __func__,__LINE__);
	
	i2c_del_driver(&aw9106b_driver);
}


late_initcall(aw9106b_init);
module_exit(aw9106b_exit);

MODULE_VERSION("1.0");
MODULE_AUTHOR("wangshuai <wang.shuai12@zte.com.cn>");
MODULE_DESCRIPTION("aw9106b Linux driver");
MODULE_ALIAS("platform:aw9106b");

