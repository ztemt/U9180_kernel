/*******************************************************************************
*                                                                              *
*       File Name:      taos_common.h                                          *
*       Description:    Common file for ioctl and configuration definitions.   *
*       		Used by kernel driver and driver access applications.  *
*       		Please include this file, and <sys/ioctl.h> in your    * 
*                       driver access application program source.	       *
*       Author:         John Koshi                                             *
*       History:        09/16/2009 - Initial creation                          *
*       		02/07/2010 - Add proximity			       *
*                                                                              *
********************************************************************************
*       Proprietary to Taos Inc., 1001 Klein Road #300, Plano, TX 75074        *
*******************************************************************************/
// ioctl numbers
//#include <types.h>
#include <linux/wakelock.h>

#define TAOS_IOCTL_MAGIC        	0XCF
#define TAOS_IOCTL_ALS_ON       	_IO(TAOS_IOCTL_MAGIC, 1)
#define TAOS_IOCTL_ALS_OFF      	_IO(TAOS_IOCTL_MAGIC, 2)
#define TAOS_IOCTL_ALS_DATA     	_IOR(TAOS_IOCTL_MAGIC, 3, short)
#define TAOS_IOCTL_ALS_CALIBRATE	_IO(TAOS_IOCTL_MAGIC, 4)
#define TAOS_IOCTL_CONFIG_GET   	_IOR(TAOS_IOCTL_MAGIC, 5, struct taos_cfg *)
#define TAOS_IOCTL_CONFIG_SET		_IOW(TAOS_IOCTL_MAGIC, 6, struct taos_cfg)
#define TAOS_IOCTL_PROX_ON		    _IO(TAOS_IOCTL_MAGIC, 7)
#define TAOS_IOCTL_PROX_OFF		    _IO(TAOS_IOCTL_MAGIC, 8)
#define TAOS_IOCTL_PROX_DATA		_IOR(TAOS_IOCTL_MAGIC, 9, struct taos_prox_info)
#define TAOS_IOCTL_PROX_EVENT       _IO(TAOS_IOCTL_MAGIC, 10)
#define TAOS_IOCTL_PROX_CALIBRATE	_IO(TAOS_IOCTL_MAGIC, 11)

#define TAOS_IOCTL_SENSOR_ON	    (TAOS_IOCTL_MAGIC, 12)
#define TAOS_IOCTL_SENSOR_OFF	    _IO(TAOS_IOCTL_MAGIC, 13)
#define TAOS_IOCTL_SENSOR_CONFIG	_IOW(TAOS_IOCTL_MAGIC, 14, struct taos_cfg)
#define TAOS_IOCTL_SENSOR_CHECK	    _IO(TAOS_IOCTL_MAGIC, 15)
#define TAOS_IOCTL_SENSOR_test	    _IO(TAOS_IOCTL_MAGIC, 16)

#define TAOS_IOCTL_ALS_SET_DELAY	_IO(TAOS_IOCTL_MAGIC, 17)


#define ALS_PS_GPIO  49 //INT for tmd2772

// device configuration
struct taos_cfg 
{
    u32 calibrate_target;
    u16 als_time;
    u16 scale_factor_als;
	u16 scale_factor_prox;
    u16 gain_trim;
    u8  filter_history;
    u8  filter_count;
    u8  gain;
	u16	prox_threshold_hi;
	u16 prox_threshold_lo;
	u16	als_threshold_hi;
	u16 als_threshold_lo;
	u8	prox_int_time;
	u8	prox_adc_time;
	u8	prox_wait_time;
	u8	prox_intr_filter;
	u8	prox_config;
	u8	prox_pulse_cnt;
	u8	prox_gain;
	u8	prox_config_offset;
	u32 als_ps_int;
};

// proximity data
struct taos_prox_info 
{
        u16 prox_clear;
        u16 prox_data;
        int prox_event;
};

struct taos_wake_lock{
    struct wake_lock lock;
    bool   locked;
    char   *name;
};

// add by clli2 for proximity debug
enum
{
	DEBUG_DEFAULT = 1U << 0,
	DEBUG_CRITICAL = 1U << 2,
	DEBUG_ABNORMAL = 1U << 3, //don't modify it
};
