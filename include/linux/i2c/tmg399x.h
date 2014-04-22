/*
 * Device driver for monitoring ambient light intensity in (lux)
 * proximity detection (prox), and Gesture functionality within the
 * AMS-TAOS TMG399X family of devices.
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

#ifndef __TMG399X_H
#define __TMG399X_H

#include <linux/types.h>
#include <linux/wakelock.h>

/* Max number of segments allowable in LUX table */
#define TMG399X_MAX_LUX_TABLE_SIZE		9
#define MAX_DEFAULT_TABLE_BYTES (sizeof(int) * TMG399X_MAX_LUX_TABLE_SIZE)

/* Default LUX and Color coefficients */

#define D_Factor	241
#define R_Coef		144
#define G_Coef		1000
#define B_Coef		400
#define CT_Coef		(3972)
#define CT_Offset	(1672)

#define D_Factor1	615*2
#define R_Coef1		443
#define G_Coef1		1000
#define B_Coef1		180
#define CT_Coef1	(2042)
#define CT_Offset1	(2396)

#define TMG399X_INT_PIN    49

struct device;

#define TMG399X_CMD_IRBEAM_INT_CLR	0xE3
#define TMG399X_CMD_PROX_INT_CLR	0xE5
#define TMG399X_CMD_ALS_INT_CLR		0xE6
#define TMG399X_CMD_NON_GES_INT_CLR	0xE7

#define PRX_PERSIST(p) (((p) & 0xf) << 4)
#define ALS_PERSIST(p) (((p) & 0xf) << 0)
#define PRX_PULSE_CNT(p) (((p-1) & 0x3f) << 0)
#define GES_PULSE_CNT(p) (((p-1) & 0x3f) << 0)

#define I2C_ADDR_OFFSET	0X80

enum tmg399x_regs {
	TMG399X_CONTROL,
	TMG399X_ALS_TIME,
	TMG399X_RESV_1,
	TMG399X_WAIT_TIME,
	TMG399X_ALS_MINTHRESHLO,
	TMG399X_ALS_MINTHRESHHI,
	TMG399X_ALS_MAXTHRESHLO,
	TMG399X_ALS_MAXTHRESHHI,
	TMG399X_RESV_2,
	TMG399X_PRX_THRES_LOW,
	TMG399X_RESV_3,
	TMG399X_PRX_THRES_HIGH,
	TMG399X_PERSISTENCE,
	TMG399X_CONFIG_1,
	TMG399X_PRX_PULSE,
	TMG399X_GAIN,

	TMG399X_CONFIG_2,
	TMG399X_REVID,
	TMG399X_CHIPID,
	TMG399X_STATUS,
	TMG399X_CLR_CHANLO,
	TMG399X_CLR_CHANHI,
	TMG399X_RED_CHANLO,
	TMG399X_RED_CHANHI,
	TMG399X_GRN_CHANLO,
	TMG399X_GRN_CHANHI,
	TMG399X_BLU_CHANLO,
	TMG399X_BLU_CHANHI,
	TMG399X_PRX_CHAN,
	TMG399X_PRX_OFFSET_NE,
	TMG399X_PRX_OFFSET_SW,
	TMG399X_CONFIG_3,

	TMG399X_GES_ENTH,
	TMG399X_GES_EXTH,
	TMG399X_GES_CFG_1,
	TMG399X_GES_CFG_2,
	TMG399X_GES_OFFSET_N,
	TMG399X_GES_OFFSET_S,
	TMG399X_GES_PULSE,
	TMG399X_GES_OFFSET_W,
	TMG399X_GES_RESV,
	TMG399X_GES_OFFSET_E,
	TMG399X_GES_CFG_3,
	TMG399X_GES_CFG_4,
	TMG399X_RESV_4,
	TMG399X_RESV_5,
	TMG399X_GES_FLVL,
	TMG399X_GES_STAT,

	TMG399X_REG_MAX,
};

enum tmg399x_irbeam_regs {
	TMG399X_IRBEAM_CFG = 0x20,
	TMG399X_IRBEAM_RESV,
	TMG399X_IRBEAM_NS,
	TMG399X_IRBEAM_ISD,
	TMG399X_IRBEAM_NP,
	TMG399X_IRBEAM_IPD,
	TMG399X_IRBEAM_DIV,
	TMG399X_IRBEAM_LEN,
	TMG399X_IRBEAM_STAT,
};

enum tmg399x_gesfifo_regs {
	TMG399X_GES_NFIFO = 0x7C,
	TMG399X_GES_SFIFO,
	TMG399X_GES_WFIFO,
	TMG399X_GES_EFIFO,
};

enum tmg399x_remote_regs {
	TMG399X_REMOTE_CFG = 0x20,
	TMG399X_REMOTE_CARR,
	TMG399X_REMOTE_NS,
	TMG399X_REMOTE_DLY2T,
	TMG399X_REMOTE_NCP,
	TMG399X_REMOTE_CPOFF,
	TMG399X_REMOTE_DIV,
	TMG399X_REMOTE_LEN,
	TMG399X_REMOTE_STAT,
	TMG399X_REMOTE_SLEN,
};

enum tmg399x_pwr_state {
	POWER_ON,
	POWER_OFF,
	POWER_STANDBY,
};

enum tmg399x_en_reg {
	TMG399X_EN_PWR_ON   = (1 << 0),
	TMG399X_EN_ALS      = (1 << 1),
	TMG399X_EN_PRX      = (1 << 2),
	TMG399X_EN_WAIT     = (1 << 3),
	TMG399X_EN_ALS_IRQ  = (1 << 4),
	TMG399X_EN_PRX_IRQ  = (1 << 5),
	TMG399X_EN_GES      = (1 << 6),
	TMG399X_EN_BEAM     = (1 << 7),
};

enum tmg399x_cfgl_reg {
	WLONG          = (1 << 1),
};

enum tmg399x_ppulse_reg {
	PPLEN_4US      = (0 << 6),
	PPLEN_8US      = (1 << 6),
	PPLEN_16US     = (2 << 6),
	PPLEN_32US     = (3 << 6),
};

enum tmg399x_ctrl_reg {
	AGAIN_1        = (0 << 0),
	AGAIN_4        = (1 << 0),
	AGAIN_16       = (2 << 0),
	AGAIN_64       = (3 << 0),
	PGAIN_1        = (0 << 2),
	PGAIN_2        = (1 << 2),
	PGAIN_4        = (2 << 2),
	PGAIN_8        = (3 << 2),
	PDRIVE_100MA   = (0 << 6),
	PDRIVE_50MA    = (1 << 6),
	PDRIVE_25MA    = (2 << 6),
	PDRIVE_12MA    = (3 << 6),
};


enum tmg399x_cfg2_reg {
	LEDBOOST_100   = (0 << 4),
	LEDBOOST_150   = (1 << 4),
	LEDBOOST_200   = (2 << 4),
	LEDBOOST_300   = (3 << 4),
	CPSIEN         = (1 << 6),
	PSIEN          = (1 << 7),
};

enum tmg399x_status {
	TMG399X_ST_ALS_VALID  = (1 << 0),
	TMG399X_ST_PRX_VALID  = (1 << 1),
	TMG399X_ST_GES_IRQ    = (1 << 2),
	TMG399X_ST_BEAM_IRQ   = (1 << 3),
	TMG399X_ST_ALS_IRQ    = (1 << 4),
	TMG399X_ST_PRX_IRQ    = (1 << 5),
	TMG399X_ST_PRX_SAT    = (1 << 6),
	TMG399X_ST_CP_SAT     = (1 << 7),
};

enum tmg399x_cfg3_reg {
	PMASK_E        = (1 << 0),
	PMASK_W        = (1 << 1),
	PMASK_S        = (1 << 2),
	PMASK_N        = (1 << 3),
	SAI            = (1 << 4),
	PCMP           = (1 << 5),
};

enum tmg399x_ges_cfg1_reg {
	GEXPERS_1      = (0 << 0),
	GEXPERS_2      = (1 << 0),
	GEXPERS_4      = (2 << 0),
	GEXPERS_7      = (3 << 0),
	GEXMSK_E       = (1 << 2),
	GEXMSK_W       = (1 << 3),
	GEXMSK_S       = (1 << 4),
	GEXMSK_N       = (1 << 5),
	FIFOTH_1       = (0 << 6),
	FIFOTH_4       = (1 << 6),
	FIFOTH_8       = (2 << 6),
	FIFOTH_16      = (3 << 6),
};

enum tmg399x_ges_cfg2_reg {
	GWTIME_0       = (0 << 0),
	GWTIME_3       = (1 << 0),
	GWTIME_6       = (2 << 0),
	GWTIME_8       = (3 << 0),
	GWTIME_14      = (4 << 0),
	GWTIME_22      = (5 << 0),
	GWTIME_30      = (6 << 0),
	GWTIME_39      = (7 << 0),
	GLDRIVE_100    = (0 << 3),
	GLDRIVE_50     = (1 << 3),
	GLDRIVE_25     = (2 << 3),
	GLDRIVE_12     = (3 << 3),
	GGAIN_1        = (0 << 5),
	GGAIN_2        = (1 << 5),
	GGAIN_4        = (2 << 5),
	GGAIN_8        = (3 << 5),
};

enum tmg399x_gpulse_reg {
	GPLEN_4US      = (0 << 6),
	GPLEN_8US      = (1 << 6),
	GPLEN_16US     = (2 << 6),
	GPLEN_32US     = (3 << 6),
};

enum tmg399x_ges_cfg3_reg {
	GBOTH_PAIR     = (0 << 0),
	GONLY_NS       = (1 << 0),
	GONLY_WE       = (2 << 0),
};

enum tmg399x_ges_cfg4 {
	TMG399X_GES_MODE     = (1 << 0),
	TMG399X_GES_EN_IRQ   = (1 << 1),
	TMG399X_GES_INT_CLR  = (1 << 2),
};

enum tmg399x_ges_status {
	TMG399X_GES_VALID     = (1 << 0),
	TMG399X_GES_FOV       = (1 << 1),
};

enum {
	TMG399X_ALS_GAIN_MASK = (3 << 0),
	TMG399X_PRX_GAIN_MASK = (3 << 2),
	TMG399X_LDRIVE_MASK   = (3 << 6),
	TMG399X_ALS_AGL_MASK  = (1 << 2),
	TMG399X_ALS_AGL_BOOST = 2,
	TMG399X_ATIME_PER_100 = 278,
	//TMG399X_ATIME_DEFAULT_MS = 50,
	SCALE_SHIFT = 11,
	RATIO_SHIFT = 10,
	MAX_ALS_VALUE = 0xffff,
	MIN_ALS_VALUE = 0,
	GAIN_SWITCH_LEVEL = 100,
	//GAIN_AUTO_INIT_VALUE = AGAIN_16,
	TMG399X_GES_ST_MASK   = (3 << 0),
    TMG399X_GES_GAIN_MASK = (3 << 5)
};

struct tmg399x_als_info {
	u32 cpl;
	u32 saturation;
	u16 clear_raw;
	u16 red_raw;
	u16 green_raw;
	u16 blue_raw;
	u16 lux;
	u16 cct;
	s16 ir;
};

struct tmg399x_prox_info {
	int raw;
	int detected;
};

struct tmg399x_parameters {
	u8 als_time;
	u8 als_gain;
	u16 als_deltaP;
	u8 wait_time;
	u8 prox_th_low;
	u8 prox_th_high;
	u8 persist;
	u8 als_prox_cfg1;
	u8 prox_pulse;
	u8 prox_gain;
	u8 ldrive;
	u8 als_prox_cfg2;
	s8 prox_offset_ne;
	s8 prox_offset_sw;
	u8 als_prox_cfg3;
	
	u8 ges_entry_th;
	u8 ges_exit_th;
	u8 ges_cfg1;
	u8 ges_cfg2;
	s8 ges_offset_n;
	s8 ges_offset_s;
	u8 ges_pulse;
	s8 ges_offset_w;
	s8 ges_offset_e;
	u8 ges_dimension;
};

struct tmg399x_ges_raw_data {
	int north;
	int south;
	int west;
	int east;
};

struct sGesture_Result
{
    int  gesture_style;
    long enter_time;
    long exit_time;
    int  enter_angle;
    int  exit_angle;
};

struct tmg399x_wake_lock{
    struct wake_lock lock;
    bool   locked;
    char   *name;
};

struct tmg399x_chip {
	struct mutex lock;
    struct tmg399x_wake_lock proximity_wakelock;
    struct tmg399x_wake_lock display_power_controller;
	struct i2c_client *client;
	struct work_struct irq_work;
	struct delayed_work als_poll_work;
	struct delayed_work prox_calibrate_work;
	struct hrtimer prox_unwakelock_timer;

	struct tmg399x_prox_info prx_inf;
	struct tmg399x_als_info als_inf;
	struct tmg399x_parameters params;
	struct tmg399x_i2c_platform_data *pdata;
    struct tmg399x_ges_raw_data gesture_data[32];
	struct lux_segment *segment;

	struct input_dev *a_idev;
	struct input_dev *p_idev;
	struct input_dev *g_idev;

    struct device *proximity_dev;
    struct device *light_dev;
    struct device *gesture_dev;

	u8 shadow[48];
	u8 device_index;

	int  in_suspend;
	int  wake_irq;
	int  irq_pending;
	int  segment_num;
	int  seg_num_max;
    int  light_poll_time;

    int  prox_calibrate_times;
    int  prox_thres_hi_max;
    int  prox_thres_lo_min;
    int  prox_data_max;
    int prox_manual_calibrate_threshold;

    char *chip_name;
    


	bool unpowered;
	bool als_enabled;
	bool wait_enabled;
	bool prx_enabled;
	bool ges_enabled;
	bool ges_debug_enable;
	bool light_debug_enable;
	bool prox_debug_enable;
	bool prox_calibrate_result;
	bool prox_calibrate_start;
	bool als_gain_auto;
	bool wakeup_from_sleep;
	bool wakelock_locked;
	bool irq_enabled;
	bool irq_work_status;
	bool gesture_start; 
	bool phone_is_sleep;
};

struct lux_segment {
	int d_factor;
	int r_coef;
	int g_coef;
	int b_coef;
	int ct_coef;
	int ct_offset;
};

struct tmg399x_i2c_platform_data {
	/* The following callback for power events received and handled by
	   the driver.  Currently only for SUSPEND and RESUME */
	int (*platform_power)(struct device *dev, enum tmg399x_pwr_state state);
	int (*platform_init)(void);
	void (*platform_teardown)(struct device *dev);
	char const *prox_name;
	char const *als_name;
	char const *ges_name;
	struct tmg399x_parameters parameters;
	bool proximity_can_wake;
	bool als_can_wake;
	struct lux_segment *segment;
	int segment_num;
};

#endif /* __TMG399X_H */
