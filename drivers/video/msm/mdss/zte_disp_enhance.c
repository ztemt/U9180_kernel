#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include "mdss_dsi.h"

enum {
	INTENSITY_NORMAL=24,
	INTENSITY_01,
	INTENSITY_02
};

#define ZTE_DISP_ENHANCE_DEBUG

unsigned int zte_intensity_value;
struct mdss_dsi_ctrl_pdata *zte_mdss_dsi_ctrl = NULL;
void zte_send_cmd(struct dsi_cmd_desc *cmds, int len);

#if defined(CONFIG_ZTEMT_NE501_LCD)
#if defined(CONFIG_ZTEMT_MIPI_720P_NT35592_SHARP_IPS_5P)
static char nt35592_55B0[] = {0x55, 0xb0};
static char nt35592_5580[] = {0x55, 0x80};
static char nt35592_55[] = {0x55, 0x00};

static char nt35592_FF03[] = {0xFF, 0x03};
static char nt35592_1A[] = {0x1A, 0x77};
static char nt35592_56[] = {0x56, 0x00};
static char nt35592_68[] = {0x68, 0x00};
static char nt35592_FB[] = {0xFB, 0x01};
static char nt35592_FF00[] = {0xFF, 0x00};
static struct dsi_cmd_desc sharp_nt35592_720p_enhance2[] = {
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_FF03)}, nt35592_FF03},
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_1A)}, nt35592_1A}, 
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_56)}, nt35592_56},
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_68)}, nt35592_68},
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_FB)}, nt35592_FB}, 
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_FF00)}, nt35592_FF00},
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_55B0)}, nt35592_55B0}, 	

};
static struct dsi_cmd_desc sharp_nt35592_720p_default[] = {
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_55)}, nt35592_55}, 	

};
static struct dsi_cmd_desc sharp_nt35592_720p_enhance1[] = {
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_FF03)}, nt35592_FF03},
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_1A)}, nt35592_1A}, 
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_56)}, nt35592_56},
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_68)}, nt35592_68},
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_FB)}, nt35592_FB}, 
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_FF00)}, nt35592_FF00},
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(nt35592_5580)}, nt35592_5580},	
};
#endif

static char hx8392b_ce_off[] = {0xE4, 0x00, 0x00};//
static struct dsi_cmd_desc sharp_hx8392b_720p_default[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8392b_ce_off)}, hx8392b_ce_off},
};

static char hx8392b_ce_low[] = {0xE4, 0x55, 0x01};//
static struct dsi_cmd_desc sharp_hx8392b_720p_enhance0[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8392b_ce_low)}, hx8392b_ce_low},
};
/*
static char hx8392b_ce_medium[] = {0xE4, 0xAA, 0x01};//
static struct dsi_cmd_desc sharp_hx8392b_720p_enhance1[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8392b_ce_medium)}, hx8392b_ce_medium},
};*/

static char hx8392b_ce_high[] = {0xE4, 0xff, 0x01};//
static struct dsi_cmd_desc sharp_hx8392b_720p_enhance2[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8392b_ce_high)}, hx8392b_ce_high},
};

void zte_NE501J_disp_inc(unsigned int state)
{
	unsigned int value;
	value =state;
#ifdef ZTE_DISP_ENHANCE_DEBUG
	printk("lcd:%s value=%d\n", __func__, value);
#endif
#if defined(CONFIG_ZTEMT_MIPI_720P_NT35592_SHARP_IPS_5P)
	if(!zte_mdss_dsi_ctrl)
	{
		pr_err("lcd:faild:%s zte_mdss_dsi_ctrl is null\n",__func__);
		return;
	}

	// lianchuang LCM's IC is the same as cs LCM's IC--nt35592
	if (zte_mdss_dsi_ctrl->panel_name 
		&& (!strcmp(zte_mdss_dsi_ctrl->panel_name, "cs nt35592 720p video mode dsi panel")
			|| !strcmp(zte_mdss_dsi_ctrl->panel_name, "lianchuang nt35592 720p video mode dsi panel"))) {
		switch (value) {
		case INTENSITY_NORMAL:
			zte_send_cmd(sharp_nt35592_720p_default
				, sizeof(sharp_nt35592_720p_default)/sizeof(sharp_nt35592_720p_default[0]));
			break;
		case INTENSITY_01:
			zte_send_cmd(sharp_nt35592_720p_enhance1
				, sizeof(sharp_nt35592_720p_enhance1)/sizeof(sharp_nt35592_720p_enhance1[0]));
			break;
		case INTENSITY_02:
			zte_send_cmd(sharp_nt35592_720p_enhance2
				, sizeof(sharp_nt35592_720p_enhance2)/sizeof(sharp_nt35592_720p_enhance2[0]));
			break;
		default:
			zte_send_cmd(sharp_nt35592_720p_enhance1
				, sizeof(sharp_nt35592_720p_enhance1)/sizeof(sharp_nt35592_720p_enhance1[0]));
			break;
		}
	} else {
#endif
		switch (value) {
		case INTENSITY_NORMAL:
			zte_send_cmd(sharp_hx8392b_720p_default
				, sizeof(sharp_hx8392b_720p_default)/sizeof(sharp_hx8392b_720p_default[0]));
			break;
		case INTENSITY_01:
			zte_send_cmd(sharp_hx8392b_720p_enhance0
			, sizeof(sharp_hx8392b_720p_enhance0)/sizeof(sharp_hx8392b_720p_enhance0[0]));
			/*zte_send_cmd(sharp_hx8392b_720p_enhance1
				, sizeof(sharp_hx8392b_720p_enhance1)/sizeof(sharp_hx8392b_720p_enhance1[0]));*/
			break;
		case INTENSITY_02:
			zte_send_cmd(sharp_hx8392b_720p_enhance2
				, sizeof(sharp_hx8392b_720p_enhance2)/sizeof(sharp_hx8392b_720p_enhance2[0]));
			break;
		default:
			zte_send_cmd(sharp_hx8392b_720p_enhance0
				, sizeof(sharp_hx8392b_720p_enhance0)/sizeof(sharp_hx8392b_720p_enhance0[0]));
			break;
		}
#if defined(CONFIG_ZTEMT_MIPI_720P_NT35592_SHARP_IPS_5P)
	}
#endif
}
#endif

void zte_send_cmd(struct dsi_cmd_desc *cmds, int len)
{
	struct dcs_cmd_req cmdreq;


	if(!zte_mdss_dsi_ctrl)
	{
		pr_err("lcd:faild:%s zte_mdss_dsi_ctrl is null\n",__func__);
		return;
	}
	
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = cmds;
	cmdreq.cmds_cnt = len;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(zte_mdss_dsi_ctrl, &cmdreq);
}

void zte_mipi_disp_inc(unsigned int state)
{
#if defined(CONFIG_ZTEMT_NE501_LCD)
  zte_NE501J_disp_inc(state);
#endif
}

void zte_disp_enhance(void)
{
  zte_mipi_disp_inc(zte_intensity_value);
}

static ssize_t intensity_show(struct kobject *kobj, struct kobj_attribute *attr,
   char *buf)
{
	//snprintf(buf, 50, "%u\n", zte_intensity_value);

	return 0;
}
static ssize_t intensity_store(struct kobject *kobj, struct kobj_attribute *attr,
    const char *buf, size_t size)
{
	ssize_t ret = 0;
	int val;
	static bool is_firsttime = true;
	sscanf(buf, "%d", &val);

#ifdef ZTE_DISP_ENHANCE_DEBUG
	printk("lcd:%s state=%d size=%d\n", __func__, (int)val, (int)size);
#endif
	zte_intensity_value = val;
	if (is_firsttime) {
		is_firsttime = false;
		return ret;
	}
	zte_mipi_disp_inc(val);

	return ret;
}

static struct kobj_attribute disptype_attribute =
 __ATTR(disptype, 0664, intensity_show, intensity_store);

 static struct attribute *attrs[] = {
  &disptype_attribute.attr,
  NULL, /* need to NULL terminate the list of attributes */
 };
 static struct attribute_group attr_group = {
  .attrs = attrs,
 };
 
 static struct kobject *id_kobj;
 
 static int __init id_init(void)
 {
  int retval;
 
  id_kobj = kobject_create_and_add("lcd_enhance", kernel_kobj);
  if (!id_kobj)
   return -ENOMEM;
 
  /* Create the files associated with this kobject */
  retval = sysfs_create_group(id_kobj, &attr_group);
  if (retval)
   kobject_put(id_kobj);
 
  return retval;
 }
 
 static void __exit id_exit(void)
 {
  kobject_put(id_kobj);
 }
 
 module_init(id_init);
 module_exit(id_exit);
 
