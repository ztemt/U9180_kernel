/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spmi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <ztemt_hw_version.h>


#define CONFIG_ZTEMT_HW_VERSION_DEBUG
#ifdef CONFIG_ZTEMT_HW_VERSION_DEBUG
#define ztemt_hw_version_debug(fmt, args...) printk(KERN_DEBUG "[ztemt_hw_version_debug]"fmt, ##args)
#else
#define ztemt_hw_version_debug(fmt, args...) do {} while(0)
#endif

#define QPNP_ZTEMT_HW_VERSION_DEV_NAME	"qcom,qpnp-ztemt_hw_version"

/**
 * struct qpnp_chg_chip - device information
 * @dev:			device pointer to access the parent
 * @spmi:			spmi pointer to access spmi information
 */
struct qpnp_ztemt_hw_version_chip {
	struct device	     *dev;
	struct spmi_device	 *spmi;
	struct work_struct	 work;
	struct mutex	     lock;
};



//#define CONFIG_ZTEMT_HW_VERSION_GPIO
#ifdef CONFIG_ZTEMT_HW_VERSION_GPIO

#define GPIO_A 51
#define GPIO_B 52
#define GPIO_C 63
#define VER_GPIO_A GPIO_CFG(GPIO_A, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA)
#define VER_GPIO_B GPIO_CFG(GPIO_B, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA)
#define VER_GPIO_C GPIO_CFG(GPIO_C, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA)
//#define VER_GPIO_A GPIO_CFG(GPIO_A, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
//#define VER_GPIO_B GPIO_CFG(GPIO_B, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
//#define VER_GPIO_C GPIO_CFG(GPIO_C, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define VER_GPIO_A_S GPIO_CFG(GPIO_A, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define VER_GPIO_B_S GPIO_CFG(GPIO_B, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define VER_GPIO_C_S GPIO_CFG(GPIO_C, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

struct ztemt_gpio_state{
int gpio_A;
int gpio_B;	
int gpio_C;	
};	
struct ztemt_gpio_state ztemt_gpio_st;

static const struct hardware_id_map_st hardware_id_map[] = {
	{HW_A,"ZTE_U9180"},  //GPIO value: 0,0,0
	{HW_B,"ZTE_V9180"},  //GPIO value: 0,1,0
	{HW_C,"ZTE_N9180"},  //GPIO value: 1,0,0
	{HW_D,"NE501J"},     //GPIO value: 1,1,0
};

static int ztemt_hw_id=-1;
static int gpio_init_flag=false;
static int ztemt_gpio_init(void)
{
		int rc=0;
		if(gpio_init_flag ==true)
	    return rc; 
		rc = gpio_request(GPIO_A, "GPIO_A");
		if (rc < 0) {
			pr_err("Failed request GPIO_A.\n");
			return rc;
		}
	  gpio_tlmm_config(VER_GPIO_A,GPIO_CFG_ENABLE);
	  
	  rc = gpio_request(GPIO_B, "GPIO_B");
		if (rc < 0) {
			pr_err("Failed request GPIO_B.\n");
			return rc;
		} 
	  gpio_tlmm_config(VER_GPIO_B,GPIO_CFG_ENABLE);
	      
	  rc = gpio_request(GPIO_C, "GPIO_C");
		if (rc < 0) {
			pr_err("Failed request GPIO_C.\n");
			return rc;
		}
	  gpio_tlmm_config(VER_GPIO_C,GPIO_CFG_ENABLE);
	  
	  gpio_init_flag=true;
	  return rc;
}

static int ztemt_get_hw_id(void)
{
  if((ztemt_hw_id >= 0)&&(ztemt_hw_id!=HW_UN))
	    return ztemt_hw_id;
	ztemt_gpio_init();
	ztemt_gpio_st.gpio_A=gpio_get_value(GPIO_A);
	ztemt_gpio_st.gpio_B=gpio_get_value(GPIO_B);
	ztemt_gpio_st.gpio_C=gpio_get_value(GPIO_C);
	
	if((ztemt_gpio_st.gpio_A==0)&&(ztemt_gpio_st.gpio_B==0)&&(ztemt_gpio_st.gpio_C==0)){  
	  ztemt_hw_id=HW_A;
	}else if ((ztemt_gpio_st.gpio_A==0)&&(ztemt_gpio_st.gpio_B==1)&&(ztemt_gpio_st.gpio_C==0)){
	  ztemt_hw_id=HW_B;
	}else if ((ztemt_gpio_st.gpio_A==1)&&(ztemt_gpio_st.gpio_B==0)&&(ztemt_gpio_st.gpio_C==0)){
	  ztemt_hw_id=HW_C;
	}else if ((ztemt_gpio_st.gpio_A==1)&&(ztemt_gpio_st.gpio_B==1)&&(ztemt_gpio_st.gpio_C==0)){
	  ztemt_hw_id=HW_D;
	}else{ 
	  ztemt_hw_id=HW_UN;
	}	
  ztemt_hw_version_debug("hw_id=%d\n",ztemt_hw_id);
  return ztemt_hw_id;
}

static void ztemt_get_hw_version(char* result)
{
    int hw_id;
    if(!result)
		return;

    hw_id = ztemt_get_hw_id();
	
    if(hw_id != HW_UN){
        strcpy(result,hardware_id_map[hw_id].hw_ver); 
     }else
	    sprintf(result, "%s","unknow");
}

static ssize_t ztemt_hw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    ztemt_get_hw_version(buf);
    //printk("%s : %d : ztemt_hw_version=%s\n",__func__,__LINE__,buf);
    ztemt_hw_version_debug("ztemt_hw_version=%s\n",buf);
    return sprintf(buf,"%s\n",buf);
}

static DEVICE_ATTR(ztemt_hw_version, 0664, ztemt_hw_version_show, NULL);

#endif /*CONFIG_ZTEMT_HW_VERSION_GPIO*/



#ifdef CONFIG_ZTEMT_HW_VERSION_ADC

#ifdef CONFIG_ZTEMT_HW_VERSION_NX601J
static const struct hardware_id_map_st hardware_id_map[] = {
	{0, 200,NX601J_HW_A,"ZTEMT_NX601J_A"},  //id_mv=0
	{700, 1100,NX601J_HW_B,"ZTEMT_NX601J_B"},  //id_mv=900
};
#elif defined CONFIG_ZTEMT_HW_VERSION_NX504J
static const struct hardware_id_map_st hardware_id_map[] = {
	{0, 200,NX504J_HW_A,"ZTEMT_NX504J_A"},  //id_mv=0
	{200, 550,NX504J_HW_B,"ZTEMT_NX504J_B"},  //id_mv=416
	{550, 900,NX504J_HW_C,"ZTEMT_NX504J_C"},  //id_mv=720
	{900, 1300,NX504J_HW_D,"ZTEMT_NX504J_D"},  //id_mv=1120
	{1300, 1650,NX504J_HW_E,"ZTEMT_NX504J_E"},  //id_mv=1475
	{1650, 1900,NX504J_HW_F,"ZTEMT_NX504J_F"},  //id_mv=1800
};
#else 
static const struct hardware_id_map_st hardware_id_map[] = {
	{0, 200,HW_A,"ZTEMT_UN_A"},  //id_mv=0
	{700, 1100,HW_B,"ZTEMT_UN_B"},  //id_mv=900
};
#endif

static int ztemt_hw_id = -1;
static int ztemt_hw_mv = 900;

static int  ztemt_board_type_setup(char *param)
{
	int magic_num = 0;
    get_option(&param, &magic_num);
	ztemt_hw_mv = magic_num;
    return 0;
}
early_param("board_type", ztemt_board_type_setup);

static int32_t ztemt_get_hardware_type(const struct hardware_id_map_st *pts,
		uint32_t tablesize, int input)
{
	uint32_t i = 0;

	if ( pts == NULL )
		return -EINVAL;

	while (i < tablesize) {
		if ( (pts[i].low_mv <= input) && (input <= pts[i].high_mv) ) 
			break;
		else 
			i++;
	}

	if ( i < tablesize ) 
		return pts[i].hw_type;
    else 
		return HW_UN;
}

int ztemt_get_hw_id(void)
{
	if(ztemt_hw_id >= 0)
	    return ztemt_hw_id;
	    
    ztemt_hw_id = ztemt_get_hardware_type(
		                hardware_id_map,
						ARRAY_SIZE(hardware_id_map),
						ztemt_hw_mv);
	
	//printk("hw_id_mv=%d mv hw_id=%d hw_ver=%s\n",
	    //ztemt_hw_mv,ztemt_hw_id,hardware_id_map[ztemt_hw_id].hw_ver);
    ztemt_hw_version_debug("hw_id_mv=%d mv hw_id=%d hw_ver=%s\n",
		ztemt_hw_mv,ztemt_hw_id,hardware_id_map[ztemt_hw_id].hw_ver);
  
	return ztemt_hw_id;
}

EXPORT_SYMBOL_GPL(ztemt_get_hw_id);

void ztemt_get_hw_version(char* result)
{
    int hw_id;
    if(!result)
		return;

    hw_id = ztemt_get_hw_id();
	
    if(hw_id != HW_UN){
        strcpy(result,hardware_id_map[hw_id].hw_ver); 
     }else
	    sprintf(result, "%s","unknow");
}
EXPORT_SYMBOL_GPL(ztemt_get_hw_version);

static ssize_t ztemt_hw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    ztemt_get_hw_version(buf);
    //printk("%s : %d : ztemt_hw_version=%s\n",__func__,__LINE__,buf);
    ztemt_hw_version_debug("ztemt_hw_version=%s\n",buf);
    return sprintf(buf,"%s\n",buf);
}

static DEVICE_ATTR(ztemt_hw_version, 0664, ztemt_hw_version_show, NULL);
#endif /*CONFIG_ZTEMT_HW_VERSION_ADC*/


/*
static struct attribute *ztemt_hw_version_attrs[] = {
    &dev_attr_ztemt_hw_version.attr,
    NULL
};
static const struct attribute_group ztemt_hw_version_attr_group = {
	.attrs = ztemt_hw_version_attrs,
};
*/

static int __devinit
qpnp_ztemt_hw_version_probe(struct spmi_device *spmi)
{
	
	struct qpnp_ztemt_hw_version_chip	*chip;
	int rc = 0;

	chip = devm_kzalloc(&spmi->dev,
			sizeof(struct qpnp_ztemt_hw_version_chip), GFP_KERNEL);
	if (chip == NULL) {
		pr_err("qpnp_ztemt_hw_version_probe : kzalloc() failed.\n");
		return -ENOMEM;
	}
    dev_set_drvdata(&spmi->dev, chip);
	chip->dev = &(spmi->dev);
	chip->spmi = spmi;
	
    rc=sysfs_create_file(&chip->dev->kobj,&dev_attr_ztemt_hw_version.attr);
    //rc=sysfs_create_group(&chip->dev->kobj,&ztemt_hw_version_attr_group);
	return rc;
}

static int __devexit
qpnp_ztemt_hw_version_remove(struct spmi_device *spmi)
{
	struct qpnp_ztemt_hw_version_chip *chip = dev_get_drvdata(&spmi->dev);
  devm_kfree(&spmi->dev,chip);
    
#ifdef CONFIG_ZTEMT_HW_VERSION_GPIO
  gpio_free(GPIO_A);
  gpio_free(GPIO_B);
  gpio_free(GPIO_C);
#endif
  
	return 0;
}

static struct of_device_id qpnp_ztemt_hw_version_match_table[] = {
	{ .compatible = QPNP_ZTEMT_HW_VERSION_DEV_NAME, },
	{}
};

static struct spmi_driver qpnp_ztemt_hw_version_driver = {
	.probe		= qpnp_ztemt_hw_version_probe,
	.remove		= __devexit_p(qpnp_ztemt_hw_version_remove),
	.driver		= {
		.name		    = QPNP_ZTEMT_HW_VERSION_DEV_NAME,
		.owner		    = THIS_MODULE,
		.of_match_table	= qpnp_ztemt_hw_version_match_table,
	},
};

/**
 * qpnp_ztemt_hw_version_init() - register spmi driver for qpnp-ztemt_hw_version
 */
int __init
qpnp_ztemt_hw_version_init(void)
{
	return spmi_driver_register(&qpnp_ztemt_hw_version_driver);
}
module_init(qpnp_ztemt_hw_version_init);

static void __exit
qpnp_ztemt_hw_version_exit(void)
{
	spmi_driver_unregister(&qpnp_ztemt_hw_version_driver);
}
module_exit(qpnp_ztemt_hw_version_exit);

MODULE_DESCRIPTION("qpnp ztemt_hw_version driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" QPNP_ZTEMT_HW_VERSION_DEV_NAME);
