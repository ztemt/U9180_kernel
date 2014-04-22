#ifndef __ZTEMT_HW_VERSION_H__
#define __ZTEMT_HW_VERSION_H__


#ifdef CONFIG_ZTEMT_HW_VERSION_ADC

#ifdef CONFIG_ZTEMT_HW_VERSION_NX601J
typedef enum
{
	NX601J_HW_A,
	NX601J_HW_B,
	HW_UN// unknow, fail read
}hw_version_type;
#elif defined CONFIG_ZTEMT_HW_VERSION_NX504J
typedef enum
{
	NX504J_HW_A,
	NX504J_HW_B,
	NX504J_HW_C,
	NX504J_HW_D,
	NX504J_HW_E,
	NX504J_HW_F,
	HW_UN// unknow, fail read
}hw_version_type;
#else
typedef enum
{
	HW_A,
	HW_B,
	HW_UN// unknow, fail read
}hw_version_type;
#endif

struct hardware_id_map_st {
	int low_mv;
	int high_mv;
	hw_version_type hw_type;
	char hw_ver[20];
};
#endif /*CONFIG_ZTEMT_HW_VERSION_ADC*/


//#define CONFIG_ZTEMT_HW_VERSION_GPIO
#ifdef CONFIG_ZTEMT_HW_VERSION_GPIO

typedef enum
{
	HW_A,
	HW_B,
	HW_C,
	HW_D,
	HW_UN// unknow, fail read
}hw_version_type;

struct hardware_id_map_st {
	hw_version_type hw_type;
	char hw_ver[20];
};

#endif /*CONFIG_ZTEMT_HW_VERSION_GPIO*/


#endif
