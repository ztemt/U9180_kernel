#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

# obj-$(CONFIG_ZTEMT_TOUCHSCREEN_SYNAPTICS_DSX) += zte-ctp-syn-board.o
obj-$(CONFIG_ZTEMT_TOUCHSCREEN_SYNAPTICS_DSX_I2C) += synaptics_dsx_i2c.o
obj-$(CONFIG_ZTEMT_TOUCHSCREEN_SYNAPTICS_DSX_RMI_DEV) += synaptics_dsx_rmi_dev.o
obj-$(CONFIG_ZTEMT_TOUCHSCREEN_SYNAPTICS_DSX_FW_UPDATE) += synaptics_dsx_fw_update.o
obj-$(CONFIG_ZTEMT_TOUCHSCREEN_SYNAPTICS_DSX_TEST_REPORTING) += synaptics_dsx_test_reporting.o

ifeq ($(CONFIG_ZTEMT_TOUCHSCREEN_SYNAPTICS_DSX_DEBUG),y)
CFLAGS_zte-ctp-syn-board.o += -DDEBUG
CFLAGS_synaptics_dsx_i2c.o += -DDEBUG
CFLAGS_synaptics_dsx_rmi_dev.o += -DDEBUG
CFLAGS_synaptics_dsx_fw_update.o += -DDEBUG
CFLAGS_synaptics_dsx_test_reporting.o += -DDEBUG
endif
