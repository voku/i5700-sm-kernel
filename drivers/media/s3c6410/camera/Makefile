#
#   Makefile S3C Camera driver
#
#   Copyright(c) 2004-2006, Samsung Electronics, Co., Ltd.
#

ifeq ($(CONFIG_MACH_BIGBANG),y)
EXTRA_CFLAGS 	+= -DCONFIG_VIDEO_SAMSUNG_S5K4CA -DCONFIG_VIDEO_SAMSUNG_PO4010 
endif
ifeq ($(CONFIG_MACH_CAPELA),y)
EXTRA_CFLAGS 	+= -DCONFIG_VIDEO_SAMSUNG_S5K4CA  
endif
ifeq ($(CONFIG_MACH_INSTINCTQ),y)
EXTRA_CFLAGS 	+= -DCONFIG_VIDEO_SAMSUNG_S5K4CA -DCONFIG_FLASH_AAT1271A 
#EXTRA_CFLAGS 	+= -DCONFIG_VIDEO_SAMSUNG_S5K4CA
endif
ifeq ($(CONFIG_MACH_NAOS),y)
EXTRA_CFLAGS 	+= -DCONFIG_VIDEO_SAMSUNG_S5K4CA  
endif
ifeq ($(CONFIG_MACH_SPICA),y)
EXTRA_CFLAGS 	+= -DCONFIG_VIDEO_SAMSUNG_S5K4CA  
endif
ifeq ($(CONFIG_MACH_SATURN),y)
EXTRA_CFLAGS 	+= -DCONFIG_VIDEO_SAMSUNG_S5K4CA  
endif
ifeq ($(CONFIG_MACH_ORION_6410),y)
EXTRA_CFLAGS 	+= -DCONFIG_VIDEO_SAMSUNG_S5K4CA  
endif
ifeq ($(CONFIG_MACH_CYGNUS),y)
EXTRA_CFLAGS 	+= -DCONFIG_VIDEO_SAMSUNG_S5K4CA -DCONFIG_FLASH_AAT1271A 
#EXTRA_CFLAGS 	+= -DCONFIG_VIDEO_SAMSUNG_S5K4CA
endif
ifeq ($(CONFIG_MACH_BONANZA),y)
EXTRA_CFLAGS 	+= -DCONFIG_VIDEO_SAMSUNG_CE131  
endif

SENSOR_DIR		= samsung

#
# S3C Camera Interface
#

obj-m			+= s3c_camera.o

s3c_camera-y	+= s3c_camif.o s3c_camera_driver.o

#
# 3M Camera Sensor
#

s3c_camera-y += $(SENSOR_DIR)/s5k4ca.o

#
# 5M Camera Sensor
#

ifeq ($(CONFIG_MACH_CAPELA),y)
#s3c_camera-y += $(SENSOR_DIR)/ait848.o
endif
ifeq ($(CONFIG_MACH_INSTINCTQ),y)
#s3c_camera-y += $(SENSOR_DIR)/ait848.o
endif
ifeq ($(CONFIG_MACH_NAOS),y)
#s3c_camera-y += $(SENSOR_DIR)/ait848.o
endif
ifeq ($(CONFIG_MACH_SPICA),y)
#s3c_camera-y += $(SENSOR_DIR)/ait848.o
endif
ifeq ($(CONFIG_MACH_ORION_6410),y)
#s3c_camera-y += $(SENSOR_DIR)/ait848.o
endif
ifeq ($(CONFIG_MACH_CYGNUS),y)
#s3c_camera-y += $(SENSOR_DIR)/ait848.o
endif
ifeq ($(CONFIG_MACH_BONANZA),y)
s3c_camera-y += $(SENSOR_DIR)/ce131.o
endif
#
# VGA Camera Sensor
#

ifeq ($(CONFIG_MACH_BIGBANG),y)
s3c_camera-y += $(SENSOR_DIR)/po4010.o
endif

#
# Camera Power
#

ifeq ($(CONFIG_MACH_CAPELA),y)
s3c_camera-y += $(SENSOR_DIR)/power/lp8720.o
endif

#
# Camera Flash
#

ifeq ($(CONFIG_MACH_INSTINCTQ),y)
s3c_camera-y += $(SENSOR_DIR)/flash/aat1271a.o
endif
ifeq ($(CONFIG_MACH_CYGNUS),y)
s3c_camera-y += $(SENSOR_DIR)/flash/aat1271a.o
endif

