ccflags-y += -I$(srctree)/drivers/misc/mediatek/alsps/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/mach/
obj-y	:=  epl259x_wt.o

ifeq ($(CONFIG_LGE_SENSORS),y)
ccflags-y += -DCONFIG_LGE_SENSORS
endif
