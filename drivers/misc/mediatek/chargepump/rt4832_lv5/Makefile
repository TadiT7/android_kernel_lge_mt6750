MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/chargepump/inc/

ccflags-y += -Wno-unused-variable
ccflags-y += -Wno-unused-label
ccflags-y += -Wno-unused-value
ccflags-y += -Wno-unused-function

obj-y	:=  rt4832_driver.o

