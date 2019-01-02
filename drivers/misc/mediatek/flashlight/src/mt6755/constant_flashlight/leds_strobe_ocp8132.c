#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
//#include "kd_camera_hw.h"
//#include <cust_gpio_usage.h>
//#include <cust_i2c.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
//#include <linux/xlog.h>
#include <linux/version.h>

//#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
//#else
//#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
//#include <linux/semaphore.h>
//#else
//#include <asm/semaphore.h>
//#endif
//#endif

#include <linux/i2c.h>
#include <linux/leds.h>
#include <mt-plat/mt_gpio.h>
#include <soc/mediatek/lge/board_lge.h>

/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        pr_warning(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __FUNCTION__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=0;  //bug253624 tiantian.wt 20170330 modify flashligth is torch mode on default
static int g_timeOutTimeMs = 0;
static BOOL g_PwmStop = 0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif

static struct work_struct workTimeOut;
static struct hrtimer g_PwmTimer;
static BOOL OCP8135 = true;

#define FLASH_GPIO_ENF (83 | 0x80000000) //GPIO_CAMERA_FLASH_EXT1_PIN //flash_en
#define FLASH_GPIO_ENM (108 | 0x80000000) //GPIO_CAMERA_FLASH_EXT2_PIN //flash_stb
#define UDELAY_TIMES  15   //5khz<1/UDELAY_TIMS<200khz

static void FL_setduty2GPIO(int duty)
{
	int i = 0;
	int flash_level= 0;
	PK_DBG("FL_setduty2GPIO duty=%d\n", duty);

	switch(duty) {
		case 1:
			flash_level = 6;
			break;
		case 2:
			flash_level = 9;
			break;
		case 3:
			flash_level = 12;
			break;
		case 4:
			flash_level = 15; //max level
			break;
		default:
			PK_DBG("FL_setduty2GPIO non-value\n");
			break;
	}

	for(i = flash_level;i > 0;i--)
	{
		mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ONE);
		udelay(100);
		mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ZERO);
		udelay(100);
	}
	mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ONE);
}
/*
g_duty = 0 , torch mode
g_duty > 0 , flash mode  ---> g_duty range [1,16]
*/
static int FL_Enable(void)
{
	PK_DBG(" FL_Enable g_duty=%d\n",g_duty);
	//ktime_t ktime;
	if(g_duty==0)//torch
	{
		mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ZERO);
		mt_set_gpio_out(FLASH_GPIO_ENM, GPIO_OUT_ONE);
		mdelay(15);  //must >=5ms
		if(!OCP8135) g_PwmStop=0;
		//ktime = ktime_set( 0, UDELAY_TIMES*1000 ); //ktime_set( s,ns )
		//hrtimer_start( &g_PwmTimer, ktime, HRTIMER_MODE_REL );
		PK_DBG(" FL_Enable line=%d\n",__LINE__);
	}
	else//flash
	{
		mt_set_gpio_out(FLASH_GPIO_ENM, GPIO_OUT_ZERO);
		if(!OCP8135) g_PwmStop=0;
		//ktime = ktime_set( 0, UDELAY_TIMES*1000 ); //ktime_set( s,ns )
		//hrtimer_start( &g_PwmTimer, ktime, HRTIMER_MODE_REL );
		if(!OCP8135) {
			mdelay(1);
			mt_set_gpio_mode(FLASH_GPIO_ENF, GPIO_MODE_00);
			mt_set_gpio_dir(FLASH_GPIO_ENF, GPIO_DIR_OUT);
			mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ONE);
		} else {
			FL_setduty2GPIO(g_duty);
		}
		PK_DBG(" FL_Enable line=%d\n",__LINE__);
	}

	return 0;
}



static int FL_Disable(void)
{
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
//	while(hrtimer_cancel( &g_PwmTimer )){
//	PK_DBG(" FL_hrtimer_cancel line=%d\n",__LINE__);
//	}
	mt_set_gpio_out(FLASH_GPIO_ENM, GPIO_OUT_ZERO);
	mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ZERO);
	return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
	g_duty = duty;
	return 0;
}




static int FL_Init(void)
{
	mt_set_gpio_mode(FLASH_GPIO_ENM, GPIO_MODE_00);
	mt_set_gpio_dir(FLASH_GPIO_ENM, GPIO_DIR_OUT);
	mt_set_gpio_out(FLASH_GPIO_ENM, GPIO_OUT_ZERO);

	mt_set_gpio_mode(FLASH_GPIO_ENF, GPIO_MODE_00);
	mt_set_gpio_dir(FLASH_GPIO_ENF, GPIO_DIR_OUT);
	mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ZERO);
	PK_DBG(" FL_Init line=%d\n",__LINE__);
	return 0;
}


static int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
static void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}

enum hrtimer_restart GpioEnmPwmFuncCallback(struct hrtimer *timer)
{
//	PK_DBG("GpioEnmPwmFunc_callback g_PwmStop=%d\n",g_PwmStop);
	ktime_t ktime;
	if(g_PwmStop==0)
	{
		mt_set_gpio_mode(FLASH_GPIO_ENM, GPIO_MODE_00);
		mt_set_gpio_dir(FLASH_GPIO_ENM, GPIO_DIR_OUT);
		mt_set_gpio_out(FLASH_GPIO_ENM, GPIO_OUT_ZERO);
	}
	else
	{
		mt_set_gpio_mode(FLASH_GPIO_ENM, GPIO_MODE_00);
		mt_set_gpio_dir(FLASH_GPIO_ENM, GPIO_DIR_OUT);
		mt_set_gpio_out(FLASH_GPIO_ENM, GPIO_OUT_ONE);
	}

	g_PwmStop=~g_PwmStop;

	ktime = ktime_set( 0, UDELAY_TIMES*1000 ); //ktime_set( s,ns )
	hrtimer_forward_now(&g_PwmTimer, ktime);
	return HRTIMER_RESTART;
}

/*static void GpioEnmPwmInt(void)
{
	hrtimer_init( &g_PwmTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_PwmTimer.function=GpioEnmPwmFuncCallback;
}*/

static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("LM3642 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift,(int)arg);
	switch(cmd)
	{

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
			g_timeOutTimeMs=arg;
			break;


		case FLASH_IOC_SET_DUTY :
			PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
			FL_dim_duty(arg);
			break;


		case FLASH_IOC_SET_STEP:
			PK_DBG("FLASH_IOC_SET_STEP: %d\n",(int)arg);

			break;

		case FLASH_IOC_SET_ONOFF :
			PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
			if(arg==1)
			{
				int s;
				int ms;
				if(g_timeOutTimeMs>1000)
				{
					s = g_timeOutTimeMs/1000;
					ms = g_timeOutTimeMs - s*1000;
				}
				else
				{
					s = 0;
					ms = g_timeOutTimeMs;
				}

				if(g_timeOutTimeMs!=0)
				{
					ktime_t ktime;
					ktime = ktime_set( s, ms*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
				}
				FL_Enable();
			}
			else
			{
				FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
			}
			break;
		default :
			PK_DBG(" No such command \n");
			i4RetValue = -EPERM;
			break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		  timerInit();
//		  GpioEnmPwmInt();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

        FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }

    if(lge_get_board_revno() < HW_REV_B) OCP8135 = false;

    return 0;
}





