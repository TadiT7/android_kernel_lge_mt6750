/******************************************************************************
 * mt6575_vibrator.c - MT6575 Android Linux Vibrator Device Driver
 *
 * Copyright 2009-2010 MediaTek Co.,Ltd.
 *
 * DESCRIPTION:
 *     This file provid the other drivers vibrator relative functions
 *
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/workqueue.h>

#include "timed_output.h"
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/kernel.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include <linux/jiffies.h>
#include <linux/timer.h>

/* #include <mach/mt6577_pm_ldo.h> */
#include "cust_vibrator.h"
#include "vibrator_hal.h"
#include "vibrator.h"



#ifndef CONFIG_PINCTRL_MT6797
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>

#endif



#include <mt-plat/mt_pwm.h>
#include <mt-plat/upmu_common.h>

#define VERSION					        "v 0.1"
#define VIB_DEVICE				"sm100"

typedef enum tagDTS_GPIO_STATE {
	DTS_GPIO_STATE_GPIO_EN = 0,	/* mode_te_gpio */
	DTS_GPIO_STATE_GPIO_DISABLE,
	DTS_GPIO_STATE_PWM_PIN,	/* mode_te_te */
	DTS_GPIO_STATE_MAX,	/* for array size */
} DTS_GPIO_STATE;

static struct pinctrl *this_pctrl; /* static pinctrl instance */

/* DTS state mapping name */
static const char *this_state_name[DTS_GPIO_STATE_MAX] = {
	"vibrator_enable",
	"vibrator_disable",
	"vibrator_pwm"
};

static int debug_enable_vib_hal = 1;
/* #define pr_fmt(fmt) "[vibrator]"fmt */
#define VIB_DEBUG(format, args...) do { \
	if (debug_enable_vib_hal) {\
		pr_debug(format, ##args);\
	} \
} while (0)

/******************************************************************************
Error Code No.
******************************************************************************/
#define RSUCCESS        0

/******************************************************************************
Debug Message Settings
******************************************************************************/

/* Debug message event */
#define DBG_EVT_NONE		0x00000000	/* No event */
#define DBG_EVT_INT			0x00000001	/* Interrupt related event */
#define DBG_EVT_TASKLET		0x00000002	/* Tasklet related event */

#define DBG_EVT_ALL			0xffffffff

#define DBG_EVT_MASK		(DBG_EVT_TASKLET)

#if 1
#define MSG(evt, fmt, args...) \
do {	\
	if ((DBG_EVT_##evt) & DBG_EVT_MASK) { \
		VIB_DEBUG(fmt, ##args); \
	} \
} while (0)

#define MSG_FUNC_ENTRY(f)	MSG(FUC, "<FUN_ENT>: %s\n", __func__)
#else
#define MSG(evt, fmt, args...) do {} while (0)
#define MSG_FUNC_ENTRY(f)	   do {} while (0)
#endif

/******************************************************************************
Global Definations
******************************************************************************/
static struct workqueue_struct *vibrator_queue;
static struct work_struct vibrator_work;
static struct hrtimer vibe_timer;
static spinlock_t vibe_lock;
static int vibe_state;
static int ldo_state;
static int shutdown_flag;

struct vibrator_hw *sm100_pvib_cust = NULL;


/* #define pr_fmt(fmt) "[vibrator]"fmt */
#define VIB_DEBUG(format, args...) do { \
	if (debug_enable_vib_hal) {\
		pr_debug(format, ##args);\
	} \
} while (0)



static struct vibrator_hw sm100_cust_vibrator_hw = {
	.vib_timer = 25,
  #ifdef CUST_VIBR_LIMIT
	.vib_limit = 9,
  #endif
  #ifdef CUST_VIBR_VOL
	.vib_vol = 0x5,
  #endif
};


struct pwm_spec_config sm100_pwm_config = {
	.pwm_no = 2,
	.mode = PWM_MODE_FIFO,
	.clk_div = CLK_DIV2,
	.clk_src = PWM_CLK_NEW_MODE_BLOCK,
	.pmic_pad = 0,
	.PWM_MODE_FIFO_REGS.IDLE_VALUE = IDLE_FALSE,
	.PWM_MODE_FIFO_REGS.GUARD_VALUE = GUARD_FALSE,
	.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 31,
	.PWM_MODE_FIFO_REGS.HDURATION = 25,	/* 1 microseconds, assume clock source is 26M */
	.PWM_MODE_FIFO_REGS.LDURATION = 15,
	.PWM_MODE_FIFO_REGS.GDURATION = 0,
	.PWM_MODE_FIFO_REGS.SEND_DATA0 = -1,
	.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0,
	.PWM_MODE_FIFO_REGS.WAVE_NUM = 0,
};

struct vibrator_hw *sm100_get_cust_vibrator_hw(void)
{
	return &sm100_cust_vibrator_hw;
}

static long _set_state(const char *name)
{
	long ret = 0;
	struct pinctrl_state *pState = 0;

	BUG_ON(!this_pctrl);

	pState = pinctrl_lookup_state(this_pctrl, name);
	if (IS_ERR(pState)) {
		pr_err("lookup state '%s' state= failed\n", name);
		ret = PTR_ERR(pState);
		goto exit;
	}

	/* select state! */
	pinctrl_select_state(this_pctrl, pState);

exit:
	return ret; /* Good! */
}

long sm100_dts_gpio_init(struct platform_device *pdev)
{
	long ret = 0;
	struct pinctrl *pctrl;

	/* retrieve */
	pctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pctrl)) {
		dev_err(&pdev->dev, "Cannot find disp pinctrl!");
		ret = PTR_ERR(pctrl);
		goto exit;
	}

	this_pctrl = pctrl;

exit:
	return ret;
}

long sm100_dts_gpio_select_state(DTS_GPIO_STATE s)
{
	BUG_ON(!((unsigned int)(s) < (unsigned int)(DTS_GPIO_STATE_MAX)));
	return _set_state(this_state_name[s]);
}

void sm100_vibr_Enable_HW(void)
{
	pmic_set_register_value(PMIC_RG_VIBR_EN, 1);	/* [bit 1]: VIBR_EN,  1=enable */
sm100_dts_gpio_select_state(DTS_GPIO_STATE_GPIO_EN);
	mt_set_intr_enable(0);
	mt_set_intr_enable(1);
	mt_pwm_26M_clk_enable_hal(1);
	pwm_set_spec_config(&sm100_pwm_config);
	mt_pwm_dump_regs();

}

void sm100_vibr_Disable_HW(void)
{
	sm100_dts_gpio_select_state(DTS_GPIO_STATE_GPIO_DISABLE);
	mt_pwm_disable(sm100_pwm_config.pwm_no, sm100_pwm_config.pmic_pad);
}


/******************************************
* Set RG_VIBR_VOSEL	Output voltage select
*  hw->vib_vol:  Voltage selection
* 3'b000: 1.3V
* 3'b001: 1.5V
* 3'b010: 1.8V
* 3'b011: 2.0V
* 3'b100: 2.5V
* 3'b101: 2.8V
* 3'b110: 3.0V
* 3'b111: 3.3V
*******************************************/
struct vibrator_hw *sm100_get_cust_vibrator_dtsi(void)
{
	int ret;
	struct device_node *led_node = NULL;

	if (sm100_pvib_cust == NULL) {
		sm100_pvib_cust = kmalloc(sizeof(struct vibrator_hw), GFP_KERNEL);
		if (sm100_pvib_cust == NULL) {
			VIB_DEBUG("get_cust_vibrator_dtsi kmalloc fail\n");
			goto out;
		}

		led_node =
		    of_find_compatible_node(NULL, NULL, "mediatek,vibrator");
		if (!led_node) {
			VIB_DEBUG("Cannot find vibrator node from dts\n");
			kfree(sm100_pvib_cust);
			sm100_pvib_cust = NULL;
			goto out;
		} else {
			ret =
			    of_property_read_u32(led_node, "vib_timer",
						 &(sm100_pvib_cust->vib_timer));
			if (!ret) {
				VIB_DEBUG
				    ("The vibrator timer from dts is : %d\n",
				     sm100_pvib_cust->vib_timer);
			} else {
				sm100_pvib_cust->vib_timer = 25;
			}
#ifdef CUST_VIBR_LIMIT
			ret =
			    of_property_read_u32(led_node, "vib_limit",
						 &(sm100_pvib_cust->vib_limit));
			if (!ret) {
				VIB_DEBUG
				    ("The vibrator limit from dts is : %d\n",
				     sm100_pvib_cust->vib_limit);
			} else {
				sm100_pvib_cust->vib_limit = 9;
			}
#endif

#ifdef CUST_VIBR_VOL
			ret =
			    of_property_read_u32(led_node, "vib_vol",
						 &(sm100_pvib_cust->vib_vol));
			if (!ret) {
				VIB_DEBUG("The vibrator vol from dts is : %d\n",
					  sm100_pvib_cust->vib_vol);
			} else {
				sm100_pvib_cust->vib_vol = 0x05;
			}
#endif
		}
	}

 out:
	return sm100_pvib_cust;
}

void sm100_vibr_power_set(void)
{
#ifdef CUST_VIBR_VOL
	pmic_set_register_value(PMIC_RG_VIBR_VOSEL, 0x06);
	pmic_set_register_value(PMIC_RG_VIBR_EN, 1);
#endif
}

struct vibrator_hw *sm100_mt_get_cust_vibrator_hw(void)
{
	struct vibrator_hw *hw = sm100_get_cust_vibrator_dtsi();
	return hw;
}

static int vibr_Enable(void)
{

	if (!ldo_state) {
		sm100_vibr_Enable_HW();
		ldo_state = 1;
	}

	return 0;
}

static int sm100_vibr_Disable(void)
{

	if (ldo_state) {
		sm100_vibr_Disable_HW();
		ldo_state = 0;
	}

	return 0;
}

static void update_vibrator(struct work_struct *work)
{
	if (!vibe_state)
		sm100_vibr_Disable();
	else
		vibr_Enable();
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);

		return ktime_to_ms(r);
	} else
		return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	unsigned long flags;

#if 1
	struct vibrator_hw *hw = sm100_mt_get_cust_vibrator_hw();
#endif

	VIB_DEBUG("vibrator_enable: vibrator first in value = %d\n", value);

	spin_lock_irqsave(&vibe_lock, flags);
	while (hrtimer_cancel(&vibe_timer))
		VIB_DEBUG("vibrator_enable: try to cancel hrtimer\n");

	if (value == 0 || shutdown_flag == 1) {
		vibe_state = 0;
	} else{
#if 1
		VIB_DEBUG("vibrator_enable: vibrator cust timer: %d\n",
			  hw->vib_timer);
#ifdef CUST_VIBR_LIMIT
		if (value > hw->vib_limit && value < hw->vib_timer)
#else
		if (value >= 10 && value < hw->vib_timer)
#endif
			value = hw->vib_timer;
#endif

		value = (value > 15000 ? 15000 : value);
		vibe_state = 1;
		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&vibe_lock, flags);
	queue_work(vibrator_queue, &vibrator_work);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	vibe_state = 0;
	queue_work(vibrator_queue, &vibrator_work);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev sm100_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};


static int vib_probe(struct device  *pdev)
{
	struct platform_device *dev;
	long dts_gpio_state = 0;

	dev = to_platform_device(pdev);
	/* repo call DTS gpio module, if not necessary, invoke nothing */
	dts_gpio_state = sm100_dts_gpio_init(dev);
	if (dts_gpio_state != 0)
		dev_err(&dev->dev, "retrieve GPIO DTS failed.");
	return 0;
}

static int vib_remove(struct device *pdev)
{
	return 0;
}

static void vib_shutdown(struct device *pdev)
{
	unsigned long flags;

	VIB_DEBUG("vib_shutdown: enter!\n");
	spin_lock_irqsave(&vibe_lock, flags);
	shutdown_flag = 1;
	if (vibe_state) {
		VIB_DEBUG("vib_shutdown: vibrator will disable\n");
		vibe_state = 0;
		spin_unlock_irqrestore(&vibe_lock, flags);
		sm100_vibr_Disable();
	} else {
		spin_unlock_irqrestore(&vibe_lock, flags);
	}
}

static const struct of_device_id sm100_of_ids[] = {
	{.compatible = "mediatek,sm100",},
	{}
};
/******************************************************************************
Device driver structure
*****************************************************************************/
static struct platform_driver vibrator_driver = {
	.driver = {
	.probe = vib_probe,
	.remove = vib_remove,
	.shutdown = vib_shutdown,
	.name = VIB_DEVICE,
	.owner = THIS_MODULE,
	.of_match_table = sm100_of_ids,
		   },
};

static struct platform_device vibrator_device = {
	.name = "sm100_vibrator",
	.id = -1,
};

static ssize_t store_vibr_on(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	if (buf != NULL && size != 0) {
		/* VIB_DEBUG("buf is %s and size is %d\n", buf, size); */
		if (buf[0] == '0')
			sm100_vibr_Disable();
		else
			vibr_Enable();
	}
	return size;
}

static DEVICE_ATTR(vibr_on, 0220, NULL, store_vibr_on);

/******************************************************************************
 * vib_mod_init
 *
 * DESCRIPTION:
 *   Register the vibrator device driver !
 *
 * PARAMETERS:
 *   None
 *
 * RETURNS:
 *   None
 *
 * NOTES:
 *   RSUCCESS : Success
 *
 ******************************************************************************/

static int __init sm100_vib_mod_init(void)
{
	s32 ret;

	VIB_DEBUG("MediaTek MTK vibrator driver register, version %s\n",
		  VERSION);
	/* set vibr voltage if needs.  Before MT6320 vibr default voltage=2.8v,
	   but in MT6323 vibr default voltage=1.2v */
	sm100_vibr_power_set();
	ret = platform_device_register(&vibrator_device);
	if (ret != 0) {
		VIB_DEBUG("Unable to register vibrator device (%d)\n", ret);
		return ret;
	}

	vibrator_queue = create_singlethread_workqueue(VIB_DEVICE);
	if (!vibrator_queue) {
		VIB_DEBUG("Unable to create workqueue\n");
		return -ENODATA;
	}
	INIT_WORK(&vibrator_work, update_vibrator);

	spin_lock_init(&vibe_lock);
	shutdown_flag = 0;
	vibe_state = 0;
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&sm100_vibrator);

	ret = platform_driver_register(&vibrator_driver);

	if (ret) {
		VIB_DEBUG("Unable to register vibrator driver (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(sm100_vibrator.dev, &dev_attr_vibr_on);
	if (ret)
		VIB_DEBUG("device_create_file vibr_on fail!\n");

	VIB_DEBUG("vib_mod_init Done\n");

	return RSUCCESS;
}

/******************************************************************************
 * vib_mod_exit
 *
 * DESCRIPTION:
 *   Free the device driver !
 *
 * PARAMETERS:
 *   None
 *
 * RETURNS:
 *   None
 *
 * NOTES:
 *   None
 *
 ******************************************************************************/

static void sm100_vib_mod_exit(void)
{
	VIB_DEBUG("MediaTek MTK vibrator driver unregister, version %s\n",
		  VERSION);
	if (vibrator_queue)
		destroy_workqueue(vibrator_queue);
	VIB_DEBUG("vib_mod_exit Done\n");
}

module_init(sm100_vib_mod_init);
module_exit(sm100_vib_mod_exit);
MODULE_AUTHOR("MediaTek Inc.");
MODULE_DESCRIPTION("MTK Vibrator Driver (VIB)");
MODULE_LICENSE("GPL");
