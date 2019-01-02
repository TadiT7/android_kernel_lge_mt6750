/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */


/* file rt4832.c
   brief This file contains all function implementations for the rt4832 in linux
   this source file refer to MT6572 platform
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <mach/gpio_const.h>
#include <mt_gpio.h>
#include <linux/platform_device.h>

#include <linux/leds.h>
#include <../inc/rt4832.h>
#include <linux/ctype.h>
#include <soc/mediatek/lge/board_lge.h>

#define LCD_LED_MAX                 0x7F
#define LCD_LED_MIN                 0

#define DEFAULT_BRIGHTNESS          0x73
#define RT4832_MIN_VALUE_SETTINGS   10
#define RT4832_MAX_VALUE_SETTINGS   255

#define MIN_MAX_SCALE(x)\
(((x) < RT4832_MIN_VALUE_SETTINGS) ? RT4832_MIN_VALUE_SETTINGS :\
    (((x) > RT4832_MAX_VALUE_SETTINGS) ? RT4832_MAX_VALUE_SETTINGS:(x)))
#define RT4832_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> bitname##__POS)
#define RT4832_SET_BITSLICE(regvar, bitname, val)\
(regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

#define BACKLIHGT_NAME          "charge-pump"
#define RT4832_DEV_NAME         "rt4832"
#define CPD_TAG                 "[ChargePump] "

#define CPD_FUN(f)              pr_err(CPD_TAG"%s\n", __func__)
#define CPD_ERR(fmt, args...)   pr_err(CPD_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define CPD_LOG(fmt, args...)   pr_err(CPD_TAG fmt, ##args)

#define LED_MODE_CUST_LCM       4
#define LED_MODE_CUST_BLS_PWM   5
#ifdef CONFIG_MTK_AAL_SUPPORT
static unsigned char bl_en_data = 0x00;
#endif

#define ENABLE  1
#define DISABLE 0

/* I2C variable */
static struct i2c_client *new_client;
static const struct i2c_device_id rt4832_i2c_id[] = { {RT4832_DEV_NAME, 0}, {} };
static struct i2c_board_info i2c_rt4832 __initdata = { I2C_BOARD_INFO(RT4832_DEV_NAME, 0x11) };

static int rt4832_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

int old_bl_level;
static int led_mode = LED_MODE_CUST_LCM;

#ifdef CONFIG_OF
static const struct of_device_id rt4832_of_match[] = {
    {.compatible = "mediatek,i2c_lcd_bias",},
    {},
};

MODULE_DEVICE_TABLE(of, rt4832_of_match);
#endif

static struct i2c_driver rt4832_driver = {
    .driver = {
           .name = "rt4832",
#ifdef CONFIG_OF
           .of_match_table = rt4832_of_match,
#endif
           },
    .probe = rt4832_driver_probe,
    .id_table = rt4832_i2c_id,
};

/* Flash control */
unsigned char strobe_ctrl;
unsigned char flash_ctrl;
unsigned char flash_status;

#ifndef GPIO_LCD_BL_EN
#define GPIO_LCD_BL_EN              (GPIO100 | 0x80000000)
#define GPIO_LCD_BL_EN_MODE         GPIO_MODE_00
#endif

// Gamma 2.2 Table
unsigned int bright_arr[] = {
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10,                                           // 19
    11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 15, 16, 16, 17, 17, 17, 18, 18,                         // 39
    19, 19, 19, 20, 20, 21, 21, 21, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 26, 26,                         // 59
    27, 28, 29, 31, 32, 34, 35, 36, 38, 39, 41, 42, 44, 45, 46, 48, 49, 51, 52, 53,                         // 79
    55, 56, 58, 59, 61, 62, 63, 65, 66, 68, 69, 70, 72, 73, 75, 76, 78, 79, 80, 82,                         // 99
    83, 85, 86, 87, 89, 90, 92, 93, 95, 97, 100, 102, 105, 108, 110, 113, 116, 118, 121, 124,               // 119
    126, 129, 132, 134, 137, 140, 142, 145, 148, 150, 153, 156, 158, 161, 163, 166, 169, 171, 174, 177,     // 139
    179, 182, 185, 187, 190, 193, 195, 198, 201, 203, 206, 209, 211, 214, 217, 219, 222, 225, 229, 233,     // 159
    237, 241, 246, 250, 254, 258, 263, 267, 271, 275, 279, 284, 288, 292, 296, 301, 305, 309, 313, 317,     // 179
    322, 326, 330, 334, 339, 343, 347, 351, 355, 360, 364, 368, 372, 377, 381, 385, 389, 393, 398, 402,     // 199
    406, 410, 415, 419, 423, 427, 432, 438, 444, 450, 456, 463, 469, 475, 481, 488, 494, 500, 506, 512,     // 219
    519, 525, 531, 537, 544, 550, 556, 562, 568, 575, 581, 587, 593, 600, 606, 612, 618, 624, 631, 637,     // 239
    643, 649, 656, 662, 668, 674, 680, 687, 693, 699, 705, 712, 718, 724, 730, 737                          // 255
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static unsigned char current_brightness;
#endif
static unsigned char is_suspend;

struct semaphore rt4832_lock;

/* generic */
#define RT4832_MAX_RETRY_I2C_XFER   (100)
#define RT4832_I2C_WRITE_DELAY_TIME 1

typedef struct {
    bool bat_exist;
    bool bat_full;
    bool bat_low;
    s32 bat_charging_state;
    s32 bat_vol;
    bool charger_exist;
    s32 pre_charging_current;
    s32 charging_current;
    s32 charger_vol;
    s32 charger_protect_status;
    s32 ISENSE;
    s32 ICharging;
    s32 temperature;
    s32 total_charging_time;
    s32 PRE_charging_time;
    s32 CC_charging_time;
    s32 TOPOFF_charging_time;
    s32 POSTFULL_charging_time;
    s32 charger_type;
    s32 PWR_SRC;
    s32 SOC;
    s32 ADC_BAT_SENSE;
    s32 ADC_I_SENSE;
} PMU_ChargerStruct;

//i2c read routine for API
static char rt4832_i2c_read(struct i2c_client *client, u8 reg_addr, u8 *data, u8 len)
{
#if !defined BMA_USE_BASIC_I2C_FUNC
    s32 dummy;

    if (NULL == client)
        return -1;

    while (0 != len--) {
#ifdef BMA_SMBUS
        dummy = i2c_smbus_read_byte_data(client, reg_addr);
        if (dummy < 0) {
            CPD_ERR("i2c bus read error");
            return -1;
        }
        *data = (u8) (dummy & 0xff);
#else
        dummy = i2c_master_send(client, (char *)&reg_addr, 1);
        if (dummy < 0) {
            CPD_ERR("send dummy is %d", dummy);
            return -1;
        }

        dummy = i2c_master_recv(client, (char *)data, 1);
        if (dummy < 0) {
            CPD_ERR("recv dummy is %d", dummy);
            return -1;
        }
#endif
        reg_addr++;
        data++;
    }
    return 0;
#else
    int retry;

    struct i2c_msg msg[] = {
        {
         .addr = client->addr,
         .flags = 0,
         .len = 1,
         .buf = &reg_addr,
         },

        {
         .addr = client->addr,
         .flags = I2C_M_RD,
         .len = len,
         .buf = data,
         },
    };

    for (retry = 0; retry < RT4832_MAX_RETRY_I2C_XFER; retry++) {
        if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
            break;
        mdelay(RT4832_I2C_WRITE_DELAY_TIME);
    }

    if (RT4832_MAX_RETRY_I2C_XFER <= retry) {
        CPD_ERR("I2C xfer error");
        return -EIO;
    }

    return 0;
#endif
}

// i2c write routine for
static char rt4832_i2c_write(struct i2c_client *client, u8 reg_addr, u8 *data, u8 len)
{
#if !defined BMA_USE_BASIC_I2C_FUNC
    s32 dummy;
#ifndef BMA_SMBUS
    /* u8 buffer[2]; */
#endif

    if (NULL == client)
        return -1;

    while (0 != len--) {
#if 1
        dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
#else
        buffer[0] = reg_addr;
        buffer[1] = *data;
        dummy = i2c_master_send(client, (char *)buffer, 2);
#endif
        reg_addr++;
        data++;
        if (dummy < 0)
            return -1;
    }

#else
    u8 buffer[2];
    int retry;
    struct i2c_msg msg[] = {
        {
         .addr = client->addr,
         .flags = 0,
         .len = 2,
         .buf = buffer,
         },
    };

    while (0 != len--) {
        buffer[0] = reg_addr;
        buffer[1] = *data;
        for (retry = 0; retry < RT4832_MAX_RETRY_I2C_XFER; retry++) {
            if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
                break;
            mdelay(RT4832_I2C_WRITE_DELAY_TIME);
        }
        if (RT4832_MAX_RETRY_I2C_XFER <= retry)
            return -EIO;
        reg_addr++;
        data++;
    }
#endif

    return 0;
}

static int rt4832_smbus_read_byte(struct i2c_client *client,
                  unsigned char reg_addr, unsigned char *data)
{
    return rt4832_i2c_read(client, reg_addr, data, 1);
}

static int rt4832_smbus_write_byte(struct i2c_client *client,
                   unsigned char reg_addr, unsigned char *data)
{
    int ret_val = 0;
    int i = 0;

    ret_val = rt4832_i2c_write(client, reg_addr, data, 1);

    for (i = 0; i < 5; i++) {
        if (ret_val != 0)
            rt4832_i2c_write(client, reg_addr, data, 1);
        else
            return ret_val;
    }
    return ret_val;
}

void chargepump_DSV_on(void)
{
    unsigned char data = 0;

    //data = 0x1E; // Vboost voltage 5.5V and 0ms delay between VPOS and VCPOUT
    data = 0xA2; // Vboost voltage 5.7V and 2ms delay between VPOS and VCPOUT
    rt4832_smbus_write_byte(new_client, 0x0D, &data);
    data = 0x1E; // DSV output voltage is +5.5V, 8.5mV/us
    rt4832_smbus_write_byte(new_client, 0x0E, &data);
    data = 0x1E; // DSV output voltage is -5.5V, 9.9mV/us
    rt4832_smbus_write_byte(new_client, 0x0F, &data);
#ifdef CONFIG_MTK_AAL_SUPPORT
    if(led_mode==LED_MODE_CUST_BLS_PWM)
        data = 0x4A; // DSV i2c control enable + PWN enable
    else
        data = 0x0A; // DSV i2c control enable + PWN disable
#else
    data = 0x0A;
#endif
    rt4832_smbus_write_byte(new_client, 0x09, &data);
    data = 0x48; // DSV VPOS and VCPOUT enable, floating, fast discharging
    rt4832_smbus_write_byte(new_client, 0x0C, &data);
    data = 0x0C;
    rt4832_smbus_write_byte(new_client, 0x08, &data);

    CPD_LOG("%s\n",__func__);
}

void chargepump_DSV_off(void)
{
    unsigned char data = 0;

    data = 0x0C;
    rt4832_smbus_write_byte(new_client, 0x08, &data);
    data = 0x0A; // DSV i2c control enable
    rt4832_smbus_write_byte(new_client, 0x09, &data);
    data = 0x24; // DSV VPOS and VCPOUT disable, grounding, fast discharging
    rt4832_smbus_write_byte(new_client, 0x0C, &data);

    CPD_LOG("%s\n",__func__);
}

void rt4832_dsv_ctrl(unsigned int enable)
{
    if (enable)
        chargepump_DSV_on();
    else
        chargepump_DSV_off();

    CPD_LOG("%s : %s\n",__func__,(enable)? "enable":"disable");
}

void rt4832_dsv_toggle_ctrl(void)
{
    unsigned char data = 0;

    data = 0x68;
    rt4832_smbus_write_byte(new_client, 0x0D, &data);

    data = 0x24;
    rt4832_smbus_write_byte(new_client, 0x0E, &data);
    rt4832_smbus_write_byte(new_client, 0x0F, &data);

    data = 0x01;
    rt4832_smbus_write_byte(new_client, 0x0C, &data);

    //periodic mode
    data = 0x8C;
    rt4832_smbus_write_byte(new_client, 0x08, &data);

    // external pin control
    data = 0x2A;
    rt4832_smbus_write_byte(new_client, 0x09, &data);

    CPD_LOG("%s\n",__func__);
}

void rt4832_dsv_mode_change(int mode)
{
    unsigned char data = 0;

    if(mode)
        data = 0x2A;// VSP/VNP external pin control mode
    else
        data = 0x0A;// VSP/VNP i2c control mode

#ifdef CONFIG_MTK_AAL_SUPPORT
    if(led_mode == LED_MODE_CUST_BLS_PWM)
        data |= 0x40;
#endif

    rt4832_smbus_write_byte(new_client, 0x09, &data);

    CPD_LOG("%s : %s control mode\n",__func__,(mode)? "pin":"i2c");
}

void rt4832_dsv_shutdown(void)
{
    unsigned char data = 0;

    data = 0x0C;// default setting
    rt4832_smbus_write_byte(new_client, 0x08, &data);

    data = 0x2A;
#ifdef CONFIG_MTK_AAL_SUPPORT
    if(led_mode==LED_MODE_CUST_BLS_PWM)
        data |= 0x40;
#endif
    rt4832_smbus_write_byte(new_client, 0x09, &data);

    data = 0x24;// ground, fast discharging on
    rt4832_smbus_write_byte(new_client, 0x0C, &data);

    CPD_LOG("%s\n",__func__);
}

void set_rt4832_switching_freq(int sw_freq)
{
    unsigned char data = 0;

    rt4832_smbus_read_byte(new_client, 0x03, &data);

    if(sw_freq == 1)
        data |= 0x80; // push 1 in bl sw frequency bit
    else
        data &= 0x7F; // push 0 in bl sw frequency bit

    rt4832_smbus_write_byte(new_client, 0x03, &data);

    CPD_LOG("%s\n",__func__);
}

int get_rt4832_switching_freq(void)
{
    int ret = 0;
    unsigned char data = 0;

    rt4832_smbus_read_byte(new_client, 0x03, &data);

    if( (data >> 7) == 1)
        ret = 1;// 500kHz
    else
        ret = 0;// 1Mhz

    CPD_LOG("%s : %s\n",__func__,(ret)? "500MHZ":"1KHZ");

    return ret;
}

bool check_charger_pump_vendor(void)
{
    int err = 0;
    unsigned char data = 0;

    err = rt4832_smbus_read_byte(new_client, 0x01, &data);

    if (err < 0)
        CPD_ERR("read charge-pump vendor id fail\n");

    if ((data & 0x03) == 0x03)  /* Richtek */
        err =  0;
    else
        err =  1;

    CPD_LOG("%s : %d\n",__func__,err);

    return err;
}

int chargepump_set_backlight_level(unsigned int level)
{
    int ret = 0;
    unsigned int level_data = 0;
    unsigned int results = 0;
    unsigned char reg_data = 0;
    unsigned char lsb_data = 0;
    unsigned char msb_data = 0;
    unsigned char read_data = 0;

    old_bl_level = level;
    CPD_LOG("%s, level : %d\n",__func__,level);

    if (level) {
        level = MIN_MAX_SCALE(level);
        level_data = bright_arr[level];

        CPD_LOG("%s, level_data : %d\n",__func__,level_data);

        if (is_suspend == ENABLE) {
            is_suspend = DISABLE;
            mdelay(10);

            results = down_interruptible(&rt4832_lock);

            reg_data = 0x00;
            rt4832_smbus_write_byte(new_client, 0x03, &reg_data);

            msb_data = (level_data >> 8) & 0x03; // 2bit
            lsb_data = (level_data) & 0xFF; // 8bit

            rt4832_smbus_write_byte(new_client, 0x04, &msb_data);
            rt4832_smbus_write_byte(new_client, 0x05, &lsb_data);

            CPD_LOG("wake up I2C check = %d\n",ret);
            CPD_LOG("brightness Setting[reg0x04][MSB:0x%x][reg0x05][LSB:0x%x]\n",msb_data,lsb_data);

            rt4832_smbus_read_byte(new_client, 0x0A, &reg_data);
            // BL_EN enable, BLED1&2 enable
            reg_data |= 0x19;
            rt4832_smbus_write_byte(new_client, 0x0A, &reg_data);
            mdelay(30);

            // BL Frequency 500KHZ, 10 bit resolution, BL Ramp rate 100ms
            reg_data = 0x48;
            rt4832_smbus_write_byte(new_client, 0x03, &reg_data);

            up(&rt4832_lock);
        }

        results = down_interruptible(&rt4832_lock);

        rt4832_smbus_read_byte(new_client, 0x02, &read_data);

        CPD_LOG("OVP[0x%x]\n",read_data);

        msb_data = (level_data >> 8) & 0x03; // 2bit
        lsb_data = (level_data) & 0xFF; // 8bit

        ret |= rt4832_smbus_write_byte(new_client, 0x04, &msb_data);
        ret |= rt4832_smbus_write_byte(new_client, 0x05, &lsb_data);

        CPD_LOG("I2C check = %d\n",ret);
        CPD_LOG("brightness Setting[reg0x04][MSB:0x%x][reg0x05][LSB:0x%x]\n",msb_data,lsb_data);

        up(&rt4832_lock);
    }  else {
        if (is_suspend == DISABLE) {
            results = down_interruptible(&rt4832_lock);

            reg_data = 0x04;
            rt4832_smbus_write_byte(new_client, 0x04, &reg_data);

            reg_data = 0x00;
            rt4832_smbus_write_byte(new_client, 0x05, &reg_data);

            rt4832_smbus_read_byte(new_client, 0x0A, &reg_data);
            reg_data &= 0xE6;
            rt4832_smbus_write_byte(new_client, 0x0A, &reg_data);

            is_suspend = ENABLE;
            up(&rt4832_lock);
        }
    }

    return 0;
}

#ifdef CONFIG_MTK_AAL_SUPPORT
#define REG_BL_ENABLE   0x01
#define REG_BL_DISABLE  0x00

void rt4832_reg_bl_en_ctrl(unsigned int enable)
{
    unsigned char reg_data = 0x00;
    unsigned char write_data = 0x00;

    rt4832_smbus_read_byte(new_client, 0x0A, &reg_data);
    bl_en_data = reg_data;

    if(enable) { // enable
        if((reg_data & REG_BL_ENABLE) == REG_BL_DISABLE)
            bl_en_data |= REG_BL_ENABLE; // push 0x01
    } else { // disable
        if((reg_data & REG_BL_ENABLE) == REG_BL_ENABLE)
            bl_en_data &= (~REG_BL_ENABLE); // pop 0x01
    }
    write_data = bl_en_data;
    rt4832_smbus_write_byte(new_client, 0x0A, &write_data);

    CPD_LOG("%s : bl_en(0x0A) %s(0x%x)\n",__func__,(enable)? "enable":"disable",write_data);
}

void rt4832_check_pwm_enable(void)
{
    unsigned char pwm_enable = 0;

    rt4832_smbus_read_byte(new_client, 0x09, &pwm_enable);
    pwm_enable |= 0x40;
    rt4832_smbus_write_byte(new_client, 0x09, &pwm_enable);

    CPD_LOG("%s for QPPS : 0x%x\n",__func__,pwm_enable);
}
#endif

void rt4832_set_led_mode(int mode)
{
    led_mode = mode;
    CPD_LOG("%s : %s\n",__func__,(mode!=LED_MODE_CUST_LCM)? "LED_MODE_CUST_BLS_PWM(5)":"LED_MODE_CUST_LCM(4)");
}

unsigned char get_rt4832_backlight_level(void)
{
    unsigned char rt4832_msb = 0;
    unsigned char rt4832_lsb = 0;
    unsigned char rt4832_level = 0;

    rt4832_smbus_read_byte(new_client, 0x04, &rt4832_msb);
    rt4832_smbus_read_byte(new_client, 0x05, &rt4832_lsb);

    rt4832_level |= ((rt4832_msb & 0x3) << 6);
    rt4832_level |= ((rt4832_lsb & 0xFC) >> 2);

    return rt4832_level;

}

void set_rt4832_backlight_level(unsigned char level)
{
    unsigned char rt4832_msb = 0;
    unsigned char rt4832_lsb = 0;
    unsigned char data = 0;

    if (level == 0) {
        chargepump_set_backlight_level(level);
    } else {
        if (is_suspend == ENABLE) {
            is_suspend = DISABLE;

            data = 0x70;    /* 0x57; */
            rt4832_smbus_write_byte(new_client, 0x02, &data);
            data = 0x00;    /* 11bit / */
            rt4832_smbus_write_byte(new_client, 0x03, &data);

            rt4832_msb = (level >> 6) | 0x04;
            rt4832_lsb = (level << 2) | 0x03;

            rt4832_smbus_write_byte(new_client, 0x04, &rt4832_msb);
            rt4832_smbus_write_byte(new_client, 0x05, &rt4832_lsb);

            rt4832_smbus_read_byte(new_client, 0x0A, &data);
            data |= 0x19;

            rt4832_smbus_write_byte(new_client, 0x0A, &data);
        } else {
            rt4832_msb = (level >> 6) | 0x04;
            rt4832_lsb = (level << 2) | 0x03;

            rt4832_smbus_write_byte(new_client, 0x04, &rt4832_msb);
            rt4832_smbus_write_byte(new_client, 0x05, &rt4832_lsb);
        }
    }
}

unsigned int get_cur_main_lcd_level(void)
{
    return old_bl_level;
}

EXPORT_SYMBOL(get_cur_main_lcd_level);

static unsigned int get_rt4832_backlight_rawdata(void)
{
    unsigned char rt4832_msb = 0;
    unsigned char rt4832_lsb = 0;
    unsigned int rt4832_level = 0;

    rt4832_smbus_read_byte(new_client, 0x04, &rt4832_msb);
    rt4832_smbus_read_byte(new_client, 0x05, &rt4832_lsb);

    rt4832_level |= ((rt4832_msb & 0x3) << 8);
    rt4832_level |= ((rt4832_lsb & 0xFF));

    return rt4832_level;
}

static void set_rt4832_backlight_rawdata(unsigned int level)
{
    unsigned char rt4832_msb = 0;
    unsigned char rt4832_lsb = 0;

    rt4832_msb = (level >> 8) & 0x03;
    rt4832_lsb = (level) & 0xFF;

    rt4832_smbus_write_byte(new_client, 0x04, &rt4832_msb);
    rt4832_smbus_write_byte(new_client, 0x05, &rt4832_lsb);
}

static ssize_t lcd_backlight_show_blmap(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    int i, j;

    buf[0] = '{';

    for (i = 0, j = 2; i < 256 && j < PAGE_SIZE; ++i) {
            if (!(i % 15)) {
                    buf[j] = '\n';
                    ++j;
            }

            sprintf(&buf[j], "%d, ", bright_arr[i]);
            if (bright_arr[i] < 10)
                    j += 3;
            else if (bright_arr[i] < 100)
                    j += 4;
            else
                    j += 5;
    }

    buf[j] = '\n';
    ++j;
    buf[j] = '}';
    ++j;

    return j;
}

static ssize_t lcd_backlight_store_blmap(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    int i;
    int j;
    int value, ret;

    if (count < 1)
            return count;

    if (buf[0] != '{')
            return -EINVAL;


    for (i = 1, j = 0; i < count && j < 256; ++i) {
        if (!isdigit(buf[i]))
                continue;

        ret = sscanf(&buf[i], "%d", &value);
        if (ret < 1)
                pr_err("read error\n");
        bright_arr[j] = (unsigned int)value;

        while (isdigit(buf[i]))
                ++i;
        ++j;
    }

    return count;
}

static ssize_t show_rt4832_rawdata(struct device *dev,struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "RT4832 brightness code : %d\n",get_rt4832_backlight_rawdata());
}

static ssize_t store_rt4832_rawdata(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    char *pvalue = NULL;
    unsigned int level = 0;
    size_t size = 0;

    level = simple_strtoul(buf,&pvalue,10);
    size = pvalue - buf;

    if (*pvalue && isspace(*pvalue))
            size++;

    printk("[RT4832] store_rt4832_rawdata : [%d] \n",level);
    set_rt4832_backlight_rawdata(level);

    return count;
}

DEVICE_ATTR(brightness_code, 0644, show_rt4832_rawdata, store_rt4832_rawdata);
DEVICE_ATTR(bl_blmap, 0644, lcd_backlight_show_blmap, lcd_backlight_store_blmap);

static int rt4832_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;

    CPD_FUN();

    new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);

    err = device_create_file(&client->dev, &dev_attr_bl_blmap);
    err = device_create_file(&client->dev, &dev_attr_brightness_code);

    memset(new_client, 0, sizeof(struct i2c_client));

    new_client = client;

    return 0;
}

static int rt4832_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    new_client = client;

    CPD_FUN();

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        CPD_LOG("i2c_check_functionality error\n");
        return -1;
    }

    if (client == NULL)
        CPD_ERR("%s client is NULL\n", __func__);
    else
        CPD_LOG("%s %p %x %x\n", __func__, client->adapter, client->addr, client->flags);
    return 0;
}


static int rt4832_i2c_remove(struct i2c_client *client)
{
    CPD_FUN();

    new_client = NULL;

    return 0;
}


static int
__attribute__ ((unused)) rt4832_detect(struct i2c_client *client, int kind,
                       struct i2c_board_info *info)
{
    CPD_FUN();

    return 0;
}

static struct i2c_driver rt4832_i2c_driver = {
    .driver.name = RT4832_DEV_NAME,
    .probe = rt4832_i2c_probe,
    .remove = rt4832_i2c_remove,
    .id_table = rt4832_i2c_id,
};

static int rt4832_pd_probe(struct platform_device *pdev)
{
    CPD_FUN();

    i2c_register_board_info(2, &i2c_rt4832, 1);

    mt_set_gpio_mode(GPIO_LCD_BL_EN, GPIO_LCD_BL_EN_MODE);
    mt_set_gpio_pull_enable(GPIO_LCD_BL_EN, GPIO_PULL_ENABLE);
    mt_set_gpio_dir(GPIO_LCD_BL_EN, GPIO_DIR_OUT);

    if (i2c_add_driver(&rt4832_driver) != 0)
        CPD_ERR("Failed to register rt4832 driver");

    return 0;
}

static int __attribute__ ((unused)) rt4832_pd_remove(struct platform_device *pdev)
{
    CPD_FUN();
    i2c_del_driver(&rt4832_i2c_driver);
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void rt4832_early_suspend(struct early_suspend *h)
{
    int err = 0;
    unsigned char data;

    CPD_FUN();

    down_interruptible(&rt4832_lock);
    data = 0x00;
    err = rt4832_smbus_write_byte(new_client, 0x05, &data);

    err = rt4832_smbus_read_byte(new_client, 0x0A, &data);
    data &= 0xE6;

    err = rt4832_smbus_write_byte(new_client, 0x0A, &data);
    up(&rt4832_lock);
    CPD_LOG("%s : [%d]\n",__func__,data);
#ifdef USING_LCM_BL_EN
    mt_set_gpio_out(GPIO_LCM_BL_EN, GPIO_OUT_ZERO);
#else
    mt_set_gpio_out(GPIO_LCD_BL_EN, GPIO_OUT_ZERO);
#endif
}

static void rt4832_late_resume(struct early_suspend *h)
{
    int err = 0;
    unsigned char data1;

    CPD_FUN();

#ifdef USING_LCM_BL_EN
    mt_set_gpio_out(GPIO_LCM_BL_EN, GPIO_OUT_ONE);
#else
    mt_set_gpio_out(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
#endif
    mdelay(50);
    down_interruptible(&rt4832_lock);
    err = rt4832_smbus_write_byte(new_client, 0x05, &current_brightness);

    err = rt4832_smbus_read_byte(new_client, 0x0A, &data1);
    data1 |= 0x19;      /* backlight enable */

    err = rt4832_smbus_write_byte(new_client, 0x0A, &data1);
    up(&rt4832_lock);
    CPD_LOG("%d : [%d]\n",__func__,data1);
}

static struct early_suspend __attribute__ ((unused)) rt4832_early_suspend_desc = {
.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN, .suspend = rt4832_early_suspend, .resume =
        rt4832_late_resume,};
#endif

static struct platform_driver rt4832_backlight_driver = {
    .remove = rt4832_pd_remove,
    .probe = rt4832_pd_probe,
    .driver = {
           .name = BACKLIHGT_NAME,
           .owner = THIS_MODULE,
           },
};

#if 0
#ifdef CONFIG_OF
static struct platform_device mtk_backlight_dev = {
    .name = BACKLIHGT_NAME,
    .id = -1,
};
#endif
#endif

/* strobe enable */
void rt4832_flash_strobe_en(void)
{
    int err = 0;
    unsigned char flash_OnOff=0;
    unsigned int results = 0;
    CPD_FUN();
    results = down_interruptible(&rt4832_lock);
    err = rt4832_smbus_read_byte(new_client, 0x0A, &flash_OnOff);
    if(flash_ctrl == 1){
        flash_OnOff &= 0xBF;
        flash_OnOff |= 0x06;//0x66;
    }

    else if(flash_ctrl == 2){
        flash_OnOff &= 0xBF;
        flash_OnOff |= 0x02;//0x62;
    }

    else{
        flash_OnOff &= 0xB9;//0x99;
    }
    err = rt4832_smbus_write_byte(new_client, 0x0A, &flash_OnOff);
    up(&rt4832_lock);
}
void rt4832_flash_strobe_prepare(char OnOff, char ActiveHigh)
{
    unsigned int results = 0;
    int err = 0;
    unsigned char flash_ovp=0; // LGE_UPDATE [yonghwan.lym@lge.com] 2016/02/03, Flash OVP Check

    CPD_FUN();
    results = down_interruptible(&rt4832_lock);
// LGE_UPDATE_S [yonghwan.lym@lge.com] 2016/02/03, Flash OVP Check
    err = rt4832_smbus_read_byte(new_client, 0x0B, &flash_ovp);
    if(flash_ovp != 0)
        printk("Flash OVP REG[0x0B]=0x%x\n",flash_ovp);
// LGE_UPDATE_E [yonghwan.lym@lge.com] 2016/02/03, Flash OVP Check
    err = rt4832_smbus_read_byte(new_client, 0x09, &strobe_ctrl);


    strobe_ctrl &= 0xF3;
    flash_ctrl = OnOff;

    if (OnOff == 1) {
        CPD_LOG("Strobe mode On\n");
        strobe_ctrl |= 0x10;
    }
    else if(OnOff == 2)
    {
        CPD_LOG("Torch mode On\n");
        strobe_ctrl |= 0x10;
    }
    else
    {
        CPD_LOG("Flash Off\n");
        strobe_ctrl &= 0xEF;
    }
    err = rt4832_smbus_write_byte(new_client, 0x09, &strobe_ctrl);
    up(&rt4832_lock);
}

/* strobe level */
void rt4832_flash_strobe_level(char level)
{
    int err = 0;
    unsigned char data1 = 0;
    unsigned char data2 = 0;
    unsigned char torch_level;
    unsigned char strobe_timeout = 0x1F;
    unsigned int results = 0;

    CPD_FUN();
    results = down_interruptible(&rt4832_lock);

    torch_level = 0x50;  //150mA

    err = rt4832_smbus_read_byte(new_client, 0x06, &data1);

    strobe_timeout = 0x18;

#if defined(CONFIG_MACH_MT6750_SF3)
    if (level < 0)
        data1 = torch_level;
    else if (level == 1)
        data1= torch_level | 0x02;  //300mA
    else if(level == 2)
        data1= torch_level | 0x04;  //500mA
    else if(level == 3)
        data1= torch_level | 0x07;  //800mA
    else if(level == 4)
        data1= torch_level | 0x09;  //1000mA
    else
        data1= torch_level | 0x09;  //1000mA
#elif defined(CONFIG_MACH_MT6750_LV5)
    if (level < 0)
        data1 = torch_level;
    else if (level == 1)
        data1= torch_level | 0x01;  //200mA
    else if(level == 2)
        data1= torch_level | 0x03;  //400mA
    else if(level == 3)
        data1= torch_level | 0x05;  //600mA
    else if(level == 4)
        data1= torch_level | 0x07;  //800mA
    else
        data1= torch_level | 0x07;  //800mA
#else
    if (level < 0)
        data1 = torch_level;
    else if (level == 1)
        data1= torch_level | 0x01;  //200mA
    else if(level == 2)
        data1= torch_level | 0x03;  //400mA
    else if(level == 3)
        data1= torch_level | 0x05;  //600mA
    else if(level == 4)
        data1= torch_level | 0x07;  //800mA
    else
        data1= torch_level | 0x07;  //800mA
#endif

    CPD_LOG("Flash Level =0x%x\n", data1);
    err = rt4832_smbus_write_byte(new_client, 0x06, &data1);

    data2 = strobe_timeout;
    CPD_LOG("Storbe Timeout =0x%x\n", data2);
    err |= rt4832_smbus_write_byte(new_client, 0x07, &data2);
    up(&rt4832_lock);
}
EXPORT_SYMBOL(rt4832_flash_strobe_en);
EXPORT_SYMBOL(rt4832_flash_strobe_prepare);
EXPORT_SYMBOL(rt4832_flash_strobe_level);

static int __init rt4832_init(void)
{
    CPD_FUN();
    sema_init(&rt4832_lock, 1);

    // I2C3
    i2c_register_board_info(3, &i2c_rt4832, 1);

    mt_set_gpio_mode(GPIO_LCD_BL_EN, GPIO_LCD_BL_EN_MODE);
    mt_set_gpio_pull_enable(GPIO_LCD_BL_EN, GPIO_PULL_ENABLE);
    mt_set_gpio_dir(GPIO_LCD_BL_EN, GPIO_DIR_OUT);

    if (i2c_add_driver(&rt4832_driver) != 0)
        CPD_ERR("Failed to register rt4832 driver");

    CPD_LOG("%s\n",__func__);
    return 0;
}

static void __exit rt4832_exit(void)
{
    platform_driver_unregister(&rt4832_backlight_driver);
}

MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("rt4832 driver");
MODULE_LICENSE("GPL");

late_initcall(rt4832_init);
module_exit(rt4832_exit);