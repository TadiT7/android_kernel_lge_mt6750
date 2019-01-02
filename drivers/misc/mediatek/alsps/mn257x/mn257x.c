/* drivers/hwmon/mt6516/amit/mn257x.c - MN257x ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 /******************************************************************************
 * driver release info
 1. 2017/08/04 release version V1.0
 2. 2017/08/17 Pocket detection
    define POCKET_DETECTION
 3. 2017/08/23 Add ps only mode wait time 100ms (psonly_wait_time)

*******************************************************************************/

#include <linux/version.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/kobject.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <cust_alsps.h>
#include <linux/input/mt.h>
#include "mn257x.h"
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include <alsps.h>
#include "upmu_common.h"
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/sched.h>

/******************************************************************************
 * driver info
*******************************************************************************/
#define MN_DEV_NAME   		    "MN257x"
#define DRIVER_VERSION          "1.0"
/******************************************************************************
 * ALS / PS sensor structure
*******************************************************************************/
#define COMPATIABLE_NAME "mediatek,mn257x"
static struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
static long long int_top_time;

/******************************************************************************
 *  ALS / PS define
 ******************************************************************************/
#define ALS_DYN_INTT    1   // ALS Auto INTT
#define ALS_LSRC        0
#define MN_SELF_TEST    1       //self test
#if defined(MN_SELF_TEST)
#define CALB_TIMES      3
#define CALIBRATION_TO_FILE 1
#include "../../sensor_cal/sensor_cal_file_io.h"
#endif

#define DTS_PARSE       1
#define PS_SUN_LIGHT    1
#define VLED_POWER_OFF       1  //for suspend VLED power off work around
#define POCKET_DETECTION  1
#define PS_11_BIT       1
#define ALSPS_DBG       1

#define LUX_PER_COUNT	400  //ALS lux per count
#define L_SENSOR_HTHD 60000   // ALS interrupt threshold high
#define L_SENSOR_LTHD 1000      // ALS interrupt threshold low

static int als_rs_value[] = {1, 2, 4, 8, 16, 32, 64, 128};
int rs_num = sizeof(als_rs_value)/sizeof(int);

int als_frame_time = 0;
int ps_frame_time = 0;

/*ALS interrupt offset*/
u16 als_intr_thd_offset = 500;

int ps_enh_gain = 2;
/******************************************************************************
 *I2C function define
*******************************************************************************/
#define TXBYTES 				2
#define PACKAGE_SIZE 			48
#define I2C_RETRY_COUNT 		2
int i2c_max_count=8;
static const struct i2c_device_id mn257x_i2c_id[] = {{MN_DEV_NAME,0},{}};

/******************************************************************************
 * extern functions
*******************************************************************************/
#define POWER_NONE_MACRO MT65XX_POWER_NONE

struct hwmsen_object *ps_hw, * als_hw;
static struct mn_sensor_priv *mn_sensor_obj = NULL;
//static struct wake_lock ps_lock;
static struct mutex sensor_mutex;
static mn_optical_sensor mn_sensor;
static struct i2c_client *mn257x_i2c_client = NULL;

typedef struct _mn_raw_data
{
    u8 raw_bytes[PACKAGE_SIZE];
} mn_raw_data;
static mn_raw_data	gRawData;

#if ALSPS_DBG
bool debug_flag = true;
#endif

#define APS_TAG                 	  	"[mn257x] "

#if ALSPS_DBG
#define APS_FUN(f)               	 if(debug_flag)\
                                        printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_LOG(fmt, args...)    	 if(debug_flag)\
                                        printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_ERR(fmt, args...)   	 printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#else
#define APS_FUN(f)              	  	printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    	    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    	    printk(KERN_INFO APS_TAG fmt, ##args)
#endif
#define APS_DBG(fmt, args...)    	    printk(KERN_INFO fmt, ##args)

static int mn257x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int mn257x_i2c_remove(struct i2c_client *client);
static int mn257x_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int mn257x_i2c_resume(struct i2c_client *client);
static irqreturn_t mn257x_eint_func(int irq, void *desc);
static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);
static int set_lsensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);
void mn257x_update_mode(struct i2c_client *client);
void mn257x_fast_update(struct i2c_client *client);
int mn257x_read_ps(struct i2c_client *client);
static int ps_sensing_time(int intt, int adc, int cycle);
static int als_sensing_time(int intt, int adc, int cycle);
#if defined(CONFIG_FB)
static int light_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif
int factory_ps_data(void);
int factory_als_data(void);
void mn257x_enable_ps(int enable);
#if MN_SELF_TEST
typedef enum
{
	CALI_SUCCESS=0,
	CALI_FAIL=1,

}cali_rst_type;
#endif

typedef enum
{
    CMC_BIT_ALS   	= 1,
    CMC_BIT_PS     	= 2,
} CMC_BIT;

typedef enum
{
    CMC_BIT_RAW   			= 0x0,
    CMC_BIT_PRE_COUNT     	= 0x1,
    CMC_BIT_DYN_INT			= 0x2,
    CMC_BIT_TABLE			= 0x3,
    CMC_BIT_INTR_LEVEL		= 0x4,
} CMC_ALS_REPORT_TYPE;

struct mn257x_i2c_addr      /*define a series of i2c slave address*/
{
    u8  write_addr;
};

struct mn_sensor_priv
{
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct delayed_work  eint_work;
    struct input_dev *gs_input_dev;
    /*i2c address group*/
    struct mn257x_i2c_addr  addr;
    /*misc*/
    atomic_t   	als_suspend;
    atomic_t    ps_suspend;
#if MN_SELF_TEST
    cali_rst_type			cali_status;
    u32  ps_crosstalk;
#endif
    /*data*/
    u16		    lux_per_count;
    ulong       enable;         	/*record HAL enalbe status*/
    /*data*/
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
    /*als interrupt*/
    int als_intr_level;
    int als_intr_lux;

#if defined(CONFIG_FB)
    struct notifier_block    fb_notif;
#endif
    struct device_node *irq_node;
    int		irq;
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,mn257x"},
	{},
};
#endif

static struct i2c_driver mn257x_i2c_driver =
{
    .probe     	= mn257x_i2c_probe,
    .remove     = mn257x_i2c_remove,
    .suspend    = mn257x_i2c_suspend,
    .resume     = mn257x_i2c_resume,
    .id_table   = mn257x_i2c_id,
    .driver = {
        .name   = MN_DEV_NAME,
#ifdef CONFIG_OF
	    .of_match_table = alsps_of_match,
#endif
    },
};

static int alsps_init_flag =-1; // 0<==>OK -1 <==> fail
static int alsps_local_init(void);
static int alsps_remove(void);
static struct alsps_init_info mn257x_init_info = {
		.name = MN_DEV_NAME,
		.init = alsps_local_init,
		.uninit = alsps_remove,
};

extern struct alsps_context *alsps_context_obj;

/******************************************************************************
 *  ALS_DYN_INTT
 ******************************************************************************/
#if ALS_DYN_INTT
//Dynamic INTT
int dynamic_intt_idx;
int dynamic_intt_init_idx = 0;	//initial dynamic_intt_idx
int c_gain;
int dynamic_intt_lux = 0;
uint16_t dynamic_intt_high_thr;
uint16_t dynamic_intt_low_thr;
uint32_t dynamic_intt_max_lux = 65535;
uint32_t dynamic_intt_min_lux = 0;
uint32_t dynamic_intt_min_unit = 1000;
static int als_dynamic_intt_intt[] = {MN_ALS_INTT_512, MN_ALS_INTT_256};
static int als_dynamic_intt_value[] = {512, 256}; //match als_dynamic_intt_intt table
static int als_dynamic_intt_gain[] = {MN_GAIN_MID, MN_GAIN_LOW};
static int als_dynamic_intt_high_thr[] = {60000, 60000};
static int als_dynamic_intt_low_thr[] = {200, 200};
static int als_dynamic_intt_intt_num =  sizeof(als_dynamic_intt_value)/sizeof(int);
#endif /*ALS_DYN_INTT*/

#if ALS_LSRC
int offset_gain = 143; // 1/7
int scale_gain = 333; // 1/3
u32 lsrc_als_offset = 0;
u16 lsrc_raw = 0;
u16 lsrc_lux = 0;
u16 als_lux = 0;
u32 lsrc_ratio = 0;
#if ALS_DYN_INTT
u16 als_dyn_intt_ch0;
#endif
#endif

#if MN_SELF_TEST
static unsigned int ps_factory_ct =0;
u16 ps_factory_offset;
u16 ps_factory_hystersis;
u16 ps_self_test_max_ct;
u32 ps_self_ps_avg;
#endif

#if DTS_PARSE
u16 ps_default_l = 500;
u16 ps_default_h = 750;
#endif

#if PS_SUN_LIGHT
#define PS_MAX_IR   50000
#endif

static u16 als_lux;

#define LOW_LUX_RESO 33

#if POCKET_DETECTION
#define POCKET_THRESHOLD 4
#if defined(CONFIG_FB)
static atomic_t driver_suspend_flag = ATOMIC_INIT(0);
#endif
u16 pkd_lux = 0;
u16 low_lux_offset = 0;
#endif


static int mn257x_I2C_Write_Block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int err, idx, num;
    char buf[C_I2C_FIFO_SIZE];
    err =0;

    if (!client)
    {
        return -EINVAL;
    }
    else if (len >= C_I2C_FIFO_SIZE)
    {
        APS_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        return -EINVAL;
    }
    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++)
    {
        buf[num++] = data[idx];
    }
    err = i2c_master_send(client, buf, num);
    if (err < 0)
    {
        APS_ERR("send command error!!\n");

        return -EFAULT;
    }

    return err;
}

static int mn257x_I2C_Write_Cmd(struct i2c_client *client, uint8_t regaddr, uint8_t data, uint8_t txbyte)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry;

    buffer[0] = regaddr;
    buffer[1] = data;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);

        if (ret == txbyte)
        {
            break;
        }

        APS_ERR("i2c write error,TXBYTES %d\n",ret);
    }
    if(retry>=I2C_RETRY_COUNT)
    {
        APS_ERR("i2c write retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    return ret;
}

static int mn257x_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t data)
{
    int ret = 0;
    ret = mn257x_I2C_Write_Cmd(client, regaddr, data, 0x02);
    return ret;
}

static int mn257x_I2C_Read(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount)
{
    int ret = 0;
    int retry;
    int read_count=0, rx_count=0;
    while(bytecount>0)
    {
        mn257x_I2C_Write_Cmd(client, regaddr+read_count, 0x00, 0x01);
        for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
        {
            rx_count = bytecount>i2c_max_count?i2c_max_count:bytecount;
            ret = i2c_master_recv(client, &gRawData.raw_bytes[read_count], rx_count);

            if (ret == rx_count)
                break;

            APS_ERR("i2c read error %d\r\n",ret);
        }
        if(retry>=I2C_RETRY_COUNT)
        {
            APS_ERR("i2c read retry over %d\n", I2C_RETRY_COUNT);
            return -EINVAL;
        }
        bytecount-=rx_count;
        read_count+=rx_count;
    }

    return ret;
}


static void set_als_ps_intr_type(struct i2c_client *client, bool ps_polling, bool als_polling)
{
    //set als / ps interrupt control mode and interrupt type
	switch((ps_polling << 1) | als_polling)
	{
		case 0: // ps and als interrupt
			mn_sensor.interrupt_control = 	MN_INT_CTRL_ALS_OR_PS;
			mn_sensor.als.interrupt_type = MN_INTTY_ACTIVE;
			mn_sensor.ps.interrupt_type = MN_INTTY_ACTIVE;
		break;
		case 1: //ps interrupt and als polling
			mn_sensor.interrupt_control = 	MN_INT_CTRL_PS;
			mn_sensor.als.interrupt_type = MN_INTTY_DISABLE;
			mn_sensor.ps.interrupt_type = MN_INTTY_ACTIVE;
		break;
		case 2: // ps polling and als interrupt
			mn_sensor.interrupt_control = 	MN_INT_CTRL_ALS;
			mn_sensor.als.interrupt_type = MN_INTTY_ACTIVE;
			mn_sensor.ps.interrupt_type = MN_INTTY_DISABLE;
		break;
		case 3: //ps and als polling
			mn_sensor.interrupt_control = 	MN_INT_CTRL_ALS_OR_PS;
			mn_sensor.als.interrupt_type = MN_INTTY_DISABLE;
			mn_sensor.ps.interrupt_type = MN_INTTY_DISABLE;
		break;
	}
}

static void write_global_variable(struct i2c_client *client)
{
    u8 buf_block[7];
    mn257x_I2C_Write(client, DEVREG_RESET, MN_POWER_ON | MN_RESETN_RUN);
    mn257x_I2C_Write(client, DEVREG_ALS_STATUS, MN_CMP_RUN | MN_UN_LOCK);
    mn257x_I2C_Write(client, DEVREG_PS_STATUS, MN_CMP_RUN | MN_UN_LOCK);

    mn257x_I2C_Write(client, DEVREG_RESET, MN_POWER_OFF | MN_RESETN_RESET);


    /*chip refrash*/
    if(mn_sensor.revno != 0x0288)
    {
        mn257x_I2C_Write(client, 0xfd, 0x8e);
        if(mn_sensor.revno == 0xa188)
        {
            mn257x_I2C_Write(client, 0xfe, 0xa2);
            mn257x_I2C_Write(client, 0xfe, 0x82);
        }
        else
        {
            mn257x_I2C_Write(client, 0xfe, 0x22);
            mn257x_I2C_Write(client, 0xfe, ps_enh_gain);
        }
        mn257x_I2C_Write(client, 0xfd, 0x00);
        mn257x_I2C_Write(client, 0xfc, MN_A_D | MN_NORMAL| MN_GFIN_ENABLE | MN_VOS_ENABLE | MN_DOC_ON);
    }

    {
        set_psensor_intr_threshold(mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);
        set_lsensor_intr_threshold(mn_sensor.als.low_threshold, mn_sensor.als.high_threshold);

        mn257x_I2C_Write(client, DEVREG_PS_OFSL, (u8)(mn_sensor.ps.cancelation& 0xff));
        mn257x_I2C_Write(client, DEVREG_PS_OFSH, (u8)((mn_sensor.ps.cancelation & 0xff00) >> 8));

        //set als / ps interrupt control mode and trigger type
        set_als_ps_intr_type(client, mn_sensor.ps.polling_mode, mn_sensor.als.polling_mode);

        if(mn_sensor.revno != 0x8188)
        {
            if(mn_sensor.revno == 0x0288)
            {

                buf_block[0] = mn_sensor.als.als_std | mn_sensor.als.integration_time | mn_sensor.als.gain;//REG0x01
                buf_block[1] = mn_sensor.als.als_rs | mn_sensor.als.adc | mn_sensor.als.cycle;
                buf_block[2] = mn_sensor.ps.ps_std | mn_sensor.ps.integration_time | mn_sensor.ps.gain;
                buf_block[3] = mn_sensor.ps.ps_rs | mn_sensor.ps.adc | mn_sensor.ps.cycle;
                buf_block[4] = mn_sensor.ps.ir_on_control | mn_sensor.ps.ir_mode | mn_sensor.ps.ir_drive;
                buf_block[5] = mn_sensor.interrupt_control | mn_sensor.ps.persist |mn_sensor.ps.interrupt_type;
                buf_block[6] = mn_sensor.als.interrupt_channel_select | mn_sensor.als.persist | mn_sensor.als.interrupt_type;
            }
            else
            {
                buf_block[0] = mn_sensor.als.als_intb_nonlos | mn_sensor.als.integration_time | mn_sensor.als.gain; //REG0x01
                buf_block[1] = mn_sensor.als.als_rs | mn_sensor.als.adc | mn_sensor.als.cycle;
                buf_block[2] = mn_sensor.ps.ps_intb_nonlos | mn_sensor.ps.integration_time | mn_sensor.ps.gain;
                buf_block[3] = mn_sensor.ps.ps_rs | mn_sensor.ps.adc | mn_sensor.ps.cycle;
                buf_block[4] = mn_sensor.ps.ps_std | mn_sensor.ps.ir_on_control | mn_sensor.ps.ir_mode | mn_sensor.ps.ir_drive;
                buf_block[5] = mn_sensor.interrupt_control | mn_sensor.ps.persist |mn_sensor.ps.interrupt_type;
                buf_block[6] = mn_sensor.als.als_std | mn_sensor.als.interrupt_channel_select | mn_sensor.als.persist | mn_sensor.als.interrupt_type;
            }
        }
        else
        {
            buf_block[0] = mn_sensor.als.integration_time | mn_sensor.als.gain; //REG0x01
            buf_block[1] = mn_sensor.als.adc | mn_sensor.als.cycle;
            buf_block[2] = mn_sensor.ps.integration_time | mn_sensor.ps.gain;
            buf_block[3] = mn_sensor.ps.adc | mn_sensor.ps.cycle;
            buf_block[4] = mn_sensor.ps.ir_on_control | mn_sensor.ps.ir_mode | mn_sensor.ps.ir_drive;
            buf_block[5] = mn_sensor.interrupt_control | mn_sensor.ps.persist |mn_sensor.ps.interrupt_type;
            buf_block[6] = mn_sensor.als.interrupt_channel_select | mn_sensor.als.persist | mn_sensor.als.interrupt_type;

        }
        mn257x_I2C_Write_Block(client, DEVREG_ALS_CONFIG, buf_block, 7);
#if !ALS_DYN_INTT
        if(mn_sensor.revno == 0x0288 && mn_sensor.als.gain == MN_AG_EN)
        {
            buf_block[0] = mn_sensor.als.als_ag_l | mn_sensor.als.als_aintt_l;//REG0x24
            buf_block[1] = mn_sensor.als.als_ag_h | mn_sensor.als.als_aintt_h;
            buf_block[2] = (u8)(mn_sensor.als.als_ag_thd & 0xff);
            buf_block[3] = (u8)((mn_sensor.als.als_ag_thd & 0xff00) >> 8);
            mn257x_I2C_Write_Block(client, 0x24, buf_block, 4);
        }
#endif
	}

    //set mode and wait
    if(mn_sensor.revno == 0x0288)
        mn257x_I2C_Write(client, DEVREG_ENABLE, (mn_sensor.wait | mn_sensor.als_single_pixel | mn_sensor.mode));
    else
        mn257x_I2C_Write(client, DEVREG_ENABLE, (mn_sensor.wait | mn_sensor.mode));

}


#if MN_SELF_TEST
static int mn257x_cal_ps_calibration( struct mn_sensor_priv* obj, int calib_times, bool write)
{
    int i, ps_time, als_time, wait_time, pflag=1;
    u16 ps_ct[10];
    u32 ps_raw_total=0, ps_raw_min=0xffff, ps_raw_max=0;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable);

    ps_time = ps_sensing_time(mn_sensor.ps.integration_time, mn_sensor.ps.adc, mn_sensor.ps.cycle);
    als_time = als_sensing_time(mn_sensor.als.integration_time, mn_sensor.als.adc, mn_sensor.als.cycle);
    wait_time = wait_value[mn_sensor.ps_wait >> 4];
    APS_LOG("[%s]: ps_time=%d, als_time=%d ", __func__, ps_time, als_time);

    APS_LOG("sensor_calibration IN \n");

    if(!obj){
        APS_ERR(" Parameter error \n");
        return -1;
    }

    /* disable ps interrupt begin */
    mn257x_I2C_Write(obj->client, DEVREG_RESET, (MN_POWER_OFF | MN_RESETN_RESET));
    mn257x_I2C_Write(obj->client, DEVREG_PS_INT, (mn_sensor.interrupt_control | mn_sensor.ps.persist | MN_INTTY_DISABLE));
    mn257x_I2C_Write(obj->client, DEVREG_RESET, (MN_POWER_ON | MN_RESETN_RUN));
    /* disable ps interrupt end */

    if(enable_ps == false)
    {
        enable_ps = true;
        pflag =0;
        mn257x_enable_ps(true);
     }

    for(i=0; i<calib_times; i++)
    {
        int sleep_time = 0;

        if( (enable_ps == true) && (enable_als == false) )
        {
            sleep_time = ps_time + wait_time;
        }
        else if((enable_ps == true) && (enable_als == true))
        {
            sleep_time = ps_time + als_time + wait_time;
        }

        msleep( sleep_time );
        mn257x_read_ps(obj->client);

        ps_ct[i] = mn_sensor.ps.data.data;
        APS_LOG("[mn_sensor_do_calibration] sleep %dms, pdata[%d] = %d\n", sleep_time, i, ps_ct[i]);

        if(ps_ct[i] <= ps_raw_min) /*min ps raw*/
            ps_raw_min = ps_ct[i];

        if(ps_ct[i] >= ps_raw_max)/*max ps raw*/
            ps_raw_max = ps_ct[i];

        ps_raw_total += ps_ct[i];
    }
    ps_raw_total = (ps_raw_total - ps_raw_min - ps_raw_max) / (calib_times-2);
    ps_self_ps_avg = ps_raw_total;


    /* disable ps interrupt begin */
    mn257x_I2C_Write(obj->client, DEVREG_RESET, (MN_POWER_OFF | MN_RESETN_RESET));
    mn257x_I2C_Write(obj->client, DEVREG_PS_INT, (mn_sensor.interrupt_control | mn_sensor.ps.persist | mn_sensor.ps.interrupt_type));
    mn257x_I2C_Write(obj->client, DEVREG_RESET, (MN_POWER_ON | MN_RESETN_RUN));
    /* disable ps interrupt end */


    APS_LOG("[mn_sensor_do_calibration]  ps_raw_min=%u, ps_raw_max=%u\r\n", ps_raw_min, ps_raw_max);
    APS_LOG("[mn_sensor_do_calibration]  ps_self_ps_avg=%u, ps_self_test_max_ct=%u\r\n", (u16)ps_self_ps_avg, (u16)ps_self_test_max_ct);

    obj->ps_crosstalk = ps_self_ps_avg;

    if(ps_raw_total > ps_self_test_max_ct)
    {
        obj->cali_status = CALI_FAIL;
        APS_LOG("[mn_sensor_do_calibration] Failed \r\n");
        if(pflag == 0)
        {
            pflag =1;
            enable_ps= false;
            mn257x_enable_ps(false);
        }
            return CALI_FAIL;
    }

    APS_LOG("[mn_sensor_do_calibration] success \r\n");
    if(write == true)
    {
        ps_factory_ct = ps_self_ps_avg;
        mn_sensor.ps.high_threshold = ps_factory_ct + ps_factory_offset;
        mn_sensor.ps.low_threshold = mn_sensor.ps.high_threshold - ps_factory_hystersis;
        set_psensor_intr_threshold(mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);
    }

    obj->cali_status = CALI_SUCCESS;
    if(pflag == 0)
    {
        pflag =1;
        enable_ps= false;
        mn257x_enable_ps(false);
    }
    return obj->cali_status;
}
static int mn257x_do_ps_calibration( struct mn_sensor_priv* obj, int calib_times, bool write)
{

	int i = 0;
	int ret;
	for( i = 0; i < CALB_TIMES; i++){
		ret = mn257x_cal_ps_calibration(obj, calib_times, write);
		if(ret == CALI_SUCCESS)
			break;
	}
	if(i >= CALB_TIMES){
		APS_ERR("failed to calibrate cross talk\n");
		return -1;
	}
#if defined(CALIBRATION_TO_FILE)
	sensor_calibration_save(ID_PROXIMITY, &obj->ps_crosstalk);
#endif
	return 0;
}
#endif

#if DTS_PARSE
static int mn257x_parse_dt(const char* name)
{
    struct device_node *node = NULL;
    int ret;
    u32 nv_ps_enh_gain[] = {0};
	u32 nv_ps_default_thd_h[] = {0};
	u32 nv_ps_default_thd_l[] = {0};
	u32 nv_ps_ct[] = {0};
	u32 nv_ps_offset[] = {0};
	u32 nv_ps_hystersis[] = {0};
	u32 nv_lux_per_count[] = {0};
	u32 nv_wait_time[] = {0};
	u32 nv_ps_wait_time[] = {0};
	u32 nv_als_intt[] = {0};
	u32 nv_als_gain[] = {0};
	u32 nv_als_filter[] = {0};
	u32 nv_als_persist[] = {0};
	u32 nv_als_intr_thd_offset[] = {0};
	u32 nv_als_report_type[] = {0};
	u32 nv_als_dyn_intt_0[] = {0};
	u32 nv_als_dyn_intt_gain_0[] = {0};
	u32 nv_als_dyn_intt_1[] = {0};
	u32 nv_als_dyn_intt_gain_1[] = {0};
#if ALS_LSRC
	u32 nv_als_scale_gain[] = {0};
	u32 nv_als_offset_gain[] = {0};
#endif
	u32 nv_ps_intt[] = {0};
	u32 nv_ps_gain[] = {0};
	u32 nv_ps_filter[] = {0};
	u32 nv_ps_ir_drive[] = {0};
	u32 nv_ps_persist[] = {0};
	u32 nv_ps_self_test_max_ct[] = {0};
#if POCKET_DETECTION
    u32 nv_als_low_lux_offset[]={0};
#endif

    node = of_find_compatible_node(NULL, NULL, name);

    if(node)
    {

        ret = of_property_read_u32_array(node, "ps_enh_gain", nv_ps_enh_gain, ARRAY_SIZE(nv_ps_enh_gain));
		if (ret == 0)
			ps_enh_gain = nv_ps_enh_gain[0];

		ret = of_property_read_u32_array(node, "ps_default_thd_h", nv_ps_default_thd_h, ARRAY_SIZE(nv_ps_default_thd_h));
		if (ret == 0)
			ps_default_h = nv_ps_default_thd_h[0];

		ret = of_property_read_u32_array(node, "ps_default_thd_l", nv_ps_default_thd_l, ARRAY_SIZE(nv_ps_default_thd_l));
		if (ret == 0)
			ps_default_l = nv_ps_default_thd_l[0];

		ret = of_property_read_u32_array(node, "ps_ct", nv_ps_ct, ARRAY_SIZE(nv_ps_ct));
		if (ret == 0)
			ps_factory_ct = nv_ps_ct[0];

		ret = of_property_read_u32_array(node, "ps_offset", nv_ps_offset, ARRAY_SIZE(nv_ps_offset));
		if (ret == 0)
			ps_factory_offset = nv_ps_offset[0];

		ret = of_property_read_u32_array(node, "ps_hystersis", nv_ps_hystersis, ARRAY_SIZE(nv_ps_hystersis));
		if (ret == 0)
			ps_factory_hystersis = nv_ps_hystersis[0];

		ret = of_property_read_u32_array(node, "lux_per_count", nv_lux_per_count, ARRAY_SIZE(nv_lux_per_count));
		if (ret == 0)
#if ALS_DYN_INTT
            c_gain = nv_lux_per_count[0];;
#else
			mn_sensor.als.factory.lux_per_count = nv_lux_per_count[0];
#endif
		ret = of_property_read_u32_array(node, "wait_time", nv_wait_time, ARRAY_SIZE(nv_wait_time));
		if (ret == 0)
			mn_sensor.wait = (nv_wait_time[0] << 4);

		ret = of_property_read_u32_array(node, "ps_wait_time", nv_ps_wait_time, ARRAY_SIZE(nv_ps_wait_time));
		if (ret == 0)
			mn_sensor.ps_wait = (nv_ps_wait_time[0] << 4);

		ret = of_property_read_u32_array(node, "als_intt", nv_als_intt, ARRAY_SIZE(nv_als_intt));
		if (ret == 0)
			mn_sensor.als.integration_time  = (nv_als_intt[0] << 2);

		ret = of_property_read_u32_array(node, "als_gain", nv_als_gain, ARRAY_SIZE(nv_als_gain));
		if (ret == 0)
			mn_sensor.als.gain = nv_als_gain[0];

		ret = of_property_read_u32_array(node, "als_filter", nv_als_filter, ARRAY_SIZE(nv_als_filter));
		if (ret == 0)
			mn_sensor.als.cycle = nv_als_filter[0];

		ret = of_property_read_u32_array(node, "als_persist", nv_als_persist, ARRAY_SIZE(nv_als_persist));
		if (ret == 0)
			mn_sensor.als.persist = (nv_als_persist[0] << 2);

		ret = of_property_read_u32_array(node, "als_intr_thd_offset", nv_als_intr_thd_offset, ARRAY_SIZE(nv_als_intr_thd_offset));
		if (ret == 0)
			als_intr_thd_offset = nv_als_intr_thd_offset[0];

		ret = of_property_read_u32_array(node, "als_report_type", nv_als_report_type, ARRAY_SIZE(nv_als_report_type));
		if (ret == 0)
			mn_sensor.als.report_type = nv_als_report_type[0];
#if ALS_DYN_INTT
		ret = of_property_read_u32_array(node, "als_dyn_intt_0", nv_als_dyn_intt_0, ARRAY_SIZE(nv_als_dyn_intt_0));
		if (ret == 0)
		{
			 als_dynamic_intt_intt[0] = (nv_als_dyn_intt_0[0] << 2);
			 als_dynamic_intt_value[0] = als_intt_value[nv_als_dyn_intt_0[0]];
        }
		ret = of_property_read_u32_array(node, "als_dyn_intt_gain_0", nv_als_dyn_intt_gain_0, ARRAY_SIZE(nv_als_dyn_intt_gain_0));
		if (ret == 0)
			als_dynamic_intt_gain[0] = nv_als_dyn_intt_gain_0[0];

		ret = of_property_read_u32_array(node, "als_dyn_intt_1", nv_als_dyn_intt_1, ARRAY_SIZE(nv_als_dyn_intt_1));
		if (ret == 0)
		{
			als_dynamic_intt_intt[1] = (nv_als_dyn_intt_1[0] << 2);
			als_dynamic_intt_value[1] = als_intt_value[nv_als_dyn_intt_1[0]];
        }

		ret = of_property_read_u32_array(node, "als_dyn_intt_gain_1", nv_als_dyn_intt_gain_1, ARRAY_SIZE(nv_als_dyn_intt_gain_1));
		if (ret == 0)
			als_dynamic_intt_gain[1] = nv_als_dyn_intt_gain_1[0];
#endif
#if ALS_LSRC
		ret = of_property_read_u32_array(node, "als_scale_gain", nv_als_scale_gain, ARRAY_SIZE(nv_als_scale_gain));
		if (ret == 0)
			scale_gain = nv_als_scale_gain[0];

		ret = of_property_read_u32_array(node, "als_offset_gain", nv_als_offset_gain, ARRAY_SIZE(nv_als_offset_gain));
		if (ret == 0)
			offset_gain = nv_als_offset_gain[0];
#endif



		ret = of_property_read_u32_array(node, "ps_intt", nv_ps_intt, ARRAY_SIZE(nv_ps_intt));
		if (ret == 0)
			mn_sensor.ps.integration_time = (nv_ps_intt[0]<<2);

		ret = of_property_read_u32_array(node, "ps_gain", nv_ps_gain, ARRAY_SIZE(nv_ps_gain));
		if (ret == 0)
			mn_sensor.ps.gain = nv_ps_gain[0];

		ret = of_property_read_u32_array(node, "ps_filter", nv_ps_filter, ARRAY_SIZE(nv_ps_filter));
		if (ret == 0)
			mn_sensor.ps.cycle = nv_ps_filter[0];

		ret = of_property_read_u32_array(node, "ps_ir_drive", nv_ps_ir_drive, ARRAY_SIZE(nv_ps_ir_drive));
		if (ret == 0)
			mn_sensor.ps.ir_drive = nv_ps_ir_drive[0];

		ret = of_property_read_u32_array(node, "ps_persist", nv_ps_persist, ARRAY_SIZE(nv_ps_persist));
		if (ret == 0)
			mn_sensor.ps.persist = nv_ps_persist[0];

		ret = of_property_read_u32_array(node, "ps_self_test_max_ct", nv_ps_self_test_max_ct, ARRAY_SIZE(nv_ps_self_test_max_ct));
		if (ret == 0)
			ps_self_test_max_ct = nv_ps_self_test_max_ct[0];
#if POCKET_DETECTION
		ret = of_property_read_u32_array(node, "als_low_lux_offset", nv_als_low_lux_offset, ARRAY_SIZE(nv_als_low_lux_offset));
		if (ret == 0)
			low_lux_offset = nv_als_low_lux_offset[0];
#endif
    }
    else
    {
        APS_ERR("Device Tree: can not find alsps node!. Go to use old cust info\n");
		return -1;
    }

    APS_LOG(">>>>>>>>>>>>>>>mn257x_parse_dt 0x%x, 0x%x \r\n ", mn_sensor.wait, mn_sensor.ps.cycle);

    return 0;
}
#endif /*DTS_PARSE*/

static void initial_global_variable(struct i2c_client *client, struct mn_sensor_priv *obj)
{
#if ALS_DYN_INTT
    int idx=0, gain_value=0, intt_value=0, total_value=0;
#endif
    /* read revno*/
    mn257x_I2C_Read(client, DEVREG_REV_ID, 2);
    mn_sensor.revno = gRawData.raw_bytes[0] | gRawData.raw_bytes[1] << 8;
	
    //general setting
    mn_sensor.power = MN_POWER_ON;
    mn_sensor.reset = MN_RESETN_RUN;
    mn_sensor.als_single_pixel = ALS_SIGLE_DIS;
    mn_sensor.mode = MN_MODE_IDLE;
#if !DTS_PARSE
    mn_sensor.wait = MN_WAIT_0_MS;
#endif
    mn_sensor.osc_sel = MN_OSC_SEL_1MHZ;


    //als setting
    mn_sensor.als.polling_mode = obj->hw->polling_mode_als;
#if !DTS_PARSE
    mn_sensor.als.integration_time = MN_ALS_INTT_1024;
    mn_sensor.als.gain = MN_GAIN_LOW;
    mn_sensor.als.adc = MN_PSALS_ADC_11;
    mn_sensor.als.cycle = MN_CYCLE_16;
    mn_sensor.als.persist = MN_PERIST_1;
    mn_sensor.als.report_type = CMC_BIT_DYN_INT; //CMC_BIT_RAW; //CMC_BIT_DYN_INT;
#endif
    mn_sensor.als.als_rs = MN_RS_0;
    mn_sensor.als.als_intb_nonlos = MN_T_DIS;
    mn_sensor.als.interrupt_channel_select = MN_ALS_INT_CHSEL_1;

	if(mn_sensor.revno == 0x0288)
        mn_sensor.als.als_std = (MN_ALS_PRE << 1);
    else
        mn_sensor.als.als_std = MN_ALS_PRE;

	mn_sensor.als.compare_reset = MN_CMP_RUN;
    mn_sensor.als.lock = MN_UN_LOCK;
    mn_sensor.als.high_threshold = L_SENSOR_HTHD;//obj->hw->als_threshold_high;
    mn_sensor.als.low_threshold = L_SENSOR_LTHD;//obj->hw->als_threshold_low;
    //als factory
    mn_sensor.als.factory.calibration_enable =  false;
    mn_sensor.als.factory.calibrated = false;
#if !DTS_PARSE
    mn_sensor.als.factory.lux_per_count = LUX_PER_COUNT;
#endif
#if ALS_DYN_INTT
    if(mn_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        dynamic_intt_idx = dynamic_intt_init_idx;
        mn_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
        mn_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
        dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
        dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
    }
#if !DTS_PARSE
    c_gain = 300; // 300/1000=0.3 /*Lux per count*/
#endif
    if(mn_sensor.revno != 0x8188)  //3638
    {
        if(als_dynamic_intt_gain[0] == MN_GAIN_MID)
        {
            gain_value = 8;
        }
        else if (als_dynamic_intt_gain[0] == MN_GAIN_LOW)
        {
            gain_value = 1;
        }

        intt_value = als_dynamic_intt_value[0] / als_dynamic_intt_value[1];
        total_value = gain_value * intt_value;

        for(idx = 0; idx < rs_num;  idx++)
    	{
    	    if(total_value < als_rs_value[idx])
    	    {
    	        break;
    	    }
    	}
    	APS_LOG("[%s]: idx=%d, als_rs_value=%d, total_value=%d\r\n", __func__, idx, als_rs_value[idx-1], total_value);
        mn_sensor.als.als_rs = ((idx-1)<<5);
        als_dynamic_intt_high_thr[0] = als_dynamic_intt_high_thr[0]/total_value;
        als_dynamic_intt_low_thr[0] = als_dynamic_intt_low_thr[0]/total_value;
    }
#else
    //als auto gain, dont support
    if(mn_sensor.als.gain == MN_AG_EN)
    {
        mn_sensor.als.als_ag_l = MN_AG_L;
        mn_sensor.als.als_ag_h = MN_AG_M;
        mn_sensor.als.als_aintt_l = (MN_ALS_INTT_1024>>2);
        mn_sensor.als.als_aintt_h = (MN_ALS_INTT_1024>>2);
        mn_sensor.als.als_ag_thd = 7500;
    }
#endif

    //ps setting
    mn_sensor.ps.polling_mode = obj->hw->polling_mode_ps;
#if !DTS_PARSE
    mn_sensor.ps.integration_time = MN_PS_INTT_144;
    mn_sensor.ps.gain = MN_GAIN_MID;
    mn_sensor.ps.cycle = MN_CYCLE_16;
    mn_sensor.ps.persist = MN_PERIST_1;
    mn_sensor.ps.ir_drive = MN_IR_DRIVE_100;
    mn_sensor.ps_wait = MN_WAIT_100_MS;
#endif
    mn_sensor.ps.adc = MN_PSALS_ADC_11;
    mn_sensor.ps.ps_rs = MN_RS_0;
    mn_sensor.ps.ps_intb_nonlos = MN_T_DIS;
    mn_sensor.ps.ps_std = MN_PS_PRE;
    mn_sensor.ps.ir_on_control = MN_IR_ON_CTRL_ON;
    mn_sensor.ps.ir_mode = MN_IR_MODE_CURRENT;
    mn_sensor.ps.compare_reset = MN_CMP_RUN;
    mn_sensor.ps.lock = MN_UN_LOCK;
    mn_sensor.ps.high_threshold = obj->hw->ps_threshold_high;
    mn_sensor.ps.low_threshold = obj->hw->ps_threshold_low;
    //ps factory
    mn_sensor.ps.factory.calibration_enable =  false;
    mn_sensor.ps.factory.calibrated = false;
    mn_sensor.ps.factory.cancelation= 0;
#if !DTS_PARSE
#if MN_SELF_TEST
    ps_self_test_max_ct = 2000;
    ps_factory_ct = 0;
    ps_factory_offset = 800;
    ps_factory_hystersis = 200;
#endif
    //knock_default_l = ps_factory_ct + (9 * mn_sensor.ps.high_threshold / 4); // 2cm_thd = ct + (3/2)^2 * 3cm_offset;
    //knock_default_h = ps_factory_ct + 9 * mn_sensor.ps.high_threshold; // 1cm_thd = ct + (3/1)^2 * 3cm_offset;
#endif /*!DTS_PARSE*/
    mn_sensor.ps.high_threshold = ps_factory_ct + ps_factory_offset;
    mn_sensor.ps.low_threshold = mn_sensor.ps.high_threshold - ps_factory_hystersis;

    //write setting to sensor
    write_global_variable(client);
}

#if ALS_LSRC
int lsrc_raw_convert_to_adc(u32 ch0, u32 ch1)
{
    u32 als_offset=0;
    u16 nor_raw=0;

    lsrc_ratio = ch0*1000 / ch1;

    if(ch1 > 0)
    {
    	als_offset = (ch0 * ch0) / (ch0+ch1) * offset_gain / 1000;
        APS_LOG("[%s]: als_offset=%u \r\n", __func__, als_offset);
    	if(als_offset < ((1000-scale_gain)*ch1 / 1000))
    		nor_raw = ch1 - als_offset;
    	else
    		nor_raw = ch1*scale_gain/1000;
    }
    else
    {
        nor_raw = ch1;
    }
    lsrc_raw = nor_raw;
    lsrc_als_offset = als_offset;

    APS_LOG("[%s]: ch0=%d, ch1=%d, nor_raw=%d \r\n", __func__, ch0, ch1, nor_raw);

    return nor_raw;
}
#endif


#if ALS_DYN_INTT
long raw_convert_to_lux(u16 raw_data)
{
    long lux = 0;
    long dyn_intt_raw = 0;
    int gain_value = 0;
#if ALS_LSRC
    u16 als_lsrc_raw = 0;
#endif

    if(mn_sensor.revno != 0x8188)  //3638
    {
#if ALS_LSRC
        als_dyn_intt_ch0 = mn_sensor.als.data.channels[0];
#endif
        dyn_intt_raw = raw_data;
    }
    else
    {
        if(mn_sensor.als.gain == MN_GAIN_MID)
        {
            gain_value = 8;
        }
        else if (mn_sensor.als.gain == MN_GAIN_LOW)
        {
            gain_value = 1;
        }
#if ALS_LSRC
        als_dyn_intt_ch0 = (mn_sensor.als.data.channels[0] * 10) / (10 * gain_value * als_dynamic_intt_value[dynamic_intt_idx] / als_dynamic_intt_value[1]); //float calculate
#endif
        dyn_intt_raw = (raw_data * 10) / (10 * gain_value * als_dynamic_intt_value[dynamic_intt_idx] / als_dynamic_intt_value[1]); //float calculate
    }

    if(dyn_intt_raw > 0xffff)
        mn_sensor.als.dyn_intt_raw = 0xffff;
    else
        mn_sensor.als.dyn_intt_raw = dyn_intt_raw;
#if ALS_LSRC
    als_lsrc_raw = lsrc_raw_convert_to_adc(als_dyn_intt_ch0, dyn_intt_raw);
    lux = c_gain * als_lsrc_raw;
    lsrc_lux = lux / 1000;
    als_lux = c_gain * dyn_intt_raw / 1000;
#else
    lux = c_gain * mn_sensor.als.dyn_intt_raw;
#endif

    if(lux >= (dynamic_intt_max_lux*dynamic_intt_min_unit)){
        APS_LOG("[%s]:raw_convert_to_lux: change max lux\r\n", __func__);
        lux = dynamic_intt_max_lux * dynamic_intt_min_unit;
    }
    else if(lux <= (dynamic_intt_min_lux*dynamic_intt_min_unit)){
        APS_LOG("[%s]:raw_convert_to_lux: change min lux\r\n", __func__);
        lux = dynamic_intt_min_lux * dynamic_intt_min_unit;
    }

    return lux;
}
#endif
static int lcount=0;
static int mn257x_get_als_value(struct mn_sensor_priv *obj, u16 als)
{
    int idx;
    int invalid = 0;
#if ALS_DYN_INTT
	long now_lux=0, lux_tmp=0;
    bool change_flag = false;
#endif

    switch(mn_sensor.als.report_type)
    {
        case CMC_BIT_RAW:
#if ALS_LSRC
            lsrc_lux = lsrc_raw_convert_to_adc(mn_sensor.als.data.channels[0], mn_sensor.als.data.channels[1]);
            return lsrc_lux;
#else
			return als;
#endif

        break;
        case CMC_BIT_PRE_COUNT:
#if ALS_LSRC
            lsrc_lux = (lsrc_raw_convert_to_adc(mn_sensor.als.data.channels[0], mn_sensor.als.data.channels[1]) * mn_sensor.als.factory.lux_per_count)/1000;
            als_lux = (als * mn_sensor.als.factory.lux_per_count)/1000;
            return lsrc_lux;
#else
			return (als * mn_sensor.als.factory.lux_per_count)/1000;
#endif
        break;
        case CMC_BIT_TABLE:
            for(idx = 0; idx < obj->als_level_num; idx++)
            {
                if(als < obj->hw->als_level[idx])
                {
                    break;
                }
            }

            if(idx >= obj->als_value_num)
            {
                APS_ERR("exceed range\n");
                invalid = 1;
                if(obj->als_value_num >= 1)
                    idx = obj->als_value_num - 1;
            }

            if(!invalid)
            {
                APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
                return obj->hw->als_value[idx];
            }
            else
            {
                APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
                return als;
            }
        break;
#if ALS_DYN_INTT
		case CMC_BIT_DYN_INT:
            if(!(++lcount &  0x0000003F) ){
                APS_LOG("[%s]: dynamic_intt_idx=%d, als_dynamic_intt_value=%d, dynamic_intt_gain=%d, als=%d\r\n",
                         __func__, dynamic_intt_idx, als_dynamic_intt_value[dynamic_intt_idx], als_dynamic_intt_gain[dynamic_intt_idx], als);
            }
            if(als > dynamic_intt_high_thr)
        	{
          		if(dynamic_intt_idx == (als_dynamic_intt_intt_num - 1)){

          		    lux_tmp = raw_convert_to_lux(als);
        	      	APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>> INTT_MAX_LUX\r\n");
          		}
                else{
                    change_flag = true;
        			als  = dynamic_intt_high_thr;
              		lux_tmp = raw_convert_to_lux(als);
                    dynamic_intt_idx++;
                    if(dynamic_intt_idx >= (als_dynamic_intt_intt_num - 1))
                        dynamic_intt_idx = (als_dynamic_intt_intt_num - 1);
                    APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>>change INTT high: %d, raw: %d \r\n", dynamic_intt_idx, als);
                }
            }
            else if(als < dynamic_intt_low_thr)
            {
                if(dynamic_intt_idx == 0){
                    lux_tmp = raw_convert_to_lux(als);
                }
                else{
                    change_flag = true;
        			als  = dynamic_intt_low_thr;
                	lux_tmp = raw_convert_to_lux(als);
                    dynamic_intt_idx--;
                    if(dynamic_intt_idx <= 0)
                        dynamic_intt_idx = 0;
                    APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>>change INTT low: %d, raw: %d \r\n", dynamic_intt_idx, als);
                }
            }
            else
            {
            	lux_tmp = raw_convert_to_lux(als);
            }

            now_lux = lux_tmp;
            dynamic_intt_lux = now_lux/dynamic_intt_min_unit;

            if(change_flag == true)
            {
                APS_LOG("[%s]: ALS_DYN_INTT:Chang Setting \r\n", __func__);
                mn_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                mn_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
                mn257x_fast_update(obj->client);

                mutex_lock(&sensor_mutex);

                if(mn_sensor.revno == 0x0288)
                    mn257x_I2C_Write(obj->client, DEVREG_ENABLE, mn_sensor.wait | mn_sensor.als_single_pixel | mn_sensor.mode);
                else
                    mn257x_I2C_Write(obj->client, DEVREG_ENABLE, mn_sensor.wait | mn_sensor.mode);
                mn257x_I2C_Write(obj->client, DEVREG_RESET, MN_POWER_ON | MN_RESETN_RUN);
                mutex_unlock(&sensor_mutex);
            }
            return dynamic_intt_lux;
		break;
#endif
    }

    return 0;
}

static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    struct i2c_client *client = obj->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;
    u8 buf[4];

#if PS_11_BIT
    low_thd  = low_thd << 5;
    high_thd = high_thd << 5;
#endif
    buf[3] = high_msb = (uint8_t) (high_thd >> 8);
    buf[2] = high_lsb = (uint8_t) (high_thd & 0x00ff);
    buf[1] = low_msb  = (uint8_t) (low_thd >> 8);
    buf[0] = low_lsb  = (uint8_t) (low_thd & 0x00ff);
    mutex_lock(&sensor_mutex);
    mn257x_I2C_Write_Block(client, DEVREG_PS_ILTL, buf, 4);
    mutex_unlock(&sensor_mutex);
    APS_LOG("%s: low_thd = %d, high_thd = %d \n", __FUNCTION__, low_thd, high_thd);
    return 0;
}

static int set_lsensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    struct i2c_client *client = obj->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;
    u8 buf[4];

    buf[3] = high_msb = (uint8_t) (high_thd >> 8);
    buf[2] = high_lsb = (uint8_t) (high_thd & 0x00ff);
    buf[1] = low_msb  = (uint8_t) (low_thd >> 8);
    buf[0] = low_lsb  = (uint8_t) (low_thd & 0x00ff);
    mutex_lock(&sensor_mutex);
    mn257x_I2C_Write_Block(client, DEVREG_ALS_ILTL, buf, 4);
    mutex_unlock(&sensor_mutex);
    if(!(lcount & 0x0000003F))
        APS_LOG("%s: low_thd = %d, high_thd = %d \n", __FUNCTION__, low_thd, high_thd);
    return 0;
}


/*----------------------------------------------------------------------------*/
static void mn257x_dumpReg(struct i2c_client *client)
{
    APS_LOG("chip id REG 0x00 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ENABLE));
    APS_LOG("chip id REG 0x01 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ALS_CONFIG));
    APS_LOG("chip id REG 0x02 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ALS_FILT));
    APS_LOG("chip id REG 0x03 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_CONFIG));
    APS_LOG("chip id REG 0x04 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_FILT));
    APS_LOG("chip id REG 0x05 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_LED_CONFIG));
    APS_LOG("chip id REG 0x06 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_INT));
    APS_LOG("chip id REG 0x07 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ALS_INT));
    APS_LOG("chip id REG 0x11 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_RESET));
    APS_LOG("chip id REG 0x12 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ALS_STATUS));
    APS_LOG("chip id REG 0x13 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_C0DATAL));
    APS_LOG("chip id REG 0x14 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_C0DATAH));
    APS_LOG("chip id REG 0x15 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_C1DATAL));
    APS_LOG("chip id REG 0x16 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_C1DATAH));
    APS_LOG("chip id REG 0x1B = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_STATUS));
    APS_LOG("chip id REG 0x1C = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_ADATAL));
    APS_LOG("chip id REG 0x1D = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_ADATAH));
    APS_LOG("chip id REG 0x1E = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_RDATAL));
    APS_LOG("chip id REG 0x1F = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_RDATAH));
    APS_LOG("chip id REG 0x20 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_REV_ID));
    APS_LOG("chip id REG 0x21 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_CHIP_ID));
    APS_LOG("chip id REG 0x22 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_OFSL));
    APS_LOG("chip id REG 0x23 = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_OFSH));
    APS_LOG("chip id REG 0x24 = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
    APS_LOG("chip id REG 0x25 = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));
    APS_LOG("chip id REG 0xfb = 0x%x\n", i2c_smbus_read_byte_data(client, 0xfb));
    APS_LOG("chip id REG 0xfc = 0x%x\n", i2c_smbus_read_byte_data(client, 0xfc));
}

int hw8k_init_device(struct i2c_client *client)
{
    APS_LOG("hw8k_init_device.........\r\n");
    mn257x_i2c_client = client;
    APS_LOG(" I2C Addr==[0x%x],line=%d\n", mn257x_i2c_client->addr,__LINE__);

    return 0;
}

int mn257x_get_addr(struct alsps_hw *hw, struct mn257x_i2c_addr *addr)
{
    if(!hw || !addr)
    {
        return -EFAULT;
    }
    addr->write_addr= hw->i2c_addr[0];

    return 0;
}

static void mn257x_power(struct alsps_hw *hw, unsigned int on)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0))
    static unsigned int power_on = 0;

    if(hw->power_id != POWER_NONE_MACRO)
    {
        if(power_on == on)
        {
            APS_LOG("ignore power control: %d\n", on);
        }
        else if(on)
        {
            if(!hwPowerOn(hw->power_id, hw->power_vol, MN_DEV_NAME))
            {
                APS_ERR("power on fails!!\n");
            }
        }
        else
        {
            if(!hwPowerDown(hw->power_id, MN_DEV_NAME))
            {
                APS_ERR("power off fail!!\n");
            }
        }
    }
    power_on = on;
#endif
}



static void mn257x_report_ps_status(int ps_status)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    int err;
    APS_LOG("[%s]: mn_sensor.ps.data.data=%d, ps status=%d \r\n", __func__, mn_sensor.ps.data.data, (mn_sensor.ps.compare_low >> 3));

    err = ps_report_interrupt_data(ps_status);
    if(err != 0)  //if report status is fail, write unlock again.
    {
        APS_ERR("mn_sensor_eint_work err: %d\n", err);
    	mn257x_I2C_Write(obj->client,DEVREG_PS_STATUS, MN_CMP_RESET | MN_UN_LOCK);
    }
}
#if 0
static void mn257x_report_als_lux(int lux)
{
    int err = 0;

#if ALS_DYN_INTT
    APS_LOG("[%s]: als_raw=%d, lux=%d \r\n", __func__, mn_sensor.als.dyn_intt_raw, lux);
#else
    APS_LOG("[%s]: als_raw=%d, lux=%d \r\n", __func__, mn_sensor.als.data.channels[1], lux);
#endif

    if((err = als_data_report(alsps_context_obj->idev, lux, SENSOR_STATUS_ACCURACY_MEDIUM)))
    {
       APS_ERR("mn_sensor call als_data_report fail = %d\n", err);
    }
}
#endif
#if POCKET_DETECTION
static void mn257x_intr_als_set_pkd_thd(struct mn_sensor_priv* obj, u16 raw)
{
	if(raw <= low_lux_offset)
	{
		APS_ERR("als_data(%d)<THRESHOLD(%d), Change bright to dark, lux(%d)\n", raw, low_lux_offset, pkd_lux);
		set_lsensor_intr_threshold( 0, low_lux_offset );
	}
	else
	{
		APS_ERR("als_data(%d)>THRESHOLD(%d), Change dark to bright, lux(%d)\n", raw, low_lux_offset, pkd_lux);
		set_lsensor_intr_threshold( low_lux_offset, 65535 );
	}
}
#endif

static void mn257x_intr_als_set_thd(struct mn_sensor_priv* obj, u16 raw)
{
    u16 thd_offset = 0;
#if ALS_DYN_INTT
    if(mn_sensor.als.report_type == CMC_BIT_DYN_INT)
    {

        if(mn_sensor.revno == 0x8188)
        {
            thd_offset = als_intr_thd_offset;
        }
        else
        {
            if(dynamic_intt_idx == 0)
                thd_offset= als_intr_thd_offset/als_rs_value[mn_sensor.als.als_rs>>5];
            else
                thd_offset = als_intr_thd_offset;
        }

        //set low threshold
        if(raw <= thd_offset)    //overflow
        {
            if(dynamic_intt_idx == 0)
                mn_sensor.als.low_threshold = 0;
            else
                mn_sensor.als.low_threshold = als_dynamic_intt_low_thr[dynamic_intt_idx];
        }
        else
            mn_sensor.als.low_threshold = raw - thd_offset;

        //set low threshold
        if(raw >= (65535-thd_offset)) //overflow
            mn_sensor.als.high_threshold = 65535;
        else
            mn_sensor.als.high_threshold = raw + thd_offset;

    }
    else
#endif
    {
        //set low threshold
        if(mn_sensor.als.data.channels[1] < thd_offset)    //overflow
            mn_sensor.als.low_threshold = 0;
        else
            mn_sensor.als.low_threshold = raw - thd_offset;

        //set high threshold
        if(mn_sensor.als.data.channels[1] > (0xffff-thd_offset)) //overflow
            mn_sensor.als.high_threshold = 65535;
        else
            mn_sensor.als.high_threshold = raw + thd_offset;

    }

    //set new threshold
    set_lsensor_intr_threshold(mn_sensor.als.low_threshold, mn_sensor.als.high_threshold);
}

static void mn257x_intr_als_report_lux(void)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    u16 lux;

    als_lux = lux = mn257x_get_als_value(obj, mn_sensor.als.data.channels[1]);

#if ALS_DYN_INTT
	if(mn_sensor.als.report_type == CMC_BIT_DYN_INT)
	{
		if(mn_sensor.revno == 0x8188)
		{
#if POCKET_DETECTION
			if(atomic_read(&driver_suspend_flag) == 1)
			{
				mn257x_intr_als_set_pkd_thd(obj, mn_sensor.als.data.channels[1]);
			}
			else
#endif
			{
				mn257x_intr_als_set_thd(obj, mn_sensor.als.data.channels[1]);
			}
		}
		else
		{
			mn257x_intr_als_set_thd(obj, mn_sensor.als.dyn_intt_raw);
		}
	}
	else
#endif
    {
        mn257x_intr_als_set_thd(obj, mn_sensor.als.data.channels[1]);
    }

}
/*----------------------------------------------------------------------------*/

int mn257x_read_als(struct i2c_client *client)
{
    struct mn_sensor_priv *obj = i2c_get_clientdata(client);
    u8 buf[5];

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }
    mutex_lock(&sensor_mutex);
    mn257x_I2C_Read(obj->client, DEVREG_ALS_STATUS, 5);
    buf[0] = gRawData.raw_bytes[0];
    buf[1] = gRawData.raw_bytes[1];
    buf[2] = gRawData.raw_bytes[2];
    buf[3] = gRawData.raw_bytes[3];
    buf[4] = gRawData.raw_bytes[4];
    mutex_unlock(&sensor_mutex);

    mn_sensor.als.saturation = (buf[0] & 0x20);
    mn_sensor.als.compare_high = (buf[0] & 0x10);
    mn_sensor.als.compare_low = (buf[0] & 0x08);
    mn_sensor.als.interrupt_flag = (buf[0] & 0x04);
    mn_sensor.als.compare_reset = (buf[0] & 0x02);
    mn_sensor.als.lock = (buf[0] & 0x01);
    mn_sensor.als.data.channels[0] = (buf[2]<<8) | buf[1];
    mn_sensor.als.data.channels[1] = (buf[4]<<8) | buf[3];

//	APS_LOG("als: ~~~~ ALS ~~~~~ \n");
//	APS_LOG("als: buf = 0x%x\n", buf[0]);
//	APS_LOG("als: sat = 0x%x\n", mn_sensor.als.saturation);
//	APS_LOG("als: cmp h = 0x%x, l = %d\n", mn_sensor.als.compare_high, mn_sensor.als.compare_low);
//	APS_LOG("als: int_flag = 0x%x\n",mn_sensor.als.interrupt_flag);
//	APS_LOG("als: cmp_rstn = 0x%x, lock = 0x%0x\n", mn_sensor.als.compare_reset, mn_sensor.als.lock);
//  APS_LOG("read als channel 0 = %d\n", mn_sensor.als.data.channels[0]);
//  APS_LOG("read als channel 1 = %d\n", mn_sensor.als.data.channels[1]);

    if(mn_sensor.revno == 0x0288 && mn_sensor.als.gain == MN_AG_EN)
    {
        mutex_lock(&sensor_mutex);
        mn257x_I2C_Read(obj->client, 0x28, 3);
        buf[0] = gRawData.raw_bytes[0];
        buf[1] = gRawData.raw_bytes[1];
        buf[2] = gRawData.raw_bytes[2];
        mutex_unlock(&sensor_mutex);
        mn_sensor.als.als_ag_l_value = buf[0];
        mn_sensor.als.als_ag_h_value = buf[1];
        mn_sensor.als.als_ag_flag = (buf[2] & 0x10);
        mn_sensor.als.als_ag_rs_weight = (buf[2] & 0x0f);
//      APS_LOG("read als_ag low value = %d\n", mn_sensor.als.als_ag_l_value);
//      APS_LOG("read als_ag high value = %d\n", mn_sensor.als.als_ag_h_value);
//      APS_LOG("read als_ag flag = 0x%x\n", mn_sensor.als.als_ag_flag);
//      APS_LOG("read als_ag rs wegiht = %d\n", mn_sensor.als.als_ag_rs_weight);
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
int mn257x_read_ps(struct i2c_client *client)
{
    u8 buf[5];

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }
    mutex_lock(&sensor_mutex);
    mn257x_I2C_Read(client,DEVREG_PS_STATUS, 5);
    buf[0] = gRawData.raw_bytes[0];
    buf[1] = gRawData.raw_bytes[1];
    buf[2] = gRawData.raw_bytes[2];
    buf[3] = gRawData.raw_bytes[3];
    buf[4] = gRawData.raw_bytes[4];
    mutex_unlock(&sensor_mutex);

    if(mn_sensor.revno != 0x8188)
        mn_sensor.ps.saturation_1 = (buf[0] & 0x40);
    mn_sensor.ps.saturation = (buf[0] & 0x20);
    mn_sensor.ps.compare_high = (buf[0] & 0x10);
    mn_sensor.ps.compare_low = (buf[0] & 0x08);
    mn_sensor.ps.interrupt_flag = (buf[0] & 0x04);
    mn_sensor.ps.compare_reset = (buf[0] & 0x02);
    mn_sensor.ps.lock= (buf[0] & 0x01);
    mn_sensor.ps.data.ir_data = (buf[2]<<8) | buf[1];
    mn_sensor.ps.data.data = (buf[4]<<8) | buf[3];
#if PS_11_BIT
    mn_sensor.ps.data.ir_data = mn_sensor.ps.data.ir_data >> 5;
    mn_sensor.ps.data.data = mn_sensor.ps.data.data >> 5;
#endif
//	APS_LOG("ps: ~~~~ PS ~~~~~ \n");
//	APS_LOG("ps: buf = 0x%x\n", buf[0]);
//	if(mn_sensor.revno != 0x8188)
//	    APS_LOG("ps: sat_1 = 0x%x\n", mn_sensor.ps.saturation_1);
//	APS_LOG("ps: sat = 0x%x\n", mn_sensor.ps.saturation);
//	APS_LOG("ps: cmp h = 0x%x, l = 0x%x\n", mn_sensor.ps.compare_high, mn_sensor.ps.compare_low);
//	APS_LOG("ps: int_flag = 0x%x\n",mn_sensor.ps.interrupt_flag);
//	APS_LOG("ps: cmp_rstn = 0x%x, lock = %x\n", mn_sensor.ps.compare_reset, mn_sensor.ps.lock);
//	APS_LOG("[%s]: data = %d\n", __func__, mn_sensor.ps.data.data);
//	APS_LOG("[%s]: ir data = %d\n", __func__, mn_sensor.ps.data.ir_data);
    return 0;
}

int factory_ps_data(void)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;

    if(enable_ps == 0)
    {
        APS_LOG("[%s]: ps is disabled \r\n", __func__);
    }
    APS_LOG("[%s]: enable_ps=%d, ps_raw=%d \r\n", __func__, enable_ps, mn_sensor.ps.data.data);

    return mn_sensor.ps.data.data;
}

int factory_als_data(void)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    u16 als_raw = 0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    if(enable_als == 1)
    {
        mn257x_read_als(obj->client);

        if(mn_sensor.als.report_type == CMC_BIT_DYN_INT)
        {
            int als_lux=0;
            als_lux = mn257x_get_als_value(obj, mn_sensor.als.data.channels[1]);
			APS_LOG("[%s]: als_lux = %d\r\n", __func__, als_lux);
			
        }
    }
    else
    {
        APS_LOG("[%s]: als is disabled \r\n", __func__);
    }

    if(mn_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        als_raw = mn_sensor.als.dyn_intt_raw;
        APS_LOG("[%s]: ALS_DYN_INTT: als_raw=%d \r\n", __func__, als_raw);
    }
    else
    {
        als_raw = mn_sensor.als.data.channels[1];
        APS_LOG("[%s]: als_raw=%d \r\n", __func__, als_raw);
    }

    return als_raw;
}

void mn257x_enable_ps(int enable)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable);

    APS_LOG("[%s]: ps enable = %d\r\n", __func__, enable);

#if VLED_POWER_OFF
    /*write all setting*/
    write_global_variable(obj->client);
#endif
        if(enable)
        {
            set_bit(CMC_BIT_PS, &obj->enable);
        }
        else
        {
            clear_bit(CMC_BIT_PS, &obj->enable);

        }
        if(enable_als == true)
            mn257x_fast_update(obj->client);
        mn257x_update_mode(obj->client);
        if(enable)
        {
            mn257x_report_ps_status(PS_FAR);    //First far

            mutex_lock(&sensor_mutex);
            mn257x_I2C_Write(obj->client,DEVREG_PS_STATUS, MN_CMP_RESET | MN_UN_LOCK);
    		mn257x_I2C_Write(obj->client,DEVREG_PS_STATUS, MN_CMP_RUN | MN_UN_LOCK);
    		mutex_unlock(&sensor_mutex);
        }


}

void mn257x_enable_als(int enable)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    APS_LOG("[%s]: als enable = %d\r\n", __func__, enable);
#if VLED_POWER_OFF
    /*write all setting*/
    write_global_variable(obj->client);
#endif
        if(enable)
        {
            set_bit(CMC_BIT_ALS, &obj->enable);
            mn_sensor.als.low_threshold = mn_sensor.als.high_threshold;
            set_lsensor_intr_threshold(mn_sensor.als.low_threshold,mn_sensor.als.high_threshold);

#if ALS_DYN_INTT
            if(mn_sensor.als.report_type == CMC_BIT_DYN_INT)
            {
                dynamic_intt_idx = dynamic_intt_init_idx;
                mn_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                mn_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
            }
#endif
            mn257x_fast_update(obj->client);
        }
        else
        {
            clear_bit(CMC_BIT_ALS, &obj->enable);
        }
        mn257x_update_mode(obj->client);

}

//for 3637
static int als_sensing_time(int intt, int adc, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int als_intt, als_adc, als_cycle;

    als_intt = als_intt_value[intt>>2];
    als_adc = adc_value[adc>>3];
    als_cycle = cycle_value[cycle];
    APS_LOG("ALS: INTT=%d, ADC=%d, Cycle=%d \r\n", als_intt, als_adc, als_cycle);

    sensing_us_time = (als_intt + als_adc*2*2) * 2 * als_cycle;
    sensing_ms_time = sensing_us_time / 1000;
    APS_LOG("[%s]: sensing=%d ms \r\n", __func__, sensing_ms_time);
    return (sensing_ms_time + 5);
}

static int ps_sensing_time(int intt, int adc, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int ps_intt, ps_adc, ps_cycle;

    ps_intt = ps_intt_value[intt>>2];
    ps_adc = adc_value[adc>>3];
    ps_cycle = cycle_value[cycle];
    APS_LOG("PS: INTT=%d, ADC=%d, Cycle=%d \r\n", ps_intt, ps_adc, ps_cycle);

    sensing_us_time = (ps_intt*3 + ps_adc*2*3) * ps_cycle;
    sensing_ms_time = sensing_us_time / 1000;
    APS_LOG("[%s]: sensing=%d ms\r\n", __func__, sensing_ms_time);
    return (sensing_ms_time + 5);
}

void mn257x_fast_update(struct i2c_client *client)
{
    int als_fast_time = 0;

    APS_FUN();
    mutex_lock(&sensor_mutex);
    als_fast_time = als_sensing_time(mn_sensor.als.integration_time, mn_sensor.als.adc, MN_CYCLE_1);

    mn257x_I2C_Write(client, DEVREG_RESET, MN_POWER_OFF | MN_RESETN_RESET);
#if 1
    if(mn_sensor.als.polling_mode == 0)
    {
        if(mn_sensor.revno == 0x0288)
            mn257x_I2C_Write(client, DEVREG_ALS_INT, mn_sensor.als.interrupt_channel_select | mn_sensor.als.persist | MN_INTTY_DISABLE);
        else
            mn257x_I2C_Write(client, DEVREG_ALS_INT, mn_sensor.als.als_std | mn_sensor.als.interrupt_channel_select | mn_sensor.als.persist | MN_INTTY_DISABLE);
    }
#endif
#if ALS_DYN_INTT
    if(mn_sensor.revno != 0x8188)  //3638
    {
        if(dynamic_intt_idx == 0)
        {
            mn257x_I2C_Write(client, DEVREG_ALS_FILT, mn_sensor.als.als_rs | mn_sensor.als.adc | MN_CYCLE_1);
        }
        else
        {
            mn257x_I2C_Write(client, DEVREG_ALS_FILT, MN_RS_0 | mn_sensor.als.adc | MN_CYCLE_1);
        }
    }
    else
#endif
    {
        mn257x_I2C_Write(client, DEVREG_ALS_FILT, mn_sensor.als.als_rs | mn_sensor.als.adc | MN_CYCLE_1);
    }
    if(mn_sensor.revno == 0x0288)
    {
        mn257x_I2C_Write(client, DEVREG_ALS_CONFIG, mn_sensor.als.als_std | mn_sensor.als.integration_time | mn_sensor.als.gain);
        mn257x_I2C_Write(client, DEVREG_ENABLE, mn_sensor.wait | mn_sensor.als_single_pixel | MN_MODE_ALS);
    }
    else
    {
        mn257x_I2C_Write(client, DEVREG_ALS_CONFIG, mn_sensor.als.als_intb_nonlos | mn_sensor.als.integration_time | mn_sensor.als.gain);
        mn257x_I2C_Write(client, DEVREG_ENABLE, mn_sensor.wait | MN_MODE_ALS);
    }
    mn257x_I2C_Write(client, DEVREG_RESET, MN_POWER_ON | MN_RESETN_RUN);
    mutex_unlock(&sensor_mutex);

    msleep(als_fast_time);
    APS_LOG("[%s]: msleep(%d)\r\n", __func__, als_fast_time);

    mutex_lock(&sensor_mutex);
    if(mn_sensor.als.polling_mode == 0)
    {
        //fast_mode is already ran one frame, so must to reset CMP bit for als intr mode
        //IDLE mode and CMMP reset
        mn257x_I2C_Write(client, DEVREG_ENABLE, mn_sensor.wait | MN_MODE_IDLE);
        mn257x_I2C_Write(client, DEVREG_ALS_STATUS, MN_CMP_RESET | MN_UN_LOCK);
        mn257x_I2C_Write(client, DEVREG_ALS_STATUS, MN_CMP_RUN | MN_UN_LOCK);
#if 1
        if(mn_sensor.revno == 0x0288)
            mn257x_I2C_Write(client, DEVREG_ALS_INT, mn_sensor.als.interrupt_channel_select | mn_sensor.als.persist | mn_sensor.als.interrupt_type);
        else
            mn257x_I2C_Write(client, DEVREG_ALS_INT, mn_sensor.als.als_std | mn_sensor.als.interrupt_channel_select | mn_sensor.als.persist | mn_sensor.als.interrupt_type);

#endif
    }

    mn257x_I2C_Write(client, DEVREG_RESET, MN_POWER_OFF | MN_RESETN_RESET);
#if ALS_DYN_INTT
    if(mn_sensor.revno != 0x8188)  //3638
    {
        if(dynamic_intt_idx == 0)
        {
            mn257x_I2C_Write(client, DEVREG_ALS_FILT, mn_sensor.als.als_rs | mn_sensor.als.adc | mn_sensor.als.cycle);
        }
        else
        {
            mn257x_I2C_Write(client, DEVREG_ALS_FILT, MN_RS_0 | mn_sensor.als.adc | mn_sensor.als.cycle);
        }
    }
    else
#endif
    {
        mn257x_I2C_Write(client, DEVREG_ALS_FILT, mn_sensor.als.als_rs | mn_sensor.als.adc | mn_sensor.als.cycle);
    }
    mutex_unlock(&sensor_mutex);
}

void mn257x_update_mode(struct i2c_client *client)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    int als_time = 0, ps_time = 0;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    uint32_t wait_time = 0;

    als_frame_time = als_time = als_sensing_time(mn_sensor.als.integration_time, mn_sensor.als.adc, mn_sensor.als.cycle);
    ps_frame_time = ps_time = ps_sensing_time(mn_sensor.ps.integration_time, mn_sensor.ps.adc, mn_sensor.ps.cycle);

	APS_LOG("mode selection =0x%x\n", enable_ps | (enable_als << 1));

	{
        //**** mode selection ****

        switch((enable_als << 1) | enable_ps)
        {
            case 0: //disable all
                mn_sensor.mode = MN_MODE_IDLE;
                wait_time = mn_sensor.wait;
                break;
            case 1: //als = 0, ps = 1
                mn_sensor.mode = MN_MODE_PS;
                wait_time = mn_sensor.ps_wait;
             break;
            case 2: //als = 1, ps = 0
                mn_sensor.mode = MN_MODE_ALS;
                wait_time = mn_sensor.wait;
                break;
            case 3: //als = 1, ps = 1
                mn_sensor.mode = MN_MODE_ALS_PS;
                wait_time = mn_sensor.wait;
                break;
        }



        mutex_lock(&sensor_mutex);
        mn257x_I2C_Write(client, DEVREG_RESET, MN_POWER_OFF | MN_RESETN_RESET);
        if(mn_sensor.revno == 0x0288)
            mn257x_I2C_Write(obj->client, DEVREG_ENABLE, mn_sensor.wait | mn_sensor.als_single_pixel | mn_sensor.mode);
        else
            mn257x_I2C_Write(obj->client, DEVREG_ENABLE, wait_time | mn_sensor.mode);

        if(mn_sensor.mode != MN_MODE_IDLE)    // if mode isnt IDLE, PWR_ON and RUN
            mn257x_I2C_Write(client, DEVREG_RESET, MN_POWER_ON | MN_RESETN_RUN);

        mutex_unlock(&sensor_mutex);

        //**** check setting ****
        if(enable_ps == 1)
        {
            APS_LOG("[%s] PS:low_thd = %d, high_thd = %d \n",__func__, mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);
        }
        if(enable_als == 1 && mn_sensor.als.polling_mode == 0)
        {
            APS_LOG("[%s] ALS:low_thd = %d, high_thd = %d \n",__func__, mn_sensor.als.low_threshold, mn_sensor.als.high_threshold);
        }
        if(mn_sensor.revno == 0x0288)
        {
            APS_LOG("[%s] reg0x00= 0x%x\n", __func__, mn_sensor.wait | mn_sensor.als_single_pixel | mn_sensor.mode);
            APS_LOG("[%s] reg0x07= 0x%x\n", __func__, mn_sensor.als.interrupt_channel_select | mn_sensor.als.persist | mn_sensor.als.interrupt_type);
        }
        else
        {
    	    APS_LOG("[%s] reg0x00= 0x%x\n", __func__,  mn_sensor.wait | mn_sensor.mode);
    	    APS_LOG("[%s] reg0x07= 0x%x\n", __func__, mn_sensor.als.als_std | mn_sensor.als.interrupt_channel_select | mn_sensor.als.persist | mn_sensor.als.interrupt_type);
        }
    	APS_LOG("[%s] reg0x06= 0x%x\n", __func__, mn_sensor.interrupt_control | mn_sensor.ps.persist |mn_sensor.ps.interrupt_type);
    	APS_LOG("[%s] reg0x11= 0x%x\n", __func__, mn_sensor.power | mn_sensor.reset);
    	APS_LOG("[%s] reg0x12= 0x%x\n", __func__, mn_sensor.als.compare_reset | mn_sensor.als.lock);
    	APS_LOG("[%s] reg0x1b= 0x%x\n", __func__, mn_sensor.ps.compare_reset | mn_sensor.ps.lock);
    }
}


/*----------------------------------------------------------------------------*/

static irqreturn_t mn257x_eint_func(int irq, void *desc)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;

    int_top_time = sched_clock();
	
	APS_LOG("[%s]: mn257x_eint_func in \r\n", __func__);

    if(!obj)
    {
        return IRQ_HANDLED;
    }

    disable_irq_nosync(obj->irq);

    schedule_delayed_work(&obj->eint_work, 0);

    return IRQ_HANDLED;
}
/*----------------------------------------------------------------------------*/
static void mn257x_eint_work(struct work_struct *work)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    int ps_report_status = PS_FAR;

    mn257x_read_ps(obj->client);
    mn257x_read_als(obj->client);
    if(mn_sensor.ps.interrupt_flag == MN_INT_TRIGGER)
    {
        if((mn_sensor.ps.compare_low >> 3) == 0) /*PS NEAR*/
            ps_report_status = PS_NEAR;

        else
            ps_report_status = PS_FAR;

#if PS_SUN_LIGHT
        if((mn_sensor.ps.saturation>>6) == 1 || mn_sensor.ps.data.ir_data > PS_MAX_IR)
        {
            APS_LOG("[%s]: ps_saturation=%d, ps_ir_data=%d \r\n", __func__, (mn_sensor.ps.saturation>>6), mn_sensor.ps.data.ir_data);
            ps_report_status = PS_FAR;
        }
#endif

        if(enable_ps)
        {
            //wake_lock_timeout(&ps_lock, 2*HZ);
            if(ps_report_status == PS_NEAR){
                APS_LOG("NEAR\n");
            }
            else if(ps_report_status == PS_FAR){
                APS_LOG("FAR\n");
            }
            else{
                ps_report_status = PS_FAR;
                APS_LOG("UNKNOWN EVENT\n");
            }
            mn257x_report_ps_status(ps_report_status);
        }
        //PS unlock interrupt pin and restart chip
		mutex_lock(&sensor_mutex);
		mn257x_I2C_Write(obj->client,DEVREG_PS_STATUS, MN_CMP_RUN | MN_UN_LOCK);
		mutex_unlock(&sensor_mutex);
    }

    if(mn_sensor.als.interrupt_flag == MN_INT_TRIGGER)
    {
#if POCKET_DETECTION
#if defined(CONFIG_FB)
        if(atomic_read(&driver_suspend_flag) == 1)
		{
			pkd_lux = raw_convert_to_lux(mn_sensor.als.data.channels[1]) / dynamic_intt_min_unit;
		}
#endif
#endif
		mn257x_intr_als_report_lux();
        //ALS unlock interrupt pin and restart chip
    	mutex_lock(&sensor_mutex);
    	mn257x_I2C_Write(obj->client,DEVREG_ALS_STATUS, MN_CMP_RESET | MN_UN_LOCK);
    	mn257x_I2C_Write(obj->client,DEVREG_ALS_STATUS, MN_CMP_RUN | MN_UN_LOCK);
    	mutex_unlock(&sensor_mutex);
    }

    enable_irq(obj->irq);
}

int mn257x_setup_eint(struct i2c_client *client)
{
    {
    struct mn_sensor_priv *obj = mn_sensor_obj;

    if (NULL == obj || NULL == obj->client){
        APS_ERR(" Parameter error \n");
        return EINVAL;
    }

#if defined(CONFIG_OF)
    if (obj->irq_node){
        unsigned int ints[2] = {0, 0};

        of_property_read_u32_array(obj->irq_node, "debounce", ints, ARRAY_SIZE(ints)); // read from dts(dws)

        gpio_set_debounce(ints[0], ints[1]);

        APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

        obj->irq = irq_of_parse_and_map(obj->irq_node, 0);
        APS_LOG("obj->irq = %d\n", obj->irq);
        if (!obj->irq){
            APS_ERR("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }

        /*register irq*/
        if(request_irq(obj->irq, mn257x_eint_func, IRQF_TRIGGER_LOW, "als-eint", NULL)) {
            APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
            return -EINVAL;
        }

    }else{
        APS_ERR("null irq node!!\n");
        return -EINVAL;
    }
#endif

    return 0;
}
}

static int mn257x_init_client(struct i2c_client *client)
{
    int err=0;
    /*  interrupt mode */
    APS_LOG("I2C Addr==[0x%x],line=%d\n", mn257x_i2c_client->addr, __LINE__);

    if((err = hw8k_init_device(client)) != 0)
    {
        APS_ERR("init dev: %d\n", err);
        return err;
    }

    return err;
}


/*----------------------------------------------------------------------------*/
static ssize_t mn257x_show_reg(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct i2c_client *client = mn_sensor_obj->client;

    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ENABLE));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ALS_CONFIG));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ALS_FILT));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_CONFIG));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_FILT));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_LED_CONFIG));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_INT));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ALS_INT));
    if(mn_sensor.als.polling_mode == 0)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x08 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ALS_ILTL));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ALS_ILTH));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0A value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ALS_IHTL));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0B value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ALS_IHTH));
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0C value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_ILTL));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_ILTH));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_IHTL));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_IHTH));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_RESET));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_ALS_STATUS));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x13 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_C0DATAL));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x14 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_C0DATAH));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x15 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_C1DATAL));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x16 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_C1DATAH));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_STATUS));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1C value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_ADATAL));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1D value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_ADATAH));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1E value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_RDATAL));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1F value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_RDATAH));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x22 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_OFSL));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x23 value = 0x%x\n", i2c_smbus_read_byte_data(client, DEVREG_PS_OFSH));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x26 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x26));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x27 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x27));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x28 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x28));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x29 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x29));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x2A value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x2A));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0xFC value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xFC));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0xFE value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xFE));
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t mn257x_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct mn_sensor_priv *obj = mn_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    if(!mn_sensor_obj)
    {
        APS_ERR("mn_sensor_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "chip is %s, ver is %s \n", MN_DEV_NAME, DRIVER_VERSION);
    len += snprintf(buf+len, PAGE_SIZE-len, "RENVO = 0x%x\n", mn_sensor.revno);
    len += snprintf(buf+len, PAGE_SIZE-len, "als/ps polling is %d-%d\n", mn_sensor.als.polling_mode, mn_sensor.ps.polling_mode);
    if(mn_sensor.revno == 0x0288)
       len += snprintf(buf+len, PAGE_SIZE-len, "wait = %d, als_single_pixel = %d, mode = %d\n",mn_sensor.wait >> 4, mn_sensor.als_single_pixel, mn_sensor.mode);
    else
       len += snprintf(buf+len, PAGE_SIZE-len, "wait = %d, mode = %d\n",mn_sensor.wait >> 4, mn_sensor.mode);
    len += snprintf(buf+len, PAGE_SIZE-len, "frame time ps=%dms, als=%dms\n", ps_frame_time, als_frame_time);
    if(enable_ps)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "PS: \n");
        len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", mn_sensor.ps.integration_time >> 2, mn_sensor.ps.gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "ADC = %d, cycle = %d, ir drive = %d\n", mn_sensor.ps.adc >> 3, mn_sensor.ps.cycle, mn_sensor.ps.ir_drive);
        len += snprintf(buf+len, PAGE_SIZE-len, "saturation = %d, int flag = %d\n", mn_sensor.ps.saturation >> 5, mn_sensor.ps.interrupt_flag >> 2);
        len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);
        len += snprintf(buf+len, PAGE_SIZE-len, "pals data = %d, data = %d\n", mn_sensor.ps.data.ir_data, mn_sensor.ps.data.data);
    }
    if(enable_als)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "ALS: \n");
        len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", mn_sensor.als.integration_time >> 2, mn_sensor.als.gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "ADC = %d, cycle = %d\n", mn_sensor.als.adc >> 3, mn_sensor.als.cycle);
#if ALS_DYN_INTT
        if(mn_sensor.als.report_type == CMC_BIT_DYN_INT)
        {
            len += snprintf(buf+len, PAGE_SIZE-len, "c_gain = %d\n", c_gain);
            len += snprintf(buf+len, PAGE_SIZE-len, "dyn_intt_raw=%d, dynamic_intt_lux=%d\n", mn_sensor.als.dyn_intt_raw, dynamic_intt_lux);
        }
#endif
    if(mn_sensor.als.polling_mode == 0)
        len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", mn_sensor.als.low_threshold, mn_sensor.als.high_threshold);
    len += snprintf(buf+len, PAGE_SIZE-len, "ch0 = %d, ch1 = %d\n", mn_sensor.als.data.channels[0], mn_sensor.als.data.channels[1]);
#if ALS_LSRC
        len += snprintf(buf+len, PAGE_SIZE-len, "offset_gain=%d, scale_gain=%d\n", offset_gain, scale_gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "lsrc_als_offset = %u\n", lsrc_als_offset);
        len += snprintf(buf+len, PAGE_SIZE-len, "lsrc_raw = %d\n", lsrc_raw);
        len += snprintf(buf+len, PAGE_SIZE-len, "lsrc_lux = %d \n", lsrc_lux);
        len += snprintf(buf+len, PAGE_SIZE-len, "lsrc_ratio = %d \n", lsrc_ratio);
        len += snprintf(buf+len, PAGE_SIZE-len, "diff = %d \n", (als_lux-lsrc_lux));
#endif

    }

    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t mn257x_store_als_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;

    APS_FUN();
    sscanf(buf, "%hu", &mode);
    APS_LOG("[%s]: als enable=%d \r\n", __func__, mode);
    mn257x_enable_als(mode);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t mn257x_store_ps_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    APS_FUN();

    sscanf(buf, "%hu", &mode);

    APS_LOG("[%s]: ps enable=%d \r\n", __func__, mode);
    mn257x_enable_ps(mode);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t mn257x_store_threshold(struct device_driver *ddri,const char *buf, size_t count)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    int low, high;

    APS_FUN();
    if(!obj)
    {
        APS_ERR("mn_sensor_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d,%d", &low, &high);

    switch(mn_sensor.mode)
    {
        case MN_MODE_PS:
            obj->hw->ps_threshold_low = low;
            obj->hw->ps_threshold_high = high;
            mn_sensor.ps.low_threshold = low;
            mn_sensor.ps.high_threshold = high;
            set_psensor_intr_threshold(mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);
            break;

        case MN_MODE_ALS:
            obj->hw->als_threshold_low = low;
            obj->hw->als_threshold_high = high;
            mn_sensor.als.low_threshold = low;
            mn_sensor.als.high_threshold = high;
            set_lsensor_intr_threshold(mn_sensor.als.low_threshold, mn_sensor.als.high_threshold);
            break;

    }
    return  count;
}
/*----------------------------------------------------------------------------*/
static ssize_t mn257x_show_threshold(struct device_driver *ddri, char *buf)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    ssize_t len = 0;

    if(!obj)
    {
        APS_ERR("mn_sensor_obj is null!!\n");
        return 0;
    }

    switch(mn_sensor.mode)
    {
        case MN_MODE_PS:
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->ps_threshold_low=%d \r\n", obj->hw->ps_threshold_low);
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->ps_threshold_high=%d \r\n", obj->hw->ps_threshold_high);
            break;

        case MN_MODE_ALS:
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->als_threshold_low=%d \r\n", obj->hw->als_threshold_low);
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->als_threshold_high=%d \r\n", obj->hw->als_threshold_high);
            break;

    }
    return  len;

}
/*----------------------------------------------------------------------------*/
static ssize_t mn257x_store_integration(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
#if ALS_DYN_INTT
    int value1=0;
#endif
    struct mn_sensor_priv *obj = mn_sensor_obj;

    APS_FUN();
    sscanf(buf, "%d",&value);

    switch (mn_sensor.mode)
    {
        case MN_MODE_PS:
            mn_sensor.ps.integration_time = (value & 0xf) << 2;
            if(mn_sensor.revno == 0x0288)
                mn257x_I2C_Write(obj->client, DEVREG_PS_CONFIG, mn_sensor.ps.ps_std | mn_sensor.ps.integration_time | mn_sensor.ps.gain);
            else
                mn257x_I2C_Write(obj->client, DEVREG_PS_CONFIG, mn_sensor.ps.ps_intb_nonlos | mn_sensor.ps.integration_time | mn_sensor.ps.gain);
            mn257x_I2C_Read(obj->client, DEVREG_PS_CONFIG, 1);
            APS_LOG("%s: 0x03 = 0x%x (0x%x)\n", __FUNCTION__, mn_sensor.ps.ps_intb_nonlos | mn_sensor.ps.integration_time | mn_sensor.ps.gain, gRawData.raw_bytes[0]);
            break;

        case MN_MODE_ALS: //als
#if ALS_DYN_INTT
            if(mn_sensor.als.report_type == CMC_BIT_DYN_INT)
            {
                sscanf(buf, "%d,%d",&value, &value1);
                if(value < 0 || value >= sizeof(als_intt_value)/sizeof(als_intt_value[0]) || value1 < 0 || value1 >= sizeof(als_intt_value)/sizeof(als_intt_value[0]))
                {
                    APS_ERR("value, value1 range error!\n");
                    return count;
                }
                als_dynamic_intt_intt[0] = (value & 0xf) << 2;
                als_dynamic_intt_value[0] = als_intt_value[value];

                als_dynamic_intt_intt[1] = (value1 & 0xf) << 2;
                als_dynamic_intt_value[1] = als_intt_value[value1];
                APS_LOG("[%s]: als_dynamic_intt_value=%d,%d \r\n", __func__, als_dynamic_intt_value[0], als_dynamic_intt_value[1]);

                dynamic_intt_idx = dynamic_intt_init_idx;
                mn_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                mn_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
            }
            else
#endif
            {
                mn_sensor.als.integration_time = (value & 0xf) << 2;
                if(mn_sensor.revno == 0x0288)
                    mn257x_I2C_Write(obj->client, DEVREG_ALS_CONFIG, mn_sensor.als.als_std | mn_sensor.als.integration_time | mn_sensor.als.gain);
                else
                    mn257x_I2C_Write(obj->client, DEVREG_ALS_CONFIG, mn_sensor.als.als_intb_nonlos | mn_sensor.als.integration_time | mn_sensor.als.gain);
                mn257x_I2C_Read(obj->client, DEVREG_ALS_CONFIG, 1);
                APS_LOG("%s: DEVREG_ALS_CONFIG = 0x%x (0x%x)\n", __FUNCTION__, mn_sensor.als.als_intb_nonlos | mn_sensor.als.integration_time | mn_sensor.als.gain, gRawData.raw_bytes[0]);
            }
            break;

    }
    mn257x_update_mode(obj->client);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t mn257x_store_gain(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mn_sensor_priv *obj = mn_sensor_obj;
	int value = 0;

    APS_FUN();
    sscanf(buf, "%d", &value);
    value = value & 0x03;

	switch (mn_sensor.mode)
	{
        case MN_MODE_PS:
            mn_sensor.ps.gain = value;
            if(mn_sensor.revno == 0x0288)
                mn257x_I2C_Write(obj->client, DEVREG_PS_CONFIG, mn_sensor.ps.ps_std | mn_sensor.ps.integration_time | mn_sensor.ps.gain);
            else
	            mn257x_I2C_Write(obj->client, DEVREG_PS_CONFIG, mn_sensor.ps.ps_intb_nonlos | mn_sensor.ps.integration_time | mn_sensor.ps.gain);
		break;

        case MN_MODE_ALS: //als
            mn_sensor.als.gain = value;
            if(mn_sensor.revno == 0x0288)
                mn257x_I2C_Write(obj->client, DEVREG_ALS_CONFIG, mn_sensor.als.als_std | mn_sensor.als.integration_time | mn_sensor.als.gain);
            else
	            mn257x_I2C_Write(obj->client, DEVREG_ALS_CONFIG, mn_sensor.als.als_intb_nonlos | mn_sensor.als.integration_time | mn_sensor.als.gain);
		break;

    }
	mn257x_update_mode(obj->client);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t mn257x_store_cycle(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct mn_sensor_priv *obj = mn_sensor_obj;

    APS_FUN();
    sscanf(buf, "%d",&value);

    switch (mn_sensor.mode)
    {
        case MN_MODE_PS:
            mn_sensor.ps.cycle = (value & 0x7);
            mn257x_I2C_Write(obj->client, DEVREG_PS_FILT, mn_sensor.ps.ps_rs | mn_sensor.ps.adc | mn_sensor.ps.cycle);
            mn257x_I2C_Read(obj->client, DEVREG_PS_FILT, 1);
            APS_LOG("%s: 0x04 = 0x%x (0x%x)\n", __FUNCTION__, mn_sensor.ps.ps_rs | mn_sensor.ps.adc | mn_sensor.ps.cycle, gRawData.raw_bytes[0]);
            break;

        case MN_MODE_ALS: //als
            mn_sensor.als.cycle = (value & 0x7);
            mn257x_I2C_Write(obj->client, DEVREG_ALS_FILT, mn_sensor.als.als_rs | mn_sensor.als.adc | mn_sensor.als.cycle);
            mn257x_I2C_Read(obj->client, DEVREG_ALS_FILT, 1);
            APS_LOG("%s: 0x02 = 0x%x (0x%x)\n", __FUNCTION__, mn_sensor.als.als_rs | mn_sensor.als.adc | mn_sensor.als.cycle, gRawData.raw_bytes[0]);
            break;
    }
    mn257x_update_mode(obj->client);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t mn257x_store_als_report_type(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;

    APS_FUN();
    sscanf(buf, "%d", &value);
    mn_sensor.als.report_type = value & 0xf;

    return count;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t mn257x_store_als_lux_per_count(struct device_driver *ddri, const char *buf, size_t count)
{
    int lux_per_count = 0;

    sscanf(buf, "%d",&lux_per_count);
    mn_sensor.als.factory.lux_per_count = lux_per_count;

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t mn257x_show_pdata(struct device_driver *ddri, char *buf)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    ssize_t len = 0;
    int ps_raw;

    mn257x_read_ps(obj->client);
    ps_raw = mn_sensor.ps.data.data;
    APS_LOG("[%s]: ps_raw=%d \r\n", __func__, ps_raw);

    len += snprintf(buf + len, PAGE_SIZE - len, "%d \n", ps_raw);

    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t mn257x_show_als_data(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    u16 als_raw = 0, lux;

    als_raw = factory_als_data();
    lux = als_raw * c_gain / 1000;
    APS_LOG("[%s]: als_raw=%d als_lux=%d\r\n", __func__, als_raw, lux);
    len += snprintf(buf + len, PAGE_SIZE - len, "als_raw=%d , als_lux=%d \n", als_raw, lux);

    return len;
}
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
#if ALS_DYN_INTT
static ssize_t mn257x_store_c_gain(struct device_driver *ddri, const char *buf, size_t count)
{
    int als_c_gain;

    APS_FUN();
    sscanf(buf, "%d",&als_c_gain);
    c_gain = als_c_gain;
    APS_LOG("c_gain = %d \r\n", c_gain);

	return count;
}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t mn257x_store_reg_write(struct device_driver *ddri, const char *buf, size_t count)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    int reg;
    int data;

    APS_FUN();
    sscanf(buf, "%x,%x",&reg, &data);
    APS_LOG("[%s]: reg=0x%x, data=0x%x", __func__, reg, data);

    if(reg == 0x00 && ((data & 0x0f) == MN_MODE_PS || (data & 0x0f) == MN_MODE_ALS_PS))
    {
        set_bit(CMC_BIT_PS, &obj->enable);
    }
    else
    {
        clear_bit(CMC_BIT_PS, &obj->enable);
    }
    mn257x_I2C_Write(obj->client, reg, data);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t mn257x_show_renvo(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    APS_FUN();
    APS_LOG("gRawData.renvo=0x%x \r\n", mn_sensor.revno);
    len += snprintf(buf+len, PAGE_SIZE-len, "%x \n", mn_sensor.revno);

    return len;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#if ALSPS_DBG
static ssize_t mn257x_store_debug_flag(struct device_driver *ddri, const char *buf, size_t count)
{
    int dub_falg=0;

    APS_FUN();
    sscanf(buf, "%d",&dub_falg);
    debug_flag = dub_falg;

    return count;
}
#endif
#if ALS_LSRC
static ssize_t mn257x_store_lsrc_offset(struct device_driver *ddri, const char *buf, size_t count)
{
    int offset=0;

    APS_FUN();
    sscanf(buf, "%d",&offset);
    offset_gain = offset;

    return count;
}

static ssize_t mn257x_store_lsrc_scale(struct device_driver *ddri, const char *buf, size_t count)
{
    int scale=0;

    APS_FUN();
    sscanf(buf, "%d",&scale);
    scale_gain = scale;

    return count;
}

static ssize_t mn257x_show_lsrc_offset( struct device_driver *dev, char *buf )
{
	APS_FUN();
	return sprintf( buf, "offset_gain=%d\n", offset_gain);
}
static ssize_t mn257x_show_lsrc_scale( struct device_driver *dev, char *buf )
{
	APS_FUN();
	return sprintf( buf, "scale_gain=%d\n", scale_gain);
}
#endif


#if MN_SELF_TEST
static ssize_t mn257x_show_ps_high_thd(struct device_driver *ddri, char *buf)
{
    int ps_hthd;
    ps_hthd = (int)mn_sensor.ps.high_threshold;
    return sprintf( buf, "%d\n", ps_hthd);
}
static ssize_t mn257x_store_ps_high_thd( struct device_driver *dev, const char *buf, size_t count )
{
    int ps_hthd=0;

    sscanf(buf, "%d", &ps_hthd);
    mn_sensor.ps.high_threshold = (u16)ps_hthd;
    set_psensor_intr_threshold(mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);
    return count;
}

static ssize_t mn257x_show_ps_low_thd(struct device_driver *ddri, char *buf)
{
    int ps_lthd;
    ps_lthd = (int)mn_sensor.ps.low_threshold;
    return sprintf( buf, "%d\n", ps_lthd);
}
static ssize_t mn257x_store_ps_low_thd( struct device_driver *dev, const char *buf, size_t count )
{
    int ps_lthd=0;

    sscanf(buf, "%d", &ps_lthd);
    mn_sensor.ps.low_threshold = (u16)ps_lthd;
    set_psensor_intr_threshold(mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);
    return count;
}

static ssize_t mn257x_show_ps_offset(struct device_driver *ddri, char *buf)
{
    return sprintf( buf, "%d\n", ps_factory_offset);
}
static ssize_t mn257x_store_ps_offset( struct device_driver *dev, const char *buf, size_t count )
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    int offset=0;

    sscanf(buf, "%d", &offset);
    ps_factory_offset = offset;

    mn_sensor.ps.high_threshold = obj->ps_crosstalk + ps_factory_offset;
    mn_sensor.ps.low_threshold = mn_sensor.ps.high_threshold - ps_factory_hystersis;

    set_psensor_intr_threshold(mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);

    return count;
}

static ssize_t mn257x_show_ps_hystersis(struct device_driver *ddri, char *buf)
{
    int hystersis;
    hystersis = (int)ps_factory_hystersis;
    return sprintf( buf, "%d\n", hystersis);
}
static ssize_t mn257x_store_ps_hystersis( struct device_driver *dev, const char *buf, size_t count )
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    int hystersis=0;

    sscanf(buf, "%d", &hystersis);
    ps_factory_hystersis = (u16)hystersis;

    mn_sensor.ps.high_threshold = obj->ps_crosstalk + ps_factory_offset;
    mn_sensor.ps.low_threshold = mn_sensor.ps.high_threshold - ps_factory_hystersis;

    set_psensor_intr_threshold(mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);

    return count;
}

static ssize_t mn257x_show_cali_value(struct device_driver *ddri, char *buf)
{

	struct mn_sensor_priv *obj = mn_sensor_obj;
    int ret = 0;
    APS_FUN();
    ret = obj->cali_status;

	return sprintf( buf, "%d\n", ret);
}
static ssize_t mn257x_store_cali_value( struct device_driver *dev, const char *buf, size_t count )
{

    struct mn_sensor_priv *obj = mn_sensor_obj;
    int calib_times=0;

	APS_FUN();

    sscanf(buf, "%d", &calib_times);
	APS_LOG("mn257x_store_cali_value calib_times = %d \r\n",calib_times);

	if (calib_times < 10) calib_times = 10;

    obj->cali_status = CALI_FAIL;

    mn257x_do_ps_calibration(obj, calib_times, true);

	return count;
}

static ssize_t mn257x_show_ps_crosstalk ( struct device_driver *dev, char *buf )
{
    struct mn_sensor_priv *obj = mn_sensor_obj;

	APS_FUN();
	return sprintf( buf, "%d\n", obj->ps_crosstalk);
}

static ssize_t mn257x_store_ps_crosstalk ( struct device_driver *dev, const char *buf, size_t count )
{

    struct mn_sensor_priv *obj = mn_sensor_obj;
	int val;
    sscanf(buf, "%d",&val);
	obj->ps_crosstalk = val;

	mn_sensor.ps.high_threshold = obj->ps_crosstalk + ps_factory_offset;
    mn_sensor.ps.low_threshold = mn_sensor.ps.high_threshold - ps_factory_hystersis;

    set_psensor_intr_threshold(mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);
	return count;
}

#endif /*MN_SELF_TEST*/

static ssize_t mn257x_show_als_enable ( struct device_driver *dev, char *buf )
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable);

	APS_FUN();
	return sprintf( buf, "Enable_als=%d\n", enable_als);
}

static ssize_t mn257x_show_ps_enable ( struct device_driver *dev, char *buf )
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable);

	APS_FUN();
	return sprintf( buf, "Enable_ps=%d\n", enable_ps);
}

static ssize_t mn257x_show_als_report_type ( struct device_driver *dev, char *buf )
{
	APS_FUN();
	return sprintf( buf, "als_report_type=%d\n", mn_sensor.als.report_type);
}

static ssize_t mn257x_show_lpc ( struct device_driver *dev, char *buf )
{
	APS_FUN();
	return sprintf( buf, "als_lux-per-count=%d\n", c_gain);
}

static ssize_t mn257x_show_integ ( struct device_driver *dev, char *buf )
{
    int len =0;
	APS_FUN();

    len += snprintf(buf + len, PAGE_SIZE - len, "als_integ=%d, ps_integ=%d\n",
                    als_intt_value[(mn_sensor.als.integration_time >> 2)], ps_intt_value[(mn_sensor.ps.integration_time >> 2)]);

    return len;
}

static ssize_t mn257x_show_gain ( struct device_driver *dev, char *buf )
{
    int len =0;
	APS_FUN();

    len += snprintf(buf + len, PAGE_SIZE - len, "als_gain=%d, ps_gain=%d\n", mn_sensor.als.gain, mn_sensor.ps.gain);

    return len;
}

static ssize_t mn257x_show_cycle( struct device_driver *dev, char *buf )
{

    int len =0;
	APS_FUN();

    len += snprintf(buf + len, PAGE_SIZE - len, "als_cycle=%d, ps_cycle=%d\n",
                    cycle_value[mn_sensor.als.cycle], cycle_value[mn_sensor.ps.cycle] );

    return len;
}

static ssize_t mn257x_show_c_gain( struct device_driver *dev, char *buf )
{

	APS_FUN();
	return sprintf( buf, "c_gain=%d\n", c_gain);
}

static ssize_t mn257x_show_debug_flag( struct device_driver *dev, char *buf )
{
	APS_FUN();
	return sprintf( buf, "debug_flag=%d\n", debug_flag);
}
/*CTS --> S_IWUSR | S_IRUGO*/
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(elan_status,					 S_IWUSR | S_IRUGO, mn257x_show_status,  	  		NULL										);
static DRIVER_ATTR(elan_reg,    				 S_IWUSR | S_IRUGO, mn257x_show_reg,   				NULL										);
static DRIVER_ATTR(als_enable,					 S_IWUSR | S_IRUGO, mn257x_show_als_enable,   							mn257x_store_als_enable					);
static DRIVER_ATTR(als_report_type,				 S_IWUSR | S_IRUGO, mn257x_show_als_report_type,								mn257x_store_als_report_type			);
static DRIVER_ATTR(als_lux_per_count,			 S_IWUSR | S_IRUGO, mn257x_show_lpc,   					 		mn257x_store_als_lux_per_count			);
static DRIVER_ATTR(ps_enable,					 S_IWUSR | S_IRUGO, mn257x_show_ps_enable,   							mn257x_store_ps_enable					);

static DRIVER_ATTR(set_threshold,     			 S_IWUSR | S_IRUGO, mn257x_show_threshold,          mn257x_store_threshold			        );
static DRIVER_ATTR(integration,					 S_IWUSR | S_IRUGO, mn257x_show_integ,								mn257x_store_integration				);
static DRIVER_ATTR(pdata,                       S_IWUSR | S_IRUGO, mn257x_show_pdata,              NULL                                        );
static DRIVER_ATTR(als_data,                    S_IWUSR | S_IRUGO, mn257x_show_als_data,           NULL                                        );
static DRIVER_ATTR(i2c_w,                       S_IWUSR | S_IRUGO, NULL,                               mn257x_store_reg_write                  );
static DRIVER_ATTR(elan_renvo,                  S_IWUSR | S_IRUGO, mn257x_show_renvo,              NULL                                        );
static DRIVER_ATTR(gain,					    S_IWUSR | S_IRUGO, mn257x_show_gain,								mn257x_store_gain					    );
static DRIVER_ATTR(cycle,						S_IWUSR | S_IRUGO, mn257x_show_cycle,								mn257x_store_cycle						);
#if ALS_DYN_INTT
static DRIVER_ATTR(als_dyn_c_gain,              S_IWUSR | S_IRUGO, mn257x_show_c_gain,                               mn257x_store_c_gain                     );
#endif
#if ALSPS_DBG
static DRIVER_ATTR(dbg_flag,			        S_IWUSR | S_IRUGO, mn257x_show_debug_flag,                               mn257x_store_debug_flag                  );
#endif
#if ALS_LSRC
static DEVICE_ATTR(als_lsrc_offset,			    S_IWUSR | S_IRUGO, mn257x_show_lsrc_offset,                               mn257x_store_lsrc_offset                );
static DEVICE_ATTR(als_lsrc_scale,			    S_IWUSR | S_IRUGO, mn257x_show_lsrc_scale,                               mn257x_store_lsrc_scale                 );
#endif
#if MN_SELF_TEST
static DRIVER_ATTR (near_offset,                S_IWUSR | S_IRUGO, mn257x_show_ps_high_thd,          mn257x_store_ps_high_thd                 );
static DRIVER_ATTR (far_offset,                 S_IWUSR | S_IRUGO, mn257x_show_ps_low_thd,           mn257x_store_ps_low_thd                 );
static DRIVER_ATTR (target_pdata,                S_IWUSR | S_IRUGO, mn257x_show_ps_offset,            mn257x_store_ps_offset                 );
static DRIVER_ATTR (near_to_far,               S_IWUSR | S_IRUGO, mn257x_show_ps_hystersis,            mn257x_store_ps_hystersis                 );
static DRIVER_ATTR (cali,                       S_IWUSR | S_IRUGO, mn257x_show_cali_value,           mn257x_store_cali_value                 );
static DRIVER_ATTR (ps_crosstalk,               S_IWUSR | S_IRUGO, mn257x_show_ps_crosstalk,         mn257x_store_ps_crosstalk               );
#endif
/*----------------------------------------------------------------------------*/
static struct driver_attribute * mn257x_attr_list[] =
{
    &driver_attr_elan_status,
    &driver_attr_elan_reg,
    &driver_attr_als_enable,
    &driver_attr_als_report_type,
    &driver_attr_als_lux_per_count,
    &driver_attr_ps_enable,
    &driver_attr_elan_renvo,
    &driver_attr_i2c_w,
    &driver_attr_set_threshold,
    &driver_attr_integration,
    &driver_attr_pdata,
    &driver_attr_als_data,
    &driver_attr_gain,
    &driver_attr_cycle,
#if ALS_DYN_INTT
    &driver_attr_als_dyn_c_gain,
#endif
#if ALSPS_DBG
    &driver_attr_dbg_flag,
#endif
#if ALS_LSRC
    &driver_als_lsrc_offset_offset,
    &driver_attr_als_lsrc_scale,
#endif
#if MN_SELF_TEST
    &driver_attr_near_offset,
    &driver_attr_far_offset,
    &driver_attr_target_pdata,
    &driver_attr_near_to_far,
    &driver_attr_cali,
    &driver_attr_ps_crosstalk,
#endif
};

/*----------------------------------------------------------------------------*/
static int mn257x_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(mn257x_attr_list)/sizeof(mn257x_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }
    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, mn257x_attr_list[idx])))
        {
            APS_ERR("driver_create_file (%s) = %d\n", mn257x_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}



/*----------------------------------------------------------------------------*/
static int mn257x_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(mn257x_attr_list)/sizeof(mn257x_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, mn257x_attr_list[idx]);
    }

    return err;
}

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int mn257x_open(struct inode *inode, struct file *file)
{
    file->private_data = mn257x_i2c_client;

    APS_FUN();

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int mn257x_release(struct inode *inode, struct file *file)
{
    APS_FUN();
    file->private_data = NULL;
    return 0;
}

/*----------------------------------------------------------------------------*/
static long mn257x_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct mn_sensor_priv *obj = i2c_get_clientdata(client);
    int err=0, als_data=0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
#if MN_SELF_TEST
    u32 ps_cali;
#endif
    APS_LOG("%s cmd = 0x%04x", __FUNCTION__, cmd);
    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }

            APS_LOG("[%s]: ps enable=%d \r\n", __func__, enable);
            mn257x_enable_ps(enable);
        break;

        case ALSPS_GET_PS_MODE:
            enable=test_bit(CMC_BIT_PS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
        break;

        case ALSPS_GET_PS_DATA:

            factory_ps_data();
            dat = mn_sensor.ps.compare_low >> 3;

            APS_LOG("ioctl ps state value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
        break;

        case ALSPS_GET_PS_RAW_DATA:

            dat = factory_ps_data();

            APS_LOG("ioctl ps raw value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_SET_ALS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            APS_LOG("[%s]: als enable=%d \r\n", __func__, enable);
            mn257x_enable_als(enable);
        break;

        case ALSPS_GET_ALS_MODE:
            enable=test_bit(CMC_BIT_ALS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
        break;

        case ALSPS_GET_ALS_DATA:
            als_data = factory_als_data();
            dat = mn257x_get_als_value(obj, als_data);

            APS_LOG("ioctl get als data = %d\n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
        break;

        case ALSPS_GET_ALS_RAW_DATA:
            dat = factory_als_data();
            APS_LOG("ioctl get als raw data = %d\n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
        break;
        case ALSPS_IOCTL_SET_CALI:
#if MN_SELF_TEST
#if defined(CALIBRATION_TO_FILE)
            ps_cali = 0;
            err = sensor_calibration_read(ID_PROXIMITY, &ps_cali);
            if(err!=0){
                APS_ERR("Read Cal Fail from file !!!\n");
#if DTS_PARSE
				mn_sensor.ps.high_threshold = ps_default_h;
				mn_sensor.ps.low_threshold = ps_default_l;
				set_psensor_intr_threshold(mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);

				break;
#endif
            }
#endif
#else
            ps_cali = obj->ps_crosstalk;
            if (copy_from_user(&obj->ps_crosstalk, ptr, sizeof(ps_cali))) {
                err = -EFAULT;
                goto err_out;
            }
#endif
            if( obj->ps_crosstalk  > ps_self_test_max_ct)
            {
                obj->ps_crosstalk = ps_factory_ct;
            }
            else
            {
                obj->ps_crosstalk = ps_cali;
            }

            mn_sensor.ps.high_threshold = obj->ps_crosstalk + ps_factory_offset;
            mn_sensor.ps.low_threshold = mn_sensor.ps.high_threshold - ps_factory_hystersis;

            set_psensor_intr_threshold(mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);

            break;

            default:
            APS_ERR("%s not supported = 0x%04x", __func__, cmd);
            err = -ENOIOCTLCMD;
        break;
    }

err_out:
    return err;
}
/*----------------------------------------------------------------------------*/
static struct file_operations mn257x_fops =
{
    .owner = THIS_MODULE,
    .open = mn257x_open,
    .release = mn257x_release,
    .unlocked_ioctl = mn257x_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice mn257x_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &mn257x_fops,
};
/*----------------------------------------------------------------------------*/
static int mn257x_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct mn_sensor_priv *obj = i2c_get_clientdata(client);


    APS_FUN();
    if(!obj)
    {
        APS_ERR("[%s]: null pointer!!\n", __func__);
        return -EINVAL;
    }

    return 0;

}
/*----------------------------------------------------------------------------*/
static int mn257x_i2c_resume(struct i2c_client *client)
{
    struct mn_sensor_priv *obj = i2c_get_clientdata(client);

    APS_FUN();
    if(!obj)
    {
        APS_ERR("[%s]: null pointer!!\n", __func__);
        return -EINVAL;
    }

#if 0 //if suspend, vdd is off. After resume, vdd is on. then write all setting to chip.
    wake_lock(&ps_lock);
    write_global_variable(obj->client);
    mn257x_I2C_Write(client, DEVREG_RESET, MN_POWER_ON | MN_RESETN_RUN);
    wake_unlock(&ps_lock);
#endif

    return 0;
}

/*----------------------------------------------------------------------------*/

#if defined(CONFIG_FB)
static int light_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    struct fb_event *evdata = (struct fb_event *)data;
    int *blank = NULL;

    if (evdata && evdata->data)
        blank = (int *)evdata->data;
    else
        return 0;

    if (event == FB_EVENT_BLANK) {
        if (*blank == FB_BLANK_POWERDOWN) {
            atomic_set(&driver_suspend_flag, 1);
            APS_ERR("[IN] LCD Sleep\n");
        } else if (*blank == FB_BLANK_UNBLANK) {
            atomic_set(&driver_suspend_flag, 0);
            APS_ERR("[OUT] LCD Sleep\n");
            if(test_bit(CMC_BIT_ALS, &obj->enable)){
                set_lsensor_intr_threshold(65535, 0);
            }
        }
    }
    return 0;
}
#endif

/*--------------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;

	if(!obj)
	{
		APS_ERR("obj is null!!\n");
		return -1;
	}

	APS_LOG("[%s]: als enable=%d \r\n", __func__, en);
    mn257x_enable_als(en);

	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_get_data(int* value, int* status)
{
	int err = 0;
	u16 report_lux = 0;
	struct mn_sensor_priv *obj = mn_sensor_obj;
	if(!obj)
	{
		APS_ERR("obj is null!!\n");
		return -1;
	}

    mn257x_read_als(obj->client);

    report_lux = mn257x_get_als_value(obj, mn_sensor.als.data.channels[1]);

    *value = report_lux;
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    if(!(lcount & 0x0000003F))
        APS_LOG("[%s]:*value = %d\n", __func__, *value);

	return err;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
    struct mn_sensor_priv *obj = mn_sensor_obj;
    if(!obj)
	{
		APS_ERR("obj is null!!\n");
		return -1;
	}

    APS_LOG("[%s]: ps enable=%d \r\n", __func__, en);
    mn257x_enable_ps(en);

	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_get_data(int* value, int* status)
{

    int err = 0;
    struct mn_sensor_priv *obj = mn_sensor_obj;

    APS_LOG("---SENSOR_GET_DATA---\n\n");
    mn257x_read_ps(obj->client);

    *value = mn_sensor.ps.compare_low >> 3;
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

    APS_LOG("[%s]:*value = %d\n", __func__, *value);

	return err;
}
/*----------------------------------------------------------------------------*/
static int mn257x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct mn_sensor_priv *obj;
    int renvo = 0;
    struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
    int err = 0;

    APS_FUN();
    mn257x_dumpReg(client);

    APS_LOG("probe");
    renvo = i2c_smbus_read_byte_data(client, DEVREG_CHIP_ID);
	if(renvo != 0x81 && renvo != 0x91 && renvo != 0xa1 && renvo != 0x02){
        APS_LOG("elan ALS/PS sensor is failed(0x%x). \n", renvo);
        err = -1;
        goto exit;
    }

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }

    memset(obj, 0, sizeof(*obj));

    mn_sensor_obj = obj;

    obj->hw = hw;

    mn257x_get_addr(obj->hw, &obj->addr);

    obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
    obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
    memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
    memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));

    INIT_DELAYED_WORK(&obj->eint_work, mn257x_eint_work);

    obj->client = client;

    mutex_init(&sensor_mutex);
    //wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");

    i2c_set_clientdata(client, obj);

    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->ps_suspend, 0);

    obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");
    obj->enable = 0;

    mn257x_i2c_client = client;

    //initial global variable and write to senosr
    initial_global_variable(client, obj);

    if((err = mn257x_init_client(client)))
    {
        goto exit_init_failed;
    }

    if((err = misc_register(&mn257x_device)))
    {
        APS_ERR("mn257x_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    if((err = mn257x_create_attr(&mn257x_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
    als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = true;
#else
    als_ctl.is_support_batch = false;
#endif
	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = mn_sensor.ps.polling_mode==0? true:false;
	ps_ctl.is_polling_mode = mn_sensor.ps.polling_mode==1? true:false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = true;
#else
    ps_ctl.is_support_batch = false;
#endif
	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_create_attr_failed;
	}
#if defined(CONFIG_FB)
    obj->fb_notif.notifier_call = light_fb_notifier_callback;
    fb_register_client(&obj->fb_notif);
#endif

    if(obj->hw->polling_mode_ps == 0 || obj->hw->polling_mode_als == 0)
        mn257x_setup_eint(client);

    alsps_init_flag = 0;

    APS_LOG("%s: OK\n", __FUNCTION__);
    return 0;

exit_create_attr_failed:
    misc_deregister(&mn257x_device);
exit_misc_device_register_failed:
exit_init_failed:

    kfree(obj);
exit:
    mn257x_i2c_client = NULL;
    alsps_init_flag = -1;
    APS_ERR("%s: err = %d\n", __FUNCTION__, err);

    return err;
}



/*----------------------------------------------------------------------------*/
static int mn257x_i2c_remove(struct i2c_client *client)
{
    int err;
	struct mn_sensor_priv *obj = mn_sensor_obj;

	if((err = mn257x_delete_attr(&mn257x_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("mn257x_delete_attr fail: %d\n", err);
    }

    if((err = misc_deregister(&mn257x_device)))
    {
        APS_ERR("misc_deregister fail: %d\n", err);
    }

#if defined(CONFIG_FB)
    fb_unregister_client(&obj->fb_notif);
#endif

    mn257x_i2c_client = NULL;
    kfree(i2c_get_clientdata(client));

    return 0;
}


/*----------------------------------------------------------------------------*/
static int alsps_local_init(void)
{
	int err = 0;

	APS_FUN();
	
	if (NULL == hw){
		APS_ERR(" Parameter error \n");
		return EINVAL;
	}

	mn257x_power(hw, 1);
    err = i2c_add_driver(&mn257x_i2c_driver);
	if(err < 0)
	{
		APS_ERR("add driver error\n");
		return -1;
	}

	if(alsps_init_flag < 0)
	{
	   return -1;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int alsps_remove(void)
{
    APS_FUN();
    mn257x_power(hw, 0);
    APS_ERR("mn_sensor remove \n");

    i2c_del_driver(&mn257x_i2c_driver);

    return 0;
}

/*----------------------------------------------------------------------------*/
static int __init mn257x_init(void)
{
    const char *name = COMPATIABLE_NAME;	// "mediatek,mn257x"
	int ret;
    APS_LOG("epl_sensor_init start +++\n");

    hw = get_alsps_dts_func(name, hw);

    if (!hw)
    {
    	APS_ERR("get dts info fail\n");
		return -1;
    }
#if DTS_PARSE
	ret = mn257x_parse_dt(COMPATIABLE_NAME);
#endif
    alsps_driver_add(&mn257x_init_info);


	APS_LOG("epl_sensor_init end---\n");

    return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit mn257x_exit(void)
{
    APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(mn257x_init);
module_exit(mn257x_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("flank.chen@eminent-tek.com");
MODULE_DESCRIPTION("MN257x ALPsr driver");
MODULE_LICENSE("GPL");

