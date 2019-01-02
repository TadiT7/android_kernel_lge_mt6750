/******************************************************************************
 * MODULE       : rohm_rpr0521_i2c.c
 * FUNCTION     : Driver source for RPR0521,RPR0531, Ambient Light and Proximity Sensor IC
 * AUTHOR       : Aaron Liu
 * MODIFICATION : Modified by Aaron-liu, Sep/5/2016
 * NOTICE       : This software had been verified using MT6795/MT6797.
 *              : When you use this code and document, Please verify all
 *              : operation in your operating system.
 * REMARKS      :
 * COPYRIGHT    : Copyright (C) 2015-2016 - ROHM CO.,LTD.
 *              : This program is free software; you can redistribute it and/or
 *              : modify it under the terms of the GNU General Public License
 *              : as published by the Free Software Foundation; either version 2
 *              : of the License, or (at your option) any later version.
 *              :
 *              : This program is distributed in the hope that it will be useful,
 *              : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *              : GNU General Public License for more details.
 *              :
 *              : You should have received a copy of the GNU General Public License
 *              : along with this program; if not, write to the Free Software
 *              : Foundation, Inc., 51 Franklin Street, Fifth Floor,Boston, MA  02110-1301, USA.
 *****************************************************************************/
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/version.h>

#include "cust_alsps.h"
#include "alsps.h"
#include "rohm_rpr0521_i2c.h"

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0))
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <hwmsensor.h>
#include <mach/eint.h>
#else
#include <linux/gpio.h>
#endif


#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0))
static struct alsps_hw *hw ;
#else
/* Maintain alsps cust info here */
static struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
#endif


#define CALIBRATION_TO_FILE 1
#if defined(CALIBRATION_TO_FILE)
#define CAL_TO_PERSIST
#include "../../sensor_cal/sensor_cal_file_io.h"
#endif

#ifdef ROHM_CALIBRATE
//This macro is used to re-initialize IC when the IC is reset by some unknown reasons(such as ESD)
#ifdef IC_RESET_NEED_INITIALIZE
static bool cali_continue = true;  //do calibration if true
#endif
#endif

#define POCKET_DETECTION 1
#if POCKET_DETECTION
#include <linux/atomic.h>
#define ALS_POCKET_THRES 0x7C
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
static atomic_t driver_suspend_flag = ATOMIC_INIT(0);
struct notifier_block fb_notif;
static int light_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif
#endif

/* *************************************************/
#define CHECK_RESULT(result)                        \
    if ( result < 0 )                               \
{                                               \
    RPR0521_ERR("Error occur !!!.\n");          \
    return result;                              \
}
/* *************************************************/


/*========================================================*/
#define COEFFICIENT               (4)
static unsigned long data0_coefficient[COEFFICIENT] = {2156, 1601, 1140, 694};  //grace modify in 2014.5.9
static unsigned long data1_coefficient[COEFFICIENT] = {0, 891,  476,  247};
static unsigned long judge_coefficient[COEFFICIENT] = {177,  1107,  1955, 2810};


/* gain table */
#define GAIN_FACTOR (16)
static const struct GAIN_TABLE {
    char data0;
    char data1;
} gain_table[GAIN_FACTOR] = {
    {  1,   1},   /*  0 */
    {  1,   2},   /*  1 */
    {  1,  64},   /*  2 */
    {  1, 128},   /*  3 */
    {  2,   1},   /*  4 */
    {  2,   2},   /*  5 */
    {  2,  64},   /*  6 */
    {  2, 128},   /*  7 */
    { 64,   1},   /*  8 */
    { 64,   2},   /*  9 */
    { 64,  64},   /* 10 */
    { 64, 128},   /* 11 */ //grace modify in 2014.4.11
    {128,   1},   /* 12 */
    {128,   2},   /* 13 */
    {128,  64},   /* 14 */
    {128, 128}    /* 15 */
};



/* logical functions */

static int    rpr0521_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int    rpr0521_remove(struct i2c_client *client);
static void   rpr0521_power(struct alsps_hw *hw, unsigned int on);
static int    rpr0521_als_open_report_data(int open);
static int    rpr0521_als_enable_nodata(int en);
static int    rpr0521_als_set_delay(u64 delay);
static int    rpr0521_als_get_data(int *als_value, int *status);
static int    rpr0521_local_init(void);
static int    rpr0521_local_remove(void);
static int    rpr0521_ps_open_report_data(int open);
static int    rpr0521_ps_enable_nodata(int en);
static int    rpr0521_ps_set_delay(u64 delay);
static int    rpr0521_ps_get_data(int *als_value, int *status);
static int    rpr0521_driver_init(struct i2c_client *client);
static int    rpr0521_driver_shutdown(struct i2c_client *client);
static int    rpr0521_driver_reset(struct i2c_client *client);
static int    rpr0521_driver_als_power_on_off(struct i2c_client *client, unsigned char data);
static int    rpr0521_driver_ps_power_on_off(struct i2c_client *client, unsigned char data);
static int    rpr0521_driver_read_data(struct i2c_client *client, READ_DATA_ARG *data);
static void   rpr0521_set_prx_thresh( unsigned short      pilt, unsigned short piht );
static void   rpr0521_ps_get_real_value(void);
unsigned int  rpr0521_als_convert_to_mlux( unsigned short  data0, unsigned short  data1, unsigned short  gain_index, unsigned short  time );
static int    rpr0521_als_open_report_data(int open);
static int    rpr0521_als_enable_nodata(int en);
static int    rpr0521_ps_open_report_data(int open);
static int    rpr0521_ps_enable_nodata(int en);

static int rpr0521_suspend(struct device *dev);
static int rpr0521_resume(struct device *dev);
#if ALS_USE_INTERRUPT_MODE
static void rpr0521_set_als_thresh(unsigned short adata0 );
static void rpr0521_set_als_thr_reg(unsigned short pilt, unsigned short piht );
#endif
static int  __init           rpr0521_init(void);
static void __exit          rpr0521_exit(void);

//#if defined(RPR0521_PS_CALIBRATIOIN_ON_CALL) || defined(RPR0521_PS_CALIBRATIOIN_ON_START)
static int                  rpr0521_ps_calibration( RPR0521_DATA * obj);
//#endif

/**************************** variable declaration ****************************/
static RPR0521_DATA                 *obj = NULL;
static unsigned long long       int_top_time = 0;
static const char               rpr0521_driver_ver[] = RPR0521_DRIVER_VER;

/**************************** structure declaration ****************************/
/* I2C device IDs supported by this driver */
static const struct i2c_device_id rpr0521_id[] = {
    { RPR0521_I2C_NAME, 0 }, /* rohm bh1745 driver */
    { }
};

static const struct dev_pm_ops rpr0521_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(rpr0521_suspend, rpr0521_resume)
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
    {.compatible = "mediatek,rpr0521"},  /* this string should be the same with dts */
    {},
};
#endif
/* represent an I2C device driver */
static struct i2c_driver rpr0521_driver = {
    .driver = {                     /* device driver model driver */
        .owner = THIS_MODULE,
        .name  = RPR0521_I2C_DRIVER_NAME,
#ifdef CONFIG_OF
        .of_match_table = alsps_of_match,
#endif
        .pm = &rpr0521_pm_ops,
    },
    .probe    = rpr0521_probe,          /* callback for device binding */
    .remove   = rpr0521_remove,         /* callback for device unbinding */
    .shutdown = NULL,
    .suspend  = NULL,
    .resume   = NULL,
    .id_table = rpr0521_id,             /* list of I2C devices supported by this driver */
};

static int rpr0521_init_flag = -1; /* 0<==>OK -1 <==> fail*/
/* MediaTek alsps information */
static struct alsps_init_info rpr0521_init_info = {
    .name = RPR0521_I2C_NAME,        /* Alsps driver name */
    .init = rpr0521_local_init,      /* Initialize alsps driver */
    .uninit = rpr0521_local_remove,        /* Uninitialize alsps driver */
};

#if POCKET_DETECTION
#if defined(CONFIG_FB)
static int light_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = (struct fb_event *)data;
    int *blank = NULL;

    if (evdata && evdata->data)
        blank = (int *)evdata->data;
    else
        return 0;

    if (event == FB_EVENT_BLANK) {
        if (*blank == FB_BLANK_POWERDOWN) {
            atomic_set(&driver_suspend_flag, 1);
            RPR0521_ERR("[IN] LCD Sleep\n");
        } else if (*blank == FB_BLANK_UNBLANK) {
            atomic_set(&driver_suspend_flag, 0);
            RPR0521_ERR("[OUT] LCD Sleep\n");
            if(obj->als_en_status == true){
				rpr0521_set_als_thr_reg(0xffff, 0);
           }
        }
    }
    return 0;
}
#endif
#endif


/**
 * @Brief: rpr0521_register_dump print register value for debug
 *
 * @Param: reg_address regsiter address
 *
 * @return: return register's value
 */
static int rpr0521_register_dump(int addr)
{
    int  read_data = 0;

    if (NULL == obj->client){
        RPR0521_ERR(" Parameter error \n");
        return -EINVAL;;
    }

    if( addr >  RPR0521_REG_AILTH_ADDR || addr < RPR0521_REG_ID_ADDR){
        RPR0521_WARNING( "reg =0x%x  is out of range!!!", addr);
        return -EINVAL;
    }

    /*read */
    read_data = i2c_smbus_read_byte_data(obj->client, addr);

    if (read_data < 0) {
        RPR0521_ERR( "ps_rpr0521_driver_general_read : transfer error \n");
        return -EINVAL;

    }

    RPR0521_WARNING( "reg(0x%x) = 0x%x \n", addr, read_data);

    return read_data;
}

/************************************************************/
static int rpr0521_open(struct inode *inode, struct file *file)
{
    file->private_data = obj->client;

    if (!file->private_data) {
        RPR0521_ERR("null pointer!!\n");
        return -EINVAL;
    }
    return nonseekable_open(inode, file);
}

/************************************************************/
static int rpr0521_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}


static long rpr0521_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client *)file->private_data;
    RPR0521_DATA *obj_tmp = i2c_get_clientdata(client);
    int err = 0, dat, enable;
    void __user *ptr = (void __user *)arg;


    switch (cmd) {
        READ_DATA_ARG data;

        case ALSPS_SET_PS_MODE:
        if (copy_from_user(&enable, ptr, sizeof(enable))) {
            err = -EFAULT;
            return err;
        }
        rpr0521_ps_open_report_data(enable);
        rpr0521_ps_enable_nodata(enable);
        break;

        case ALSPS_GET_PS_MODE:
        enable = obj_tmp->ps_en_status ? (1) : (0);
        if (copy_to_user(ptr, &enable, sizeof(enable))) {
            err = -EFAULT;
            return err;
        }
        break;

        case ALSPS_GET_PS_DATA:

        //read adata0 adata1 and pdata
        err = rpr0521_driver_read_data(obj_tmp->client, &data);   //get ps raw data
        CHECK_RESULT(err);

        RPR0521_WARNING("pdata = 0x%x, adata0 = 0x%x, adata1 = 0x%x \n", data.pdata, data.adata0, data.adata1);

        rpr0521_ps_get_real_value();  //calculate ps status: far or near, and the result is saved to obj->prx_detection_state

        RPR0521_WARNING("ps raw 0x%x,   ps status = %d  \n", obj_tmp->ps_raw_data , obj_tmp->prx_detection_state);


        dat = (int)obj_tmp->prx_detection_state;   //get ready to report

        if (copy_to_user(ptr, &dat, sizeof(dat))) {
            err = -EFAULT;
            return err;
        }

        break;

        case ALSPS_GET_PS_RAW_DATA:

        //read adata0 adata1 and pdata
        err = rpr0521_driver_read_data(obj_tmp->client, &data);   //get ps raw data
        CHECK_RESULT(err);

        RPR0521_WARNING("pdata = 0x%x, adata0 = 0x%x, adata1 = 0x%x \n", data.pdata, data.adata0, data.adata1);

        dat = (int)obj_tmp->ps_raw_data;   
        if (copy_to_user(ptr, &dat, sizeof(dat))) {
            err = -EFAULT;
            return err;
        }
        break;

        case ALSPS_SET_ALS_MODE:

        if (copy_from_user(&enable, ptr, sizeof(enable))) {
            err = -EFAULT;
            return err;
        }

        rpr0521_als_open_report_data(enable);
        rpr0521_als_enable_nodata(enable);

        break;

        case ALSPS_GET_ALS_MODE:
        enable = (int)obj_tmp->als_en_status ? (1) : (0);
        if (copy_to_user(ptr, &enable, sizeof(enable))) {
            err = -EFAULT;
            return err;
        }
        break;

        case ALSPS_GET_ALS_DATA:

        //read adata0 adata1 and pdata
        err = rpr0521_driver_read_data(obj_tmp->client, &data);
        CHECK_RESULT(err);
        RPR0521_WARNING("pdata = 0x%x, adata0 = 0x%x, adata1 = 0x%x \n", data.pdata, data.adata0, data.adata1);


        if (copy_to_user(ptr, &data, sizeof(data))) {
            err = -EFAULT;
            return err;
        }
        break;


        case ALSPS_GET_ALS_RAW_DATA:


        //read adata0 adata1 and pdata
        err = rpr0521_driver_read_data(obj_tmp->client, &data);
        CHECK_RESULT(err);
        RPR0521_WARNING("pdata = 0x%x, adata0 = 0x%x, adata1 = 0x%x \n", data.pdata, data.adata0, data.adata1);

        obj_tmp->data_mlux = rpr0521_als_convert_to_mlux(data.adata0, data.adata1, obj_tmp->als_gain_index, obj_tmp->als_measure_time);

        dat = (int)obj_tmp->data_mlux / 1000;  //Should transfer mlux to lux       
        if (copy_to_user(ptr, &dat, sizeof(dat))) {
            err = -EFAULT;
            return err;
        }
        break;

        case ALSPS_IOCTL_SET_CALI:
#if defined(CALIBRATION_TO_FILE)
        obj->ps_crosstalk = 0;
        err = sensor_calibration_read(ID_PROXIMITY, &obj->ps_crosstalk);
        if(err!=0){
            RPR0521_ERR("Read Cal Fail from file !!!\n");
        }
#else
        if (copy_from_user(&obj->ps_crosstalk, ptr, sizeof(ps_cali))) {
            err = -EFAULT;
            goto err_out;
        }
#endif
        if(0 == obj->ps_crosstalk || 65535 == obj->ps_crosstalk)
        {
            obj->ps_crosstalk = RPR521_PS_NOISE_DEFAULT;
        }
        obj->thresh_near = obj->ps_crosstalk + obj->target_pdata;//should add calibration value
        obj->thresh_far  = obj->thresh_near - obj->near_to_far;// and so is
        rpr0521_set_prx_thresh(obj->thresh_far, obj->thresh_near);
        break;

        default:
        RPR0521_ERR("%s not supported = 0x%04x", __func__, cmd);
        err = -ENOIOCTLCMD;
        break;
    }

    return err;
}

#ifdef CONFIG_COMPAT
static long compat_rpr0521_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    RPR0521_FUN();

    if (!filp->f_op || !filp->f_op->unlocked_ioctl) {
        RPR0521_ERR("compat_ion_ioctl file has no f_op or no f_op->unlocked_ioctl.\n");
        return -ENOTTY;
    }

    switch (cmd) {
        case COMPAT_ALSPS_SET_PS_MODE:
        case COMPAT_ALSPS_GET_PS_MODE:
        case COMPAT_ALSPS_GET_PS_DATA:
        case COMPAT_ALSPS_GET_PS_RAW_DATA:
        case COMPAT_ALSPS_SET_ALS_MODE:
        case COMPAT_ALSPS_GET_ALS_MODE:
        case COMPAT_ALSPS_GET_ALS_DATA:
        case COMPAT_ALSPS_GET_ALS_RAW_DATA:
        case COMPAT_ALSPS_GET_PS_TEST_RESULT:
        case COMPAT_ALSPS_GET_ALS_TEST_RESULT:
        case COMPAT_ALSPS_GET_PS_THRESHOLD_HIGH:
        case COMPAT_ALSPS_GET_PS_THRESHOLD_LOW:
        case COMPAT_ALSPS_GET_ALS_THRESHOLD_HIGH:
        case COMPAT_ALSPS_GET_ALS_THRESHOLD_LOW:
        case COMPAT_ALSPS_IOCTL_CLR_CALI:
        case COMPAT_ALSPS_IOCTL_GET_CALI:
        case COMPAT_ALSPS_IOCTL_SET_CALI:
        case COMPAT_ALSPS_SET_PS_THRESHOLD:
        case COMPAT_ALSPS_SET_ALS_THRESHOLD:
        case COMPAT_AAL_SET_ALS_MODE:
        case COMPAT_AAL_GET_ALS_MODE:
        case COMPAT_AAL_GET_ALS_DATA:
            return filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
        default:
            RPR0521_ERR("compat_ion_ioctl : No such command!! 0x%x\n", cmd);
            return -ENOIOCTLCMD;
    }
}
#endif

static const struct file_operations rpr0521_fops = {
    .owner = THIS_MODULE,
    .open = rpr0521_open,
    .release = rpr0521_release,
    .unlocked_ioctl = rpr0521_unlocked_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
    .compat_ioctl = compat_rpr0521_unlocked_ioctl,
#endif    
};
/*----------------------------------------------------------------------------*/
static struct miscdevice rpr0521_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &rpr0521_fops,

};
/*----------------------------------------------------------------------------*/
static ssize_t rpr0521_show_als_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = obj->client;
    int len = 0;
    int als_value=0;
    int status=0;

    if (NULL == client){
        len += snprintf(buf+len, PAGE_SIZE-len, "i2c client is NULL!!!\n");
        return len;
    }

    rpr0521_als_get_data(&als_value, &status);

    len = snprintf(buf, PAGE_SIZE, "%d\n", als_value);

    return len;
}

static ssize_t rpr0521_show_ps_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = obj->client;
    READ_DATA_ARG       data;
    int len = 0;

    if (NULL == client){
        len += snprintf(buf+len, PAGE_SIZE-len, "i2c client is NULL!!!\n");
        return len;
    }
    len = rpr0521_driver_read_data(obj->client, &data);
    if(len < 0){
        len += snprintf(buf+len, PAGE_SIZE-len, "read ps data fialed!!!\n");
        return len;
    }

    len = snprintf(buf, PAGE_SIZE, " pdata_raw=%d;\n adata0=%d;\n adata1=%d;\n thresh_near:%d;\n thresh_far:%d\n", data.pdata, data.adata0, data.adata1, obj->thresh_near, obj->thresh_far);

    return len;
}

static ssize_t rpr0521_show_allreg(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    if(!obj){
        len += snprintf(buf+len, PAGE_SIZE-len, "obj is null!!\n");
        return len;
    }

    len += snprintf(buf+len, PAGE_SIZE-len,
            "You can read/write a register just like the follow:\n        read:  echo \"r 0x40     \" > rpr05xx_reg\n        write: ehco \"w 0x40 0xFF\" > rpr05xx_reg\n        para:  echo \"para       \" > rpr05xx_reg\n        (Use dmesg to see kernel log)\n\n");

    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_ID_ADDR       , rpr0521_register_dump( RPR0521_REG_ID_ADDR       ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_ENABLE_ADDR   , rpr0521_register_dump( RPR0521_REG_ENABLE_ADDR   ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_ALS_ADDR      , rpr0521_register_dump( RPR0521_REG_ALS_ADDR      ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_PRX_ADDR      , rpr0521_register_dump( RPR0521_REG_PRX_ADDR      ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_PDATAL_ADDR   , rpr0521_register_dump( RPR0521_REG_PDATAL_ADDR   ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_PDATAH_ADDR   , rpr0521_register_dump( RPR0521_REG_PDATAH_ADDR   ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_ADATA0L_ADDR  , rpr0521_register_dump( RPR0521_REG_ADATA0L_ADDR  ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_ADATA0H_ADDR  , rpr0521_register_dump( RPR0521_REG_ADATA0H_ADDR  ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_ADATA1L_ADDR  , rpr0521_register_dump( RPR0521_REG_ADATA1L_ADDR  ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_ADATA1H_ADDR  , rpr0521_register_dump( RPR0521_REG_ADATA1H_ADDR  ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_INTERRUPT_ADDR, rpr0521_register_dump( RPR0521_REG_INTERRUPT_ADDR));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_PIHTL_ADDR    , rpr0521_register_dump( RPR0521_REG_PIHTL_ADDR    ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_PIHTH_ADDR    , rpr0521_register_dump( RPR0521_REG_PIHTH_ADDR    ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_PILTL_ADDR    , rpr0521_register_dump( RPR0521_REG_PILTL_ADDR    ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_PILTH_ADDR    , rpr0521_register_dump( RPR0521_REG_PILTH_ADDR    ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_AIHTL_ADDR    , rpr0521_register_dump( RPR0521_REG_AIHTL_ADDR    ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_AIHTH_ADDR    , rpr0521_register_dump( RPR0521_REG_AIHTH_ADDR    ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_AILTL_ADDR    , rpr0521_register_dump( RPR0521_REG_AILTL_ADDR    ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", RPR0521_REG_AILTH_ADDR    , rpr0521_register_dump( RPR0521_REG_AILTH_ADDR    ));

    return len;
}



static ssize_t rpr0521_store_reg(struct device_driver *ddri, const char *buf, size_t count)
{
#define MAX_LENGTH (3)

    int reg , i2c_data;
    int i = 0;
    int ret = 0;

    char * str_dest[MAX_LENGTH] = {0};
    char str_src[128];

    char delims[] = " ";
    char *str_result = NULL;
    char *cur_str = str_src;

    if(!obj){
        RPR0521_ERR("obj is null !!!\n");
        return 0;
    }

    memcpy(str_src, buf, count);
    RPR0521_WARNING("Your input buf is: %s\n", str_src );

    //spilt buf by space(" "), and seperated string are saved in str_src[]
    while(( str_result = strsep( &cur_str, delims ))) {
        if( i < MAX_LENGTH){  //max length should be 3
            str_dest[i++] = str_result;
        }
        else{
            //RPR0521_WARNING("break\n");
            break;
        }
    }

    if (!strncmp(str_dest[0], "r", 1)){
        reg = simple_strtol(str_dest[1], NULL, 16);

        //check reg valid
        if(((reg&0xFF) > RPR0521_REG_AILTH_ADDR) || ((reg&0xFF) < RPR0521_REG_ID_ADDR)){
            RPR0521_ERR("reg=0x%x is out of range !!!\n", reg );
            return -1;
        }

        //read i2c data
        rpr0521_register_dump(reg&0xFF);
    }else if (!strncmp(str_dest[0], "w",  1)) {
        reg      = simple_strtol(str_dest[1], NULL, 16);
        i2c_data = simple_strtol(str_dest[2], NULL, 16);

        //check reg valid
        if(((reg&0xFF) > RPR0521_REG_AILTH_ADDR) || ((reg&0xFF) < RPR0521_REG_ID_ADDR)){
            RPR0521_ERR("reg=0x%x is out of range !!!\n", reg );
            return -1;
        }

        //write i2c data
        ret = i2c_smbus_write_byte_data(obj->client, reg&0xFF, i2c_data&0xFF);
        if (ret < 0) {
            RPR0521_ERR( " I2C read error !!!  \n" );
            return -1;
        }
        RPR0521_WARNING("writing...reg=0x%x, i2c_data=0x%x success\n", reg, i2c_data);
    }else if(!strncmp(str_dest[0], "para",  4)){  //print parameter
        int i;
        for(i=0;i < COEFFICIENT; i++){
            RPR0521_WARNING ("data0_coefficient[%d] = %5lu, ", i, data0_coefficient[i]);
            RPR0521_WARNING ("data1_coefficient[%d] = %5lu, ", i, data1_coefficient[i]);
            RPR0521_WARNING ("judge_coefficient[%d] = %5lu, ", i, judge_coefficient[i]);
            printk("\n");
        }
    } else{
        RPR0521_ERR("Please input right format: \"r 0x40\", \"w 0x40 0xFF\"\n");
    }

    RPR0521_WARNING( "rpr0521_store_reg count=%d\n", (int)count);

    return count;
}
//==========================================================
// RPR0521 ADB Shell command function
//==========================================================
static ssize_t rpr0521_show_cali_value ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);
	return sprintf( buf, "%d\n", rpr0521_data->cali_status );
}

static ssize_t rpr0521_store_cali_value ( struct device_driver *dev, const char *buf, size_t count )
{
    struct i2c_client *client = obj->client;
    RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);
	int i,ret;

	rpr0521_data->cali_status = CALI_FAIL;
	for( i=0; i < CALIB_TIMES; i++)
	{
		ret = rpr0521_ps_calibration(rpr0521_data);
		if(ret == 0 && rpr0521_data->ps_crosstalk <= 350 )
		{
			RPR0521_ERR("%dth rpr0521_ps_cal = %d\n",i, rpr0521_data->ps_crosstalk);
			break;
		}
	}	
	if(i >= CALIB_TIMES )
	{
		RPR0521_ERR("rpr0521_ps_calibration failed\n");
		rpr0521_data->ps_crosstalk = RPR521_PS_NOISE_DEFAULT; 
		return -1;
	}
	rpr0521_data->cali_status = CALI_SUCCESS;
	
	return count;
}

static ssize_t rpr0521_show_ps_enable ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);
	switch(rpr0521_data -> ps_en_status) {//need for fix (enable_ps_sensor and enable_als_sensor)
		case 0:
			return sprintf ( buf, "%s\n", "Proximity Disabled");
		case 1:
			return sprintf ( buf, "%s\n", "Proximity Enabled" );

		default:
			return sprintf ( buf, "%s\n", "Proximity Error" );
	}

}

static ssize_t rpr0521_store_ps_enable ( struct device_driver *dev, const char *buf, size_t count )
{
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	/*int ret=0;

	switch(val) {
		case 0:
			ret = rpr0521_ps_enable_nodata ( 0 );
			break;
		case 1:
			ret = rpr0521_ps_enable_nodata ( 1 );
			break;
		default:
			break;
	}

	if ( ret < 0 )
		return ret;*/
	rpr0521_ps_open_report_data((int)val);
    rpr0521_ps_enable_nodata((int)val);
			
	return count;
}

static ssize_t rpr0521_show_als_enable ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);	
	switch(rpr0521_data -> als_en_status) {//need for fix (enable_ps_sensor and enable_als_sensor)
		case 0:
			return sprintf ( buf, "%s\n", "Ambient Light Disabled");
		case 1:
			return sprintf ( buf, "%s\n", "Ambient Light Enabled" );
		default:
			return sprintf ( buf, "%s\n", "Ambient Light Error" );
	}
}

static ssize_t rpr0521_store_als_enable ( struct device_driver *dev, const char *buf, size_t count )
{
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	/*int ret=0;

	switch(val) {
		case 0:
			ret = rpr0521_als_enable_nodata ( 0 );
			break;
		case 1:
			ret = rpr0521_als_enable_nodata ( 1 );
			break;
		default:
			break;
	}


	if ( ret < 0 )
		return ret;*/
		
	rpr0521_als_open_report_data((int)val);
    rpr0521_als_enable_nodata((int)val);
	return count;
}
static ssize_t rpr0521_show_near_offset ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);	
	
	return sprintf ( buf, "%d\n", rpr0521_data -> thresh_near);
	
}

static ssize_t rpr0521_store_near_offset ( struct device_driver *dev, const char *buf, size_t count )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	
	rpr0521_data -> thresh_near = (unsigned int)val;
	
    rpr0521_set_prx_thresh(rpr0521_data->thresh_far, rpr0521_data->thresh_near);
	return count;
}
static ssize_t rpr0521_show_far_offset ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);	
	
	return sprintf ( buf, "%d\n", rpr0521_data -> thresh_far);
}

static ssize_t rpr0521_store_far_offset ( struct device_driver *dev, const char *buf, size_t count )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	
	rpr0521_data -> thresh_far = (unsigned int)val;
	
    rpr0521_set_prx_thresh(rpr0521_data->thresh_far, rpr0521_data->thresh_near);
	return count;
}
static ssize_t rpr0521_show_target_pdata ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);	
	
	return sprintf ( buf, "%d\n", rpr0521_data -> target_pdata);
}

static ssize_t rpr0521_store_target_pdata ( struct device_driver *dev, const char *buf, size_t count )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	
	rpr0521_data -> target_pdata = (unsigned int)val;
	
	//should set threshold
    rpr0521_data->thresh_near = rpr0521_data->ps_crosstalk + rpr0521_data->target_pdata;
    rpr0521_data->thresh_far  = rpr0521_data->thresh_near - rpr0521_data->near_to_far;
    rpr0521_set_prx_thresh(rpr0521_data->thresh_far, rpr0521_data->thresh_near);
	
	return count;
}
static ssize_t rpr0521_show_near_to_far ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);	
	
	return sprintf ( buf, "%d\n", rpr0521_data -> near_to_far);
}

static ssize_t rpr0521_store_near_to_far ( struct device_driver *dev, const char *buf, size_t count )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	
	rpr0521_data -> near_to_far = (unsigned int)val;
	
	//should set threshold
    rpr0521_data->thresh_near = rpr0521_data->ps_crosstalk + rpr0521_data->target_pdata;
    rpr0521_data->thresh_far  = rpr0521_data->thresh_near - rpr0521_data->near_to_far;
    rpr0521_set_prx_thresh(rpr0521_data->thresh_far, rpr0521_data->thresh_near);
	
	return count;
}

static ssize_t rpr0521_show_ps_status ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);
	
    if(rpr0521_data -> prx_detection_state == PRX_FAR_AWAY) //get ps status
		return sprintf(buf, "%s\n", "far");
    else if(rpr0521_data -> prx_detection_state == PRX_NEAR_BY) //get ps status
		return sprintf(buf, "%s\n", "near");
	else
		return sprintf(buf, "%s\n", "unknown");
}

static ssize_t rpr0521_show_ps_crosstalk ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);	
	
	return sprintf ( buf, "%d\n", rpr0521_data -> ps_crosstalk);
}

static ssize_t rpr0521_store_ps_crosstalk ( struct device_driver *dev, const char *buf, size_t count )
{
	struct i2c_client *client = obj->client;
	RPR0521_DATA *rpr0521_data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	
	rpr0521_data -> ps_crosstalk = (unsigned int)val;
	
	//should set threshold
    rpr0521_data->thresh_near = rpr0521_data->ps_crosstalk + rpr0521_data->target_pdata;
    rpr0521_data->thresh_far  = rpr0521_data->thresh_near - rpr0521_data->near_to_far;
    rpr0521_set_prx_thresh(rpr0521_data->thresh_far, rpr0521_data->thresh_near);
	
	return count;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR ( rpr05xx_reg, S_IRUGO | S_IWUSR, rpr0521_show_allreg, rpr0521_store_reg);
static DRIVER_ATTR ( luxdata, S_IRUGO, rpr0521_show_als_value, NULL);
static DRIVER_ATTR ( ps, S_IRUGO, rpr0521_show_ps_value, NULL);
static DRIVER_ATTR ( cali, S_IRUGO | S_IWUSR, rpr0521_show_cali_value, rpr0521_store_cali_value );
static DRIVER_ATTR ( enable, S_IRUGO | S_IWUSR, rpr0521_show_ps_enable, rpr0521_store_ps_enable );
static DRIVER_ATTR ( als_enable, S_IRUGO | S_IWUSR, rpr0521_show_als_enable, rpr0521_store_als_enable );
static DRIVER_ATTR ( near_offset, S_IRUGO | S_IWUSR, rpr0521_show_near_offset, rpr0521_store_near_offset );
static DRIVER_ATTR ( far_offset, S_IRUGO | S_IWUSR, rpr0521_show_far_offset, rpr0521_store_far_offset );
static DRIVER_ATTR ( target_pdata, S_IRUGO | S_IWUSR, rpr0521_show_target_pdata, rpr0521_store_target_pdata );
static DRIVER_ATTR ( near_to_far, S_IRUGO | S_IWUSR, rpr0521_show_near_to_far, rpr0521_store_near_to_far );
static DRIVER_ATTR ( status, S_IRUGO, rpr0521_show_ps_status, NULL );
static DRIVER_ATTR ( ps_crosstalk, S_IRUGO | S_IWUSR, rpr0521_show_ps_crosstalk, rpr0521_store_ps_crosstalk );


/*----------------------------------------------------------------------------*/
static struct driver_attribute *rpr0521_attr_list[] = {
    &driver_attr_rpr05xx_reg,
    &driver_attr_luxdata,               /* read als value */
    &driver_attr_ps,                /* read ps value */
	&driver_attr_cali,		   /*show calibration data*/
	&driver_attr_enable,
	&driver_attr_als_enable,
	&driver_attr_near_offset,
	&driver_attr_far_offset,
	&driver_attr_target_pdata,
	&driver_attr_near_to_far,
	&driver_attr_status,
	&driver_attr_ps_crosstalk,
};

/*----------------------------------------------------------------------------*/
static int rpr0521_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(rpr0521_attr_list)/sizeof(rpr0521_attr_list[0]));
    if (driver == NULL){
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++){
        if((err = driver_create_file(driver, rpr0521_attr_list[idx]))){
            RPR0521_WARNING("driver_create_file (%s) = %d\n", rpr0521_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}
/*----------------------------------------------------------------------------*/
static int rpr0521_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(rpr0521_attr_list)/sizeof(rpr0521_attr_list[0]));

    if (!driver){
        return -EINVAL;
    }
    
    for (idx = 0; idx < num; idx++){
        driver_remove_file(driver, rpr0521_attr_list[idx]);
    }

    return err;
}

/*----------------------------------------------------------------------------*/
/**
 * @Brief: rpr0521_power Power control for rpr0521 hardware
 *
 * @Param: hw BM1383 hardware ldo and voltage
 * @Param: on True for power on,flase for power off
 */
static void rpr0521_power(struct alsps_hw *hw, unsigned int on)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0))
    static unsigned int power_on;

    /* check parameter */
    if (NULL == hw){
        RPR0521_ERR(" Parameter error \n");
        return ;
    }

    if (hw->power_id != MT65XX_POWER_NONE) {
        if (power_on == on){
            RPR0521_INFO("ignore power control: %d\n", on);
        }else if (on) {
            if (!hwPowerOn(hw->power_id, hw->power_vol, RPR0521_I2C_NAME))
                RPR0521_ERR("power on fails!!\n");
        }else {
            if (!hwPowerDown(hw->power_id, RPR0521_I2C_NAME))
                RPR0521_ERR("power off fail!!\n");
        }
    }
    power_on = on;
#endif
}

#ifdef ROHM_CALIBRATE
int rpr521_read_ps(struct i2c_client *client, u16 *data)
{

    int tmp ;

    if(client == NULL){
        return -1;
    }

    tmp = i2c_smbus_read_word_data(client, RPR0521_REG_PDATAL_ADDR);
    if(tmp < 0){
        printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
        return tmp;
    }
    
    *data = (unsigned short)(tmp&0xFFF);

    return 0;

}

int rpr521_judge_infra(struct i2c_client *client)
{
    int tmp ;
    unsigned char infrared_data;

    if(client == NULL){
        return -1;
    }

    tmp = i2c_smbus_read_byte_data(client, RPR0521_REG_PRX_ADDR);
    if(tmp < 0){
        printk(KERN_ERR "%s: i2c read ps infra data fail. \n", __func__);
        return tmp;
    }

    infrared_data = tmp;

    if(infrared_data>>6){
        return 0;
    }

    return 1;

}


static int rpr0521_get_offset(RPR0521_DATA *als_ps , unsigned short ct_value)
{
    if(ct_value >= RPR0521_RAW_700){
        als_ps->rpr_max_min_diff = RPR0521_RAW_700_DIFF;
        als_ps->rpr_ht_n_ct = RPR0521_RAW_700_HT_N_CT;
        als_ps->rpr_lt_n_ct = RPR0521_RAW_700_LT_N_CT;
    }
    else if(ct_value >= RPR0521_RAW_500){
        als_ps->rpr_max_min_diff = RPR0521_RAW_500_DIFF;
        als_ps->rpr_ht_n_ct = RPR0521_RAW_500_HT_N_CT;
        als_ps->rpr_lt_n_ct = RPR0521_RAW_500_LT_N_CT;
    }
    else if(ct_value >= RPR0521_RAW_400){
        als_ps->rpr_max_min_diff = RPR0521_RAW_400_DIFF;
        als_ps->rpr_ht_n_ct = RPR0521_RAW_400_HT_N_CT;
        als_ps->rpr_lt_n_ct = RPR0521_RAW_400_LT_N_CT;
    }
    else if(ct_value >= RPR0521_RAW_300){
        als_ps->rpr_max_min_diff = RPR0521_RAW_300_DIFF;
        als_ps->rpr_ht_n_ct = RPR0521_RAW_300_HT_N_CT;
        als_ps->rpr_lt_n_ct = RPR0521_RAW_300_LT_N_CT;
    }
    else if(ct_value >= RPR0521_RAW_200){
        als_ps->rpr_max_min_diff = RPR0521_RAW_200_DIFF;
        als_ps->rpr_ht_n_ct = RPR0521_RAW_200_HT_N_CT;
        als_ps->rpr_lt_n_ct = RPR0521_RAW_200_LT_N_CT;
    }
    else{
        als_ps->rpr_max_min_diff = RPR0521_RAW_DEFAULT_DIFF;
        als_ps->rpr_ht_n_ct =  RPR0521_RAW_DEFAULT_HT_N_CT;
        als_ps->rpr_lt_n_ct =  RPR0521_RAW_DEFAULT_LT_N_CT;
    }
    printk(KERN_INFO "%s: change diff=%d, htnct=%d, ltnct=%d\n", __func__, als_ps->rpr_max_min_diff, als_ps->rpr_ht_n_ct,  als_ps->rpr_lt_n_ct);
    return 0;
}


static int rpr0521_ps_tune_zero_final(RPR0521_DATA *als_ps)
{
    int res ;

    als_ps->tune_zero_init_proc = false;

    res = i2c_smbus_write_byte_data(als_ps->client, RPR0521_REG_ENABLE_ADDR, PS_ALS_SET_MODE_CONTROL); //disable ps interrupt
    if(res< 0){
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return res;
    }


    res = i2c_smbus_write_byte_data(als_ps->client, RPR0521_REG_INTERRUPT_ADDR, PS_ALS_SET_INTR | MODE_PROXIMITY); //disable ps interrupt
    if(res < 0){
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return res;
    }

    if(als_ps->data_count == -1){
        als_ps->ps_th_h_boot = PS_ALS_SET_PS_TH;
        als_ps->ps_th_l_boot = PS_ALS_SET_PS_TL;
        als_ps->thresh_near= als_ps->ps_th_h_boot;
        als_ps->thresh_far = als_ps->ps_th_l_boot;
        als_ps->boot_cali = 0;
        printk(KERN_INFO "%s: exceed limit\n", __func__);
        hrtimer_cancel(&als_ps->ps_tune0_timer);
        return 0;
    }

    als_ps->psa = als_ps->ps_stat_data[0];
    als_ps->psi = als_ps->ps_stat_data[2];
    als_ps->boot_ct = als_ps->ps_stat_data[1];

    rpr0521_get_offset(als_ps, als_ps->boot_ct);

    als_ps->ps_th_h_boot = als_ps->boot_ct + als_ps->rpr_ht_n_ct;
    als_ps->ps_th_l_boot = als_ps->boot_ct + als_ps->rpr_lt_n_ct;
    als_ps->boot_cali = 1;  //boot calibration is successful

    als_ps->thresh_near = als_ps->ps_th_h_boot;
    als_ps->thresh_far = als_ps->ps_th_l_boot;
    als_ps->last_ct = als_ps->ps_stat_data[2];


    rpr0521_set_prx_thresh(als_ps->ps_th_l_boot, als_ps->ps_th_h_boot);

    printk(KERN_INFO "%s: set HT=%d,LT=%d\n", __func__, als_ps->thresh_near,  als_ps->thresh_far);
    hrtimer_cancel(&als_ps->ps_tune0_timer);

    return 0;
}


static int rpr0521_tune_zero_get_ps_data(RPR0521_DATA *als_ps)
{
    unsigned short ps_adc;
    int ret;
    char infra_flag ;

    ret = rpr521_read_ps(als_ps->client, &als_ps->ps);
    if(ret < 0){
        als_ps->data_count = -1;
        rpr0521_ps_tune_zero_final(als_ps);
        return 0;
    }

    ps_adc = als_ps->ps;
    printk(KERN_INFO "%s: ps_adc #%d=%d\n", __func__, als_ps->data_count, ps_adc);
    //if(ps_adc < 0)
    //return ps_adc;

    infra_flag = rpr521_judge_infra(als_ps->client);
    if(infra_flag != 1){
        als_ps->data_count = -1;
        rpr0521_ps_tune_zero_final(als_ps);
        return 0;
    }

    als_ps->ps_stat_data[1]  +=  ps_adc;
    if(ps_adc > als_ps->ps_stat_data[0])
        als_ps->ps_stat_data[0] = ps_adc;
    
    if(ps_adc < als_ps->ps_stat_data[2])
        als_ps->ps_stat_data[2] = ps_adc;
    
    als_ps->data_count++;

    if(als_ps->data_count == RPR521_PS_CALI_LOOP){
        als_ps->ps_stat_data[1]  /= als_ps->data_count;
        rpr0521_ps_tune_zero_final(als_ps);
    }

    return 0;
}

#ifdef IC_RESET_NEED_INITIALIZE
static int rpr521_check_need_reset(RPR0521_DATA *als_ps)
{
    int ret;
    if(!(als_ps->client)){
        return -1;
    }
    ret = i2c_smbus_read_byte_data(als_ps->client, RPR0521_REG_ENABLE_ADDR);
    if(ret < 0){
        printk(KERN_ERR "%s: i2c read  RPR0521_REG_ENABLE_ADDR data fail. \n", __func__);
        return ret;
    }
    
    if(unlikely(0 == ret)){ //ic was reset(default value is 0), initialize again here.
        printk(KERN_ERR "IC need to reset !!! \n");
        hrtimer_cancel(&als_ps->ps_tune0_timer);
        rpr0521_ps_open_report_data(POWER_ON);
        rpr0521_ps_enable_nodata(POWER_ON);
        msleep(100);  //wait 100ms for measurement data ready
    }
    return ret;
}
#endif

static int rpr0521_ps_tune_zero_func_fae(RPR0521_DATA *als_ps)
{
    unsigned short word_data;
    int ret;
    char infra_flag;

    if(!als_ps->ps_en_status){
        return 0;
    }
#ifdef IC_RESET_NEED_INITIALIZE
    rpr521_check_need_reset(als_ps);
    if(false == cali_continue){//skip calibration, beacause calibration was finished
    
        printk(KERN_ERR" Calibration finished, return\n");
        return 0;
    }
#endif

    ret = rpr521_read_ps(als_ps->client, &als_ps->ps);
    if(ret < 0){
        printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
    }

    word_data = als_ps->ps;

    if(word_data == 0){
        //printk(KERN_ERR "%s: incorrect word data (0)\n", __func__);
        return 0xFFFF;
    }

    infra_flag = rpr521_judge_infra(als_ps->client);
    if(infra_flag != 1){
        printk(KERN_ERR "%sinvalid  infra data , infra_flag=0x%x", __func__, infra_flag);
        return 0xFFFF;
    }

    if(word_data > als_ps->psa){
        als_ps->psa = word_data;
        printk(KERN_INFO "%s: update psa: psa=%d,psi=%d\n", __func__, als_ps->psa, als_ps->psi);
    }
    if(word_data < als_ps->psi){
        als_ps->psi = word_data;
        printk(KERN_INFO "%s: update psi: psa=%d,psi=%d\n", __func__, als_ps->psa, als_ps->psi);
    }

#if 0
    if(als_ps->psi_set)
    {
        if(word_data < als_ps->last_ct)
        {
            rpr0521_get_offset( als_ps, word_data);
            als_ps->thresh_near= word_data + als_ps->rpr_ht_n_ct;
            als_ps->thresh_far = word_data + als_ps->rpr_lt_n_ct;
            //rpr0521_set_prx_thresh(als_ps->thresh_far, als_ps->thresh_near);
            rpr0521_set_prx_thresh(RPR0521_PILT_FAR, als_ps->thresh_near);
            als_ps->last_ct = word_data;
            printk("func:%s line:%d\n",__func__, __LINE__);
        }


        //grace modify begin
        if (word_data >= als_ps->thresh_near )
        {
            if (word_data > 1000)
            {
                als_ps->thresh_near = als_ps->psi + 2*als_ps->rpr_ht_n_ct;
                als_ps->thresh_far = als_ps->psi + 3*als_ps->rpr_lt_n_ct;
                als_ps->last_ct = als_ps->psi; 
                rpr0521_set_prx_thresh(als_ps->thresh_far, RPR0521_PIHT_NEAR);
            }
        }
        //grace modify end
        printk("tune1 %s: update HT=%d, LT=%d, word_data=%d als_ps->last_ct=%d  line=%d\n", __func__, als_ps->thresh_near, als_ps->thresh_far, word_data, als_ps->last_ct, __LINE__);
    }
    else
#endif
    {
        int diff;
        diff = als_ps->psa - als_ps->psi;
        if( diff > als_ps->rpr_max_min_diff){

            als_ps->psi_set = als_ps->psi;

            rpr0521_get_offset( als_ps, als_ps->psi);

            als_ps->thresh_near= als_ps->psi + als_ps->rpr_ht_n_ct;
            als_ps->thresh_far = als_ps->psi + als_ps->rpr_lt_n_ct;

            //make sure the max thresh_near is less than (ps_boot + RPR0521_RAW_PS_TH_MAX[120]) when calibrating
            if((als_ps->thresh_near > als_ps->ps_th_h_boot + RPR0521_RAW_PS_TH_MAX)  && (als_ps->boot_cali > 0)){
                als_ps->thresh_near = als_ps->ps_th_h_boot ;
                als_ps->thresh_far  = als_ps->ps_th_l_boot ;

                printk(KERN_INFO "%s: update boot HT=%d, LT=%d\n", __func__,als_ps->ps_th_h_boot, als_ps->ps_th_l_boot);
            }

            rpr0521_set_prx_thresh(als_ps->thresh_far, als_ps->thresh_near);
            als_ps->last_ct = als_ps->psi;

            printk("tune0 %s: update HT=%d, LT=%d line=%d\n", __func__, als_ps->thresh_near, als_ps->thresh_far, __LINE__);

#ifndef IC_RESET_NEED_INITIALIZE
            hrtimer_cancel(&als_ps->ps_tune0_timer);
#else
            cali_continue = false; //calibrate finished, and skip
#endif
        }

    }
#ifdef RPR_DEBUG_PRINTF
    printk("tune0 finsh %s: boot HT=%d, LT=%d, boot_cali=%d  line=%d\n", __func__, als_ps->thresh_near, als_ps->thresh_far, als_ps->boot_cali, __LINE__);
#endif
    return 0;
}

static void rpr0521_ps_tune0_work_func(struct work_struct *work)
{
    RPR0521_DATA *als_ps = container_of(work, RPR0521_DATA, rpr_ps_tune0_work);

    //RPR0521_WARNING("%s\n", __func__);

    //wake_lock(&als_ps->w_wake_lock);
    if(als_ps->tune_zero_init_proc){
        rpr0521_tune_zero_get_ps_data(als_ps);
    }else{
        rpr0521_ps_tune_zero_func_fae(als_ps);
    }
    //wake_unlock(&als_ps->w_wake_lock);
    
    return;
}


static enum hrtimer_restart rpr0521_ps_tune0_timer_func(struct hrtimer *timer)
{
    RPR0521_DATA *als_ps = container_of(timer, RPR0521_DATA, ps_tune0_timer);
    queue_work(als_ps->rpr_ps_tune0_wq, &als_ps->rpr_ps_tune0_work);
    hrtimer_forward_now(&als_ps->ps_tune0_timer, als_ps->ps_tune0_delay);
    return HRTIMER_RESTART;
}
#endif


static int rpr0521_suspend(struct device *dev)
{
    RPR0521_WARNING("%s\n", __func__);
#if 0
    RPR0521_DATA *als_ps = dev_get_drvdata(dev);

    if(als_ps->als_en_status){
        RPR0521_WARNING("%s: Enable ALS : 0\n", __func__);
        rpr0521_driver_als_power_on_off(als_ps->client, POWER_OFF);
        als_ps->als_suspend = true;
    }

    if(als_ps->ps_en_status){
        RPR0521_WARNING("%s: Enable PS : 0\n", __func__);

        disable_irq_nosync(als_ps->irq);

#ifdef ROHM_CALIBRATE
        RPR0521_WARNING("timer state = %lu \n", als_ps->ps_tune0_timer.state);
        if(hrtimer_active(&als_ps->ps_tune0_timer)){
            als_ps->timer_canceled = true;   // timer cacel becuase of sleep
            rpr0521_ps_enable_nodata(POWER_OFF);
        }
        else
#endif
        {
            rpr0521_driver_ps_power_on_off(als_ps->client, POWER_OFF);
        }
        als_ps->ps_suspend = true;
    }
#endif
    return 0;
}

static int rpr0521_resume(struct device *dev)
{
    RPR0521_WARNING("%s\n", __func__);
#if 0
    RPR0521_DATA *als_ps = dev_get_drvdata(dev);

    if(als_ps->als_suspend){
        RPR0521_WARNING("%s: Enable ALS : 1\n", __func__);
        rpr0521_driver_als_power_on_off(als_ps->client, POWER_ON);
        als_ps->als_suspend = false;
    }

    if(als_ps->ps_suspend){
        RPR0521_WARNING("%s: Enable PS : 1\n", __func__);
#ifdef ROHM_CALIBRATE
        if(als_ps->timer_canceled){
            als_ps->timer_canceled = false;
            rpr0521_ps_enable_nodata(POWER_ON);
        }
        else
#endif
        {
             //rpr0521_driver_ps_power_on_off(als_ps->client, POWER_ON);
        }

        enable_irq(als_ps->irq);
        als_ps->ps_suspend = false;
    }
#endif
    return 0;
}


/**
 * @Brief: rpr0521_als_open_report_data RPR0521 initialization or uninitialization
 *
 * @Param: open 1 for initialize,0 for uninitialize
 *
 * @Returns: 0 for success,other for failed.
 */
static int rpr0521_als_open_report_data(int open)
{
    int result = 0;

    RPR0521_WARNING(" open=%d \n", open);

    /* check parameter */
    if (NULL == obj){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    RPR0521_WARNING(" als_en_status=%d ps_en_status=%d\n",obj->als_en_status, obj->ps_en_status);

    if (open){
        //First, we check ps enable status
        if(false == obj->ps_en_status){
            result = rpr0521_driver_init(obj->client);
        }
        obj->als_en_status = true;
    }else{
        //First, we check ps enable status
        if(false == obj->ps_en_status){
            result = rpr0521_driver_shutdown(obj->client);
        }
        obj->als_en_status = false;
    }

    return result;
}

/**
 * @Brief: rpr0521_als_enable_nodata Enable or disable RPR0521
 *
 * @Param: en 1 for enable,0 for disable
 *
 * @Returns: 0 for success,others for failed.
 */
static int rpr0521_als_enable_nodata(int en)
{
    int result = 0;

    RPR0521_FUN();

    if (NULL == obj){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    result = rpr0521_driver_als_power_on_off(obj->client, en);
    if(en){
        msleep(100);
    }
    return result;
}

/**
 * @Brief: rpr0521_als_set_delay Set delay,not used for now.
 *
 * @Param: delay Delay time.
 *
 * @Returns: 0 for success,other for failed.
 */
static int rpr0521_als_set_delay(u64 delay)
{
    RPR0521_FUN();
    //msleep(delay/1000/1000);

    return 0;
}

static void rpr0521_als_init_data(RPR0521_DATA     * obj_ptr)
{
	int i = 0;

	RPR0521_FUN();

	if (!obj_ptr || !obj_ptr->hw) {
		RPR0521_ERR("parameter error! \n");
		return ;
	}

	/* Initialize */
	data0_coefficient[0] = hw->als_level[0];
	data0_coefficient[1] = hw->als_level[1];
	data0_coefficient[2] = hw->als_level[2];
	data0_coefficient[3] = hw->als_level[3];
	data1_coefficient[0] = hw->als_level[4];
	data1_coefficient[1] = hw->als_level[5];
	data1_coefficient[2] = hw->als_level[6];
	data1_coefficient[3] = hw->als_level[7];
	judge_coefficient[0] = hw->als_level[8];
	judge_coefficient[1] = hw->als_level[9];
	judge_coefficient[2] = hw->als_level[10];
	judge_coefficient[3] = hw->als_level[11];
	obj_ptr->polling_mode_als = hw->polling_mode_als;

	for (i = 0; i < 12; i++) {
		RPR0521_WARNING("hw->als_level[%d] = %d\n", i, hw->als_level[i]);
	}
}



/*****************************************************************************
 * @Brief: rpr0521_ps_calibration is used to get ps noise raw data.
 *         we just read 5 times ps raw data and get average, and set this average as ps noise raw data.
 *         Be note: als is disabled during this process, this process will cost 5*15ms at least
 *
 * @Param: obj_ptr point to global RPR0521_DATA
 *
 * @Returns: 0 for success, other for failed.
 *
 *****************************************************************************/
//#if (defined(RPR0521_PS_CALIBRATIOIN_ON_CALL) || defined(RPR0521_PS_CALIBRATIOIN_ON_START))
static int rpr0521_ps_calibration( RPR0521_DATA * obj_ptr)
{
    int ret = 0;
    int i, result, average = 0;
    unsigned char   i2c_read_data[2];
    char i2c_data , enable_old_setting, interrupt_old_setting;
    unsigned char   infrared_data;
    unsigned short  pdata;

    RPR0521_WARNING("rpr0521_calibration IN \n");

    if(!obj_ptr || !obj_ptr->client){
        RPR0521_ERR(" Parameter error \n");
        return -1;
    }

    /* disable ps interrupt begin */
    interrupt_old_setting = i2c_smbus_read_byte_data(obj_ptr->client, RPR0521_REG_INTERRUPT_ADDR);
    if (interrupt_old_setting < 0) {
        RPR0521_ERR( " I2C read error !!!  \n" );
        return -1;
    }

    i2c_data = interrupt_old_setting & 0XFE;

    result = i2c_smbus_write_byte_data(obj_ptr->client, RPR0521_REG_INTERRUPT_ADDR, i2c_data);
    if (result < 0) {
        RPR0521_ERR( " I2C read error !!!  \n" );
        return -1;
    }
    /* disable ps interrupt end */

    /* set ps measurment time to 10ms begin */
    enable_old_setting = i2c_smbus_read_byte_data(obj_ptr->client, RPR0521_REG_ENABLE_ADDR);
    if (enable_old_setting < 0) {
        RPR0521_ERR( " I2C read error !!!  \n" );
        ret = -1;
        goto err_interrupt_status;
    }
    i2c_data = (enable_old_setting & 0x30)  | PS_EN| PS10MS;    //just keep ps_pulse and ps operating mode
    result = i2c_smbus_write_byte_data(obj_ptr->client, RPR0521_REG_ENABLE_ADDR, i2c_data);
    if(result < 0){
        RPR0521_ERR( " I2C read error !!!  \n" );
        ret = -1;
        goto err_interrupt_status;
    }
    /* set ps measurment time to 10ms end */

    /* check the infrared valid */
    i2c_data = i2c_smbus_read_byte_data(obj_ptr->client, RPR0521_REG_PRX_ADDR);
    if(i2c_data < 0){
        RPR0521_ERR( " I2C read error !!!  \n" );
        ret = -1;
        goto err_exit;
    }
    infrared_data = i2c_data;
    if(infrared_data >> 6){  //ambient infrared level is high(too high)
        ret = -1;
        goto err_exit;
    }
    i2c_data = (PS_GAIN_2X << 4) | PS_PERSISTENCE_SETTING;
    result = i2c_smbus_write_byte_data(obj_ptr->client, RPR0521_REG_PRX_ADDR, i2c_data);
    if(result < 0){
        RPR0521_ERR( " I2C read error !!!  \n" );
        ret = -1;
        goto err_exit;
    }
    /* begin read ps raw data */
    for(i = 0; i < RPR521_PS_CALI_LOOP; i ++){
        msleep(15); //sleep, wait IC to finish this measure
        result = i2c_smbus_read_i2c_block_data(obj_ptr->client, RPR0521_REG_PDATAL_ADDR, sizeof(i2c_read_data), i2c_read_data);
        if(result < 0){
            RPR0521_ERR( " I2C read error !!!  \n" );
            ret = -1;
            goto err_exit;
        }
        pdata = (unsigned short)(((unsigned short)i2c_read_data[1] << 8) | i2c_read_data[0]);
        average += pdata & 0xFFF;
        RPR0521_WARNING(" pdata=%d i =%d\n", pdata, i);
    }

    average /= RPR521_PS_CALI_LOOP;  //average

    RPR0521_WARNING("rpr0521_ps_calib average=%d \n", average);
	
    obj_ptr->ps_crosstalk = average;
    obj_ptr->thresh_near = obj_ptr->ps_crosstalk + obj_ptr->target_pdata;
    obj_ptr->thresh_far  = obj_ptr->thresh_near - obj_ptr->near_to_far;
    rpr0521_set_prx_thresh(obj->thresh_far, obj->thresh_near);
#if defined(CALIBRATION_TO_FILE)
	sensor_calibration_save(ID_PROXIMITY, &obj_ptr->ps_crosstalk);
#endif


err_exit:
    i2c_smbus_write_byte_data(obj_ptr->client, RPR0521_REG_ENABLE_ADDR, enable_old_setting);
err_interrupt_status:
    i2c_smbus_write_byte_data(obj_ptr->client, RPR0521_REG_INTERRUPT_ADDR, interrupt_old_setting);

    RPR0521_WARNING("rpr521 PS calibration end\r\n");

    return ret;
}
//#endif

static void rpr0521_ps_init_data(RPR0521_DATA     * obj_ptr)
{
    RPR0521_FUN();

    if(!obj_ptr || !obj_ptr->hw ){
        RPR0521_ERR("parameter error! \n");
        return ;
    }

    /* Initialize */
    obj_ptr->last_nearby = PRX_NEAR_BY_UNKNOWN;
	obj_ptr->cali_status = CALI_FAIL;	
	obj_ptr->ps_crosstalk = RPR521_PS_NOISE_DEFAULT;//common calibration path should be applied
	
	obj_ptr->target_pdata = hw->ps_threshold_high;
	obj_ptr->near_to_far = hw->ps_threshold_low;
    obj_ptr->thresh_near = obj_ptr->ps_crosstalk + obj_ptr->target_pdata;//hw->ps_threshold_high;  // Todo
    obj_ptr->thresh_far  = obj_ptr->thresh_near - obj_ptr->near_to_far;//hw->ps_threshold_low;
    //obj_ptr->thresh_near = RPR521_PS_NOISE_DEFAULT + RPR0521_RAW_DEFAULT_HT_N_CT;  // Todo
    //obj_ptr->thresh_far  = RPR521_PS_NOISE_DEFAULT + RPR0521_RAW_DEFAULT_LT_N_CT;
    obj_ptr->polling_mode_ps   =  obj_ptr->hw->polling_mode_ps;

}




/* #ifdef VENDOR_EDIT */
/* #aaron-liu@rohm, 2017-07-26, Add for rpr0521(interrupt mode for als) */
#if ALS_USE_INTERRUPT_MODE

/*===========================================================================

  FUNCTION      rpr0521_set_als_thresh

  DESCRIPTION   set als interrupt threshold

  DEPENDENCIES  None

  RETURN VALUE  None

  SIDE EFFECT   None

  ===========================================================================*/
static void rpr0521_set_als_thresh(unsigned short adata0 )
{
    unsigned int pilt = 0, piht = 0 , delt = 0;
    RPR0521_FUN();

    if (NULL == obj || NULL == obj->client){
        RPR0521_ERR(" Parameter error \n");
        return ;
    }

#if POCKET_DETECTION
#if defined(CONFIG_FB)
	if (atomic_read(&driver_suspend_flag) == 1)
        {
        RPR0521_ERR("One threshold detect mode\n");
        if (adata0 <= ALS_POCKET_THRES) {
            RPR0521_ERR("als_data(%d)<THRESHOLD(%d), Change bright to dark\n", adata0, ALS_POCKET_THRES);
            pilt = 0;
            piht = ALS_POCKET_THRES;
        } else {
            RPR0521_ERR("als_data(%d)>THRESHOLD(%d), Change dark to bright\n", adata0, ALS_POCKET_THRES);
            pilt = ALS_POCKET_THRES;
            piht = 0xFFFF;
        }
    }
	else if(atomic_read(&driver_suspend_flag)==0)
#endif
#endif
    {
	//20%
		delt = adata0 * RPR0521_ALS_TH_PERCENT / 100;  //20%
		pilt = adata0 - delt;
		piht = adata0 + delt;
		if(piht >= 0xFFFF){
				piht = 0xFFFF;
				pilt = 0xFFFF - (piht - pilt);
				RPR0521_WARNING("piht is out of range!!!\n");
		}
    }
    rpr0521_set_als_thr_reg((unsigned short)pilt, (unsigned short)piht);
}
static void rpr0521_set_als_thr_reg(unsigned short pilt, unsigned short piht)
{
    int result;
    unsigned char thresh[4];
#ifdef RPR0521_DEBUG
    unsigned char read_thresh[4];
#endif

    RPR0521_WARNING("pilt low: 0x%x, piht hig: 0x%x\n", pilt, piht);

    thresh[2] = (pilt & 0xFF); /* PILTL */
    thresh[3] = (pilt >> 8);   /* PILTH */
    thresh[0] = (piht & 0xFF); /* PIHTL */
    thresh[1] = (piht >> 8);   /* PIHTH */

    /*  write block failed, and change to write byte */
    result  = i2c_smbus_write_byte_data(obj->client, RPR0521_REG_AIHTL_ADDR, thresh[0]);
    result |= i2c_smbus_write_byte_data(obj->client, RPR0521_REG_AIHTH_ADDR, thresh[1]);
    result |= i2c_smbus_write_byte_data(obj->client, RPR0521_REG_AILTL_ADDR, thresh[2]);
    result |= i2c_smbus_write_byte_data(obj->client, RPR0521_REG_AILTH_ADDR, thresh[3]);
    if ( result < 0 ){
        RPR0521_ERR("write data from IC error.\n");
        return ;
    }

#ifdef RPR0521_DEBUG
    /*  check write value is successful, or not */
    result = i2c_smbus_read_i2c_block_data(obj->client, RPR0521_REG_AIHTL_ADDR, sizeof(read_thresh), read_thresh);
    if ( result < 0 ){
        RPR0521_ERR("Read data from IC error.\n");
        return ;
    }

    RPR0521_WARNING(" after write reg(0x4F) = 0x%x, reg(0x50) = 0x%x, reg(0x51) = 0x%x, reg(0x52) = 0x%x\n",
            read_thresh[0], read_thresh[1], read_thresh[2], read_thresh[3] );
#endif

}
#endif
/* #endif VENDOR_EDIT */


/*===========================================================================

  FUNCTION      rpr0521_set_prx_thresh

  DESCRIPTION   set ps interrupt threshold

  DEPENDENCIES  None

  RETURN VALUE  None

  SIDE EFFECT   None

  ===========================================================================*/
static void rpr0521_set_prx_thresh( unsigned short pilt, unsigned short piht )
{
    int result;
    unsigned char thresh[4];
    unsigned short pilt_tmp, piht_tmp;

#ifdef RPR0521_DEBUG
    unsigned char read_thresh[4];
#endif

    RPR0521_FUN();

    if (NULL == obj || NULL == obj->client){
        RPR0521_ERR(" Parameter error \n");
        return ;
    }

    pilt_tmp = pilt;
    piht_tmp = piht;

    if(piht >= 0x0FFF){
        piht_tmp = 0x0FFF;
        pilt_tmp = 0x0FFF - (piht - pilt);
        RPR0521_WARNING("piht is out of range!!!\n");
    }

    RPR0521_WARNING("pilt low: 0x%x, piht hig: 0x%x\n", pilt_tmp, piht_tmp);
    thresh[2] = (pilt_tmp & 0xFF); /* PILTL */
    thresh[3] = (pilt_tmp >> 8);   /* PILTH */
    thresh[0] = (piht_tmp & 0xFF); /* PIHTL */
    thresh[1] = (piht_tmp >> 8);   /* PIHTH */

    RPR0521_WARNING(" befor write reg(0x4B) = 0x%x, reg(0x4C) = 0x%x, reg(0x4D) = 0x%x, reg(0x4E) = 0x%x\n",
            thresh[0], thresh[1], thresh[2], thresh[3] );

    /*  write block failed, and change to write byte */
    result  = i2c_smbus_write_byte_data(obj->client, RPR0521_REG_PIHTL_ADDR, thresh[0]);
    result |= i2c_smbus_write_byte_data(obj->client, RPR0521_REG_PIHTH_ADDR, thresh[1]);
    result |= i2c_smbus_write_byte_data(obj->client, RPR0521_REG_PILTL_ADDR, thresh[2]);
    result |= i2c_smbus_write_byte_data(obj->client, RPR0521_REG_PILTH_ADDR, thresh[3]);
    if ( result < 0 ){
        RPR0521_ERR("write data from IC error.\n");
        return ;
    }

#ifdef RPR0521_DEBUG
    /*  check write value is successful, or not */
    result = i2c_smbus_read_i2c_block_data(obj->client, RPR0521_REG_PIHTL_ADDR, sizeof(read_thresh), read_thresh);
    if ( result < 0 ){
        RPR0521_ERR("Read data from IC error.\n");
        return ;
    }

    RPR0521_WARNING(" after write reg(0x4B) = 0x%x, reg(0x4C) = 0x%x, reg(0x4D) = 0x%x, reg(0x4E) = 0x%x\n",
            read_thresh[0], read_thresh[1], read_thresh[2], read_thresh[3] );
#endif

}

/**
 * @Brief: rpr0521_ps_open_report_data RPR0521 initialization or uninitialization
 *
 * @Param: open 1 for initialize,0 for uninitialize
 *
 * @Returns: 0 for success,other for failed.
 */
static int rpr0521_ps_open_report_data(int open)
{
    int result = 0;

    RPR0521_WARNING(" open=%d \n", open);

    /* Check parameter */
    if (NULL == obj || NULL == obj->client){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    RPR0521_WARNING(" als_en_status=%d ps_en_status=%d\n",obj->als_en_status, obj->ps_en_status);

    if (open){  //enable
        /* First, we check als enable status */
        if(obj->als_en_status == false){
            result = rpr0521_driver_init(obj->client);
        }

        obj->ps_en_status = true;
                
    }else{  //disable
        /*  First, we check als enable status */
        if(obj->als_en_status == false){
            result = rpr0521_driver_shutdown(obj->client);
        }
        obj->ps_en_status = false;
    }
    return result;
}

/**
 * @Brief: rpr0521_ps_enable_nodata Enable or disable RPR0521
 *
 * @Param: en 1 for enable,0 for disable
 *
 * @Returns: 0 for success,others for failed.
 */
static int rpr0521_ps_enable_nodata(int en)
{
    int result = 0;
#ifdef OLDVERSION 
	struct hwm_sensor_data sensor_data;
#endif	
    RPR0521_FUN();
    if (NULL == obj || NULL == obj->client){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    
#ifdef RPR0521_PS_CALIBRATIOIN_ON_CALL
        /* calibration */
        result = rpr0521_ps_calibration(obj);
        if(result == 0)
        {
            RPR0521_WARNING("ps calibration success! \n");
        }
        else
        {
            RPR0521_WARNING("ps calibration failed! \n");
        }
#endif

#ifdef ROHM_CALIBRATE
    if (!en){
        hrtimer_cancel(&obj->ps_tune0_timer);
        cancel_work_sync(&obj->rpr_ps_tune0_work);
#ifdef IC_RESET_NEED_INITIALIZE
        cali_continue = false;  //set to false
#endif
    }

    if(en){//enable   
        obj->psi_set = 0;
        obj->psa = 0;
        obj->psi = 0xFFF;
        obj->last_ct = 0xFFF;
        obj->thresh_near = 0XFFF;
        obj->thresh_far  = 0;

        obj->timer_canceled    = false;
    }
#endif

    if(en){

        if(obj->polling_mode_ps == 0){   // set ps interrupt threshold
        	obj->thresh_near = obj->ps_crosstalk + obj->target_pdata;//should add calibration value
        	obj->thresh_far  = obj->thresh_near - obj->near_to_far;// and so is
            rpr0521_set_prx_thresh(obj->thresh_far, obj->thresh_near);
        }
#ifdef OLDVERSION 
		/* inform to upper layer ( hwmsen ) */
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		sensor_data.value_divide = 1;
		sensor_data.values[0] = PRX_FAR_AWAY;
		if (( result =  hwmsen_get_interrupt_data ( ID_PROXIMITY, &sensor_data ))  )
		{
				RPR0521_ERR ( "failed to send inform ( err = %d )\n", result );
		//		goto enable_error;
		}
#endif
#ifdef QUESTION
		obj->prx_detection_state = PRX_FAR_AWAY; //grace modify
		//report far away firstly after enable
		if(ps_report_interrupt_data(obj->prx_detection_state)){  // report far_away at begining
				RPR0521_ERR("call ps_report_interrupt_data fail\n");
		}
#endif
	}

    result = rpr0521_driver_ps_power_on_off(obj->client, en);
#ifdef ROHM_CALIBRATE
    if (en){
#ifdef IC_RESET_NEED_INITIALIZE
        cali_continue = true;  //set to true
#endif
        hrtimer_start(&obj->ps_tune0_timer, obj->ps_tune0_delay, HRTIMER_MODE_REL);  //start ps cali
    }
#endif

    return result;
}

/**
 * @Brief: rpr0521_als_set_delay Set delay,not used for now.
 *
 * @Param: Delay time: unit ns.
 *
 * @Returns: 0 for success,other for failed.
 */
static int rpr0521_ps_set_delay(u64 delay)
{
    RPR0521_FUN();

    //msleep(delay/1000/1000);
    return 0;
}

/*===========================================================================

  FUNCTION      rpr0521_ps_get_real_value

  DESCRIPTION   This function is called to get real proximity status which will be saved to obj->prx_detection_state

  DEPENDENCIES  None

  RETURN VALUE  None

  SIDE EFFECT   None

  ===========================================================================*/
static void rpr0521_ps_get_real_value(void)
{
    unsigned short pdata;

    /* check parameter */
    if (NULL == obj || NULL == obj->client){
        RPR0521_ERR(" Parameter error \n");
        return ;
    }

    pdata = obj->ps_raw_data;

    RPR0521_WARNING("obj->thresh_near(0x%x) obj->thresh_far(0x%x)\n", obj->thresh_near, obj->thresh_far);

    if ( pdata >= obj->thresh_near ){
        int result;

        /* grace modify for special case, ired is too large ,or not */
        result = i2c_smbus_read_byte_data(obj->client, RPR0521_REG_PRX_ADDR);
        if ( result < 0 ){
            RPR0521_ERR("Read data from IC error.\n");
            return ;
        }
        RPR0521_WARNING("RPR0521_PRX_ADDR(0x%x) value = 0x%x\n", RPR0521_REG_PRX_ADDR, result);

        if ( 0 == (result >>  RPR0521_AMBIENT_IR_FLAG) ){ //ir is low
            /* check last ps status */
            if ( obj->last_nearby != PRX_NEAR_BY ){
                obj->prx_detection_state = PRX_NEAR_BY;  //get ps status
            }
            
			RPR0521_WARNING("Near\n");
            obj->last_nearby = PRX_NEAR_BY;

            if (obj->polling_mode_ps == 0){ //interrupt
                rpr0521_set_prx_thresh( obj->thresh_far, RPR0521_PIHT_NEAR ); /* set threshold for last interrupt */
            }
        }
        else{ //special case: ir is high
			RPR0521_WARNING("Sunlight Far\n");
            /* check last ps status */
            if ( obj->last_nearby != PRX_FAR_AWAY ){
                obj->prx_detection_state = PRX_FAR_AWAY; //get ps status
                obj->last_nearby = PRX_FAR_AWAY;
                rpr0521_set_prx_thresh(RPR0521_PILT_FAR, obj->thresh_near);  /* set threshold for last interrupt */
            }
        }
    }
    else if ( pdata < obj->thresh_far ){
        /* check last ps status */
        if ( obj->last_nearby != PRX_FAR_AWAY ){
            obj->prx_detection_state = PRX_FAR_AWAY; //get ps status
        }
        
		RPR0521_WARNING("Far\n");
		obj->last_nearby = PRX_FAR_AWAY;

        if (obj->polling_mode_ps == 0){ //interrupt
            rpr0521_set_prx_thresh(RPR0521_PILT_FAR, obj->thresh_near);   /* set threshold for last interrupt */
        }
    }
}


/*===========================================================================

  FUNCTION      rpr0521_als_convert_to_mlux

  DESCRIPTION   Convert a raw data to a real milli lux

  DEPENDENCIES  None

  RETURN VALUE  milli lux value or 0 if there was a error

  SIDE EFFECT   None

  ===========================================================================*/
unsigned int rpr0521_als_convert_to_mlux( unsigned short  data0, unsigned short  data1, unsigned short  gain_index, unsigned short  time )
{

#define JUDGE_FIXED_COEF (1000)
#define MAX_OUTRANGE     (65535)  //grace modify in 2014.4.9
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXSET_CASE      (4)
#define MLUX_UNIT        (1000)    // Lux to mLux
#define CALC_ERROR       (0xFFFFFFFF)

    unsigned int       final_data;
    calc_data_type     calc_data;
    calc_ans_type      calc_ans;
    unsigned long      max_range;
    unsigned char      gain_factor;

    //RPR0521_DBG("rpr0521 calc als\n");

    /* set the value of measured als data */
    calc_data.als_data0  = data0;
    calc_data.als_data1  = data1;
    gain_factor          = gain_index & 0x0F;
    calc_data.gain_data0 = gain_table[gain_factor].data0;

    max_range = (unsigned long)MAX_OUTRANGE;

    /* calculate data */
    if (calc_data.als_data0 == MAXRANGE_NMODE){
        calc_ans.positive = max_range;
        RPR0521_WARNING("LUX over max range!\n");
    }else{
        unsigned long      calc_judge;
        unsigned char      set_case;

        /* get the value which is measured from power table */
        calc_data.als_time = time ;
        if (calc_data.als_time == 0){
            /* issue error value when time is 0 */
            RPR0521_ERR("calc_data.als_time  == 0\n"); //grace modify in 2014.4.9
            return (CALC_ERROR);
        }

        calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;
        if (calc_judge < (calc_data.als_data0 * judge_coefficient[0])){
            set_case = 0;
        }
        else if (calc_judge < (calc_data.als_data0 * judge_coefficient[1])){
            set_case = 1;
        }
        else if (calc_judge < (calc_data.als_data0 * judge_coefficient[2])){
            set_case = 2;
        }
        else if (calc_judge < (calc_data.als_data0 * judge_coefficient[3])){
            set_case = 3;
        }else{
            set_case = MAXSET_CASE;
        }

        //RPR0521_WARNING("rpr0521-set_case = %d , calc_data.als_time= %d\n", set_case, calc_data.als_time);
        if (set_case >= MAXSET_CASE){
            calc_ans.positive = 0; //which means that lux output is 0
            RPR0521_WARNING("Over max case!\n");
        }
        else{
            calc_data.gain_data1 = gain_table[gain_factor].data1;
            //RPR0521_WARNING("calc_data.gain_data0 = %d calc_data.gain_data1 = %d\n", calc_data.gain_data0, calc_data.gain_data1);
            calc_data.data0      = (unsigned long long )(data0_coefficient[set_case] * calc_data.als_data0 / 10 / calc_data.als_time) * calc_data.gain_data1;
            calc_data.data1      = (unsigned long long )(data1_coefficient[set_case] * calc_data.als_data1 / 10 / calc_data.als_time) * calc_data.gain_data0;
            if(calc_data.data0 < calc_data.data1){    //In this case, data will be less than 0. As data is unsigned long long, it will become extremely big.
                RPR0521_ERR("rpr0521 calc_data.data0 < calc_data.data1\n");
                return (CALC_ERROR);
            }
            //RPR0521_WARNING("gain_data0 = %d gain_data1 = %d data0 = %lld data1 = %lld\n", calc_data.gain_data0, calc_data.gain_data1, calc_data.data0, calc_data.data1);

            calc_data.data     = calc_data.data0 - calc_data.data1;
            calc_data.dev_unit = calc_data.gain_data0 * calc_data.gain_data1;    //24 bit at max (128 * 128 * 100 * 10)
            if (calc_data.dev_unit == 0){
                /* issue error value when dev_unit is 0 */
                RPR0521_ERR("rpr0521 calc_data.dev_unit == 0\n");
                return (CALC_ERROR);
            }

            /* calculate a positive number */
            calc_ans.positive = (unsigned long)((unsigned long)(calc_data.data) / calc_data.dev_unit);
            if (calc_ans.positive > max_range){
                calc_ans.positive = max_range;
            } else {
                // non process
            }
            RPR0521_WARNING("lux=%lu, gain0=%d gain1=%d, data0=%lld data1=%lld, case=%d\n",calc_ans.positive, calc_data.gain_data0, calc_data.gain_data1, calc_data.data0, calc_data.data1, set_case);
        }
    }

    final_data = calc_ans.positive * MLUX_UNIT;

    return (final_data);

#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXSET_CASE
#undef MLUX_UNIT
}

/**
 * @Brief: rpr0521_als_get_data Get data from RPR0521 hardware.
 *
 * @Param: als_value Return value including lux and rgbc.
 * @Param: status Return rpr0521 status.
 *
 * @Returns: 0 for success,other for failed.
 */
static int rpr0521_als_get_data(int *als_value, int *status)
{
    int result;
    READ_DATA_ARG data;

    RPR0521_FUN();

    if((!als_value) || (!status) || (!obj)){
        RPR0521_ERR(" Parameter error \n");
        return -EINVAL;
    }

    // set default value to ALSPS_INVALID_VALUE if it can't get an valid data
    *als_value = ALSPS_INVALID_VALUE;

    //read adata0 adata1 and pdata
    result = rpr0521_driver_read_data(obj->client, &data);
    CHECK_RESULT(result);
//  RPR0521_WARNING("pdata = 0x%x, adata0 = 0x%x, adata1 = 0x%x \n", data.pdata, data.adata0, data.adata1);


    obj->data_mlux = rpr0521_als_convert_to_mlux(data.adata0, data.adata1, obj->als_gain_index, obj->als_measure_time);

/* #ifdef VENDOR_EDIT */
/* #aaron-liu@rohm, 2017-07-26, Add for rpr0521(interrupt mode for als) */
#if ALS_USE_INTERRUPT_MODE
    //set threshold for the next interrupt, by aaron, 2017-07-25
    //rpr0521_set_als_thresh(data.adata0);
#endif
/* #endif VENDOR_EDIT */

    *als_value = obj->data_mlux / 1000;  //Should transfer mlux to lux

#if RPR0521_ALS_GAIN_AUTO_CHANGE
    //if current lux >= 1000 (last_lux < 1000), change gain to 2x
    if( obj->data_mlux  >= RPR0521_ALS_AUTO_LUX_HIGH && obj->last_mlux < RPR0521_ALS_AUTO_LUX_HIGH){
        unsigned char gain_value, enable_value;

        //disable als        
        enable_value = i2c_smbus_read_byte_data(obj->client, RPR0521_REG_ENABLE_ADDR);
        enable_value = enable_value & 0x7f;
        i2c_smbus_write_byte_data(obj->client, RPR0521_REG_ENABLE_ADDR, enable_value);

        RPR0521_WARNING("enable status 0x%x", enable_value);


        //read als current gain data
        //gain_value = i2c_smbus_read_byte_data(obj->client, RPR0521_REG_ALS_ADDR);

        //set to 2X gain
        gain_value = (LEDCURRENT_100MA | ALS_GAIN_2X << 4 | ALS_GAIN_2X << 2); 
        i2c_smbus_write_byte_data(obj->client, RPR0521_REG_ALS_ADDR, gain_value);
        obj->als_gain_index = ALS_GAIN_2X << 2 | ALS_GAIN_2X;

        //enabel als
        enable_value = enable_value | 0x80;
        i2c_smbus_write_byte_data(obj->client, RPR0521_REG_ENABLE_ADDR, enable_value);
        RPR0521_WARNING("enable status 0x%x\n", enable_value);
        RPR0521_WARNING("rpr0521 als raw data : change gain to 2X\n");
    }

    //if current lux < 900, change gain to 64x
    if( obj->data_mlux  < RPR0521_ALS_AUTO_LUX_LOW && obj->last_mlux >= RPR0521_ALS_AUTO_LUX_LOW){
        unsigned char gain_value, enable_value;

        //disable als

        enable_value = i2c_smbus_read_byte_data(obj->client, RPR0521_REG_ENABLE_ADDR);
        enable_value = enable_value & 0x7f;
        i2c_smbus_write_byte_data(obj->client, RPR0521_REG_ENABLE_ADDR, enable_value);

        RPR0521_WARNING("enable status 0x%x\n", enable_value);

        //read als current gain data
        //read als current gain data
        //gain_value = i2c_smbus_read_byte_data(obj->client, RPR0521_REG_ALS_ADDR);

        //set to 64X gain
        gain_value = (LEDCURRENT_100MA | ALS_GAIN_64X << 4 | ALS_GAIN_64X << 2);
        i2c_smbus_write_byte_data(obj->client, RPR0521_REG_ALS_ADDR, gain_value);
        obj->als_gain_index = ALS_GAIN_64X << 2 | ALS_GAIN_64X;


        //enabel als
        enable_value = enable_value | 0x80;
        i2c_smbus_write_byte_data(obj->client, RPR0521_REG_ENABLE_ADDR, enable_value);

        RPR0521_WARNING("enable status 0x%x\n", enable_value);
        RPR0521_WARNING("rpr0521 als raw data : change gain to 64X\n");
    }
#endif  //endif dimmy_auto_change

    obj->last_mlux = obj->data_mlux;   //save last mlux to last_mlux

    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

#ifdef RPR0521_DEBUG
    //debug ps interrupt
    rpr0521_register_dump(RPR0521_REG_ID_ADDR);
    rpr0521_register_dump(RPR0521_REG_INTERRUPT_ADDR);
#endif

    return 0;
}



/**
 * @Brief: rpr0521_ps_get_data Get data from RPR0521 hardware.
 *
 * @Param: als_value Return value including lux and rgbc.
 * @Param: status Return rpr0521 status.
 *
 * @Returns: 0 for success,other for failed.
 */
static int rpr0521_ps_get_data(int *als_value, int *status)
{
    int result;
    READ_DATA_ARG data;

    if(als_value == NULL || status == NULL || obj == NULL){
        RPR0521_ERR(" Parameter error \n");
        return -EINVAL;
    }

    RPR0521_FUN();

    //set default value to ALSPS_INVALID_VALUE if it can't get an valid data
    *als_value = ALSPS_INVALID_VALUE;

    //read adata0 adata1 and pdata
    result = rpr0521_driver_read_data(obj->client, &data);   //get ps raw data
    CHECK_RESULT(result);

    RPR0521_WARNING("pdata = 0x%x, adata0 = 0x%x, adata1 = 0x%x \n",
            data.pdata, data.adata0, data.adata1);

    rpr0521_ps_get_real_value();  //calculate ps status: far or near, and the result is saved to obj->prx_detection_state

    RPR0521_WARNING("ps raw 0x%x -> ps status = %d  \n", obj->ps_raw_data , obj->prx_detection_state);


    *als_value = obj->prx_detection_state;   //get ready to report

    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

    return 0;
}


/**
 * @Brief: rpr0521_local_init Initial RPR0521 driver.
 *
 * @Returns: 0 for success,others for failed.
 */
static int rpr0521_local_init(void)
{
	int err = 0;

	RPR0521_FUN();

	/* check parameter */
	if (NULL == hw){
		RPR0521_ERR(" Parameter error \n");
		return EINVAL;
	}

	/* Power on */
	rpr0521_power(hw, POWER_ON);

	err = i2c_add_driver(&rpr0521_driver);
	if (err < 0) {
		RPR0521_ERR("add driver error\n");
		return -1;
	}

	if (rpr0521_init_flag < 0)
		return -1;

	return 0;
}

/**
 * @Brief: rpr0521_remove Remove RPR0521 driver.
 *
 * @Returns: 0 for success,others for failed.
 */
static int rpr0521_local_remove(void)
{

    RPR0521_FUN();

    /* check parameter */
    if (NULL == hw){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    rpr0521_power(hw, POWER_OFF);
    i2c_del_driver(&rpr0521_driver);

    return 0;
}


/*----------------------------------------------------------------------------*/
static void rpr0521_eint_work(struct work_struct *work)
{
    int result, interrupt_status;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0))
    struct hwm_sensor_data sensor_data;
#endif
    READ_DATA_ARG       data;

    if(NULL == obj){
        return;
    }

    if(NULL == obj->client){
        RPR0521_ERR(" rpr0521_eint_work \n" );
        enable_irq(obj->irq);
        return;
    }

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0))
    memset(&sensor_data, 0, sizeof(sensor_data));
#endif
    RPR0521_WARNING("rpr0521 int top half time = %lld\n", int_top_time);

    //check interrtup status
    interrupt_status = i2c_smbus_read_byte_data(obj->client, RPR0521_REG_INTERRUPT_ADDR);
    if ( interrupt_status < 0 ){
        RPR0521_ERR("Read data from IC error.\n");
        enable_irq(obj->irq);
        return;
    }

    RPR0521_WARNING("RPR0521_REG_INTERRUPT_ADDR(0x4A) value = 0x%x\n", interrupt_status);

    if ((interrupt_status & RPR0521_REG_PINT_STATUS ) && (obj->polling_mode_ps == 0)){
        //get adata0 adata1 and pdata
        result = rpr0521_driver_read_data(obj->client, &data);
        if ( result < 0 ){
            RPR0521_ERR("Read data from IC error.\n");
            enable_irq(obj->irq);
            return;
        }
        RPR0521_WARNING("pdata = 0x%x, adata0 = 0x%x, adata1 = 0x%x \n", data.pdata, data.adata0, data.adata1);
        rpr0521_ps_get_real_value();  //calculate ps status: far or near, and the result is saved to obj->prx_detection_state
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0))
#ifdef OLDVERSION 
        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
        sensor_data.value_divide = 1;
        sensor_data.values[0] = obj->prx_detection_state;      // get to report
        if((result = hwmsen_get_interrupt_data ( ID_PROXIMITY, &sensor_data )))
		{
            RPR0521_ERR("call ps_report_interrupt_data fail\n");
        }
        RPR0521_WARNING("ps raw 0x%x -> value 0x%x \n", data.pdata, sensor_data.values[0]);
#else
        sensor_data.values[0] = obj->prx_detection_state;      // get to report
        sensor_data.value_divide = 1;
        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

        RPR0521_WARNING("ps raw 0x%x -> value 0x%x \n", obj->ps_raw_data, sensor_data.values[0]);

        //let up layer to know
        if(ps_report_interrupt_data(sensor_data.values[0])){  // report
            RPR0521_ERR("call ps_report_interrupt_data fail\n");
        }
#endif
#else
        RPR0521_WARNING("ps raw 0x%x -> value 0x%x \n", obj->ps_raw_data, obj->prx_detection_state);

        //let up layer to know
        if(ps_report_interrupt_data(obj->prx_detection_state)){  // report
            RPR0521_ERR("call ps_report_interrupt_data fail\n");
        }
#endif
    }

/* #ifdef VENDOR_EDIT */
/* #aaron-liu@rohm, 2017-07-26, Add for rpr0521(interrupt mode for als) */
#if ALS_USE_INTERRUPT_MODE
    if ((interrupt_status & RPR0521_REG_AINT_STATUS ) && (obj->polling_mode_als == 0)){
        //get adata0 adata1 and pdata
        result = rpr0521_driver_read_data(obj->client, &data);
        if ( result < 0 ){
            RPR0521_ERR("Read data from IC error.\n");
            enable_irq(obj->irq);
            return;
        }
        RPR0521_WARNING("pdata = 0x%x, adata0 = 0x%x, adata1 = 0x%x \n", data.pdata, data.adata0, data.adata1);
        obj->data_mlux = rpr0521_als_convert_to_mlux(data.adata0, data.adata1, obj->als_gain_index, obj->als_measure_time);
        
        RPR0521_WARNING(" als data_mlux=%d\n", obj->data_mlux);
        rpr0521_set_als_thresh(data.adata0);
    }
#endif
/* #endif VENDOR_EDIT */
    enable_irq(obj->irq);
    RPR0521_WARNING(" irq handler end\n");
}


/*----------------------------------------------------------------------------*/
static irqreturn_t rpr0521_eint_handler(int irq, void *desc)
{

    RPR0521_WARNING(" interrupt handler\n");

    if (NULL == obj){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    int_top_time = sched_clock();

    disable_irq_nosync(obj->irq);
    if(obj->polling_mode_ps == 0)
        schedule_delayed_work(&obj->eint_work,0);

    return IRQ_HANDLED;
}


/*----------------------------------------------------------------------------*/
static int rpr0521_setup_eint(RPR0521_DATA * obj_ptr)
{

    if (NULL == obj_ptr || NULL == obj_ptr->client){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

#if defined(CONFIG_OF)
    if (obj_ptr->irq_node){
        unsigned int ints[2] = {0, 0};

        of_property_read_u32_array(obj_ptr->irq_node, "debounce", ints, ARRAY_SIZE(ints)); // read from dts(dws)
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0))
        mt_gpio_set_debounce(ints[0], ints[1]);
#else
        gpio_set_debounce(ints[0], ints[1]);
#endif
        RPR0521_WARNING("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

        obj_ptr->irq = irq_of_parse_and_map(obj_ptr->irq_node, 0);
        RPR0521_WARNING("obj_ptr->irq = %d\n", obj_ptr->irq);
        if (!obj_ptr->irq){
            RPR0521_ERR("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }

        /*register irq*/
        if(request_irq(obj_ptr->irq, rpr0521_eint_handler, IRQF_TRIGGER_FALLING, "ALS-eint", NULL)) {
            RPR0521_ERR("IRQ LINE NOT AVAILABLE!!\n");
            return -EINVAL;
        }

    }else{
        RPR0521_ERR("null irq node!!\n");
        return -EINVAL;
    }
#endif

    return 0;
}
#ifdef OLDVERSION 
static int rpr0521_ps_operate ( void *self, uint32_t command, void *buff_in, int size_in, void *buff_out, int size_out, int *actualout )
{
	int err = 0;
	int value;
	struct hwm_sensor_data* sensor_data;

	RPR0521_FUN ();

	switch ( command )
	{
		case SENSOR_DELAY:
			RPR0521_WARNING ( "CMD = SENSOR_DELAY\n" );
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				RPR0521_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				RPR0521_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				value = *( int * ) buff_in;
				if ( value )
				{
					RPR0521_WARNING ( "CMD = SENSOR_ENABLE ( Enable )\n" );
					err = rpr0521_ps_enable_nodata ( 1 );
					if ( err ) {
						RPR0521_ERR ( "failed to activate RPR0521 ( err = %d )\n", err );
						return -1;
					}
				}
				else
				{
					RPR0521_WARNING ( "CMD = SENSOR_ENABLE ( Disable )\n" );
					err = rpr0521_ps_enable_nodata ( 0 );
					if ( err ) {
						RPR0521_ERR ( "failed to deactivate RPR0521 ( err = %d )\n", err );
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			RPR0521_WARNING ( "CMD = SENSOR_GET_DATA\n" );
			if ( ( buff_out == NULL ) || ( size_out < sizeof ( struct hwm_sensor_data ) ) )
			{
				RPR0521_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				sensor_data = (struct hwm_sensor_data *)buff_out;
				sensor_data->values[0] = obj -> prx_detection_state;
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				RPR0521_WARNING("values[0] =%d\n",obj -> prx_detection_state);
			}
			break;

		default:
			RPR0521_ERR ( "Invalid Command = %d\n", command );
			err = -1;
			break;
	}

	return err;
}

static int rpr0521_als_operate ( void *self, uint32_t command, void *buff_in, int size_in, void *buff_out, int size_out, int *actualout )
{
	int err = 0;
	int value;
	int status;
	struct hwm_sensor_data* sensor_data;


	RPR0521_FUN ();

	switch ( command )
	{
		case SENSOR_DELAY:
			RPR0521_WARNING ( "CMD = SENSOR_DELAY\n" );
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				RPR0521_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else {
				/* ALS Integration Time setting as fast as polling rate */
				value = *(int *)buff_in;
				rpr0521_als_set_delay((u64)(value*1000));
			}
			break;

		case SENSOR_ENABLE:
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				RPR0521_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				value = *( int * ) buff_in;
				if ( value )
				{
					RPR0521_WARNING ( "CMD = SENSOR_ENABLE ( Enable )\n" );
					err = rpr0521_als_enable_nodata ( 1 );
					if ( err ) {
						RPR0521_ERR ( "failed to activate RPR0521 ( err = %d )\n", err );
						return -1;
					}
				}
				else
				{
					RPR0521_WARNING ( "CMD = SENSOR_ENABLE ( Disable )\n" );
					err = rpr0521_als_enable_nodata ( 0 );
					if ( err ) {
						RPR0521_ERR ( "failed to deactivate RPR0521 ( err = %d )\n", err );
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			RPR0521_WARNING ( "CMD = SENSOR_GET_DATA\n" );
			if ( ( buff_out == NULL ) || ( size_out < sizeof ( struct hwm_sensor_data ) ) )
			{
				RPR0521_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				sensor_data = (struct hwm_sensor_data *)buff_out;
				#if 1
					rpr0521_als_get_data(&(sensor_data->values[0]),&status);
					RPR0521_WARNING("**********lux : %u ***********\n",obj->data_mlux / 1000);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				#else
			        rpr0521_get_alsdata0 ( obj->client, &data0 );
					APS_LOG("********** TEMP Cdata0 : %d \n",data0);
					sensor_data->values[0] = data0;
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				#endif
			}
			break;

		default:
			RPR0521_ERR ( "Invalid Command = %d\n", command );
			err = -1;
			break;
	}

	return err;
}
#endif
/*----------------------------------------------------------------------------*/
static int rpr0521_init_client(RPR0521_DATA * obj_ptr)
{
    int result;

    RPR0521_FUN();

    if (NULL == obj_ptr || NULL == obj_ptr->client){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    if((result = rpr0521_driver_reset(obj_ptr->client))){
        RPR0521_ERR("software reset error, err=%d \n", result);
        return result;
    }
//ps calibrate on boot
#ifdef ROHM_CALIBRATE
    obj_ptr->psi_set = 0;
    obj_ptr->ps_stat_data[0] = 0;
    obj_ptr->ps_stat_data[2] = 4095;
    obj_ptr->ps_stat_data[1] = 0;
    obj_ptr->data_count = 0;
    obj_ptr->ps_th_h_boot = PS_ALS_SET_PS_TH;
    obj_ptr->ps_th_l_boot = PS_ALS_SET_PS_TL;
    obj_ptr->tune_zero_init_proc = true;
    obj_ptr->boot_ct = 0xFFF;
    obj_ptr->boot_cali = 0;

    rpr0521_set_prx_thresh(obj_ptr->ps_th_l_boot, obj_ptr->ps_th_h_boot);

    result =  i2c_smbus_write_byte_data(obj_ptr->client, RPR0521_REG_ENABLE_ADDR, PS_ALS_SET_MODE_CONTROL|PS_EN);    //soft-reset
    if (result != 0) {
        return (result);
    }

    hrtimer_start(&obj_ptr->ps_tune0_timer, obj_ptr->ps_tune0_delay, HRTIMER_MODE_REL);

#endif

    if(obj_ptr->polling_mode_ps == 0 ){  //interrupt mode
    
        RPR0521_FUN();
        if((result = rpr0521_setup_eint(obj_ptr))){
            RPR0521_ERR("setup eint error: %d\n", result);
            return result;
        }
    }

    return 0;
}

/******************************************************************************
 * NAME       : rpr0521_probe
 * FUNCTION   : initialize system
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int result;
#ifdef OLDVERSION 
	int err;
#endif
    RPR0521_DATA *rpr0521_data;
    struct als_control_path als_ctl = {0};  //als control function pointer
    struct als_data_path    als_data = {0}; //als report funcation pointer
    struct ps_control_path ps_ctl = {0};    //ps control function pointer
    struct ps_data_path    ps_data = {0};   //ps report funcation pointer
#ifdef OLDVERSION 
	struct hwmsen_object obj_ps, obj_als;
#endif
	
    RPR0521_WARNING("called rpr0521_probe for RPR0521 \n");

    /* check parameter */
    if (NULL == client){
        RPR0521_ERR(" Parameter error !!! \n");
        return EINVAL;
    }
    result = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
    if (!result) {
        RPR0521_ERR( "need I2C_FUNC_I2C !!!\n");
        result = -ENODEV;
        return result;
    }

    /* read IC id and check valid */
    result = i2c_smbus_read_byte_data(client, RPR0521_REG_ID_ADDR);
    if (result < 0){
        RPR0521_ERR("Read data from IC error !!!\n");
        result = -EIO;
        return result;
    }

    RPR0521_WARNING("MANUFACT_VALUE=0x%x\n", result);
    if((result&0x3F) != RPR0521_REG_ID_VALUE){
        RPR0521_ERR("Error, IC value NOT correct !!! \n");
        result = -EINVAL;
        return result;
    }


    rpr0521_data = kzalloc(sizeof(*rpr0521_data), GFP_KERNEL);
    if (rpr0521_data == NULL) {
        result = -ENOMEM;
        return result;
    }

    //initilize to zero
    memset(rpr0521_data, 0 , sizeof(*rpr0521_data));

    obj = rpr0521_data;

    rpr0521_data->client = client;

    rpr0521_data->hw = hw;


    i2c_set_clientdata(client, rpr0521_data);

    rpr0521_data->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint"); //read irq_node from dws

    if(rpr0521_data->irq_node){
        if(rpr0521_data->irq_node->name){
            RPR0521_WARNING("rpr0521_data->irq_node name = %s\n", rpr0521_data->irq_node->name);
        }
        else{
            RPR0521_WARNING("rpr0521_data->irq_node name is NULL \n");
        }
    }else{
        RPR0521_WARNING("rpr0521_data->irq_node is NULL \n");
    }

    rpr0521_als_init_data(rpr0521_data); /* Initialize als data*/
    rpr0521_ps_init_data(rpr0521_data); /* Initialize ps data*/

#ifdef RPR0521_PS_CALIBRATIOIN_ON_START
    rpr0521_driver_init(client); // Here we should initilize ps if we want to calibrate ps

    /* calibration */
    result = rpr0521_ps_calibration(rpr0521_data);
    if(0 == result)
    {
        RPR0521_WARNING("ps calibration success! \n");
    }
    else
    {
        RPR0521_WARNING("ps calibration failed! \n");
    }
#endif

    INIT_DELAYED_WORK(&rpr0521_data->eint_work, rpr0521_eint_work); /* interrupt work quene*/

#ifdef ROHM_CALIBRATE
    rpr0521_data->rpr_ps_tune0_wq = create_singlethread_workqueue("rpr_ps_tune0_wq");
    INIT_WORK(&rpr0521_data->rpr_ps_tune0_work, rpr0521_ps_tune0_work_func);
    hrtimer_init(&rpr0521_data->ps_tune0_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    rpr0521_data->ps_tune0_delay = ns_to_ktime(100 * NSEC_PER_MSEC);  //interval 100ms 
    rpr0521_data->ps_tune0_timer.function = rpr0521_ps_tune0_timer_func;
    
    //wake_lock_init(&rpr0521_data->w_wake_lock, WAKE_LOCK_SUSPEND, "rpr0521_timer_wakelock");
    
#endif

    if((result = rpr0521_init_client(rpr0521_data))){  /* ps cali, interrupt register*/
            goto err_power_failed;
    }

    if((result = misc_register(&rpr0521_device))){
        RPR0521_WARNING("rpr0521_device register failed result = %d \n", result);
        goto exit_misc_device_register_failed;
    }

	if((result = rpr0521_create_attr(&(rpr0521_init_info.platform_diver_addr->driver) ) )){
    //if((result = rpr0521_create_attr(&(rpr0521_driver.driver)))){
        RPR0521_WARNING("create attribute err = %d\n", result);
        goto exit_create_attr_failed;
    }

    /* als initialize */
    als_ctl.open_report_data = rpr0521_als_open_report_data;
    als_ctl.enable_nodata    = rpr0521_als_enable_nodata;
    als_ctl.set_delay        = rpr0521_als_set_delay;
    als_ctl.is_use_common_factory = false;

    result = als_register_control_path(&als_ctl);
    if (result){
        RPR0521_ERR("als_register_control_path failed, error = %d\n", result);
        goto err_power_failed;
    }

    als_data.get_data   = rpr0521_als_get_data;
    als_data.vender_div = 1;

    result = als_register_data_path(&als_data);
    if (result){
        RPR0521_ERR("als_register_data_path failed, error = %d\n", result);
        goto err_power_failed;
    }

    /* ps initialize */
    ps_ctl.open_report_data = rpr0521_ps_open_report_data;
    ps_ctl.enable_nodata    = rpr0521_ps_enable_nodata;
    ps_ctl.set_delay        = rpr0521_ps_set_delay;
    ps_ctl.is_use_common_factory = false;

    result = ps_register_control_path(&ps_ctl);
    if (result){
        RPR0521_ERR("ps_register_control_path failed, error = %d\n", result);
        goto err_power_failed;
    }

    ps_data.get_data   = rpr0521_ps_get_data;
    ps_data.vender_div = 1;

    result = ps_register_data_path(&ps_data);
    if (result){
        RPR0521_ERR("ps_register_data_path failed, error = %d\n", result);
        goto err_power_failed;
    }
#if POCKET_DETECTION
#if defined(CONFIG_FB)
    fb_notif.notifier_call = light_fb_notifier_callback;
    fb_register_client(&fb_notif);
#endif
#endif

#ifdef OLDVERSION	
	/* hwmsen attach */
	if( 1== obj->polling_mode_ps)
    {
		obj_ps.polling = 1;
    }
    else
    {
		obj_ps.polling = 0;
    }
    obj_ps.self = obj;
    obj_ps.sensor_operate = rpr0521_ps_operate;
    err = hwmsen_attach(ID_PROXIMITY, &obj_ps);
    if(err)
    {
        RPR0521_ERR("attach fail = %d\n", err);
        goto exit_sensor_obj_attach_fail;

    }

	if( 1== obj->polling_mode_als)
    {
        obj_als.polling = 1;
    }
    else
    {
        obj_als.polling = 0;
    }
    obj_als.self = obj;
    obj_als.sensor_operate = rpr0521_als_operate;
    err = hwmsen_attach(ID_LIGHT, &obj_als);
    if(err)
    {
        RPR0521_ERR("attach fail = %d\n", err);
        goto exit_sensor_obj_attach_fail;

    }
#endif	
	rpr0521_init_flag = 0;
	RPR0521_WARNING(" rpr0521_probe for RPR0521 OK \n");

	return (result);

exit_create_attr_failed:
	misc_deregister(&rpr0521_device);
exit_misc_device_register_failed:
#ifdef OLDVERSION 
exit_sensor_obj_attach_fail:
#endif
err_power_failed:
	kfree(rpr0521_data);
	rpr0521_init_flag = -1;
	return (result);
}

/******************************************************************************
 * NAME       : rpr0521_remove
 * FUNCTION   : close system
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_remove(struct i2c_client *client)
{
    int err;
    RPR0521_DATA *rpr0521_data;

    if (NULL == client){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    if((err = rpr0521_delete_attr(&(rpr0521_init_info.platform_diver_addr->driver)))){
        RPR0521_ERR("rpr0521_delete_attr fail: %d\n", err);
    }

    if((err = misc_deregister(&rpr0521_device)))
    {
        RPR0521_ERR("misc_deregister fail: %d\n", err);
    }

#if POCKET_DETECTION
#if defined(CONFIG_FB)
    fb_unregister_client(&fb_notif);
#endif
#endif
    rpr0521_data    = i2c_get_clientdata(client);

    kfree(rpr0521_data);

    return (0);
}

/************************************************************
 * NAME       : rpr0521_driver_init
 * FUNCTION   : initialize RPR0521
 * REMARKS    :
 ***********************************************************/
static int rpr0521_driver_init(struct i2c_client *client)
{
    unsigned char w_mode[4];
    int result;

    RPR0521_FUN();

    if (NULL == client){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* execute software reset */
    result = rpr0521_driver_reset(client);
    if (result != 0) {
        return (result);
    }

    w_mode[0] = (LEDCURRENT_100MA | (ALS_GAIN_2X << 4) | (ALS_GAIN_2X << 2));  // led current 100 mA
    w_mode[1] = (PS_GAIN_2X << 4) | PS_PERSISTENCE_SETTING;
/* #ifdef VENDOR_EDIT */
/* #aaron-liu@rohm, 2017-07-26, Add for rpr0521(interrupt mode for als) */
#if ALS_USE_INTERRUPT_MODE
    w_mode[2] = (PS_THH_BOTH_OUTSIDE| POLA_ACTIVEL | OUTPUT_LATCH | INT_TRIG_BY_PS_AND_ALS);
#else
    w_mode[2] = (PS_THH_BOTH_OUTSIDE| POLA_ACTIVEL | OUTPUT_LATCH | INT_TRIG_BY_ONLY_PS);
#endif
/* #endif VENDOR_EDIT */
    w_mode[3] = (BOTH100MS);


    result = i2c_smbus_write_byte_data(client, RPR0521_REG_ALS_ADDR, w_mode[0]);
    if (result == 0) {
        result = i2c_smbus_write_byte_data(client, RPR0521_REG_PRX_ADDR, w_mode[1]);
        if (result == 0) {
            result = i2c_smbus_write_byte_data(client, RPR0521_REG_INTERRUPT_ADDR, w_mode[2]);
            if (result == 0) {
                result = i2c_smbus_write_byte_data(client, RPR0521_REG_ENABLE_ADDR, w_mode[3]);
            }
        }
    }
/* force als interrupt  */
#if ALS_USE_INTERRUPT_MODE
	rpr0521_set_als_thr_reg(0xffff,0);
#endif
    //set measure time
    if(BOTH100MS == w_mode[3]){
        obj->als_measure_time = CALC_MEASURE_100MS;
    }else if(BOTH400MS == w_mode[3]){
        obj->als_measure_time = CALC_MEASURE_400MS;
    }
    //set measure gain
    obj->als_gain_index = (ALS_GAIN_2X << 2) | (ALS_GAIN_2X);  //saved for caculate lux




#ifdef RPR0521_DEBUG
    rpr0521_register_dump(RPR0521_REG_ENABLE_ADDR);
    rpr0521_register_dump(RPR0521_REG_INTERRUPT_ADDR);
    rpr0521_register_dump(RPR0521_REG_PRX_ADDR);
    rpr0521_register_dump(RPR0521_REG_ALS_ADDR);
#endif

    return (result);
}

/******************************************************************************
 * NAME       : rpr0521_driver_shutdown
 * FUNCTION   : shutdown RPR0521
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_driver_shutdown(struct i2c_client *client)
{
    int result;

    if (NULL == client){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* set soft ware reset */
    result = rpr0521_driver_reset(client);

    return (result);
}

/******************************************************************************
 * NAME       : rpr0521_driver_reset
 * FUNCTION   : reset RPR0521 register
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_driver_reset(struct i2c_client *client)
{
    int result;

    RPR0521_FUN();

    if (NULL == client){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* set soft ware reset */
    result = i2c_smbus_write_byte_data(client, RPR0521_REG_ID_ADDR, 0xC0);
    CHECK_RESULT(result);

    return 0;
}

/******************************************************************************
 * NAME       : rpr0521_driver_power_on_off
 * FUNCTION   : power on and off RPR0521
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_driver_als_power_on_off(struct i2c_client *client, unsigned char data)
{
    int result;
    unsigned char mode_ctl2;
    unsigned char power_set;
    unsigned char write_data;

    if (NULL == client){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }


    RPR0521_WARNING(" data=%d\n", data);

    /* read enable_addr register */
    result = i2c_smbus_read_byte_data(client, RPR0521_REG_ENABLE_ADDR);
    CHECK_RESULT(result);

    if (data == 0) {
        power_set = ALS_OFF;
    } else {
        power_set = ALS_EN;
    }

    /* read enable_addr and mask ALS_EN  */
    mode_ctl2  = (unsigned char)(result & ~ALS_EN);
    write_data = mode_ctl2 | power_set;
    result = i2c_smbus_write_byte_data(client, RPR0521_REG_ENABLE_ADDR, write_data);
    CHECK_RESULT(result);

    return (0);
}


/******************************************************************************
 * NAME       : rpr0521_driver_power_on_off
 * FUNCTION   : power on and off RPR0521
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_driver_ps_power_on_off(struct i2c_client *client, unsigned char data)
{
    int result;
    unsigned char mode_ctl2;
    unsigned char power_set;
    unsigned char write_data;

    if (NULL == client){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }


    RPR0521_WARNING(" data=%d\n", data);

    /* read enable_addr register */
    result = i2c_smbus_read_byte_data(client, RPR0521_REG_ENABLE_ADDR);
    CHECK_RESULT(result);

    if (data == POWER_OFF) {
        power_set = PS_OFF;
    } else {
        power_set = PS_EN;
    }

    RPR0521_WARNING(" RPR0521_REG_ENABLE_ADDR(0x41)=0x%x\n", result);

    /* read enable_addr and mask ALS_EN  */
    mode_ctl2  = (unsigned char)(result & ~PS_EN);
    write_data = mode_ctl2 | power_set;

    RPR0521_WARNING(" write_data=0x%x\n", write_data);

    result = i2c_smbus_write_byte_data(client, RPR0521_REG_ENABLE_ADDR, write_data);
    CHECK_RESULT(result);


#ifdef RPR0521_DEBUG
    rpr0521_register_dump(RPR0521_REG_ENABLE_ADDR);
#endif

    return (0);
}

/******************************************************************************
 * NAME       : rpr0521_driver_read_data
 * FUNCTION   : read the value of RGB data and status in RPR0521
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_driver_read_data(struct i2c_client *client, READ_DATA_ARG *data)
{
    int result;
    unsigned char  read_data[6];

    if (NULL == client || NULL == data){
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* block read */
    result = i2c_smbus_read_i2c_block_data(client, RPR0521_REG_PDATAL_ADDR, sizeof(read_data), read_data);
    if (result < 0) {
        RPR0521_ERR( "ps_rpr0521_driver_general_read : transfer error \n");
    } else {
        data->pdata =  (unsigned short )(((unsigned short )read_data[1] << 8) | read_data[0]);
        data->adata0 = (unsigned short )(((unsigned short )read_data[3] << 8) | read_data[2]);
        data->adata1 = (unsigned short )(((unsigned short )read_data[5] << 8) | read_data[4]);
        result  = 0;
    }

    obj->ps_raw_data = data->pdata;  //save ps raw data

    return (result);
}

/******************************************************************************
 * NAME       : rpr0521_init
 * FUNCTION   : register driver to kernel
 * REMARKS    :
 *****************************************************************************/
static int __init rpr0521_init(void)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0))
    hw = get_cust_alsps_hw();
#else
    const char *name = "mediatek,rpr0521";

    RPR0521_WARNING("rpr0521_init\n");

    hw =   get_alsps_dts_func(name, hw);
    if (!hw) {
        RPR0521_ERR("get dts info fail\n");
		return -1;
	}
#endif

    alsps_driver_add(&rpr0521_init_info);
    return 0;

}

/******************************************************************************
 * NAME       : rpr0521_exit
 * FUNCTION   : remove driver from kernel
 * REMARKS    :
 *****************************************************************************/
static void __exit rpr0521_exit(void)
{
    //nothing to do
    return;
}


MODULE_DESCRIPTION("ROHM Ambient Light And Proximity Sensor Driver");
MODULE_LICENSE("GPL");

module_init(rpr0521_init);
module_exit(rpr0521_exit);
