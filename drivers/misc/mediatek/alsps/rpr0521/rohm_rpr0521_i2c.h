/******************************************************************************
 * MODULE       : rohm_rpr0521_i2c.c
 * FUNCTION     : Driver source for RPR0521, Ambient Light and Proximity Sensor IC
 * AUTHOR       : Aaron Liu
 * MODIFICATION : Modified by Aaron-liu, Sep/5/2016
 * NOTICE       : This software had been verified using MT6795/MT6797.
 *              : When you use this code and document, Please verify all
 *              : operation in your operating system.
 * REMARKS      :
 * COPYRIGHT    : Copyright (C) 2015 - ROHM CO.,LTD.
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

#ifndef _ROHM_RPR0521_I2C_H_
#define _ROHM_RPR0521_I2C_H_


/* comment out for closing debug info */
//#define RPR0521_DEBUG  (NULL)
#define RPR             "[RPR0521-ALS/PS]"

#define  RPR0521_ERR(fmt, arg...)       do {printk(KERN_ERR RPR "ERROR (%s, %d):" fmt, __func__,  __LINE__, ##arg); } while (0)
#define  RPR0521_WARNING(fmt, arg...)   do {printk(KERN_WARNING RPR "(%s, %d):" fmt, __func__,  __LINE__, ##arg);} while (0)

#ifdef RPR0521_DEBUG
#define  RPR0521_FUN(f)              do {printk(KERN_WARNING RPR "(%s, %d)\n", __func__,  __LINE__);} while (0)
#define  RPR0521_INFO(fmt, arg...)   do {printk(KERN_INFO RPR "INFO (%s, %d):" fmt, __func__,  __LINE__, ##arg);} while (0)
#define  RPR0521_DBG(fmt, arg...)    do {printk(KERN_DEBUG RPR "DEBUG (%s, %d):" fmt, __func__,  __LINE__, ##arg);} while (0)
#else
#define  RPR0521_FUN()              do {} while (0)
#define  RPR0521_INFO(f, a...)      do {} while (0)
#define  RPR0521_DBG(f, a...)       do {} while (0)
#endif


/* *************************************************/

/*if the customer do not want to calibrate on call, comment out this macro*/
//#define RPR0521_PS_CALIBRATIOIN_ON_CALL    (1)

/*if the customer do not want to calibrate on boot, comment out this macro*/
//#define RPR0521_PS_CALIBRATIOIN_ON_START   (1)


//#define ROHM_CALIBRATE           (1)         // calibrate by timer
//#define IC_RESET_NEED_INITIALIZE (1)         // IC re-initialize cuase of ESD

#define CALIB_TIMES				(3)
#define RPR521_PS_CALI_LOOP     (10)   //average of 5 times
#define RPR521_PS_NOISE_DEFAULT (14)    //default ps noise data

/* ldo power on/off */
#define POWER_ON    (1)  //on
#define POWER_OFF   (0)  //off

#define RPR0521_AMBIENT_IR_FLAG              (6)  /*    ambient ir flag:
                                                   *     00: infrared low
                                                   *     01: infrared high
                                                   *     02: infrared too hight */

/* *************************************************/


/************ definition to dependent on sensor IC ************/
#define RPR0521_I2C_NAME        "rpr0521_i2c"
#define RPR0521_I2C_DRIVER_NAME "rpr0521_i2c"
#define RPR0521_DRIVER_VER      "rpr0521.2016.10.28"

/************ define register for IC ************/
/* RPR0521 REGSTER */
/* Register Addresses define */
#define RPR0521_REG_ID_ADDR        (0x40)
#define RPR0521_REG_ID_VALUE     (0x0A)      /*ID Value*/
#define RPR0521_REG_ENABLE_ADDR    (0x41)
#define RPR0521_REG_ALS_ADDR        (0x42)
#define RPR0521_REG_PRX_ADDR        (0x43)
#define RPR0521_REG_PDATAL_ADDR    (0x44)
#define RPR0521_REG_PDATAH_ADDR    (0x45)
#define RPR0521_REG_ADATA0L_ADDR    (0x46)
#define RPR0521_REG_ADATA0H_ADDR    (0x47)
#define RPR0521_REG_ADATA1L_ADDR    (0x48)
#define RPR0521_REG_ADATA1H_ADDR    (0x49)
#define RPR0521_REG_INTERRUPT_ADDR    (0x4A)
#define RPR0521_REG_PIHTL_ADDR    (0x4B)
#define RPR0521_REG_PIHTH_ADDR    (0x4C)
#define RPR0521_REG_PILTL_ADDR    (0x4D)
#define RPR0521_REG_PILTH_ADDR    (0x4E)
#define RPR0521_REG_AIHTL_ADDR    (0x4F)
#define RPR0521_REG_AIHTH_ADDR    (0x50)
#define RPR0521_REG_AILTL_ADDR    (0x51)
#define RPR0521_REG_AILTH_ADDR    (0x52)
/* Register Value define : STATUS */
#define RPR0521_REG_PINT_STATUS         (0x80)  /* PRX interrupt status */
#define RPR0521_REG_AINT_STATUS         (0x40)  /* ALS interrupt status */


#define RPR0521_PIHT_NEAR             (0x0FFF)
#define RPR0521_PILT_FAR              (0)

//add for cppcheck, aaron, 2016/8/26
#define RPR0521_RAW_700          (700)
#define RPR0521_RAW_700_DIFF     (80)
#define RPR0521_RAW_700_HT_N_CT  (85)
#define RPR0521_RAW_700_LT_N_CT  (35)

#define RPR0521_RAW_500          (500)
#define RPR0521_RAW_500_DIFF     (80)
#define RPR0521_RAW_500_HT_N_CT  (85)
#define RPR0521_RAW_500_LT_N_CT  (35)

#define RPR0521_RAW_400          (400)
#define RPR0521_RAW_400_DIFF     (80)
#define RPR0521_RAW_400_HT_N_CT  (85)
#define RPR0521_RAW_400_LT_N_CT  (35)

#define RPR0521_RAW_300          (300)
#define RPR0521_RAW_300_DIFF     (80)
#define RPR0521_RAW_300_HT_N_CT  (85)
#define RPR0521_RAW_300_LT_N_CT  (35)

#define RPR0521_RAW_200          (200)
#define RPR0521_RAW_200_DIFF     (80)
#define RPR0521_RAW_200_HT_N_CT  (85)
#define RPR0521_RAW_200_LT_N_CT  (35)

#define RPR0521_RAW_DEFAULT_DIFF     (80)
#define RPR0521_RAW_DEFAULT_HT_N_CT  (85)
#define RPR0521_RAW_DEFAULT_LT_N_CT  (35)

#define RPR0521_RAW_PS_TH_MAX         (120)    


typedef enum
{
    ALS_GAIN_1X    = 0,    /* 1x AGAIN */
    ALS_GAIN_2X    = 1,    /* 2x AGAIN */
    ALS_GAIN_64X   = 2,   /* 64x AGAIN */
    ALS_GAIN_128X  = 3   /* 128x AGAIN */
} als_gain_e;

typedef enum
{
    PS_GAIN_1X   = 0,
    PS_GAIN_2X   = 1,
    PS_GAIN_4X   = 2,
    PS_GAIN_INVALID  =3,
} ps_gain_e;


//grace modify in 2014.4.11 begin
#define ALS_EN          (0x1 << 7)
#define ALS_OFF         (0x0 << 7)
#define PS_EN           (0x1 << 6)
#define PS_OFF          (0x0 << 6)
#define BOTH_STANDBY    (0)
#define PS10MS          (0x1)
#define PS40MS          (0x2)
#define PS100MS         (0x3)
#define ALS100MS_PS50MS (0x5)
#define BOTH100MS       (0x6)
#define BOTH400MS       (0xB)

#define CALC_MEASURE_100MS   (100)
#define CALC_MEASURE_400MS   (400)


#define LEDCURRENT_025MA    (0)
#define LEDCURRENT_050MA    (1)
#define LEDCURRENT_100MA    (2)
#define LEDCURRENT_200MA    (3)

#define PS_THH_ONLY         (0b00 << 4)        /* PS_TH_H and PS_TH_L are effective as outside detection */
#define PS_THH_BOTH_HYS     (0b01 << 4)        /* PS_TH_H and PS_TH_L are effective as outside detection */
#define PS_THH_BOTH_OUTSIDE (0b10 << 4)        /* PS_TH_H and PS_TH_L are effective as outside detection */
#define POLA_ACTIVEL        (0b00 << 3)        /* 0:  Interrupt output "L" is stable if newer measurement result is also interrupt active */
#define POLA_INACTIVEL      (0b01 << 3)        /* 1:  Interrupt output "L" is de-assert and re-assert if newer measurement result is also interrupt active */
#define OUTPUT_ANYTIME      (0b00 << 2)        /* 0: INT pin is latched until INTERRUPT register is read or initialized */
#define OUTPUT_LATCH        (0b01 << 2)        /* 1: INT pin is updated after each measurement */
#define MODE_NONUSE         (0)
#define MODE_PROXIMITY      (1)
#define MODE_ILLUMINANCE    (2)
#define MODE_BOTH           (3)
#define INT_TRIG_IS_INACTIVE    (0b00)   /* INT pin is inactive */
#define INT_TRIG_BY_ONLY_PS     (0b01)   /* Triggerded by only PS */
#define INT_TRIG_BY_ONLY_ALS    (0b10)   /* Triggerded by only ALS */
#define INT_TRIG_BY_PS_AND_ALS  (0b11)   /* Triggerded by PS and ALS */

#define PS_PERSISTENCE_SETTING      (2)  /* interrupt is updated at each measurement end */



#define PS_DOUBLE_PULSE           (1 << 5)
#define PS_ALS_SET_MODE_CONTROL   (PS_DOUBLE_PULSE | ALS100MS_PS50MS)  //grace modify in 2015.2.16
#define PS_ALS_SET_INTR           (PS_THH_BOTH_OUTSIDE| POLA_ACTIVEL | OUTPUT_LATCH | MODE_NONUSE) //grace modify in 2015.2.16
#define PS_ALS_SET_PS_TH          (1000)    //grace modify in 2015.2.16
#define PS_ALS_SET_PS_TL          (999)     //grace modify in 2015.2.16



#define RPR0521_ALS_GAIN_AUTO_CHANGE                (1)        /* 1:  change gain dynamicly */
                                                                /* 0:  don't change gain dynamicly*/
#define RPR0521_ALS_AUTO_LUX_HIGH                (1000000)  /* mlux, high level */
#define RPR0521_ALS_AUTO_LUX_LOW                 (800000)   /* mlux, low level  */

#define RPR0521_ALS_TH_PERCENT                   (20)   /* unit[%], threshold percent  */


/************ typedef struct ************/
/* structure to read data value from sensor */
typedef struct {
    unsigned short pdata;         /* data value of pdata from sensor  */
    unsigned short adata0;        /* data value of adata0 from sensor */
    unsigned short adata1;        /* data value of adata1 from sensor */
} READ_DATA_ARG;



/******************************* define *******************************/
/* structure of peculiarity to use by system */

/* proximity status */
typedef enum
{
    PRX_NEAR_BY,
    PRX_FAR_AWAY,
    PRX_NEAR_BY_UNKNOWN
} prx_nearby_type;
typedef enum
{
	CALI_SUCCESS=0,
	CALI_FAIL=1,

}cali_rst_type;

typedef struct {
    struct i2c_client       *client;                /* structure pointer for i2c bus */
    struct alsps_hw         *hw;                    /*  hw resource */
    struct delayed_work     eint_work;              /*  interrupt work queue */
    struct device_node      *irq_node;              /*  interrupt node in dsti which is saved in dws */

    prx_nearby_type         prx_detection_state;    /* proximity status: Far away or Near by */
    prx_nearby_type         last_nearby;            /* last proximity status */
	cali_rst_type			cali_status;
    bool                    als_en_status;           /* current als enable status */
    bool                    ps_en_status;            /* current ps enable status  */
	
    bool                    als_suspend;             /* current als suspend status */
    bool                    ps_suspend;              /* current ps suspend status  */

    unsigned short          ps_raw_data;             /* proximity raw data */
    unsigned short          thresh_near;             /* ps near threshold */
    unsigned short          thresh_far;              /* ps near threshold */
	unsigned short 			target_pdata;
	unsigned short			near_to_far;
    unsigned short          als_measure_time;        /* als gain for Lux calculation */
    unsigned short          als_gain_index;          /* als gain for Lux calculation */
    int                     irq;                     /* als and ps interrupt number */
    int                     polling_mode_ps;         /* 1: polling mode ; 0:interrupt mode */
    int                     polling_mode_als;        /* 1: polling mode ; 0:interrupt mode */
    unsigned int            data_mlux;               /* als lux(unit:micro lux)*/
    unsigned int            last_mlux;               /* als last lux(unit:micro lux)*/
	/* control flag from HAL*/
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;
	unsigned int ps_crosstalk;
	unsigned int enable;
	unsigned int alslt;
	unsigned int alsht;
#ifdef ROHM_CALIBRATE
    unsigned short last_ct;
    unsigned short psa;
    unsigned short psi;
    unsigned short psi_set;
    unsigned short boot_ct;
    unsigned short boot_cali;
    struct hrtimer ps_tune0_timer;
    struct workqueue_struct *rpr_ps_tune0_wq;
    struct work_struct rpr_ps_tune0_work;
    //struct wake_lock w_wake_lock;                  /* wake lock : system can't enter suspend if this lock was hold */
    ktime_t ps_tune0_delay;
    bool tune_zero_init_proc;
    bool timer_canceled;          /* timer is canceled  */
    unsigned int ps_stat_data[3];
    int data_count;
    int rpr_max_min_diff;
    int rpr_lt_n_ct;
    int rpr_ht_n_ct;
    unsigned short ps_th_h_boot;
    unsigned short ps_th_l_boot;
    unsigned short  ps;

#endif

} RPR0521_DATA;

typedef struct {
    unsigned long long  data;
    unsigned long long  data0;
    unsigned long long  data1;
    unsigned char       gain_data0;
    unsigned char       gain_data1;
    unsigned long       dev_unit;
    unsigned short      als_time; //grace modify in 2014.4.11
    unsigned short      als_data0;
    unsigned short      als_data1;
} calc_data_type;

typedef struct {
    unsigned long positive;
    unsigned long decimal;
} calc_ans_type;


#endif /* _ROHM_RPR0521_I2C_H_ */
