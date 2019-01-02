/* BMA255 motion sensor driver
 *
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 *
 * VERSION: V1.5
 * HISTORY: V1.0 --- Driver creation
 *          V1.1 --- Add share I2C address function
 *          V1.2 --- Fix the bug that sometimes sensor is stuck after system resume.
 *          V1.3 --- Add FIFO interfaces.
 *          V1.4 --- Use basic i2c function to read fifo data instead of i2c DMA mode.
 *          V1.5 --- Add compensated value performed by MTK acceleration calibration process.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/module.h>

#include <accel.h>
#include <sensors_io.h>

#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <hwmsen_helper.h>

#include <cust_acc.h>
#include "bma255.h"
/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_BMA255 255
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
//#define CONFIG_BMA255_LOWPASS   /*apply low pass filter on output*/       
//#define SW_CALIBRATION          /*jin.joo 16/03/31 disable SW calibration
#define CONFIG_I2C_BASIC_FUNCTION
//tad3sgh add ++
#define BMM050_DEFAULT_DELAY	100
#define CALIBRATION_DATA_SIZE	12

#define TRUE 1
#define FALSE 0
/*----------------------------------------------------------------------------*/
/*
* Enable the driver to block e-compass daemon on suspend
*/
#define BMC050_BLOCK_DAEMON_ON_SUSPEND
#undef	BMC050_BLOCK_DAEMON_ON_SUSPEND
/*
* Enable gyroscope feature with BMC050
*/
#define BMC050_M4G	
//#undef BMC050_M4G
/*
* Enable rotation vecter feature with BMC050
*/
#define BMC050_VRV	
//#undef BMC050_VRV	

/*
* Enable virtual linear accelerometer feature with BMC050
*/
#define BMC050_VLA	
//#undef BMC050_VLA

/*
* Enable virtual gravity feature with BMC050
*/
#define BMC050_VG	
//#undef BMC050_VG

#ifdef BMC050_M4G
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_GFLAG			_IOR(MSENSOR, 0x30, short)
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_GDELAY			_IOR(MSENSOR, 0x31, int)
#endif //BMC050_M4G
#ifdef BMC050_VRV
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_VRVFLAG			_IOR(MSENSOR, 0x32, short)
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_VRVDELAY			_IOR(MSENSOR, 0x33, int)
#endif //BMC050_VRV
#ifdef BMC050_VLA
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_VLAFLAG			_IOR(MSENSOR, 0x34, short)
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_VLADELAY			_IOR(MSENSOR, 0x35, int)
#endif //BMC050_VLA
#ifdef BMC050_VG
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_VGFLAG			_IOR(MSENSOR, 0x36, short)
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_VGDELAY			_IOR(MSENSOR, 0x37, int)
#endif //BMC050_VG
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define BMM_IOC_GET_EVENT_FLAG	ECOMPASS_IOC_GET_OPEN_STATUS
//add for non-block
#define BMM_IOC_GET_NONBLOCK_EVENT_FLAG _IOR(MSENSOR, 0x38, int)

// calibration msensor and orientation data
static int sensor_data[CALIBRATION_DATA_SIZE];
#if defined(BMC050_M4G) || defined(BMC050_VRV)
static int m4g_data[CALIBRATION_DATA_SIZE];
#endif //BMC050_M4G || BMC050_VRV
#if defined(BMC050_VLA)
static int vla_data[CALIBRATION_DATA_SIZE];
#endif //BMC050_VLA

#if defined(BMC050_VG)
static int vg_data[CALIBRATION_DATA_SIZE];
#endif //BMC050_VG

static struct mutex sensor_data_mutex;
static DECLARE_WAIT_QUEUE_HEAD(uplink_event_flag_wq);

static int bmm050d_delay = BMM050_DEFAULT_DELAY;
#ifdef BMC050_M4G
static int m4g_delay = BMM050_DEFAULT_DELAY;
#endif //BMC050_M4G
#ifdef BMC050_VRV
static int vrv_delay = BMM050_DEFAULT_DELAY;
#endif //BMC050_VRV
#ifdef BMC050_VLA
static int vla_delay = BMM050_DEFAULT_DELAY;
#endif //BMC050_VRV

#ifdef BMC050_VG
static int vg_delay = BMM050_DEFAULT_DELAY;
#endif //BMC050_VG

static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);
#ifdef BMC050_M4G
static atomic_t g_flag = ATOMIC_INIT(0);
#endif //BMC050_M4G
#ifdef BMC050_VRV
static atomic_t vrv_flag = ATOMIC_INIT(0);
#endif //BMC050_VRV
#ifdef BMC050_VLA
static atomic_t vla_flag = ATOMIC_INIT(0);
#endif //BMC050_VLA
#ifdef BMC050_VG
static atomic_t vg_flag = ATOMIC_INIT(0);
#endif //BMC050_VG

#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
static atomic_t driver_suspend_flag = ATOMIC_INIT(0);
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND

static struct mutex uplink_event_flag_mutex;
/* uplink event flag */
static volatile u32 uplink_event_flag = 0;
/* uplink event flag bitmap */
enum {
	/* active */
	BMMDRV_ULEVT_FLAG_O_ACTIVE = 0x0001,
	BMMDRV_ULEVT_FLAG_M_ACTIVE = 0x0002,
	BMMDRV_ULEVT_FLAG_G_ACTIVE = 0x0004,
	BMMDRV_ULEVT_FLAG_VRV_ACTIVE = 0x0008,/* Virtual Rotation Vector */
	BMMDRV_ULEVT_FLAG_FLIP_ACTIVE = 0x0010,
	BMMDRV_ULEVT_FLAG_VLA_ACTIVE = 0x0020,/* Virtual Linear Accelerometer */
	BMMDRV_ULEVT_FLAG_VG_ACTIVE = 0x0040,/* Virtual Gravity */
	
	/* delay */
	BMMDRV_ULEVT_FLAG_O_DELAY = 0x0100,
	BMMDRV_ULEVT_FLAG_M_DELAY = 0x0200,
	BMMDRV_ULEVT_FLAG_G_DELAY = 0x0400,
	BMMDRV_ULEVT_FLAG_VRV_DELAY = 0x0800,
	BMMDRV_ULEVT_FLAG_FLIP_DELAY = 0x1000,
	BMMDRV_ULEVT_FLAG_VLA_DELAY = 0x2000,
	BMMDRV_ULEVT_FLAG_VG_DELAY = 0x4000,

	/* all */
	BMMDRV_ULEVT_FLAG_ALL = 0xffff
};

/* range for selftest */
enum BMA_RANGE_ENUM {
	BMA_RANGE_2G = 0x0, /* +/- 2G */
	BMA_RANGE_4G,	    /* +/- 4G */
	BMA_RANGE_8G,	    /* +/- 8G */
	BMA_UNDEFINED_RANGE = 0xff
};

//tad3sgh add --
#define MAX_FIFO_F_LEVEL 32
#define MAX_FIFO_F_BYTES 6

#ifdef CONFIG_LGE_SENSOR_SLOPE_DETECTOR
#ifdef CONFIG_MTK_BMA255_INT_ENABLE
#define BMA255_ENABLE_INT1
#endif
#endif

/*----------------------------------------------------------------------------*/
#define BMA255_AXIS_X          0
#define BMA255_AXIS_Y          1
#define BMA255_AXIS_Z          2
#define BMA255_AXES_NUM        3
#define BMA255_DATA_LEN        6
#define BMA255_DEV_NAME        "BMA255"

#define BMA255_MODE_NORMAL      0
#define BMA255_MODE_LOWPOWER    1
#define BMA255_MODE_SUSPEND     2
#ifdef BMA255_ENABLE_INT1
#define BMA255_MODE_DEEP_SUSPEND       3
#define BMA255_MODE_LOWPOWER2          4
#define BMA255_MODE_STANDBY            5

#define HIGH_G_INTERRUPT_X_HAPPENED                 1
#define HIGH_G_INTERRUPT_Y_HAPPENED                 2
#define HIGH_G_INTERRUPT_Z_HAPPENED                 3
#define HIGH_G_INTERRUPT_X_NEGATIVE_HAPPENED        4
#define HIGH_G_INTERRUPT_Y_NEGATIVE_HAPPENED        5
#define HIGH_G_INTERRUPT_Z_NEGATIVE_HAPPENED        6
#define SLOPE_INTERRUPT_X_HAPPENED                  7
#define SLOPE_INTERRUPT_Y_HAPPENED                  8
#define SLOPE_INTERRUPT_Z_HAPPENED                  9
#define SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED         10
#define SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED         11
#define SLOPE_INTERRUPT_Z_NEGATIVE_HAPPENED         12
#define DOUBLE_TAP_INTERRUPT_HAPPENED               13
#define SINGLE_TAP_INTERRUPT_HAPPENED               14
#define UPWARD_PORTRAIT_UP_INTERRUPT_HAPPENED       15
#define UPWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED     16
#define UPWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED    17
#define UPWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED   18
#define DOWNWARD_PORTRAIT_UP_INTERRUPT_HAPPENED     19
#define DOWNWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED   20
#define DOWNWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED  21
#define DOWNWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED 22
#define FLAT_INTERRUPT_TURE_HAPPENED                23
#define FLAT_INTERRUPT_FALSE_HAPPENED               24
#define LOW_G_INTERRUPT_HAPPENED                    25
#define SLOW_NO_MOTION_INTERRUPT_HAPPENED           26


#define PAD_LOWG					0
#define PAD_HIGHG					1
#define PAD_SLOP					2
#define PAD_DOUBLE_TAP				3
#define PAD_SINGLE_TAP				4
#define PAD_ORIENT					5
#define PAD_FLAT					6
#define PAD_SLOW_NO_MOTION			7
#endif // BMA255_ENABLE_INT1

//for bma255 chip.
#define BMA255_ACC_X_LSB__POS           4
#define BMA255_ACC_X_LSB__LEN           4
#define BMA255_ACC_X_LSB__MSK           0xC0//0xF0
#define BMA255_ACC_X_LSB__REG           BMA255_X_AXIS_LSB_REG

#define BMA255_ACC_X_MSB__POS           0
#define BMA255_ACC_X_MSB__LEN           8
#define BMA255_ACC_X_MSB__MSK           0xFF
#define BMA255_ACC_X_MSB__REG           BMA255_X_AXIS_MSB_REG

#define BMA255_ACC_Y_LSB__POS           4
#define BMA255_ACC_Y_LSB__LEN           4
#define BMA255_ACC_Y_LSB__MSK           0xC0//0xF0
#define BMA255_ACC_Y_LSB__REG           BMA255_Y_AXIS_LSB_REG

#define BMA255_ACC_Y_MSB__POS           0
#define BMA255_ACC_Y_MSB__LEN           8
#define BMA255_ACC_Y_MSB__MSK           0xFF
#define BMA255_ACC_Y_MSB__REG           BMA255_Y_AXIS_MSB_REG

#define BMA255_ACC_Z_LSB__POS           4
#define BMA255_ACC_Z_LSB__LEN           4
#define BMA255_ACC_Z_LSB__MSK           0xC0//0xF0
#define BMA255_ACC_Z_LSB__REG           BMA255_Z_AXIS_LSB_REG

#define BMA255_ACC_Z_MSB__POS           0
#define BMA255_ACC_Z_MSB__LEN           8
#define BMA255_ACC_Z_MSB__MSK           0xFF
#define BMA255_ACC_Z_MSB__REG           BMA255_Z_AXIS_MSB_REG

#define BMA255_EN_LOW_POWER__POS          6
#define BMA255_EN_LOW_POWER__LEN          1
#define BMA255_EN_LOW_POWER__MSK          0x40
#define BMA255_EN_LOW_POWER__REG          BMA255_REG_POWER_CTL

#define BMA255_EN_SUSPEND__POS            7
#define BMA255_EN_SUSPEND__LEN            1
#define BMA255_EN_SUSPEND__MSK            0x80
#define BMA255_EN_SUSPEND__REG            BMA255_REG_POWER_CTL

#ifdef BMA255_ENABLE_INT1
#define BMA255_RANGE_SEL__POS             0
#define BMA255_RANGE_SEL__LEN             4
#define BMA255_RANGE_SEL__MSK             0x0F
#define BMA255_RANGE_SEL__REG             BMA255_REG_DATA_FORMAT

#define BMA255_BANDWIDTH__POS             0
#define BMA255_BANDWIDTH__LEN             5
#define BMA255_BANDWIDTH__MSK             0x1F
#define BMA255_BANDWIDTH__REG             BMA255_REG_BW_RATE

#define BMA255_EN_INT1_PAD_LOWG__POS        0
#define BMA255_EN_INT1_PAD_LOWG__LEN        1
#define BMA255_EN_INT1_PAD_LOWG__MSK        0x01
#define BMA255_EN_INT1_PAD_LOWG__REG        BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_INT1_PAD_HIGHG__POS       1
#define BMA255_EN_INT1_PAD_HIGHG__LEN       1
#define BMA255_EN_INT1_PAD_HIGHG__MSK       0x02
#define BMA255_EN_INT1_PAD_HIGHG__REG       BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_INT1_PAD_SLOPE__POS       2
#define BMA255_EN_INT1_PAD_SLOPE__LEN       1
#define BMA255_EN_INT1_PAD_SLOPE__MSK       0x04
#define BMA255_EN_INT1_PAD_SLOPE__REG       BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_INT1_PAD_SLO_NO_MOT__POS        3
#define BMA255_EN_INT1_PAD_SLO_NO_MOT__LEN        1
#define BMA255_EN_INT1_PAD_SLO_NO_MOT__MSK        0x08
#define BMA255_EN_INT1_PAD_SLO_NO_MOT__REG        BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_INT1_PAD_DB_TAP__POS      4
#define BMA255_EN_INT1_PAD_DB_TAP__LEN      1
#define BMA255_EN_INT1_PAD_DB_TAP__MSK      0x10
#define BMA255_EN_INT1_PAD_DB_TAP__REG      BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_INT1_PAD_SNG_TAP__POS     5
#define BMA255_EN_INT1_PAD_SNG_TAP__LEN     1
#define BMA255_EN_INT1_PAD_SNG_TAP__MSK     0x20
#define BMA255_EN_INT1_PAD_SNG_TAP__REG     BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_INT1_PAD_ORIENT__POS      6
#define BMA255_EN_INT1_PAD_ORIENT__LEN      1
#define BMA255_EN_INT1_PAD_ORIENT__MSK      0x40
#define BMA255_EN_INT1_PAD_ORIENT__REG      BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_INT1_PAD_FLAT__POS        7
#define BMA255_EN_INT1_PAD_FLAT__LEN        1
#define BMA255_EN_INT1_PAD_FLAT__MSK        0x80
#define BMA255_EN_INT1_PAD_FLAT__REG        BMA255_INT1_PAD_SEL_REG

#define BMA255_EN_SLOPE_X_INT__POS         0
#define BMA255_EN_SLOPE_X_INT__LEN         1
#define BMA255_EN_SLOPE_X_INT__MSK         0x01
#define BMA255_EN_SLOPE_X_INT__REG         BMA255_INT_REG_1

#define BMA255_EN_SLOPE_Y_INT__POS         1
#define BMA255_EN_SLOPE_Y_INT__LEN         1
#define BMA255_EN_SLOPE_Y_INT__MSK         0x02
#define BMA255_EN_SLOPE_Y_INT__REG         BMA255_INT_REG_1

#define BMA255_EN_SLOPE_Z_INT__POS         2
#define BMA255_EN_SLOPE_Z_INT__LEN         1
#define BMA255_EN_SLOPE_Z_INT__MSK         0x04
#define BMA255_EN_SLOPE_Z_INT__REG         BMA255_INT_REG_1

#define BMA255_SLOPE_DUR__POS              0
#define BMA255_SLOPE_DUR__LEN              2
#define BMA255_SLOPE_DUR__MSK              0x03
#define BMA255_SLOPE_DUR__REG              BMA255_SLOPE_DURN_REG

#define BMA255_SLOPE_THRES__POS                  0
#define BMA255_SLOPE_THRES__LEN                  8
#define BMA255_SLOPE_THRES__MSK                  0xFF
#define BMA255_SLOPE_THRES__REG                  BMA255_SLOPE_THRES_REG

#define BMA255_SLOPE_FIRST_X__POS        0
#define BMA255_SLOPE_FIRST_X__LEN        1
#define BMA255_SLOPE_FIRST_X__MSK        0x01
#define BMA255_SLOPE_FIRST_X__REG        BMA255_STATUS_TAP_SLOPE_REG

#define BMA255_SLOPE_FIRST_Y__POS        1
#define BMA255_SLOPE_FIRST_Y__LEN        1
#define BMA255_SLOPE_FIRST_Y__MSK        0x02
#define BMA255_SLOPE_FIRST_Y__REG        BMA255_STATUS_TAP_SLOPE_REG

#define BMA255_SLOPE_FIRST_Z__POS        2
#define BMA255_SLOPE_FIRST_Z__LEN        1
#define BMA255_SLOPE_FIRST_Z__MSK        0x04
#define BMA255_SLOPE_FIRST_Z__REG        BMA255_STATUS_TAP_SLOPE_REG

#define BMA255_SLOPE_SIGN_S__POS         3
#define BMA255_SLOPE_SIGN_S__LEN         1
#define BMA255_SLOPE_SIGN_S__MSK         0x08
#define BMA255_SLOPE_SIGN_S__REG         BMA255_STATUS_TAP_SLOPE_REG


#define SLOPE_THRESHOLD_VALUE		13
#define SLOPE_DURATION_VALUE		3
#endif // BMA255_ENABLE_INT1

/* fifo mode*/
#define BMA255_FIFO_MODE__POS                 6
#define BMA255_FIFO_MODE__LEN                 2
#define BMA255_FIFO_MODE__MSK                 0xC0
#define BMA255_FIFO_MODE__REG                 BMA255_FIFO_MODE_REG

#define BMA255_FIFO_FRAME_COUNTER_S__POS             0
#define BMA255_FIFO_FRAME_COUNTER_S__LEN             7
#define BMA255_FIFO_FRAME_COUNTER_S__MSK             0x7F
#define BMA255_FIFO_FRAME_COUNTER_S__REG             BMA255_STATUS_FIFO_REG

/* Use CALIBRATION_TO_FILE define for saving cal data to file      */
/* Without CALIBRATION_TO_FILE define, Cal data is saved to MISC2 */
#define CALIBRATION_TO_FILE 1
#define GET_CALIBRATION_Z_FROM_PLACE(x)	(x < 4) ? 1 : 2

#if defined(CALIBRATION_TO_FILE)
#include <linux/syscalls.h>
#include <linux/fs.h>
#include "../../sensor_cal/sensor_cal_file_io.h"
#define CAL_TO_PERSIST

#ifdef CAL_TO_PERSIST
#define SENSOR_CAL_FILENAME "/persist-lg/sensor/sensor_cal_data.txt"
#else
#define SENSOR_CAL_FILENAME "/data/misc/sensor/sensor_cal_data.txt"
#endif
#endif

#define BMA255_ACCEL_CALIBRATION 1

#ifdef BMA255_ACCEL_CALIBRATION
#define BMA255_SHAKING_DETECT_THRESHOLD_XY	(200)
#define BMA255_SHAKING_DETECT_THRESHOLD_Z	(400)
/* Calibration zero-g offset check */
#define BMA255_AXIS_X             0
#define BMA255_AXIS_Y             1
#define BMA255_AXIS_Z             2
#define CALIBRATION_DATA_AMOUNT         10
#define TESTLIMIT_XY                     (300)
#define TESTLIMIT_Z_USL_LSB             (1226)
#define TESTLIMIT_Z_LSL_LSB             (822)

#define SHAKING_ZERO_G_ERROR            (-222) 
/* Calibration zero-g offset check */
#endif

struct bma255acc{
	s16	x,
		y,
		z;
} ;
#define BMA255_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define BMA255_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id bma255_i2c_id[] = {{BMA255_DEV_NAME,0},{}};

#ifdef BMA255_ENABLE_INT1
unsigned int bma255_int_gpio_number = 0;
static unsigned int bma255_irq;
#endif

/*----------------------------------------------------------------------------*/
static int bma255_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int bma255_i2c_remove(struct i2c_client *client);

#ifdef BMA255_ENABLE_INT1
static int bma255_set_mode(struct i2c_client *client, unsigned char mode);
static int bma255_set_bandwidth(struct i2c_client *client, unsigned char bandwidth);
#endif // BMA255_ENABLE_INT1

/*----------------------------------------------------------------------------*/
typedef enum {
    BMA_TRC_FILTER  = 0x01,
    BMA_TRC_RAWDATA = 0x02,
    BMA_TRC_IOCTL   = 0x04,
    BMA_TRC_CALI	= 0X08,
    BMA_TRC_INFO	= 0X10,
} BMA_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][BMA255_AXES_NUM];
    int sum[BMA255_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct bma255_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;

    enum BMA_RANGE_ENUM range;
    u8 bandwidth;   
 
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
    s16                     cali_sw[BMA255_AXES_NUM+1];
    struct mutex lock;

    /*data*/
    s8                      offset[BMA255_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[BMA255_AXES_NUM+1];
    u8			    fifo_count;

#if defined(CONFIG_BMA255_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
	/* accelerometer fast calibration */
	atomic_t fast_calib_x_rslt;
	atomic_t fast_calib_y_rslt;
	atomic_t fast_calib_z_rslt;
	atomic_t fast_calib_rslt;
#ifdef BMA255_ENABLE_INT1
	atomic_t slop_detect_enable;
	struct mutex enable_slop_mutex;
	int	slope_value;
	struct work_struct irq_work;
	u8	sld_threshold;
	u8	sld_duration;
#endif
};
#if !defined(CONFIG_HAS_EARLYSUSPEND)    
static int bma255_suspend(struct i2c_client *client, pm_message_t msg);
static int bma255_resume(struct i2c_client *client);
#endif
/*----------------------------------------------------------------------------*/

#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
	{ .compatible = "mediatek,bma255", },
	{},
};
#endif

static struct i2c_driver bma255_i2c_driver = {
    .driver = {
        .name           = BMA255_DEV_NAME,
#ifdef CONFIG_OF
	.of_match_table = gsensor_of_match,
#endif
    },
	.probe      		= bma255_i2c_probe,
	.remove    			= bma255_i2c_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)    
    .suspend            = bma255_suspend,
    .resume             = bma255_resume,
#endif
	.id_table = bma255_i2c_id,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *bma255_i2c_client = NULL;
static struct acc_init_info bma255_init_info;
static struct bma255_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = true;
static int test_status = 0;
static struct GSENSOR_VECTOR3D gsensor_gain;
//static char selftestRes[8]= {0};
static int bma255_init_flag =-1; // 0<==>OK -1 <==> fail

/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(GSE_TAG fmt, ##args)


#define COMPATIABLE_NAME "mediatek,bma255"

struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;

/*----------------------------------------------------------------------------*/
static struct data_resolution bma255_data_resolution[1] = {
 /* combination by {FULL_RES,RANGE}*/
 //for bma255 12bit
    {{ 1, 95}, 512},   // dataformat +/-4g  in 12-bit resolution;  { 1, 95} = 1.95 = (2*4*1000)/(2^12);  512 = (2^12)/(2*4)
};
/*----------------------------------------------------------------------------*/
static struct data_resolution bma255_offset_resolution = {{ 1, 95}, 512};

/* I2C operation functions */
static int bma_i2c_read_block(struct i2c_client *client,
			u8 addr, u8 *data, u8 len)
{
#ifdef CONFIG_I2C_BASIC_FUNCTION
	u8 beg = addr;
	struct i2c_msg msgs[2] = { {0}, {0} };
	int err;

	if (!client)
		return -EINVAL;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",
			addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;/*no error*/
	}

	return err;
#else
	int err = 0;
	err = i2c_smbus_read_i2c_block_data(client, addr, len, data);
	if (err < 0)
		return -1;
	return 0;
#endif
}
#define I2C_BUFFER_SIZE 256
static int bma_i2c_write_block(struct i2c_client *client, u8 addr,
			u8 *data, u8 len)
{
#ifdef CONFIG_I2C_BASIC_FUNCTION
	/*
	*because address also occupies one byte,
	*the maximum length for write is 7 bytes
	*/
	int err, idx = 0, num = 0;
	char buf[32];

	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		GSE_ERR("send command error!!\n");
		return -EFAULT;
	} else {
		err = 0;/*no error*/
	}
	return err;
#else
	int err = 0;
	err = i2c_smbus_write_i2c_block_data(client, addr, len, data);
	if (err < 0)
		return -1;
	return 0;
#endif
}

#ifdef BMA255_ENABLE_INT1
static int bma255_smbus_read_byte(struct i2c_client *client, unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

static int bma255_smbus_write_byte(struct i2c_client *client, unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma255_smbus_read_byte_block(struct i2c_client *client, unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -1;
	return 0;
}
#endif // BMA255_ENABLE_INT1


#ifdef BMA255_ENABLE_INT1
static int bma255_set_int1_pad_sel(struct i2c_client *client, unsigned char int1sel)
{
	int comres = 0;
	unsigned char data;
	unsigned char state;
	state = 0x01;


	switch (int1sel) {
	case 0:
		comres = bma255_smbus_read_byte(client,
				BMA255_EN_INT1_PAD_LOWG__REG, &data);
		data = BMA255_SET_BITSLICE(data, BMA255_EN_INT1_PAD_LOWG,
				state);
		comres = bma255_smbus_write_byte(client,
				BMA255_EN_INT1_PAD_LOWG__REG, &data);
		break;
	case 1:
		comres = bma255_smbus_read_byte(client,
				BMA255_EN_INT1_PAD_HIGHG__REG, &data);
		data = BMA255_SET_BITSLICE(data, BMA255_EN_INT1_PAD_HIGHG,
				state);
		comres = bma255_smbus_write_byte(client,
				BMA255_EN_INT1_PAD_HIGHG__REG, &data);
		break;
	case 2:
		comres = bma255_smbus_read_byte(client,
				BMA255_EN_INT1_PAD_SLOPE__REG, &data);
		data = BMA255_SET_BITSLICE(data, BMA255_EN_INT1_PAD_SLOPE,
				state);
		comres = bma255_smbus_write_byte(client,
				BMA255_EN_INT1_PAD_SLOPE__REG, &data);
		break;
	case 3:
		comres = bma255_smbus_read_byte(client,
				BMA255_EN_INT1_PAD_DB_TAP__REG, &data);
		data = BMA255_SET_BITSLICE(data, BMA255_EN_INT1_PAD_DB_TAP,
				state);
		comres = bma255_smbus_write_byte(client,
				BMA255_EN_INT1_PAD_DB_TAP__REG, &data);
		break;
	case 4:
		comres = bma255_smbus_read_byte(client,
				BMA255_EN_INT1_PAD_SNG_TAP__REG, &data);
		data = BMA255_SET_BITSLICE(data, BMA255_EN_INT1_PAD_SNG_TAP,
				state);
		comres = bma255_smbus_write_byte(client,
				BMA255_EN_INT1_PAD_SNG_TAP__REG, &data);
		break;
	case 5:
		comres = bma255_smbus_read_byte(client,
				BMA255_EN_INT1_PAD_ORIENT__REG, &data);
		data = BMA255_SET_BITSLICE(data, BMA255_EN_INT1_PAD_ORIENT,
				state);
		comres = bma255_smbus_write_byte(client,
				BMA255_EN_INT1_PAD_ORIENT__REG, &data);
		break;
	case 6:
		comres = bma255_smbus_read_byte(client,
				BMA255_EN_INT1_PAD_FLAT__REG, &data);
		data = BMA255_SET_BITSLICE(data, BMA255_EN_INT1_PAD_FLAT,
				state);
		comres = bma255_smbus_write_byte(client,
				BMA255_EN_INT1_PAD_FLAT__REG, &data);
		break;
	case 7:
		comres = bma255_smbus_read_byte(client,
				BMA255_EN_INT1_PAD_SLO_NO_MOT__REG, &data);
		data = BMA255_SET_BITSLICE(data, BMA255_EN_INT1_PAD_SLO_NO_MOT,
				state);
		comres = bma255_smbus_write_byte(client,
				BMA255_EN_INT1_PAD_SLO_NO_MOT__REG, &data);
		break;

	default:
		break;
	}

	return comres;
}

static void bma255_int_init(struct i2c_client *client)
{
	/* maps interrupt to INT1 pin */
	bma255_set_int1_pad_sel(client, PAD_LOWG);
	bma255_set_int1_pad_sel(client, PAD_HIGHG);
	bma255_set_int1_pad_sel(client, PAD_SLOP);
	bma255_set_int1_pad_sel(client, PAD_DOUBLE_TAP);
	bma255_set_int1_pad_sel(client, PAD_SINGLE_TAP);
	bma255_set_int1_pad_sel(client, PAD_ORIENT);
	bma255_set_int1_pad_sel(client, PAD_FLAT);
	bma255_set_int1_pad_sel(client, PAD_SLOW_NO_MOTION);
}



static int bma255_set_slope_duration(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);

	comres = bma255_smbus_read_byte(client,
			BMA255_SLOPE_DUR__REG, &data);
	data = BMA255_SET_BITSLICE(data, BMA255_SLOPE_DUR, duration);
	comres = bma255_smbus_write_byte(client,
			BMA255_SLOPE_DUR__REG, &data);
	obj->sld_duration = data;

	return comres;
}

static int bma255_get_slope_duration(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;


	comres = bma255_smbus_read_byte(client,
			BMA255_SLOPE_DURN_REG, &data);
	data = BMA255_GET_BITSLICE(data, BMA255_SLOPE_DUR);
	*status = data;


	return comres;
}

static int bma255_set_slope_threshold(struct i2c_client *client,
		unsigned char threshold)
{
	int comres = 0;
	unsigned char data;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);

	data = threshold;
	comres = bma255_smbus_write_byte(client,
			BMA255_SLOPE_THRES__REG, &data);
	obj->sld_threshold =  threshold;

	return comres;
}

static int bma255_get_slope_threshold(struct i2c_client *client,
		unsigned char *status)
{
	int comres = 0;
	unsigned char data;


	comres = bma255_smbus_read_byte(client,
			BMA255_SLOPE_THRES_REG, &data);
	*status = data;

	return comres;
}

static void bma255_set_slop_enable(struct i2c_client *client, int enable)
{
	int ret = 0;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	unsigned char data1 = 0;

	mutex_lock(&obj->enable_slop_mutex);

	if (enable) {	//enable
		GSE_LOG("slop enable\n");

		if (!atomic_read(&obj->slop_detect_enable))
		{
			atomic_set(&obj->slop_detect_enable, 1);

			GSE_LOG("slop enable : Accel enable\n");

			bma255_set_mode(client,BMA255_MODE_NORMAL);
			bma255_int_init(client);

			//set slop irq register
			GSE_LOG("debug_core slop enable step 3");
			data1 = BMA255_SET_BITSLICE(data1, BMA255_EN_SLOPE_X_INT, 1);
			ret= bma255_smbus_write_byte(client, BMA255_INT_REG_1,	&data1);
			data1 = BMA255_SET_BITSLICE(data1, BMA255_EN_SLOPE_Y_INT, 1);
			ret= bma255_smbus_write_byte(client, BMA255_INT_REG_1, &data1);
			data1 = BMA255_SET_BITSLICE(data1, BMA255_EN_SLOPE_Z_INT, 1);
			ret= bma255_smbus_write_byte(client, BMA255_INT_REG_1, &data1);

			bma255_set_bandwidth(client, (unsigned char)BMA255_BW_15_63HZ);
			bma255_set_slope_duration(client, (unsigned char) obj->sld_duration);
			bma255_set_slope_threshold(client, (unsigned char) obj->sld_threshold);
		}
    }
    else // disable
    {
        if (atomic_read(&obj->slop_detect_enable))
		{
			GSE_LOG("debug_core slop disable");
			atomic_set(&obj->slop_detect_enable, 0);
			GSE_LOG("debug_core slop disable step 2");
			data1 = BMA255_SET_BITSLICE(data1, BMA255_EN_SLOPE_X_INT, 0);
			ret= bma255_smbus_write_byte(client, BMA255_INT_REG_1, &data1);
			data1 = BMA255_SET_BITSLICE(data1, BMA255_EN_SLOPE_Y_INT, 0);
			ret= bma255_smbus_write_byte(client, BMA255_INT_REG_1, &data1);
			data1 = BMA255_SET_BITSLICE(data1, BMA255_EN_SLOPE_Z_INT, 0);
			ret= bma255_smbus_write_byte(client, BMA255_INT_REG_1, &data1);
			// TODO:
			if (!sensor_power) // if accel enabled, do not disable sensor.
			{
				GSE_LOG("debug_core slop disable step 3");
				bma255_set_mode(client, BMA255_MODE_SUSPEND); // TODO save power...
			}
			cancel_work_sync(&obj->irq_work);
			flush_work(&obj->irq_work);
		}
    }
	mutex_unlock(&obj->enable_slop_mutex);
}

static int of_get_BMA255_platform_data(struct device *dev)
{
	struct device_node *node = NULL;
	u32 ints[2] = { 0, 0 };

	node = of_find_compatible_node(NULL, NULL, "mediatek, gse_1-eint");
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		bma255_int_gpio_number = ints[0];

		bma255_irq = irq_of_parse_and_map(node, 0);
		if (bma255_irq < 0) {
			GSE_ERR("bma255 request_irq IRQ LINE NOT AVAILABLE!.");
			return -1;
		}
		GSE_ERR("bma255_int_gpio_number %d; bma255_irq : %d\n", bma255_int_gpio_number, bma255_irq);
	}
	GSE_ERR("bma255_int_gpio_number %d; bma255_irq : %d\n", bma255_int_gpio_number, bma255_irq);
	return 0;
}
#endif // BMA255_ENABLE_INT1

/*--------------------BMA255 power control function----------------------------------*/
static void BMA255_power(struct acc_hw *hw, unsigned int on) 
{
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int BMA255_SetDataResolution(struct bma255_i2c_data *obj)
{

/*set g sensor dataresolution here*/

/*BMA255 only can set to 10-bit dataresolution, so do nothing in bma255 driver here*/

/*end of set dataresolution*/
 
 /*we set measure range from -2g to +2g in BMA255_SetDataFormat(client, BMA255_RANGE_2G), 
                                                    and set 10-bit dataresolution BMA255_SetDataResolution()*/
                                                    
 /*so bma255_data_resolution[0] set value as {{ 3, 9}, 256} when declaration, and assign the value to obj->reso here*/  

 	obj->reso = &bma255_data_resolution[0];
	return 0;
	
/*if you changed the measure range, for example call: BMA255_SetDataFormat(client, BMA255_RANGE_4G), 
you must set the right value to bma255_data_resolution*/

}
/*----------------------------------------------------------------------------*/
static int BMA255_ReadData(struct i2c_client *client, s16 data[BMA255_AXES_NUM])
{
//	struct bma255_i2c_data *priv = i2c_get_clientdata(client);
	u8 addr = BMA255_REG_DATAXLOW;
	u8 buf[BMA255_DATA_LEN] = {0};
	int err = 0;

	if(NULL == client)
	{
		err = -EINVAL;
		return err;
	}
	err = bma_i2c_read_block(client, addr, buf, BMA255_DATA_LEN);
	if(err)
	{
		GSE_ERR("error: %d\n", err);
	}
	else
	{
		/* Convert sensor raw data to 16-bit integer */
		data[BMA255_AXIS_X] = BMA255_GET_BITSLICE(buf[0], BMA255_ACC_X_LSB)
			|(BMA255_GET_BITSLICE(buf[1],
						BMA255_ACC_X_MSB)<<BMA255_ACC_X_LSB__LEN);
		data[BMA255_AXIS_X] = data[BMA255_AXIS_X] << (sizeof(short)*8-(BMA255_ACC_X_LSB__LEN
					+ BMA255_ACC_X_MSB__LEN));
		data[BMA255_AXIS_X] = data[BMA255_AXIS_X] >> (sizeof(short)*8-(BMA255_ACC_X_LSB__LEN
					+ BMA255_ACC_X_MSB__LEN));
		data[BMA255_AXIS_Y] = BMA255_GET_BITSLICE(buf[2], BMA255_ACC_Y_LSB)
			| (BMA255_GET_BITSLICE(buf[3],
						BMA255_ACC_Y_MSB)<<BMA255_ACC_Y_LSB__LEN);
		data[BMA255_AXIS_Y] = data[BMA255_AXIS_Y] << (sizeof(short)*8-(BMA255_ACC_Y_LSB__LEN
					+ BMA255_ACC_Y_MSB__LEN));
		data[BMA255_AXIS_Y] = data[BMA255_AXIS_Y] >> (sizeof(short)*8-(BMA255_ACC_Y_LSB__LEN
					+ BMA255_ACC_Y_MSB__LEN));
		data[BMA255_AXIS_Z] = BMA255_GET_BITSLICE(buf[4], BMA255_ACC_Z_LSB)
			| (BMA255_GET_BITSLICE(buf[5],
						BMA255_ACC_Z_MSB)<<BMA255_ACC_Z_LSB__LEN);
		data[BMA255_AXIS_Z] = data[BMA255_AXIS_Z] << (sizeof(short)*8-(BMA255_ACC_Z_LSB__LEN
					+ BMA255_ACC_Z_MSB__LEN));
		data[BMA255_AXIS_Z] = data[BMA255_AXIS_Z] >> (sizeof(short)*8-(BMA255_ACC_Z_LSB__LEN
					+ BMA255_ACC_Z_MSB__LEN));

#ifdef CONFIG_BMA255_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][BMA255_AXIS_X] = data[BMA255_AXIS_X];
					priv->fir.raw[priv->fir.num][BMA255_AXIS_Y] = data[BMA255_AXIS_Y];
					priv->fir.raw[priv->fir.num][BMA255_AXIS_Z] = data[BMA255_AXIS_Z];
					priv->fir.sum[BMA255_AXIS_X] += data[BMA255_AXIS_X];
					priv->fir.sum[BMA255_AXIS_Y] += data[BMA255_AXIS_Y];
					priv->fir.sum[BMA255_AXIS_Z] += data[BMA255_AXIS_Z];
					if(atomic_read(&priv->trace) & BMA_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][BMA255_AXIS_X], priv->fir.raw[priv->fir.num][BMA255_AXIS_Y], priv->fir.raw[priv->fir.num][BMA255_AXIS_Z],
							priv->fir.sum[BMA255_AXIS_X], priv->fir.sum[BMA255_AXIS_Y], priv->fir.sum[BMA255_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[BMA255_AXIS_X] -= priv->fir.raw[idx][BMA255_AXIS_X];
					priv->fir.sum[BMA255_AXIS_Y] -= priv->fir.raw[idx][BMA255_AXIS_Y];
					priv->fir.sum[BMA255_AXIS_Z] -= priv->fir.raw[idx][BMA255_AXIS_Z];
					priv->fir.raw[idx][BMA255_AXIS_X] = data[BMA255_AXIS_X];
					priv->fir.raw[idx][BMA255_AXIS_Y] = data[BMA255_AXIS_Y];
					priv->fir.raw[idx][BMA255_AXIS_Z] = data[BMA255_AXIS_Z];
					priv->fir.sum[BMA255_AXIS_X] += data[BMA255_AXIS_X];
					priv->fir.sum[BMA255_AXIS_Y] += data[BMA255_AXIS_Y];
					priv->fir.sum[BMA255_AXIS_Z] += data[BMA255_AXIS_Z];
					priv->fir.idx++;
					data[BMA255_AXIS_X] = priv->fir.sum[BMA255_AXIS_X]/firlen;
					data[BMA255_AXIS_Y] = priv->fir.sum[BMA255_AXIS_Y]/firlen;
					data[BMA255_AXIS_Z] = priv->fir.sum[BMA255_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & BMA_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][BMA255_AXIS_X], priv->fir.raw[idx][BMA255_AXIS_Y], priv->fir.raw[idx][BMA255_AXIS_Z],
						priv->fir.sum[BMA255_AXIS_X], priv->fir.sum[BMA255_AXIS_Y], priv->fir.sum[BMA255_AXIS_Z],
						data[BMA255_AXIS_X], data[BMA255_AXIS_Y], data[BMA255_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int BMA255_ReadOffset(struct i2c_client *client, s8 ofs[BMA255_AXES_NUM])
{    
	int err;
#ifdef SW_CALIBRATION
	ofs[0]=ofs[1]=ofs[2]=0x0;
#else
	err = bma_i2c_read_block(client, BMA255_REG_OFSX, ofs, BMA255_AXES_NUM);
	if(err)
	{
		GSE_ERR("error: %d\n", err);
	}
#endif
	//printk("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]);
	
	return err;    
}
/*----------------------------------------------------------------------------*/
static int BMA255_ResetCalibration(struct i2c_client *client)
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	u8 ofs[4]={0,0,0,0};
	int err;
	
	#ifdef SW_CALIBRATION
		
	#else
	err = bma_i2c_write_block(client, BMA255_REG_OFSX, ofs, 4);
		if(err)
		{
			GSE_ERR("error: %d\n", err);
		}
	#endif

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;    
}
/*----------------------------------------------------------------------------*/
static int BMA255_ReadCalibration(struct i2c_client *client, int dat[BMA255_AXES_NUM])
{
    struct bma255_i2c_data *obj = i2c_get_clientdata(client);
    int err = 0;
    int mul;
	
	GSE_LOG("BMA255_ReadCalibration start");
	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
		err = BMA255_ReadOffset(client, obj->offset);
	    if (err) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    	}    
    	mul = obj->reso->sensitivity/bma255_offset_resolution.sensitivity;
	#endif

    dat[obj->cvt.map[BMA255_AXIS_X]] = obj->cvt.sign[BMA255_AXIS_X]*(obj->offset[BMA255_AXIS_X]*mul + obj->cali_sw[BMA255_AXIS_X]);
    dat[obj->cvt.map[BMA255_AXIS_Y]] = obj->cvt.sign[BMA255_AXIS_Y]*(obj->offset[BMA255_AXIS_Y]*mul + obj->cali_sw[BMA255_AXIS_Y]);
    dat[obj->cvt.map[BMA255_AXIS_Z]] = obj->cvt.sign[BMA255_AXIS_Z]*(obj->offset[BMA255_AXIS_Z]*mul + obj->cali_sw[BMA255_AXIS_Z]);                        
                                       
    return err;
}
/*----------------------------------------------------------------------------*/
#if 0
static int BMA255_ReadCalibrationEx(struct i2c_client *client, int act[BMA255_AXES_NUM], int raw[BMA255_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;

	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
		err = BMA255_ReadOffset(client, obj->offset);
		if(err)
		{
			GSE_ERR("read offset fail, %d\n", err);
			return err;
		}   
		mul = obj->reso->sensitivity/bma255_offset_resolution.sensitivity;
	#endif
	
	raw[BMA255_AXIS_X] = obj->offset[BMA255_AXIS_X]*mul + obj->cali_sw[BMA255_AXIS_X];
	raw[BMA255_AXIS_Y] = obj->offset[BMA255_AXIS_Y]*mul + obj->cali_sw[BMA255_AXIS_Y];
	raw[BMA255_AXIS_Z] = obj->offset[BMA255_AXIS_Z]*mul + obj->cali_sw[BMA255_AXIS_Z];

	act[obj->cvt.map[BMA255_AXIS_X]] = obj->cvt.sign[BMA255_AXIS_X]*raw[BMA255_AXIS_X];
	act[obj->cvt.map[BMA255_AXIS_Y]] = obj->cvt.sign[BMA255_AXIS_Y]*raw[BMA255_AXIS_Y];
	act[obj->cvt.map[BMA255_AXIS_Z]] = obj->cvt.sign[BMA255_AXIS_Z]*raw[BMA255_AXIS_Z];                        
	                       
	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
#if 0
static int BMA255_WriteCalibration(struct i2c_client *client, int dat[BMA255_AXES_NUM])
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[BMA255_AXES_NUM], raw[BMA255_AXES_NUM];
	int lsb = bma255_offset_resolution.sensitivity;
	int divisor = obj->reso->sensitivity/lsb;
//	unsigned char backup_offset_x, backup_offset_y, backup_offset_z;
	err = BMA255_ReadCalibrationEx(client, cali, raw);
	if(err)	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		raw[BMA255_AXIS_X], raw[BMA255_AXIS_Y], raw[BMA255_AXIS_Z],
		obj->offset[BMA255_AXIS_X], obj->offset[BMA255_AXIS_Y], obj->offset[BMA255_AXIS_Z],
		obj->cali_sw[BMA255_AXIS_X], obj->cali_sw[BMA255_AXIS_Y], obj->cali_sw[BMA255_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[BMA255_AXIS_X] += dat[BMA255_AXIS_X];
	cali[BMA255_AXIS_Y] += dat[BMA255_AXIS_Y];
	cali[BMA255_AXIS_Z] += dat[BMA255_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
		dat[BMA255_AXIS_X], dat[BMA255_AXIS_Y], dat[BMA255_AXIS_Z]);

#ifdef SW_CALIBRATION
	//obj->cali_sw[BMA255_AXIS_X] = obj->cvt.sign[BMA255_AXIS_X]*(cali[obj->cvt.map[BMA255_AXIS_X]]);
	//obj->cali_sw[BMA255_AXIS_Y] = obj->cvt.sign[BMA255_AXIS_Y]*(cali[obj->cvt.map[BMA255_AXIS_Y]]);
	//obj->cali_sw[BMA255_AXIS_Z] = obj->cvt.sign[BMA255_AXIS_Z]*(cali[obj->cvt.map[BMA255_AXIS_Z]]);	
	
	obj->offset[BMA255_AXIS_X] = (s8)(obj->cvt.sign[BMA255_AXIS_X]*(cali[obj->cvt.map[BMA255_AXIS_X]])/(divisor));
	obj->offset[BMA255_AXIS_Y] = (s8)(obj->cvt.sign[BMA255_AXIS_Y]*(cali[obj->cvt.map[BMA255_AXIS_Y]])/(divisor));
	obj->offset[BMA255_AXIS_Z] = (s8)(obj->cvt.sign[BMA255_AXIS_Z]*(cali[obj->cvt.map[BMA255_AXIS_Z]])/(divisor));
#else
	obj->offset[BMA255_AXIS_X] = (s8)(obj->cvt.sign[BMA255_AXIS_X]*(cali[obj->cvt.map[BMA255_AXIS_X]])/(divisor));
	obj->offset[BMA255_AXIS_Y] = (s8)(obj->cvt.sign[BMA255_AXIS_Y]*(cali[obj->cvt.map[BMA255_AXIS_Y]])/(divisor));
	obj->offset[BMA255_AXIS_Z] = (s8)(obj->cvt.sign[BMA255_AXIS_Z]*(cali[obj->cvt.map[BMA255_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[BMA255_AXIS_X] = obj->cvt.sign[BMA255_AXIS_X]*(cali[obj->cvt.map[BMA255_AXIS_X]])%(divisor);
	obj->cali_sw[BMA255_AXIS_Y] = obj->cvt.sign[BMA255_AXIS_Y]*(cali[obj->cvt.map[BMA255_AXIS_Y]])%(divisor);
	obj->cali_sw[BMA255_AXIS_Z] = obj->cvt.sign[BMA255_AXIS_Z]*(cali[obj->cvt.map[BMA255_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		obj->offset[BMA255_AXIS_X]*divisor + obj->cali_sw[BMA255_AXIS_X], 
		obj->offset[BMA255_AXIS_Y]*divisor + obj->cali_sw[BMA255_AXIS_Y], 
		obj->offset[BMA255_AXIS_Z]*divisor + obj->cali_sw[BMA255_AXIS_Z], 
		obj->offset[BMA255_AXIS_X], obj->offset[BMA255_AXIS_Y], obj->offset[BMA255_AXIS_Z],
		obj->cali_sw[BMA255_AXIS_X], obj->cali_sw[BMA255_AXIS_Y], obj->cali_sw[BMA255_AXIS_Z]);

	err = bma_i2c_write_block(obj->client, BMA255_REG_OFSX, obj->offset, BMA255_AXES_NUM);

	if(err)
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif

	return err;
}
#endif

/*----------------------------------------------------------------------------*/
static int BMA255_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[2] = {0,0};
	int res = 0;

	memset(databuf, 0, sizeof(u8)*2);


	res = bma_i2c_read_block(client, BMA255_REG_DEVID, databuf, 0x01);
	res = bma_i2c_read_block(client, BMA255_REG_DEVID, databuf, 0x01);
	if(res < 0)
		goto exit_BMA255_CheckDeviceID;

	if(databuf[0]!=BMA255_FIXED_DEVID)
	{
		printk("BMA255_CheckDeviceID %d failt!\n ", databuf[0]);
		return BMA255_ERR_IDENTIFICATION;
	}
	else
	{
		printk("BMA255_CheckDeviceID %d pass!\n ", databuf[0]);
	}

	exit_BMA255_CheckDeviceID:
	if (res < 0)
	{
		return BMA255_ERR_I2C;
	}
	
	return BMA255_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int BMA255_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};    
	int res = 0;
//	u8 addr = BMA255_REG_POWER_CTL;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	u8 actual_power_mode = 0;
	
	if(enable == sensor_power )
	{
		GSE_LOG("Sensor power status is newest!\n");
		return BMA255_SUCCESS;
	}
// if slop_detect is enabled do nothing but update power state only
#ifdef BMA255_ENABLE_INT1
	if (atomic_read(&obj->slop_detect_enable) == TRUE)
	{
		sensor_power = enable;
		test_status = sensor_power;
		return BMA255_SUCCESS;
	}
#endif
	
	mutex_lock(&obj->lock);
	if(enable == true)
	{
		actual_power_mode = BMA255_MODE_NORMAL;
	}
	else
	{
		actual_power_mode = BMA255_MODE_SUSPEND;
	}
	
	res = bma_i2c_read_block(client,
			BMA255_MODE_CTRL_REG, &databuf[0], 1);
	res += bma_i2c_read_block(client,
		BMA255_LOW_POWER_CTRL_REG, &databuf[1], 1);

	switch (actual_power_mode) {
	case BMA255_MODE_NORMAL:
		databuf[0] = BMA255_SET_BITSLICE(databuf[0],
			BMA255_MODE_CTRL, 0);
		databuf[1] = BMA255_SET_BITSLICE(databuf[1],
			BMA255_LOW_POWER_MODE, 0);
		res += bma_i2c_write_block(client,
			BMA255_MODE_CTRL_REG, &databuf[0], 1);
		mdelay(1);
		res += bma_i2c_write_block(client,
			BMA255_LOW_POWER_CTRL_REG, &databuf[1], 1);
		mdelay(1);
	break;
	case BMA255_MODE_SUSPEND:
		databuf[0] = BMA255_SET_BITSLICE(databuf[0],
			BMA255_MODE_CTRL, 4);
		databuf[1] = BMA255_SET_BITSLICE(databuf[1],
			BMA255_LOW_POWER_MODE, 0);
		res += bma_i2c_write_block(client,
			BMA255_LOW_POWER_CTRL_REG, &databuf[1], 1);
		mdelay(1);
		res += bma_i2c_write_block(client,
			BMA255_MODE_CTRL_REG, &databuf[0], 1);
		mdelay(1);
	break;
	}

	if(res < 0)
	{
		GSE_ERR("set power mode failed, res = %d\n", res);
		mutex_unlock(&obj->lock);
		return BMA255_ERR_I2C;
	}
	sensor_power = enable;
	mutex_unlock(&obj->lock);
	
	return BMA255_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int BMA255_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[2] = {0};    
	int res = 0;

	mutex_lock(&obj->lock);
	res = bma_i2c_read_block(client,
		BMA255_RANGE_SEL_REG, &databuf[0], 1);
	databuf[0] = BMA255_SET_BITSLICE(databuf[0],
		BMA255_RANGE_SEL, dataformat);
	res += bma_i2c_write_block(client,
		BMA255_RANGE_SEL_REG, &databuf[0], 1);
	mdelay(1);

	if(res < 0)
	{
		GSE_ERR("set data format failed, res = %d\n", res);
		mutex_unlock(&obj->lock);
		return BMA255_ERR_I2C;
	}
	mutex_unlock(&obj->lock);
	
	return BMA255_SetDataResolution(obj);    
}
/*----------------------------------------------------------------------------*/
static int BMA255_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[2] = {0};    
	int res = 0;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);

	mutex_lock(&obj->lock);
	res = bma_i2c_read_block(client,
		BMA255_BANDWIDTH__REG, &databuf[0], 1);
	databuf[0] = BMA255_SET_BITSLICE(databuf[0],
		BMA255_BANDWIDTH, bwrate);
	res += bma_i2c_write_block(client,
		BMA255_BANDWIDTH__REG, &databuf[0], 1);
	mdelay(1);

	if(res < 0)
	{
		GSE_ERR("set bandwidth failed, res = %d\n", res);
		mutex_unlock(&obj->lock);
		return BMA255_ERR_I2C;
	}
	mutex_unlock(&obj->lock);

	return BMA255_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int BMA255_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	int res = 0;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	
	mutex_lock(&obj->lock);
	res = bma_i2c_write_block(client, BMA255_INT_REG_1, &intenable, 0x01);
	mdelay(1);
	if(res != BMA255_SUCCESS) 
	{
		mutex_unlock(&obj->lock);
		return res;
	}

	res = bma_i2c_write_block(client, BMA255_INT_REG_2, &intenable, 0x01);
	mdelay(1);
	if(res != BMA255_SUCCESS) 
	{
		mutex_unlock(&obj->lock);
		return res;
	}
	mutex_unlock(&obj->lock);
	GSE_LOG("BMA255 disable interrupt ...\n");

	/*for disable interrupt function*/

	return BMA255_SUCCESS;	  
}

/*----------------------------------------------------------------------------*/
static int bma255_init_client(struct i2c_client *client, int reset_cali)
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
	GSE_LOG("bma255_init_client \n");
	if (client == NULL)
	{
		GSE_LOG("bma255_init_client client null!\n");
		return res;
	}
	res = BMA255_CheckDeviceID(client); 
	if(res != BMA255_SUCCESS)
	{
		return res;
	}	
	GSE_LOG("BMA255_CheckDeviceID ok \n");
	
	res = BMA255_SetBWRate(client, BMA255_BW_100HZ);
	if(res != BMA255_SUCCESS ) 
	{
		return res;
	}
	GSE_LOG("BMA255_SetBWRate OK!\n");
	
	res = BMA255_SetDataFormat(client, BMA255_RANGE_4G);
	if(res != BMA255_SUCCESS) 
	{
		return res;
	}
	GSE_LOG("BMA255_SetDataFormat OK!\n");

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	res = BMA255_SetIntEnable(client, 0x00);        
	if(res != BMA255_SUCCESS)
	{
		return res;
	}
	GSE_LOG("BMA255 disable interrupt function!\n");

	res = BMA255_SetPowerMode(client, false);
	if(res != BMA255_SUCCESS)
	{
		return res;
	}
	GSE_LOG("BMA255_SetPowerMode OK!\n");

	if(0 != reset_cali)
	{ 
		/*reset calibration only in power on*/
		res = BMA255_ResetCalibration(client);
		if(res != BMA255_SUCCESS)
		{
			return res;
		}
	}
    GSE_LOG("bma255_init_client OK!\n");
#ifdef CONFIG_BMA255_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	mdelay(20);

	return BMA255_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int BMA255_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "BMA255 Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA255_CompassReadData(struct i2c_client *client, char *buf, int bufsize)
{
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);
	//u8 databuf[20];
	int acc[BMA255_AXES_NUM];
	int res = 0;
	s16 databuf[BMA255_AXES_NUM];
	//memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_power == false)
	{
		res = BMA255_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on bma255 error %d!\n", res);
		}
	}

	res = BMA255_ReadData(client, databuf);
	if(res)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		/* Add compensated value performed by MTK calibration process*/
		databuf[BMA255_AXIS_X] += obj->cali_sw[BMA255_AXIS_X];
		databuf[BMA255_AXIS_Y] += obj->cali_sw[BMA255_AXIS_Y];
		databuf[BMA255_AXIS_Z] += obj->cali_sw[BMA255_AXIS_Z];

		/*remap coordinate*/
		acc[obj->cvt.map[BMA255_AXIS_X]] = obj->cvt.sign[BMA255_AXIS_X]*databuf[BMA255_AXIS_X];
		acc[obj->cvt.map[BMA255_AXIS_Y]] = obj->cvt.sign[BMA255_AXIS_Y]*databuf[BMA255_AXIS_Y];
		acc[obj->cvt.map[BMA255_AXIS_Z]] = obj->cvt.sign[BMA255_AXIS_Z]*databuf[BMA255_AXIS_Z];
		//printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[BMA255_AXIS_X],obj->cvt.sign[BMA255_AXIS_Y],obj->cvt.sign[BMA255_AXIS_Z]);

		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[BMA255_AXIS_X], acc[BMA255_AXIS_Y], acc[BMA255_AXIS_Z]);

		sprintf(buf, "%d %d %d", (s16)acc[BMA255_AXIS_X], (s16)acc[BMA255_AXIS_Y], (s16)acc[BMA255_AXIS_Z]);
		if(atomic_read(&obj->trace) & BMA_TRC_IOCTL)
		{
			GSE_LOG("gsensor data for compass: %s!\n", buf);
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA255_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);
	//u8 databuf[20];
	int acc[BMA255_AXES_NUM];
	int res = 0;
	s16 databuf[BMA255_AXES_NUM];
	//memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_power == false)
	{
		res = BMA255_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on bma255 error %d!\n", res);
		}
	}
	res = BMA255_ReadData(client, databuf);
	if(res)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
	//	printk("raw data x=%d, y=%d, z=%d \n",obj->data[BMA255_AXIS_X],obj->data[BMA255_AXIS_Y],obj->data[BMA255_AXIS_Z]);
		databuf[BMA255_AXIS_X] += obj->cali_sw[BMA255_AXIS_X];
		databuf[BMA255_AXIS_Y] += obj->cali_sw[BMA255_AXIS_Y];
		databuf[BMA255_AXIS_Z] += obj->cali_sw[BMA255_AXIS_Z];
		
	//	printk("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[BMA255_AXIS_X],obj->cali_sw[BMA255_AXIS_Y],obj->cali_sw[BMA255_AXIS_Z]);
		
		/*remap coordinate*/
		acc[obj->cvt.map[BMA255_AXIS_X]] = obj->cvt.sign[BMA255_AXIS_X]*databuf[BMA255_AXIS_X];
		acc[obj->cvt.map[BMA255_AXIS_Y]] = obj->cvt.sign[BMA255_AXIS_Y]*databuf[BMA255_AXIS_Y];
		acc[obj->cvt.map[BMA255_AXIS_Z]] = obj->cvt.sign[BMA255_AXIS_Z]*databuf[BMA255_AXIS_Z];
	//	printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[BMA255_AXIS_X],obj->cvt.sign[BMA255_AXIS_Y],obj->cvt.sign[BMA255_AXIS_Z]);

//		GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[BMA255_AXIS_X], acc[BMA255_AXIS_Y], acc[BMA255_AXIS_Z]);

		//Out put the mg
		//printk("mg acc=%d, GRAVITY=%d, sensityvity=%d \n",acc[BMA255_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);
		acc[BMA255_AXIS_X] = acc[BMA255_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA255_AXIS_Y] = acc[BMA255_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA255_AXIS_Z] = acc[BMA255_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		

		sprintf(buf, "%04x %04x %04x", acc[BMA255_AXIS_X], acc[BMA255_AXIS_Y], acc[BMA255_AXIS_Z]);
		if(atomic_read(&obj->trace) & BMA_TRC_IOCTL)
		{
			GSE_LOG("gsensor data: %s!\n", buf);
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA255_ReadRawData(struct i2c_client *client, char *buf)
{
//	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);
	int res = 0;
	s16 databuf[BMA255_AXES_NUM];

	if (!buf || !client)
	{
		return EINVAL;
	}

	res = BMA255_ReadData(client, databuf);	
	if(res)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "BMA255_ReadRawData %04x %04x %04x", databuf[BMA255_AXIS_X], 
			databuf[BMA255_AXIS_Y], databuf[BMA255_AXIS_Z]);
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int bma255_set_mode(struct i2c_client *client, unsigned char mode)
{
	int comres = 0;
	unsigned char data[2] = {0};
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);

	if ((client == NULL) || (mode >= 3))
	{
		return -1;
	}
	mutex_lock(&obj->lock);
	comres = bma_i2c_read_block(client,
			BMA255_EN_LOW_POWER__REG, &data[0], 1);
	comres += bma_i2c_read_block(client,
		BMA255_LOW_POWER_CTRL_REG, &data[1], 1);
	switch (mode) {
	case BMA255_MODE_NORMAL:
		data[0]  = BMA255_SET_BITSLICE(data[0],
				BMA255_MODE_CTRL, 0);
		data[1]  = BMA255_SET_BITSLICE(data[1],
				BMA255_LOW_POWER_MODE, 0);
		comres += bma_i2c_write_block(client,
				BMA255_MODE_CTRL_REG, &data[0], 0x01);
		mdelay(1);
		comres += bma_i2c_write_block(client,
			BMA255_LOW_POWER_CTRL_REG, &data[1], 0x01);
		break;
	case BMA255_MODE_LOWPOWER:
		data[0]  = BMA255_SET_BITSLICE(data[0],
				BMA255_MODE_CTRL, 2);
		data[1]  = BMA255_SET_BITSLICE(data[1],
				BMA255_LOW_POWER_MODE, 0);
		comres += bma_i2c_write_block(client,
				BMA255_MODE_CTRL_REG, &data[0], 0x01);
		mdelay(1);
		comres += bma_i2c_write_block(client,
			BMA255_LOW_POWER_CTRL_REG, &data[1], 0x01);
		break;
	case BMA255_MODE_SUSPEND:
		data[0]  = BMA255_SET_BITSLICE(data[0],
				BMA255_MODE_CTRL, 4);
		data[1]  = BMA255_SET_BITSLICE(data[1],
				BMA255_LOW_POWER_MODE, 0);
		comres += bma_i2c_write_block(client,
			BMA255_LOW_POWER_CTRL_REG, &data[1], 0x01);
		mdelay(1);
		comres += bma_i2c_write_block(client,
			BMA255_MODE_CTRL_REG, &data[0], 0x01);
		break;
	default:
		break;
	}

	mutex_unlock(&obj->lock);

	if(comres <= 0)
	{
		return BMA255_ERR_I2C;
	}
	else
	{
		return comres;
	}
}
/*----------------------------------------------------------------------------*/
static int bma255_get_mode(struct i2c_client *client, unsigned char *mode)
{
	int comres = 0;

	if (client == NULL) 
	{
		return -1;
	}
	comres = bma_i2c_read_block(client,
			BMA255_EN_LOW_POWER__REG, mode, 1);
	*mode  = (*mode) >> 6;
		
	return comres;
}

/*----------------------------------------------------------------------------*/
static int bma255_set_range(struct i2c_client *client, unsigned char range)
{
	int comres = 0;
	unsigned char data[2] = {BMA255_RANGE_SEL__REG};
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);

	if (client == NULL)
	{
		return -1;
	}
	mutex_lock(&obj->lock);
	comres = bma_i2c_read_block(client,
			BMA255_RANGE_SEL__REG, data+1, 1);

	data[1]  = BMA255_SET_BITSLICE(data[1],
			BMA255_RANGE_SEL, range);

	comres = i2c_master_send(client, data, 2);
	mutex_unlock(&obj->lock);
	if(comres <= 0)
	{
		return BMA255_ERR_I2C;
	}
	else
	{
		return comres;
	}
}
/*----------------------------------------------------------------------------*/
static int bma255_get_range(struct i2c_client *client, unsigned char *range)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) 
	{
		return -1;
	}

	comres = bma_i2c_read_block(client, BMA255_RANGE_SEL__REG,	&data, 1);
	*range = BMA255_GET_BITSLICE(data, BMA255_RANGE_SEL);

	return comres;
}
/*----------------------------------------------------------------------------*/
static int bma255_set_bandwidth(struct i2c_client *client, unsigned char bandwidth)
{
	int comres = 0;
	unsigned char data[2] = {BMA255_BANDWIDTH__REG};
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);

	if (client == NULL)
	{
		return -1;
	}

	mutex_lock(&obj->lock);
	comres = bma_i2c_read_block(client,
			BMA255_BANDWIDTH__REG, data+1, 1);

	data[1]  = BMA255_SET_BITSLICE(data[1],
			BMA255_BANDWIDTH, bandwidth);

	comres = i2c_master_send(client, data, 2);
	mutex_unlock(&obj->lock);
	if(comres <= 0)
	{
		return BMA255_ERR_I2C;
	}
	else
	{
		return comres;
	}
}
/*----------------------------------------------------------------------------*/
static int bma255_get_bandwidth(struct i2c_client *client, unsigned char *bandwidth)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) 
	{
		return -1;
	}

	comres = bma_i2c_read_block(client, BMA255_BANDWIDTH__REG, &data, 1);
	data = BMA255_GET_BITSLICE(data, BMA255_BANDWIDTH);

	if (data < 0x08) //7.81Hz
	{
		*bandwidth = 0x08;
	}
	else if (data > 0x0f)	// 1000Hz
	{
		*bandwidth = 0x0f;
	}
	else
	{
		*bandwidth = data;
	}
	return comres;
}

/*----------------------------------------------------------------------------*/
static int bma255_set_fifo_mode(struct i2c_client *client, unsigned char fifo_mode)
{
	int comres = 0;
	unsigned char data[2] = {BMA255_FIFO_MODE__REG};
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);

	if (client == NULL || fifo_mode >= 4)
	{
		return -1;
	}

	mutex_lock(&obj->lock);
	comres = bma_i2c_read_block(client,
			BMA255_FIFO_MODE__REG, data+1, 1);

	data[1]  = BMA255_SET_BITSLICE(data[1],
			BMA255_FIFO_MODE, fifo_mode);

	comres = i2c_master_send(client, data, 2);
	mutex_unlock(&obj->lock);
	if(comres <= 0)
	{
		return BMA255_ERR_I2C;
	}
	else
	{
		return comres;
	}
}
/*----------------------------------------------------------------------------*/
static int bma255_get_fifo_mode(struct i2c_client *client, unsigned char *fifo_mode)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) 
	{
		return -1;
	}

	comres = bma_i2c_read_block(client, BMA255_FIFO_MODE__REG, &data, 1);
	*fifo_mode = BMA255_GET_BITSLICE(data, BMA255_FIFO_MODE);

	return comres;
}

static int bma255_get_fifo_framecount(struct i2c_client *client, unsigned char *framecount)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) 
	{
		return -1;
	}

	comres = bma_i2c_read_block(client, BMA255_FIFO_FRAME_COUNTER_S__REG, &data, 1);
	*framecount = BMA255_GET_BITSLICE(data, BMA255_FIFO_FRAME_COUNTER_S);
	return comres;
}

//tad3sgh add++
// Daemon application save the data
static int ECS_SaveData(int buf[CALIBRATION_DATA_SIZE])
{
#if DEBUG	
	struct bma255_i2c_data *data = i2c_get_clientdata(bma255_i2c_client);
#endif

	mutex_lock(&sensor_data_mutex);
	switch (buf[0])
	{
	case 2:	/* SENSOR_HANDLE_MAGNETIC_FIELD */
		memcpy(sensor_data+4, buf+1, 4*sizeof(int));	
		break;
	case 3:	/* SENSOR_HANDLE_ORIENTATION */
		memcpy(sensor_data+8, buf+1, 4*sizeof(int));	
		break;
#ifdef BMC050_M4G
	case 4:	/* SENSOR_HANDLE_GYROSCOPE */
		memcpy(m4g_data, buf+1, 4*sizeof(int));
		break;
#endif //BMC050_M4G
#ifdef BMC050_VRV
	case 11:	/* SENSOR_HANDLE_ROTATION_VECTOR */
		memcpy(m4g_data+4, buf+1, 4*sizeof(int));
		break;
#endif //BMC050_VRV
#ifdef BMC050_VLA
	case 10: /* SENSOR_HANDLE_LINEAR_ACCELERATION */
		memcpy(vla_data, buf+1, 4*sizeof(int));
		break;
#endif //BMC050_VLA
#ifdef BMC050_VG
	case 9: /* SENSOR_HANDLE_GRAVITY */
		memcpy(vg_data, buf+1, 4*sizeof(int));
		break;
#endif //BMC050_VG
	default:
		break;
	}
	mutex_unlock(&sensor_data_mutex);
	
#if DEBUG
	if(atomic_read(&data->trace) & BMA_TRC_INFO)
	{
		GSE_LOG("Get daemon data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],
			sensor_data[4],sensor_data[5],sensor_data[6],sensor_data[7],
			sensor_data[8],sensor_data[9],sensor_data[10],sensor_data[11]);
#if defined(BMC050_M4G) || defined(BMC050_VRV)
		GSE_LOG("Get m4g data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			m4g_data[0],m4g_data[1],m4g_data[2],m4g_data[3],
			m4g_data[4],m4g_data[5],m4g_data[6],m4g_data[7],
			m4g_data[8],m4g_data[9],m4g_data[10],m4g_data[11]);
#endif //BMC050_M4G || BMC050_VRV
#if defined(BMC050_VLA)
		GSE_LOG("Get vla data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			vla_data[0],vla_data[1],vla_data[2],vla_data[3],
			vla_data[4],vla_data[5],vla_data[6],vla_data[7],
			vla_data[8],vla_data[9],vla_data[10],vla_data[11]);
#endif //BMC050_VLA

#if defined(BMC050_VG)
		GSE_LOG("Get vg data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			vg_data[0],vg_data[1],vg_data[2],vg_data[3],
			vg_data[4],vg_data[5],vg_data[6],vg_data[7],
			vg_data[8],vg_data[9],vg_data[10],vg_data[11]);
#endif //BMC050_VG
	}	
#endif

	return 0;
}

//tad3sgh add--
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
#if 1//def BMA255_ACCEL_CALIBRATION
static int bma255_set_offset_target(struct i2c_client *client, unsigned char channel, unsigned char offset)
{
	unsigned char data;
	int comres = 0;

	switch (channel)
	{
		case BMA255_CUT_OFF:
			comres = bma_i2c_read_block(client, BMA255_COMP_CUTOFF__REG, &data, 1);
			data = BMA255_SET_BITSLICE(data, BMA255_COMP_CUTOFF, offset);
			comres = bma_i2c_write_block(client, BMA255_COMP_CUTOFF__REG, &data, 1);
			break;

		case BMA255_OFFSET_TRIGGER_X:
			comres = bma_i2c_read_block(client, BMA255_COMP_TARGET_OFFSET_X__REG, &data, 1);
			data = BMA255_SET_BITSLICE(data, BMA255_COMP_TARGET_OFFSET_X, offset);
			comres = bma_i2c_write_block(client, BMA255_COMP_TARGET_OFFSET_X__REG, &data, 1);
			break;

		case BMA255_OFFSET_TRIGGER_Y:
			comres = bma_i2c_read_block(client, BMA255_COMP_TARGET_OFFSET_Y__REG, &data, 1);
			data = BMA255_SET_BITSLICE(data, BMA255_COMP_TARGET_OFFSET_Y, offset);
			comres = bma_i2c_write_block(client, BMA255_COMP_TARGET_OFFSET_Y__REG, &data, 1);
			break;

		case BMA255_OFFSET_TRIGGER_Z:
			comres = bma_i2c_read_block(client, BMA255_COMP_TARGET_OFFSET_Z__REG, &data, 1);
			data = BMA255_SET_BITSLICE(data, BMA255_COMP_TARGET_OFFSET_Z, offset);
			comres = bma_i2c_write_block(client, BMA255_COMP_TARGET_OFFSET_Z__REG, &data, 1);
			break;

		default:
			comres = -1;
			break;
	}

	return comres;
}

static int bma255_get_cal_ready(struct i2c_client *client, unsigned char *calrdy)
{
	int comres = 0 ;
	unsigned char data;

	comres = bma_i2c_read_block(client, BMA255_FAST_CAL_RDY_S__REG, &data, 1);
	data = BMA255_GET_BITSLICE(data, BMA255_FAST_CAL_RDY_S);
	*calrdy = data;

	return comres;
}

static int bma255_set_cal_trigger(struct i2c_client *client, unsigned char caltrigger)
{
	int comres = 0;
	unsigned char data;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	if(atomic_read(&bma255->fast_calib_rslt) != 0)
	{
		atomic_set(&bma255->fast_calib_rslt, 0);
		GSE_LOG(KERN_INFO "[set] bma2X2->fast_calib_rslt:%d\n",atomic_read(&bma255->fast_calib_rslt));
	}

	comres = bma_i2c_read_block(client, BMA255_CAL_TRIGGER__REG, &data, 1);
	data = BMA255_SET_BITSLICE(data, BMA255_CAL_TRIGGER, caltrigger);
	comres = bma_i2c_write_block(client, BMA255_CAL_TRIGGER__REG, &data, 1);

	return comres;
}

static int bma255_set_offset_x(struct i2c_client *client, unsigned char offsetfilt)
{
	int comres = 0;
	unsigned char data;

	data =  offsetfilt;
	comres = bma_i2c_write_block(client, BMA255_OFFSET_X_AXIS_REG, &data, 1);

	return comres;
}

static int bma255_get_offset_x(struct i2c_client *client, unsigned char *offsetfilt)
{
	int comres = 0;
	unsigned char data;

	comres = bma_i2c_read_block(client, BMA255_OFFSET_X_AXIS_REG, &data, 1);
	*offsetfilt = data;

	return comres;
}

static int bma255_set_offset_y(struct i2c_client *client, unsigned char offsetfilt)
{
	int comres = 0;
	unsigned char data;

	data =  offsetfilt;
	comres = bma_i2c_write_block(client, BMA255_OFFSET_Y_AXIS_REG, &data, 1);

	return comres;
}

static int bma255_get_offset_y(struct i2c_client *client, unsigned char *offsetfilt)
{
	int comres = 0;
	unsigned char data;

	comres = bma_i2c_read_block(client, BMA255_OFFSET_Y_AXIS_REG, &data, 1);
	*offsetfilt = data;

	return comres;
}

static int bma255_set_offset_z(struct i2c_client *client, unsigned char offsetfilt)
{
	int comres = 0;
	unsigned char data;

	data =  offsetfilt;
	comres = bma_i2c_write_block(client, BMA255_OFFSET_Z_AXIS_REG, &data, 1);

	return comres;
}

static int bma255_get_offset_z(struct i2c_client *client, unsigned char *offsetfilt)
{
	int comres = 0;
	unsigned char data;

	comres = bma_i2c_read_block(client, BMA255_OFFSET_Z_AXIS_REG, &data, 1);
	*offsetfilt = data;

	return comres;
}

static int bma255_read_accel_xyz(struct i2c_client *client, struct bma255acc *acc)
{
	int comres = 0;
	unsigned char data[6];

	comres = bma_i2c_read_block(client, BMA255_ACC_X_LSB__REG, data, 6);

	acc->x = BMA255_GET_BITSLICE(data[0], BMA255_ACC_X_LSB) | (BMA255_GET_BITSLICE(data[1], BMA255_ACC_X_MSB)<<(BMA255_ACC_X_LSB__LEN));
	acc->x = acc->x << (sizeof(short)*8-(BMA255_ACC_X_LSB__LEN + BMA255_ACC_X_MSB__LEN));
	acc->x = acc->x >> (sizeof(short)*8-(BMA255_ACC_X_LSB__LEN + BMA255_ACC_X_MSB__LEN));

	acc->y = BMA255_GET_BITSLICE(data[2], BMA255_ACC_Y_LSB) | (BMA255_GET_BITSLICE(data[3], BMA255_ACC_Y_MSB)<<(BMA255_ACC_Y_LSB__LEN));
	acc->y = acc->y << (sizeof(short)*8-(BMA255_ACC_Y_LSB__LEN + BMA255_ACC_Y_MSB__LEN));
	acc->y = acc->y >> (sizeof(short)*8-(BMA255_ACC_Y_LSB__LEN + BMA255_ACC_Y_MSB__LEN));

	acc->z = BMA255_GET_BITSLICE(data[4], BMA255_ACC_Z_LSB) | (BMA255_GET_BITSLICE(data[5], BMA255_ACC_Z_MSB)<<(BMA255_ACC_Z_LSB__LEN));
	acc->z = acc->z << (sizeof(short)*8-(BMA255_ACC_Z_LSB__LEN + BMA255_ACC_Z_MSB__LEN));
	acc->z = acc->z >> (sizeof(short)*8-(BMA255_ACC_Z_LSB__LEN + BMA255_ACC_Z_MSB__LEN));

	return comres;
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
    struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);
	unsigned char offset_x,offset_y,offset_z;

	if(bma255_get_offset_x(bma255->client, &offset_x) < 0)
		return -EINVAL;
	if(bma255_get_offset_y(bma255->client, &offset_y) < 0)
		return -EINVAL;
	if(bma255_get_offset_z(bma255->client, &offset_z) < 0)
		return -EINVAL;

    GSE_LOG("offset_x: %d, offset_y: %d, offset_z: %d\n",offset_x,offset_y,offset_z);

	return snprintf(buf, PAGE_SIZE, "%d %d %d \n", (unsigned int)offset_x, (unsigned int)offset_y, (unsigned int)offset_z);
}


static ssize_t show_cali_backup_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
    struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);
	unsigned char offset_x,offset_y,offset_z;
	int cal_backup[6]={0,};
	//int i;

#if defined(CALIBRATION_TO_FILE)
   if(sensor_calibration_read(ID_ACCELEROMETER, cal_backup + 3) == 0)
		GSE_LOG("previous offset_x: %d,  offset_y: %d,  offset_z: %d\n",
				cal_backup[3],cal_backup[4],cal_backup[5]);
   else
        GSE_ERR("Fail to read previous gsensor cal value from Cal File\n");
#else
	if(LGE_FacReadAccelerometerCalibration((unsigned int*)cal_backup) == TRUE)
		GSE_LOG("Read from LGPServer gsensor previous offset_x: %d, offset_y: %d, offset_z: %d\n",
				cal_backup[3],cal_backup[4],cal_backup[5]);
	else
        GSE_ERR("Fail to read previous gsensor cal value from LGPserver\n");
#endif

	if(bma255_get_offset_x(bma255->client, &offset_x) < 0)
		return -EINVAL;
	if(bma255_get_offset_y(bma255->client, &offset_y) < 0)
		return -EINVAL;
	if(bma255_get_offset_z(bma255->client, &offset_z) < 0)
		return -EINVAL;

    GSE_LOG("Current offset_x: %d, offset_y: %d, offset_z: %d\n",
			offset_x,offset_y,offset_z);

	return snprintf(buf, PAGE_SIZE, "Old %d %d %d   Now %d %d %d \n",
			(unsigned int)cal_backup[3], (unsigned int)cal_backup[4],
			(unsigned int)cal_backup[5], (unsigned int)offset_x,
			(unsigned int)offset_y, (unsigned int)offset_z);
}

static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);
	int err;
	int offset_x,offset_y,offset_z;
	unsigned char offsets[3];
	//int dat[BMA255_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		err = BMA255_ResetCalibration(client);
		if(err)
		{
			GSE_ERR("reset offset err = %d\n", err);
		}
	}
	else if(3 == sscanf(buf, "%d %d %d", &offset_x, &offset_y, &offset_z))
	{
		GSE_LOG("store_cali_value: x=%d, y=%d, z=%d\n", offset_x, offset_y, offset_z);
		offsets[0] = (unsigned char)offset_x;
		offsets[1] = (unsigned char)offset_y;
		offsets[2] = (unsigned char)offset_z;
		if(bma255_set_offset_x(bma255->client, (unsigned char)offsets[0]) < 0)
			return -EINVAL;
		if(bma255_set_offset_y(bma255->client, (unsigned char)offsets[1]) < 0)
			return -EINVAL;
		if(bma255_set_offset_z(bma255->client, (unsigned char)offsets[2]) < 0)
			return -EINVAL;
		GSE_LOG("store_cali_value success\n");
	}
	else
	{
		GSE_ERR("invalid format\n");
	}

	return count;
}

static ssize_t bma255_fast_calibration_x_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma255->fast_calib_x_rslt));
}

static int bma255_do_calibration(void);

static ssize_t bma255_fast_calibration_x_store(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client =bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);
	test_status = 4;	// calibration status

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	atomic_set(&bma255->fast_calib_x_rslt, 0);

    if(bma255_do_calibration() != BMA255_SUCCESS)
    {
    	atomic_set(&bma255->fast_calib_x_rslt, 0);
    	return -EINVAL;
    }

	if (bma255_set_offset_target(bma255->client, 1, (unsigned char)data) < 0)
		return -EINVAL;

	if (bma255_set_cal_trigger(bma255->client, 1) < 0)
		return -EINVAL;

	do{
		mdelay(2);
		bma255_get_cal_ready(bma255->client, &tmp);

		GSE_LOG("x wait 2ms cal ready flag is %d\n", tmp);

		timeout++;
		if (timeout == 50)
		{
			GSE_ERR("get fast calibration ready error\n");
			return -EINVAL;
		}
	} while (tmp == 0);

	atomic_set(&bma255->fast_calib_x_rslt, 1);
	GSE_LOG("x axis fast calibration finished\n");

	return count;
}

static ssize_t bma255_fast_calibration_y_show(struct device_driver *ddri, char *buf)
{

	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma255->fast_calib_y_rslt));
}

static ssize_t bma255_fast_calibration_y_store(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	atomic_set(&bma255->fast_calib_y_rslt, 0);

	if (bma255_set_offset_target(bma255->client, 2, (unsigned char)data) < 0)
		return -EINVAL;

	if (bma255_set_cal_trigger(bma255->client, 2) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma255_get_cal_ready(bma255->client, &tmp);

		GSE_LOG("y wait 2ms cal ready flag is %d\n", tmp);

		timeout++;
		if (timeout == 50)
		{
			GSE_ERR("get fast calibration ready error\n");
			return -EINVAL;
		}
	} while (tmp == 0);

	atomic_set(&bma255->fast_calib_y_rslt, 1);
	GSE_LOG("y axis fast calibration finished\n");

	return count;
}

static ssize_t bma255_fast_calibration_z_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma255->fast_calib_z_rslt));
}

static ssize_t bma255_fast_calibration_z_store(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	atomic_set(&bma255->fast_calib_z_rslt, 0);

	if (bma255_set_offset_target(bma255->client, 3, (unsigned char)data) < 0)
		return -EINVAL;

	if (bma255_set_cal_trigger(bma255->client, 3) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma255_get_cal_ready(bma255->client, &tmp);

		GSE_LOG(" z wait 2ms cal ready flag is %d\n", tmp);

		timeout++;
		if (timeout == 50)
		{
			GSE_ERR("get fast calibration ready error\n");
			return -EINVAL;
		}
	} while (tmp == 0);

	atomic_set(&bma255->fast_calib_z_rslt, 1);
	GSE_LOG("z axis fast calibration finished\n");

	test_status = sensor_power;

	return count;
}

/* LGE_BSP_COMMON LGE_CHANGE_S 140228 : Calibration for user Apps */
static int bma255_runCalibration(void)
{
	//signed char tmp;
	//unsigned char timeout = 0;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);
	unsigned char backup_offset_x, backup_offset_y, backup_offset_z;
	int res = 0;
	int res2 = 0;
	#if 0 //motion sensor cal backup
	int cali[3];
	#else
	int cali[6];  
	#endif

	GSE_LOG("<============= bma255_runCalibration start =============>\n");

	GSE_FUN();
	if(bma255_get_offset_x(bma255->client, &backup_offset_x) < 0)
		return FALSE;
	if(bma255_get_offset_y(bma255->client, &backup_offset_y) < 0)
		return FALSE;
	if(bma255_get_offset_z(bma255->client, &backup_offset_z) < 0)
		return FALSE;

	GSE_LOG("backup_offset_x: %d, backup_offset_y: %d, backup_offset_z: %d\n",
			backup_offset_x,backup_offset_y,backup_offset_z);

	if(sensor_power == FALSE)
	{
		res = BMA255_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on bma255 error %d!\n", res);
			return FALSE;
		}
	}

	res2= BMA255_SetDataFormat(client, BMA255_RANGE_2G);
	if (res2 < 0)//change range before calibration
	{
		GSE_ERR("SetDataFormat 2G error");
	}
	res = bma255_do_calibration();

	if(SHAKING_ZERO_G_ERROR==res)
	{
		GSE_ERR("Restart do_calibration()");
		mdelay(1000);
		res = bma255_do_calibration();
	}
	if(res != BMA255_SUCCESS)
	{
	    res2 = BMA255_SetDataFormat(client, BMA255_RANGE_4G);             
		if(bma255_set_offset_x(bma255->client, (unsigned char)backup_offset_x) < 0)
			return -EINVAL;
		if(bma255_set_offset_y(bma255->client, (unsigned char)backup_offset_y) < 0)
			return -EINVAL;
		if(bma255_set_offset_z(bma255->client, (unsigned char)backup_offset_z) < 0)
			return -EINVAL;
		GSE_LOG("Recovery backup cal value: x=%d, y=%d, z=%d\n",
				backup_offset_x, backup_offset_y, backup_offset_z);

		if(res==BMA255_ERR_SETUP_FAILURE)
			return BMA255_ERR_SETUP_FAILURE;
		else
			return BMA255_ERR_STATUS;
    }
	else
	{
		cali[3] = (int)backup_offset_x;
		cali[4] = (int)backup_offset_y;
		cali[5] = (int)backup_offset_z;
	
        bma255_get_offset_x(bma255->client, &backup_offset_x);
        bma255_get_offset_y(bma255->client, &backup_offset_y);
        bma255_get_offset_z(bma255->client, &backup_offset_z);
        GSE_LOG("new_offset_x: %d, new_offset_y: %d, new_offset_z: %d\n",
				(int)backup_offset_x,(int)backup_offset_y,(int)backup_offset_z);
		cali[0] = (int)backup_offset_x;
		cali[1] = (int)backup_offset_y;
		cali[2] = (int)backup_offset_z;
        #if defined(CALIBRATION_TO_FILE)
         sensor_calibration_save(ID_ACCELEROMETER, cali);
        #else
		 if(LGE_FacWriteAccelerometerCalibration((unsigned int*)cali) == TRUE)
		 {
		 	atomic_set(&bma255->fast_calib_rslt, 1);
		 	GSE_LOG("Calibration factory write END\n");
		 }
	#endif
	}

	res2 = BMA255_SetDataFormat(client, BMA255_RANGE_4G);
	if (res2 < 0)//change range before calibration
	{
		GSE_ERR("SetDataFormat 4G error");
	}

	return BMA255_SUCCESS;
}
/* LGE_BSP_COMMON LGE_CHANGE_E 140228 : Calibration for user Apps */

static int bma255_do_calibration(void)
{
	signed char tmp;
	unsigned char timeout = 0;
	unsigned int timeout_shaking = 0;
	unsigned int shaking_cnt = 0;
	int sum[3] = {0, };
	int err = 0;
	struct i2c_client *client =bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);
	struct bma255acc acc_cal;
	struct bma255acc acc_cal_pre;

	GSE_FUN();
	test_status = 4;	// calibration status
	/* set axis off set to zero */
	if(bma255_set_offset_x(bma255->client, (unsigned char)0) < 0)
		return -EINVAL;
	if(bma255_set_offset_y(bma255->client, (unsigned char)0) < 0)
		return -EINVAL;
	if(bma255_set_offset_z(bma255->client, (unsigned char)0) < 0)
		return -EINVAL;
	/* set axis off set to zero */

	mdelay(100);

	bma255_read_accel_xyz(bma255->client, &acc_cal_pre);
	do {
		mdelay(20);
		bma255_read_accel_xyz(bma255->client, &acc_cal);

		GSE_LOG("===============moved x=============== timeout = %d\n",
		timeout_shaking);
		GSE_LOG("(%d, %d, %d) (%d, %d, %d)\n",
		acc_cal_pre.x, acc_cal_pre.y,
		acc_cal_pre.z, acc_cal.x,
		acc_cal.y, acc_cal.z);

		if((abs(acc_cal.x - acc_cal_pre.x) > BMA255_SHAKING_DETECT_THRESHOLD_XY)
			|| (abs((acc_cal.y - acc_cal_pre.y)) > BMA255_SHAKING_DETECT_THRESHOLD_XY)
			|| (abs((acc_cal.z - acc_cal_pre.z)) > BMA255_SHAKING_DETECT_THRESHOLD_Z))
		{
			GSE_LOG("(threshold:%d,%d,%d):(%d, %d, %d) (%d, %d, %d)\n",
				BMA255_SHAKING_DETECT_THRESHOLD_XY, BMA255_SHAKING_DETECT_THRESHOLD_XY,BMA255_SHAKING_DETECT_THRESHOLD_Z,
				acc_cal_pre.x,acc_cal_pre.y,acc_cal_pre.z, acc_cal.x,acc_cal.y,acc_cal.z );
			shaking_cnt++;
		}
		else
		{
         /* Calibration zero-g offset check */
            sum[BMA255_AXIS_X] += acc_cal.x;
            sum[BMA255_AXIS_Y] += acc_cal.y;
            sum[BMA255_AXIS_Z] += acc_cal.z;
         /* Calibration zero-g offset check */

			acc_cal_pre.x = acc_cal.x;
			acc_cal_pre.y = acc_cal.y;
			acc_cal_pre.z = acc_cal.z;
		}

		if(shaking_cnt > 2){
			atomic_set(&bma255->fast_calib_rslt, 0);
			GSE_ERR("===============shaking error : %d ===============\n", shaking_cnt);
			return SHAKING_ZERO_G_ERROR;
		}
		timeout_shaking++;
		GSE_LOG("===============timeout_shaking: %d=============== \n",timeout_shaking);
	} while(timeout_shaking < 10);

	GSE_LOG("===============complete shaking check===============\n");
	/* Calibration zero-g offset check */
	// check zero-g offset
	GSE_LOG("(LIMIT_XY:%d)(LIMIT_Z_USL_LSB:%d)(LIMIT_Z_LSL_LSB:%d)\n",TESTLIMIT_XY, TESTLIMIT_Z_USL_LSB, TESTLIMIT_Z_LSL_LSB );
	GSE_ERR("sum(%d, %d, %d)\n", sum[BMA255_AXIS_X]/CALIBRATION_DATA_AMOUNT,
			sum[BMA255_AXIS_Y]/CALIBRATION_DATA_AMOUNT, sum[BMA255_AXIS_Z]/CALIBRATION_DATA_AMOUNT);
	if((abs(sum[BMA255_AXIS_X]/CALIBRATION_DATA_AMOUNT) >TESTLIMIT_XY)
		|| (abs(sum[BMA255_AXIS_Y]/CALIBRATION_DATA_AMOUNT) >TESTLIMIT_XY)
		|| ((abs(sum[BMA255_AXIS_Z]/CALIBRATION_DATA_AMOUNT) > TESTLIMIT_Z_USL_LSB)
		|| (abs(sum[BMA255_AXIS_Z]/CALIBRATION_DATA_AMOUNT) < TESTLIMIT_Z_LSL_LSB))) {
		GSE_ERR("Calibration zero-g offset check failed (%d, %d, %d)\n",
		sum[BMA255_AXIS_X]/CALIBRATION_DATA_AMOUNT,
		sum[BMA255_AXIS_Y]/CALIBRATION_DATA_AMOUNT,
		sum[BMA255_AXIS_Z]/CALIBRATION_DATA_AMOUNT);
		atomic_set(&bma255->fast_calib_rslt, 0);
		return SHAKING_ZERO_G_ERROR;
	}

	GSE_LOG("===============complete zero-g check===============\n");

	atomic_set(&bma255->fast_calib_x_rslt, 0);
	if (bma255_set_offset_target(bma255->client, 1, (unsigned char)0) < 0)
		return -EINVAL;
	if (bma255_set_cal_trigger(bma255->client, 1) < 0)
		return -EINVAL;
	do {
		mdelay(2);
		bma255_get_cal_ready(bma255->client, &tmp);

		GSE_LOG("x wait 2ms cal ready flag is %d\n", tmp);

		timeout++;
		if (timeout == 50) {
			GSE_ERR("get fast calibration ready error\n");
			return -EINVAL;
		}
	} while (tmp == 0);
	atomic_set(&bma255->fast_calib_x_rslt, 1);
	GSE_LOG("===============x axis fast calibration finished\n");

	atomic_set(&bma255->fast_calib_y_rslt, 0);
	if (bma255_set_offset_target(bma255->client, 2, (unsigned char)0) < 0)
		return -EINVAL;
	if (bma255_set_cal_trigger(bma255->client, 2) < 0)
		return -EINVAL;
	do {
		mdelay(2);
		bma255_get_cal_ready(bma255->client, &tmp);

		GSE_LOG("y wait 2ms cal ready flag is %d\n", tmp);

		timeout++;
		if (timeout == 50) {
			GSE_ERR("get fast calibration ready error\n");
			return -EINVAL;
		}
	} while (tmp == 0);
	atomic_set(&bma255->fast_calib_y_rslt, 1);
	GSE_LOG("===============y axis fast calibration finished\n");

	atomic_set(&bma255->fast_calib_z_rslt, 0);
	if (bma255_set_offset_target(bma255->client, 3,
		(unsigned char)GET_CALIBRATION_Z_FROM_PLACE(bma255->hw->direction)) < 0)
	return -EINVAL;
	if (bma255_set_cal_trigger(bma255->client, 3) < 0)
		return -EINVAL;
	do {
		mdelay(2);
		bma255_get_cal_ready(bma255->client, &tmp);

		GSE_LOG(" z wait 2ms cal ready flag is %d\n", tmp);

		timeout++;
		if (timeout == 50) {
			GSE_ERR("get fast calibration ready error\n");
			return -EINVAL;
		}
	} while (tmp == 0);
	atomic_set(&bma255->fast_calib_z_rslt, 1);
	GSE_LOG("===============z axis fast calibration finished\n");

	test_status = sensor_power;

	return err;
}

/*----------------------------------------------------------------------------*/
/* LGE_BSP_COMMON LGE_CHANGE_S 140228 : Calibration for user Apps */
static int selfCalibration = 1;
static ssize_t bma255_runCalibration_store(struct device_driver *dev, const char *buf, size_t count)
{
	int res = 0;

	GSE_LOG("<============= bma255_runCalibration_store now start =============>\n");
	res = bma255_runCalibration();
	if(res == BMA255_SUCCESS)
	{
		GSE_LOG("self calibration BMA255_SUCCESS\n");
		selfCalibration = BMA255_SUCCESS;
	}
	else if(res == BMA255_ERR_SETUP_FAILURE)
	{
		GSE_ERR("self calibration BMA255_ERR_SETUP_FAILURE\n");
		selfCalibration = 2;  // cal fail(flat)
	}
	else
	{
		GSE_LOG("self calibration FAIL\n");
		selfCalibration = 1;  // cal fail
	}
	GSE_LOG("<============= bma255_runCalibration_store now done =============>\n");
	return count;
}

static ssize_t bma255_runCalibration_show(struct device_driver *ddri, char *buf)
{
	GSE_LOG("calibration result show: %d\n", selfCalibration);
	return sprintf(buf, "%d\n", selfCalibration);
}

/*----------------------------------------------------------------------------*/
static ssize_t bma255_eeprom_writing_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma255->fast_calib_rslt));
}

static ssize_t bma255_eeprom_writing_store(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned char offset_x, offset_y, offset_z;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	if(bma255_get_offset_x(bma255->client, &offset_x) < 0)
		return -EINVAL;
	if(bma255_get_offset_y(bma255->client, &offset_y) < 0)
		return -EINVAL;
	if(bma255_get_offset_z(bma255->client, &offset_z) < 0)
		return -EINVAL;

	atomic_set(&bma255->fast_calib_rslt, 1);

	return count;
}

#endif


static int bma255_soft_reset(struct i2c_client *client)
{
	int comres = 0;
	unsigned char data = BMA255_EN_SOFT_RESET_VALUE;

	comres = bma_i2c_write_block(client, BMA255_EN_SOFT_RESET__REG, &data, 1);

	return comres;
}
static ssize_t bma255_softreset_store(struct device_driver *dev, const char *buf, size_t count)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	if (bma255_soft_reset(bma255->client) < 0)
		return -EINVAL;

	return count;
}


/*----------------------------------------------------------------------------*/
/* for self test */
static int bma255_set_selftest_st(struct i2c_client *client, unsigned char
		selftest)
{
	int comres = 0;
	unsigned char data;

	comres = bma_i2c_read_block(client, BMA255_EN_SELF_TEST__REG,
			&data, 1);
	data = BMA255_SET_BITSLICE(data, BMA255_EN_SELF_TEST, selftest);
	comres = bma_i2c_write_block(client, BMA255_EN_SELF_TEST__REG,
			&data, 1);
	//comres = i2c_master_send(client, &data, 1);
	GSE_LOG("selftest_st comres : %d\n",comres);
	return comres;
}

static int bma255_set_selftest_stn(struct i2c_client *client, unsigned char stn)
{
	int comres = 0;
	unsigned char data;

	comres = bma_i2c_read_block(client, BMA255_NEG_SELF_TEST__REG,
			&data, 1);
	data = BMA255_SET_BITSLICE(data, BMA255_NEG_SELF_TEST, stn);
	comres = bma_i2c_write_block(client, BMA255_NEG_SELF_TEST__REG,
			&data, 1);
	//comres = i2c_master_send(client, &data, 1);
	GSE_LOG("selftest_stN comres : %d\n",comres);
	return comres;
}

/*
Read:
0 ------ success
1 ------ x  axis failed
2 ------ y  axis failed
3 ------ x and y axis failed
4 ------ z  axis failed
5 ------ x and z axis failed
6 ------ y and z axis failed
7 ------ x, y and z axis failed
*/
static ssize_t bma255_SelfTest_store(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data = 0;
    int error = -1;
	int res = 0;
	struct i2c_client *client = bma255_i2c_client;
    struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	enum BMA_RANGE_ENUM range;
	bool power_mode;
	u8 bandwidth = 0;
	u8 value = 0;
	u8 amp = 0;
	s16 value1[BMA255_AXES_NUM];
	s16 value2[BMA255_AXES_NUM];
	s16 diff = 0;
	u8 result = 0;

    error = kstrtoul(buf, 10, &data);
    if (error)
    {
	    return error;
    }
    GSE_LOG("Self test CMD value : %d\n", (int)data);

    if(data == 1)// self test start command
    {
	    /*  power on!  */		
	    power_mode = sensor_power;
	    
		if(sensor_power == FALSE)
	    {
		    res = BMA255_SetPowerMode(client, true);
		    if(res)
		    {
			    GSE_ERR("Power on bma255 error %d!\n", res);
			    return FALSE;
		    }
	    }

	    /*backup settings*/
	    range = obj->range;
	    //power_mode = sensor_power;
	    bandwidth = obj->bandwidth;
	    /*Step 1:Soft Reset*/
		bma255_soft_reset(obj->client);
		/*Step 2:Clear Selftest Register*/
		{
			value = 0;
			bma_i2c_write_block(obj->client, BMA255_SELF_TEST_REG, &value, 1);
		}

		/*Step 3:Set to +/-4G Range*/
		BMA255_SetDataFormat(obj->client, BMA255_RANGE_4G);
		//bma255_set_range(obj->client, BMA_RANGE_4G);
		/*Step 4:Set Amplitude of Deflection*/
		{
			value = 0;
			amp = 1;
			bma_i2c_read_block(obj->client, BMA255_SELF_TEST_AMP__REG, &value, 1);
			value = BMA255_SET_BITSLICE(value, BMA255_SELF_TEST_AMP, amp);
			bma_i2c_write_block(obj->client, BMA255_SELF_TEST_AMP__REG, &value, 1);
		}

		/*Step 5:X-axis Selftest*/
		bma255_set_selftest_st(obj->client, 1);/*1 for x-axis*/
		bma255_set_selftest_stn(obj->client, 0);/*positive direction*/
		mdelay(10);
		error = BMA255_ReadData(client, obj->data);
		if(error)
		{
			GSE_ERR("I2C error: ret value=%d", error);
			return EIO;
		}
		else
		{
			value1[BMA255_AXIS_X] = obj->data[BMA255_AXIS_X];
			value1[BMA255_AXIS_Y] = obj->data[BMA255_AXIS_Y];
			value1[BMA255_AXIS_Z] = obj->data[BMA255_AXIS_Z];
		}
		bma255_set_selftest_stn(obj->client, 1);/*negative direction*/
		mdelay(10);
		error = BMA255_ReadData(client, obj->data);
		if(error)
		{
			GSE_ERR("I2C error: ret value=%d", error);
			return EIO;
		}
		else
		{
			value2[BMA255_AXIS_X] = obj->data[BMA255_AXIS_X];
			value2[BMA255_AXIS_Y] = obj->data[BMA255_AXIS_Y];
			value2[BMA255_AXIS_Z] = obj->data[BMA255_AXIS_Z];
		}
		diff = value1[BMA255_AXIS_X]-value2[BMA255_AXIS_X];

		GSE_LOG("diff x is %d,value1 is %d, value2 is %d\n", diff,
				value1[BMA255_AXIS_X], value2[BMA255_AXIS_X]);

		if (abs(diff) < 204)
			result |= 1;

		/*Step 6:Y-axis Selftest*/
		bma255_set_selftest_st(obj->client, 2);/*2 for y-axis*/
		bma255_set_selftest_stn(obj->client, 0);/*positive direction*/
		mdelay(10);
		error = BMA255_ReadData(client, obj->data);
		if(error)
		{
			GSE_ERR("I2C error: ret value=%d", error);
			return EIO;
		}
		else
		{
			value1[BMA255_AXIS_X] = obj->data[BMA255_AXIS_X];
			value1[BMA255_AXIS_Y] = obj->data[BMA255_AXIS_Y];
			value1[BMA255_AXIS_Z] = obj->data[BMA255_AXIS_Z];
		}
		bma255_set_selftest_stn(obj->client, 1);/*negative direction*/
		mdelay(10);
		error = BMA255_ReadData(client, obj->data);
		if(error)
		{
			GSE_ERR("I2C error: ret value=%d", error);
			return EIO;
		}
		else
		{
			value2[BMA255_AXIS_X] = obj->data[BMA255_AXIS_X];
			value2[BMA255_AXIS_Y] = obj->data[BMA255_AXIS_Y];
			value2[BMA255_AXIS_Z] = obj->data[BMA255_AXIS_Z];
		}
		diff = value1[BMA255_AXIS_Y]-value2[BMA255_AXIS_Y];
		GSE_LOG("diff y is %d,value1 is %d, value2 is %d\n", diff,
				value1[BMA255_AXIS_Y], value2[BMA255_AXIS_Y]);

		if (abs(diff) < 204)
			result |= 2;

		/*Step 7:Z-axis Selftest*/
		bma255_set_selftest_st(obj->client, 3);/*3 for z-axis*/
		bma255_set_selftest_stn(obj->client, 0);/*positive direction*/
		mdelay(10);
		error = BMA255_ReadData(client, obj->data);
		if(error)
		{
			GSE_ERR("I2C error: ret value=%d", error);
			return EIO;
		}
		else
		{
			value1[BMA255_AXIS_X] = obj->data[BMA255_AXIS_X];
			value1[BMA255_AXIS_Y] = obj->data[BMA255_AXIS_Y];
			value1[BMA255_AXIS_Z] = obj->data[BMA255_AXIS_Z];
		}
		bma255_set_selftest_stn(obj->client, 1);/*negative direction*/
		mdelay(10);
		error = BMA255_ReadData(client, obj->data);
		if(error)
		{
			GSE_ERR("I2C error: ret value=%d", error);
			return EIO;
		}
		else
		{
			value2[BMA255_AXIS_X] = obj->data[BMA255_AXIS_X];
			value2[BMA255_AXIS_Y] = obj->data[BMA255_AXIS_Y];
			value2[BMA255_AXIS_Z] = obj->data[BMA255_AXIS_Z];
		}
		diff = value1[BMA255_AXIS_Z]-value2[BMA255_AXIS_Z];

		GSE_LOG("diff z is %d,value1 is %d, value2 is %d\n", diff,
				value1[BMA255_AXIS_Z], value2[BMA255_AXIS_Z]);

		if (abs(diff) < 204)
			result |= 4;

		/*Step 8:Soft Reset*/
		bma255_soft_reset(obj->client);
		/*Sync sw settings to hw settings*/
		obj->range = range;//fix for 4G
		obj->bandwidth= bandwidth;

		atomic_set(&obj->selftest, result);;

		/*restore settings*/
		BMA255_SetDataFormat(obj->client, range);
		//bma255_set_range(obj->client, range);
		bma255_set_bandwidth(obj->client, bandwidth);
		BMA255_SetPowerMode(obj->client, power_mode);
		GSE_LOG("self test finished. result : %d\n",result);
    }
	else // wrong input
	{
    	GSE_ERR("SelfTest command FAIL\n");
    	return -EINVAL;
	}
	return count;
}

static ssize_t bma255_SelfTest_show(struct device_driver *ddri, char *buf)
{
    int selftest_rslt = 1;  // fail

    struct i2c_client *client = bma255_i2c_client;
    struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	if (obj == NULL) {
		GSE_ERR("bma i2c data pointer is null\n");
		return 0;
	}

//    if(atomic_read(&obj->selftest) == 1)  // selftest success
	if(atomic_read(&obj->selftest) == 0)  // selftest success
    {
        selftest_rslt = 0;  // success
    }
    else
    {
    	GSE_LOG("Self Test Fail : %d\n",atomic_read(&obj->selftest));
        selftest_rslt = 1;  // fail
    }
	return sprintf(buf, "%d\n", selftest_rslt);
}


static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	char strbuf[BMA255_BUFSIZE] = {0,};
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	BMA255_ReadChipInfo(client, strbuf, BMA255_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

#if 0
static ssize_t gsensor_init(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = bma255_i2c_client;
	char strbuf[BMA255_BUFSIZE] = {0,};
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	bma255_init_client(client, 1);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
}
#endif

/*----------------------------------------------------------------------------*/
/*
g sensor opmode for compass tilt compensation
*/
static ssize_t show_cpsopmode_value(struct device_driver *ddri, char *buf)
{
	unsigned char data;

	if (bma255_get_mode(bma255_i2c_client, &data) < 0)
	{
		return sprintf(buf, "Read error\n");
	}
	else
	{
		return sprintf(buf, "%d\n", data);
	}
}

/*----------------------------------------------------------------------------*/
/*
g sensor opmode for compass tilt compensation
*/
static ssize_t store_cpsopmode_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
	{
		return error;
	}
	if (data == BMA255_MODE_NORMAL)
	{
		BMA255_SetPowerMode(bma255_i2c_client, true);
	}
	else if (data == BMA255_MODE_SUSPEND)
	{
		BMA255_SetPowerMode(bma255_i2c_client, false);
	}
	else if (bma255_set_mode(bma255_i2c_client, (unsigned char) data) < 0)
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
/*
g sensor range for compass tilt compensation
*/
static ssize_t show_cpsrange_value(struct device_driver *ddri, char *buf)
{
	unsigned char data;

	if (bma255_get_range(bma255_i2c_client, &data) < 0)
	{
		return sprintf(buf, "Read error\n");
	}
	else
	{
		return sprintf(buf, "%d\n", data);
	}
}

/*----------------------------------------------------------------------------*/
/*
g sensor range for compass tilt compensation
*/
static ssize_t store_cpsrange_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
	{
		return error;
	}
	if (bma255_set_range(bma255_i2c_client, (unsigned char) data) < 0)
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
/*
g sensor bandwidth for compass tilt compensation
*/
static ssize_t show_cpsbandwidth_value(struct device_driver *ddri, char *buf)
{
	unsigned char data;

	if (bma255_get_bandwidth(bma255_i2c_client, &data) < 0)
	{
		return sprintf(buf, "Read error\n");
	}
	else
	{
		return sprintf(buf, "%d\n", data);
	}
}

/*----------------------------------------------------------------------------*/
/*
g sensor bandwidth for compass tilt compensation
*/
static ssize_t store_cpsbandwidth_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
	{
		return error;
	}
	if (bma255_set_bandwidth(bma255_i2c_client, (unsigned char) data) < 0)
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
/*
g sensor data for compass tilt compensation
*/
static ssize_t show_cpsdata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	char strbuf[BMA255_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	BMA255_CompassReadData(client, strbuf, BMA255_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	char strbuf[BMA255_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	BMA255_ReadSensorData(client, strbuf, BMA255_BUFSIZE);
	//BMA255_ReadRawData(client, strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}
#if 0
/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[BMA255_AXES_NUM];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);

	if(err = BMA255_ReadOffset(client, obj->offset))
	{
		return -EINVAL;
	}
	else if(err = BMA255_ReadCalibration(client, tmp))
	{
		return -EINVAL;
	}
	else
	{    
		mul = obj->reso->sensitivity/bma255_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[BMA255_AXIS_X], obj->offset[BMA255_AXIS_Y], obj->offset[BMA255_AXIS_Z],
			obj->offset[BMA255_AXIS_X], obj->offset[BMA255_AXIS_Y], obj->offset[BMA255_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[BMA255_AXIS_X], obj->cali_sw[BMA255_AXIS_Y], obj->cali_sw[BMA255_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[BMA255_AXIS_X]*mul + obj->cali_sw[BMA255_AXIS_X],
			obj->offset[BMA255_AXIS_Y]*mul + obj->cali_sw[BMA255_AXIS_Y],
			obj->offset[BMA255_AXIS_Z]*mul + obj->cali_sw[BMA255_AXIS_Z],
			tmp[BMA255_AXIS_X], tmp[BMA255_AXIS_Y], tmp[BMA255_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = bma255_i2c_client;  
	int err, x, y, z;
	int dat[BMA255_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if(err = BMA255_ResetCalibration(client))
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[BMA255_AXIS_X] = x;
		dat[BMA255_AXIS_Y] = y;
		dat[BMA255_AXIS_Z] = z;
		if(err = BMA255_WriteCalibration(client, dat))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return count;
}
#endif


/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_BMA255_LOWPASS
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][BMA255_AXIS_X], obj->fir.raw[idx][BMA255_AXIS_Y], obj->fir.raw[idx][BMA255_AXIS_Z]);
		}
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[BMA255_AXIS_X], obj->fir.sum[BMA255_AXIS_Y], obj->fir.sum[BMA255_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[BMA255_AXIS_X]/len, obj->fir.sum[BMA255_AXIS_Y]/len, obj->fir.sum[BMA255_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_BMA255_LOWPASS
	struct i2c_client *client = bma255_i2c_client;  
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct bma255_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bma255_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct bma255_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	if(sensor_power)
		GSE_LOG("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		GSE_LOG("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_teststatus_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;

	if(client == NULL)
	{
        	GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", test_status);
}

/*----------------------------------------------------------------------------*/
static ssize_t bma255_bandwidth_show(struct device_driver *ddri, char *buf)
{
	unsigned char data;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	if (bma255_get_bandwidth(bma255->client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}
/*----------------------------------------------------------------------------*/
static ssize_t bma255_bandwidth_store(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma255_set_bandwidth(bma255->client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_fifo_mode_value(struct device_driver *ddri, char *buf)
{
	unsigned char data;

	if (bma255_get_fifo_mode(bma255_i2c_client, &data) < 0)
	{
		return sprintf(buf, "Read error\n");
	}
	else
	{
		return sprintf(buf, "%d\n", data);
	}
}

/*----------------------------------------------------------------------------*/
static ssize_t store_fifo_mode_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
	{
		return error;
	}
	if (bma255_set_fifo_mode(bma255_i2c_client, (unsigned char) data) < 0)
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_fifo_framecount_value(struct device_driver *ddri, char *buf)
{
	unsigned char data;

	if (bma255_get_fifo_framecount(bma255_i2c_client, &data) < 0)
	{
		return sprintf(buf, "Read error\n");
	}
	else
	{
		return sprintf(buf, "%d\n", data);
	}
}

/*----------------------------------------------------------------------------*/
static ssize_t store_fifo_framecount_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct bma255_i2c_data *obj = obj_i2c_data;

	error = kstrtoul(buf, 10, &data);
	if (error)
	{
		return error;
	}
	mutex_lock(&obj->lock);
	obj->fifo_count = (unsigned char)data;
	mutex_unlock(&obj->lock);

	return count;
}


/*----------------------------------------------------------------------------*/
static ssize_t show_fifo_data_out_frame_value(struct device_driver *ddri, char *buf)
{
	int err = 0, i, len = 0;
//	int addr = 0;
	u8 fifo_data_out[MAX_FIFO_F_BYTES] = {0};
	/* Select X Y Z axis data output for every fifo frame, not single axis data */
	unsigned char f_len = 6;/* FIXME: ONLY USE 3-AXIS */
	struct bma255_i2c_data *obj = obj_i2c_data;
	s16 acc[BMA255_AXES_NUM];
	s16 databuf[BMA255_AXES_NUM];

	if (obj->fifo_count == 0) {
		return -EINVAL;
	}

	for (i = 0; i < obj->fifo_count; i++) {
		if (bma_i2c_read_block(bma255_i2c_client,
			BMA255_FIFO_DATA_OUTPUT_REG, fifo_data_out, f_len) < 0)
		{
			GSE_ERR("[a]fatal error\n");
			return sprintf(buf, "Read byte block error\n");
		}
		/*data combination*/
		databuf[BMA255_AXIS_X] = ((s16)(((u16)fifo_data_out[1] << 8) |
						(u16)fifo_data_out[0])) >> 6;
		databuf[BMA255_AXIS_Y] = ((s16)(((u16)fifo_data_out[3] << 8) |
						(u16)fifo_data_out[2])) >> 6;
		databuf[BMA255_AXIS_Z] = ((s16)(((u16)fifo_data_out[5] << 8) |
						(u16)fifo_data_out[4])) >> 6;
		/*axis remap*/
		acc[obj->cvt.map[BMA255_AXIS_X]] = obj->cvt.sign[BMA255_AXIS_X]*databuf[BMA255_AXIS_X];
		acc[obj->cvt.map[BMA255_AXIS_Y]] = obj->cvt.sign[BMA255_AXIS_Y]*databuf[BMA255_AXIS_Y];
		acc[obj->cvt.map[BMA255_AXIS_Z]] = obj->cvt.sign[BMA255_AXIS_Z]*databuf[BMA255_AXIS_Z];
		
		len = sprintf(buf, "%d %d %d ", acc[BMA255_AXIS_X], acc[BMA255_AXIS_Y], acc[BMA255_AXIS_Z]);
		buf += len;
		err += len;
	}

	return err;
}

#ifdef BMA255_ENABLE_INT1
static ssize_t bma255_slop_enable_show(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client =bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma255->slop_detect_enable));
}

static ssize_t bma255_slop_enable_store(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client =bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if ((data == 0) || (data == 1))
		bma255_set_slop_enable(bma255->client, data);

	return count;
}

static ssize_t bma255_slope_duration_show(struct device_driver *ddri, char *buf)
{
	unsigned char data;
	struct i2c_client *client =bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	if (bma255_get_slope_duration(bma255->client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma255_slope_duration_store(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client =bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma255_set_slope_duration(bma255->client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma255_slope_threshold_show(struct device_driver *ddri, char *buf)
{
	unsigned char data;
	struct i2c_client *client =bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	if (bma255_get_slope_threshold(bma255->client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma255_slope_threshold_store(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client =bma255_i2c_client;
	struct bma255_i2c_data *bma255 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma255_set_slope_threshold(bma255->client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}
#endif

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(cpsdata, S_IRUGO, show_cpsdata_value, NULL);
static DRIVER_ATTR(cpsopmode, S_IRUGO | S_IWUSR, show_cpsopmode_value, store_cpsopmode_value);
static DRIVER_ATTR(cpsrange, S_IRUGO | S_IWUSR, show_cpsrange_value, store_cpsrange_value);
static DRIVER_ATTR(cpsbandwidth, S_IRUGO | S_IWUSR, show_cpsbandwidth_value, store_cpsbandwidth_value);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IRUGO | S_IWUSR, show_cali_value, store_cali_value);
static DRIVER_ATTR(firlen, S_IRUGO | S_IWUSR, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(powerstatus, S_IRUGO, show_power_status_value, NULL);
static DRIVER_ATTR(softreset, S_IWUSR, NULL, bma255_softreset_store);
static DRIVER_ATTR(teststatus, S_IRUGO, show_teststatus_value, NULL);
static DRIVER_ATTR(fifo_mode, S_IRUGO | S_IWUSR, show_fifo_mode_value, store_fifo_mode_value);
static DRIVER_ATTR(fifo_framecount, S_IRUGO | S_IWUSR, show_fifo_framecount_value, store_fifo_framecount_value);
static DRIVER_ATTR(fifo_data_frame, S_IRUGO, show_fifo_data_out_frame_value, NULL);
#ifdef BMA255_ACCEL_CALIBRATION
static DRIVER_ATTR(fast_calibration_x, S_IRUGO | S_IWUSR, bma255_fast_calibration_x_show, bma255_fast_calibration_x_store);
static DRIVER_ATTR(fast_calibration_y, S_IRUGO | S_IWUSR, bma255_fast_calibration_y_show, bma255_fast_calibration_y_store);
static DRIVER_ATTR(fast_calibration_z, S_IRUGO | S_IWUSR, bma255_fast_calibration_z_show, bma255_fast_calibration_z_store);
static DRIVER_ATTR(run_fast_calibration, S_IRUGO | S_IWUSR, bma255_runCalibration_show, bma255_runCalibration_store);
static DRIVER_ATTR(eeprom_writing, S_IRUGO | S_IWUSR, bma255_eeprom_writing_show, bma255_eeprom_writing_store);

static DRIVER_ATTR(cali_backup, S_IWUSR|S_IRUGO, show_cali_backup_value, NULL);
#endif
static DRIVER_ATTR(bandwidth, S_IRUGO|S_IWUSR, bma255_bandwidth_show, bma255_bandwidth_store);
static DRIVER_ATTR(selftest, S_IRUGO|S_IWUSR|S_IWGRP, bma255_SelfTest_show, bma255_SelfTest_store);
#ifdef BMA255_ENABLE_INT1
static DRIVER_ATTR(enable_slop, S_IRUGO|S_IWUSR, bma255_slop_enable_show, bma255_slop_enable_store);
static DRIVER_ATTR(slope_duration, S_IRUGO|S_IWUSR, bma255_slope_duration_show, bma255_slope_duration_store);
static DRIVER_ATTR(slope_threshold, S_IRUGO|S_IWUSR, bma255_slope_threshold_show, bma255_slope_threshold_store);
#endif

/*----------------------------------------------------------------------------*/
static struct driver_attribute *bma255_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_cpsdata,	/*g sensor data for compass tilt compensation*/
	&driver_attr_cpsopmode,	/*g sensor opmode for compass tilt compensation*/
	&driver_attr_cpsrange,	/*g sensor range for compass tilt compensation*/
	&driver_attr_cpsbandwidth,	/*g sensor bandwidth for compass tilt compensation*/
	&driver_attr_fifo_mode,
	&driver_attr_fifo_framecount,
	&driver_attr_fifo_data_frame,
	&driver_attr_bandwidth,
	&driver_attr_softreset,
	&driver_attr_teststatus,
	&driver_attr_fast_calibration_x,
	&driver_attr_fast_calibration_y,
	&driver_attr_fast_calibration_z,
	&driver_attr_run_fast_calibration,
	&driver_attr_eeprom_writing,
    &driver_attr_selftest,
	&driver_attr_cali_backup,		  /*show calibration backup data*/
#ifdef BMA255_ENABLE_INT1
	&driver_attr_enable_slop,
	&driver_attr_slope_duration,
	&driver_attr_slope_threshold,
#endif
};
/*----------------------------------------------------------------------------*/
static int bma255_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(bma255_attr_list)/sizeof(bma255_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		err = driver_create_file(driver, bma255_attr_list[idx]);
		if(err)
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", bma255_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int bma255_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(bma255_attr_list)/sizeof(bma255_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, bma255_attr_list[idx]);
	}
	
	return err;
}

/*----------------------------------------------------------------------------*/
#ifdef BMC050_M4G
int bmm050_m4g_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* g_data;	
#if DEBUG	
//	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *data = i2c_get_clientdata(bma255_i2c_client);
#endif
	
#if DEBUG
	if(atomic_read(&data->trace) & BMA_TRC_INFO)
	{
		GSE_FUN();
	}	
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				m4g_delay = value;
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_DELAY;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}	
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&g_flag, 1);
				}
				else
				{
					atomic_set(&g_flag, 0);
				}	
				
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_ACTIVE;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR( "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				g_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				
				g_data->values[0] = m4g_data[0];
				g_data->values[1] = m4g_data[1];
				g_data->values[2] = m4g_data[2];
				g_data->status = m4g_data[3];
				g_data->value_divide = CONVERT_G_DIV;
					
				mutex_unlock(&sensor_data_mutex);
#if DEBUG
				if(atomic_read(&data->trace) & BMA_TRC_INFO)
				{
					GSE_LOG("Hwm get m4g data: %d, %d, %d. divide %d, status %d!\n",
						g_data->values[0],g_data->values[1],g_data->values[2],
						g_data->value_divide,g_data->status);
				}	
#endif
			}
			break;
		default:
			GSE_ERR( "m4g operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif //BMC050_M4G
/*----------------------------------------------------------------------------*/
#ifdef BMC050_VRV
int bmm050_vrv_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* vrv_data;	
#if DEBUG	
//	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *data = i2c_get_clientdata(bma255_i2c_client);
#endif
	
#if DEBUG
	if(atomic_read(&data->trace) & BMA_TRC_INFO)
	{
		GSE_FUN();
	}	
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				vrv_delay = value;
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_VRV_DELAY;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}	
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&vrv_flag, 1);
				}
				else
				{
					atomic_set(&vrv_flag, 0);
				}	
				
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_VRV_ACTIVE;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR( "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				vrv_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				
				vrv_data->values[0] = m4g_data[4];
				vrv_data->values[1] = m4g_data[5];
				vrv_data->values[2] = m4g_data[6];
				vrv_data->status = m4g_data[7];
				vrv_data->value_divide = CONVERT_VRV_DIV;
					
				mutex_unlock(&sensor_data_mutex);
#if DEBUG
				if(atomic_read(&data->trace) & BMA_TRC_INFO)
				{
					GSE_LOG("Hwm get rotation vector data: %d, %d, %d. divide %d, status %d!\n",
						vrv_data->values[0],vrv_data->values[1],vrv_data->values[2],
						vrv_data->value_divide,vrv_data->status);
				}	
#endif
			}
			break;
		default:
			GSE_ERR( "rotation vector operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif //BMC050_VRV
/*----------------------------------------------------------------------------*/
#ifdef BMC050_VLA
int bmm050_vla_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* vla_value;	
#if DEBUG	
//	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *data = i2c_get_clientdata(bma255_i2c_client);
#endif
	
#if DEBUG
	if(atomic_read(&data->trace) & BMA_TRC_INFO)
	{
		GSE_FUN();
	}	
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				vla_delay = value;
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_VLA_DELAY;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}	
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&vla_flag, 1);
				}
				else
				{
					atomic_set(&vla_flag, 0);
				}	
				
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_VLA_ACTIVE;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR( "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				vla_value = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				
				vla_value->values[0] = vla_data[0];
				vla_value->values[1] = vla_data[1];
				vla_value->values[2] = vla_data[2];
				vla_value->status = vla_data[3];
				vla_value->value_divide = CONVERT_VLA_DIV;
					
				mutex_unlock(&sensor_data_mutex);
#if DEBUG
				if(atomic_read(&data->trace) & BMA_TRC_INFO)
				{
					GSE_LOG("Hwm get virtual linear accelerometer data: %d, %d, %d. divide %d, status %d!\n",
						vla_value->values[0],vla_value->values[1],vla_value->values[2],
						vla_value->value_divide,vla_value->status);
				}	
#endif
			}
			break;
		default:
			GSE_ERR( "virtual linear accelerometer operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif //BMC050_VLA
/*----------------------------------------------------------------------------*/
#ifdef BMC050_VG
int bmm050_vg_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* vg_value;	
#if DEBUG	
//	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *data = i2c_get_clientdata(bma255_i2c_client);
#endif
	
#if DEBUG
	if(atomic_read(&data->trace) & BMA_TRC_INFO)
	{
		GSE_FUN();
	}	
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				vg_delay = value;
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_VG_DELAY;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}	
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&vg_flag, 1);
				}
				else
				{
					atomic_set(&vg_flag, 0);
				}	
				
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_VG_ACTIVE;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR( "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				vg_value = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				
				vg_value->values[0] = vg_data[0];
				vg_value->values[1] = vg_data[1];
				vg_value->values[2] = vg_data[2];
				vg_value->status = vg_data[3];
				vg_value->value_divide = CONVERT_VG_DIV;
					
				mutex_unlock(&sensor_data_mutex);
#if DEBUG
				if(atomic_read(&data->trace) & BMA_TRC_INFO)
				{
					GSE_LOG("Hwm get virtual gravity data: %d, %d, %d. divide %d, status %d!\n",
						vg_value->values[0],vg_value->values[1],vg_value->values[2],
						vg_value->value_divide,vg_value->status);
				}	
#endif
			}
			break;
		default:
			GSE_ERR( "virtual gravity operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif //BMC050_VG
//tad3sgh add --

#if defined(BMA255_ENABLE_INT1) || defined(BMA255_ENABLE_INT2)
/*----------------------------------------------------------------------------*/
int sld_sensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;
	struct bma255_i2c_data *priv = (struct bma255_i2c_data*)self;
	struct hwm_sensor_data* gsensor_data;
	char buff[BMA255_BUFSIZE];
	GSE_ERR("slope detctor operate cmd= %d!\n", command);

	switch (command)
	{
		case SENSOR_DELAY:
			// pass
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				GSE_LOG("sld SENSOR_ENABLE %d\n",value);
				bma255_set_slop_enable(priv->client, value);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (struct hwm_sensor_data *)buff_out;
				gsensor_data->values[0] = priv->slope_value;
				gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				gsensor_data->value_divide = 1;
			}
			break;
		default:
			GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

static int bma255_get_slope_first(struct i2c_client *client, unsigned char
	param, unsigned char *intstatus)
{
	int comres = 0;
	unsigned char data;

	switch (param) {
	case 0:
		comres = bma255_smbus_read_byte(client,
				BMA255_STATUS_TAP_SLOPE_REG, &data);
		data = BMA255_GET_BITSLICE(data, BMA255_SLOPE_FIRST_X);
		*intstatus = data;
		break;
	case 1:
		comres = bma255_smbus_read_byte(client,
				BMA255_STATUS_TAP_SLOPE_REG, &data);
		data = BMA255_GET_BITSLICE(data, BMA255_SLOPE_FIRST_Y);
		*intstatus = data;
		break;
	case 2:
		comres = bma255_smbus_read_byte(client,
				BMA255_STATUS_TAP_SLOPE_REG, &data);
		data = BMA255_GET_BITSLICE(data, BMA255_SLOPE_FIRST_Z);
		*intstatus = data;
		break;
	default:
		break;
	}

	return comres;
}

static int bma255_get_slope_sign(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data;

	comres = bma255_smbus_read_byte(client, BMA255_STATUS_TAP_SLOPE_REG,
			&data);
	data = BMA255_GET_BITSLICE(data, BMA255_SLOPE_SIGN_S);
	*intstatus = data;

	return comres;
}



static int bma255_get_interruptstatus1(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data;

	comres = bma255_smbus_read_byte(client, BMA255_STATUS1_REG, &data);
	*intstatus = data;

	return comres;
}

// TODO change data path.


static void bma255_irq_work_func(struct work_struct *work)
{
	struct bma255_i2c_data *bma255 = container_of((struct work_struct *)work,
			struct bma255_i2c_data, irq_work);

	unsigned char status = 0;
	unsigned char i;
	unsigned char first_value = 0;
	unsigned char sign_value = 0;
	struct hwm_sensor_data sensor_data;
	GSE_LOG("bma255_irq_work_func\n");
	disable_irq_nosync(bma255_irq);

	bma255_get_interruptstatus1(bma255->client, &status);

	switch (status) {
	case 0x04:
		for (i = 0; i < 3; i++) {
			bma255_get_slope_first(bma255->client, i,
					   &first_value);
			if (first_value == 1) {
				// get slope data??
				bma255_get_slope_sign(bma255->client,
						   &sign_value);
				sensor_data.value_divide =1;
				sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
				if (sign_value == 1) {
					if (i == 0)
						//input_report_rel(bma255->input, SLOP_INTERRUPT, SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED);
						sensor_data.values[0] = SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED;
					else if (i == 1)
						//input_report_rel(bma255->input, SLOP_INTERRUPT, SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED);
						sensor_data.values[0] = SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED;
					else if (i == 2)
						//input_report_rel(bma255->input, SLOP_INTERRUPT, SLOPE_INTERRUPT_Z_NEGATIVE_HAPPENED);
						sensor_data.values[0] = SLOPE_INTERRUPT_Z_NEGATIVE_HAPPENED;
				} else {
					if (i == 0)
						//input_report_rel(bma255->input, SLOP_INTERRUPT, SLOPE_INTERRUPT_X_HAPPENED);
						sensor_data.values[0] = SLOPE_INTERRUPT_X_HAPPENED;
					else if (i == 1)
						//input_report_rel(bma255->input, SLOP_INTERRUPT, SLOPE_INTERRUPT_Y_HAPPENED);
						sensor_data.values[0] = SLOPE_INTERRUPT_Y_HAPPENED;
					else if (i == 2)
						//input_report_rel(bma255->input, SLOP_INTERRUPT, SLOPE_INTERRUPT_Z_HAPPENED);
						sensor_data.values[0] = SLOPE_INTERRUPT_Z_HAPPENED;
				}
				bma255->slope_value = sensor_data.values[0];
				hwmsen_get_interrupt_data(ID_SLOPE_DETECTOR, &sensor_data);
			}
		}
		break;
	default:
		break;
	}
	enable_irq(bma255_irq);
	enable_irq_wake(bma255_irq);
}

static irqreturn_t bma255_irq_handler(int irq, void *handle)
{

	struct bma255_i2c_data *data = handle;

	if (data == NULL)
		return IRQ_HANDLED;
	if (data->client == NULL)
		return IRQ_HANDLED;

	schedule_work(&data->irq_work);

	return IRQ_HANDLED;


}
#endif /* defined(BMA255_ENABLE_INT1)||defined(BMA255_ENABLE_INT2) */


/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int bma255_open(struct inode *inode, struct file *file)
{
	file->private_data = bma255_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int bma255_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long bma255_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[BMA255_BUFSIZE]={0,};
	void __user *data;
	struct SENSOR_DATA sensor_data;
	long err = 0;
	uint32_t enable = 0;
#if defined(CALIBRATION_TO_FILE)
	int cali[6];
#endif
	//tad3sgh add ++
	int status; 				/* for OPEN/CLOSE_STATUS */
	short sensor_status;		/* for Orientation and Msensor status */
	int value[CALIBRATION_DATA_SIZE];			/* for SET_YPR */
	//tad3sgh add --
	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_SET_ENABLE:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;
			}
			if(copy_from_user(&enable, data, sizeof(enable)))
			{
				return -EFAULT;
			}
			else
			{
				GSE_LOG("GSENSOR_IOCTL_SET_ENABLE %d\n",(int)data);
				if(enable == 1)
				{
					BMA255_SetPowerMode(obj_i2c_data->client, 1);
				}
				else if(enable == 0)
				{
					BMA255_SetPowerMode(obj_i2c_data->client, 0);
				}
			}
			break;
		case GSENSOR_IOCTL_INIT:
			bma255_init_client(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			BMA255_ReadChipInfo(client, strbuf, BMA255_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			BMA255_ReadSensorData(client, strbuf, BMA255_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			BMA255_ReadRawData(client, strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			GSE_ERR(" GSENSOR_IOCTL_SET_CALI !!\n");
			if(data == NULL )
			{
				err = -EINVAL;
				break;	  
			}

#if defined(CALIBRATION_TO_FILE)

			err = sensor_calibration_read(ID_ACCELEROMETER, cali);
			if(err!=0){
				GSE_LOG("Read Cal Fail from file !!!\n");
				break;
			}
			else
			{
			sensor_data.x = cali[0];
			sensor_data.y = cali[1];
			sensor_data.z = cali[2];
			}
#else
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
#endif

			if((sensor_data.x==0)&&(sensor_data.y==0)&&(sensor_data.z==0))
             {
			    GSE_LOG("Read Cal Data x : %d / y : %d / z : %d\n",sensor_data.x,sensor_data.y,sensor_data.z);
			    GSE_LOG("Read Cal Data all Zero, Do not set register\n");
				err = -EINVAL;
				break;
             }
						
			if(bma255_set_offset_x(obj->client, (unsigned char)sensor_data.x) < 0){
				err = -EINVAL;
				break;
				}
			if(bma255_set_offset_y(obj->client, (unsigned char)sensor_data.y) < 0){
				err = -EINVAL;
				break;
				}
			if(bma255_set_offset_z(obj->client, (unsigned char)sensor_data.z) < 0){
				err = -EINVAL;
				break;
				}

			GSE_LOG("GSENSOR_IOCTL_SET_CALI, Cal data : x=%d, y=%d, z=%d\n", sensor_data.x, sensor_data.y, sensor_data.z);
			break; 


	/*		if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{				
				cali[BMA255_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMA255_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMA255_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;					  
				err = BMA255_WriteCalibration(client, cali);			 
			}
			break; */

		case GSENSOR_IOCTL_CLR_CALI:
			err = BMA255_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			GSE_ERR(" GSENSOR_IOCTL_GET_CALI !!\n");
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			err = BMA255_ReadCalibration(client, cali);
			if(err)
			{
				break;
			}
			
			sensor_data.x = cali[BMA255_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[BMA255_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[BMA255_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
		
			break;
		//tad3sgh add ++
		case BMM_IOC_GET_EVENT_FLAG:	// used by daemon only
			data = (void __user *) arg;
			/* block if no event updated */
			wait_event_interruptible(uplink_event_flag_wq, (uplink_event_flag != 0));
			mutex_lock(&uplink_event_flag_mutex);
			status = uplink_event_flag;
			mutex_unlock(&uplink_event_flag_mutex);
			if(copy_to_user(data, &status, sizeof(status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case BMM_IOC_GET_NONBLOCK_EVENT_FLAG:	// used by daemon only
			data = (void __user *) arg;
			/* nonblock daemon process */
			//wait_event_interruptible(uplink_event_flag_wq, (uplink_event_flag != 0));
			mutex_lock(&uplink_event_flag_mutex);
			status = uplink_event_flag;
			mutex_unlock(&uplink_event_flag_mutex);
			if(copy_to_user(data, &status, sizeof(status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			break;
			
		case ECOMPASS_IOC_GET_DELAY:			//used by daemon
			data = (void __user *) arg;
			if(copy_to_user(data, &bmm050d_delay, sizeof(bmm050d_delay)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_M_DELAY) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_M_DELAY;
			}
			else if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_O_DELAY) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_O_DELAY;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;		
			
		case ECOMPASS_IOC_SET_YPR:				//used by daemon
			data = (void __user *) arg;
			if(data == NULL)
			{
				GSE_ERR("invalid argument.");
				return -EINVAL;
			}
			if(copy_from_user(value, data, sizeof(value)))
			{
				GSE_ERR("copy_from_user failed.");
				return -EFAULT;
			}
			ECS_SaveData(value);
			break;

		case ECOMPASS_IOC_GET_MFLAG:		//used by daemon
			data = (void __user *) arg;
			sensor_status = atomic_read(&m_flag);
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
			if ((sensor_status == 1) && (atomic_read(&driver_suspend_flag) == 1))
			{
				/* de-active m-channel when driver suspend regardless of m_flag*/
				sensor_status = 0;
			}
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
			if(copy_to_user(data, &sensor_status, sizeof(sensor_status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_M_ACTIVE) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_M_ACTIVE;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;
			
		case ECOMPASS_IOC_GET_OFLAG:		//used by daemon
			data = (void __user *) arg;
			sensor_status = atomic_read(&o_flag);
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
			if ((sensor_status == 1) && (atomic_read(&driver_suspend_flag) == 1))
			{
				/* de-active m-channel when driver suspend regardless of m_flag*/
				sensor_status = 0;
			}
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
			if(copy_to_user(data, &sensor_status, sizeof(sensor_status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_O_ACTIVE) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_O_ACTIVE;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;			
		                
#ifdef BMC050_M4G
		case ECOMPASS_IOC_GET_GDELAY:			//used by daemon
			data = (void __user *) arg;
			if(copy_to_user(data, &m4g_delay, sizeof(m4g_delay)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_G_DELAY) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_G_DELAY;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;		

		case ECOMPASS_IOC_GET_GFLAG:		//used by daemon
			data = (void __user *) arg;
			sensor_status = atomic_read(&g_flag);
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
			if ((sensor_status == 1) && (atomic_read(&driver_suspend_flag) == 1))
			{
				/* de-active g-channel when driver suspend regardless of g_flag*/
				sensor_status = 0;
			}
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
			if(copy_to_user(data, &sensor_status, sizeof(sensor_status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_G_ACTIVE) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_G_ACTIVE;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;			
#endif //BMC050_M4G

#ifdef BMC050_VRV
		case ECOMPASS_IOC_GET_VRVDELAY:			//used by daemon
			data = (void __user *) arg;
			if(copy_to_user(data, &vrv_delay, sizeof(vrv_delay)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_VRV_DELAY) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_VRV_DELAY;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;		

		case ECOMPASS_IOC_GET_VRVFLAG:		//used by daemon
			data = (void __user *) arg;
			sensor_status = atomic_read(&vrv_flag);
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
			if ((sensor_status == 1) && (atomic_read(&driver_suspend_flag) == 1))
			{
				/* de-active vrv-channel when driver suspend regardless of vrv_flag*/
				sensor_status = 0;
			}
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
			if(copy_to_user(data, &sensor_status, sizeof(sensor_status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_VRV_ACTIVE) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_VRV_ACTIVE;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;			
#endif //BMC050_VRV

#ifdef BMC050_VLA
		case ECOMPASS_IOC_GET_VLADELAY: 		//used by daemon
			data = (void __user *) arg;
			if(copy_to_user(data, &vla_delay, sizeof(vla_delay)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_VLA_DELAY) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_VLA_DELAY;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;		

		case ECOMPASS_IOC_GET_VLAFLAG:		//used by daemon
			data = (void __user *) arg;
			sensor_status = atomic_read(&vla_flag);
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
			if ((sensor_status == 1) && (atomic_read(&driver_suspend_flag) == 1))
			{
				/* de-active vla-channel when driver suspend regardless of vla_flag*/
				sensor_status = 0;
			}
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
			if(copy_to_user(data, &sensor_status, sizeof(sensor_status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_VLA_ACTIVE) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_VLA_ACTIVE;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;			
#endif //BMC050_VLA

#ifdef BMC050_VG
		case ECOMPASS_IOC_GET_VGDELAY: 		//used by daemon
			data = (void __user *) arg;
			if(copy_to_user(data, &vg_delay, sizeof(vg_delay)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_VG_DELAY) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_VG_DELAY;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;		

		case ECOMPASS_IOC_GET_VGFLAG:		//used by daemon
			data = (void __user *) arg;
			sensor_status = atomic_read(&vg_flag);
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
			if ((sensor_status == 1) && (atomic_read(&driver_suspend_flag) == 1))
			{
				/* de-active vla-channel when driver suspend regardless of vla_flag*/
				sensor_status = 0;
			}
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
			if(copy_to_user(data, &sensor_status, sizeof(sensor_status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_VG_ACTIVE) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_VG_ACTIVE;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;			
#endif //BMC050_VG
		//tad3sgh add --
		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations bma255_fops = {
	//.owner = THIS_MODULE,
	.open = bma255_open,
	.release = bma255_release,
	.unlocked_ioctl = bma255_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice bma255_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &bma255_fops,
};
/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int bma255_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;
	
	GSE_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);
		//tad3sgh add ++
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
		/* set driver suspend flag */
		atomic_set(&driver_suspend_flag, 1);
		if (atomic_read(&m_flag) == 1)
		{
			/* set the flag to block e-compass daemon*/
			mutex_lock(&uplink_event_flag_mutex);
			uplink_event_flag |= BMMDRV_ULEVT_FLAG_M_ACTIVE;
			mutex_unlock(&uplink_event_flag_mutex);
		}
		if (atomic_read(&o_flag) == 1)
		{
			/* set the flag to block e-compass daemon*/
			mutex_lock(&uplink_event_flag_mutex);
			uplink_event_flag |= BMMDRV_ULEVT_FLAG_O_ACTIVE;
			mutex_unlock(&uplink_event_flag_mutex);
		}
#ifdef BMC050_M4G
		if (atomic_read(&g_flag) == 1)
		{
			/* set the flag to block e-compass daemon*/
			mutex_lock(&uplink_event_flag_mutex);
			uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_ACTIVE;
			mutex_unlock(&uplink_event_flag_mutex);
		}
#endif //BMC050_M4G
#ifdef BMC050_VRV
		if (atomic_read(&vrv_flag) == 1)
		{
			/* set the flag to block e-compass daemon*/
			mutex_lock(&uplink_event_flag_mutex);
			uplink_event_flag |= BMMDRV_ULEVT_FLAG_VRV_ACTIVE;
			mutex_unlock(&uplink_event_flag_mutex);
		}
#endif //BMC050_VRV
#ifdef BMC050_VLA
		if (atomic_read(&vla_flag) == 1)
		{
			/* set the flag to block e-compass daemon*/
			mutex_lock(&uplink_event_flag_mutex);
			uplink_event_flag |= BMMDRV_ULEVT_FLAG_VLA_ACTIVE;
			mutex_unlock(&uplink_event_flag_mutex);
		}
#endif //BMC050_VLA
#ifdef BMC050_VG
		if (atomic_read(&vg_flag) == 1)
		{
			/* set the flag to block e-compass daemon*/
			mutex_lock(&uplink_event_flag_mutex);
			uplink_event_flag |= BMMDRV_ULEVT_FLAG_VG_ACTIVE;
			mutex_unlock(&uplink_event_flag_mutex);
		}
#endif //BMC050_VG

		/* wake up the wait queue */
		wake_up(&uplink_event_flag_wq);
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND

//tad3sgh add --
		err = BMA255_SetPowerMode(obj->client, false);
		if(err)
		{
			GSE_ERR("write power control fail!!\n");
			return -EINVAL;
		}       
		BMA255_power(obj->hw, 0);
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int bma255_resume(struct i2c_client *client)
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	BMA255_power(obj->hw, 1);
	err = bma255_init_client(client, 0);
	if(err)
	{
		GSE_ERR("initialize client fail!!\n");
		return err;        
	}
	//tad3sgh add ++
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
	/* clear driver suspend flag */
	atomic_set(&driver_suspend_flag, 0);
	if (atomic_read(&m_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_M_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
	if (atomic_read(&o_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_O_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#ifdef BMC050_M4G
	if (atomic_read(&g_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_M4G
#ifdef BMC050_VRV
	if (atomic_read(&vrv_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VRV_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VRV
#ifdef BMC050_VG
	if (atomic_read(&vg_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VG_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VG

	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
//tad3sgh add --

	atomic_set(&obj->suspend, 0);

#ifdef BMA255_ENABLE_INT1
	if (atomic_read(&obj->slop_detect_enable) == 1) {
		atomic_set(&obj->slop_detect_enable, 0);
		bma255_set_slop_enable(client, 1);
	}
#endif

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void bma255_early_suspend(struct early_suspend *h) 
{
	struct bma255_i2c_data *obj = container_of(h, struct bma255_i2c_data, early_drv);   
	int err;
	
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1); 
//tad3sgh add ++
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
	/* set driver suspend flag */
	atomic_set(&driver_suspend_flag, 1);
	if (atomic_read(&m_flag) == 1)
	{
		/* set the flag to block e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_M_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
	if (atomic_read(&o_flag) == 1)
	{
		/* set the flag to block e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_O_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#ifdef BMC050_M4G
	if (atomic_read(&g_flag) == 1)
	{
		/* set the flag to block e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_M4G
#ifdef BMC050_VRV
	if (atomic_read(&vrv_flag) == 1)
	{
		/* set the flag to block e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VRV_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VRV
#ifdef BMC050_VLA
	if (atomic_read(&vla_flag) == 1)
	{
		/* set the flag to block e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VLA_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VLA
#ifdef BMC050_VG
	if (atomic_read(&vg_flag) == 1)
	{
		/* set the flag to block e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VG_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VG

	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND

//tad3sgh add --
	if(err = BMA255_SetPowerMode(obj->client, false))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}

	BMA255_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void bma255_late_resume(struct early_suspend *h)
{
	struct bma255_i2c_data *obj = container_of(h, struct bma255_i2c_data, early_drv);         
	int err;
	
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

	BMA255_power(obj->hw, 1);
	if(err = bma255_init_client(obj->client, 0))
	{
		GSE_ERR("initialize client fail!!\n");
		return;        
	}
//tad3sgh add ++
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
	/* clear driver suspend flag */
	atomic_set(&driver_suspend_flag, 0);
	if (atomic_read(&m_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_M_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
	if (atomic_read(&o_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_O_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#ifdef BMC050_M4G
	if (atomic_read(&g_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_M4G
#ifdef BMC050_VRV
	if (atomic_read(&vrv_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VRV_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VRV
#ifdef BMC050_VG
	if (atomic_read(&vg_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VG_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VG

	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
//tad3sgh add --
	atomic_set(&obj->suspend, 0);    
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int bma255_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int bma255_enable_nodata(int en)
{
	int err = 0;

	if(((en == 0) && (sensor_power == false))
			||((en == 1) && (sensor_power == true))) {
		GSE_LOG("Gsensor device have updated!\n");
	} else {
		err = BMA255_SetPowerMode(obj_i2c_data->client, !sensor_power);
	}

	return err;
}

static int bma255_set_delay(u64 ns)
{
	int err = 0;
	int value, sample_delay;

	value = (int)ns/1000/1000;
	if(value <= 5) {
		sample_delay = BMA255_BW_200HZ;
	} else if(value <= 10) {
		sample_delay = BMA255_BW_100HZ;
	} else {
		sample_delay = BMA255_BW_50HZ;
	}

	//err = BMA255_SetBWRate(obj_i2c_data->client, sample_delay);
	if(err != BMA255_SUCCESS ) {
		GSE_ERR("Set delay parameter error!\n");
	}

	if(value >= 50) {
		atomic_set(&obj_i2c_data->filter, 0);
	} else {
#if defined(CONFIG_BMA255_LOWPASS)
		obj_i2c_data->fir.num = 0;
		obj_i2c_data->fir.idx = 0;
		obj_i2c_data->fir.sum[BMA255_AXIS_X] = 0;
		obj_i2c_data->fir.sum[BMA255_AXIS_Y] = 0;
		obj_i2c_data->fir.sum[BMA255_AXIS_Z] = 0;
		atomic_set(&obj_i2c_data->filter, 1);
#endif
	}

	return 0;
}

static int bma255_get_data(int* x ,int* y,int* z, int* status)
{
	char buff[BMA255_BUFSIZE];
	/* use acc raw data for gsensor */
	BMA255_ReadSensorData(obj_i2c_data->client, buff, BMA255_BUFSIZE);

	sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

int bsx_algo_m_enable(int en)
{
	if(en == 1) {
		atomic_set(&m_flag, 1);
	} else {
		atomic_set(&m_flag, 0);
	}

	/* set the flag */
	mutex_lock(&uplink_event_flag_mutex);
	uplink_event_flag |= BMMDRV_ULEVT_FLAG_M_ACTIVE;
	mutex_unlock(&uplink_event_flag_mutex);
	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);

	return 0;
}

int bsx_algo_m_set_delay(u64 ns)
{
	int value = (int)ns/1000/1000;

	bmm050d_delay = value;
	/* set the flag */
	mutex_lock(&uplink_event_flag_mutex);
	uplink_event_flag |= BMMDRV_ULEVT_FLAG_M_DELAY;
	mutex_unlock(&uplink_event_flag_mutex);
	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);

	return 0;
}

int bsx_algo_m_open_report_data(int open)
{
	return 0;
}

int bsx_algo_m_get_data(int* x ,int* y,int* z, int* status)
{
	mutex_lock(&sensor_data_mutex);

	*x = sensor_data[4];
	*y = sensor_data[5];
	*z = sensor_data[6];
	*status = sensor_data[7];

	mutex_unlock(&sensor_data_mutex);

	return 0;
}

int bsx_algo_o_enable(int en)
{
	if(en == 1) {
		atomic_set(&o_flag, 1);
	} else {
		atomic_set(&o_flag, 0);
	}

	/* set the flag */
	mutex_lock(&uplink_event_flag_mutex);
	uplink_event_flag |= BMMDRV_ULEVT_FLAG_O_ACTIVE;
	mutex_unlock(&uplink_event_flag_mutex);
	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);

	return 0;
}

int bsx_algo_o_set_delay(u64 ns)
{
	int value = (int)ns/1000/1000;

	bmm050d_delay = value;
	/* set the flag */
	mutex_lock(&uplink_event_flag_mutex);
	uplink_event_flag |= BMMDRV_ULEVT_FLAG_O_DELAY;
	mutex_unlock(&uplink_event_flag_mutex);
	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);

	return 0;
}

int bsx_algo_o_open_report_data(int open)
{
	return 0;
}

int bsx_algo_o_get_data(int* x ,int* y,int* z, int* status)
{
	mutex_lock(&sensor_data_mutex);

	*x = sensor_data[8];
	*y = sensor_data[9];
	*z = sensor_data[10];
	*status = sensor_data[11];

	mutex_unlock(&sensor_data_mutex);

	return 0;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
int bsx_algo_gyro_open_report_data(int open)
{
        //should queuq work to report event if  is_report_input_direct=true
        return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

int bsx_algo_gyro_enable_nodata(int en)
{

        if(en == 1) {
                atomic_set(&g_flag, 1);
        } else {
                atomic_set(&g_flag, 0);
        }

        /* set the flag */
        mutex_lock(&uplink_event_flag_mutex);
        uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_ACTIVE;
        mutex_unlock(&uplink_event_flag_mutex);
        /* wake up the wait queue */
        wake_up(&uplink_event_flag_wq);

        return 0;
}

int bsx_algo_gyro_set_delay(u64 ns)
{
        int value = (int)ns/1000/1000 ;

        m4g_delay = value;

        /* set the flag */
        mutex_lock(&uplink_event_flag_mutex);
        uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_DELAY;
        mutex_unlock(&uplink_event_flag_mutex);

        /* wake up the wait queue */
        wake_up(&uplink_event_flag_wq);

        return 0;
}

int bsx_algo_gyro_get_data(int* x ,int* y,int* z, int* status)
{

        mutex_lock(&sensor_data_mutex);

        *x = m4g_data[0];
        *y = m4g_data[1];
        *z = m4g_data[2];
        *status = m4g_data[3];

        mutex_unlock(&sensor_data_mutex);

        return 0;
}

/*----------------------------------------------------------------------------*/
static int bma255_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct bma255_i2c_data *obj;
	//tad3sgh add ++
	struct acc_control_path ctl={0};
	struct acc_data_path data={0};
#ifdef BMA255_ENABLE_INT1
	struct hwmsen_object sldobj; /* slope detector */
#endif
#ifdef BMC050_M4G
		struct hwmsen_object sobj_g;
#endif //BMC050_M4G
#ifdef BMC050_VRV
		struct hwmsen_object sobj_vrv;
#endif //BMC050_VRV
#ifdef BMC050_VLA
		struct hwmsen_object sobj_vla;
#endif //BMC050_VLA
#ifdef BMC050_VG
		struct hwmsen_object sobj_vg;
#endif //BMC050_VG
//tad3sgh add --
	int err = 0;
	
	GSE_FUN();

#ifdef BMA255_ENABLE_INT1
	of_get_BMA255_platform_data(&client->dev);
	/* configure the gpio pins */
	err = gpio_request_one(bma255_int_gpio_number, GPIOF_IN,
				 "accel_int");
	if (err < 0) {
		GSE_ERR("Unable to request gpio int_pin\n");
		goto exit;
	}
#endif

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct bma255_i2c_data));

	obj->hw = hw;
	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if(err)
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
#ifdef BMA255_ENABLE_INT1
	INIT_WORK(&obj->irq_work, bma255_irq_work_func);
#endif	
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	mutex_init(&obj->lock);
	//tad3sgh add ++
	mutex_init(&sensor_data_mutex);
	mutex_init(&uplink_event_flag_mutex);
	
	init_waitqueue_head(&uplink_event_flag_wq);
	//tad3sgh add --
	
#ifdef CONFIG_BMA255_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}
	
	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}
	
#endif

	bma255_i2c_client = new_client;	
	err = bma255_init_client(new_client, 1);
	if(err)
	{
		goto exit_init_failed;
	}
	
	err = misc_register(&bma255_device);
	if(err)
	{
		GSE_ERR("bma255_device register failed\n");
		goto exit_misc_device_register_failed;
	}


#if defined(BMA255_ENABLE_INT1)
	bma255_int_init(client);
	gpio_direction_input(bma255_int_gpio_number);
	err = request_irq(bma255_irq, bma255_irq_handler, IRQF_TRIGGER_RISING, "gse_1-eint", obj);
	if (err) {
		GSE_ERR("could not request irq");
		goto exit_kfree;
	}
	obj->sld_duration = (unsigned char) SLOPE_DURATION_VALUE;
	obj->sld_threshold = (unsigned char) SLOPE_THRESHOLD_VALUE;
#endif

	err = bma255_create_attr(&(bma255_init_info.platform_diver_addr->driver));
	if(err) {
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.open_report_data= bma255_open_report_data;
	ctl.enable_nodata = bma255_enable_nodata;
	ctl.set_delay  = bma255_set_delay;
	ctl.is_report_input_direct = false;

	err = acc_register_control_path(&ctl);
	if(err)
	{
		GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data = bma255_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if(err)
	{
		GSE_ERR("register acc data path err\n");
		goto exit_kfree;
	}

#ifdef BMC050_M4G
	sobj_g.self = obj;
	sobj_g.polling = 1;
	sobj_g.sensor_operate = bmm050_m4g_operate;
	err = hwmsen_attach(ID_GYROSCOPE, &sobj_g);
	if(err)
	{
		GSE_ERR( "attach fail = %d\n", err);
		goto exit_kfree;
	}
#endif //BMC050_M4G

#ifdef BMC050_VRV
	sobj_vrv.self = obj;
	sobj_vrv.polling = 1;
	sobj_vrv.sensor_operate = bmm050_vrv_operate;
	err = hwmsen_attach(ID_ROTATION_VECTOR, &sobj_vrv);
	if(err)
	{
		GSE_ERR( "attach fail = %d\n", err);
		goto exit_kfree;
	}
#endif //BMC050_VRV

#ifdef BMC050_VLA
	sobj_vla.self = obj;
	sobj_vla.polling = 1;
	sobj_vla.sensor_operate = bmm050_vla_operate;
	err = hwmsen_attach(ID_LINEAR_ACCELERATION, &sobj_vla);
	if(err)
	{
		GSE_ERR( "attach fail = %d\n", err);
		goto exit_kfree;
	}
#endif //BMC050_VLA

#ifdef BMC050_VG
	sobj_vg.self = obj;
	sobj_vg.polling = 1;
	sobj_vg.sensor_operate = bmm050_vg_operate;
	err = hwmsen_attach(ID_GRAVITY, &sobj_vg);
	if(err)
	{
		GSE_ERR( "attach fail = %d\n", err);
		goto exit_kfree;
	}
#endif //BMC050_VG

#ifdef BMA255_ENABLE_INT1
	sldobj.self = obj;
	sldobj.polling = 0;
	sldobj.sensor_operate = sld_sensor_operate;
	err = hwmsen_attach(ID_SLOPE_DETECTOR, &sldobj);
	if(err)
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}
	mutex_init(&obj->enable_slop_mutex);
#endif

//tad3sgh add --
#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = bma255_early_suspend,
	obj->early_drv.resume   = bma255_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 

	bma255_init_flag =0;
	GSE_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&bma255_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
	GSE_ERR("%s: err = %d\n", __func__, err);        
	bma255_init_flag =-1;
	return err;
}

/*----------------------------------------------------------------------------*/
static int bma255_i2c_remove(struct i2c_client *client)
{
	int err = 0;	
	
	err = bma255_delete_attr(&(bma255_init_info.platform_diver_addr->driver));
	if(err) {
		GSE_ERR("bma150_delete_attr fail: %d\n", err);
	}
	err = misc_deregister(&bma255_device);
	if(err)
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}
#ifdef BMA255_ENABLE_INT1
	err = hwmsen_detach(ID_SLOPE_DETECTOR);
	if(err)
    {
		GSE_ERR("hwmsen_detach fail: %d\n", err);
    }
#endif
	bma255_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int bma255_local_init(void) 
{
	GSE_FUN();

	BMA255_power(hw, 1);
	if(i2c_add_driver(&bma255_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	if(-1 == bma255_init_flag)
	{
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int bma255_remove(void)
{
    GSE_FUN();    
    BMA255_power(hw, 0);    
    i2c_del_driver(&bma255_i2c_driver);
    return 0;
}

static struct acc_init_info bma255_init_info = {
	.name = "bma255",
	.init = bma255_local_init,
	.uninit = bma255_remove,
};

/*----------------------------------------------------------------------------*/
static int __init bma255_init(void)
{
	GSE_FUN();

	hw = get_accel_dts_func(COMPATIABLE_NAME, hw);
	if (!hw)
	{
		GSE_ERR("get dts info fail\n");
		return -1;
	}

	acc_driver_add(&bma255_init_info);
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit bma255_exit(void)
{
	GSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(bma255_init);
module_exit(bma255_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMA255 I2C driver");

