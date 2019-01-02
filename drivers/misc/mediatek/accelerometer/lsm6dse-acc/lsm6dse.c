/* ST LSM6DSE Accelerometer and Gyroscope sensor driver
 *
 *
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/irqchip/mt-eic.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
/* #include <linux/earlysuspend.h> */
#include <linux/platform_device.h>

#include <hwmsensor.h>
#if 0
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#include "lsm6dse.h"
#include <linux/of_irq.h>
#include <mt_gpio.h>
#include <linux/gpio.h>
#include <mach/gpio_const.h>

/* #include <linux/hwmsen_helper.h> */
/* #include <linux/kernel.h> */
/* #include <mach/mt_pm_ldo.h> */
/* #include <cust_eint.h> */
/* #include <mach/eint.h> */



#define POWER_NONE_MACRO -1

#define LSM6DSE_NEW_ARCH	/* kk and L compatialbe */
#ifdef CONFIG_CUSTOM_KERNEL_SIGNIFICANT_MOTION
#define LSM6DSE_STEP_COUNTER	/* it depends on the MACRO LSM6DSE_NEW_ARCH */
#define LSM6DSE_TILT_FUNC	/* dependency on LSM6DSE_STEP_COUNTER */
#define LSM6DSE_SIGNIFICANT_MOTION	/* dependency on LSM6DSE_STEP_COUNTER */
#define LSM6DSE_STEP_DETECT	/* it depends on the MACRO LSM6DSE_NEW_ARCH */
#endif

#ifndef LSM6DSE_NEW_ARCH	/* new sensor type depend on new arch */
#undef LSM6DSE_STEP_COUNTER
#undef LSM6DSE_TILT_FUNC
#undef LSM6DSE_SIGNIFICANT_MOTION
#undef LSM6DSE_STEP_DETECT
#endif

#ifndef LSM6DSE_STEP_COUNTER	/* significant_motion depend on step_counter */
#undef LSM6DSE_SIGNIFICANT_MOTION
#undef LSM6DSE_STEP_DETECT
#endif

#define CALIBRATION_TO_FILE
#ifdef CALIBRATION_TO_FILE
#include <linux/syscalls.h>
#include <linux/fs.h>
#endif

#include <cust_acc.h>

#ifdef LSM6DSE_NEW_ARCH
#include <accel.h>
#ifdef LSM6DSE_STEP_COUNTER	/* step counter */
#include "../../step_counter/step_counter.h"
#endif
#ifdef LSM6DSE_TILT_FUNC	/* tilt detector */
#include "../../tilt_detector_sensor/tilt_detector.h"
#endif

#endif
/****************************************************************/

/*---------------------------------------------------------------------------*/
#define DEBUG 1
#define DEBUG_ST 1


/*----------------------------------------------------------------------------*/
#define CONFIG_LSM6DSE_LOWPASS	/*apply low pass filter on output */
/*----------------------------------------------------------------------------*/
#define LSM6DSE_AXIS_X          0
#define LSM6DSE_AXIS_Y          1
#define LSM6DSE_AXIS_Z          2
#define LSM6DSE_ACC_AXES_NUM        3
#define LSM6DSE_GYRO_AXES_NUM       3
#define LSM6DSE_ACC_DATA_LEN        6
#define LSM6DSE_GYRO_DATA_LEN       6
#define LSM6DSE_ACC_DEV_NAME        "lsm6dse-acc"
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id lsm6dse_i2c_id[] = { {LSM6DSE_ACC_DEV_NAME, 0}, {} };
static struct i2c_board_info i2c_lsm6dse __initdata = {
	I2C_BOARD_INFO(LSM6DSE_ACC_DEV_NAME, 0x6A) };

/*----------------------------------------------------------------------------*/
static int lsm6dse_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lsm6dse_i2c_remove(struct i2c_client *client);
/* static int lsm6dse_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info); */
static int LSM6DSE_init_client(struct i2c_client *client, bool enable);
static int LSM6DSE_acc_SetPowerMode(struct i2c_client *client, bool enable);

static int LSM6DSE_ReadAccRawData(struct i2c_client *client, s16 data[LSM6DSE_ACC_AXES_NUM]);
#ifndef CONFIG_HAS_EARLYSUSPEND

static int lsm6dse_acc_suspend(struct i2c_client *client, pm_message_t msg);
static int lsm6dse_acc_resume(struct i2c_client *client);
#endif
static int LSM6DSE_acc_SetSampleRate(struct i2c_client *client, u8 sample_rate);

#if defined(LSM6DSE_SIGNIFICANT_MOTION) || defined(LSM6DSE_STEP_COUNTER) || defined(LSM6DSE_TILT_FUNC)
static int LSM6DSE_acc_Enable_Func(struct i2c_client *client, LSM6DSE_ACC_GYRO_FUNC_EN_t newValue);
static int LSM6DSE_Int_Ctrl(struct i2c_client *client, LSM6DSE_ACC_GYRO_INT_ACTIVE_t int_act,
			    LSM6DSE_ACC_GYRO_INT_LATCH_CTL_t int_latch);
#endif

#ifdef LSM6DSE_STEP_COUNTER	/* step counter */
static int LSM6DSE_acc_Enable_Pedometer_Func(struct i2c_client *client, bool enable);

static int LSM6DSE_Write_PedoThreshold(struct i2c_client *client, u8 newValue);
static int LSM6DSE_Reset_Pedo_Data(struct i2c_client *client,
				   LSM6DSE_ACC_GYRO_PEDO_RST_STEP_t newValue);
#ifdef LSM6DSE_SIGNIFICANT_MOTION

static int LSM6DSE_Enable_SigMotion_Func(struct i2c_client *client,
					 LSM6DSE_ACC_GYRO_SIGN_MOT_t newValue);
#endif
#endif
#ifdef LSM6DSE_TILT_FUNC	/* tilt detector */

static int LSM6DSE_Enable_Tilt_Func(struct i2c_client *client, bool enable);
static int LSM6DSE_Enable_Tilt_Func_On_Int(struct i2c_client *client,
					   LSM6DSE_ACC_GYRO_ROUNT_INT_t tilt_int, bool enable);


#endif

struct acc_hw acc_cust;

static struct acc_hw *hw = &acc_cust;

struct acc_hw *get_cust_accel(void)
{
	return &acc_cust;
}

static s16 cali_old[LSM6DSE_ACC_AXES_NUM + 1];
/*----------------------------------------------------------------------------*/
typedef enum {
	ADX_TRC_FILTER = 0x01,
	ADX_TRC_RAWDATA = 0x02,
	ADX_TRC_IOCTL = 0x04,
	ADX_TRC_CALI = 0X08,
	ADX_TRC_INFO = 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
typedef enum {
	ACCEL_TRC_FILTER = 0x01,
	ACCEL_TRC_RAWDATA = 0x02,
	ACCEL_TRC_IOCTL = 0x04,
	ACCEL_TRC_CALI = 0X08,
	ACCEL_TRC_INFO = 0X10,
	ACCEL_TRC_DATA = 0X20,
} ACCEL_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor {
	u8 whole;
	u8 fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
	struct scale_factor scalefactor;
	int sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][LSM6DSE_ACC_AXES_NUM];
	int sum[LSM6DSE_ACC_AXES_NUM];
	int num;
	int idx;
};
struct gyro_data_filter {
	s16 raw[C_MAX_FIR_LENGTH][LSM6DSE_GYRO_AXES_NUM];
	int sum[LSM6DSE_GYRO_AXES_NUM];
	int num;
	int idx;
};
/*----------------------------------------------------------------------------*/
struct lsm6dse_i2c_data {
	struct i2c_client *client;
	struct acc_hw *hw;
	struct hwmsen_convert cvt;
	atomic_t layout;
	/*misc */
	/* struct data_resolution *reso; */
	struct work_struct eint_work;
	atomic_t trace;
	atomic_t suspend;
	atomic_t selftest;
	atomic_t filter;
	atomic_t fast_calib_rslt;
	s16 cali_sw[LSM6DSE_GYRO_AXES_NUM + 1];

	/*data */
	s16 offset[LSM6DSE_ACC_AXES_NUM + 1];	/*+1: for 4-byte alignment */
	s16 data[LSM6DSE_ACC_AXES_NUM + 1];

	struct device_node *irq_node;
	int irq;

	int sensitivity;
	int sample_rate;
	int full_scale;
#if defined(CONFIG_LSM6DSE_LOWPASS)
	atomic_t firlen;
	atomic_t fir_en;
	struct data_filter fir;
#endif
	/*early suspend */
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
#endif
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id gs_of_match[] = {
	{.compatible = "mediatek,gsensor",},
	{},
};
#endif
static struct i2c_driver lsm6dse_i2c_driver = {
	.driver = {
		   .name = LSM6DSE_ACC_DEV_NAME,
#ifdef CONFIG_OF
		   .of_match_table = gs_of_match,
#endif
		   },
	.probe = lsm6dse_i2c_probe,
	.remove = lsm6dse_i2c_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = lsm6dse_acc_suspend,
	.resume = lsm6dse_acc_resume,
#endif
	.id_table = lsm6dse_i2c_id,
};

#ifdef LSM6DSE_NEW_ARCH
static int lsm6dse_local_init(void);
static int lsm6dse_local_uninit(void);
static int lsm6dse_acc_init_flag = -1;
static unsigned long lsm6dse_init_flag_test;	/* initial state */
static DEFINE_MUTEX(lsm6dse_init_mutex);
typedef enum {
	LSM6DSE_ACC = 1,
	LSM6DSE_STEP_C = 2,
	LSM6DSE_TILT = 3,
} LSM6DSE_INIT_TYPE;
static struct acc_init_info lsm6dse_init_info = {
	.name = LSM6DSE_ACC_DEV_NAME,
	.init = lsm6dse_local_init,
	.uninit = lsm6dse_local_uninit,
};

#ifdef LSM6DSE_STEP_COUNTER
static int lsm6dse_step_c_local_init(void);
static int lsm6dse_step_c_local_uninit(void);
static struct step_c_init_info lsm6dse_step_c_init_info = {
	.name = "LSM6DSE_STEP_C",
	.init = lsm6dse_step_c_local_init,
	.uninit = lsm6dse_step_c_local_uninit,
};
#endif
#ifdef LSM6DSE_TILT_FUNC
static int lsm6dse_tilt_local_init(void);
static int lsm6dse_tilt_local_uninit(void);
static struct tilt_init_info lsm6dse_tilt_init_info = {
	.name = "LSM6DSE_TILT",
	.init = lsm6dse_tilt_local_init,
	.uninit = lsm6dse_tilt_local_uninit,
};
#endif

#endif
/*----------------------------------------------------------------------------*/
static struct i2c_client *lsm6dse_i2c_client;

#ifndef LSM6DSE_NEW_ARCH
static struct platform_driver lsm6dse_driver;
#endif

static struct lsm6dse_i2c_data *obj_i2c_data;
static bool sensor_power;
static bool enable_status;
static bool pedo_enable_status;
static bool smd_enable_status;
#if defined(LSM6DSE_SIGNIFICANT_MOTION) || defined(LSM6DSE_STEP_COUNTER)
static bool step_d_enable_status;
#endif
static bool tilt_enable_status;


/*----------------------------------------------------------------------------*/

#define GSE_TAG                  "[Gsensor] "

#define GSE_FUN(f)               pr_info(GSE_TAG"%s\n", __func__)
#define GSE_ERR(fmt, args...)    pr_err(GSE_TAG "%s %d : " fmt, __func__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    pr_err(GSE_TAG "%s %d : " fmt, __func__, __LINE__, ##args)

/*----------------------------------------------------------------------------*/

static void LSM6DSE_dumpReg(struct i2c_client *client)
{
	int i = 0;
	u8 addr = 0x10;
	u8 regdata = 0;

	for (i = 0; i < 25; i++) {
		/* dump all */
		hwmsen_read_byte(client, addr, &regdata);
		HWM_LOG("Reg addr=%x regdata=%x\n", addr, regdata);
		addr++;
	}
}

static void LSM6DSE_power(struct acc_hw *hw, unsigned int on)
{
	static unsigned int power_on;

	power_on = on;
}

/*----------------------------------------------------------------------------*/

static int LSM6DSE_acc_ResetCalibration(struct i2c_client *client)
{
	struct lsm6dse_i2c_data *obj = i2c_get_clientdata(client);

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;
}

/*----------------------------------------------------------------------------*/
static int LSM6DSE_acc_ReadCalibration(int *cal_read)
{
	int fd;
	int i;
	int res;
	int ret;
	char *fname = "/persist-lg/sensor/sensor_cal_data.txt";
	mm_segment_t old_fs = get_fs();
	char temp_str[5];

	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_RDONLY, 0);
	if (fd < 0) {
		GSE_ERR("File Open Error\n");
		sys_close(fd);
		return -EINVAL;
	}
	for (i = 0; i < 6; i++) {
		memset(temp_str, 0x00, sizeof(temp_str));
		res = sys_read(fd, temp_str, sizeof(temp_str));
		if (res < 0) {
			GSE_ERR("Read Error\n");
			sys_close(fd);
			return -EINVAL;
		}
		ret = kstrtoint(temp_str, 0, &cal_read[i]);
		if (ret != 0)
			GSE_ERR("kstrtoint read error!!\n");

		GSE_LOG("lsm6dse_calibration_read : cal_read[%d]=%d\n", i, cal_read[i]);
	}
	sys_close(fd);
	set_fs(old_fs);
	GSE_LOG("lsm6dse_calibration_read Done.\n");
	cali_old[0] = cal_read[3];
	cali_old[1] = cal_read[4];
	cali_old[2] = cal_read[5];
	GSE_LOG("data : %d %d %d / %d %d %d\n", cal_read[0], cal_read[1], cal_read[2], cali_old[0],
		cali_old[1], cali_old[2]);

	return 0;
}

/*----------------------------------------------------------------------------*/

static int LSM6DSE_acc_WriteCalibration(int *cal)
{
	int fd;
	int i;
	int res;
	char *fname = "/persist-lg/sensor/sensor_cal_data.txt";
	mm_segment_t old_fs = get_fs();
	char temp_str[5];

	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_WRONLY | O_CREAT | S_IROTH, 0666);
	if (fd < 0) {
		GSE_ERR("File Open Error (%d)\n", fd);
		sys_close(fd);
		return -EINVAL;
	}
	for (i = 0; i < 6; i++) {
		memset(temp_str, 0x00, sizeof(temp_str));
		sprintf(temp_str, "%d", cal[i]);
		res = sys_write(fd, temp_str, sizeof(temp_str));

		if (res < 0) {
			GSE_ERR("Write Error\n");
			sys_close(fd);
			return -EINVAL;
		}
	}
	sys_fsync(fd);
	sys_close(fd);

	sys_chmod(fname, 0664);
	set_fs(old_fs);

	GSE_LOG("lsm6dse_calibration_save Done.\n");

	cali_old[0] = cal[3];
	cali_old[1] = cal[4];
	cali_old[2] = cal[5];
	GSE_LOG("data : %d %d %d / %d %d %d\n", cal[0], cal[1], cal[2], cal[3], cal[4], cal[5]);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int make_cal_data_file(void)
{
	int fd;
	int i;
	int res;
	char *fname = "/persist-lg/sensor/sensor_cal_data.txt";
	mm_segment_t old_fs = get_fs();
	char temp_str[5];
	int cal_misc[6] = { 0, };

	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_RDONLY, 0);	/* Cal file exist check */

	if (fd == -2)
		GSE_LOG("Open Cal File Error. Need to make file (%d)\n", fd);

	if (fd < 0) {
		fd = sys_open(fname, O_WRONLY | O_CREAT | S_IROTH, 0666);
		if (fd < 0) {
			GSE_ERR("Open or Make Cal File Error (%d)\n", fd);
			sys_close(fd);
			return -EINVAL;
		}

		for (i = 0; i < 6; i++) {
			memset(temp_str, 0x00, sizeof(temp_str));
			sprintf(temp_str, "%d", cal_misc[i]);
			res = sys_write(fd, temp_str, sizeof(temp_str));

			if (res < 0) {
				GSE_ERR("Write Cal Error\n");
				sys_close(fd);
				return -EINVAL;
			}
		}
		GSE_LOG("make_cal_data_file Done.\n");
	} else
		GSE_LOG("Sensor Cal File exist.\n");

	sys_fsync(fd);
	sys_close(fd);

	sys_chmod(fname, 0664);
	set_fs(old_fs);
	return LSM6DSE_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int LSM6DSE_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);
	databuf[0] = LSM6DSE_FIXED_DEVID;

	res = hwmsen_read_byte(client, LSM6DSE_WHO_AM_I, databuf);
	GSE_LOG(" LSM6DSE  id %x!\n", databuf[0]);
	if (databuf[0] != LSM6DSE_FIXED_DEVID)
		return LSM6DSE_ERR_IDENTIFICATION;

	if (res < 0)
		return LSM6DSE_ERR_I2C;

	return LSM6DSE_SUCCESS;
}

#ifdef LSM6DSE_TILT_FUNC	/* tilt detector */
static int LSM6DSE_enable_tilt(struct i2c_client *client, bool enable)
{
	int res = 0;
	struct lsm6dse_i2c_data *obj = i2c_get_clientdata(client);	/* obj_i2c_data; */

	if (enable) {
		/* set ODR to 26 hz */
		/* res = LSM6DSE_acc_SetSampleRate(client, LSM6DSE_ACC_ODR_26HZ); */
		res = LSM6DSE_acc_SetSampleRate(client, obj->sample_rate);
		if (LSM6DSE_SUCCESS == res)
			GSE_LOG(" %s set 26hz odr to acc\n", __func__);

		res = LSM6DSE_Enable_Tilt_Func(client, enable);
		if (res != LSM6DSE_SUCCESS) {
			GSE_LOG(" LSM6DSE_Enable_Tilt_Func failed!\n");
			return LSM6DSE_ERR_STATUS;
		}

		res = LSM6DSE_acc_Enable_Func(client, LSM6DSE_ACC_GYRO_FUNC_EN_ENABLED);
		if (res != LSM6DSE_SUCCESS) {
			GSE_LOG(" LSM6DSE_acc_Enable_Func failed!\n");
			return LSM6DSE_ERR_STATUS;
		}

		/* default route to INT1 */
		res = LSM6DSE_Enable_Tilt_Func_On_Int(client, LSM6DSE_ACC_GYRO_INT1, true);
		if (res != LSM6DSE_SUCCESS) {
			GSE_LOG(" LSM6DSE_Enable_Tilt_Func_On_Int failed!\n");
			return LSM6DSE_ERR_STATUS;
		}
		enable_irq(obj_i2c_data->irq);
	} else {
		res = LSM6DSE_Enable_Tilt_Func(client, enable);
		if (res != LSM6DSE_SUCCESS) {
			GSE_LOG(" LSM6DSE_Enable_Tilt_Func failed!\n");
			return LSM6DSE_ERR_STATUS;
		}
		if (!enable_status && !pedo_enable_status) {
			res = LSM6DSE_acc_SetPowerMode(client, false);
			if (res != LSM6DSE_SUCCESS) {
				GSE_LOG(" LSM6DSE_acc_SetPowerMode failed!\n");
				return LSM6DSE_ERR_STATUS;
			}
		}
		disable_irq(obj_i2c_data->irq);
	}
/* tilt_enable_status = enable; */
	return LSM6DSE_SUCCESS;
}
#endif

#ifdef LSM6DSE_STEP_COUNTER	/* step counter */
static int LSM6DSE_enable_pedo(struct i2c_client *client, bool enable)
{
/* u8 databuf[2] = {0}; */
	int res = 0;
	struct lsm6dse_i2c_data *obj = i2c_get_clientdata(client);	/* obj_i2c_data; */

	if (true == enable) {
		/* software reset */
		/* set ODR to 26 hz */
		/* res = LSM6DSE_acc_SetSampleRate(client, LSM6DSE_ACC_ODR_26HZ); */
		res = LSM6DSE_acc_SetSampleRate(client, obj->sample_rate);
		if (LSM6DSE_SUCCESS == res)
			GSE_LOG(" %s set 26hz odr to acc\n", __func__);

		/* enable tilt feature and pedometer feature */
		res = LSM6DSE_acc_Enable_Pedometer_Func(client, enable);
		if (res != LSM6DSE_SUCCESS) {
			GSE_LOG(" LSM6DSE_acc_Enable_Pedometer_Func failed!\n");
			return LSM6DSE_ERR_STATUS;
		}

		res = LSM6DSE_acc_Enable_Func(client, LSM6DSE_ACC_GYRO_FUNC_EN_ENABLED);
		if (res != LSM6DSE_SUCCESS) {
			GSE_LOG(" LSM6DSE_acc_Enable_Func failed!\n");
			return LSM6DSE_ERR_STATUS;
		}
		res = LSM6DSE_Write_PedoThreshold(client, 0x11);	/* set threshold to a certain value here */
		if (res != LSM6DSE_SUCCESS) {
			GSE_LOG(" LSM6DSE_Write_PedoThreshold failed!\n");
			return LSM6DSE_ERR_STATUS;
		}
		res = LSM6DSE_Reset_Pedo_Data(client, LSM6DSE_ACC_GYRO_PEDO_RST_STEP_ENABLED);

		if (res != LSM6DSE_SUCCESS) {
			GSE_LOG(" LSM6DSE_Reset_Pedo_Data failed!\n");
			return LSM6DSE_ERR_STATUS;
		}
	} else {
		res = LSM6DSE_acc_Enable_Pedometer_Func(client, enable);
		if (res != LSM6DSE_SUCCESS) {
			GSE_LOG(" LSM6DSE_acc_Enable_Func failed at disable pedo!\n");
			return LSM6DSE_ERR_STATUS;
		}
		/* do not turn off the func */
		if (!enable_status && !tilt_enable_status) {
			res = LSM6DSE_acc_SetPowerMode(client, false);
			if (res != LSM6DSE_SUCCESS) {
				GSE_LOG(" LSM6DSE_acc_SetPowerMode failed at disable pedo!\n");
				return LSM6DSE_ERR_STATUS;
			}
		}
	}
	return LSM6DSE_SUCCESS;
}
#endif

static int LSM6DSE_acc_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = { 0 };
	int res = 0;
	struct lsm6dse_i2c_data *obj = i2c_get_clientdata(client);	/* obj_i2c_data; */

	if (enable == sensor_power) {
		GSE_LOG("Sensor power status is newest!\n");
		return LSM6DSE_SUCCESS;
	}

	if (hwmsen_read_byte(client, LSM6DSE_CTRL1_XL, databuf)) {
		GSE_ERR("read lsm6dse power ctl register err!\n");
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("LSM6DSE_CTRL1_XL:databuf[0] =  %x!\n", databuf[0]);


	if (true == enable) {
		databuf[0] &= ~LSM6DSE_ACC_ODR_MASK;	/* clear lsm6dse gyro ODR bits */
		databuf[0] |= obj->sample_rate;	/* LSM6DSE_ACC_ODR_104HZ; //default set 100HZ for LSM6DSE acc */

		/*reset full scale */
		databuf[0] &= ~LSM6DSE_ACC_RANGE_MASK;
		databuf[0] |= obj->full_scale;
	} else {
		/* do nothing */
		databuf[0] &= ~LSM6DSE_ACC_ODR_MASK;	/* clear lsm6dse acc ODR bits */
		databuf[0] |= LSM6DSE_ACC_ODR_POWER_DOWN;
	}
	databuf[1] = databuf[0];
	databuf[0] = LSM6DSE_CTRL1_XL;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_LOG("LSM6DSE set power mode: ODR 100hz failed!\n");
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("set LSM6DSE gyro power mode:ODR 100HZ ok %d!\n", enable);

	sensor_power = enable;

	return LSM6DSE_SUCCESS;
}


/*----------------------------------------------------------------------------*/
static int LSM6DSE_acc_SetFullScale(struct i2c_client *client, u8 acc_fs)
{
	u8 databuf[2] = { 0 };
	int res = 0;
	struct lsm6dse_i2c_data *obj = i2c_get_clientdata(client);

	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DSE_CTRL1_XL, databuf)) {
		GSE_ERR("read LSM6DSE_CTRL1_XL err!\n");
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("read  LSM6DSE_CTRL1_XL register: 0x%x\n", databuf[0]);

	obj->full_scale = acc_fs;

	databuf[0] &= ~LSM6DSE_ACC_RANGE_MASK;	/* clear */
	databuf[0] |= acc_fs;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DSE_CTRL1_XL;

	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("write full scale register err!\n");
		return LSM6DSE_ERR_I2C;
	}
	switch (acc_fs) {
	case LSM6DSE_ACC_RANGE_2g:
		obj->sensitivity = LSM6DSE_ACC_SENSITIVITY_2G;
		break;
	case LSM6DSE_ACC_RANGE_4g:
		obj->sensitivity = LSM6DSE_ACC_SENSITIVITY_4G;
		break;
	case LSM6DSE_ACC_RANGE_8g:
		obj->sensitivity = LSM6DSE_ACC_SENSITIVITY_8G;
		break;
	case LSM6DSE_ACC_RANGE_16g:
		obj->sensitivity = LSM6DSE_ACC_SENSITIVITY_16G;
		break;
	default:
		obj->sensitivity = LSM6DSE_ACC_SENSITIVITY_2G;
		break;
	}

	if (hwmsen_read_byte(client, LSM6DSE_CTRL9_XL, databuf)) {
		GSE_ERR("read LSM6DSE_CTRL9_XL err!\n");
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("read  LSM6DSE_CTRL9_XL register: 0x%x\n", databuf[0]);

	databuf[0] &= ~LSM6DSE_ACC_ENABLE_AXIS_MASK;	/* clear */
	databuf[0] |=
	    LSM6DSE_ACC_ENABLE_AXIS_X | LSM6DSE_ACC_ENABLE_AXIS_Y | LSM6DSE_ACC_ENABLE_AXIS_Z;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DSE_CTRL9_XL;

	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("write full scale register err!\n");
		return LSM6DSE_ERR_I2C;
	}

	return LSM6DSE_SUCCESS;
}

/*----------------------------------------------------------------------------*/
/* set the acc sample rate */
static int LSM6DSE_acc_SetSampleRate(struct i2c_client *client, u8 sample_rate)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GSE_FUN();

	/* set Sample Rate will enable power and should changed power status */
	res = LSM6DSE_acc_SetPowerMode(client, true);
	if (res != LSM6DSE_SUCCESS)
		return res;

	if (hwmsen_read_byte(client, LSM6DSE_CTRL1_XL, databuf)) {
		GSE_ERR("read acc data format register err!\n");
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);

	databuf[0] &= ~LSM6DSE_ACC_ODR_MASK;	/* clear */
	databuf[0] |= sample_rate;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DSE_CTRL1_XL;

	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("write sample rate register err!\n");
		return LSM6DSE_ERR_I2C;
	}

	return LSM6DSE_SUCCESS;
}

#ifdef LSM6DSE_TILT_FUNC	/* tilt detector */
static int LSM6DSE_Enable_Tilt_Func(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DSE_TAP_CFG, databuf)) {
		GSE_ERR("read acc data format register err!\n");
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);

	if (enable) {
		databuf[0] &= ~LSM6DSE_TILT_EN_MASK;	/* clear */
		databuf[0] |= LSM6DSE_ACC_GYRO_TILT_EN_ENABLED;
	} else {
		databuf[0] &= ~LSM6DSE_TILT_EN_MASK;	/* clear */
		databuf[0] |= LSM6DSE_ACC_GYRO_TILT_EN_DISABLED;
	}

	databuf[1] = databuf[0];
	databuf[0] = LSM6DSE_TAP_CFG;
	res = i2c_master_send(client, databuf, 0x2);
	if (res < 0) {
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DSE_ERR_I2C;
	}

	return LSM6DSE_SUCCESS;
}
#endif

#ifdef LSM6DSE_SIGNIFICANT_MOTION
static int LSM6DSE_Enable_SigMotion_Func_On_Int(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = { 0 };
	int res = 0;
	u8 op_reg = 0;

	LSM6DSE_ACC_GYRO_FUNC_EN_t func_enable;
	LSM6DSE_ACC_GYRO_SIGN_MOT_t sigm_enable;

	GSE_FUN();

	if (enable) {
		func_enable = LSM6DSE_ACC_GYRO_FUNC_EN_ENABLED;
		sigm_enable = LSM6DSE_ACC_GYRO_SIGN_MOT_ENABLED;

		res = LSM6DSE_acc_Enable_Func(client, func_enable);
		if (res != LSM6DSE_SUCCESS) {
			GSE_LOG(" LSM6DSE_acc_Enable_Func failed!\n");
			return LSM6DSE_ERR_STATUS;
		}
	} else {
		/* func_enable = LSM6DSE_ACC_GYRO_FUNC_EN_DISABLED; */
		sigm_enable = LSM6DSE_ACC_GYRO_SIGN_MOT_DISABLED;
	}

	res = LSM6DSE_Enable_SigMotion_Func(client, sigm_enable);
	if (res != LSM6DSE_SUCCESS) {
		GSE_LOG(" LSM6DSE_acc_Enable_Func failed!\n");
		return LSM6DSE_ERR_STATUS;
	}
	/* Config interrupt for significant motion */

	op_reg = LSM6DSE_INT1_CTRL;

	if (hwmsen_read_byte(client, op_reg, databuf)) {
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);

	if (enable) {
		databuf[0] &= ~LSM6DSE_ACC_GYRO_INT_STEP_D_MASK;	/* clear */
		databuf[0] |= LSM6DSE_ACC_GYRO_INT_STEP_D_ENABLED;
	} else {
		databuf[0] &= ~LSM6DSE_ACC_GYRO_INT_STEP_D_MASK;	/* clear */
		databuf[0] |= LSM6DSE_ACC_GYRO_INT_STEP_D_DISABLED;
	}

	databuf[1] = databuf[0];
	databuf[0] = op_reg;
	res = i2c_master_send(client, databuf, 0x2);
	if (res < 0) {
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DSE_ERR_I2C;
	}
	res = LSM6DSE_Int_Ctrl(client, LSM6DSE_ACC_GYRO_INT_ACTIVE_LOW, LSM6DSE_ACC_GYRO_INT_LATCH);
	if (res < 0) {
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DSE_ERR_I2C;
	}
	return LSM6DSE_SUCCESS;
}
#endif

#ifdef LSM6DSE_STEP_COUNTER
static int LSM6DSE_Enable_Step_Detector_On_Int(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = { 0 };
	int res = 0;
	u8 op_reg = 0;

	/* Config interrupt for significant motion */
	op_reg = LSM6DSE_INT1_CTRL;
	if (hwmsen_read_byte(client, op_reg, databuf)) {
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);

	if (enable) {
		databuf[0] &= ~LSM6DSE_ACC_GYRO_INT_STEP_D_MASK;	/* clear */
		databuf[0] |= LSM6DSE_ACC_GYRO_INT_STEP_D_ENABLED;
	} else {
		databuf[0] &= ~LSM6DSE_ACC_GYRO_INT_STEP_D_MASK;	/* clear */
		databuf[0] |= LSM6DSE_ACC_GYRO_INT_STEP_D_DISABLED;
	}

	databuf[1] = databuf[0];
	databuf[0] = op_reg;
	res = i2c_master_send(client, databuf, 0x2);
	if (res < 0) {
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DSE_ERR_I2C;
	}
	res = LSM6DSE_Int_Ctrl(client, LSM6DSE_ACC_GYRO_INT_ACTIVE_LOW, LSM6DSE_ACC_GYRO_INT_LATCH);
	if (res < 0) {
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DSE_ERR_I2C;
	}
	return LSM6DSE_SUCCESS;
}
#endif

#ifdef LSM6DSE_TILT_FUNC	/* tilt detector */
static int LSM6DSE_Enable_Tilt_Func_On_Int(struct i2c_client *client,
					   LSM6DSE_ACC_GYRO_ROUNT_INT_t tilt_int, bool enable)
{
	u8 databuf[2] = { 0 };
	int res = 0;
	u8 op_reg = 0;

	GSE_FUN();

	if (LSM6DSE_ACC_GYRO_INT1 == tilt_int)
		op_reg = LSM6DSE_MD1_CFG;
	else if (LSM6DSE_ACC_GYRO_INT2 == tilt_int)
		op_reg = LSM6DSE_MD2_CFG;

	if (hwmsen_read_byte(client, op_reg, databuf)) {
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);

	if (enable) {
		databuf[0] &= ~LSM6DSE_ACC_GYRO_INT_TILT_MASK;	/* clear */
		databuf[0] |= LSM6DSE_ACC_GYRO_INT_TILT_ENABLED;
	} else {
		databuf[0] &= ~LSM6DSE_ACC_GYRO_INT_TILT_MASK;	/* clear */
		databuf[0] |= LSM6DSE_ACC_GYRO_INT_TILT_DISABLED;
	}

	databuf[1] = databuf[0];
	databuf[0] = op_reg;
	res = i2c_master_send(client, databuf, 0x2);
	if (res < 0) {
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DSE_ERR_I2C;
	}
	res = LSM6DSE_Int_Ctrl(client, LSM6DSE_ACC_GYRO_INT_ACTIVE_LOW, LSM6DSE_ACC_GYRO_INT_LATCH);
	if (res < 0) {
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DSE_ERR_I2C;
	}

	return LSM6DSE_SUCCESS;
}
#endif

#ifdef LSM6DSE_STEP_COUNTER	/* step counter */
static int LSM6DSE_acc_Enable_Pedometer_Func(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DSE_TAP_CFG, databuf)) {
		GSE_ERR("read acc data format register err!\n");
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);

	if (enable) {
		databuf[0] &= ~LSM6DSE_PEDO_EN_MASK;	/* clear */
		databuf[0] |= LSM6DSE_ACC_GYRO_PEDO_EN_ENABLED;
	} else {
		databuf[0] &= ~LSM6DSE_PEDO_EN_MASK;	/* clear */
		databuf[0] |= LSM6DSE_ACC_GYRO_PEDO_EN_DISABLED;
	}

	databuf[1] = databuf[0];
	databuf[0] = LSM6DSE_TAP_CFG;
	res = i2c_master_send(client, databuf, 0x2);
	if (res < 0) {
		GSE_ERR("write enable pedometer func register err!\n");
		return LSM6DSE_ERR_I2C;
	}

	return LSM6DSE_SUCCESS;
}

#ifdef LSM6DSE_SIGNIFICANT_MOTION
static int LSM6DSE_Set_SigMotion_Threshold(struct i2c_client *client, u8 SigMotion_Threshold)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DSE_FUNC_CFG_ACCESS, databuf)) {
		GSE_ERR("%s read LSM6DSE_CTRL10_C register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);

	databuf[0] = 0x80;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DSE_FUNC_CFG_ACCESS;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("%s write LSM6DSE_CTRL10_C register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}

	databuf[1] = SigMotion_Threshold;
	databuf[0] = LSM6DSE_SM_THS;

	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("%s write LSM6DSE_CTRL10_C register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}

	databuf[1] = 0x00;
	databuf[0] = LSM6DSE_FUNC_CFG_ACCESS;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("%s write LSM6DSE_CTRL10_C register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}
	return LSM6DSE_SUCCESS;
}

static int LSM6DSE_Enable_SigMotion_Func(struct i2c_client *client,
					 LSM6DSE_ACC_GYRO_SIGN_MOT_t newValue)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DSE_CTRL10_C, databuf)) {
		GSE_ERR("%s read LSM6DSE_CTRL10_C register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);

	databuf[0] &= ~LSM6DSE_ACC_GYRO_SIGN_MOT_MASK;	/* clear */
	databuf[0] |= newValue;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DSE_CTRL10_C;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("%s write LSM6DSE_CTRL10_C register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}

	return LSM6DSE_SUCCESS;
}
#endif
#endif

#if defined(LSM6DSE_SIGNIFICANT_MOTION) || defined(LSM6DSE_STEP_COUNTER) || defined(LSM6DSE_TILT_FUNC)
static int LSM6DSE_Int_Ctrl(struct i2c_client *client, LSM6DSE_ACC_GYRO_INT_ACTIVE_t int_act,
			    LSM6DSE_ACC_GYRO_INT_LATCH_CTL_t int_latch)
{
	u8 databuf[2] = { 0 };
	int res = 0;
	u8 op_reg = 0;

	GSE_FUN();

	/* config latch int or no latch */
	op_reg = LSM6DSE_TAP_CFG;
	if (hwmsen_read_byte(client, op_reg, databuf)) {
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);

	databuf[0] &= ~LSM6DSE_ACC_GYRO_INT_LATCH_CTL_MASK;	/* clear */
	databuf[0] |= int_latch;

	databuf[1] = databuf[0];
	databuf[0] = op_reg;
	res = i2c_master_send(client, databuf, 0x2);
	if (res < 0) {
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DSE_ERR_I2C;
	}
	/* config high or low active */
	op_reg = LSM6DSE_CTRL3_C;
	if (hwmsen_read_byte(client, op_reg, databuf)) {
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);

	databuf[0] &= ~LSM6DSE_ACC_GYRO_INT_ACTIVE_MASK;	/* clear */
	databuf[0] |= int_act;

	databuf[1] = databuf[0];
	databuf[0] = op_reg;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DSE_ERR_I2C;
	}

	return LSM6DSE_SUCCESS;
}

static int LSM6DSE_acc_Enable_Func(struct i2c_client *client, LSM6DSE_ACC_GYRO_FUNC_EN_t newValue)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DSE_CTRL10_C, databuf)) {
		GSE_ERR("%s read LSM6DSE_CTRL10_C register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);

	databuf[0] &= ~LSM6DSE_ACC_GYRO_FUNC_EN_MASK;	/* clear */
	databuf[0] |= newValue;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DSE_CTRL10_C;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("%s write LSM6DSE_CTRL10_C register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}

	return LSM6DSE_SUCCESS;
}
#endif

#ifdef LSM6DSE_STEP_COUNTER	/* step counter */
static int LSM6DSE_W_Open_RAM_Page(struct i2c_client *client, LSM6DSE_ACC_GYRO_RAM_PAGE_t newValue)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DSE_RAM_ACCESS, databuf)) {
		GSE_ERR("%s read LSM6DSE_RAM_ACCESS register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);

	databuf[0] &= ~LSM6DSE_RAM_PAGE_MASK;	/* clear */
	databuf[0] |= newValue;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DSE_RAM_ACCESS;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("%s write LSM6DSE_RAM_ACCESS register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}

	return LSM6DSE_SUCCESS;
}

static int LSM6DSE_Write_PedoThreshold(struct i2c_client *client, u8 newValue)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GSE_FUN();

	res = LSM6DSE_W_Open_RAM_Page(client, LSM6DSE_ACC_GYRO_RAM_PAGE_ENABLED);
	if (LSM6DSE_SUCCESS != res)
		return res;

	if (hwmsen_read_byte(client, LSM6DSE_CONFIG_PEDO_THS_MIN, databuf)) {
		GSE_ERR("%s read LSM6DSE_CTRL10_C register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);

	databuf[0] &= ~0x1F;
	databuf[0] |= (newValue & 0x1F);

	databuf[1] = databuf[0];
	databuf[0] = LSM6DSE_CONFIG_PEDO_THS_MIN;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("%s write LSM6DSE_CTRL10_C register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}

	databuf[0] = 0x14;
	databuf[1] = 0x6e;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("%s write LSM6DSE_CTRL10_C register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}
	res = LSM6DSE_W_Open_RAM_Page(client, LSM6DSE_ACC_GYRO_RAM_PAGE_DISABLED);
	if (LSM6DSE_SUCCESS != res) {
		GSE_ERR("%s write LSM6DSE_W_Open_RAM_Page failed!\n", __func__);
		return res;
	}

	return LSM6DSE_SUCCESS;
}

static int LSM6DSE_Reset_Pedo_Data(struct i2c_client *client,
				   LSM6DSE_ACC_GYRO_PEDO_RST_STEP_t newValue)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DSE_CTRL10_C, databuf)) {
		GSE_ERR("%s read LSM6DSE_CTRL10_C register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("%s read acc LSM6DSE_CTRL10_C data format register: 0x%x\n", __func__,
		databuf[0]);

	databuf[0] &= ~LSM6DSE_PEDO_RST_STEP_MASK;	/* clear */
	databuf[0] |= newValue;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DSE_CTRL10_C;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("%s write LSM6DSE_CTRL10_C register err!\n", __func__);
		return LSM6DSE_ERR_I2C;
	}

	return LSM6DSE_SUCCESS;
}

static int LSM6DSE_Get_Pedo_DataReg(struct i2c_client *client, u16 *Value)
{
	u8 databuf[2] = { 0 };

	GSE_FUN();

	if (hwmsen_read_block(client, LSM6DSE_STEP_COUNTER_L, databuf, 2)) {
		GSE_ERR("LSM6DSE read acc data  error\n");
		return -2;
	}

	*Value = (databuf[1] << 8) | databuf[0];

	return LSM6DSE_SUCCESS;
}
#endif
/*----------------------------------------------------------------------------*/
static int LSM6DSE_ReadAccData(struct i2c_client *client, char *buf, int bufsize)
{
	struct lsm6dse_i2c_data *obj = (struct lsm6dse_i2c_data *)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[LSM6DSE_ACC_AXES_NUM];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	if (NULL == buf)
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	if (sensor_power == false) {
		res = LSM6DSE_acc_SetPowerMode(client, true);
		if (res)
			GSE_ERR("Power on lsm6dse error %d!\n", res);

		msleep(20);
	}

	res = LSM6DSE_ReadAccRawData(client, obj->data);
	if (res < 0) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
#if 1
	obj->data[LSM6DSE_AXIS_X] =
	    (long)(obj->data[LSM6DSE_AXIS_X]) * obj->sensitivity / 1000 * GRAVITY_EARTH_1000 / 1000;
	obj->data[LSM6DSE_AXIS_Y] =
	    (long)(obj->data[LSM6DSE_AXIS_Y]) * obj->sensitivity / 1000 *
	    GRAVITY_EARTH_1000 / 1000;
	obj->data[LSM6DSE_AXIS_Z] =
	    (long)(obj->data[LSM6DSE_AXIS_Z]) * obj->sensitivity / 1000 *
	    GRAVITY_EARTH_1000 / 1000;

	obj->data[LSM6DSE_AXIS_X] += obj->cali_sw[LSM6DSE_AXIS_X];
	obj->data[LSM6DSE_AXIS_Y] += obj->cali_sw[LSM6DSE_AXIS_Y];
	obj->data[LSM6DSE_AXIS_Z] += obj->cali_sw[LSM6DSE_AXIS_Z];

	/*remap coordinate */
	acc[obj->cvt.map[LSM6DSE_AXIS_X]] =
	    obj->cvt.sign[LSM6DSE_AXIS_X] * obj->data[LSM6DSE_AXIS_X];
	acc[obj->cvt.map[LSM6DSE_AXIS_Y]] =
	    obj->cvt.sign[LSM6DSE_AXIS_Y] * obj->data[LSM6DSE_AXIS_Y];
	acc[obj->cvt.map[LSM6DSE_AXIS_Z]] =
	    obj->cvt.sign[LSM6DSE_AXIS_Z] * obj->data[LSM6DSE_AXIS_Z];

	/* Out put the mg */
#endif


	sprintf(buf, "%04x %04x %04x", acc[LSM6DSE_AXIS_X], acc[LSM6DSE_AXIS_Y],
		acc[LSM6DSE_AXIS_Z]);

	/* atomic_read(&obj->trace) & ADX_TRC_IOCTL */
	if (atomic_read(&obj->trace) & ADX_TRC_IOCTL) {
		/* GSE_LOG("gsensor data: %s!\n", buf); */
		GSE_LOG("raw data:obj->data:%04x %04x %04x\n", obj->data[LSM6DSE_AXIS_X],
			obj->data[LSM6DSE_AXIS_Y], obj->data[LSM6DSE_AXIS_Z]);
		GSE_LOG("acc:%04x %04x %04x\n", acc[LSM6DSE_AXIS_X], acc[LSM6DSE_AXIS_Y],
			acc[LSM6DSE_AXIS_Z]);

		/* LSM6DSE_dumpReg(client); */
	}

	return 0;
}

static int LSM6DSE_ReadAccRawData(struct i2c_client *client, s16 data[LSM6DSE_ACC_AXES_NUM])
{
	int err = 0;
	char databuf[6] = { 0 };

	if (NULL == client) {
		err = -EINVAL;
	} else {
		if (hwmsen_read_block(client, LSM6DSE_OUTX_L_XL, databuf, 6)) {
			GSE_ERR("LSM6DSE read acc data  error\n");
			return -2;
		}
		data[LSM6DSE_AXIS_X] =
		    (s16) ((databuf[LSM6DSE_AXIS_X * 2 + 1] << 8) |
			   (databuf[LSM6DSE_AXIS_X * 2]));
		data[LSM6DSE_AXIS_Y] =
		    (s16) ((databuf[LSM6DSE_AXIS_Y * 2 + 1] << 8) |
			   (databuf[LSM6DSE_AXIS_Y * 2]));
		data[LSM6DSE_AXIS_Z] =
		    (s16) ((databuf[LSM6DSE_AXIS_Z * 2 + 1] << 8) |
			   (databuf[LSM6DSE_AXIS_Z * 2]));
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int LSM6DSE_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8) * 10);

	if ((NULL == buf) || (bufsize <= 30))
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "LSM6DSE Chip");
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lsm6dse_i2c_client;
	char strbuf[LSM6DSE_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	LSM6DSE_ReadChipInfo(client, strbuf, LSM6DSE_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lsm6dse_i2c_client;
	char strbuf[LSM6DSE_BUFSIZE];
	int x, y, z;
	int ret;

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	LSM6DSE_ReadAccData(client, strbuf, LSM6DSE_BUFSIZE);
	ret = sscanf(strbuf, "%x %x %x", &x, &y, &z);
	return snprintf(buf, PAGE_SIZE, "%d, %d, %d\n", x, y, z);
}

static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lsm6dse_i2c_client;
	s16 data[LSM6DSE_ACC_AXES_NUM] = { 0 };

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	LSM6DSE_ReadAccRawData(client, data);
	return snprintf(buf, PAGE_SIZE, "%x,%x,%x\n", data[0], data[1], data[2]);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lsm6dse_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6dse_i2c_data *obj = obj_i2c_data;
	int trace;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&obj->trace, trace);
	else
		GSE_ERR("invalid content: '%s', length = %zu\n", buf, count);

	return count;
}

static ssize_t show_chipinit_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lsm6dse_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6dse_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}

	LSM6DSE_init_client(obj->client, true);
	LSM6DSE_dumpReg(obj->client);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct lsm6dse_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (obj->hw) {
		len +=
		    snprintf(buf + len, PAGE_SIZE - len,
			     "CUST: i2c_num=%d, direction=%d, sensitivity = %d,(power_id=%d, power_vol=%d)\n",
			     obj->hw->i2c_num, obj->hw->direction, obj->sensitivity,
			     obj->hw->power_id, obj->hw->power_vol);
		LSM6DSE_dumpReg(obj->client);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");
	}
	return len;
}

static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_i2c_data *data = obj_i2c_data;

	if (NULL == data) {
		GSE_ERR("lsm6dse_i2c_data is null!!\n");
		return -1;
	}

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		       data->hw->direction, atomic_read(&data->layout), data->cvt.sign[0],
		       data->cvt.sign[1], data->cvt.sign[2], data->cvt.map[0], data->cvt.map[1],
		       data->cvt.map[2]);
}

/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int layout = 0;
	int ret;
	struct lsm6dse_i2c_data *data = obj_i2c_data;

	if (NULL == data) {
		GSE_ERR("lsm6dse_i2c_data is null!!\n");
		return count;
	}

	ret = kstrtoint(buf, 0, &layout);
	if (ret == 0) {
		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt)) {
			GSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		} else if (!hwmsen_get_convert(data->hw->direction, &data->cvt)) {
			GSE_ERR("invalid layout: %d, restore to %d\n", layout,
			       data->hw->direction);
		} else {
			GSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	} else {
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}


static int lsm6dse_update_reg(struct i2c_client *client, u8 reg, u8 mask, u8 value)
{
	int ret = 0;
	u8 v;

	ret = hwmsen_read_byte(client, reg, &v);
	if (ret != 0)
		return -1;

	v &= ~((u8) mask);
	v |= value;

	ret = hwmsen_write_byte(client, reg, v);
	if (ret != 0)
		return -1;

	return ret;
}

static int lsm6dse_do_xl_selftest(const struct lsm6dse_i2c_data *priv)
{
	int status = 0;
	u8 testset_regs[10] = { 0x30, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00 };
	u8 backup_regs[10];
	u8 readBuffer[1] = { 0x00, };
	s16 nOutData[3] = { 0, };
	s32 NOST[3] = { 0, }, ST[3] = {
	0,};
	s32 i, retry;
	u8 init_status = 0;
	struct i2c_client *client = NULL;

	if (priv == NULL)
		return -EINVAL;

	client = priv->client;

	/* check WHO_AM_I */
	status = hwmsen_read_block(client, LSM6DSE_WHO_AM_I, readBuffer, 1);
	if ((status == 0) && (readBuffer[0] == LSM6DSE_FIXED_DEVID))
		init_status = 1;
	else
		return LSM6DSE_ERR_IDENTIFICATION;

	/* backup registers */
	status = hwmsen_read_block(client, LSM6DSE_CTRL1_XL, backup_regs, 5);
	if (status != 0)
		goto XL_HW_SELF_EXIT;

	status = hwmsen_read_block(client, LSM6DSE_CTRL6_C, &backup_regs[5], 5);
	if (status != 0)
		goto XL_HW_SELF_EXIT;

	/* self-test setting #1 */
	status = hwmsen_write_block(client, LSM6DSE_CTRL1_XL, testset_regs, 5);
	if (status != 0)
		goto XL_HW_SELF_EXIT;

	status = hwmsen_write_block(client, LSM6DSE_CTRL6_C, &testset_regs[5], 5);
	if (status != 0)
		goto XL_HW_SELF_EXIT;

	mdelay(200);

	retry = LSM6DSE_ACC_RETRY_CNT;
	do {
		mdelay(5);
		status = hwmsen_read_block(client, LSM6DSE_STATUS_REG, readBuffer, 1);
		if (status != 0)
			goto XL_HW_SELF_EXIT;

		retry--;
		if (!retry)
			break;
	} while (!(readBuffer[0] & LSM6DSE_STATUS_REG_XLDA));

	status = hwmsen_read_block(client, LSM6DSE_OUTX_L_XL, (u8 *) nOutData, 6);
	if (status != 0)
		goto XL_HW_SELF_EXIT;

	for (i = 0; i < 6; i++) {
		retry = LSM6DSE_ACC_RETRY_CNT;
		do {
			mdelay(5);
			status = hwmsen_read_block(client, LSM6DSE_STATUS_REG, readBuffer, 1);
			if (status != 0)
				goto XL_HW_SELF_EXIT;

			retry--;
			if (!retry)
				break;
		} while (!(readBuffer[0] & LSM6DSE_STATUS_REG_XLDA));

		status = hwmsen_read_block(client, LSM6DSE_OUTX_L_XL, (u8 *) nOutData, 6);
		if (status != 0)
			goto XL_HW_SELF_EXIT;

#if DEBUG_ST == 1
		GSE_LOG("NOST sample[%d] :  %d %d %d\n", i, nOutData[0], nOutData[1], nOutData[2]);
#endif
		if (i > 0) {
			NOST[0] += nOutData[0];
			NOST[1] += nOutData[1];
			NOST[2] += nOutData[2];
		}
	}

	NOST[0] /= 5;
	NOST[1] /= 5;
	NOST[2] /= 5;

	/* self-test setting #2 */
	status =
	    lsm6dse_update_reg(client, LSM6DSE_CTRL5_C, LSM6DSE_ST_XL_MASK, LSM6DSE_ST_XL_ON_POS);
	if (status != 0)
		goto XL_HW_SELF_EXIT;

	mdelay(200);

	retry = LSM6DSE_ACC_RETRY_CNT;
	do {
		mdelay(5);
		status = hwmsen_read_block(client, LSM6DSE_STATUS_REG, readBuffer, 1);
		if (status != 0)
			goto XL_HW_SELF_EXIT;

		retry--;
		if (!retry)
			break;
	} while (!(readBuffer[0] & LSM6DSE_STATUS_REG_XLDA));

	status = hwmsen_read_block(client, LSM6DSE_OUTX_L_XL, (u8 *) nOutData, 6);
	if (status != 0)
		goto XL_HW_SELF_EXIT;

	for (i = 0; i < 6; i++) {
		retry = LSM6DSE_ACC_RETRY_CNT;
		do {
			mdelay(5);
			status = hwmsen_read_block(client, LSM6DSE_STATUS_REG, readBuffer, 1);
			if (status != 0)
				goto XL_HW_SELF_EXIT;

			retry--;
			if (!retry)
				break;
		} while (!(readBuffer[0] & LSM6DSE_STATUS_REG_XLDA));

		status = hwmsen_read_block(client, LSM6DSE_OUTX_L_XL, (u8 *) nOutData, 6);
		if (status != 0)
			goto XL_HW_SELF_EXIT;

#if DEBUG_ST == 1
		GSE_LOG("ST sample[%d] :  %d %d %d\n", i, nOutData[0], nOutData[1], nOutData[2]);
#endif

		if (i > 0) {
			ST[0] += nOutData[0];
			ST[1] += nOutData[1];
			ST[2] += nOutData[2];
		}
	}

	ST[0] /= 5;
	ST[1] /= 5;
	ST[2] /= 5;

	status = lsm6dse_update_reg(client, LSM6DSE_CTRL1_XL, 0xff, 0x00);
	if (status != 0)
		goto XL_HW_SELF_EXIT;

	status = lsm6dse_update_reg(client, LSM6DSE_CTRL5_C, LSM6DSE_ST_XL_MASK, LSM6DSE_ST_XL_OFF);
	if (status != 0)
		goto XL_HW_SELF_EXIT;

	retry = 0;
	for (i = 0; i < 3; i++) {
		ST[i] = abs(ST[i] - NOST[i]);

#if DEBUG_ST == 1
		GSE_LOG("ST Delta[%d] :  %d\n", i, ST[i]);
#endif

		if ((LSM6DSE_ACC_MIN_ST_2G > ST[i])
		    || (LSM6DSE_ACC_MAX_ST_2G < ST[i])) {
			retry++;
		}
	}

	if (retry > 0)
		status = -1;
	else
		status = 0;

XL_HW_SELF_EXIT:

	/* restore registers */
	hwmsen_write_block(client, LSM6DSE_CTRL1_XL, backup_regs, 5);
	hwmsen_write_block(client, LSM6DSE_CTRL6_C, &backup_regs[5], 5);

	mdelay(200);

	return status;

}

static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	struct lsm6dse_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->selftest));
	return res;
}

static ssize_t store_selftest_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int st_result = 0;
	int ret;

	struct lsm6dse_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	ret = kstrtoint(buf, 0, &st_result);
	if (ret == 0) {
		st_result = lsm6dse_do_xl_selftest(obj);
		atomic_set(&obj->selftest, st_result);
	} else {
		GSE_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}

	return count;
}

static int lsm6dse_do_xl_calibration(struct lsm6dse_i2c_data *priv)
{
	int res, i = 0;
	int cali[6];
	int sum[3] = { 0 };
	long cnvt[3];
	s16 data[3] = { 0 }, data_prev[3] = {
	0};
	struct i2c_client *client;
	s16 cali_backup[3];

	if (priv == NULL)
		return -EINVAL;

	client = priv->client;

	/*backup calibrated data */
	memcpy(cali_backup, priv->cali_sw, sizeof(cali_backup));

	if (sensor_power == false) {
		res = LSM6DSE_acc_SetPowerMode(client, true);
		if (res) {
			GSE_ERR("Power on lsm6dse error %d!\n", res);
			return LSM6DSE_ERR_SETUP_FAILURE;
		}
		msleep(20);
	}

	res = LSM6DSE_ReadAccRawData(client, data_prev);
	if (res < 0) {
		GSE_ERR("I2C error: ret value=%d", res);
		return LSM6DSE_ERR_SETUP_FAILURE;
	}

	for (i = 0; i < LSM6DSE_CALI_DATA_NUM; i++) {
		mdelay(20);
		res = LSM6DSE_ReadAccRawData(client, data);
		if (res < 0) {
			GSE_ERR("I2C error: ret value=%d", res);
			return LSM6DSE_ERR_SETUP_FAILURE;
		}
#if DEBUG_ST == 1
		GSE_LOG("sample[%d] = %d %d %d\n", i, data[0], data[1], data[2]);
#endif

		if ((abs(data[0] - data_prev[0]) > LSM6DSE_SHAKING_DETECT_THRESHOLD)
		    || (abs(data[1] - data_prev[1]) > LSM6DSE_SHAKING_DETECT_THRESHOLD)
		    || (abs(data[2] - data_prev[2]) > LSM6DSE_SHAKING_DETECT_THRESHOLD)) {
			GSE_LOG("===============shaking x===============\n");
		} else {
			sum[0] += data[0];
			sum[1] += data[1];
			sum[2] += data[2];

			memcpy(data_prev, data, sizeof(data));
		}

	}

	GSE_LOG("===============complete shaking x check===============\n");

#if DEBUG_ST == 1
	GSE_LOG("average :  %d %d %d\n", sum[0] / LSM6DSE_CALI_DATA_NUM,
		sum[1] / LSM6DSE_CALI_DATA_NUM, sum[2] / LSM6DSE_CALI_DATA_NUM);
#endif

	if ((abs(sum[0]) / LSM6DSE_CALI_DATA_NUM > LSM6DSE_TESTLIMIT_XY)
	    || (abs(sum[1]) / LSM6DSE_CALI_DATA_NUM > LSM6DSE_TESTLIMIT_XY)
	    || (abs(sum[2]) / LSM6DSE_CALI_DATA_NUM > LSM6DSE_TESTLIMIT_Z_USL_LSB)
	    || (abs(sum[2]) / LSM6DSE_CALI_DATA_NUM < LSM6DSE_TESTLIMIT_Z_LSL_LSB)) {
		GSE_ERR("I2C error: ret value=%d", res);
		return LSM6DSE_ERR_SETUP_FAILURE;
	}

	cnvt[0] = -1 * sum[0] / LSM6DSE_CALI_DATA_NUM;
	cnvt[1] = -1 * sum[1] / LSM6DSE_CALI_DATA_NUM;

	if (sum[2] < 0)
		cnvt[2] = -1 * (sum[2] / LSM6DSE_CALI_DATA_NUM + 8192);
	else
		cnvt[2] = -1 * (sum[2] / LSM6DSE_CALI_DATA_NUM - 8192);

	cnvt[0] = cnvt[0] * priv->sensitivity / 100 * GRAVITY_EARTH_1000 / 10000;
	cnvt[1] = cnvt[1] * priv->sensitivity / 100 * GRAVITY_EARTH_1000 / 10000;
	cnvt[2] = cnvt[2] * priv->sensitivity / 100 * GRAVITY_EARTH_1000 / 10000;

	priv->cali_sw[0] = (s16) cnvt[0];
	priv->cali_sw[1] = (s16) cnvt[1];
	priv->cali_sw[2] = (s16) cnvt[2];
	cali[3] = (int)cali_backup[0];
	cali[4] = (int)cali_backup[1];
	cali[5] = (int)cali_backup[2];

	cali[0] = (int)priv->cali_sw[0];
	cali[1] = (int)priv->cali_sw[1];
	cali[2] = (int)priv->cali_sw[2];

#ifdef CALIBRATION_TO_FILE
	LSM6DSE_acc_WriteCalibration(cali);
#endif

#if DEBUG_ST == 1
	GSE_LOG("updated cali_sw : %d %d %d\n", priv->cali_sw[0],
		priv->cali_sw[1], priv->cali_sw[2]);
#endif

	return LSM6DSE_SUCCESS;
}

static ssize_t store_run_fast_calibration_value(struct device_driver *ddri, const char *buf,
						size_t count)
{
	struct lsm6dse_i2c_data *data = obj_i2c_data;
	int do_calibrate = 0;
	int res = 0;
	int ret;

	if (NULL == data) {
		GSE_ERR("lsm6dse_i2c_data is null!!\n");
		return count;
	}

	ret = kstrtoint(buf, 0, &do_calibrate);
	if (ret != 0)
		GSE_ERR("kstrtoint read error!!\n");

	res = lsm6dse_do_xl_calibration(data);
	if (LSM6DSE_SUCCESS == res)
		atomic_set(&data->fast_calib_rslt, LSM6DSE_SUCCESS);
	else if (LSM6DSE_ERR_SETUP_FAILURE == res) {
		atomic_set(&data->fast_calib_rslt, 2);
		GSE_ERR("calibration LSM6DSE_ERR_SETUP_FAILURE!\n");
	} else {
		atomic_set(&data->fast_calib_rslt, 1);
		GSE_ERR("calibration FAIL!\n");
	}
	return count;
}

static ssize_t show_run_fast_calibration_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	struct lsm6dse_i2c_data *data = obj_i2c_data;

	if (data == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&data->fast_calib_rslt));
	return res;
}


/*----------------------------------------------------------------------------*/

static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensorrawdata, S_IRUGO, show_sensorrawdata_value, NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(trace, S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(chipinit, S_IRUGO, show_chipinit_value, store_chipinit_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(layout, S_IRUGO, show_layout_value, store_layout_value);
static DRIVER_ATTR(selftest, S_IRUGO | S_IWUSR, show_selftest_value, store_selftest_value);
static DRIVER_ATTR(run_fast_calibration, S_IRUGO | S_IWUSR, show_run_fast_calibration_value,
		   store_run_fast_calibration_value);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *LSM6DSE_attr_list[] = {
	&driver_attr_chipinfo,	/*chip information */
	&driver_attr_sensordata,	/*dump sensor data */
	&driver_attr_sensorrawdata,	/*dump sensor raw data */
	&driver_attr_trace,	/*trace log */
	&driver_attr_status,
	&driver_attr_chipinit,
	&driver_attr_layout,
	&driver_attr_selftest,
	&driver_attr_run_fast_calibration,

};

/*----------------------------------------------------------------------------*/
static int lsm6dse_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(LSM6DSE_attr_list) / sizeof(LSM6DSE_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, LSM6DSE_attr_list[idx]);
		if (0 != err) {
			GSE_ERR("driver_create_file (%s) = %d\n", LSM6DSE_attr_list[idx]->attr.name,
				err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int lsm6dse_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(LSM6DSE_attr_list) / sizeof(LSM6DSE_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, LSM6DSE_attr_list[idx]);

	return err;
}

static int LSM6DSE_Set_RegInc(struct i2c_client *client, bool inc)
{
	u8 databuf[2] = { 0 };
	int res = 0;
	/* GSE_FUN(); */

	if (hwmsen_read_byte(client, LSM6DSE_CTRL3_C, databuf)) {
		GSE_ERR("read LSM6DSE_CTRL3_XL err!\n");
		return LSM6DSE_ERR_I2C;
	}
	GSE_LOG("read  LSM6DSE_CTRL3_C register: 0x%x\n", databuf[0]);

	if (inc) {
		databuf[0] |= LSM6DSE_CTRL3_C_IFINC;

		databuf[1] = databuf[0];
		databuf[0] = LSM6DSE_CTRL3_C;

		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			GSE_ERR("write full scale register err!\n");
			return LSM6DSE_ERR_I2C;
		}
	}

	return LSM6DSE_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int LSM6DSE_init_client(struct i2c_client *client, bool enable)
{
	struct lsm6dse_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

	GSE_FUN();
	GSE_LOG(" lsm6dse addr %x!\n", client->addr);
	res = LSM6DSE_CheckDeviceID(client);
	if (res != LSM6DSE_SUCCESS)
		return res;

	res = LSM6DSE_Set_RegInc(client, true);
	if (res != LSM6DSE_SUCCESS)
		return res;

	res = LSM6DSE_acc_SetFullScale(client, LSM6DSE_ACC_RANGE_4g);	/* we have only this choice */
	if (res != LSM6DSE_SUCCESS)
		return res;

	/* res = LSM6DSE_acc_SetSampleRate(client, LSM6DSE_ACC_ODR_104HZ); */
	res = LSM6DSE_acc_SetSampleRate(client, obj->sample_rate);
	if (res != LSM6DSE_SUCCESS)
		return res;

	res = LSM6DSE_acc_SetPowerMode(client, enable);
	if (res != LSM6DSE_SUCCESS)
		return res;

	GSE_LOG("LSM6DSE_init_client OK!\n");
	/* acc setting */

#ifdef CONFIG_LSM6DSE_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	return LSM6DSE_SUCCESS;
}

/*----------------------------------------------------------------------------*/
#ifdef LSM6DSE_NEW_ARCH
static int lsm6dse_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */

	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int lsm6dse_enable_nodata(int en)
{
	int value = en;
	int err = 0;
	struct lsm6dse_i2c_data *priv = obj_i2c_data;

	if (priv == NULL) {
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}

	if (value == 1) {
		enable_status = true;
	} else {
		enable_status = false;
		priv->sample_rate = LSM6DSE_ACC_ODR_104HZ;	/* default rate */
	}
	GSE_LOG("enable value=%d, sensor_power =%d\n", value, sensor_power);

	if (((value == 0) && (sensor_power == false)) || ((value == 1) && (sensor_power == true)))
		GSE_LOG("Gsensor device have updated!\n");
	else if (!pedo_enable_status && !tilt_enable_status && !smd_enable_status)
		err = LSM6DSE_acc_SetPowerMode(priv->client, enable_status);

	GSE_LOG("%s OK!\n", __func__);
	return err;
}

static int lsm6dse_set_delay(u64 ns)
{
	int value = 0;
	int err = 0;
	int sample_delay;
	struct lsm6dse_i2c_data *priv = obj_i2c_data;

	value = (int)ns / 1000 / 1000;

	if (priv == NULL) {
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}

	if (value <= 5)
		sample_delay = LSM6DSE_ACC_ODR_208HZ;
	else if (value <= 10)
		sample_delay = LSM6DSE_ACC_ODR_104HZ;
	else
		sample_delay = LSM6DSE_ACC_ODR_52HZ;

	priv->sample_rate = sample_delay;
	err = LSM6DSE_acc_SetSampleRate(priv->client, sample_delay);
	if (err != LSM6DSE_SUCCESS)
		GSE_ERR("Set delay parameter error!\n");

	if (value >= 50) {
		atomic_set(&priv->filter, 0);
	} else {
		priv->fir.num = 0;
		priv->fir.idx = 0;
		priv->fir.sum[LSM6DSE_AXIS_X] = 0;
		priv->fir.sum[LSM6DSE_AXIS_Y] = 0;
		priv->fir.sum[LSM6DSE_AXIS_Z] = 0;
		atomic_set(&priv->filter, 1);
	}

	GSE_LOG("%s (%d), chip only use 1024HZ\n", __func__, value);
	return 0;
}

static int lsm6dse_get_data(int *x, int *y, int *z, int *status)
{
	char buff[LSM6DSE_BUFSIZE];
	struct lsm6dse_i2c_data *priv = obj_i2c_data;
	int ret;

	if (priv == NULL) {
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}

	if (atomic_read(&priv->trace) & ACCEL_TRC_DATA)
		GSE_LOG("%s (%d),\n", __func__, __LINE__);

	memset(buff, 0, sizeof(buff));
	LSM6DSE_ReadAccData(priv->client, buff, LSM6DSE_BUFSIZE);

	ret = sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_HIGH;

	return 0;
}

#ifdef LSM6DSE_TILT_FUNC
static int lsm6dse_tilt_open_report_data(int open)
{
	int res = 0;
	struct lsm6dse_i2c_data *priv = obj_i2c_data;

	if (1 == open) {
		tilt_enable_status = true;
		res = LSM6DSE_enable_tilt(priv->client, true);
		if (LSM6DSE_SUCCESS != res)
			GSE_ERR("%s run LSM6DSE_enable_tilt to true failed!\n", __func__);
	} else if (0 == open) {
		tilt_enable_status = false;
		res = LSM6DSE_enable_tilt(priv->client, false);
		if (LSM6DSE_SUCCESS != res)
			GSE_ERR("%s run LSM6DSE_enable_tilt to false failed!\n", __func__);
	}

	return res;
}
#endif

#ifdef LSM6DSE_SIGNIFICANT_MOTION
static int lsm6dse_step_c_enable_significant(int en)
{
	int res = 0;
	struct lsm6dse_i2c_data *priv = obj_i2c_data;

	if (1 == en) {
		/* pedo_enable_status = true; */
		res = LSM6DSE_Set_SigMotion_Threshold(priv->client, 0x08);
		if (LSM6DSE_SUCCESS != res)
			GSE_ERR("%s run LSM6DSE_Set_SigMotion_Threshold to fail!\n", __func__);

		/* res = LSM6DSE_acc_SetSampleRate(priv->client, LSM6DSE_ACC_ODR_26HZ); */
		res = LSM6DSE_acc_SetSampleRate(priv->client, priv->sample_rate);
		if (LSM6DSE_SUCCESS != res)
			GSE_ERR("%s run LSM6DSE_Set_SigMotion_Threshold to fail!\n", __func__);

		res = LSM6DSE_Enable_SigMotion_Func_On_Int(priv->client, true);	/* default route to INT2 */
		if (LSM6DSE_SUCCESS != res)
			GSE_ERR("%s run LSM6DSE_Enable_SigMotion_Func_On_Int to fail!\n", __func__);

		res = LSM6DSE_acc_SetFullScale(priv->client, LSM6DSE_ACC_RANGE_4g);
		if (LSM6DSE_SUCCESS != res)
			GSE_ERR("%s run LSM6DSE_Enable_SigMotion_Func_On_Int to fail!\n", __func__);

		/* enable pedometer feature */
		LSM6DSE_enable_pedo(priv->client, true);
		smd_enable_status = true;

		enable_irq(obj_i2c_data->irq);
	} else if (0 == en) {
		/* pedo_enable_status = false; */
		res = LSM6DSE_Enable_SigMotion_Func_On_Int(priv->client, false);
		if (LSM6DSE_SUCCESS != res)
			GSE_ERR("%s run LSM6DSE_Enable_SigMotion_Func_On_Int to fail!\n", __func__);

		/* disable pedometer if step detector and pedo are disabled */
		if (!pedo_enable_status && !step_d_enable_status)
			LSM6DSE_enable_pedo(priv->client, false);

		smd_enable_status = false;

		if (!enable_status && !tilt_enable_status) {
			res = LSM6DSE_acc_SetPowerMode(priv->client, false);
			if (LSM6DSE_SUCCESS != res)
				GSE_ERR("%s run LSM6DSE_acc_SetPowerMode to fail!\n", __func__);
		}

		disable_irq(obj_i2c_data->irq);
	}

	return res;
}
#endif

#ifdef LSM6DSE_STEP_COUNTER	/* step counter */
static int lsm6dse_step_c_open_report_data(int open)
{

	return LSM6DSE_SUCCESS;
}

static int lsm6dse_step_c_enable_nodata(int en)
{
	int res = 0;
	int value = en;
	int err = 0;
	struct lsm6dse_i2c_data *priv = obj_i2c_data;

	if (priv == NULL) {
		GSE_ERR("%s obj_i2c_data is NULL!\n", __func__);
		return -1;
	}

	if (value == 1) {
		pedo_enable_status = true;
		res = LSM6DSE_enable_pedo(priv->client, true);
		if (LSM6DSE_SUCCESS != res) {
			GSE_LOG("LSM6DSE_enable_pedo failed at open action!\n");
			return res;
		}
	} else {
		pedo_enable_status = false;
		res = LSM6DSE_enable_pedo(priv->client, false);
		if (LSM6DSE_SUCCESS != res) {
			GSE_LOG("LSM6DSE_enable_pedo failed at close action!\n");
			return res;
		}

	}

	GSE_LOG("lsm6dse_step_c_enable_nodata OK!\n");
	return err;
}

static int lsm6dse_step_c_enable_step_detect(int en)
{
	int res = 0;
	struct lsm6dse_i2c_data *priv = obj_i2c_data;

	if (en)
		step_d_enable_status = true;
	else
		step_d_enable_status = false;

	res = LSM6DSE_Enable_Step_Detector_On_Int(priv->client, en);	/* enable step detector interrupt */
	if (res != LSM6DSE_SUCCESS) {
		GSE_LOG(" LSM6DSE_Enable_Step_Detector_On_Int failed!\n");
		return res;
	}
	return lsm6dse_step_c_enable_nodata(en);
}

static int lsm6dse_step_c_set_delay(u64 delay)
{
	return 0;
}

static int lsm6dse_step_d_set_delay(u64 delay)
{

	return 0;
}

static int lsm6dse_step_c_get_data(u64 *value, int *status)
{
	int err = 0;
	u16 pedo_data = 0;

	struct lsm6dse_i2c_data *priv = obj_i2c_data;

	err = LSM6DSE_Get_Pedo_DataReg(priv->client, &pedo_data);
	*value = (u64) pedo_data;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return err;
}

static int lsm6dse_step_c_get_data_step_d(u64 *value, int *status)
{
	return 0;
}

static int lsm6dse_step_c_get_data_significant(u64 *value, int *status)
{
	return 0;
}
#endif
#else
static int LSM6DSE_acc_operate(void *self, uint32_t command, void *buff_in, int size_in,
			       void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value, sample_delay;
	struct lsm6dse_i2c_data *priv = (struct lsm6dse_i2c_data *)self;
	hwm_sensor_data *gsensor_data;
	char buff[LSM6DSE_BUFSIZE];

	/* GSE_FUN(f); */
	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			GSE_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 5)
				sample_delay = LSM6DSE_ACC_ODR_208HZ;
			else if (value <= 10)
				sample_delay = LSM6DSE_ACC_ODR_104HZ;
			else
				sample_delay = LSM6DSE_ACC_ODR_52HZ;

			priv->sample_rate = sample_delay;
			LSM6DSE_acc_SetSampleRate(priv->client, sample_delay);
			if (err != LSM6DSE_SUCCESS)
				GSE_ERR("Set delay parameter error!\n");

			if (value >= 50) {
				atomic_set(&priv->filter, 0);
			} else {
				priv->fir.num = 0;
				priv->fir.idx = 0;
				priv->fir.sum[LSM6DSE_AXIS_X] = 0;
				priv->fir.sum[LSM6DSE_AXIS_Y] = 0;
				priv->fir.sum[LSM6DSE_AXIS_Z] = 0;
				atomic_set(&priv->filter, 1);
			}
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			GSE_ERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {

			value = *(int *)buff_in;
			if (value == 1) {
				enable_status = true;
			} else {
				enable_status = false;
				priv->sample_rate = LSM6DSE_ACC_ODR_104HZ;	/* default rate */
			}
			GSE_LOG("enable value=%d, sensor_power =%d\n", value, sensor_power);

			if (((value == 0) && (sensor_power == false))
			    || ((value == 1) && (sensor_power == true))) {
				GSE_LOG("Gsensor device have updated!\n");
			} else if (!pedo_enable_status && !tilt_enable_status && !smd_enable_status) {
				err = LSM6DSE_acc_SetPowerMode(priv->client, enable_status);
			}

		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(hwm_sensor_data))) {
			GSE_ERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			int ret;

			gsensor_data = (hwm_sensor_data *) buff_out;
			LSM6DSE_ReadAccData(priv->client, buff, LSM6DSE_BUFSIZE);

			ret = sscanf(buff, "%x %x %x", &gsensor_data->values[0],
			       &gsensor_data->values[1], &gsensor_data->values[2]);
			gsensor_data->status = SENSOR_STATUS_ACCURACY_HIGH;
			gsensor_data->value_divide = 1000;
		}
		break;
	default:
		GSE_ERR("gsensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}
#endif

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int lsm6dse_open(struct inode *inode, struct file *file)
{
	file->private_data = lsm6dse_i2c_client;

	if (file->private_data == NULL) {
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int lsm6dse_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static long lsm6dse_acc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct lsm6dse_i2c_data *obj = (struct lsm6dse_i2c_data *)i2c_get_clientdata(client);
	char strbuf[LSM6DSE_BUFSIZE] = { 0, };
	void __user *data;
	struct SENSOR_DATA sensor_data;
	int err = 0;
	int cali[6];

	/* GSE_FUN(f); */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err) {
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
#if 0
	case GSENSOR_IOCTL_SET_ENABLE:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_from_user(&enable, data, sizeof(enable)))
			return -EFAULT;

		GSE_LOG("GSENSOR_IOCTL_SET_ENABLE %d\n", (int)enable);
		if (enable == 1)
			err = LSM6DSE_acc_SetPowerMode(obj->client, 1);
		else if (enable == 0)
			err = LSM6DSE_acc_SetPowerMode(obj->client, 0);
		break;
#endif
	case GSENSOR_IOCTL_INIT:
		LSM6DSE_init_client(client, 0);
		break;

	case GSENSOR_IOCTL_READ_CHIPINFO:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		LSM6DSE_ReadChipInfo(client, strbuf, LSM6DSE_BUFSIZE);

		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_SENSORDATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		LSM6DSE_ReadAccData(client, strbuf, LSM6DSE_BUFSIZE);

		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_GAIN:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		break;

	case GSENSOR_IOCTL_READ_OFFSET:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		break;

	case GSENSOR_IOCTL_READ_RAW_DATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		LSM6DSE_ReadAccRawData(client, (s16 *) strbuf);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_SET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
#ifdef CALIBRATION_TO_FILE
		if (make_cal_data_file() == LSM6DSE_SUCCESS) {
			err = LSM6DSE_acc_ReadCalibration(cali);
			if (err != 0) {
				GSE_ERR("Read Cal Fail from file!!\n");
				break;
			}
			sensor_data.x = cali[0];
			sensor_data.y = cali[1];
			sensor_data.z = cali[2];
		}
#endif
		if (atomic_read(&obj->suspend)) {
			GSE_ERR("Perform calibration in suspend state!!\n");
			err = -EINVAL;
		} else {
			cali[LSM6DSE_AXIS_X] = (s64) (sensor_data.x);
			cali[LSM6DSE_AXIS_Y] = (s64) (sensor_data.y);
			cali[LSM6DSE_AXIS_Z] = (s64) (sensor_data.z);

			obj->cali_sw[LSM6DSE_AXIS_X] = cali[LSM6DSE_AXIS_X];
			obj->cali_sw[LSM6DSE_AXIS_Y] = cali[LSM6DSE_AXIS_Y];
			obj->cali_sw[LSM6DSE_AXIS_Z] = cali[LSM6DSE_AXIS_Z];
		}
		break;

	case GSENSOR_IOCTL_CLR_CALI:
		err = LSM6DSE_acc_ResetCalibration(client);
		break;

	case GSENSOR_IOCTL_GET_CALI:
		break;

	default:
		GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;

	}

	return err;
}

#ifdef CONFIG_COMPAT
static long lsm6dse_acc_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long err = 0;

	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA,
					       (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_SET_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_GET_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_CLR_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	default:
		GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;

	}

	return err;
}
#endif

/*----------------------------------------------------------------------------*/
static const struct file_operations lsm6dse_acc_fops = {
	.owner = THIS_MODULE,
	.open = lsm6dse_open,
	.release = lsm6dse_release,
	.unlocked_ioctl = lsm6dse_acc_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = lsm6dse_acc_compat_ioctl,
#endif
};

/*----------------------------------------------------------------------------*/
static struct miscdevice lsm6dse_acc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &lsm6dse_acc_fops,
};

/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int lsm6dse_acc_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct lsm6dse_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	GSE_FUN();

	if (msg.event == PM_EVENT_SUSPEND) {
		if (obj == NULL) {
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);

		if (pedo_enable_status || smd_enable_status || tilt_enable_status)
			return 0;

		err = LSM6DSE_acc_SetPowerMode(obj->client, false);
		if (err) {
			GSE_ERR("write power control fail!!\n");
			return err;
		}

		sensor_power = false;

		LSM6DSE_power(obj->hw, 0);

	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int lsm6dse_acc_resume(struct i2c_client *client)
{
	struct lsm6dse_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	GSE_FUN();

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return -1;
	}

	if (pedo_enable_status || smd_enable_status || tilt_enable_status) {
		atomic_set(&obj->suspend, 0);
		return 0;
	}
	LSM6DSE_power(obj->hw, 1);
	err = LSM6DSE_acc_SetPowerMode(obj->client, enable_status);
	if (err) {
		GSE_ERR("initialize client fail! err code %d!\n", err);
		return err;
	}
	atomic_set(&obj->suspend, 0);

	return 0;
}

/*----------------------------------------------------------------------------*/
#else				/*CONFIG_HAS_EARLY_SUSPEND is defined */
/*----------------------------------------------------------------------------*/
static void lsm6dse_early_suspend(struct early_suspend *h)
{
	struct lsm6dse_i2c_data *obj = container_of(h, struct lsm6dse_i2c_data, early_drv);
	int err;

	GSE_FUN();

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1);

	if (pedo_enable_status || smd_enable_status || tilt_enable_status)
		return;

	err = LSM6DSE_acc_SetPowerMode(obj->client, false);
	if (err) {
		GSE_ERR("write power control fail!!\n");
		return;
	}

	sensor_power = false;

	LSM6DSE_power(obj->hw, 0);
}

/*----------------------------------------------------------------------------*/
static void lsm6dse_late_resume(struct early_suspend *h)
{
	struct lsm6dse_i2c_data *obj = container_of(h, struct lsm6dse_i2c_data, early_drv);
	int err;

	GSE_FUN();

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return;
	}

	if (pedo_enable_status || smd_enable_status || tilt_enable_status) {
		atomic_set(&obj->suspend, 0);
		return;
	}

	LSM6DSE_power(obj->hw, 1);

	err = LSM6DSE_acc_SetPowerMode(obj->client, enable_status);

	if (err) {
		GSE_ERR("initialize client fail! err code %d!\n", err);
		return;
	}
	atomic_set(&obj->suspend, 0);
}
#endif				/*CONFIG_HAS_EARLYSUSPEND */

#ifdef LSM6DSE_TILT_FUNC
static void lsm6dse_eint_work(struct work_struct *work)
{
	u8 databuf[2] = { 0 };
	struct lsm6dse_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("obj_i2c_data is null pointer!!\n");
		return;
	}

	if (hwmsen_read_byte(obj->client, LSM6DSE_FUNC_SRC, databuf)) {
		GSE_ERR("%s read LSM6DSE_CTRL10_C register err!\n", __func__);
		goto lsm6dse_eint_work_exit;
	}

	if (atomic_read(&obj->trace) & ACCEL_TRC_DATA) {
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}

	else if (LSM6DSE_STEP_DETECT_INT_STATUS & databuf[0]) {
#ifdef LSM6DSE_STEP_COUNTER
		/* add the action when receive step detection interrupt */
		if (step_d_enable_status)
			step_notify(TYPE_STEP_DETECTOR);
#endif

#ifdef LSM6DSE_SIGNIFICANT_MOTION
/* the smd is one-shot sensor, so user need to re-enable it to get this event again while this event happened */
		if (smd_enable_status) {
			smd_enable_status = false;
			step_notify(TYPE_SIGNIFICANT);
		}
#endif
	}

	else if (LSM6DSE_TILT_INT_STATUS & databuf[0]) {
#ifdef LSM6DSE_TILT_FUNC
		/* add the action when receive the tilt interrupt */
		if (tilt_enable_status)
			tilt_notify();
#endif
	}

lsm6dse_eint_work_exit:
	enable_irq(obj_i2c_data->irq);
}
#endif
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int lsm6dse_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct lsm6dse_i2c_data *obj;
	int err = 0;
#ifndef LSM6DSE_NEW_ARCH
	struct hwmsen_object gyro_sobj;
	struct hwmsen_object acc_sobj;
#endif

	GSE_FUN();

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct lsm6dse_i2c_data));

#ifdef LSM6DSE_TILT_FUNC
	INIT_WORK(&obj->eint_work, lsm6dse_eint_work);
#endif

	obj->hw = get_cust_accel();
	obj->sample_rate = LSM6DSE_ACC_ODR_104HZ;

	atomic_set(&obj->layout, obj->hw->direction);
	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if (err) {
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit_kfree;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

	lsm6dse_i2c_client = new_client;
	err = LSM6DSE_init_client(new_client, false);
	if (err) {
		GSE_ERR("lsm6dse_init_client failed!\n");
		goto exit_init_failed;
	}

	err = misc_register(&lsm6dse_acc_device);
	if (err) {
		GSE_ERR("lsm6dse_gyro_device misc register failed!\n");
		goto exit_misc_device_register_failed;
	}
#ifdef LSM6DSE_NEW_ARCH

#else
	err = lsm6dse_create_attr(&lsm6dse_driver.driver);
#endif
	if (err) {
		GSE_ERR("lsm6dse create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
#ifdef LSM6DSE_NEW_ARCH

#else
	acc_sobj.self = obj;
	acc_sobj.polling = 1;
	acc_sobj.sensor_operate = LSM6DSE_acc_operate;
	err = hwmsen_attach(ID_ACCELEROMETER, &acc_sobj);
	if (err) {
		GSE_ERR("hwmsen_attach Accelerometer fail = %d\n", err);
		goto exit_kfree;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	    obj->early_drv.suspend = lsm6dse_early_suspend,
	    obj->early_drv.resume = lsm6dse_late_resume, register_early_suspend(&obj->early_drv);
#endif
#ifdef LSM6DSE_NEW_ARCH
	lsm6dse_acc_init_flag = 0;
#endif
	GSE_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
	misc_deregister(&lsm6dse_acc_device);
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
	kfree(obj);
exit:
#ifdef LSM6DSE_NEW_ARCH
	lsm6dse_acc_init_flag = -1;
#endif
	GSE_ERR("%s: err = %d\n", __func__, err);
	return err;
}

/*----------------------------------------------------------------------------*/
static int lsm6dse_i2c_remove(struct i2c_client *client)
{
	int err = 0;

#ifdef LSM6DSE_NEW_ARCH
	if (test_bit(LSM6DSE_ACC, &lsm6dse_init_flag_test)) {
		GSE_ERR("lsm6dse_i2c init flag -1\n");
		err = lsm6dse_delete_attr(&(lsm6dse_init_info.platform_diver_addr->driver));
	}
	lsm6dse_acc_init_flag = -1;
#else
	err = lsm6dse_delete_attr(&lsm6dse_driver.driver);
#endif
	if (err)
		GSE_ERR("lsm6dse_i2c_remove fail: %d\n", err);

	err = misc_deregister(&lsm6dse_acc_device);
	if (err)
		GSE_ERR("misc_deregister lsm6dse_gyro_device fail: %d\n", err);

	lsm6dse_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}

/*----------------------------------------------------------------------------*/
#ifdef LSM6DSE_NEW_ARCH
static int lsm6dse_local_init_common(void)
{
	GSE_FUN();

	LSM6DSE_power(hw, 1);

	i2c_register_board_info(1, &i2c_lsm6dse, 1);

	if (i2c_add_driver(&lsm6dse_i2c_driver)) {
		GSE_ERR("add driver error\n");
		return -1;
	}

	return 0;
}

static int lsm6dse_local_init(void)
{
	int res = 0;
	struct acc_control_path ctl = { 0 };
	struct acc_data_path data = { 0 };
	struct lsm6dse_i2c_data *obj = NULL;

	mutex_lock(&lsm6dse_init_mutex);

	set_bit(LSM6DSE_ACC, &lsm6dse_init_flag_test);

	if ((0 == test_bit(LSM6DSE_STEP_C, &lsm6dse_init_flag_test))
	    && (0 == test_bit(LSM6DSE_TILT, &lsm6dse_init_flag_test))) {
		res = lsm6dse_local_init_common();
		if (res < 0) {
			GSE_ERR("%s local init common failed!\n", __func__);
			goto lsm6dse_local_init_failed;
		}

	}


	if (lsm6dse_acc_init_flag == -1) {
		mutex_unlock(&lsm6dse_init_mutex);
		GSE_ERR("%s init failed!\n", __func__);
		return -1;
	}

	obj = obj_i2c_data;
	if (NULL == obj) {
		GSE_ERR("i2c_data obj is null!!\n");
		goto lsm6dse_local_init_failed;
	}

	res = lsm6dse_create_attr(&(lsm6dse_init_info.platform_diver_addr->driver));
	if (res < 0)
		goto lsm6dse_local_init_failed;

	ctl.open_report_data = lsm6dse_open_report_data;
	ctl.enable_nodata = lsm6dse_enable_nodata;
	ctl.set_delay = lsm6dse_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = obj->hw->is_batch_supported;

	res = acc_register_control_path(&ctl);
	if (res) {
		GSE_ERR("register acc control path err\n");
		goto lsm6dse_local_init_failed;
	}

	data.get_data = lsm6dse_get_data;
	data.vender_div = 1000;
	res = acc_register_data_path(&data);
	if (res) {
		GSE_ERR("register acc data path err= %d\n", res);
		goto lsm6dse_local_init_failed;
	}

	mutex_unlock(&lsm6dse_init_mutex);
	return 0;
lsm6dse_local_init_failed:
	GSE_ERR("%s init failed\n", __func__);
	mutex_unlock(&lsm6dse_init_mutex);
	return res;

}

static int lsm6dse_local_uninit(void)
{
	clear_bit(LSM6DSE_ACC, &lsm6dse_init_flag_test);

	/* GSE_FUN(); */
	LSM6DSE_power(hw, 0);
	i2c_del_driver(&lsm6dse_i2c_driver);
	return 0;
}

#ifdef LSM6DSE_TILT_FUNC
static int lsm6dse_tilt_get_data(u16 *value, int *status)
{
	return 0;
}

static void lsm6dse_eint_func(void)
{
	struct lsm6dse_i2c_data *priv = obj_i2c_data;
	/* GSE_FUN(); */
	if (!priv)
		return;

	schedule_work(&priv->eint_work);
}

static irqreturn_t lsm6dse_eint_handler(int irq, void *desc)
{
	lsm6dse_eint_func();
	disable_irq_nosync(obj_i2c_data->irq_node);

	return IRQ_HANDLED;
}

static int lsm6dse_setup_eint(void)
{
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	obj_i2c_data->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, gse_1-eint");

	if (obj_i2c_data->irq_node) {
		of_property_read_u32_array(obj_i2c_data->irq_node, "debounce", ints,
					   ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		GSE_ERR("accel request_irq IRQ %d!.", ints[0]);
		obj_i2c_data->irq = irq_of_parse_and_map(obj_i2c_data->irq_node, 0);
		if (obj_i2c_data->irq < 0) {
			GSE_ERR("accel request_irq IRQ LINE NOT AVAILABLE!.");
			return -1;
		}
		GSE_ERR("gyro int_gpio_number : %d irq:%d!!\n", ints[0], obj_i2c_data->irq);
		ret =
		    request_irq(obj_i2c_data->irq, lsm6dse_eint_handler, IRQF_TRIGGER_FALLING,
				"gse_1-eint", NULL);
		if (ret > 0) {
			GSE_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
	} else {
		GSE_ERR("null irq node!!\n");
		return -EINVAL;
	}
	return ret;
}

static int lsm6dse_tilt_local_init(void)
{
	int res = 0;

	struct tilt_control_path tilt_ctl = { 0 };
	struct tilt_data_path tilt_data = { 0 };

	mutex_lock(&lsm6dse_init_mutex);
	set_bit(LSM6DSE_TILT, &lsm6dse_init_flag_test);

	if ((0 == test_bit(LSM6DSE_ACC, &lsm6dse_init_flag_test))
	    && (0 == test_bit(LSM6DSE_STEP_C, &lsm6dse_init_flag_test))) {
		res = lsm6dse_local_init_common();
		if (res < 0)
			goto lsm6dse_tilt_local_init_failed;
	}

	if (lsm6dse_acc_init_flag == -1) {
		mutex_unlock(&lsm6dse_init_mutex);
		GSE_ERR("%s init failed!\n", __func__);
		return -1;

	res = lsm6dse_setup_eint();
	tilt_ctl.open_report_data = lsm6dse_tilt_open_report_data;
	res = tilt_register_control_path(&tilt_ctl);

	tilt_data.get_data = lsm6dse_tilt_get_data;
	res = tilt_register_data_path(&tilt_data);

	mutex_unlock(&lsm6dse_init_mutex);
	return 0;

lsm6dse_tilt_local_init_failed:
	mutex_unlock(&lsm6dse_init_mutex);
	GSE_ERR("%s init failed!\n", __func__);
	return -1;
}

static int lsm6dse_tilt_local_uninit(void)
{
	clear_bit(LSM6DSE_TILT, &lsm6dse_init_flag_test);
	return 0;
}
#endif

#ifdef LSM6DSE_STEP_COUNTER
static int lsm6dse_step_c_local_init(void)
{
	int res = 0;

	struct step_c_control_path step_ctl = { 0 };
	struct step_c_data_path step_data = { 0 };

	mutex_lock(&lsm6dse_init_mutex);

	set_bit(LSM6DSE_STEP_C, &lsm6dse_init_flag_test);

	if ((0 == test_bit(LSM6DSE_ACC, &lsm6dse_init_flag_test))
	    && (0 == test_bit(LSM6DSE_TILT, &lsm6dse_init_flag_test))) {
		res = lsm6dse_local_init_common();
		if (res < 0)
			goto lsm6dse_step_c_local_init_failed;
	}

	if (lsm6dse_acc_init_flag == -1) {
		mutex_unlock(&lsm6dse_init_mutex);
		GSE_ERR("%s init failed!\n", __func__);
		return -1;

	step_ctl.open_report_data = lsm6dse_step_c_open_report_data;
	step_ctl.enable_nodata = lsm6dse_step_c_enable_nodata;
	step_ctl.enable_step_detect = lsm6dse_step_c_enable_step_detect;
#ifndef LSM6DSE_STEP_DETECT
	step_ctl.set_delay = lsm6dse_step_c_set_delay;
#else
	step_ctl.step_c_set_delay = lsm6dse_step_c_set_delay;
	step_ctl.step_d_set_delay = lsm6dse_step_d_set_delay;
#endif
	step_ctl.is_report_input_direct = false;
	step_ctl.is_support_batch = false;
#ifdef LSM6DSE_SIGNIFICANT_MOTION
	step_ctl.enable_significant = lsm6dse_step_c_enable_significant;
#endif

	res = step_c_register_control_path(&step_ctl);
	if (res) {
		GSE_ERR("register step counter control path err\n");
		goto lsm6dse_step_c_local_init_failed;
	}

	step_data.get_data = lsm6dse_step_c_get_data;
	step_data.get_data_step_d = lsm6dse_step_c_get_data_step_d;
	step_data.get_data_significant = lsm6dse_step_c_get_data_significant;

	step_data.vender_div = 1;
	res = step_c_register_data_path(&step_data);
	if (res) {
		GSE_ERR("register step counter data path err= %d\n", res);
		goto lsm6dse_step_c_local_init_failed;
	}

	mutex_unlock(&lsm6dse_init_mutex);
	return 0;

lsm6dse_step_c_local_init_failed:
	mutex_unlock(&lsm6dse_init_mutex);
	GSE_ERR("%s init failed!\n", __func__);
	return res;

}

static int lsm6dse_step_c_local_uninit(void)
{
	clear_bit(LSM6DSE_STEP_C, &lsm6dse_init_flag_test);
	return 0;
}
#endif
#else
static int lsm6dse_probe(struct platform_device *pdev)
{
	GSE_FUN();

	LSM6DSE_power(hw, 1);

	if (i2c_add_driver(&lsm6dse_i2c_driver)) {
		GSE_ERR("add driver error\n");
		return -1;
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static int lsm6dse_remove(struct platform_device *pdev)
{

	/* GSE_FUN(); */
	LSM6DSE_power(hw, 0);
	i2c_del_driver(&lsm6dse_i2c_driver);
	return 0;
}

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
	{.compatible = "mediatek,lsm6dse-acc",},
	{},
};
#endif

static struct platform_driver lsm6dse_driver = {
	.probe = lsm6dse_probe,
	.remove = lsm6dse_remove,
	.driver = {
		   .name = "gsensor",
		   /* .owner  = THIS_MODULE, */
#ifdef CONFIG_OF
		   .of_match_table = gsensor_of_match,
#endif
		   }
};

#endif
/*----------------------------------------------------------------------------*/
static int __init lsm6dse_init(void)
{
	/* GSE_FUN(); */
	const char *name = "mediatek,lsm6dse-acc";

	hw = get_accel_dts_func(name, hw);
	if (!hw)
		GSE_LOG("get dts info fail\n");

#ifdef LSM6DSE_NEW_ARCH
	acc_driver_add(&lsm6dse_init_info);
#ifdef LSM6DSE_STEP_COUNTER	/* step counter */
	step_c_driver_add(&lsm6dse_step_c_init_info);	/* step counter */
#endif
#ifdef LSM6DSE_TILT_FUNC
	tilt_driver_add(&lsm6dse_tilt_init_info);
#endif

#else
	if (platform_driver_register(&lsm6dse_driver)) {
		GSE_ERR("failed to register driver");
		return -ENODEV;
	}
#endif

	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit lsm6dse_exit(void)
{
	GSE_FUN();
#ifndef LSM6DSE_NEW_ARCH
	platform_driver_unregister(&lsm6dse_driver);
#endif

}

/*----------------------------------------------------------------------------*/
module_init(lsm6dse_init);
module_exit(lsm6dse_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LSM6DSE Accelerometer");
MODULE_AUTHOR("tim.kim@st.com");






/*------------------------------------------- LSM6DSE ------------------------*/
