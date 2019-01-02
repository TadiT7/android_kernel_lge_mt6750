/*****************************************************************************
 *
 * Filename:
 * ---------
 *   S5K4H9mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>

/* #include <asm/system.h> */
/* #include <linux/xlog.h> */

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k4h9mipi_Sensor.h"

#ifdef S5K4H9_DEBUG
#define PFX "S5K4H9_camera_sensor"
#define LOG_INF(fmt, args...)    pr_debug(PFX "[%s] " fmt, __func__, ##args)
#else
#define LOG_INF(fmt, args...)
#endif

#define LOG_1 LOG_INF("S5K4H9,MIPI 2LANE\n")
#define LOG_2 \
LOG_INF("preview 1640*1232@30fps,1488Mbps/lane\n")\
LOG_INF("video 4192*3104@30fps,1260Mbps/lane;capture 3280*2464@30fps,1488Mbps/lane\n")
/* #define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args) */
/* #define LOG_INF(format, args...)      xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args) */
/* #define LOGE(format, args...)   xlog_printk(ANDROID_LOG_ERROR, PFX, "[%s] " format, __FUNCTION__, ##args) */

static DEFINE_SPINLOCK(imgsensor_drv_lock);
static imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5K4H9_SENSOR_ID,
	.checksum_value = 0xe6328192,
	.pre = {
		.pclk = 360000000,	/* record different mode's pclk */
		.linelength = 4656,	/* record different mode's linelength */
		.framelength = 2564,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 1640,	/* record different mode's width of grabwindow */
		.grabwindow_height = 1232,	/* record different mode's height of grabwindow */
		/*       following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		.mipi_data_lp2hs_settle_dc = 85,
		/*       following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
		},
	.cap = {
		.pclk = 360000000,
		.linelength = 4656,
		.framelength = 2564,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3280,
		.grabwindow_height = 2464,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		},
#if 1
	.cap1 = {
		 .pclk = 360000000,
		 .linelength = 4656,
		 .framelength = 3220,
		 .startx = 0,
		 .starty = 0,
		 .grabwindow_width = 3280,
		 .grabwindow_height = 2464,
		 .mipi_data_lp2hs_settle_dc = 85,
		 .max_framerate = 240,
		 },
#else
	.cap1 = {
		 .pclk = 360000000,
		 .linelength = 4656,
		 .framelength = 5128,
		 .startx = 0,
		 .starty = 0,
		 .grabwindow_width = 3280,
		 .grabwindow_height = 2464,
		 .mipi_data_lp2hs_settle_dc = 85,
		 .max_framerate = 150,
		 },
#endif
	.normal_video = {
			 .pclk = 360000000,
			 .linelength = 4656,
			 .framelength = 2564,
			 .startx = 0,
			 .starty = 0,
			 .grabwindow_width = 3280,
			 .grabwindow_height = 2464,
			 .mipi_data_lp2hs_settle_dc = 85,
			 .max_framerate = 300,
			 },
	.hs_video = {
		     .pclk = 360000000,
		     .linelength = 4432,
		     .framelength = 675,
		     .startx = 0,
		     .starty = 0,
		     .grabwindow_width = 816,
		     .grabwindow_height = 608,
		     .mipi_data_lp2hs_settle_dc = 85,
		     .max_framerate = 1200,
		     },
	.slim_video = {
		       .pclk = 360000000,	/* record different mode's pclk */
		       .linelength = 4656,	/* record different mode's linelength */
		       .framelength = 2564,	/* record different mode's framelength */
		       .startx = 0,	/* record different mode's startx of grabwindow */
		       .starty = 0,	/* record different mode's starty of grabwindow */
		       .grabwindow_width = 1280,	/* record different mode's width of grabwindow */
		       .grabwindow_height = 720,	/* record different mode's height of grabwindow */
		       /*       following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		       .mipi_data_lp2hs_settle_dc = 85,
		       /*       following for GetDefaultFramerateByScenario()  */
		       .max_framerate = 300,

		       },
	.custom1 = {
		    .pclk = 440000000,	/* record different mode's pclk */
		    .linelength = 4592,	/* record different mode's linelength */
		    .framelength = 3188,	/* 3168,                     //record different mode's framelength */
		    .startx = 0,	/* record different mode's startx of grabwindow */
		    .starty = 0,	/* record different mode's starty of grabwindow */
		    .grabwindow_width = 2096,	/* record different mode's width of grabwindow */
		    .grabwindow_height = 1552,	/* record different mode's height of grabwindow */

		    /*       following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		    .mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		    /*       following for GetDefaultFramerateByScenario()  */
		    .max_framerate = 300,
		    },
	.custom2 = {
		    .pclk = 440000000,	/* record different mode's pclk */
		    .linelength = 4592,	/* record different mode's linelength */
		    .framelength = 3188,	/* 3168,                     //record different mode's framelength */
		    .startx = 0,	/* record different mode's startx of grabwindow */
		    .starty = 0,	/* record different mode's starty of grabwindow */
		    .grabwindow_width = 2096,	/* record different mode's width of grabwindow */
		    .grabwindow_height = 1552,	/* record different mode's height of grabwindow */

		    /*       following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		    .mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		    /*       following for GetDefaultFramerateByScenario()  */
		    .max_framerate = 300,
		    },
	.custom3 = {
		    .pclk = 440000000,	/* record different mode's pclk */
		    .linelength = 4592,	/* record different mode's linelength */
		    .framelength = 3188,	/* 3168,                     //record different mode's framelength */
		    .startx = 0,	/* record different mode's startx of grabwindow */
		    .starty = 0,	/* record different mode's starty of grabwindow */
		    .grabwindow_width = 2096,	/* record different mode's width of grabwindow */
		    .grabwindow_height = 1552,	/* record different mode's height of grabwindow */

		    /*       following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		    .mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		    /*       following for GetDefaultFramerateByScenario()  */
		    .max_framerate = 300,
		    },
	.custom4 = {
		    .pclk = 440000000,	/* record different mode's pclk */
		    .linelength = 4592,	/* record different mode's linelength */
		    .framelength = 3188,	/* 3168,                     //record different mode's framelength */
		    .startx = 0,	/* record different mode's startx of grabwindow */
		    .starty = 0,	/* record different mode's starty of grabwindow */
		    .grabwindow_width = 2096,	/* record different mode's width of grabwindow */
		    .grabwindow_height = 1552,	/* record different mode's height of grabwindow */

		    /*       following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		    .mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		    /*       following for GetDefaultFramerateByScenario()  */
		    .max_framerate = 300,
		    },
	.custom5 = {
		    .pclk = 440000000,	/* record different mode's pclk */
		    .linelength = 4592,	/* record different mode's linelength */
		    .framelength = 3188,	/* 3168,                     //record different mode's framelength */
		    .startx = 0,	/* record different mode's startx of grabwindow */
		    .starty = 0,	/* record different mode's starty of grabwindow */
		    .grabwindow_width = 2096,	/* record different mode's width of grabwindow */
		    .grabwindow_height = 1552,	/* record different mode's height of grabwindow */

		    /*       following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		    .mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		    /*       following for GetDefaultFramerateByScenario()  */
		    .max_framerate = 300,
		    },
	.margin = 5,
	.min_shutter = 4,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 5,	/* support sensor mode num */

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,
	.custom3_delay_frame = 2,
	.custom4_delay_frame = 2,
	.custom5_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */

	/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x20, 0x5a, 0xff},
	.i2c_speed = 300,	/* i2c read/write speed */
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* mirrorflip information */

	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
	INIT, Preview, Capture, Video,High Speed Video, Slim Video */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x200,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 0,	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */

	/*auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker*/
	.autoflicker_en = KAL_FALSE,

	/*test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output*/
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,	/* current scenario id */
	.ihdr_en = KAL_FALSE,	/* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x5a,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
{3280, 2464, 0, 0, 3280, 2464, 1640, 1232, 0000, 0000, 1640, 1232, 0, 0, 1640, 1232},	/* Preview */
{3280, 2464, 0, 0, 3280, 2464, 3280, 2464, 0000, 0000, 3280, 2464, 0, 0, 3280, 2464},	/* capture */
{3280, 2464, 0, 0, 3280, 2464, 3280, 2464, 0000, 0000, 3280, 2464, 0, 0, 3280, 2464},	/* video */
{3280, 2464, 16, 16, 3264, 2448, 816, 612, 0000, 0000, 816, 612, 0, 0, 816, 608},/*hight speed video*/
{3280, 2464, 720, 1024, 2560, 1440, 1280, 720, 0000, 0000, 1280, 720, 0, 0, 1280, 720},	/*slim video*/
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };
	/* kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor */
	iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte << 8) & 0xff00) | ((get_byte >> 8) & 0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {
	    (char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF) };
	/* kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor */
	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };
	/* kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor */
	iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[4] = { (char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF) };
	/* kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor */
	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d ", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* write_cmos_sensor_8(0x0104, 0x01); */
	write_cmos_sensor(0x0340, imgsensor.frame_length);
	write_cmos_sensor(0x0342, imgsensor.line_length);
	/* write_cmos_sensor_8(0x0104, 0x00); */

}				/*      set_dummy  */

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/* kal_int16 dummy_line; */
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	LOG_INF("framerate = %d, min framelength %d should enable?\n", framerate,
		min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length =
	    (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	/* dummy_line = frame_length - imgsensor.min_frame_length; */
	/* if (dummy_line < 0) */
	/* imgsensor.dummy_line = 0; */
	/* else */
	/* imgsensor.dummy_line = dummy_line; */
	/* imgsensor.frame_length = frame_length + imgsensor.dummy_line; */
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}				/*      set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	/* kal_uint32 frame_length = 0; */
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter =
	    (shutter >
	     (imgsensor_info.max_frame_length -
	      imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
					 imgsensor_info.margin) : shutter;
#if 1
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
	} else {
		/* Extend frame length */
		/* write_cmos_sensor_8(0x0104,0x01); */
		write_cmos_sensor(0x0340, imgsensor.frame_length);
		/* write_cmos_sensor_8(0x0104,0x00); */
	}
#endif
	/* Update Shutter */
	/* write_cmos_sensor_8(0x0104,0x01); */
	write_cmos_sensor(0x0340, imgsensor.frame_length);
	write_cmos_sensor(0x0202, shutter);
	/* write_cmos_sensor_8(0x0104,0x00); */
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);
}				/*      write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
/*
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}		//set_shutter
*/


static kal_uint16 gain2reg(kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0;

	reg_gain = gain / 2;
	return (kal_uint16) reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X  */
	/* [4:9] = M meams M X       */
	/* Total gain = M + N /16 X   */

	/*  */
	if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 32 * BASEGAIN)
			gain = 32 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	/* LOG_INF("gain = %d , reg_gain = 0x%x ", gain, reg_gain); */

	/* write_cmos_sensor_8(0x0104, 0x01); */
	write_cmos_sensor_8(0x0204, (reg_gain >> 8));
	write_cmos_sensor_8(0x0205, (reg_gain & 0xff));
	/* write_cmos_sensor_8(0x0104, 0x00); */

	return gain;
}				/*      set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	/* kal_uint8 iRation; */
	/* kal_uint8 iReg; */

	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
	se = le / 4;		/* 长曝光时间是短曝光时间8倍 */
#if 1
	if (imgsensor.ihdr_en) {

		spin_lock(&imgsensor_drv_lock);
		if (le > imgsensor.min_frame_length - imgsensor_info.margin)
			imgsensor.frame_length = le + imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length = imgsensor_info.max_frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (le < imgsensor_info.min_shutter)
			le = imgsensor_info.min_shutter;
		if (se < imgsensor_info.min_shutter)
			se = imgsensor_info.min_shutter;

		LOG_INF("frame length:0x%x, le:0x%x, se:0x%x, gain:0x%x\n", imgsensor.frame_length,
			le, se, gain);

		/* Extend frame length */

		write_cmos_sensor_8(0x0104, 0x01);

		write_cmos_sensor(0x0340, imgsensor.frame_length);

		write_cmos_sensor(0x0202, le);
		write_cmos_sensor_8(0x0218, 0x04);
		write_cmos_sensor_8(0x021A, 0x01);

		set_gain(gain);

		write_cmos_sensor_8(0x0104, 0x00);
	}
#endif

}

static void ihdr_write_shutter(kal_uint16 le, kal_uint16 se)
{
}



static void set_mirror_flip(kal_uint8 image_mirror)
{
#if 0
	LOG_INF("image_mirror = %d", image_mirror);

    /********************************************************
       *
       *   0x3820[2] ISP Vertical flip
       *   0x3820[1] Sensor Vertical flip
       *
       *   0x3821[2] ISP Horizontal mirror
       *   0x3821[1] Sensor Horizontal mirror
       *
       *   ISP and Sensor flip or mirror register bit should be the same!!
       *
       ********************************************************/
	spin_lock(&imgsensor_drv_lock);
	imgsensor.mirror = image_mirror;
	spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {

	case IMAGE_NORMAL:
		write_cmos_sensor_8(0x0101, 0x00);	/* Gr */
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor_8(0x0101, 0x01);
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor_8(0x0101, 0x02);
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor_8(0x0101, 0x03);	/* Gb */
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
	}
#endif
}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

/*************************************************************************
* FUNCTION
*	check_stremoff
*
* DESCRIPTION
*	waiting function until sensor streaming finish.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void check_stremoff(void)
{
	unsigned int i = 0, framecnt = 0;

	for (i = 0; i < 100; i++) {
		framecnt = read_cmos_sensor_8(0x0005);
		if (framecnt == 0xFF)
			return;
	}
	LOG_INF(" Stream Off Fail!\n");
	/*return;*/
}

static void sensor_init(void)
{
	LOG_INF("sensor_init() E\n");
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6010, 0x0001);
	mdelay(3);		/* Wait value must be at least 20000 MCLKs */

	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7971);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0xF468, 0x0017);
	write_cmos_sensor(0xF466, 0x0010);
	write_cmos_sensor(0xF416, 0x44C6);
	write_cmos_sensor(0xF418, 0x002F);
	write_cmos_sensor(0xF482, 0x09DC);
	write_cmos_sensor(0xF410, 0x0005);
	write_cmos_sensor(0xF412, 0x0002);
	write_cmos_sensor(0xF452, 0x001A);
	write_cmos_sensor(0xF448, 0x0013);
	write_cmos_sensor(0xF450, 0x000C);
	write_cmos_sensor(0x364C, 0x0014);
	write_cmos_sensor(0x3646, 0x0DE7);
	write_cmos_sensor(0x3648, 0x03FD);
	write_cmos_sensor(0x3C74, 0x0040);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1444);
	write_cmos_sensor(0x6F12, 0x8010);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0200, 0x0000);
	write_cmos_sensor(0x31B2, 0x0001);
	write_cmos_sensor(0xB136, 0x0000);
	write_cmos_sensor(0xB138, 0x0000);
	write_cmos_sensor(0x31BA, 0x0200);
}				/*      sensor_init  */

static void preview_setting(void)
{
	/* Preview 2320*1744 30fps 24M MCLK 4lane 1488Mbps/lane */
	/* preview 30.01fps */
	LOG_INF("preview_setting() E\n");
	write_cmos_sensor_8(0x0100, 0x00);
	check_stremoff();

	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x360A, 0x0705);
	write_cmos_sensor(0x3644, 0x084C);
	write_cmos_sensor(0x3C90, 0x06C0);
	write_cmos_sensor(0x3C92, 0x076C);
	write_cmos_sensor(0x35E4, 0x0001);
	write_cmos_sensor(0x3240, 0x005A);
	write_cmos_sensor(0x3244, 0x0071);
	write_cmos_sensor(0x324C, 0x0074);
	write_cmos_sensor(0x3460, 0x008B);
	write_cmos_sensor(0x3464, 0x0073);
	write_cmos_sensor(0x3468, 0x0031);
	write_cmos_sensor(0x346C, 0x000E);
	write_cmos_sensor(0x3470, 0x0085);
	write_cmos_sensor(0x3474, 0x0079);
	write_cmos_sensor(0x3478, 0x002B);
	write_cmos_sensor(0x3480, 0x002E);
	write_cmos_sensor(0x3484, 0x0011);
	write_cmos_sensor(0x3488, 0x0085);
	write_cmos_sensor(0x348C, 0x0079);
	write_cmos_sensor(0x3490, 0x002B);
	write_cmos_sensor(0x3254, 0x010F);
	write_cmos_sensor(0x3258, 0x005A);
	write_cmos_sensor(0x325C, 0x010D);
	write_cmos_sensor(0x3264, 0x0077);
	write_cmos_sensor(0x326C, 0x0021);
	write_cmos_sensor(0x3274, 0x001D);
	write_cmos_sensor(0x327C, 0x0020);
	write_cmos_sensor(0x3290, 0x0071);
	write_cmos_sensor(0x3294, 0x007B);
	write_cmos_sensor(0x32A4, 0x0006);
	write_cmos_sensor(0x32A8, 0x0025);
	write_cmos_sensor(0x32AC, 0x010D);
	write_cmos_sensor(0x32B0, 0x002E);
	write_cmos_sensor(0x32B8, 0x0088);
	write_cmos_sensor(0x32BC, 0x010D);
	write_cmos_sensor(0x32C0, 0x002C);
	write_cmos_sensor(0x32C8, 0x0086);
	write_cmos_sensor(0x32CC, 0x010F);
	write_cmos_sensor(0x32D4, 0x0005);
	write_cmos_sensor(0x32F8, 0x0091);
	write_cmos_sensor(0x32FC, 0x010B);
	write_cmos_sensor(0x3348, 0x008F);
	write_cmos_sensor(0x334C, 0x010D);
	write_cmos_sensor(0x3380, 0x002A);
	write_cmos_sensor(0x3384, 0x0111);
	write_cmos_sensor(0x3388, 0x002B);
	write_cmos_sensor(0x338C, 0x002D);
	write_cmos_sensor(0x3398, 0x010D);
	write_cmos_sensor(0x339C, 0x010F);
	write_cmos_sensor(0x349C, 0x006E);
	write_cmos_sensor(0x34B8, 0x0063);
	write_cmos_sensor(0x34CC, 0x006E);
	write_cmos_sensor(0x34D0, 0x0015);
	write_cmos_sensor(0x34D8, 0x0063);
	write_cmos_sensor(0x34E4, 0x0074);
	write_cmos_sensor(0x34FC, 0x00C8);
	write_cmos_sensor(0x3500, 0x0063);
	write_cmos_sensor(0x3514, 0x0074);
	write_cmos_sensor(0x3518, 0x0015);
	write_cmos_sensor(0x3520, 0x0063);
	write_cmos_sensor(0x3232, 0x0006);
	write_cmos_sensor(0x323A, 0x0007);
	write_cmos_sensor(0x323E, 0x0061);
	write_cmos_sensor(0x3242, 0x0078);
	write_cmos_sensor(0x3246, 0x005F);
	write_cmos_sensor(0x345E, 0x0097);
	write_cmos_sensor(0x3462, 0x0089);
	write_cmos_sensor(0x3466, 0x003B);
	write_cmos_sensor(0x346E, 0x0091);
	write_cmos_sensor(0x3472, 0x008F);
	write_cmos_sensor(0x3476, 0x0035);
	write_cmos_sensor(0x347E, 0x0038);
	write_cmos_sensor(0x3486, 0x0091);
	write_cmos_sensor(0x348A, 0x008F);
	write_cmos_sensor(0x348E, 0x0035);
	write_cmos_sensor(0x324E, 0x005F);
	write_cmos_sensor(0x3252, 0x011E);
	write_cmos_sensor(0x3256, 0x0061);
	write_cmos_sensor(0x325A, 0x011C);
	write_cmos_sensor(0x325E, 0x005F);
	write_cmos_sensor(0x3262, 0x007E);
	write_cmos_sensor(0x326A, 0x001F);
	write_cmos_sensor(0x3272, 0x001B);
	write_cmos_sensor(0x327A, 0x001E);
	write_cmos_sensor(0x3282, 0x005F);
	write_cmos_sensor(0x328E, 0x0078);
	write_cmos_sensor(0x3292, 0x0085);
	write_cmos_sensor(0x329A, 0x0006);
	write_cmos_sensor(0x32A6, 0x0023);
	write_cmos_sensor(0x32AA, 0x011C);
	write_cmos_sensor(0x32AE, 0x0030);
	write_cmos_sensor(0x32B2, 0x0061);
	write_cmos_sensor(0x32B6, 0x0090);
	write_cmos_sensor(0x32BA, 0x011C);
	write_cmos_sensor(0x32BE, 0x002E);
	write_cmos_sensor(0x32C2, 0x0063);
	write_cmos_sensor(0x32C6, 0x008E);
	write_cmos_sensor(0x32CA, 0x011E);
	write_cmos_sensor(0x32CE, 0x0004);
	write_cmos_sensor(0x32D2, 0x0003);
	write_cmos_sensor(0x32EE, 0x003A);
	write_cmos_sensor(0x32F2, 0x005F);
	write_cmos_sensor(0x32F6, 0x009A);
	write_cmos_sensor(0x32FA, 0x011A);
	write_cmos_sensor(0x32FE, 0x0067);
	write_cmos_sensor(0x3302, 0x007E);
	write_cmos_sensor(0x3306, 0x006E);
	write_cmos_sensor(0x330A, 0x0086);
	write_cmos_sensor(0x330E, 0x0076);
	write_cmos_sensor(0x3312, 0x0086);
	write_cmos_sensor(0x3316, 0x0067);
	write_cmos_sensor(0x331A, 0x0069);
	write_cmos_sensor(0x3326, 0x006E);
	write_cmos_sensor(0x332A, 0x0086);
	write_cmos_sensor(0x3336, 0x0067);
	write_cmos_sensor(0x333A, 0x0069);
	write_cmos_sensor(0x333E, 0x0038);
	write_cmos_sensor(0x3342, 0x0061);
	write_cmos_sensor(0x3346, 0x0098);
	write_cmos_sensor(0x334A, 0x011C);
	write_cmos_sensor(0x335E, 0x0067);
	write_cmos_sensor(0x3362, 0x0081);
	write_cmos_sensor(0x336E, 0x0067);
	write_cmos_sensor(0x3372, 0x0069);
	write_cmos_sensor(0x337E, 0x0025);
	write_cmos_sensor(0x3382, 0x0120);
	write_cmos_sensor(0x3386, 0x0026);
	write_cmos_sensor(0x338A, 0x0028);
	write_cmos_sensor(0x338E, 0x0061);
	write_cmos_sensor(0x3392, 0x0063);
	write_cmos_sensor(0x3396, 0x011C);
	write_cmos_sensor(0x339A, 0x011E);
	write_cmos_sensor(0x339E, 0x0061);
	write_cmos_sensor(0x349A, 0x0080);
	write_cmos_sensor(0x34B2, 0x00DC);
	write_cmos_sensor(0x34B6, 0x0061);
	write_cmos_sensor(0x34CA, 0x0080);
	write_cmos_sensor(0x34CE, 0x000B);
	write_cmos_sensor(0x34D6, 0x0061);
	write_cmos_sensor(0x34E2, 0x0086);
	write_cmos_sensor(0x34FA, 0x00E2);
	write_cmos_sensor(0x34FE, 0x0061);
	write_cmos_sensor(0x3512, 0x0086);
	write_cmos_sensor(0x3516, 0x000B);
	write_cmos_sensor(0x351E, 0x0061);
	write_cmos_sensor(0x3652, 0x007B);
	write_cmos_sensor(0x3654, 0x005F);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0348, 0x0CCF);
	write_cmos_sensor(0x0346, 0x0000);
	write_cmos_sensor(0x034A, 0x099F);
	write_cmos_sensor(0x034C, 0x0668);
	write_cmos_sensor(0x034E, 0x04D0);
	write_cmos_sensor(0x31AC, 0x0004);
	write_cmos_sensor(0x31B0, 0x0005);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0B04, 0x0101);
	write_cmos_sensor(0x307C, 0x0340);
	write_cmos_sensor(0x030E, 0x0079);
	write_cmos_sensor(0x300A, 0x0000);
	write_cmos_sensor(0x0342, 0x1230);
	write_cmos_sensor(0x0340, 0x0A04);
	write_cmos_sensor(0x3178, 0x003D);
	write_cmos_sensor(0x0900, 0x0112);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0003);
	write_cmos_sensor(0x0400, 0x0001);
	write_cmos_sensor(0x0404, 0x0020);
	write_cmos_sensor(0x0B00, 0x0080);
	write_cmos_sensor(0x0100, 0x0100);

	write_cmos_sensor_8(0x0100, 0x01);
}				/*      preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("capture_setting() E! currefps:%d\n", currefps);
	write_cmos_sensor_8(0x0100, 0x00);
	check_stremoff();
	/* full size 29.76fps */
	/* capture setting 4640*3488  24MHz 560MHz 1464Mbps/lane */
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x360A, 0x0705);
	write_cmos_sensor(0x3644, 0x084C);
	write_cmos_sensor(0x3C90, 0x06C0);
	write_cmos_sensor(0x3C92, 0x076C);
	write_cmos_sensor(0x35E4, 0x0001);
	write_cmos_sensor(0x3240, 0x005A);
	write_cmos_sensor(0x3244, 0x0071);
	write_cmos_sensor(0x324C, 0x0074);
	write_cmos_sensor(0x3460, 0x008B);
	write_cmos_sensor(0x3464, 0x0073);
	write_cmos_sensor(0x3468, 0x0031);
	write_cmos_sensor(0x346C, 0x000E);
	write_cmos_sensor(0x3470, 0x0085);
	write_cmos_sensor(0x3474, 0x0079);
	write_cmos_sensor(0x3478, 0x002B);
	write_cmos_sensor(0x3480, 0x002E);
	write_cmos_sensor(0x3484, 0x0011);
	write_cmos_sensor(0x3488, 0x0085);
	write_cmos_sensor(0x348C, 0x0079);
	write_cmos_sensor(0x3490, 0x002B);
	write_cmos_sensor(0x3254, 0x010F);
	write_cmos_sensor(0x3258, 0x005A);
	write_cmos_sensor(0x325C, 0x010D);
	write_cmos_sensor(0x3264, 0x0077);
	write_cmos_sensor(0x326C, 0x0021);
	write_cmos_sensor(0x3274, 0x001D);
	write_cmos_sensor(0x327C, 0x0020);
	write_cmos_sensor(0x3290, 0x0071);
	write_cmos_sensor(0x3294, 0x007B);
	write_cmos_sensor(0x32A4, 0x0006);
	write_cmos_sensor(0x32A8, 0x0025);
	write_cmos_sensor(0x32AC, 0x010D);
	write_cmos_sensor(0x32B0, 0x002E);
	write_cmos_sensor(0x32B8, 0x0088);
	write_cmos_sensor(0x32BC, 0x010D);
	write_cmos_sensor(0x32C0, 0x002C);
	write_cmos_sensor(0x32C8, 0x0086);
	write_cmos_sensor(0x32CC, 0x010F);
	write_cmos_sensor(0x32D4, 0x0005);
	write_cmos_sensor(0x32F8, 0x0091);
	write_cmos_sensor(0x32FC, 0x010B);
	write_cmos_sensor(0x3348, 0x008F);
	write_cmos_sensor(0x334C, 0x010D);
	write_cmos_sensor(0x3380, 0x002A);
	write_cmos_sensor(0x3384, 0x0111);
	write_cmos_sensor(0x3388, 0x002B);
	write_cmos_sensor(0x338C, 0x002D);
	write_cmos_sensor(0x3398, 0x010D);
	write_cmos_sensor(0x339C, 0x010F);
	write_cmos_sensor(0x349C, 0x006E);
	write_cmos_sensor(0x34B8, 0x0063);
	write_cmos_sensor(0x34CC, 0x006E);
	write_cmos_sensor(0x34D0, 0x0015);
	write_cmos_sensor(0x34D8, 0x0063);
	write_cmos_sensor(0x34E4, 0x0074);
	write_cmos_sensor(0x34FC, 0x00C8);
	write_cmos_sensor(0x3500, 0x0063);
	write_cmos_sensor(0x3514, 0x0074);
	write_cmos_sensor(0x3518, 0x0015);
	write_cmos_sensor(0x3520, 0x0063);
	write_cmos_sensor(0x3232, 0x0006);
	write_cmos_sensor(0x323A, 0x0007);
	write_cmos_sensor(0x323E, 0x0061);
	write_cmos_sensor(0x3242, 0x0078);
	write_cmos_sensor(0x3246, 0x005F);
	write_cmos_sensor(0x345E, 0x0097);
	write_cmos_sensor(0x3462, 0x0089);
	write_cmos_sensor(0x3466, 0x003B);
	write_cmos_sensor(0x346E, 0x0091);
	write_cmos_sensor(0x3472, 0x008F);
	write_cmos_sensor(0x3476, 0x0035);
	write_cmos_sensor(0x347E, 0x0038);
	write_cmos_sensor(0x3486, 0x0091);
	write_cmos_sensor(0x348A, 0x008F);
	write_cmos_sensor(0x348E, 0x0035);
	write_cmos_sensor(0x324E, 0x005F);
	write_cmos_sensor(0x3252, 0x011E);
	write_cmos_sensor(0x3256, 0x0061);
	write_cmos_sensor(0x325A, 0x011C);
	write_cmos_sensor(0x325E, 0x005F);
	write_cmos_sensor(0x3262, 0x007E);
	write_cmos_sensor(0x326A, 0x001F);
	write_cmos_sensor(0x3272, 0x001B);
	write_cmos_sensor(0x327A, 0x001E);
	write_cmos_sensor(0x3282, 0x005F);
	write_cmos_sensor(0x328E, 0x0078);
	write_cmos_sensor(0x3292, 0x0085);
	write_cmos_sensor(0x329A, 0x0006);
	write_cmos_sensor(0x32A6, 0x0023);
	write_cmos_sensor(0x32AA, 0x011C);
	write_cmos_sensor(0x32AE, 0x0030);
	write_cmos_sensor(0x32B2, 0x0061);
	write_cmos_sensor(0x32B6, 0x0090);
	write_cmos_sensor(0x32BA, 0x011C);
	write_cmos_sensor(0x32BE, 0x002E);
	write_cmos_sensor(0x32C2, 0x0063);
	write_cmos_sensor(0x32C6, 0x008E);
	write_cmos_sensor(0x32CA, 0x011E);
	write_cmos_sensor(0x32CE, 0x0004);
	write_cmos_sensor(0x32D2, 0x0003);
	write_cmos_sensor(0x32EE, 0x003A);
	write_cmos_sensor(0x32F2, 0x005F);
	write_cmos_sensor(0x32F6, 0x009A);
	write_cmos_sensor(0x32FA, 0x011A);
	write_cmos_sensor(0x32FE, 0x0067);
	write_cmos_sensor(0x3302, 0x007E);
	write_cmos_sensor(0x3306, 0x006E);
	write_cmos_sensor(0x330A, 0x0086);
	write_cmos_sensor(0x330E, 0x0076);
	write_cmos_sensor(0x3312, 0x0086);
	write_cmos_sensor(0x3316, 0x0067);
	write_cmos_sensor(0x331A, 0x0069);
	write_cmos_sensor(0x3326, 0x006E);
	write_cmos_sensor(0x332A, 0x0086);
	write_cmos_sensor(0x3336, 0x0067);
	write_cmos_sensor(0x333A, 0x0069);
	write_cmos_sensor(0x333E, 0x0038);
	write_cmos_sensor(0x3342, 0x0061);
	write_cmos_sensor(0x3346, 0x0098);
	write_cmos_sensor(0x334A, 0x011C);
	write_cmos_sensor(0x335E, 0x0067);
	write_cmos_sensor(0x3362, 0x0081);
	write_cmos_sensor(0x336E, 0x0067);
	write_cmos_sensor(0x3372, 0x0069);
	write_cmos_sensor(0x337E, 0x0025);
	write_cmos_sensor(0x3382, 0x0120);
	write_cmos_sensor(0x3386, 0x0026);
	write_cmos_sensor(0x338A, 0x0028);
	write_cmos_sensor(0x338E, 0x0061);
	write_cmos_sensor(0x3392, 0x0063);
	write_cmos_sensor(0x3396, 0x011C);
	write_cmos_sensor(0x339A, 0x011E);
	write_cmos_sensor(0x339E, 0x0061);
	write_cmos_sensor(0x349A, 0x0080);
	write_cmos_sensor(0x34B2, 0x00DC);
	write_cmos_sensor(0x34B6, 0x0061);
	write_cmos_sensor(0x34CA, 0x0080);
	write_cmos_sensor(0x34CE, 0x000B);
	write_cmos_sensor(0x34D6, 0x0061);
	write_cmos_sensor(0x34E2, 0x0086);
	write_cmos_sensor(0x34FA, 0x00E2);
	write_cmos_sensor(0x34FE, 0x0061);
	write_cmos_sensor(0x3512, 0x0086);
	write_cmos_sensor(0x3516, 0x000B);
	write_cmos_sensor(0x351E, 0x0061);
	write_cmos_sensor(0x3652, 0x007B);
	write_cmos_sensor(0x3654, 0x005F);
	if (currefps == 300) {
		write_cmos_sensor(0x0344, 0x0000);
		write_cmos_sensor(0x0348, 0x0CCF);
		write_cmos_sensor(0x0346, 0x0000);
		write_cmos_sensor(0x034A, 0x099F);
		write_cmos_sensor(0x034C, 0x0CD0);
		write_cmos_sensor(0x034E, 0x09A0);
		write_cmos_sensor(0x31AC, 0x0004);
		write_cmos_sensor(0x31B0, 0x0005);
		write_cmos_sensor(0x0202, 0x0100);
		write_cmos_sensor(0x0B04, 0x0101);
		write_cmos_sensor(0x307C, 0x0340);
		write_cmos_sensor(0x030E, 0x0079);
		write_cmos_sensor(0x300A, 0x0000);
		write_cmos_sensor(0x0342, 0x1230);
		write_cmos_sensor(0x0340, 0x0A04);
		write_cmos_sensor(0x3178, 0x007B);
		write_cmos_sensor(0x0900, 0x0011);
		write_cmos_sensor(0x0380, 0x0001);
		write_cmos_sensor(0x0382, 0x0001);
		write_cmos_sensor(0x0384, 0x0001);
		write_cmos_sensor(0x0386, 0x0001);
		write_cmos_sensor(0x0400, 0x0000);
		write_cmos_sensor(0x0404, 0x0010);
		write_cmos_sensor(0x0B00, 0x0080);
		write_cmos_sensor(0x0100, 0x0100);
	} else if (currefps == 240) {
		write_cmos_sensor(0x0344, 0x0000);
		write_cmos_sensor(0x0348, 0x0CCF);
		write_cmos_sensor(0x0346, 0x0000);
		write_cmos_sensor(0x034A, 0x099F);
		write_cmos_sensor(0x034C, 0x0CD0);
		write_cmos_sensor(0x034E, 0x09A0);
		write_cmos_sensor(0x31AC, 0x0004);
		write_cmos_sensor(0x31B0, 0x0005);
		write_cmos_sensor(0x0202, 0x0100);
		write_cmos_sensor(0x0B04, 0x0101);
		write_cmos_sensor(0x307C, 0x0340);
		write_cmos_sensor(0x030E, 0x0079);
		write_cmos_sensor(0x300A, 0x0000);
		write_cmos_sensor(0x0342, 0x1230);
		write_cmos_sensor(0x0340, 0x0C94);
		write_cmos_sensor(0x3178, 0x007B);
		write_cmos_sensor(0x0900, 0x0011);
		write_cmos_sensor(0x0380, 0x0001);
		write_cmos_sensor(0x0382, 0x0001);
		write_cmos_sensor(0x0384, 0x0001);
		write_cmos_sensor(0x0386, 0x0001);
		write_cmos_sensor(0x0400, 0x0000);
		write_cmos_sensor(0x0404, 0x0010);
		write_cmos_sensor(0x0B00, 0x0080);
		write_cmos_sensor(0x0100, 0x0100);
	} else {		/* 15fps */
		write_cmos_sensor(0x0344, 0x0000);
		write_cmos_sensor(0x0348, 0x0CCF);
		write_cmos_sensor(0x0346, 0x0000);
		write_cmos_sensor(0x034A, 0x099F);
		write_cmos_sensor(0x034C, 0x0CD0);
		write_cmos_sensor(0x034E, 0x09A0);
		write_cmos_sensor(0x31AC, 0x0004);
		write_cmos_sensor(0x31B0, 0x0005);
		write_cmos_sensor(0x0202, 0x0100);
		write_cmos_sensor(0x0B04, 0x0101);
		write_cmos_sensor(0x307C, 0x0340);
		write_cmos_sensor(0x030E, 0x0079);
		write_cmos_sensor(0x300A, 0x0000);
		write_cmos_sensor(0x0342, 0x1230);
		write_cmos_sensor(0x0340, 0x1408);
		write_cmos_sensor(0x3178, 0x007B);
		write_cmos_sensor(0x0900, 0x0011);
		write_cmos_sensor(0x0380, 0x0001);
		write_cmos_sensor(0x0382, 0x0001);
		write_cmos_sensor(0x0384, 0x0001);
		write_cmos_sensor(0x0386, 0x0001);
		write_cmos_sensor(0x0400, 0x0000);
		write_cmos_sensor(0x0404, 0x0010);
		write_cmos_sensor(0x0B00, 0x0080);
		write_cmos_sensor(0x0100, 0x0100);
	}

	write_cmos_sensor_8(0x0100, 0x01);
}




static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal_video_setting() E! currefps:%d\n", currefps);
	/* full size 30fps */
	write_cmos_sensor_8(0x0100, 0x00);
	check_stremoff();

	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x360A, 0x0705);
	write_cmos_sensor(0x3644, 0x084C);
	write_cmos_sensor(0x3C90, 0x06C0);
	write_cmos_sensor(0x3C92, 0x076C);
	write_cmos_sensor(0x35E4, 0x0001);
	write_cmos_sensor(0x3240, 0x005A);
	write_cmos_sensor(0x3244, 0x0071);
	write_cmos_sensor(0x324C, 0x0074);
	write_cmos_sensor(0x3460, 0x008B);
	write_cmos_sensor(0x3464, 0x0073);
	write_cmos_sensor(0x3468, 0x0031);
	write_cmos_sensor(0x346C, 0x000E);
	write_cmos_sensor(0x3470, 0x0085);
	write_cmos_sensor(0x3474, 0x0079);
	write_cmos_sensor(0x3478, 0x002B);
	write_cmos_sensor(0x3480, 0x002E);
	write_cmos_sensor(0x3484, 0x0011);
	write_cmos_sensor(0x3488, 0x0085);
	write_cmos_sensor(0x348C, 0x0079);
	write_cmos_sensor(0x3490, 0x002B);
	write_cmos_sensor(0x3254, 0x010F);
	write_cmos_sensor(0x3258, 0x005A);
	write_cmos_sensor(0x325C, 0x010D);
	write_cmos_sensor(0x3264, 0x0077);
	write_cmos_sensor(0x326C, 0x0021);
	write_cmos_sensor(0x3274, 0x001D);
	write_cmos_sensor(0x327C, 0x0020);
	write_cmos_sensor(0x3290, 0x0071);
	write_cmos_sensor(0x3294, 0x007B);
	write_cmos_sensor(0x32A4, 0x0006);
	write_cmos_sensor(0x32A8, 0x0025);
	write_cmos_sensor(0x32AC, 0x010D);
	write_cmos_sensor(0x32B0, 0x002E);
	write_cmos_sensor(0x32B8, 0x0088);
	write_cmos_sensor(0x32BC, 0x010D);
	write_cmos_sensor(0x32C0, 0x002C);
	write_cmos_sensor(0x32C8, 0x0086);
	write_cmos_sensor(0x32CC, 0x010F);
	write_cmos_sensor(0x32D4, 0x0005);
	write_cmos_sensor(0x32F8, 0x0091);
	write_cmos_sensor(0x32FC, 0x010B);
	write_cmos_sensor(0x3348, 0x008F);
	write_cmos_sensor(0x334C, 0x010D);
	write_cmos_sensor(0x3380, 0x002A);
	write_cmos_sensor(0x3384, 0x0111);
	write_cmos_sensor(0x3388, 0x002B);
	write_cmos_sensor(0x338C, 0x002D);
	write_cmos_sensor(0x3398, 0x010D);
	write_cmos_sensor(0x339C, 0x010F);
	write_cmos_sensor(0x349C, 0x006E);
	write_cmos_sensor(0x34B8, 0x0063);
	write_cmos_sensor(0x34CC, 0x006E);
	write_cmos_sensor(0x34D0, 0x0015);
	write_cmos_sensor(0x34D8, 0x0063);
	write_cmos_sensor(0x34E4, 0x0074);
	write_cmos_sensor(0x34FC, 0x00C8);
	write_cmos_sensor(0x3500, 0x0063);
	write_cmos_sensor(0x3514, 0x0074);
	write_cmos_sensor(0x3518, 0x0015);
	write_cmos_sensor(0x3520, 0x0063);
	write_cmos_sensor(0x3232, 0x0006);
	write_cmos_sensor(0x323A, 0x0007);
	write_cmos_sensor(0x323E, 0x0061);
	write_cmos_sensor(0x3242, 0x0078);
	write_cmos_sensor(0x3246, 0x005F);
	write_cmos_sensor(0x345E, 0x0097);
	write_cmos_sensor(0x3462, 0x0089);
	write_cmos_sensor(0x3466, 0x003B);
	write_cmos_sensor(0x346E, 0x0091);
	write_cmos_sensor(0x3472, 0x008F);
	write_cmos_sensor(0x3476, 0x0035);
	write_cmos_sensor(0x347E, 0x0038);
	write_cmos_sensor(0x3486, 0x0091);
	write_cmos_sensor(0x348A, 0x008F);
	write_cmos_sensor(0x348E, 0x0035);
	write_cmos_sensor(0x324E, 0x005F);
	write_cmos_sensor(0x3252, 0x011E);
	write_cmos_sensor(0x3256, 0x0061);
	write_cmos_sensor(0x325A, 0x011C);
	write_cmos_sensor(0x325E, 0x005F);
	write_cmos_sensor(0x3262, 0x007E);
	write_cmos_sensor(0x326A, 0x001F);
	write_cmos_sensor(0x3272, 0x001B);
	write_cmos_sensor(0x327A, 0x001E);
	write_cmos_sensor(0x3282, 0x005F);
	write_cmos_sensor(0x328E, 0x0078);
	write_cmos_sensor(0x3292, 0x0085);
	write_cmos_sensor(0x329A, 0x0006);
	write_cmos_sensor(0x32A6, 0x0023);
	write_cmos_sensor(0x32AA, 0x011C);
	write_cmos_sensor(0x32AE, 0x0030);
	write_cmos_sensor(0x32B2, 0x0061);
	write_cmos_sensor(0x32B6, 0x0090);
	write_cmos_sensor(0x32BA, 0x011C);
	write_cmos_sensor(0x32BE, 0x002E);
	write_cmos_sensor(0x32C2, 0x0063);
	write_cmos_sensor(0x32C6, 0x008E);
	write_cmos_sensor(0x32CA, 0x011E);
	write_cmos_sensor(0x32CE, 0x0004);
	write_cmos_sensor(0x32D2, 0x0003);
	write_cmos_sensor(0x32EE, 0x003A);
	write_cmos_sensor(0x32F2, 0x005F);
	write_cmos_sensor(0x32F6, 0x009A);
	write_cmos_sensor(0x32FA, 0x011A);
	write_cmos_sensor(0x32FE, 0x0067);
	write_cmos_sensor(0x3302, 0x007E);
	write_cmos_sensor(0x3306, 0x006E);
	write_cmos_sensor(0x330A, 0x0086);
	write_cmos_sensor(0x330E, 0x0076);
	write_cmos_sensor(0x3312, 0x0086);
	write_cmos_sensor(0x3316, 0x0067);
	write_cmos_sensor(0x331A, 0x0069);
	write_cmos_sensor(0x3326, 0x006E);
	write_cmos_sensor(0x332A, 0x0086);
	write_cmos_sensor(0x3336, 0x0067);
	write_cmos_sensor(0x333A, 0x0069);
	write_cmos_sensor(0x333E, 0x0038);
	write_cmos_sensor(0x3342, 0x0061);
	write_cmos_sensor(0x3346, 0x0098);
	write_cmos_sensor(0x334A, 0x011C);
	write_cmos_sensor(0x335E, 0x0067);
	write_cmos_sensor(0x3362, 0x0081);
	write_cmos_sensor(0x336E, 0x0067);
	write_cmos_sensor(0x3372, 0x0069);
	write_cmos_sensor(0x337E, 0x0025);
	write_cmos_sensor(0x3382, 0x0120);
	write_cmos_sensor(0x3386, 0x0026);
	write_cmos_sensor(0x338A, 0x0028);
	write_cmos_sensor(0x338E, 0x0061);
	write_cmos_sensor(0x3392, 0x0063);
	write_cmos_sensor(0x3396, 0x011C);
	write_cmos_sensor(0x339A, 0x011E);
	write_cmos_sensor(0x339E, 0x0061);
	write_cmos_sensor(0x349A, 0x0080);
	write_cmos_sensor(0x34B2, 0x00DC);
	write_cmos_sensor(0x34B6, 0x0061);
	write_cmos_sensor(0x34CA, 0x0080);
	write_cmos_sensor(0x34CE, 0x000B);
	write_cmos_sensor(0x34D6, 0x0061);
	write_cmos_sensor(0x34E2, 0x0086);
	write_cmos_sensor(0x34FA, 0x00E2);
	write_cmos_sensor(0x34FE, 0x0061);
	write_cmos_sensor(0x3512, 0x0086);
	write_cmos_sensor(0x3516, 0x000B);
	write_cmos_sensor(0x351E, 0x0061);
	write_cmos_sensor(0x3652, 0x007B);
	write_cmos_sensor(0x3654, 0x005F);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0348, 0x0CCF);
	write_cmos_sensor(0x0346, 0x0000);
	write_cmos_sensor(0x034A, 0x099F);
	write_cmos_sensor(0x034C, 0x0CD0);
	write_cmos_sensor(0x034E, 0x09A0);
	write_cmos_sensor(0x31AC, 0x0004);
	write_cmos_sensor(0x31B0, 0x0005);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0B04, 0x0101);
	write_cmos_sensor(0x307C, 0x0340);
	write_cmos_sensor(0x030E, 0x0079);
	write_cmos_sensor(0x300A, 0x0000);
	write_cmos_sensor(0x0342, 0x1230);
	write_cmos_sensor(0x0340, 0x0A04);
	write_cmos_sensor(0x3178, 0x007B);
	write_cmos_sensor(0x0900, 0x0011);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0001);
	write_cmos_sensor(0x0400, 0x0000);
	write_cmos_sensor(0x0404, 0x0010);
	write_cmos_sensor(0x0B00, 0x0080);
	write_cmos_sensor(0x0100, 0x0100);

	write_cmos_sensor_8(0x0100, 0x01);
}

static void hs_video_setting(void)
{
	LOG_INF("hs_video_setting() E\n");
	/* 720p 120fps */
	write_cmos_sensor_8(0x0100, 0x00);
	check_stremoff();

	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x360A, 0x0507);
	write_cmos_sensor(0x3644, 0x0800);
	write_cmos_sensor(0x3C90, 0x076C);
	write_cmos_sensor(0x3C92, 0x06C0);
	write_cmos_sensor(0x35E4, 0x0101);
	write_cmos_sensor(0x3240, 0x0059);
	write_cmos_sensor(0x3244, 0x0070);
	write_cmos_sensor(0x324C, 0x0072);
	write_cmos_sensor(0x3460, 0x0083);
	write_cmos_sensor(0x3464, 0x0079);
	write_cmos_sensor(0x3468, 0x0039);
	write_cmos_sensor(0x346C, 0x0010);
	write_cmos_sensor(0x3470, 0x007F);
	write_cmos_sensor(0x3474, 0x007D);
	write_cmos_sensor(0x3478, 0x0035);
	write_cmos_sensor(0x3480, 0x0037);
	write_cmos_sensor(0x3484, 0x0012);
	write_cmos_sensor(0x3488, 0x007F);
	write_cmos_sensor(0x348C, 0x007D);
	write_cmos_sensor(0x3490, 0x0035);
	write_cmos_sensor(0x3254, 0x0110);
	write_cmos_sensor(0x3258, 0x0059);
	write_cmos_sensor(0x325C, 0x010E);
	write_cmos_sensor(0x3264, 0x0076);
	write_cmos_sensor(0x326C, 0x001F);
	write_cmos_sensor(0x3274, 0x001B);
	write_cmos_sensor(0x327C, 0x001E);
	write_cmos_sensor(0x3290, 0x0070);
	write_cmos_sensor(0x3294, 0x0076);
	write_cmos_sensor(0x32A4, 0x0007);
	write_cmos_sensor(0x32A8, 0x0023);
	write_cmos_sensor(0x32AC, 0x010E);
	write_cmos_sensor(0x32B0, 0x0030);
	write_cmos_sensor(0x32B8, 0x008B);
	write_cmos_sensor(0x32BC, 0x010E);
	write_cmos_sensor(0x32C0, 0x002E);
	write_cmos_sensor(0x32C8, 0x0089);
	write_cmos_sensor(0x32CC, 0x0110);
	write_cmos_sensor(0x32D4, 0x0003);
	write_cmos_sensor(0x32F8, 0x0092);
	write_cmos_sensor(0x32FC, 0x010C);
	write_cmos_sensor(0x3348, 0x0090);
	write_cmos_sensor(0x334C, 0x010E);
	write_cmos_sensor(0x3380, 0x0025);
	write_cmos_sensor(0x3384, 0x0112);
	write_cmos_sensor(0x3388, 0x0026);
	write_cmos_sensor(0x338C, 0x0028);
	write_cmos_sensor(0x3398, 0x010E);
	write_cmos_sensor(0x339C, 0x0110);
	write_cmos_sensor(0x349C, 0x0078);
	write_cmos_sensor(0x34B8, 0x0059);
	write_cmos_sensor(0x34CC, 0x0078);
	write_cmos_sensor(0x34D0, 0x0009);
	write_cmos_sensor(0x34D8, 0x0059);
	write_cmos_sensor(0x34E4, 0x007C);
	write_cmos_sensor(0x34FC, 0x00C6);
	write_cmos_sensor(0x3500, 0x0059);
	write_cmos_sensor(0x3514, 0x007C);
	write_cmos_sensor(0x3518, 0x0009);
	write_cmos_sensor(0x3520, 0x0059);
	write_cmos_sensor(0x3232, 0x0007);
	write_cmos_sensor(0x323A, 0x0008);
	write_cmos_sensor(0x323E, 0x0060);
	write_cmos_sensor(0x3242, 0x0077);
	write_cmos_sensor(0x3246, 0x005D);
	write_cmos_sensor(0x345E, 0x008B);
	write_cmos_sensor(0x3462, 0x0073);
	write_cmos_sensor(0x3466, 0x0031);
	write_cmos_sensor(0x346E, 0x0085);
	write_cmos_sensor(0x3472, 0x0079);
	write_cmos_sensor(0x3476, 0x002B);
	write_cmos_sensor(0x347E, 0x002E);
	write_cmos_sensor(0x3486, 0x0085);
	write_cmos_sensor(0x348A, 0x0079);
	write_cmos_sensor(0x348E, 0x002B);
	write_cmos_sensor(0x324E, 0x005D);
	write_cmos_sensor(0x3252, 0x011D);
	write_cmos_sensor(0x3256, 0x0060);
	write_cmos_sensor(0x325A, 0x011B);
	write_cmos_sensor(0x325E, 0x005D);
	write_cmos_sensor(0x3262, 0x007D);
	write_cmos_sensor(0x326A, 0x0021);
	write_cmos_sensor(0x3272, 0x001D);
	write_cmos_sensor(0x327A, 0x0020);
	write_cmos_sensor(0x3282, 0x005D);
	write_cmos_sensor(0x328E, 0x0077);
	write_cmos_sensor(0x3292, 0x0081);
	write_cmos_sensor(0x329A, 0x0007);
	write_cmos_sensor(0x32A6, 0x0025);
	write_cmos_sensor(0x32AA, 0x011B);
	write_cmos_sensor(0x32AE, 0x002E);
	write_cmos_sensor(0x32B2, 0x005F);
	write_cmos_sensor(0x32B6, 0x008F);
	write_cmos_sensor(0x32BA, 0x011B);
	write_cmos_sensor(0x32BE, 0x002C);
	write_cmos_sensor(0x32C2, 0x0061);
	write_cmos_sensor(0x32C6, 0x008D);
	write_cmos_sensor(0x32CA, 0x011D);
	write_cmos_sensor(0x32CE, 0x0005);
	write_cmos_sensor(0x32D2, 0x0005);
	write_cmos_sensor(0x32EE, 0x0038);
	write_cmos_sensor(0x32F2, 0x005D);
	write_cmos_sensor(0x32F6, 0x0099);
	write_cmos_sensor(0x32FA, 0x0119);
	write_cmos_sensor(0x32FE, 0x0065);
	write_cmos_sensor(0x3302, 0x007C);
	write_cmos_sensor(0x3306, 0x006C);
	write_cmos_sensor(0x330A, 0x0084);
	write_cmos_sensor(0x330E, 0x0074);
	write_cmos_sensor(0x3312, 0x0084);
	write_cmos_sensor(0x3316, 0x0065);
	write_cmos_sensor(0x331A, 0x0067);
	write_cmos_sensor(0x3326, 0x006C);
	write_cmos_sensor(0x332A, 0x0084);
	write_cmos_sensor(0x3336, 0x0065);
	write_cmos_sensor(0x333A, 0x0067);
	write_cmos_sensor(0x333E, 0x0036);
	write_cmos_sensor(0x3342, 0x005F);
	write_cmos_sensor(0x3346, 0x0097);
	write_cmos_sensor(0x334A, 0x011B);
	write_cmos_sensor(0x335E, 0x0065);
	write_cmos_sensor(0x3362, 0x007F);
	write_cmos_sensor(0x336E, 0x0065);
	write_cmos_sensor(0x3372, 0x0067);
	write_cmos_sensor(0x337E, 0x002A);
	write_cmos_sensor(0x3382, 0x011F);
	write_cmos_sensor(0x3386, 0x002B);
	write_cmos_sensor(0x338A, 0x002D);
	write_cmos_sensor(0x338E, 0x005F);
	write_cmos_sensor(0x3392, 0x0061);
	write_cmos_sensor(0x3396, 0x011B);
	write_cmos_sensor(0x339A, 0x011D);
	write_cmos_sensor(0x339E, 0x005F);
	write_cmos_sensor(0x349A, 0x0074);
	write_cmos_sensor(0x34B2, 0x00CE);
	write_cmos_sensor(0x34B6, 0x0069);
	write_cmos_sensor(0x34CA, 0x0074);
	write_cmos_sensor(0x34CE, 0x0015);
	write_cmos_sensor(0x34D6, 0x0069);
	write_cmos_sensor(0x34E2, 0x007A);
	write_cmos_sensor(0x34FA, 0x00D4);
	write_cmos_sensor(0x34FE, 0x0069);
	write_cmos_sensor(0x3512, 0x007A);
	write_cmos_sensor(0x3516, 0x0015);
	write_cmos_sensor(0x351E, 0x0069);
	write_cmos_sensor(0x3652, 0x0065);
	write_cmos_sensor(0x3654, 0x0073);
	write_cmos_sensor(0x0344, 0x0008);
	write_cmos_sensor(0x0348, 0x0CC7);
	write_cmos_sensor(0x0346, 0x0008);
	write_cmos_sensor(0x034A, 0x0997);
	write_cmos_sensor(0x034C, 0x0330);
	write_cmos_sensor(0x034E, 0x0260);/* origin is 0x0264 */
	write_cmos_sensor(0x31AC, 0x0004);
	write_cmos_sensor(0x31B0, 0x0005);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0B04, 0x0101);
	write_cmos_sensor(0x307C, 0x0340);
	write_cmos_sensor(0x030E, 0x0079);
	write_cmos_sensor(0x300A, 0x0000);
	write_cmos_sensor(0x0342, 0x1150);
	write_cmos_sensor(0x0340, 0x02A3);
	write_cmos_sensor(0x3178, 0x0030);
	write_cmos_sensor(0x0900, 0x0113);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0007);
	write_cmos_sensor(0x0400, 0x0001);
	write_cmos_sensor(0x0404, 0x0040);
	write_cmos_sensor(0x0B00, 0x0080);
	write_cmos_sensor(0x0100, 0x0100);

	write_cmos_sensor_8(0x0100, 0x01);
}

static void slim_video_setting(void)
{
	LOG_INF("slim_video_setting() E\n");
	/* 1080p 60fps */
	write_cmos_sensor_8(0x0100, 0x00);
	check_stremoff();

	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x360A, 0x0705);
	write_cmos_sensor(0x3644, 0x084C);
	write_cmos_sensor(0x3C90, 0x06C0);
	write_cmos_sensor(0x3C92, 0x076C);
	write_cmos_sensor(0x35E4, 0x0001);
	write_cmos_sensor(0x3240, 0x005A);
	write_cmos_sensor(0x3244, 0x0071);
	write_cmos_sensor(0x324C, 0x0074);
	write_cmos_sensor(0x3460, 0x008B);
	write_cmos_sensor(0x3464, 0x0073);
	write_cmos_sensor(0x3468, 0x0031);
	write_cmos_sensor(0x346C, 0x000E);
	write_cmos_sensor(0x3470, 0x0085);
	write_cmos_sensor(0x3474, 0x0079);
	write_cmos_sensor(0x3478, 0x002B);
	write_cmos_sensor(0x3480, 0x002E);
	write_cmos_sensor(0x3484, 0x0011);
	write_cmos_sensor(0x3488, 0x0085);
	write_cmos_sensor(0x348C, 0x0079);
	write_cmos_sensor(0x3490, 0x002B);
	write_cmos_sensor(0x3254, 0x010F);
	write_cmos_sensor(0x3258, 0x005A);
	write_cmos_sensor(0x325C, 0x010D);
	write_cmos_sensor(0x3264, 0x0077);
	write_cmos_sensor(0x326C, 0x0021);
	write_cmos_sensor(0x3274, 0x001D);
	write_cmos_sensor(0x327C, 0x0020);
	write_cmos_sensor(0x3290, 0x0071);
	write_cmos_sensor(0x3294, 0x007B);
	write_cmos_sensor(0x32A4, 0x0006);
	write_cmos_sensor(0x32A8, 0x0025);
	write_cmos_sensor(0x32AC, 0x010D);
	write_cmos_sensor(0x32B0, 0x002E);
	write_cmos_sensor(0x32B8, 0x0088);
	write_cmos_sensor(0x32BC, 0x010D);
	write_cmos_sensor(0x32C0, 0x002C);
	write_cmos_sensor(0x32C8, 0x0086);
	write_cmos_sensor(0x32CC, 0x010F);
	write_cmos_sensor(0x32D4, 0x0005);
	write_cmos_sensor(0x32F8, 0x0091);
	write_cmos_sensor(0x32FC, 0x010B);
	write_cmos_sensor(0x3348, 0x008F);
	write_cmos_sensor(0x334C, 0x010D);
	write_cmos_sensor(0x3380, 0x002A);
	write_cmos_sensor(0x3384, 0x0111);
	write_cmos_sensor(0x3388, 0x002B);
	write_cmos_sensor(0x338C, 0x002D);
	write_cmos_sensor(0x3398, 0x010D);
	write_cmos_sensor(0x339C, 0x010F);
	write_cmos_sensor(0x349C, 0x006E);
	write_cmos_sensor(0x34B8, 0x0063);
	write_cmos_sensor(0x34CC, 0x006E);
	write_cmos_sensor(0x34D0, 0x0015);
	write_cmos_sensor(0x34D8, 0x0063);
	write_cmos_sensor(0x34E4, 0x0074);
	write_cmos_sensor(0x34FC, 0x00C8);
	write_cmos_sensor(0x3500, 0x0063);
	write_cmos_sensor(0x3514, 0x0074);
	write_cmos_sensor(0x3518, 0x0015);
	write_cmos_sensor(0x3520, 0x0063);
	write_cmos_sensor(0x3232, 0x0006);
	write_cmos_sensor(0x323A, 0x0007);
	write_cmos_sensor(0x323E, 0x0061);
	write_cmos_sensor(0x3242, 0x0078);
	write_cmos_sensor(0x3246, 0x005F);
	write_cmos_sensor(0x345E, 0x0097);
	write_cmos_sensor(0x3462, 0x0089);
	write_cmos_sensor(0x3466, 0x003B);
	write_cmos_sensor(0x346E, 0x0091);
	write_cmos_sensor(0x3472, 0x008F);
	write_cmos_sensor(0x3476, 0x0035);
	write_cmos_sensor(0x347E, 0x0038);
	write_cmos_sensor(0x3486, 0x0091);
	write_cmos_sensor(0x348A, 0x008F);
	write_cmos_sensor(0x348E, 0x0035);
	write_cmos_sensor(0x324E, 0x005F);
	write_cmos_sensor(0x3252, 0x011E);
	write_cmos_sensor(0x3256, 0x0061);
	write_cmos_sensor(0x325A, 0x011C);
	write_cmos_sensor(0x325E, 0x005F);
	write_cmos_sensor(0x3262, 0x007E);
	write_cmos_sensor(0x326A, 0x001F);
	write_cmos_sensor(0x3272, 0x001B);
	write_cmos_sensor(0x327A, 0x001E);
	write_cmos_sensor(0x3282, 0x005F);
	write_cmos_sensor(0x328E, 0x0078);
	write_cmos_sensor(0x3292, 0x0085);
	write_cmos_sensor(0x329A, 0x0006);
	write_cmos_sensor(0x32A6, 0x0023);
	write_cmos_sensor(0x32AA, 0x011C);
	write_cmos_sensor(0x32AE, 0x0030);
	write_cmos_sensor(0x32B2, 0x0061);
	write_cmos_sensor(0x32B6, 0x0090);
	write_cmos_sensor(0x32BA, 0x011C);
	write_cmos_sensor(0x32BE, 0x002E);
	write_cmos_sensor(0x32C2, 0x0063);
	write_cmos_sensor(0x32C6, 0x008E);
	write_cmos_sensor(0x32CA, 0x011E);
	write_cmos_sensor(0x32CE, 0x0004);
	write_cmos_sensor(0x32D2, 0x0003);
	write_cmos_sensor(0x32EE, 0x003A);
	write_cmos_sensor(0x32F2, 0x005F);
	write_cmos_sensor(0x32F6, 0x009A);
	write_cmos_sensor(0x32FA, 0x011A);
	write_cmos_sensor(0x32FE, 0x0067);
	write_cmos_sensor(0x3302, 0x007E);
	write_cmos_sensor(0x3306, 0x006E);
	write_cmos_sensor(0x330A, 0x0086);
	write_cmos_sensor(0x330E, 0x0076);
	write_cmos_sensor(0x3312, 0x0086);
	write_cmos_sensor(0x3316, 0x0067);
	write_cmos_sensor(0x331A, 0x0069);
	write_cmos_sensor(0x3326, 0x006E);
	write_cmos_sensor(0x332A, 0x0086);
	write_cmos_sensor(0x3336, 0x0067);
	write_cmos_sensor(0x333A, 0x0069);
	write_cmos_sensor(0x333E, 0x0038);
	write_cmos_sensor(0x3342, 0x0061);
	write_cmos_sensor(0x3346, 0x0098);
	write_cmos_sensor(0x334A, 0x011C);
	write_cmos_sensor(0x335E, 0x0067);
	write_cmos_sensor(0x3362, 0x0081);
	write_cmos_sensor(0x336E, 0x0067);
	write_cmos_sensor(0x3372, 0x0069);
	write_cmos_sensor(0x337E, 0x0025);
	write_cmos_sensor(0x3382, 0x0120);
	write_cmos_sensor(0x3386, 0x0026);
	write_cmos_sensor(0x338A, 0x0028);
	write_cmos_sensor(0x338E, 0x0061);
	write_cmos_sensor(0x3392, 0x0063);
	write_cmos_sensor(0x3396, 0x011C);
	write_cmos_sensor(0x339A, 0x011E);
	write_cmos_sensor(0x339E, 0x0061);
	write_cmos_sensor(0x349A, 0x0080);
	write_cmos_sensor(0x34B2, 0x00DC);
	write_cmos_sensor(0x34B6, 0x0061);
	write_cmos_sensor(0x34CA, 0x0080);
	write_cmos_sensor(0x34CE, 0x000B);
	write_cmos_sensor(0x34D6, 0x0061);
	write_cmos_sensor(0x34E2, 0x0086);
	write_cmos_sensor(0x34FA, 0x00E2);
	write_cmos_sensor(0x34FE, 0x0061);
	write_cmos_sensor(0x3512, 0x0086);
	write_cmos_sensor(0x3516, 0x000B);
	write_cmos_sensor(0x351E, 0x0061);
	write_cmos_sensor(0x3652, 0x007B);
	write_cmos_sensor(0x3654, 0x005F);
	write_cmos_sensor(0x0344, 0x0168);
	write_cmos_sensor(0x0348, 0x0B67);
	write_cmos_sensor(0x0346, 0x0200);
	write_cmos_sensor(0x034A, 0x079F);
	write_cmos_sensor(0x034C, 0x0500);
	write_cmos_sensor(0x034E, 0x02D0);
	write_cmos_sensor(0x31AC, 0x0004);
	write_cmos_sensor(0x31B0, 0x0005);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0B04, 0x0101);
	write_cmos_sensor(0x307C, 0x0340);
	write_cmos_sensor(0x030E, 0x0079);
	write_cmos_sensor(0x300A, 0x0000);
	write_cmos_sensor(0x0342, 0x1230);
	write_cmos_sensor(0x0340, 0x0A04);
	write_cmos_sensor(0x3178, 0x003D);
	write_cmos_sensor(0x0900, 0x0112);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0003);
	write_cmos_sensor(0x0400, 0x0001);
	write_cmos_sensor(0x0404, 0x0020);
	write_cmos_sensor(0x0B00, 0x0080);
	write_cmos_sensor(0x0100, 0x0100);

	write_cmos_sensor_8(0x0100, 0x01);
}



/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 1;
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			write_cmos_sensor(0x602C, 0x4000);
			write_cmos_sensor(0x602E, 0x0000);
			*sensor_id = read_cmos_sensor(0x6F12);
			/* *sensor_id = imgsensor_info.sensor_id; */
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, sensor id = 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 1;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	/* const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2}; */
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

	LOG_1;
	LOG_2;
	/* sensor have two i2c address 0x5a 0x5b & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			write_cmos_sensor(0x602C, 0x4000);
			write_cmos_sensor(0x602E, 0x0000);
			sensor_id = read_cmos_sensor(0x6F12);
			/* sensor_id = imgsensor_info.sensor_id; */
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", sensor_id);	/* imgsensor.i2c_write_id,sensor_id); */
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = KAL_FALSE;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}				/*      open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function */

	return ERROR_NONE;
}				/*      close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}				/*      preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
/*
	 else if(imgsensor.current_fps == imgsensor_info.cap2.max_framerate){
		imgsensor.pclk = imgsensor_info.cap2.pclk;
		imgsensor.line_length = imgsensor_info.cap2.linelength;
		imgsensor.frame_length = imgsensor_info.cap2.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
*/
	else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF
			    ("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			     imgsensor.current_fps, imgsensor_info.cap.max_framerate / 10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}


	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("Caputre fps:%d\n", imgsensor.current_fps);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}				/*      normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}				/*      hs_video   */


static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
* Custom1
*
* DESCRIPTION
*   This function start the sensor Custom1.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}				/*  Custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}				/*  Custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}				/*  Custom3   */

static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
	imgsensor.pclk = imgsensor_info.custom4.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.custom4.linelength;
	imgsensor.frame_length = imgsensor_info.custom4.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}				/*  Custom4   */


static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
	imgsensor.pclk = imgsensor_info.custom5.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.custom5.linelength;
	imgsensor.frame_length = imgsensor_info.custom5.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}				/*  Custom5   */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width = imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height = imgsensor_info.custom2.grabwindow_height;

	sensor_resolution->SensorCustom3Width = imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height = imgsensor_info.custom3.grabwindow_height;

	sensor_resolution->SensorCustom4Width = imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height = imgsensor_info.custom4.grabwindow_height;

	sensor_resolution->SensorCustom5Width = imgsensor_info.custom5.grabwindow_width;
	sensor_resolution->SensorCustom5Height = imgsensor_info.custom5.grabwindow_height;
	return ERROR_NONE;
}				/*      get_resolution  */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d", scenario_id);


	/* sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10;*/ /* not use */
	/* sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10;*/ /* not use */
	/* imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate;*/ /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
	sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
	sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	/* The frame of setting sensor gain */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*      get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		Custom1(image_window, sensor_config_data);	/* Custom1 */
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		Custom2(image_window, sensor_config_data);	/* Custom1 */
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		Custom3(image_window, sensor_config_data);	/* Custom1 */
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		Custom4(image_window, sensor_config_data);	/* Custom1 */
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		Custom5(image_window, sensor_config_data);	/* Custom1 */
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id,
						MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.pre.framelength) ? (frame_length -
							imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length =
		    imgsensor_info.normal_video.pclk / framerate * 10 /
		    imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.normal_video.framelength) ? (frame_length -
								 imgsensor_info.normal_video.
								 framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (framerate == 300) {
			frame_length =
			    imgsensor_info.cap.pclk / framerate * 10 /
			    imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			    (frame_length >
			     imgsensor_info.cap.framelength) ? (frame_length -
								imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
			    imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			frame_length =
			    imgsensor_info.cap1.pclk / framerate * 10 /
			    imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			    (frame_length >
			     imgsensor_info.cap1.framelength) ? (frame_length -
								 imgsensor_info.cap1.
								 framelength) : 0;
			imgsensor.frame_length =
			    imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length =
		    imgsensor_info.hs_video.pclk / framerate * 10 /
		    imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.hs_video.framelength) ? (frame_length -
							     imgsensor_info.hs_video.
							     framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length =
		    imgsensor_info.slim_video.pclk / framerate * 10 /
		    imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.slim_video.framelength) ? (frame_length -
							       imgsensor_info.slim_video.
							       framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength
								+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length =
		    imgsensor_info.custom1.pclk / framerate * 10 /
		    imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.custom1.framelength) ? (frame_length -
							    imgsensor_info.custom1.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length =
		    imgsensor_info.custom2.pclk / framerate * 10 /
		    imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.custom2.framelength) ? (frame_length -
							    imgsensor_info.custom2.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length =
		    imgsensor_info.custom3.pclk / framerate * 10 /
		    imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.custom3.framelength) ? (frame_length -
							    imgsensor_info.custom3.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length =
		    imgsensor_info.custom4.pclk / framerate * 10 /
		    imgsensor_info.custom4.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.custom4.framelength) ? (frame_length -
							    imgsensor_info.custom4.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		frame_length =
		    imgsensor_info.custom5.pclk / framerate * 10 /
		    imgsensor_info.custom5.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.custom5.framelength) ? (frame_length -
							    imgsensor_info.custom5.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	default:		/* coding with  preview scenario by default */
		frame_length =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.pre.framelength) ? (frame_length -
							imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id,
						    MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		*framerate = imgsensor_info.custom4.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		*framerate = imgsensor_info.custom5.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		/* 0x5E00[8]: 1 enable,  0 disable */
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK */
		write_cmos_sensor(0x0600, 0x0002);
	} else {
		/* 0x5E00[8]: 1 enable,  0 disable */
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK */
		write_cmos_sensor(0x0600, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	/* unsigned long long *feature_return_para=(unsigned long long *) feature_para; */

	LOG_INF("feature_id = %d", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor.pclk, imgsensor.current_fps);
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		write_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		if ((sensor_reg_data->RegData >> 8) > 0)
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		else
			write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) *feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) *feature_data,
					      *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) *feature_data,
						  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) *feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:	/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", (int)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", (BOOL) *feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (BOOL) *feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (int)*feature_data);
		wininfo = (SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16) *feature_data,
			(UINT16) *(feature_data + 1), (UINT16) *(feature_data + 2));
		ihdr_write_shutter_gain((UINT16) *feature_data, (UINT16) *(feature_data + 1),
					(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n", (UINT16) *feature_data,
			(UINT16) *(feature_data + 1));
		ihdr_write_shutter((UINT16) *feature_data, (UINT16) *(feature_data + 1));
		break;

	default:
		break;
	}
	return ERROR_NONE;
}				/*      feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5K4H9_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}				/*      s5k4h9_MIPI_RAW_SensorInit      */
