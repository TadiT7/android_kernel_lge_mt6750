/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 hi846_byd_front_wt_mipi_raw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *       info : 8M   front camera  module vendor: BYD  1st
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
#include <asm/atomic.h>
//#include <asm/system.h>
//#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi846_byd_front_wt_mipi_raw_Sensor.h"

#define PFX "HI846_front_byd_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define Hi846_MaxGain 16
//#define Hi846_I2C_BURST // If  open, you can use "i2c burst mode". 

#define Hi846_FRONT_BYD_OTP_FUNCTION    0

#if Hi846_FRONT_BYD_OTP_FUNCTION //Hi-846 OPT

//#define RG_Ratio_Typical (0x17B)
//#define BG_Ratio_Typical (0x13C)


void HI846_front_byd_Sensor_update_wb_gain(kal_uint32 r_gain, kal_uint32 g_gain,kal_uint32 b_gain);
kal_uint16 HI846_front_byd_Sensor_OTP_read(kal_uint16 otp_addr);
void HI846_front_byd_Sensor_calc_wbdata(void);
// zhaozhensen@wined-mobi.com 20161122 for hi846_otp problem begin
extern int iReadRegI2C_OTP(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
// zhaozhensen@wined-mobi.com 20161122 for hi846_otp problem end
#endif //Hi-846 OTP


static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI846_BYD_FRONT_WT_SENSOR_ID ,

	.checksum_value = 0xe32ded8,

	.pre = {
		.pclk = 288000000,				//record different mode's pclk
		.linelength = 3800,				//record different mode's linelength
		.framelength = 2492,			//record different mode's framelength
		.startx =0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1632,		//record different mode's width of grabwindow
		.grabwindow_height = 1224,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	
	.cap = {
		.pclk = 288000000,
		.linelength = 3800,	
		.framelength = 2492,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	
    .cap1 = {                            //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 288000000,
		.linelength = 3800,	
		.framelength = 5052,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 150,
    },
    
	.normal_video = {
		.pclk = 288000000,				//record different mode's pclk
		.linelength = 3800,				//record different mode's linelength
		.framelength = 2492,			//record different mode's framelength
		.startx =0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 3264,		//record different mode's width of grabwindow
		.grabwindow_height = 2448,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	
	 .hs_video = {
       .pclk = 288000000,				//record different mode's pclk
        .linelength = 3800,				//record different mode's linelength
		.framelength = 631,			//record different mode's framelength
		.startx =0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 640,		//record different mode's width of grabwindow
		.grabwindow_height = 480,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 1200,
    },
    
    .slim_video = {
       .pclk = 288000000,				//record different mode's pclk
		.linelength = 3800,
		.framelength = 842,
		.startx =0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1280,		//record different mode's width of grabwindow
		.grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 900,
    },
	.margin = 6,
	.min_shutter = 6,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 3, 
	.pre_delay_frame = 3, 
	.video_delay_frame = 3,
    .hs_video_delay_frame = 3,    //enter high speed video  delay frame num
    .slim_video_delay_frame = 3,//enter slim video delay frame num
	
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr, 	//zhaozhensen@wind-mobi.com 20161017
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x44, 0xff},//0x44,0x42,0x46,0xff
};



static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x0100,					//current shutter
	.gain = 0xe0,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE, 
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = 0, 
	.i2c_write_id = 0x44,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
 { 3264, 2448,   0,   0,   3264, 2448,   1632, 1224,   0, 0, 1632, 1224,   0, 0, 1632, 1224}, // previeow
 { 3264, 2448,   0,   0,   3264, 2448,   3264, 2448,   0, 0, 3264, 2448,   0, 0, 3264, 2448}, // capture
 { 3264, 2448,   0,   0,   3264, 2448,   3264, 2448,   0, 0, 3264, 2448,   0, 0, 3264, 2448}, // video
 { 3264, 2448,   0, 264,   3264, 1920,    816, 1920,  88, 0,  640,  480,   0, 0,  640, 480 }, //hight speed video
 { 3280, 2448,   0, 504,   3264, 1440,   1632,  720, 176, 0, 1280,  720,   0, 0, 1280, 720}};// slim video


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;


	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	kdSetI2CSpeed(400);
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8),(char)(para & 0xFF)};
    kdSetI2CSpeed(400);
	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x0046, 0x0100);
	//write_cmos_sensor(0x0006, imgsensor.frame_length );
	//write_cmos_sensor(0x0007, imgsensor.frame_length & 0xFF);	  
	//write_cmos_sensor(0x0008, imgsensor.line_length );
	//write_cmos_sensor(0x0009, imgsensor.line_length & 0xFF);
	write_cmos_sensor(0x0006, imgsensor.frame_length & 0xFFFF );
	write_cmos_sensor(0x0008, imgsensor.line_length & 0xFFFF );
	write_cmos_sensor(0x0046, 0x0000);
  
}	/*	set_dummy  */

static kal_uint16 read_cmos_sensor_byte(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    kdSetI2CSpeed(400);
    //iReadRegI2C_OTP(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);//modify of yewei 20170928
    return get_byte;
}
static void HI846_OTP_byd_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para)};
	kdSetI2CSpeed(400);
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);

}

void HI846FRONTBYDOTPSetting(void)
{
	LOG_INF("%s enter\n",__func__);
	HI846_OTP_byd_write_cmos_sensor(0x0A02, 0x01); //Fast sleep on
	HI846_OTP_byd_write_cmos_sensor(0x0A00, 0x00);//stand by on
	mdelay(10);
	HI846_OTP_byd_write_cmos_sensor(0x0f02, 0x00);//pll disable
	HI846_OTP_byd_write_cmos_sensor(0x071a, 0x01);//CP TRIM_H
	HI846_OTP_byd_write_cmos_sensor(0x071b, 0x09);//IPGM TRIM_H
	HI846_OTP_byd_write_cmos_sensor(0x0d04, 0x01);//Fsync(OTP busy)Output Enable
	HI846_OTP_byd_write_cmos_sensor(0x0d00, 0x07);//Fsync(OTP busy)Output Drivability
	HI846_OTP_byd_write_cmos_sensor(0x003e, 0x10);//OTP r/w mode
	HI846_OTP_byd_write_cmos_sensor(0x0a00, 0x01);//standby off

  LOG_INF("%s exit\n",__func__);
}

kal_uint16 HI846_front_byd_Sensor_OTP_read(kal_uint16 otp_addr)
{
    kal_uint16 i, data;
	i = otp_addr;

    HI846_OTP_byd_write_cmos_sensor(0x070a, (i >> 8) & 0xFF); //start address H
    HI846_OTP_byd_write_cmos_sensor(0x070b, i & 0xFF); //start address L
    HI846_OTP_byd_write_cmos_sensor(0x0702, 0x01); //read enable
    data = read_cmos_sensor_byte(0x0708); //OTP data read
    //data = read_cmos_sensor(0x0708); //OTP data read
    return data;
}

#if Hi846_FRONT_BYD_OTP_FUNCTION //Hi-846 OPT fuction begin

void Hi846_front_byd_Sensor_OTP_info(void)
{
	uint16_t ModuleID = 0,Year = 0,Month = 0,Day = 0,LensID = 0,VCMID = 0;
	uint16_t infocheck = 0,checksum = 0;
	int i = 0,index = 0;
	int start_address = 0;
	int info_group = -1;
	int info_flag = -1;

	LOG_INF("%s enter\n",__func__);

	for(index = 0;index < 2;index++)
	{
		start_address = index*8 + 0x0201;
		info_flag = HI846_front_byd_Sensor_OTP_read(0x0201);
		LOG_INF("%s,info_flag = 0x%x\n",__func__,info_flag);
		if(0x55 == info_flag)
		{
			for(i=0;i<7;i++)
			{
				checksum += HI846_front_byd_Sensor_OTP_read(start_address+i);
			}
			ModuleID = HI846_front_byd_Sensor_OTP_read(start_address+1);
			LensID = HI846_front_byd_Sensor_OTP_read(start_address+2);
			VCMID = HI846_front_byd_Sensor_OTP_read(start_address+3);
			Year = HI846_front_byd_Sensor_OTP_read(start_address+4);
			Month = HI846_front_byd_Sensor_OTP_read(start_address+5);
			Day = HI846_front_byd_Sensor_OTP_read(start_address+6);
			infocheck = HI846_front_byd_Sensor_OTP_read(start_address+7);
		}
		LOG_INF("HI846_Sensor:infocheck = 0x%x checksum = 0x%x \n",infocheck,checksum);
		if(infocheck == ((checksum%0xff) + 1))
		{
			LOG_INF("HI846_Sensor: Module information checksum PASS \n");
			info_group = index + 1;
			break;
		}
		else
		{
			LOG_INF("HI846_Sensor: Module information checksum Fail \n");
		}
	}

	LOG_INF("%s,info_group = %d\n",__func__,info_group);
	LOG_INF("ModuleID = 0x%x \n", ModuleID);
	LOG_INF("Year = %d, Month = %d, Day = %d\n", Year, Month, Day);
	LOG_INF("LensID = %d \n", LensID);
	LOG_INF("VCMID = %d \n", VCMID);
	LOG_INF("checksum = %d \n", checksum);
	LOG_INF("%s exit\n",__func__);
}
/*
void Hi846_Sensor_OTP_byd_update_LSC(void)
{
    uint16_t lsc_flag = 0,
       temp = 0;
  LOG_INF("%s enter\n",__func__);
  //lsc_flag = HI846_front_byd_Sensor_OTP_read(0xBCED);
	if (HI846_front_byd_Sensor_OTP_read(0x0235) == 0x01)
		{lsc_flag = 1;}
	else if (HI846_front_byd_Sensor_OTP_read(0x0235) == 0x13)
		{lsc_flag = 2;}
	else if (HI846_front_byd_Sensor_OTP_read(0x0235) == 0x37)
		{lsc_flag = 3;}
  pr_debug("%s,lsc_flag = 0x%x\n",__func__,lsc_flag);

  temp = read_cmos_sensor (0x0a05) | 0x04;
  //`temp = temp & 0xF7;
  pr_debug("%s,lsc_enable flag = 0x%x\n",__func__,temp);
  HI846_OTP_byd_write_cmos_sensor(0x0a05, temp); //LSC enable
  LOG_INF("%s exit\n",__func__);

}
*/
void HI846_front_byd_Sensor_update_wb_gain(kal_uint32 r_gain, kal_uint32 g_gain,kal_uint32 b_gain)
{
	kal_int16 temp;
    LOG_INF("%s,r_gain = 0x%x, b_gain = 0x%x\n", __func__,r_gain, b_gain);
	temp = read_cmos_sensor(0x0a05) | 0x08;
	LOG_INF("%s, = 0x%x\n", __func__,temp);
    HI846_OTP_byd_write_cmos_sensor(0x0a05, temp); //Digital Gain enable

    HI846_OTP_byd_write_cmos_sensor(0x0078, g_gain >> 8); //gr_gain
    HI846_OTP_byd_write_cmos_sensor(0x0079, g_gain & 0xFF); //gr_gain
    HI846_OTP_byd_write_cmos_sensor(0x007a, g_gain >> 8); //gb_gain
    HI846_OTP_byd_write_cmos_sensor(0x007b, g_gain & 0xFF); //gb_gain

    HI846_OTP_byd_write_cmos_sensor(0x007c, r_gain >> 8); //r_gain
    HI846_OTP_byd_write_cmos_sensor(0x007d, r_gain & 0xFF); //r_gain
    HI846_OTP_byd_write_cmos_sensor(0x007e, b_gain >> 8); //b_gain
    HI846_OTP_byd_write_cmos_sensor(0x007f, b_gain & 0xFF); //b_gain
}

void HI846_front_byd_Sensor_calc_wbdata(void)
{
	uint16_t awbcheck = 0,
	checksum = 0,
	awb_group = 0;
	int i,index,start_address;
	uint16_t r_g_low=0,
	r_g_high=0,
	b_g_low=0,
	b_g_high=0,
	r_g_golden_low=0,
	r_g_golden_high=0,
	b_g_golden_low=0,
	b_g_golden_high=0;
	uint16_t RGain=0,BGain=0,RGain_Golden=0,BGain_Golden=0;
	uint16_t R_gain= 0x0200, G_gain= 0x0200, B_gain= 0x0200;

	LOG_INF("%s enter\n",__func__);

	for(index = 0;index < 3;index++)
	{
		start_address = index*10 + 0x0211;
		if(0x55 == HI846_front_byd_Sensor_OTP_read(start_address))
		{
			for(i=0;i<9;i++)
			{
				checksum += HI846_front_byd_Sensor_OTP_read(start_address+i);
			}
			r_g_low = HI846_front_byd_Sensor_OTP_read(start_address+1);
			r_g_high = HI846_front_byd_Sensor_OTP_read(start_address+2);
			b_g_low = HI846_front_byd_Sensor_OTP_read(start_address+3);
			b_g_high = HI846_front_byd_Sensor_OTP_read(start_address+4);
			r_g_golden_low = HI846_front_byd_Sensor_OTP_read(start_address+5);
			r_g_golden_high = HI846_front_byd_Sensor_OTP_read(start_address+6);
			b_g_golden_low = HI846_front_byd_Sensor_OTP_read(start_address+7);
			b_g_golden_high = HI846_front_byd_Sensor_OTP_read(start_address+8);
			awbcheck = HI846_front_byd_Sensor_OTP_read(start_address+9);
		}
		LOG_INF("HI846_Sensor: awbcheck = 0x%x checksum = 0x%x\n",awbcheck,checksum);
		if(awbcheck == ((checksum%0xff) + 1))
		{
			LOG_INF("HI846_Sensor: awb data checksum PASS \n");
			awb_group = index + 1;
			break;
		}
		else
		{
			LOG_INF("HI846_Sensor: awb data checksum Fail \n");
		}
	}

	LOG_INF("%s,awb_group= %d\n",__func__,awb_group);
	LOG_INF("r_g_low = 0x%x\n", r_g_low);
	LOG_INF("r_g_high = 0x%x\n", r_g_high);
	LOG_INF("b_g_low = 0x%x\n", b_g_low);
	LOG_INF("b_g_high = 0x%x\n", b_g_high);
	LOG_INF("r_g_golden_low = 0x%x\n", r_g_golden_low);
	LOG_INF("r_g_golden_high = 0x%x\n", r_g_golden_high);
	LOG_INF("b_g_golden_low = 0x%x\n", b_g_golden_low);
	LOG_INF("b_g_golden_high = 0x%x\n", b_g_golden_high);


	RGain = ((r_g_high << 8) | r_g_low);
	BGain = ((b_g_high << 8) | b_g_low);
	RGain_Golden = ((r_g_golden_high << 8) | r_g_golden_low);
	BGain_Golden = ((b_g_golden_high << 8) | b_g_golden_low);

	LOG_INF("RGain = 0x%x\n", RGain);
	LOG_INF("BGain = 0x%x\n", BGain);
	LOG_INF("RGain_Golden = 0x%x\n", RGain_Golden);
	LOG_INF("BGain_Golden = 0x%x\n", BGain_Golden);

	R_gain = 0x200 * RGain_Golden/RGain;
	B_gain = 0x200 * BGain_Golden/BGain;

	if(R_gain < B_gain)
	{
		if(R_gain < 0x200)
		{
			B_gain= 0x200 * B_gain/R_gain;
			G_gain= 0x200 * G_gain/R_gain;
			R_gain= 0x200;
		}
	}
	else
	{
		if(B_gain < 0x200)
		{
			R_gain= 0x200 * R_gain/B_gain;
			G_gain= 0x200 * G_gain/B_gain;
			B_gain= 0x200;
		}
	}

	pr_debug("%s,After calcuate: R_gain = 0x%x, G_gain = 0x%x, B_gain = 0x%x\n",__func__, R_gain, G_gain, B_gain);
	LOG_INF("%s exit\n",__func__);
	HI846_OTP_byd_write_cmos_sensor(0x0a00, 0x00);
	mdelay(10);
	HI846_OTP_byd_write_cmos_sensor(0x003e, 0x00);
	HI846_OTP_byd_write_cmos_sensor(0x0a00, 0x00);
	HI846_front_byd_Sensor_update_wb_gain(R_gain, G_gain,B_gain);
}
#endif //Hi-846 OTP function finish

#if 1

#define HI846_FRONT_BYD_AWB_DATA_SIZE    11
unsigned char Hi846_front_byd_awb_data[HI846_FRONT_BYD_AWB_DATA_SIZE];

#define HI846_DONGCI_LSC_DATA_SIZE    1868
unsigned char hi846_byd_lsc_data[HI846_DONGCI_LSC_DATA_SIZE];

int Hi846_front_byd_Awb_Sensor_OTP_update(void)
{
	kal_uint16 tt = 0;
	kal_uint16 loop = 0;
	kal_uint16 checksum = 0x55;
	kal_uint16 Hi846_front_byd_awb_flag = 0;
	kal_uint16 Hi846_front_byd_awb_addr = 0;

	LOG_INF("%s enter\n",__func__);
	if (HI846_front_byd_Sensor_OTP_read(0x021D) == 0x55)
		{
			Hi846_front_byd_awb_flag = 2;
		}
	else if (HI846_front_byd_Sensor_OTP_read(0x0211) == 0x55)
		{
			Hi846_front_byd_awb_flag = 1;
		}
	LOG_INF("%s,awb_flag = 0x%x\n",__func__,Hi846_front_byd_awb_flag);
	if(Hi846_front_byd_awb_flag == 1)
		Hi846_front_byd_awb_addr = 0x0212;
	else if(Hi846_front_byd_awb_flag == 2)
		Hi846_front_byd_awb_addr = 0x021E;
	for (loop = 0; loop < HI846_FRONT_BYD_AWB_DATA_SIZE; loop++){
		tt = HI846_front_byd_Sensor_OTP_read(Hi846_front_byd_awb_addr);
		Hi846_front_byd_awb_data[loop]=tt;
		Hi846_front_byd_awb_addr++;
	}
	for(loop = 0;loop < HI846_FRONT_BYD_AWB_DATA_SIZE-1;loop++)
	{
		LOG_INF("%s: Hi846_front_byd_awb_data[%d]:0x%x\n",__func__,loop,Hi846_front_byd_awb_data[loop]);
		checksum += Hi846_front_byd_awb_data[loop];
	}
	LOG_INF("%s: Hi846_front_byd_awb_data[10] = 0x%x checksum = 0x%x,0x%x\n",__func__,Hi846_front_byd_awb_data[10],checksum,((checksum%0xff) + 1));
	if(((checksum%0xff) + 1) ==  Hi846_front_byd_awb_data[10])
	{
		LOG_INF("%s: awb data checksum PASS\n",__func__);
	}
	else
	{
		LOG_INF("%s: awb data checksum Fail\n",__func__);
		return 0;
	}
	LOG_INF("%s exit\n",__func__);
	return 1;
}
int Hi846_front_byd_lsc_Sensor_OTP_update_LSC(void)
{
	kal_uint16 tt = 0;
	kal_uint16 loop = 0;
	kal_uint32 checksum = 0x55;
	kal_uint16 checksum_value = 0;
	kal_uint16 hi846_byd_lsc_checksum_addr = 0;
	kal_uint16 hi846_byd_lsc_flag = 0;
	kal_uint16 hi846_byd_lsc_addr = 0;

	  LOG_INF("%s enter\n",__func__);

	if (HI846_front_byd_Sensor_OTP_read(0x0983) == 0x55)
	{
		hi846_byd_lsc_flag = 2;
	}
	else if (HI846_front_byd_Sensor_OTP_read(0x0235) == 0x55)
	{
		hi846_byd_lsc_flag = 1;
	}
	LOG_INF("%s,lsc_flag = 0x%x\n",__func__,hi846_byd_lsc_flag);
	if(hi846_byd_lsc_flag == 1)
	{
		hi846_byd_lsc_addr = 0x0236;
		hi846_byd_lsc_checksum_addr = 0x982;
	}
	else if(hi846_byd_lsc_flag == 2)
	{
		hi846_byd_lsc_addr = 0x0984;
		hi846_byd_lsc_checksum_addr = 0x10D0;
	}
	for (loop = 0; loop < HI846_DONGCI_LSC_DATA_SIZE; loop++){
		tt = HI846_front_byd_Sensor_OTP_read(hi846_byd_lsc_addr);
		hi846_byd_lsc_data[loop]=tt;
		hi846_byd_lsc_addr++;
	}
	for(loop = 0;loop < HI846_DONGCI_LSC_DATA_SIZE;loop++)
	{
		LOG_INF("%s,hi846_byd_lsc_data[%d] = 0x%x\n",__func__,loop,hi846_byd_lsc_data[loop]);
		checksum += hi846_byd_lsc_data[loop];
	}
	checksum_value = HI846_front_byd_Sensor_OTP_read(hi846_byd_lsc_checksum_addr);
	LOG_INF("%s: checksum = 0x%x checksum = 0x%x,0x%x\n",__func__,checksum_value,checksum,((checksum%0xff) + 1));
	if(((checksum%0xff) + 1) ==  checksum_value)
	{
		LOG_INF("%s: lsc data checksum PASS\n",__func__);
	}
	else
	{
		LOG_INF("%s: lsc data checksum Fail\n",__func__);
		return 0;
	}
	LOG_INF("%s exit\n",__func__);
	return 1;
}


uint16_t hi846_front_byd_get_module_id(void)
{
	int index = 0;
	int start_address = 0;
	uint16_t module_id = 0;
	int ret = -1;

	HI846FRONTBYDOTPSetting();

	for(index = 1;index >= 0;index--)// group 2,1
	{
		start_address = index*8 + 0x0201;
		if(0x55 == HI846_front_byd_Sensor_OTP_read(start_address))
		{
			module_id = HI846_front_byd_Sensor_OTP_read(start_address+1);
			LOG_INF("%s,index = %d, module_id = 0x%x\n",__func__,index,module_id);
			break;
		}
	}
	ret = Hi846_front_byd_Awb_Sensor_OTP_update();
	if(ret == 0)
		return ret;// checksum fail return error
	ret = Hi846_front_byd_lsc_Sensor_OTP_update_LSC();
	if(ret == 0)
		return ret;// checksum fail return error
	HI846_OTP_byd_write_cmos_sensor(0x0a00, 0x00);
    mdelay(10);
    HI846_OTP_byd_write_cmos_sensor(0x003e, 0x00);
    HI846_OTP_byd_write_cmos_sensor(0x0a00, 0x00);
	return module_id;
}
#endif

static kal_uint32 return_sensor_id(void)
{
	pr_debug("zzs 0x0f14 = %x\n", read_cmos_sensor(0x0F14));
	pr_debug("zzs 846 returnsensorid = %x\n",(read_cmos_sensor(0x0F17) << 8) | read_cmos_sensor(0x0F16));
    return ((read_cmos_sensor(0x0F17) << 8) | read_cmos_sensor(0x0F16));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	//kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

#define MAX_TIME1 864619 // 0x3800 * 0xfff9 / PCLK   us  (PCLK @ 288MHz)
#define MAX_TIME2 13835285  // 0x3800 * 0xfff9 / PCLK us  (PCLK @ 288MHz)

static unsigned int set_shutter_flag = 0;
static void set_shutter(unsigned long long shutter)
{
    unsigned long long exposure_time;
    unsigned long long frame_length_value;
    while(set_shutter_flag){
        ;
    }
    set_shutter_flag  = 1;
    LOG_INF("Enter! shutter =%lld\n",shutter);
    if(shutter >= 0xFFF9 )
    {
	//exposure_time = (shutter + 6) * imgsensor.line_length * 1000000/ imgsensor.pclk ;   // us
	exposure_time = (unsigned long long)((unsigned long) ((shutter + 6) * imgsensor.line_length) * 1000000 / imgsensor.pclk) ;   // us  modify of yewei 20170928
        LOG_INF("ciel_test_long_shutter :shutter = %lld,  exposure_time = %lld\n", shutter,exposure_time);
        if(exposure_time <= MAX_TIME1)
        {
            frame_length_value = shutter + 6;
            write_cmos_sensor (0x0046, 0x0100);
            write_cmos_sensor(0x0008, 0x0ed8);
            write_cmos_sensor(0x0006, frame_length_value);                       
            write_cmos_sensor(0x0072, (shutter & 0xFFFFF)>>16);
            write_cmos_sensor(0x0074, (shutter&0xFFFF));
            write_cmos_sensor (0x0046, 0x0000);
        } else if ((exposure_time > MAX_TIME1) && (exposure_time < MAX_TIME2)) {
            frame_length_value = 2492;
            write_cmos_sensor (0x0046, 0x0100);
            write_cmos_sensor(0x0008, 0x0ed8);
            write_cmos_sensor(0x0006, frame_length_value);                       
            write_cmos_sensor(0x0072, (shutter & 0xFFFFF)>>16);
            write_cmos_sensor(0x0074, (shutter&0xFFFF));
            write_cmos_sensor (0x0046, 0x0000);
        } else {
            frame_length_value = 2492;
            shutter = 0xFFFF9;
            write_cmos_sensor (0x0046, 0x0100);
            write_cmos_sensor(0x0008, 0x0ed8);
            write_cmos_sensor(0x0006, frame_length_value);      
            write_cmos_sensor(0x0072, (shutter & 0xFFFFF)>>16);
            write_cmos_sensor(0x0074, (shutter&0xFFFF));
            write_cmos_sensor (0x0046, 0x0000);
        }
    } else {
        kal_uint16 realtime_fps = 0;
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
        shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
        write_cmos_sensor(0x0072, 0X0000);   
        LOG_INF("ciel_test_short_shutter :normalshutter  %lld\n", shutter);
        if (imgsensor.autoflicker_en) { 
            realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
            if(realtime_fps >= 297 && realtime_fps <= 305)
                set_max_framerate(296,0);
            else if(realtime_fps >= 147 && realtime_fps <= 150)
                set_max_framerate(146,0);   
            else				
	        write_cmos_sensor(0x0046, 0x0100);
                write_cmos_sensor(0x0006, imgsensor.frame_length);
	        write_cmos_sensor(0x0046, 0x0000);
        } else {
            // Extend frame length
            write_cmos_sensor(0x0046, 0x0100);
            write_cmos_sensor(0x0006, imgsensor.frame_length);
	        write_cmos_sensor(0x0046, 0x0000);
        }
	    // Update Shutter
	    write_cmos_sensor(0x0046, 0x0100);
		//write_cmos_sensor(0x0072, ((shutter & 0xFF0000) >> 16));
            write_cmos_sensor(0x0074, shutter);	  
            write_cmos_sensor(0x0046, 0x0000);
    }
	LOG_INF("shutter =%lld, framelength =%d\n", shutter,imgsensor.frame_length);
    set_shutter_flag  = 0;
}	/*	set_shutter  */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
  kal_uint16 reg_gain = 0x0000;
	reg_gain = gain / 4 - 16;

	return (kal_uint16)reg_gain;
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
    /* [0:3] = N meams N /16 X    */
    /* [4:9] = M meams M X         */
    /* Total gain = M + N /16 X   */

    if (gain < BASEGAIN || gain > Hi846_MaxGain * BASEGAIN) {
        LOG_INF("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
        else if (gain > Hi846_MaxGain * BASEGAIN)
            gain = Hi846_MaxGain * BASEGAIN;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	write_cmos_sensor(0x0046, 0x0100);
	write_cmos_sensor(0x0076,(reg_gain & 0xFFFF));
	write_cmos_sensor(0x0046, 0x0000);

    return gain;
		
}	/*	set_gain  */
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	#if 0
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    if (imgsensor.ihdr_en) {

        spin_lock(&imgsensor_drv_lock);
        if (le > imgsensor.min_frame_length - imgsensor_info.margin)
            imgsensor.frame_length = le + imgsensor_info.margin;
        else
            imgsensor.frame_length = imgsensor.min_frame_length;
        if (imgsensor.frame_length > imgsensor_info.max_frame_length)
            imgsensor.frame_length = imgsensor_info.max_frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
        if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;


        // Extend frame length first
        write_cmos_sensor(0x0006, imgsensor.frame_length);

        write_cmos_sensor(0x3502, (le << 4) & 0xFF);
        write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
        write_cmos_sensor(0x3500, (le >> 12) & 0x0F);

        write_cmos_sensor(0x3508, (se << 4) & 0xFF);
        write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
        write_cmos_sensor(0x3506, (se >> 12) & 0x0F);

        set_gain(gain);
    }
#endif
}



#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
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
	
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x000e,0x0000);
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x000e,0x0100);
			
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x000e,0x0200);
			
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x000e,0x0300);

			break;
		default:
			LOG_INF("Error image_mirror setting");
	}

}
#endif
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
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/

static void sensor_init(void)
{
LOG_INF("Enter init\n");
write_cmos_sensor(0x0a00, 0x0000); //sleep on
write_cmos_sensor(0x2000, 0x100A); 
write_cmos_sensor(0x2002, 0x00FF); 
write_cmos_sensor(0x2004, 0x0007); 
write_cmos_sensor(0x2006, 0x3FFF); 
write_cmos_sensor(0x2008, 0x3FFF); 
write_cmos_sensor(0x200A, 0xC216); 
write_cmos_sensor(0x200C, 0x1292); 
write_cmos_sensor(0x200E, 0xC01A); 
write_cmos_sensor(0x2010, 0x403D); 
write_cmos_sensor(0x2012, 0x000E); 
write_cmos_sensor(0x2014, 0x403E); 
write_cmos_sensor(0x2016, 0x0B80); 
write_cmos_sensor(0x2018, 0x403F); 
write_cmos_sensor(0x201A, 0x82AE); 
write_cmos_sensor(0x201C, 0x1292); 
write_cmos_sensor(0x201E, 0xC00C); 
write_cmos_sensor(0x2020, 0x4130); 
write_cmos_sensor(0x2022, 0x43E2); 
write_cmos_sensor(0x2024, 0x0180); 
write_cmos_sensor(0x2026, 0x4130); 
write_cmos_sensor(0x2028, 0x7400); 
write_cmos_sensor(0x202A, 0x5000); 
write_cmos_sensor(0x202C, 0x0253); 
write_cmos_sensor(0x202E, 0x0AD1); 
write_cmos_sensor(0x2030, 0x2360); 
write_cmos_sensor(0x2032, 0x0009); 
write_cmos_sensor(0x2034, 0x5020); 
write_cmos_sensor(0x2036, 0x000B); 
write_cmos_sensor(0x2038, 0x0002); 
write_cmos_sensor(0x203A, 0x0044); 
write_cmos_sensor(0x203C, 0x0016); 
write_cmos_sensor(0x203E, 0x1792); 
write_cmos_sensor(0x2040, 0x7002); 
write_cmos_sensor(0x2042, 0x154F); 
write_cmos_sensor(0x2044, 0x00D5); 
write_cmos_sensor(0x2046, 0x000B); 
write_cmos_sensor(0x2048, 0x0019); 
write_cmos_sensor(0x204A, 0x1698); 
write_cmos_sensor(0x204C, 0x000E); 
write_cmos_sensor(0x204E, 0x099A); 
write_cmos_sensor(0x2050, 0x0058); 
write_cmos_sensor(0x2052, 0x7000); 
write_cmos_sensor(0x2054, 0x1799); 
write_cmos_sensor(0x2056, 0x0310); 
write_cmos_sensor(0x2058, 0x03C3); 
write_cmos_sensor(0x205A, 0x004C); 
write_cmos_sensor(0x205C, 0x064A); 
write_cmos_sensor(0x205E, 0x0001); 
write_cmos_sensor(0x2060, 0x0007); 
write_cmos_sensor(0x2062, 0x0BC7); 
write_cmos_sensor(0x2064, 0x0055); 
write_cmos_sensor(0x2066, 0x7000); 
write_cmos_sensor(0x2068, 0x1550); 
write_cmos_sensor(0x206A, 0x158A); 
write_cmos_sensor(0x206C, 0x0004); 
write_cmos_sensor(0x206E, 0x1488); 
write_cmos_sensor(0x2070, 0x7010); 
write_cmos_sensor(0x2072, 0x1508); 
write_cmos_sensor(0x2074, 0x0004); 
write_cmos_sensor(0x2076, 0x0016); 
write_cmos_sensor(0x2078, 0x03D5); 
write_cmos_sensor(0x207A, 0x0055); 
write_cmos_sensor(0x207C, 0x08CA); 
write_cmos_sensor(0x207E, 0x2019); 
write_cmos_sensor(0x2080, 0x0007); 
write_cmos_sensor(0x2082, 0x7057); 
write_cmos_sensor(0x2084, 0x0FC7); 
write_cmos_sensor(0x2086, 0x5041); 
write_cmos_sensor(0x2088, 0x12C8); 
write_cmos_sensor(0x208A, 0x5060); 
write_cmos_sensor(0x208C, 0x5080); 
write_cmos_sensor(0x208E, 0x2084); 
write_cmos_sensor(0x2090, 0x12C8); 
write_cmos_sensor(0x2092, 0x7800); 
write_cmos_sensor(0x2094, 0x0802); 
write_cmos_sensor(0x2096, 0x040F); 
write_cmos_sensor(0x2098, 0x1007); 
write_cmos_sensor(0x209A, 0x0803); 
write_cmos_sensor(0x209C, 0x080B); 
write_cmos_sensor(0x209E, 0x3803); 
write_cmos_sensor(0x20A0, 0x0807); 
write_cmos_sensor(0x20A2, 0x0404); 
write_cmos_sensor(0x20A4, 0x0400); 
write_cmos_sensor(0x20A6, 0xFFFF); 
write_cmos_sensor(0x20A8, 0xF0B2); 
write_cmos_sensor(0x20AA, 0xFFEF); 
write_cmos_sensor(0x20AC, 0x0A84); 
write_cmos_sensor(0x20AE, 0x1292); 
write_cmos_sensor(0x20B0, 0xC02E); 
write_cmos_sensor(0x20B2, 0x4130); 
write_cmos_sensor(0x23FE, 0xC056); 
write_cmos_sensor(0x3232, 0xFC0C); 
write_cmos_sensor(0x3236, 0xFC22); 
write_cmos_sensor(0x3248, 0xFCA8); 
write_cmos_sensor(0x326A, 0x8302); 
write_cmos_sensor(0x326C, 0x830A); 
write_cmos_sensor(0x326E, 0x0000); 
write_cmos_sensor(0x32CA, 0xFC28); 
write_cmos_sensor(0x32CC, 0xC3BC); 
write_cmos_sensor(0x32CE, 0xC34C); 
write_cmos_sensor(0x32D0, 0xC35A); 
write_cmos_sensor(0x32D2, 0xC368); 
write_cmos_sensor(0x32D4, 0xC376); 
write_cmos_sensor(0x32D6, 0xC3C2); 
write_cmos_sensor(0x32D8, 0xC3E6); 
write_cmos_sensor(0x32DA, 0x0003); 
write_cmos_sensor(0x32DC, 0x0003); 
write_cmos_sensor(0x32DE, 0x00C7); 
write_cmos_sensor(0x32E0, 0x0031); 
write_cmos_sensor(0x32E2, 0x0031); 
write_cmos_sensor(0x32E4, 0x0031); 
write_cmos_sensor(0x32E6, 0xFC28); 
write_cmos_sensor(0x32E8, 0xC3BC); 
write_cmos_sensor(0x32EA, 0xC384); 
write_cmos_sensor(0x32EC, 0xC392); 
write_cmos_sensor(0x32EE, 0xC3A0); 
write_cmos_sensor(0x32F0, 0xC3AE); 
write_cmos_sensor(0x32F2, 0xC3C4); 
write_cmos_sensor(0x32F4, 0xC3E6); 
write_cmos_sensor(0x32F6, 0x0003); 
write_cmos_sensor(0x32F8, 0x0003); 
write_cmos_sensor(0x32FA, 0x00C7); 
write_cmos_sensor(0x32FC, 0x0031); 
write_cmos_sensor(0x32FE, 0x0031); 
write_cmos_sensor(0x3300, 0x0031); 
write_cmos_sensor(0x3302, 0x82CA); 
write_cmos_sensor(0x3304, 0xC164); 
write_cmos_sensor(0x3306, 0x82E6); 
write_cmos_sensor(0x3308, 0xC19C); 
write_cmos_sensor(0x330A, 0x001F); 
write_cmos_sensor(0x330C, 0x001A); 
write_cmos_sensor(0x330E, 0x0034); 
write_cmos_sensor(0x3310, 0x0000); 
write_cmos_sensor(0x3312, 0x0000); 
write_cmos_sensor(0x3314, 0xFC94); 
write_cmos_sensor(0x3316, 0xC3D8);   
write_cmos_sensor(0x0A00, 0x0000); 
write_cmos_sensor(0x0E04, 0x0012); 
write_cmos_sensor(0x002E, 0x1111); 
write_cmos_sensor(0x0032, 0x1111); 
write_cmos_sensor(0x0022, 0x0008); 
write_cmos_sensor(0x0026, 0x0040); 
write_cmos_sensor(0x0028, 0x0017); 
write_cmos_sensor(0x002C, 0x09CF); 
write_cmos_sensor(0x005C, 0x2101); 
write_cmos_sensor(0x0006, 0x09BC); 
write_cmos_sensor(0x0008, 0x0ED8); 
write_cmos_sensor(0x000E, 0x0200); //image_orient
write_cmos_sensor(0x000C, 0x0022); 
write_cmos_sensor(0x0A22, 0x0000); 
write_cmos_sensor(0x0A24, 0x0000); 
write_cmos_sensor(0x0804, 0x0000); 
write_cmos_sensor(0x0A12, 0x0CC0); 
write_cmos_sensor(0x0A14, 0x0990); 
write_cmos_sensor(0x0074, 0x09B6); 
write_cmos_sensor(0x0076, 0x0000); 
//write_cmos_sensor(0x0046, 0x0000); //group para off/per frame control off
write_cmos_sensor(0x051E, 0x0000); 
write_cmos_sensor(0x0200, 0x0400); 
write_cmos_sensor(0x0A1A, 0x0C00); 
write_cmos_sensor(0x0A0C, 0x0010); 
write_cmos_sensor(0x0A1E, 0x0CCF); 
write_cmos_sensor(0x0402, 0x0110); 
write_cmos_sensor(0x0404, 0x00F4); 
write_cmos_sensor(0x0408, 0x0000); 
write_cmos_sensor(0x0410, 0x008D); 
write_cmos_sensor(0x0412, 0x011A); 
write_cmos_sensor(0x0414, 0x864C); 
write_cmos_sensor(0x021C, 0x0001);
write_cmos_sensor(0x021e, 0x0235); 
write_cmos_sensor(0x0C00, 0x9150); 
write_cmos_sensor(0x0C06, 0x0021); 
write_cmos_sensor(0x0C10, 0x0040); 
write_cmos_sensor(0x0C12, 0x0040); 
write_cmos_sensor(0x0C14, 0x0040); 
write_cmos_sensor(0x0C16, 0x0040); 
write_cmos_sensor(0x0A02, 0x0100); 
write_cmos_sensor(0x0A04, 0x014A);
write_cmos_sensor(0x0418, 0x0000); 
write_cmos_sensor(0x012A, 0x03B4); 
write_cmos_sensor(0x0120, 0x0046); 
write_cmos_sensor(0x0122, 0x0376); 
write_cmos_sensor(0x0B02, 0xE04D); 
write_cmos_sensor(0x0B10, 0x6821); 
write_cmos_sensor(0x0B12, 0x0020); 
write_cmos_sensor(0x0B14, 0x0001); 
write_cmos_sensor(0x2008, 0x38FD); 
write_cmos_sensor(0x326E, 0x0000); 
write_cmos_sensor(0x0900, 0x0300); 
write_cmos_sensor(0x0902, 0xC319); 
write_cmos_sensor(0x0914, 0xC109); 
write_cmos_sensor(0x0916, 0x061A); 
write_cmos_sensor(0x0918, 0x0407); 
write_cmos_sensor(0x091A, 0x0A0B); 
write_cmos_sensor(0x091C, 0x0E08); 
write_cmos_sensor(0x091E, 0x0A00); 
write_cmos_sensor(0x090C, 0x0427); 
write_cmos_sensor(0x090E, 0x0069); 
write_cmos_sensor(0x0954, 0x0089); 
write_cmos_sensor(0x0956, 0x0000); 
write_cmos_sensor(0x0958, 0xa880); 
write_cmos_sensor(0x095A, 0x9240); 
write_cmos_sensor(0x0F08, 0x2F04); 
write_cmos_sensor(0x0F30, 0x001F); 
write_cmos_sensor(0x0F36, 0x001F); 
write_cmos_sensor(0x0F04, 0x3A00); 
write_cmos_sensor(0x0F32, 0x025A); 
write_cmos_sensor(0x0F38, 0x025A); 
write_cmos_sensor(0x0F2A, 0x4124); 
write_cmos_sensor(0x006A, 0x0100);
write_cmos_sensor(0x004C, 0x0100); 
//write_cmos_sensor(0x0A00, 0x0100); 

}
                 
static void preview_setting(void)
{

LOG_INF("E preview\n"); 

write_cmos_sensor(0x0A00, 0x0000);
write_cmos_sensor(0x021C, 0x0001); 
write_cmos_sensor(0x021e, 0x0235); 
write_cmos_sensor(0x002E, 0x3311);
write_cmos_sensor(0x0032, 0x3311);
write_cmos_sensor(0x0026, 0x0040);
write_cmos_sensor(0x002C, 0x09CF);
write_cmos_sensor(0x005C, 0x4202);
write_cmos_sensor(0x0006, 0x09BC);
write_cmos_sensor(0x0008, 0x0ED8);
write_cmos_sensor(0x000C, 0x0122);
write_cmos_sensor(0x0A22, 0x0100);
write_cmos_sensor(0x0A24, 0x0000);
write_cmos_sensor(0x0804, 0x0000);
write_cmos_sensor(0x0A12, 0x0660);
write_cmos_sensor(0x0A14, 0x04C8);
write_cmos_sensor(0x0074, 0x09B6);
write_cmos_sensor(0x0A04, 0x016A); 
write_cmos_sensor(0x0418, 0x0000);
write_cmos_sensor(0x0B02, 0xE04D);
write_cmos_sensor(0x0B10, 0x6C21);
write_cmos_sensor(0x0B12, 0x0020);
write_cmos_sensor(0x0B14, 0x0005);
write_cmos_sensor(0x2008, 0x38FD);//V15
write_cmos_sensor(0x326E, 0x0000); 
//=============================================//
//      MIPI 4lane 360Mbps
//=============================================//
write_cmos_sensor(0x0900, 0x0300); 
write_cmos_sensor(0x0902, 0xC319); 
write_cmos_sensor(0x0914, 0xC105); 
write_cmos_sensor(0x0916, 0x030C); 
write_cmos_sensor(0x0918, 0x0304); 
write_cmos_sensor(0x091A, 0x0708); 
write_cmos_sensor(0x091C, 0x0B04); 
write_cmos_sensor(0x091E, 0x0500); 
write_cmos_sensor(0x090C, 0x0208); 
write_cmos_sensor(0x090E, 0x002C); 
write_cmos_sensor(0x0954, 0x0089); 
write_cmos_sensor(0x0956, 0x0000); 
write_cmos_sensor(0x0958, 0xa880); 
write_cmos_sensor(0x095A, 0x9240); 
write_cmos_sensor(0x0F2A, 0x4924); 
write_cmos_sensor(0x004C, 0x0100); 
write_cmos_sensor(0x0A00, 0x0100); 

}


static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E capture, currefps = %d\n",currefps);
	if( currefps == 300 )
	{
		//Sensor Information////////////////////////////
		//Sensor	  : Hi-846
		//Date		  : 2017-01-25
		//Customer        : MTK_validation
		//Image size	  : 3264x2448
		//MCLK/PCLK	  : 24MHz /288Mhz
		//MIPI speed(Mbps): 720Mbps x 4Lane
		//Frame Length	  : 2492
		//Line Length 	  : 3800
		//Max Fps 	  : 30.41fps
		//Pixel order 	  : Green 1st (=GB)
		//X/Y-flip        : X-flip
		//BLC offset	    : 64code
		//Firmware Ver.   : v15
		////////////////////////////////////////////////
		write_cmos_sensor(0x0A00, 0x0000);
		write_cmos_sensor(0x021C, 0x0001); 
		write_cmos_sensor(0x021e, 0x0235); 
		write_cmos_sensor(0x002E, 0x1111);
		write_cmos_sensor(0x0032, 0x1111);
		write_cmos_sensor(0x0026, 0x0040);
		write_cmos_sensor(0x002C, 0x09CF);
		write_cmos_sensor(0x005C, 0x2101);
		write_cmos_sensor(0x0006, 0x09BC);
		write_cmos_sensor(0x0008, 0x0ED8);
		write_cmos_sensor(0x000C, 0x0022);
		write_cmos_sensor(0x0A22, 0x0000);
		write_cmos_sensor(0x0A24, 0x0000);
		write_cmos_sensor(0x0804, 0x0000);
		write_cmos_sensor(0x0A12, 0x0CC0);
		write_cmos_sensor(0x0A14, 0x0990);
		write_cmos_sensor(0x0074, 0x09B6);
		write_cmos_sensor(0x0A04, 0x014A);
		write_cmos_sensor(0x0418, 0x0000);
		write_cmos_sensor(0x0B02, 0xE04D);
		write_cmos_sensor(0x0B10, 0x6821);
		write_cmos_sensor(0x0B12, 0x0020);
		write_cmos_sensor(0x0B14, 0x0001);
		write_cmos_sensor(0x2008, 0x38FD);
		write_cmos_sensor(0x326E, 0x0000);
		//=============================================//
		//      MIPI 4lane 720Mbps
		//=============================================//
		write_cmos_sensor(0x0900, 0x0300);
		write_cmos_sensor(0x0902, 0xC319);
		write_cmos_sensor(0x0914, 0xC109);
		write_cmos_sensor(0x0916, 0x061A);
		write_cmos_sensor(0x0918, 0x0407);
		write_cmos_sensor(0x091A, 0x0A0B);
		write_cmos_sensor(0x091C, 0x0E08);
		write_cmos_sensor(0x091E, 0x0A00);
		write_cmos_sensor(0x090C, 0x0427);
		write_cmos_sensor(0x090E, 0x0069);
		write_cmos_sensor(0x0954, 0x0089);
		write_cmos_sensor(0x0956, 0x0000);
		write_cmos_sensor(0x0958, 0xa880);
		write_cmos_sensor(0x095A, 0x9240);
		write_cmos_sensor(0x0F2A, 0x4124);
		write_cmos_sensor(0x004C, 0x0100);
		write_cmos_sensor(0x0A00, 0x0100);
	}
	else	// PIP
	{
		//Sensor Information////////////////////////////
		//Sensor	  : Hi-846
		//Date		  : 2017-01-25
		//Customer        : MTK_validation
		//Image size	  : 3264x2448
		//MCLK/PCLK	  : 24MHz /288Mhz
		//MIPI speed(Mbps): 720Mbps x 4Lane
		//Frame Length	  : 5052
		//Line Length 	  : 3800
		//Max Fps 	  : 15.0fps
		//Pixel order 	  : Green 1st (=GB)
		//X/Y-flip        : X-flip
		//BLC offset	    : 64code
		//Firmware Ver.   : v15
		////////////////////////////////////////////////
		write_cmos_sensor(0x0A00, 0x0000);
		write_cmos_sensor(0x021C, 0x0001); 
		write_cmos_sensor(0x021e, 0x0235); 
		write_cmos_sensor(0x002E, 0x1111);
		write_cmos_sensor(0x0032, 0x1111);
		write_cmos_sensor(0x0026, 0x0040);
		write_cmos_sensor(0x002C, 0x09CF);
		write_cmos_sensor(0x005C, 0x2101);
		write_cmos_sensor(0x0006, 0x13BC);
		write_cmos_sensor(0x0008, 0x0ED8);
		write_cmos_sensor(0x000C, 0x0022);
		write_cmos_sensor(0x0A22, 0x0000);
		write_cmos_sensor(0x0A24, 0x0000);
		write_cmos_sensor(0x0804, 0x0000);
		write_cmos_sensor(0x0A12, 0x0CC0);
		write_cmos_sensor(0x0A14, 0x0990);
		write_cmos_sensor(0x0074, 0x13B6);
		write_cmos_sensor(0x0A04, 0x014A);
		write_cmos_sensor(0x0418, 0x0000);
		write_cmos_sensor(0x0B02, 0xE04D);
		write_cmos_sensor(0x0B10, 0x6821);
		write_cmos_sensor(0x0B12, 0x0020);
		write_cmos_sensor(0x0B14, 0x0001);
		write_cmos_sensor(0x2008, 0x38FD);
		write_cmos_sensor(0x326E, 0x0000);
		//=============================================//
		//      MIPI 4lane 720Mbps
		//=============================================//
		write_cmos_sensor(0x0900, 0x0300);
		write_cmos_sensor(0x0902, 0xC319);
		write_cmos_sensor(0x0914, 0xC109);
		write_cmos_sensor(0x0916, 0x061A);
		write_cmos_sensor(0x0918, 0x0407);
		write_cmos_sensor(0x091A, 0x0A0B);
		write_cmos_sensor(0x091C, 0x0E08);
		write_cmos_sensor(0x091E, 0x0A00);
		write_cmos_sensor(0x090C, 0x0427);
		write_cmos_sensor(0x090E, 0x0069);
		write_cmos_sensor(0x0954, 0x0089);
		write_cmos_sensor(0x0956, 0x0000);
		write_cmos_sensor(0x0958, 0xa880);
		write_cmos_sensor(0x095A, 0x9240);
		write_cmos_sensor(0x0F2A, 0x4124);
		write_cmos_sensor(0x004C, 0x0100);
		write_cmos_sensor(0x0A00, 0x0100);
	}
}
static void hs_video_setting(void)
{

LOG_INF("E hs_video_setting\n");


write_cmos_sensor(0x0A00, 0x0000);
write_cmos_sensor(0x021C, 0x0001); 
write_cmos_sensor(0x021e, 0x0235); 
write_cmos_sensor(0x002E, 0x7711);
write_cmos_sensor(0x0032, 0x7711);
write_cmos_sensor(0x0026, 0x0148);
write_cmos_sensor(0x002C, 0x08C7);
write_cmos_sensor(0x005C, 0x4404);
write_cmos_sensor(0x0006, 0x0277);
write_cmos_sensor(0x0008, 0x0ED8);
write_cmos_sensor(0x000C, 0x0322);
write_cmos_sensor(0x0A22, 0x0200);
write_cmos_sensor(0x0A24, 0x0000);
write_cmos_sensor(0x0804, 0x0058);
write_cmos_sensor(0x0A12, 0x0280);
write_cmos_sensor(0x0A14, 0x01E0);
write_cmos_sensor(0x0074, 0x0271);
write_cmos_sensor(0x0A04, 0x016A); 
write_cmos_sensor(0x0418, 0x0210);
write_cmos_sensor(0x0B02, 0xE04D);
write_cmos_sensor(0x0B10, 0x7021);
write_cmos_sensor(0x0B12, 0x0020);
write_cmos_sensor(0x0B14, 0x0001);
write_cmos_sensor(0x2008, 0x38FD);//V15
write_cmos_sensor(0x326E, 0x0000); 
//=============================================//
//      MIPI 4lane 180Mbps
//=============================================//
write_cmos_sensor(0x0900, 0x0300); 
write_cmos_sensor(0x0902, 0xC319); 
write_cmos_sensor(0x0914, 0xC103); 
write_cmos_sensor(0x0916, 0x0207); 
write_cmos_sensor(0x0918, 0x0202); 
write_cmos_sensor(0x091A, 0x0305); 
write_cmos_sensor(0x091C, 0x0902); 
write_cmos_sensor(0x091E, 0x0300); 
write_cmos_sensor(0x090C, 0x00FB); 
write_cmos_sensor(0x090E, 0x0046); 
write_cmos_sensor(0x0954, 0x0089); 
write_cmos_sensor(0x0956, 0x0000); 
write_cmos_sensor(0x0958, 0xa880); 
write_cmos_sensor(0x095A, 0x9240); 
write_cmos_sensor(0x0F2A, 0x5124); 
write_cmos_sensor(0x004C, 0x0100); 
write_cmos_sensor(0x0A00, 0x0100); 
};

static void slim_video_setting(void)
{
LOG_INF("E slim_video_setting\n");

write_cmos_sensor(0x0A00, 0x0000);
write_cmos_sensor(0x021C, 0x0001); 
write_cmos_sensor(0x021e, 0x0235);
write_cmos_sensor(0x002E, 0x3311);
write_cmos_sensor(0x0032, 0x3311);
write_cmos_sensor(0x0026, 0x0238);
write_cmos_sensor(0x002C, 0x07D7);
write_cmos_sensor(0x005C, 0x4202);
write_cmos_sensor(0x0006, 0x034A);
write_cmos_sensor(0x0008, 0x0ED8);
write_cmos_sensor(0x000C, 0x0122);
write_cmos_sensor(0x0A22, 0x0100);
write_cmos_sensor(0x0A24, 0x0000);
write_cmos_sensor(0x0804, 0x00B0);
write_cmos_sensor(0x0A12, 0x0500);
write_cmos_sensor(0x0A14, 0x02D0);
write_cmos_sensor(0x0074, 0x0344);
write_cmos_sensor(0x0A04, 0x016A);
write_cmos_sensor(0x0418, 0x0410);
write_cmos_sensor(0x0B02, 0xE04D);
write_cmos_sensor(0x0B10, 0x6C21);
write_cmos_sensor(0x0B12, 0x0020);
write_cmos_sensor(0x0B14, 0x0005);
write_cmos_sensor(0x2008, 0x38FD);//V15
write_cmos_sensor(0x326E, 0x0000); 
//=============================================//
//      MIPI 4lane 360Mbps
//=============================================//
write_cmos_sensor(0x0900, 0x0300); 
write_cmos_sensor(0x0902, 0xC319); 
write_cmos_sensor(0x0914, 0xC105); 
write_cmos_sensor(0x0916, 0x030C); 
write_cmos_sensor(0x0918, 0x0304); 
write_cmos_sensor(0x091A, 0x0708); 
write_cmos_sensor(0x091C, 0x0B04); 
write_cmos_sensor(0x091E, 0x0500); 
write_cmos_sensor(0x090C, 0x0208); 
write_cmos_sensor(0x090E, 0x009A); 
write_cmos_sensor(0x0954, 0x0089); 
write_cmos_sensor(0x0956, 0x0000); 
write_cmos_sensor(0x0958, 0xa880); 
write_cmos_sensor(0x095A, 0x9240); 
write_cmos_sensor(0x0F2A, 0x4924); 
write_cmos_sensor(0x004C, 0x0100); 
write_cmos_sensor(0x0A00, 0x0100); 
};

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
	kal_uint8 retry = 2;
	uint16_t hi846_byd_module_id = 0;
    LOG_INF("[get_imgsensor_id] ");

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
			spin_lock(&imgsensor_drv_lock);
			imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
			spin_unlock(&imgsensor_drv_lock);
			do {
				*sensor_id =return_sensor_id()+1;
				if (*sensor_id == imgsensor_info.sensor_id) {
					printk("i2c write id  : 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
#if 1
				sensor_init();
				hi846_byd_module_id = hi846_front_byd_get_module_id();
				printk("hi846_byd_module_id: 0x%x\n", hi846_byd_module_id);
				if(0x10 != hi846_byd_module_id)
				{
					// if module ID is not correct, Must set *sensor_id to 0xFFFFFFFF for exit
					//*sensor_id = 0xFFFFFFFF;
					//return ERROR_SENSOR_CONNECT_FAIL;
					printk("[HI846_BYD] vendor id is different!! you should check the EEPROM or Module hardware\n");
				}
#endif
					return ERROR_NONE;
				}
				printk("get_imgsensor_id Read sensor id fail, i2c write id: 0x%x,sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				retry--;
			} while(retry > 0);
			i++;
			retry = 2;
}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
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
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0; 
	LOG_INF("[open]: PLATFORM:MT6735,MIPI 4LANE\n");
	LOG_INF("preview 1632*1224@30fps,360Mbps/lane; capture 3264*2448@30fps,720Mbps/lane\n");

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {

		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		LOG_INF("SP\n");
		do {
			sensor_id = return_sensor_id()+1;

			if (sensor_id == imgsensor_info.sensor_id) {
				printk("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			printk("open:Read sensor id fail open i2c write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

    sensor_init();
    /* initail sequence write in  */

#if Hi846_FRONT_BYD_OTP_FUNCTION
    HI846FRONTBYDOTPSetting();
    Hi846_front_byd_Sensor_OTP_info();
    HI846_front_byd_Sensor_calc_wbdata();
#endif

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
  
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}	/*	open  */



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
	LOG_INF("close E");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


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
	LOG_INF("E");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	//set_mirror_flip(imgsensor.mirror); //zhaozhensen@wind-mobi.com 20161017
	preview_setting();
	return ERROR_NONE;
}	/*	preview   */

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
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

    if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) // 30fps
    {
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	else //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
    {
//		if (imgsensor.current_fps != imgsensor_info.cap1.max_framerate)
//			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor_info.cap1.max_framerate/10);   
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} 

	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("Caputre fps:%d\n",imgsensor.current_fps);
	//set_mirror_flip(imgsensor.mirror); //zhaozhensen@wind-mobi.com 20161017
	capture_setting(imgsensor.current_fps); 
	return ERROR_NONE;
    
}	/* capture() */
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
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	//set_mirror_flip(imgsensor.mirror); //zhaozhensen@wind-mobi.com 20161017
	capture_setting(imgsensor.current_fps);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
	//set_mirror_flip(imgsensor.mirror); //zhaozhensen@wind-mobi.com 20161017
    hs_video_setting();	
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

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
	//set_mirror_flip(imgsensor.mirror); //zhaozhensen@wind-mobi.com 20161017
    slim_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);

    return ERROR_NONE;
}    /*    slim_video     */






static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;
    return ERROR_NONE;
}    /*    get_resolution    */


static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);


    //sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
    //sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}    /*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("[contrlo]scenario_id = %d", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		
			LOG_INF("preview\n");
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
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
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		//imgsensor.current_fps = 10 * framerate;
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);
	set_dummy();
	return ERROR_NONE;
}


static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) 	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
         //   set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
          //  set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
          //  set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
          //  set_dummy();
            break;
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
          //  set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
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
        default:
            break;
    }

    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d", enable);

	if (enable) {
		  LOG_INF("enter color bar");
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0A00, 0x0000);
		write_cmos_sensor(0x0a04, 0x0141);
		write_cmos_sensor(0x020a, 0x0200);
		write_cmos_sensor(0x0A00, 0x0100);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0A00, 0x0000);
		write_cmos_sensor(0x0a04, 0x0140);
		write_cmos_sensor(0x020a, 0x0000);
		write_cmos_sensor(0x0A00, 0x0100);
	}	 
	spin_lock(&imgsensor_drv_lock);
  imgsensor.test_pattern = enable;
  spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}



static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    //unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;

        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
		//[LGE_UPDATE_S][bk.bae@lge.com][2017-05-18] update FOV value
		case SENSOR_FEATURE_GET_SENSOR_VIEWANGLE:
			{
				uintptr_t *pHorFOV = (uintptr_t *)(uintptr_t)(*(feature_data));
				uintptr_t *pVerFOV = (uintptr_t *)(uintptr_t)(*(feature_data+1));

				LOG_INF("SENSOR_FEATURE_GET_SENSOR_VIEWANGLE:\n");
				*pHorFOV = (0x4A)<<16 | (0x4A); // 16:9 = 74, 4:3 = 74 // (74)<<16 | (74)   //74 degree
				*pVerFOV = (0x31)<<16 | (0x31); // 16:9 = 49, 4:3 = 49 // (49)<<16 | (49)
				*feature_para_len = sizeof(uintptr_t) * 2;
			}
			break;
		//[LGE_UPDATE_E][bk.bae@lge.com][2017-05-18] update FOV value
        default:
            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 HI846_BYD_FRONT_WT_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	HI846_MIPI_RAW_SensorInit	*/
