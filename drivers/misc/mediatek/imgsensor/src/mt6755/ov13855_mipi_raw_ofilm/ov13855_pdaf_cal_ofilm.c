#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>

#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov13855mipiraw_Sensor_ofilm.h"
#define PFX "OV13855_eeprom"
#define LOG_INF(format, args...)	pr_err(PFX "[%s] " format, __FUNCTION__, ##args)

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);

#define USHORT             unsigned short
//#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define OV13855_EEPROM_READ_ID  0xA1
#define OV13855_EEPROM_WRITE_ID   0xA0//eeprom id 0xA0

#define OV13855_I2C_SPEED        100
#define OV13855_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048
unsigned char OV13855_eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;

static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > OV13855_MAX_OFFSET)
        return false;
	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, OV13855_EEPROM_WRITE_ID)<0)
		return false;
    return true;
}

static bool _read_ov13855_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0;
	int offset = addr;
	for(i = 0; i < size; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			return false;
		}
		
//		LOG_INF("read_eeprom proc[%d] %d\n",offset, data[i]);
//		if (i >= 1360)
//		LOG_INF("read_eeprom proc[%d] %d\n",offset, data[i]);

		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

bool read_ov13855_eeprom( kal_uint16 addr, unsigned char* data, kal_uint32 size){
	unsigned char checksum[2] = {0};
	kal_uint16 checksum_h_addr = 0x1b87;
	kal_uint32 checksum_size = 2;
	kal_uint32 checksum_reg_value = 0;
	kal_uint32 checksum_value = 0;
	int i = 0;
	addr = 0x162B;//0x801;//from the first valid data on
	size = 1372;//0x57c//the total valid data size

	LOG_INF("read_ov13855_eeprom, size = %d\n", size);

	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_ov13855_eeprom(addr, OV13855_eeprom_data, size)){
			get_done = 0;
			last_size = 0;
			last_offset = 0;
			return false;
		}
		if(!_read_ov13855_eeprom(checksum_h_addr, checksum, checksum_size)){
			get_done = 0;
			last_size = 0;
			last_offset = 0;
			return false;
		}
		checksum_reg_value = (checksum[0]&0xff) << 8 | (checksum[1]&0xff);
		for(i = 0; i < size; i++)
		{
			checksum_value += OV13855_eeprom_data[i];
		}
		printk("checksum_reg_value = %d checksum_value=%d %d \n",checksum_reg_value, checksum_value,((checksum_value % 255) + 1));
		if(checksum_reg_value == ((checksum_value % 255) + 1))
		{
			printk("PDAF cali checksum ok\n");
		}
		else
		{
			printk("PDAF cali checksum fail\n");
			get_done = 0;
			last_size = 0;
			last_offset = 0;
			return false;
		}
	}

	memcpy(data, OV13855_eeprom_data, size);
   	return true;
}
