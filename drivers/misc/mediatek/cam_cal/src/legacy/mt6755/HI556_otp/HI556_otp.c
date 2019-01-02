/*
 * Driver for OTP
 *
 *
 */
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"

#include "HI556_otp.h"
//#include <asm/system.h>  // for SMP

#define COMMON_CAM_CAL_DRV
//#define EEPROMGETDLT_DEBUG
//#define EEPROM_DEBUG  //plz enable this flag when you need
#ifdef EEPROM_DEBUG
#define EEPROMDB pr_err
#else
#define EEPROMDB(x,...)
#endif

#define EEPROM_I2C_BUSNUM 1

/*******************************************************************************
*
********************************************************************************/
#define EEPROM_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define EEPROM_DRVNAME "CAM_CAL_DRV"
#define EEPROM_I2C_GROUP_ID 0

/*******************************************************************************
*
********************************************************************************/
//#define FM50AF_EEPROM_I2C_ID 0x28
//#define FM50AF_EEPROM_I2C_ID 0x28


/*******************************************************************************/
/* define LSC data for M24C08F EEPROM on L10 project */
/********************************************************************************/
#define SampleNum 221
#define Read_NUMofEEPROM 2
#define Boundary_Address 256
//#define EEPROM_Address_Offset 0xC


/*******************************************************************************
*
********************************************************************************/
//static unsigned short EEPROM_Address[2] = {0x0,0x0} ;

extern struct hi556_otp_struct qt_main_otp;
extern struct hi556_otp_struct qh_sub_otp;

/*******************************************************************************
*
********************************************************************************/
#if defined( COMMON_CAM_CAL_DRV)
int HI556_OTP_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
#else //COMMON_CAM_CAL_DRV

#define NEW_UNLOCK_IOCTL

#endif//COMMON_CAM_CAL_DRV
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf = NULL;

#ifdef EEPROMGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            EEPROMDB("[HI556_OTP] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                EEPROMDB("[HI556_OTP] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
        ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
        //pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length, GFP_KERNEL);
    }
#if 0
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        EEPROMDB("[HI556_OTP] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
     EEPROMDB("[HI556_OTP] init Working buffer address 0x%8x  command is 0x%8x\n", (u32)pWorkingBuff, (u32)a_u4Command);

    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        EEPROMDB("[HI556_OTP] ioctl copy from user failed\n");
        return -EFAULT;
    }
#endif

    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:
            EEPROMDB("[HI556_OTP] Write CMD, There is no write case \n");
            break;
        case CAM_CALIOC_G_READ:
            pr_err("[HI556_OTP] Read CMD \n");
#ifdef EEPROMGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            EEPROMDB("[HI556_OTP] offset %x \n", ptempbuf->u4Offset);
            EEPROMDB("[HI556_OTP] length %x \n", ptempbuf->u4Length);
            EEPROMDB("[HI556_OTP] Before read Working buffer address 0x%8x \n", (u32)pWorkingBuff);

            if(ptempbuf->u4Offset == 0x70000556)
	          pWorkingBuff = (u8*)&qt_main_otp;
            else if(ptempbuf->u4Offset == 0x70000559)
                pWorkingBuff = (u8*)&qh_sub_otp;
            else{
		   pWorkingBuff = NULL;
		   return -EFAULT;
            	}
			
            EEPROMDB("[HI556_OTP] After read Working buffer data  0x%4x \n", *pWorkingBuff);

#ifdef EEPROMGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif

            break;
        default :
      	     EEPROMDB("[HI556_OTP] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        EEPROMDB("[HI556_OTP] to user length %d \n", ptempbuf->u4Length);
        EEPROMDB("[HI556_OTP] to user  Working buffer address 0x%8x \n", (u32)pWorkingBuff);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            EEPROMDB("[HI556_OTP] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
#if 0 //do not free 
    kfree(pWorkingBuff);
#endif
    return i4RetValue;
}

#if defined( COMMON_CAM_CAL_DRV)
EXPORT_SYMBOL(HI556_OTP_Ioctl);
#endif

#if 0
MODULE_DESCRIPTION("OTP driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");
#endif

