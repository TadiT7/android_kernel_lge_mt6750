/*
 * Driver for EEPROM
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

#include "DW9768.h"
//#include <asm/system.h>  // for SMP
#define COMMON_CAM_CAL_DRV
//#define EEPROMGETDLT_DEBUG
#define EEPROM_DEBUG
#ifdef EEPROM_DEBUG
#define EEPROMDB pr_debug
#else
#define EEPROMDB(x,...)
#endif


static DEFINE_SPINLOCK(g_EEPROMLock); // for SMP
#define EEPROM_I2C_BUSNUM 1
//static struct i2c_board_info __initdata kd_eeprom_dev={ I2C_BOARD_INFO("CAM_CAL_DRV", 0xA0>>1)};

/*******************************************************************************
*
********************************************************************************/
#define EEPROM_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define EEPROM_DRVNAME "CAM_CAL_DRV"

#define Read_NUMofEEPROM 2

/*******************************************************************************
*
********************************************************************************/
static struct i2c_client * g_pstI2Cclient = NULL;
static dev_t g_EEPROMdevno = MKDEV(EEPROM_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pEEPROM_CharDrv = NULL;
static struct class *EEPROM_class = NULL;
static atomic_t g_EEPROMatomic;
static unsigned short EEPROM_Address[2] = {0x0,0x0} ;

/*******************************************************************************
*
********************************************************************************/
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iBurstReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);

/*******************************************************************************
*
********************************************************************************/

static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
#if 0
   int  i4RetValue = 0;
   int  i4ResidueDataLength;
   u32 u4IncOffset = 0;
   u32 u4CurrentOffset;
   u8 * pBuff;

   EEPROMDB("[DW9768_EEPROM] iWriteData\n" );

   if (ui4_offset + ui4_length >= 0x2000)
   {
      EEPROMDB("[DW9768_EEPROM] Write Error!! S-24CS64A not supprt address >= 0x2000!! \n" );
      return -1;
   }

   i4ResidueDataLength = (int)ui4_length;
   u4CurrentOffset = ui4_offset;
   pBuff = pinputdata;

   EEPROMDB("[DW9768_EEPROM] iWriteData u4CurrentOffset is %d \n",u4CurrentOffset);

   do
   {
       if(i4ResidueDataLength >= 6)
       {
           //i4RetValue = iWriteEEPROM((u16)u4CurrentOffset, 6, pBuff);
           if (i4RetValue != 0)
           {
                EEPROMDB("[EEPROM] I2C iWriteData failed!! \n");
                return -1;
           }
           u4IncOffset += 6;
           i4ResidueDataLength -= 6;
           u4CurrentOffset = ui4_offset + u4IncOffset;
           pBuff = pinputdata + u4IncOffset;
       }
       else
       {
           //i4RetValue = iWriteEEPROM((u16)u4CurrentOffset, i4ResidueDataLength, pBuff);
           if (i4RetValue != 0)
           {
                EEPROMDB("[EEPROM] I2C iWriteData failed!! \n");
                return -1;
           }
           u4IncOffset += 6;
           i4ResidueDataLength -= 6;
           u4CurrentOffset = ui4_offset + u4IncOffset;
           pBuff = pinputdata + u4IncOffset;
           //break;
       }
   }while (i4ResidueDataLength > 0);
   EEPROMDB("[DW9768_EEPROM] iWriteData done\n" );
#endif
   return 0;
}

int iReadDataFromDW9768(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{

        char puSendCmd[2];// = {(char)(ui4_offset & 0xFF) };
#if 0
        unsigned short SampleOffset = (unsigned short)((ui4_offset &0xFFF00000) >> 20) ;
#else
        unsigned short tempOffset = (unsigned short)((ui4_offset &0xFFF00000) >> 20) ;
        unsigned short SampleOffset = ((int)(tempOffset / 1024) << 10) | (tempOffset % 1024);
#endif
        short loop[2], loopCount;
        u8 * pBuff ;
        u32 u4IncOffset = 0;
        int  i4RetValue = 0;

        pBuff = pinputdata;

        EEPROMDB("[EEPROM] ui4_offset=%x ui4_offset(80)=%x ui4_offset(8)=%x\n",ui4_offset , (unsigned short)( (ui4_offset>>8) & 0x0000FFFF),SampleOffset);
        EEPROMDB("[EEPROM] EEPROM_Address[0]=%x EEPROM_Address[1]=%x\n",(EEPROM_Address[0]) ,(EEPROM_Address[1]));

        loop[0] = ((ui4_length>>4)<<4);
        loop[1] = ui4_length - loop[0];

        EEPROMDB("[EEPROM] loop[0]=%d loop[1]=%d\n",(loop[0]) ,(loop[1]));

        puSendCmd[0] = (char)( ((SampleOffset+u4IncOffset)>>8) & 0xFF);
        puSendCmd[1] = (char)( (SampleOffset+u4IncOffset) & 0xFF);

        for(loopCount=0; loopCount < Read_NUMofEEPROM; loopCount++)
        {
               do
               {
                   if( 16 <= loop[loopCount])
                   {

                       EEPROMDB("[EEPROM]1 loopCount=%d loop[loopCount]=%d puSendCmd[0]=%x puSendCmd[1]=%x, EEPROM(%x)\n",loopCount ,loop[loopCount],puSendCmd[0],puSendCmd[1],EEPROM_Address[loopCount] );
                       i4RetValue = iBurstReadRegI2C(puSendCmd , 2, (u8 *)pBuff, 16, EEPROM_Address[loopCount]);

                       if (i4RetValue != 0)
                       {
                            EEPROMDB("[EEPROM] I2C iReadData failed!! \n");
                            return -1;
                       }
                       u4IncOffset += 16;
                       loop[loopCount] -= 16;
                       puSendCmd[0] = (char)( ((SampleOffset+u4IncOffset)>>8) & 0xFF);
                       puSendCmd[1] = (char)( (SampleOffset+u4IncOffset) & 0xFF);
                       pBuff = pinputdata + u4IncOffset;
                   }
                   else if(0 < loop[loopCount])
                   {
                       EEPROMDB("[EEPROM]2 loopCount=%d loop[loopCount]=%d puSendCmd[0]=%x puSendCmd[1]=%x \n",loopCount ,loop[loopCount],puSendCmd[0],puSendCmd[1] );
                       i4RetValue = iBurstReadRegI2C(puSendCmd , 2, (u8 *)pBuff, 16, EEPROM_Address[loopCount]);
                       if (i4RetValue != 0)
                       {
                            EEPROMDB("[EEPROM] I2C iReadData failed!! \n");
                            return -1;
                       }
                       u4IncOffset += loop[loopCount];
                       loop[loopCount] -= loop[loopCount];
                       puSendCmd[0] = (char)( ((SampleOffset+u4IncOffset)>>8) & 0xFF);
                       puSendCmd[1] = (char)( (SampleOffset+u4IncOffset) & 0xFF);
                       pBuff = pinputdata + u4IncOffset;
                   }
               }while (loop[loopCount] > 0);
        }

   return 0;
}

/*******************************************************************************
*
********************************************************************************/
#if defined( COMMON_CAM_CAL_DRV)
int DW9768_EEPROM_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
#else //COMMON_CAM_CAL_DRV

#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int EEPROM_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else
static long EEPROM_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
#endif


#endif//COMMON_CAM_CAL_DRV
{
    int i4RetValue = 0, ResetCheck = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
    u8 readTryagain=0;

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
            EEPROMDB("[DW9768_EEPROM] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                EEPROMDB("[DW9768_EEPROM] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }
    if(NULL == pBuff)
    {
        EEPROMDB("[DW9768_EEPROM] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        EEPROMDB("[DW9768_EEPROM] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
     EEPROMDB("[DW9768_EEPROM] init Working buffer address 0x%8x  command is 0x%8x\n", (u32)pWorkingBuff, (u32)a_u4Command);


    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        EEPROMDB("[DW9768_EEPROM] ioctl copy from user failed\n");
        return -EFAULT;
    }

    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:
            EEPROMDB("[DW9768_EEPROM] Write CMD \n");
#ifdef EEPROMGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
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
            printk("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif
            break;
        case CAM_CALIOC_G_READ:
            EEPROMDB("[DW9768_EEPROM] Read CMD \n");
#ifdef EEPROMGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            EEPROMDB("[EEPROM] offset %x \n", ptempbuf->u4Offset);
            EEPROMDB("[EEPROM] length %x \n", ptempbuf->u4Length);
            EEPROMDB("[EEPROM] Before read Working buffer address 0x%8x \n", (u32)pWorkingBuff);

            if(4 <= ptempbuf->u4Length )
            {
                ResetCheck = *((u32 *)(ptempbuf->pu1Params));
            }
            if(ptempbuf->u4Offset == 0x800)
            {
              *(u16 *)pWorkingBuff = 0x3;

            }
            else if( ((ptempbuf->u4Offset & 0x000FFFFF) == 0x00009768) && (EEPROM_Address[0]==0x0) )
            {
                *(u32 *)pWorkingBuff = (ptempbuf->u4Offset | 0x10000000);
                EEPROM_Address[0] = (ptempbuf->u4Offset & 0xfff00000) >> 20;
                EEPROM_Address[1] = (ptempbuf->u4Offset & 0xfff00000) >> 20;
            }
            else if( ((ptempbuf->u4Offset & 0x000FFFFF) == 0x00009768) && (ResetCheck == 0xABCDFEDC) )
            {
                *(u32 *)pWorkingBuff = (ptempbuf->u4Offset | 0x10000000);
                EEPROMDB("[DW9768_EEPROM] Reset I2C address %x \n", *(u32 *)pWorkingBuff );

            }
            else
            {
               readTryagain=3;
               while(0 < readTryagain)
               {
                   i4RetValue =  iReadDataFromDW9768(ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
                   EEPROMDB("[DW9768_EEPROM] error (%d) Read retry (%d) \n", i4RetValue,readTryagain);
                   if(i4RetValue != 0)
                   {
                       readTryagain--;
                   }
                   else
                   {
                       readTryagain = 0;
                   }
               }
            }
            EEPROMDB("[DW9768_EEPROM] After read Working buffer data  0x%4x \n", *pWorkingBuff);

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
               EEPROMDB("[DW9768_EEPROM] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        EEPROMDB("[DW9768_EEPROM] to user length %d \n", ptempbuf->u4Length);
        EEPROMDB("[DW9768_EEPROM] to user  Working buffer address 0x%8x \n", (u32)pWorkingBuff);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            EEPROMDB("[DW9768_EEPROM] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int EEPROM_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    EEPROMDB("[DW9768_EEPROM] EEPROM_Open\n");
    spin_lock(&g_EEPROMLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_EEPROMLock);
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_EEPROMatomic,0);
    }
    spin_unlock(&g_EEPROMLock);

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int EEPROM_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_EEPROMLock);

    g_u4Opened = 0;

    atomic_set(&g_EEPROMatomic,0);

    spin_unlock(&g_EEPROMLock);

    return 0;
}

static const struct file_operations g_stEEPROM_fops =
{
    .owner = THIS_MODULE,
    .open = EEPROM_Open,
    .release = EEPROM_Release,
    //.ioctl = EEPROM_Ioctl
#if defined( COMMON_CAM_CAL_DRV)
#else
    .unlocked_ioctl = EEPROM_Ioctl
#endif
};

#define EEPROM_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterEEPROMCharDrv(void)
{
    struct device* EEPROM_device = NULL;

#if EEPROM_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_EEPROMdevno, 0, 1,EEPROM_DRVNAME) )
    {
        EEPROMDB("[DW9768_EEPROM] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_EEPROMdevno , 1 , EEPROM_DRVNAME) )
    {
        EEPROMDB("[DW9768_EEPROM] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pEEPROM_CharDrv = cdev_alloc();

    if(NULL == g_pEEPROM_CharDrv)
    {
        unregister_chrdev_region(g_EEPROMdevno, 1);

        EEPROMDB("[DW9768_EEPROM] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pEEPROM_CharDrv, &g_stEEPROM_fops);

    g_pEEPROM_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pEEPROM_CharDrv, g_EEPROMdevno, 1))
    {
        EEPROMDB("[DW9768_EEPROM] Attatch file operation failed\n");

        unregister_chrdev_region(g_EEPROMdevno, 1);

        return -EAGAIN;
    }

    EEPROM_class = class_create(THIS_MODULE, "EEPROMdrv");
    if (IS_ERR(EEPROM_class)) {
        int ret = PTR_ERR(EEPROM_class);
        EEPROMDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    EEPROM_device = device_create(EEPROM_class, NULL, g_EEPROMdevno, NULL, EEPROM_DRVNAME);

    return 0;
}

inline static void UnregisterEEPROMCharDrv(void)
{
    //Release char driver
    cdev_del(g_pEEPROM_CharDrv);

    unregister_chrdev_region(g_EEPROMdevno, 1);

    device_destroy(EEPROM_class, g_EEPROMdevno);
    class_destroy(EEPROM_class);
}


//////////////////////////////////////////////////////////////////////
#ifndef EEPROM_ICS_REVISION
static int EEPROM_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int EEPROM_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int EEPROM_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int EEPROM_i2c_remove(struct i2c_client *);

static const struct i2c_device_id EEPROM_i2c_id[] = {{EEPROM_DRVNAME,0},{}};


static struct i2c_driver EEPROM_i2c_driver = {
    .probe = EEPROM_i2c_probe,
    .remove = EEPROM_i2c_remove,
//   .detect = EEPROM_i2c_detect,
    .driver.name = EEPROM_DRVNAME,
    .id_table = EEPROM_i2c_id,
};

#ifndef EEPROM_ICS_REVISION
static int EEPROM_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
    strcpy(info->type, EEPROM_DRVNAME);
    return 0;
}
#endif
static int EEPROM_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
int i4RetValue = 0;
    EEPROMDB("[DW9768_EEPROM] Attach I2C \n");
//    spin_lock_init(&g_EEPROMLock);

    //get sensor i2c client
    spin_lock(&g_EEPROMLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = DW9768_DEVICE_ID>>1;
    spin_unlock(&g_EEPROMLock); // for SMP

    EEPROMDB("[EEPROM] g_pstI2Cclient->addr = 0x%8x \n",g_pstI2Cclient->addr);
    //Register char driver
    i4RetValue = RegisterEEPROMCharDrv();

    if(i4RetValue){
        EEPROMDB("[DW9768_EEPROM] register char device failed!\n");
        return i4RetValue;
    }


    EEPROMDB("[DW9768_EEPROM] Attached!! \n");
    return 0;
}

static int EEPROM_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static int EEPROM_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&EEPROM_i2c_driver);
}

static int EEPROM_remove(struct platform_device *pdev)
{
    i2c_del_driver(&EEPROM_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stEEPROM_Driver = {
    .probe        = EEPROM_probe,
    .remove    = EEPROM_remove,
    .driver        = {
        .name    = EEPROM_DRVNAME,
        .owner    = THIS_MODULE,
    }
};

#if 0
static struct platform_device g_stEEPROM_Device = {
    .name = EEPROM_DRVNAME,
    .id = 0,
    .dev = {
    }
};
#endif
/*
static int __init EEPROM_i2C_init(void)
{
    i2c_register_board_info(EEPROM_I2C_BUSNUM, &kd_eeprom_dev, 1);
    if(platform_driver_register(&g_stEEPROM_Driver)){
        EEPROMDB("failed to register DW9768_EEPROM driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stEEPROM_Device))
    {
        EEPROMDB("failed to register DW9768_EEPROM driver, 2nd time\n");
        return -ENODEV;
    }

    return 0;
}
*/
static void __exit EEPROM_i2C_exit(void)
{
    platform_driver_unregister(&g_stEEPROM_Driver);
}
void read_moduleID_DW9768(unsigned char * pinputdata)
{
    unsigned int module_vendor_id = 0x700;
    unsigned int offset = 0x09768;
    unsigned int module_vendor_id_temp = 0;

    module_vendor_id_temp = (module_vendor_id<<20)&0xfff00000;
    module_vendor_id_temp |= offset;
    iReadDataFromDW9768(module_vendor_id_temp, 2, pinputdata);

    return;
}

#if defined( COMMON_CAM_CAL_DRV)
EXPORT_SYMBOL(DW9768_EEPROM_Ioctl);
#else
//module_init(EEPROM_i2C_init);
module_exit(EEPROM_i2C_exit);
#endif
#if 0
MODULE_DESCRIPTION("EEPROM driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");
#endif
