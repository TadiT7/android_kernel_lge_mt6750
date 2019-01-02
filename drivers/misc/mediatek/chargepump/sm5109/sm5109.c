/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/timer.h>
#include <string.h>
#else
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#endif

#include "sm5109.h"

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
#ifdef BUILD_LK
static struct mt_i2c_t sm5109_i2c;
#else
static struct i2c_client *sm5109_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int sm5109_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int sm5109_i2c_remove(struct i2c_client *client);
#endif

#define CPD_TAG                  "[chargepump] "
#define CPD_FUN(f)               printk(CPD_TAG"%s\n", __FUNCTION__)
#define CPD_ERR(fmt, args...)    printk(CPD_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define CPD_LOG(fmt, args...)    printk(CPD_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)

/*****************************************************************************
 * Extern Area
 *****************************************************************************/

#define SM5109_MAX_RETRY_I2C_XFER (100)
#define SM5109_I2C_WRITE_DELAY_TIME 1

/* i2c read routine for API*/
static char sm5109_i2c_read(struct i2c_client *client, u8 reg_addr,
            u8 *data, u8 len)
{
        #if !defined BMA_USE_BASIC_I2C_FUNC
        s32 dummy;
        if (NULL == client)
            return -1;

        while (0 != len--) {
            #ifdef BMA_SMBUS
            dummy = i2c_smbus_read_byte_data(client, reg_addr);
            if (dummy < 0) {
                CPD_ERR("i2c bus read error\n");
                return -1;
            }
            *data = (u8)(dummy & 0xff);
            #else
            dummy = i2c_master_send(client, (char *)&reg_addr, 1);
            if (dummy < 0)
            {
                CPD_ERR("send dummy is %d\n", dummy);
                return -1;
            }

            dummy = i2c_master_recv(client, (char *)data, 1);
            if (dummy < 0)
            {
                CPD_ERR("recv dummy is %d\n", dummy);
                return -1;
            }
            #endif
            reg_addr++;
            data++;
        }
        return 0;
        #else
        int retry;

        struct i2c_msg msg[] = {
                {
                    .addr = client->addr,
                    .flags = 0,
                    .len = 1,
                    .buf = &reg_addr,
                },

                {
                    .addr = client->addr,
                    .flags = I2C_M_RD,
                    .len = len,
                    .buf = data,
                },
        };
        for (retry = 0; retry < SM5109_MAX_RETRY_I2C_XFER; retry++) {
            if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
                break;
            else
                mdelay(SM5109_I2C_WRITE_DELAY_TIME);
        }

        if (SM5109_MAX_RETRY_I2C_XFER <= retry) {
            CPD_ERR("I2C xfer error\n");
            return -EIO;
        }

        return 0;
        #endif
}

/* i2c write routine for */
static char sm5109_i2c_write(struct i2c_client *client, u8 reg_addr,
            u8 *data, u8 len)
{
        #if !defined BMA_USE_BASIC_I2C_FUNC
        s32 dummy;

        if (NULL == client)
            return -1;

        while (0 != len--) {
            #if 1
            dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
            #else
            buffer[0] = reg_addr;
            buffer[1] = *data;
            dummy = i2c_master_send(client, (char *)buffer, 2);
            #endif

            reg_addr++;
            data++;
            if (dummy < 0) {
                return -1;
            }
        }

        #else
        u8 buffer[2];
        int retry;
        struct i2c_msg msg[] = {
                {
                    .addr = client->addr,
                    .flags = 0,
                    .len = 2,
                    .buf = buffer,
                },
        };

        while (0 != len--) {
            buffer[0] = reg_addr;
            buffer[1] = *data;
            for (retry = 0; retry < SM5109_MAX_RETRY_I2C_XFER; retry++) {
                if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0) {
                    break;
                }
                else {
                    mdelay(SM5109_I2C_WRITE_DELAY_TIME);
                }
            }
            if (SM5109_MAX_RETRY_I2C_XFER <= retry) {
                return -EIO;
            }
            reg_addr++;
            data++;
        }
        #endif
        //CPD_LOG("dw8767_i2c_write \n");
        return 0;
}

static int sm5109_smbus_read_byte(struct i2c_client *client,
            unsigned char reg_addr, unsigned char *data)
{
        return sm5109_i2c_read(client,reg_addr,data,1);
}

static int sm5109_smbus_write_byte(struct i2c_client *client,
            unsigned char reg_addr, unsigned char *data)
{
        int ret_val = 0;
        int i = 0;

        ret_val = sm5109_i2c_write(client,reg_addr,data,1);

        for ( i = 0; i < 5; i++)
        {
            if (ret_val != 0)
                sm5109_i2c_write(client,reg_addr,data,1);
            else{
                CPD_ERR("[sm5109] : sm5109_smbus_write_byte fail: %d\n",ret_val);
                return ret_val;
            }
        }
        return ret_val;
}

void sm5109_dsv_init(void)
{
	unsigned char data = 0;

	if(sm5109_i2c_client == NULL){
		LCD_BIAS_PRINT("sm5109 i2c client is null\n");
		return;
	}

	data = 0x2F;
	sm5109_smbus_write_byte(sm5109_i2c_client, LCD_BIAS_VPOS_ADDR, &data);
	data = 0x0F;
	sm5109_smbus_write_byte(sm5109_i2c_client, LCD_BIAS_VNEG_ADDR, &data);
	data = 0x03;
	sm5109_smbus_write_byte(sm5109_i2c_client, LCD_BIAS_APPS_ADDR, &data);

	sm5109_smbus_read_byte(sm5109_i2c_client, LCD_BIAS_VPOS_ADDR, &data);
	LCD_BIAS_PRINT("[LCD][BIAS] LCD_BIAS_VPOS_ADDR data = 0x%x\n", data);
	sm5109_smbus_read_byte(sm5109_i2c_client, LCD_BIAS_VNEG_ADDR, &data);
	LCD_BIAS_PRINT("[LCD][BIAS] LCD_BIAS_VNEG_ADDR data = 0x%x\n", data);
	sm5109_smbus_read_byte(sm5109_i2c_client, LCD_BIAS_APPS_ADDR, &data);
	LCD_BIAS_PRINT("[LCD][BIAS] LCD_BIAS_APPS_ADDR data = 0x%x\n", data);
}

/*****************************************************************************
 * Data Structure
 *****************************************************************************/
static const struct of_device_id i2c_of_match[] = {
    { .compatible = "mediatek,i2c_lcd_bias", },
    {},
};

static const struct i2c_device_id sm5109_i2c_id[] = {
    {SM5109_I2C_ID_NAME, 0},
    {},
};

static struct i2c_driver sm5109_i2c_driver = {
/************************************************************
Attention:
Althouh i2c_bus do not use .id_table to match, but it must be defined,
otherwise the probe function will not be executed!
************************************************************/
    .id_table = sm5109_i2c_id,
    .probe = sm5109_i2c_probe,
    .remove = sm5109_i2c_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = SM5109_I2C_ID_NAME,
#ifdef CONFIG_OF
        .of_match_table = i2c_of_match,
#endif
    },
};

/*****************************************************************************
 * Function
 *****************************************************************************/

static int sm5109_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    if (NULL == client) {
        LCD_BIAS_PRINT("[LCD][BIAS] i2c_client is NULL\n");
        return -1;
    }

    sm5109_i2c_client = client;
    LCD_BIAS_PRINT("[LCD][BIAS] sm5109_i2c_probe success addr = 0x%x\n", client->addr);

    return 0;
}

static int sm5109_i2c_remove(struct i2c_client *client)
{
    sm5109_i2c_client = NULL;
    i2c_unregister_device(client);

    return 0;
}

static int __init sm5109_init(void)
{
    if (i2c_add_driver(&sm5109_i2c_driver)) {
        LCD_BIAS_PRINT("[LCD][BIAS] Failed to register sm5109_i2c_driver!\n");
        return -1;
    }

    return 0;
}

static void __exit sm5109_exit(void)
{
    i2c_del_driver(&sm5109_i2c_driver);
}

module_init(sm5109_init);
module_exit(sm5109_exit);

MODULE_AUTHOR("Woonghwan.lee <woonghwan.lee@lge.com>");
MODULE_DESCRIPTION("MTK LCD BIAS I2C Driver");
MODULE_LICENSE("GPL");
