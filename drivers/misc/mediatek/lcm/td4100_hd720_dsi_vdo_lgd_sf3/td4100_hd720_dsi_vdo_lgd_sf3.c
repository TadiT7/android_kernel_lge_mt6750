/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifdef BUILD_LK
#include <string.h>
#include <mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "upmu_common.h"
#include <linux/string.h>
#include "mt_gpio.h"
#include <mach/gpio_const.h>
#endif

#include "lcm_drv.h"
#if defined(BUILD_LK)
#define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif
#if defined(BUILD_LK)
#include <boot_mode.h>
#else
#include <linux/types.h>
#include <upmu_hw.h>
#include "rt4832.h"
#endif
#include <linux/input/lge_touch_notify.h>


/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
/* pixel */
#define FRAME_WIDTH              (720)
#define FRAME_HEIGHT             (1280)

/* physical dimension */
#define PHYSICAL_WIDTH        (66)
#define PHYSICAL_HEIGHT         (117)

#define LCM_ID       (0xb9)
#define LCM_DSI_CMD_MODE        0

#define REGFLAG_DELAY 0xAB
#define REGFLAG_END_OF_TABLE 0xAA	/* END OF REGISTERS MARKER */

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#define UDELAY(n)                                             (lcm_util.udelay(n))
#define MDELAY(n)                                             (lcm_util.mdelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V3(para_tbl, size, force_update)    lcm_util.dsi_set_cmdq_V3(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)    lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                   lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)               lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                    lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)            lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static unsigned int need_set_lcm_addr = 1;

#ifndef GPIO_LCM_RST
#define GPIO_LCM_RST         (GPIO158 | 0x80000000)
#define GPIO_LCM_RST_M_GPIO   GPIO_MODE_00
#define GPIO_LCM_RST_M_LCM_RST   GPIO_MODE_01
#endif

#ifndef GPIO_LCD_LDO_EN
#define GPIO_LCD_LDO_EN         (GPIO16 | 0x80000000)
#define GPIO_LCD_LDO_EN_M_GPIO   GPIO_MODE_00
#define GPIO_LCD_LDO_EN_M_LCM_RST   GPIO_MODE_01
#endif

#ifndef GPIO_TOUCH_LDO_EN
#define GPIO_TOUCH_LDO_EN         (GPIO43 | 0x80000000)
#define GPIO_TOUCH_LDO_EN_M_GPIO   GPIO_MODE_00
#define GPIO_TOUCH_LDO_EN_M_LCM_RST   GPIO_MODE_01
#endif

#define LGE_LPWG_SUPPORT

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

/* extern unsigned short mt6328_set_register_value(PMU_FLAGS_LIST_ENUM flagname, unsigned int val); */

static LCM_setting_table_V3 lcm_initialization_setting_V3[] = {
	/*{0x29, 0xB0,  1, {0x04}},
	   {0x29, 0xB3,  3, {0x10, 0x00, 0x06}},
	   {0x29, 0xB6,  7, {0x32, 0x33, 0x80, 0x00, 0x00, 0x07, 0x86}},
	   {0x29, 0xB8,  6, {0x07, 0x87, 0x26, 0x18, 0x00, 0x32}},
	   {0x29, 0xB9,  6, {0x07, 0x75, 0x61, 0x20, 0x16, 0x87}},
	   {0x29, 0xBA,  6, {0x07, 0x70, 0x81, 0x20, 0x45, 0xB4}},
	   {0x29, 0xBB,  3, {0x01, 0x1E, 0x14}},
	   {0x29, 0xBC,  3, {0x01, 0x50, 0x32}},
	   {0x29, 0xBD,  3, {0x01, 0xB4, 0xA0}},
	   {0x29, 0xBE,  1, {0x04}},
	   {0x29, 0x02,  3, {0x3C, 0x80, 0x09}},
	   {0x29, 0xC0,  17, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	   {0x29, 0xC1,  34, {0x00, 0x48, 0x00, 0x00, 0x33, 0x08, 0x11, 0x00, 0x11, 0x00,
				0x73, 0x23, 0x23, 0x11, 0x00, 0x00, 0x00,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDF, 0x00, 0x30, 0x00,
				0x01, 0x00, 0x00, 0x00}},
	   {0x29, 0xC2,  19, {0x00, 0xF0, 0x05, 0x00, 0x0A, 0x04, 0x08, 0x00, 0x24, 0x19,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00}},
	   {0x29, 0xC3,  63, {0x51, 0x15, 0x11, 0x51, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	   0x00, 0x01, 0x01, 0x03, 0x28, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x67, 0x01,
				0x00, 0x00, 0x00, 0x00, 0x67, 0x01,
	   0x00, 0x00, 0x00, 0x00, 0x40, 0x20, 0x01}},
	   {0x29, 0xC4,  40, {0x70, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01}},
	   {0x29, 0xC5,  10, {0x08, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x2D, 0x41}},
	   {0x29, 0xC6,  33, {0xC2, 0x14, 0x89, 0x00, 0x00, 0x14, 0x89, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC2, 0xC2}},
	   {0x39, 0xC7,  46, {0x00, 0x0C, 0x16, 0x24, 0x2F, 0x39, 0x4F, 0x5E, 0x6B, 0x76,
				0x28, 0x33, 0x42, 0x58, 0x63, 0x72, 0x85, 0x96, 0x97,
	   0x00, 0x0C, 0x16, 0x24, 0x2F, 0x39, 0x4F, 0x5E, 0x6B, 0x76, 0x28, 0x33, 0x42,
				0x58, 0x63, 0x72, 0x85, 0x96, 0x97,
	   0x00, 0x97, 0x00, 0x97, 0x00, 0x97, 0x00, 0x97}},
	   {0x39, 0xC8,  55, {0x01, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00,
				0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00,
	   0x00, 0xFF, 0x02, 0x01, 0xFC, 0x00, 0x00, 0x01, 0xFF, 0x01, 0xC4, 0x00, 0x00,
				0xFE, 0x01, 0xFC, 0xF3, 0x00, 0x00,
	   0xFF, 0x02, 0x01, 0xFC, 0x00, 0x00, 0x01, 0xFF, 0x03, 0x75, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x06, 0x00}},
	   {0x29, 0xC9,  19, {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00,
				0x00, 0x00, 0x00, 0xFC, 0x00}},
	   {0x29, 0xCA,  43, {0x1D, 0xFC, 0xFC, 0xFC, 0x00, 0xE2, 0xE2, 0xD3, 0x00, 0xD3, 0xE2, 0xE2, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	   {0x39, 0xCC,  45, {0x00, 0x00, 0x00, 0x00, 0xD2, 0x72, 0x12, 0x16, 0x1A, 0x1E, 0xD0, 0x70, 0x42, 0x46,
				0x00, 0x00, 0x00, 0x00, 0x00,
	   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x40, 0x70, 0xD0, 0x1C, 0x18,
				0x14, 0x10,
	   0x72, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x00}},
	   {0x29, 0xCD,  19, {0x03, 0x06, 0x69, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x01, 0x00, 0x00}},
	   {0x29, 0xCE,  24, {0x7D, 0x40, 0x48, 0x56, 0x67, 0x78, 0x88, 0x98, 0xA7, 0xB5, 0xC3, 0xD1, 0xDE, 0xE9,
				0xF2, 0xFA, 0xFF, 0x04, 0x00, 0x0F, 0x0F, 0x42, 0x00, 0x00}},
	   {0x29, 0xCF,  2, {0x48, 0x10}},
	   {0x29, 0xD0,  18, {0x11, 0x04, 0x59, 0xD9, 0x03, 0x10, 0x10, 0x40, 0x19, 0x08, 0x99, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00}},
	   {0x29, 0xD1,  1, {0x04}},
	   {0x29, 0xD3,  27, {0xBB, 0x3B, 0x33, 0x3B, 0x44, 0x3B, 0x44, 0x3B, 0x00, 0x00, 0xEC, 0x91, 0x81, 0x23,
				0x22, 0xD3, 0xD3, 0x3B, 0x3B,
	   0xBB, 0x4F, 0xD0, 0x3C, 0x10, 0x12, 0x10, 0x00, 0x10}},
	   {0x29, 0xD4,  24, {0x80, 0x04, 0x04, 0x33, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x0A, 0x90, 0x05, 0x00, 0x64, 0x94}},
	   {0x29, 0xD5,  10, {0x06, 0x00, 0x00, 0x01, 0x7D, 0x01, 0x7D, 0x01, 0x00, 0x00}},
	   {0x29, 0xD6,  1, {0xC1}},
	   {0x29, 0xD7,  36, {0xF6, 0xFF, 0x03, 0x05, 0x43, 0x24, 0x80, 0x1F, 0xC7, 0x1F, 0x1B, 0x00, 0x0F, 0x01,
				0x20, 0x08, 0x80, 0x3F, 0x00,
	   0x78, 0x00, 0x40, 0x24, 0x15, 0x00, 0x33, 0x02, 0xC0, 0xAF, 0xCB, 0x60, 0x30, 0xFC, 0x00, 0x3F, 0x00}},
	   {0x29, 0xD8,  8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	   {0x29, 0xDD,  4, {0x30, 0x06, 0x23, 0x65}}, */
	{0x05, 0x11, 1, {0x00} },
	{0x05, 0x29, 1, {0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static struct LCM_setting_table __attribute__ ((unused)) lcm_backlight_level_setting[] = {
	{
		0x51, 1, {
	0xFF} }, {
		REGFLAG_END_OF_TABLE, 0x00, {
	} }
};

#if defined(CONFIG_LGE_READER_MODE)
static LCM_setting_table_V3 lcm_reader_mode_off[] = {
	/* reader_mode off command */
	{0x15, 0x84, 1, {0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_reader_mode_low[] = {
	/* 6200K */
	{0x15, 0x84, 1, {0x50} },	/* cold setting(temp) */
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_reader_mode_mid[] = {
	/* 5500K */
	{0x15, 0x84, 1, {0x70} },	/* neutral */
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_reader_mode_high[] = {
	/* 5300K */
	{0x15, 0x84, 1, {0x7F} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_reader_mode_mono[] = {
	/* reader_mode mono(set_mid_setting) */
	{0x15, 0x84, 1, {0x70} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static void lcm_reader_mode(LCM_READER_MODE mode)
{
	switch (mode) {
	case READER_MODE_OFF:
		dsi_set_cmdq_V3(lcm_reader_mode_off,
				sizeof(lcm_reader_mode_off) / sizeof(LCM_setting_table_V3), 1);
		break;
	case READER_MODE_LOW:
		dsi_set_cmdq_V3(lcm_reader_mode_low,
				sizeof(lcm_reader_mode_low) / sizeof(LCM_setting_table_V3), 1);
		break;
	case READER_MODE_MID:
		dsi_set_cmdq_V3(lcm_reader_mode_mid,
				sizeof(lcm_reader_mode_mid) / sizeof(LCM_setting_table_V3), 1);
		break;
	case READER_MODE_HIGH:
		dsi_set_cmdq_V3(lcm_reader_mode_high,
				sizeof(lcm_reader_mode_high) / sizeof(LCM_setting_table_V3), 1);
		break;
	case READER_MODE_MONO:
		dsi_set_cmdq_V3(lcm_reader_mode_mono,
				sizeof(lcm_reader_mode_mono) / sizeof(LCM_setting_table_V3), 1);
		break;
	default:
		break;
	}
	LCM_PRINT("READER MODE : %d\n", mode);
}
#endif

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy((void *)&lcm_util, (void *)util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	/* physical size */
	params->physical_width = PHYSICAL_WIDTH;
	params->physical_height = PHYSICAL_HEIGHT;

	params->dsi.mode = SYNC_PULSE_VDO_MODE;	/* SYNC_EVENT_VDO_MODE; //BURST_VDO_MODE;//SYNC_PULSE_VDO_MODE; */
	/* enable tearing-free */
	params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* params->dsi.intermediat_buffer_num = 0; */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.cont_clock = 1;

	params->dsi.vertical_sync_active = 191;	/* 20150824 , 1->2 */
	params->dsi.vertical_backporch = 230;
	params->dsi.vertical_frontporch = 17;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 40;
	params->dsi.horizontal_frontporch = 80;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
/*
	params->dsi.HS_TRAIL = 10;
	params->dsi.HS_ZERO= 20;
	params->dsi.LPX= 10;
	params->dsi.HS_PRPR= 8;
*/
	params->dsi.PLL_CLOCK = 262;
}

static void init_lcm_registers(void)
{
	dsi_set_cmdq_V3(lcm_initialization_setting_V3,
			sizeof(lcm_initialization_setting_V3) / sizeof(LCM_setting_table_V3), 1);
	LCM_PRINT("[LCD] init_lcm_registers\n");
}

static void init_lcm_registers_sleep(void)
{
	unsigned int data_array[1];
	/* MDELAY(10); */
	data_array[0] = 0x00280500;	/* Display Off */
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
	data_array[0] = 0x00100500;	/* enter sleep */
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(50);
	LCM_PRINT("[LCD] init_lcm_registers_sleep\n");

	/* Call touch lpwg setting for recommened sequence */
	/* LCM_PRINT("[LCD][TOUCH] SetNextState_Touch called.....here.......start\n"); */
	/* SetNextState_For_Touch(0); */
	/* LCM_PRINT("[LCD][TOUCH] SetNextState_Touch called.....here.......end\n"); */
}

/* 1.8v LDO is always on */
static void ldo_1v8io_on(void)
{
	mt_set_gpio_out(GPIO_LCD_LDO_EN, GPIO_OUT_ONE);
}

/* 1.8v LDO is always on */
static void ldo_1v8io_off(void)
{
	mt_set_gpio_out(GPIO_LCD_LDO_EN, GPIO_OUT_ZERO);
}

#if 0
/* VGP1 3.0v LDO enable */
static void ldo_3v0_on(void)
{
	mt_set_gpio_out(GPIO_TOUCH_LDO_EN, GPIO_OUT_ONE);
}

/* VGP1 3.0v LDO disable */
static void ldo_3v0_off(void)
{
	mt_set_gpio_out(GPIO_TOUCH_LDO_EN, GPIO_OUT_ZERO);
}
#endif

/* DSV power +5.8V,-5.8v */
static void ldo_p5m5_dsv_5v5_on(void)
{
#if defined(BUILD_LK)
	chargepump_DSV_on();
#else
	rt4832_dsv_ctrl(1);
#endif
}

/* DSV power +5.8V,-5.8v */
static void ldo_p5m5_dsv_5v5_off(void)
{
#if defined(BUILD_LK)
	chargepump_DSV_off();
#else
	rt4832_dsv_ctrl(0);
#endif

}


static void reset_lcd_module(unsigned char reset)
{
	mt_set_gpio_mode(GPIO_LCM_RST, GPIO_LCM_RST_M_GPIO);
	/* mt_set_gpio_pull_enable(GPIO_LCM_RST, GPIO_PULL_ENABLE); */
	mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);

	if (reset) {
		mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
		LCM_PRINT("Reset High\n");
	} else {
		mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
		LCM_PRINT("Reset Low\n");
	}
}

static void lcm_init(void)
{
	reset_lcd_module(0);

	ldo_1v8io_on();
	MDELAY(10);

	reset_lcd_module(1);
	MDELAY(200);

	ldo_p5m5_dsv_5v5_on();
	MDELAY(30);

	init_lcm_registers();

	need_set_lcm_addr = 1;
	LCM_PRINT("[LCD] lcm_init\n");
}

static void lcm_suspend(void)
{
	rt4832_dsv_mode_change(1);
	MDELAY(10);
	init_lcm_registers_sleep();
	LCM_PRINT("[LCD] lcm_suspend\n");

#if 0
	reset_lcd_module(0);
	ldo_p5m5_dsv_5v5_off();
	MDELAY(20);
	ldo_1v8io_off();

	LCM_PRINT("[LCD] lcm_shutdown\n");
#endif
}

static void lcm_shutdown(void)
{
	return;
	reset_lcd_module(0);
	ldo_p5m5_dsv_5v5_off();
	MDELAY(20);
	ldo_1v8io_off();

	LCM_PRINT("[LCD] lcm_shutdown\n");
}

#if 0
static void lcm_suspend_mfts(void)
{
	reset_lcd_module(0);
	MDELAY(10);
	ldo_p5m5_dsv_5v5_off();
	ldo_1v8io_off();
	MDELAY(20);

	LCM_PRINT("[LCD] lcm_suspend_for_mfts\n");
}
#endif

static void lcm_resume_power(void)
{
/* ldo_1v8io_off(); */
/* ldo_p5m5_dsv_5v5_off(); */
/* reset_lcd_module(0); */
/* MDELAY(5); */

/* ldo_1v8io_on(); */
/* MDELAY(10); */
	LCM_PRINT("[LCD] no lcm_resume_power\n");
}

static void lcm_resume(void)
{
	rt4832_dsv_mode_change(0);
	MDELAY(10);
	lcm_init();
	need_set_lcm_addr = 1;
	LCM_PRINT("[LCD] lcm_resume\n");
}

#if 0
static void lcm_esd_recover(void)
{
	lcm_suspend();
	MDELAY(20);
	lcm_resume();

	LCM_PRINT("[LCD] lcm_esd_recover\n");
}

static void lcm_resume_mfts(void)
{
/* ldo_1v8io_on(); */

/* MDELAY(5); */

	LCM_PRINT("[LCD] lcm_resume_mfts\n");
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	/* need update at the first time */
	if (need_set_lcm_addr) {
		data_array[0] = 0x00053902;
		data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
		data_array[2] = (x1_LSB);
		dsi_set_cmdq(data_array, 3, 1);

		data_array[0] = 0x00053902;
		data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
		data_array[2] = (y1_LSB);
		dsi_set_cmdq(data_array, 3, 1);
		need_set_lcm_addr = 0;
	}

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
	LCM_PRINT("lcm_update\n");
}

static unsigned int lcm_compare_id(void)
{
	LCM_PRINT("lcm_compare_id\n");
	return 1;
}
#endif

/* --------------------------------------------------------------------------- */
/* Get LCM Driver Hooks */
/* --------------------------------------------------------------------------- */
LCM_DRIVER td4100_hd720_dsi_vdo_lgd_sf3_drv = {
	.name = "td4100_hd720_dsi_vdo_lgd_sf3",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.resume_power = lcm_resume_power,
	.suspend_2nd = lcm_shutdown,
/* .shutdown = lcm_shutdown, */
/* .suspend_mfts = lcm_suspend_mfts, */
/* .resume_mfts = lcm_resume_mfts, */
/* .compare_id = lcm_compare_id, */
/* .update = lcm_update, */
#if (!defined(BUILD_UBOOT) && !defined(BUILD_LK))
/* .esd_recover = lcm_esd_recover, */
#endif
#if defined(CONFIG_LGE_READER_MODE)
	.reader_mode = lcm_reader_mode,
#endif
};
