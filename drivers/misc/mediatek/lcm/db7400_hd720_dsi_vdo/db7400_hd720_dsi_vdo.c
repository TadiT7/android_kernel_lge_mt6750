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

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
/* pixel */
#define FRAME_WIDTH			(720)
#define FRAME_HEIGHT			(1280)

/* physical dimension */
#define PHYSICAL_WIDTH        (71)
#define PHYSICAL_HEIGHT         (126)

#define LCM_ID       (0xb9)
#define LCM_DSI_CMD_MODE		0

#define REGFLAG_DELAY 0xAB
#define REGFLAG_END_OF_TABLE 0xAA	/* END OF REGISTERS MARKER */

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define UDELAY(n)	(lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V3(para_tbl, size, force_update)\
lcm_util.dsi_set_cmdq_V3(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)\
lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)\
lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)\
lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)\
lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)\
lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)\
lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static unsigned int need_set_lcm_addr = 1;

#ifndef GPIO_CTP_EINT_PIN
#define GPIO_CTP_EINT_PIN         (GPIO1 | 0x80000000)
#define GPIO_CTP_EINT_PIN_M_EINT  GPIO_MODE_00
#define GPIO_CTP_EINT_PIN_M_GPIO  GPIO_MODE_00
#endif

#ifndef GPIO_I2C0_SDA_PIN
#define GPIO_I2C0_SDA_PIN         (GPIO92 | 0x80000000)
#define GPIO_I2C0_SDA_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_I2C0_SDA_PIN_M_SDA   GPIO_MODE_01
#endif

#ifndef GPIO_I2C0_SCA_PIN
#define GPIO_I2C0_SCA_PIN         (GPIO93 | 0x80000000)
#define GPIO_I2C0_SCA_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_I2C0_SCA_PIN_M_SCL   GPIO_MODE_01
#endif

#ifndef GPIO_LCM_RST
#define GPIO_LCM_RST         (GPIO158 | 0x80000000)
#define GPIO_LCM_RST_M_GPIO   GPIO_MODE_00
#define GPIO_LCM_RST_M_LCM_RST   GPIO_MODE_01
#endif

#ifndef GPIO_TOUCH_RESET
#define GPIO_TOUCH_RESET         (GPIO10 | 0x80000000)
#define GPIO_TOUCH_RESET_M_GPIO   GPIO_MODE_00
#define GPIO_TOUCH_RESET_M_LCM_RST   GPIO_MODE_01
#endif

#ifndef GPIO_DSV_ENP_EN
#define GPIO_DSV_ENP_EN (GPIO11 | 0x80000000)
#define GPIO_DSV_ENP_EN_M_GPIO GPIO_MODE_00
#define GPIO_DSV_ENP_EN_M_KROW GPIO_MODE_06
#define GPIO_DSV_ENP_EN_M_PWM GPIO_MODE_05
#endif

#ifndef GPIO_DSV_ENN_EN
#define GPIO_DSV_ENN_EN (GPIO12 | 0x80000000)
#define GPIO_DSV_ENN_EN_M_GPIO GPIO_MODE_00
#define GPIO_DSV_ENN_EN_M_KROW GPIO_MODE_06
#define GPIO_DSV_ENN_EN_M_PWM GPIO_MODE_05
#endif

#ifndef GPIO_TOUCH_EN
#define GPIO_TOUCH_EN         (GPIO43 | 0x80000000)
#define GPIO_TOUCH_EN_M_GPIO   GPIO_MODE_00
#define GPIO_TOUCH_EN_M_LCM_RST   GPIO_MODE_01
#endif

/* extern kal_uint16 pmic_set_register_value(PMU_FLAGS_LIST_ENUM flagname, kal_uint32 val); */
/* extern void rt4832_dsv_ctrl(int enable); */
/* extern void rt4832_dsv_toggle_ctrl(void); */
static int db7400_cut_ver = 0xabab;

#define LGE_LPWG_SUPPORT

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static LCM_setting_table_V3 lcm_initialization_setting_cut5_V3[] = {
	{0x15, 0x36, 1, {0x00} },	/* set address mode */
	{0x15, 0xB0, 1, {0x00} },	/* Manufacturer Cmd Access Protect */
	{0x29, 0xB1, 2, {0x00, 0x30} },	/* RGB interface Set */
	{0x29, 0xB2, 6, {0x50, 0x00, 0x0C, 0x00, 0x00, 0x00} },	/* Module Characteristics Set */
	{0x29, 0XB3, 4, {0x4F, 0x80, 0x32, 0x56} },	/* Internal Timing Set */
	{0x29, 0XB4, 6, {0x05, 0x04, 0x02, 0x28, 0x11, 0x00} },	/* Channel Control */
	{0x29, 0XB5, 51, {0x19, 0x04, 0x1C, 0x22, 0x2F,
			  0x80, 0x2F, 0x1F, 0x04, 0x40,
			  0x05, 0x00, 0x20, 0x09, 0x04,
			  0x40, 0x05, 0x00, 0xA5, 0x0D,
			  0x2F, 0x28, 0x3F, 0xFF, 0x3F,
			  0xFF, 0x01, 0x7C, 0x00, 0x02,
			  0x50, 0x23, 0x40, 0x15, 0x6C,
			  0xCB, 0xBA, 0xA9, 0x97, 0x8D,
			  0x1F, 0x23, 0x40, 0x15, 0x6C,
			  0xCB, 0xBA, 0xA9, 0x97, 0x8D, 0x1F} },	/* GIP Control Set */
	{0x29, 0XB6, 2, {0x01, 0x03} },	/* Touch Enable */
	{0x29, 0XB7, 3, {0x00, 0x3C, 0x3E} },	/* GVDDP control */
	{0x29, 0XBA, 2, {0x00, 0x43} },	/* VCOM control */
	{0x29, 0XBB, 15, {0x00, 0x94, 0x8B, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x13} },	/* VGHL Control */
	{0x29, 0XBD, 2, {0x04, 0x33} },	/* Bias current */
	{0x15, 0XBE, 1, {0x44} },	/* VCI1 Control */
	{0x29, 0X95, 6, {0x00, 0x08, 0x10, 0x00, 0x00, 0x00} },	/* Write DSI Control */
	{0x29, 0xC6, 2, {0x0A, 0x00} },	/* MIPI Parameter Setting */
	{0x29, 0xCF, 1, {0x0F} },	/* Touch Control Set */
	{0x29, 0xD7, 26, {0x01, 0x13, 0xFF, 0x39, 0x0B,
			  0x04, 0x14, 0xF4, 0x01, 0x00,
			  0x00, 0x00, 0x00, 0x60, 0x01,
			  0x57, 0x57, 0x2F, 0x28, 0x28,
			  0x50, 0x00, 0x00, 0x00, 0x00, 0x00} },	/* Touch GIP control */
	{0x29, 0XF5, 5, {0x00, 0x06, 0x00, 0x00, 0x80} },	/* DB Check need */
	{0x15, 0XF6, 1, {0x06} },	/* VDD1 Set */
	{0x29, 0x81, 3, {0x18, 0x34, 0x00} },
	{0x29, 0x83, 33, {0x10, 0x00, 0x07, 0x0F, 0x17,
			  0x1F, 0x27, 0x2F, 0x37, 0x3F,
			  0x47, 0x4F, 0x57, 0x5F, 0x67,
			  0x6F, 0x77, 0x7F, 0x87, 0x8F,
			  0x97, 0x9F, 0xA7, 0xAF, 0xB7,
			  0xBF, 0xC7, 0xCF, 0xD7, 0xDF,
			  0xE7, 0xEF, 0xF5} },
	{0x29, 0xF0, 35, {0x18, 0x00, 0x00, 0x14, 0x00,
			  0x00, 0x00, 0x01, 0x00, 0x03,
			  0x2B, 0x01, 0x02, 0x53, 0x58,
			  0x5D, 0x62, 0xA6, 0xAB, 0xB0,
			  0xB0, 0xB0, 0xAF, 0xB0, 0xAD,
			  0x85, 0xB0, 0xB0, 0x5D, 0x58, 0x53, 0x4E,
			  0x0A, 0x05, 0x00} },	/* Power Sequence control */
	{0x29, 0xD0, 9, {0x63, 0x04, 0x72, 0x03, 0x00, 0x00, 0x42, 0x01, 0x02} },	/* Gamma */
	{0x29, 0xD1, 9, {0x62, 0x03, 0x73, 0x02, 0x00, 0x00, 0x42, 0x12, 0x01} },
	{0x29, 0xD2, 9, {0x53, 0x04, 0x72, 0x03, 0x00, 0x00, 0x42, 0x01, 0x02} },
	{0x29, 0xD3, 9, {0x52, 0x03, 0x73, 0x02, 0x00, 0x00, 0x42, 0x12, 0x01} },
	{0x29, 0xD4, 9, {0x53, 0x04, 0x72, 0x03, 0x00, 0x00, 0x42, 0x01, 0x02} },
	{0x29, 0xD5, 9, {0x52, 0x03, 0x73, 0x02, 0x00, 0x00, 0x42, 0x12, 0x01} },
	{0x05, 0x11, 1, {0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_initialization_setting_cut6_V3[] = {
	{0x15, 0x36, 1, {0x00} },	/* set address mode */
	{0x15, 0xB0, 1, {0x00} },	/* Manufacturer Cmd Access Protect */
	{0x29, 0xB1, 2, {0x00, 0x30} },	/* RGB interface Set */
	{0x29, 0xB2, 6, {0x50, 0x00, 0x0C, 0x00, 0x00, 0x00} },	/* Module Characteristics Set */
	{0x29, 0XB3, 4, {0x4F, 0x80, 0x32, 0x56} },	/* Internal Timing Set */
	{0x29, 0XB4, 6, {0x05, 0x04, 0x0A, 0x18, 0x11, 0x00} },	/* Channel Control */
	{0x29, 0XB5, 54, {0x19, 0x04, 0x1C, 0x22, 0x2F,
			  0x80, 0x2F, 0x1F, 0x04, 0x40,
			  0x05, 0x05, 0x00, 0x00, 0x09,
			  0x04, 0x40, 0x05, 0x05, 0x00,
			  0xA5, 0x0D, 0x2F, 0x28, 0x3F,
			  0xFF, 0x3F, 0xFF, 0x01, 0x7C,
			  0x00, 0x03, 0x78, 0x23, 0x40,
			  0x15, 0x6C, 0xCB, 0xBA, 0xA9,
			  0x97, 0x8D, 0x1F, 0x23, 0x40,
			  0x15, 0x6C, 0xCB, 0xBA, 0xA9,
			  0x97, 0x8D, 0x1F, 0x00} },	/* GIP Control Set */
	{0x29, 0XB6, 2, {0x01, 0x01} },	/* Touch Enable */
	{0x29, 0XB7, 3, {0x00, 0x3C, 0x3E} },	/* GVDDP control */
	{0x29, 0XBA, 2, {0x00, 0x43} },	/* VCOM control */
	{0x29, 0XBB, 15, {0x00, 0x94, 0x8B, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x13} },	/* VGHL Control */
	{0x29, 0XBD, 2, {0x03, 0x31} },	/* Bias current */
	{0x15, 0XBE, 1, {0x44} },	/* VCI1 Control */
	{0x29, 0X95, 6, {0x00, 0x08, 0x10, 0x00, 0x00, 0x00} },	/* Write DSI Control */
	{0x29, 0xC6, 2, {0x08, 0x00} },	/* MIPI Parameter Setting */
	{0x15, 0xCF, 1, {0x03} },	/* Touch Control Set */
	{0x29, 0xD7, 26, {0x00, 0x03, 0xFF, 0x00, 0x00,
			  0x04, 0x14, 0xCC, 0x01, 0x00,
			  0x00, 0x00, 0x00, 0x40, 0x01,
			  0x57, 0x57, 0x16, 0x28, 0x28,
			  0x69, 0x00, 0x00, 0x00, 0x00, 0x00} },	/* Touch GIP control */
	{0x15, 0X83, 1, {0x00} },	/* Contrast off */
	{0x39, 0X77, 46, {0x03, 0x00, 0x00, 0x00, 0x11,
			  0x05, 0x10, 0xd1, 0x0b, 0x20,
			  0xee, 0x05, 0x20, 0xb2, 0x18,
			  0x30, 0x11, 0x05, 0x50, 0xd1,
			  0x0b, 0x60, 0xb2, 0x18, 0x70,
			  0xb2, 0x18, 0x70, 0xb2, 0x18,
			  0x70, 0xb2, 0x18, 0x70, 0xb2,
			  0x18, 0x70, 0xb2, 0x18, 0x70,
			  0xb2, 0x18, 0x70, 0xb2, 0x18, 0x70} },	/* ief 1 */
	{0x39, 0x78, 65, {0x25, 0x29, 0xFF, 0xFD, 0xE1,
			  0x5F, 0x91, 0xFE, 0xDB, 0xFF,
			  0xFB, 0x2D, 0x01, 0x25, 0x81,
			  0x3E, 0x03, 0xE0, 0xBF, 0x10,
			  0x03, 0xD8, 0x1F, 0x09, 0xC3,
			  0x00, 0x1D, 0x71, 0xFF, 0xFD,
			  0xDB, 0x8F, 0x91, 0xFE, 0x77,
			  0xE0, 0xF6, 0x2D, 0x01, 0x25,
			  0X29, 0xFF, 0xFD, 0xE1, 0x5F,
			  0x91, 0xFE, 0xDB, 0xFF, 0xFB,
			  0x2D, 0x01, 0x25, 0x29, 0xFF,
			  0xFD, 0xE1, 0x5F, 0x91, 0xFE,
			  0xDB, 0xFF, 0xFB, 0x2D, 0x01} },	/* ief 2 */
	{0x39, 0x79, 65, {0x25, 0x29, 0xFF, 0xFD, 0xE1,
			  0x5F, 0x91, 0xFE, 0xDB, 0xFF,
			  0xFB, 0x2D, 0x01, 0x25, 0x29,
			  0xFF, 0xFD, 0xE1, 0x5F, 0x91,
			  0xFE, 0xDB, 0xFF, 0xFB, 0x2D,
			  0x01, 0x25, 0x29, 0xFF, 0xFD,
			  0xE1, 0x5F, 0x91, 0xFE, 0xDB,
			  0xFF, 0xFB, 0x2D, 0x01, 0x25,
			  0X29, 0xFF, 0xFD, 0xE1, 0x5F,
			  0x91, 0xFE, 0xDB, 0xFF, 0xFB,
			  0x2D, 0x01, 0x25, 0x29, 0xFF,
			  0xFD, 0xE1, 0x5F, 0x91, 0xFE,
			  0xDB, 0xFF, 0xFB, 0x2D, 0x01} },
	{0x39, 0x7A, 69, {0x25, 0x29, 0xFF, 0xFD, 0xE1,
			  0x5F, 0x91, 0xFE, 0xDB, 0xFF,
			  0xFB, 0x2D, 0x01, 0x25, 0x29,
			  0xFF, 0xFD, 0xE1, 0x5F, 0x91,
			  0xFE, 0xDB, 0xFF, 0xFB, 0x2D,
			  0x01, 0x25, 0x29, 0xFF, 0xFD,
			  0xE1, 0x5F, 0x91, 0xFE, 0xDB,
			  0xFF, 0xFB, 0x2D, 0x01, 0x25,
			  0X29, 0xFF, 0xFD, 0xE1, 0x5F,
			  0x91, 0xFE, 0xDB, 0xFF, 0xFB,
			  0x2D, 0x01, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00,
			  0x80, 0x00, 0x00, 0x00} },	/* ief 3 */
	{0x29, 0XF5, 5, {0x00, 0x06, 0x00, 0x00, 0x80} },	/* DB Check need */
	{0x15, 0XF6, 1, {0x06} },	/* VDD1 Set */
	{0x29, 0xF0, 35, {0x18, 0x00, 0x00, 0x14, 0x00,
			  0x00, 0x00, 0x01, 0x00, 0x03,
			  0x2B, 0x01, 0x02, 0x53, 0x58,
			  0x5D, 0x62, 0xA6, 0xAB, 0xB0,
			  0xB0, 0xB0, 0xAF, 0xB0, 0xAD,
			  0x85, 0xB0, 0xB0, 0x5D, 0x58,
			  0x53, 0x4E, 0x0A, 0x05, 0x00} },	/* Power Sequence control */
	{0x29, 0xD0, 9, {0x63, 0x04, 0x72, 0x03, 0x00, 0x00, 0x42, 0x01, 0x02} },	/* Gamma */
	{0x29, 0xD1, 9, {0x62, 0x03, 0x73, 0x02, 0x00, 0x00, 0x42, 0x12, 0x01} },
	{0x29, 0xD2, 9, {0x53, 0x04, 0x72, 0x03, 0x00, 0x00, 0x42, 0x01, 0x02} },
	{0x29, 0xD3, 9, {0x52, 0x03, 0x73, 0x02, 0x00, 0x00, 0x42, 0x12, 0x01} },
	{0x29, 0xD4, 9, {0x53, 0x04, 0x72, 0x03, 0x00, 0x00, 0x42, 0x01, 0x02} },
	{0x29, 0xD5, 9, {0x52, 0x03, 0x73, 0x02, 0x00, 0x00, 0x42, 0x12, 0x01} },
	{0x05, 0x11, 1, {0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_initialization_setting_V3_Disp_On[] = {
	{0x05, 0x29, 1, {0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_initialization_for_Disp_Off[] = {
	{0x05, 0x28, 1, {0x00} },
};

static LCM_setting_table_V3 lcm_initialization_for_Sleep_In[] = {
	{0x05, 0x10, 1, {0x00} },
};

static LCM_setting_table_V3 lcm_initialization_for_sleep_in_post_cut5[] = {
	{0x29, 0XB5, 27, {0x19, 0x04, 0x1C, 0x22, 0x2F,
			  0x80, 0x2F, 0x1F, 0x04, 0x40,
			  0x05, 0x00, 0x20, 0x09, 0x04,
			  0x40, 0x05, 0x00, 0xA5, 0x0D,
			  0x2F, 0x1F, 0x04, 0x00, 0x04,
			  0x00, 0x00} },
	{0x29, 0XB6, 2, {0x01, 0x01} },	/* Touch Enable */
	{0x15, 0xCF, 1, {0x07} },	/* Touch Control Set */
	{0x29, 0xD7, 26, {0x01, 0x13, 0xFF, 0x39, 0x0B,
			  0x04, 0x14, 0xF4, 0x01, 0x00,
			  0x00, 0x00, 0x00, 0x40, 0x01,
			  0x67, 0x67, 0x47, 0x10, 0x10,
			  0x34, 0x00, 0x00, 0x00, 0x00, 0x00} },	/* Touch Control Set */
	{0x29, 0XF5, 5, {0x00, 0x06, 0x00, 0x00, 0xC0} },
	{0x29, 0XC8, 2, {0x01, 0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_initialization_for_sleep_in_post_cut6[] = {
	{0x29, 0XB5, 27, {0x19, 0x04, 0x1C, 0x22, 0x2F, 0x80,
			  0x2F, 0x1F, 0x04, 0x40, 0x05, 0x05,
			  0x00, 0x20, 0x09, 0x04, 0x40, 0x05,
			  0x05, 0x00, 0xA5, 0x0D, 0x2F, 0x1F,
			  0x04, 0x00, 0x04, 0x00, 0x00} },

	{0x29, 0XF5, 5, {0x00, 0x06, 0x00, 0x00, 0xC0} },
	{0x29, 0XC8, 2, {0x01, 0x00} },
	{0x29, 0xB6, 2, {0x01, 0x03} },
	{0x15, 0xCF, 1, {0x0F} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

#if defined(CONFIG_LGE_READER_MODE)
static LCM_setting_table_V3 lcm_reader_mode_off[] = {
	{0x15, 0x89, 1, {0x00} },	/* reader mode disable */
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_reader_mode_low[] = {
	{0x29, 0x89, 13, {0x10, 0xFF, 0xFF, 0xE9, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xDE,
			  0x00, 0x00, 0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_reader_mode_mid[] = {
	{0x29, 0x89, 13, {0x10, 0xFF, 0xFF, 0xDE, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xDE,
			  0x00, 0x00, 0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_reader_mode_high[] = {
	{0x29, 0x89, 13, {0x10, 0xFF, 0xFF, 0xCE, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xDE,
			  0x00, 0x00, 0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};
#endif

static struct LCM_setting_table __attribute__ ((unused)) lcm_backlight_level_setting[] = {
	{
		0x51, 1, {
	0xFF} }, {
		REGFLAG_END_OF_TABLE, 0x00, {
	} }
};

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

	params->dsi.vertical_sync_active = 1;
	params->dsi.vertical_backporch = 11;
	params->dsi.vertical_frontporch = 580;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 120;
	params->dsi.horizontal_frontporch = 72;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 338;	/* 312; // 234; */
}

static void get_ver_info(void)
{
	char *p, *q;
	char vernum[10];

	LCM_PRINT("[LCD] get_ver_info!\n");

	if (db7400_cut_ver != 0xabab)
		return;

	p = strstr(saved_command_line, "lcmver=");
	if (p == NULL) {
		LCM_PRINT("[LCD] no lcmver in cmdline! set as cut5[default]\n");
		db7400_cut_ver = 5;
		return;
	}

	p += 7;
	if ((p - saved_command_line) > strlen(saved_command_line + 1))
		return;

	q = p;
	while (*q != ' ' && *q != '\0')
		q++;

	memset((void *)vernum, 0, sizeof(vernum));

	if (((int)(q - p)) == 1) {
		strncpy((char *)vernum, (const char *)p, (int)(q - p));
		vernum[q - p] = '\0';
	} else {
		LCM_PRINT("[LCD] lcmver parsing size worng! set as cut5[default]\n");
		db7400_cut_ver = 5;
		return;
	}

	db7400_cut_ver = vernum[0] - '0';

	LCM_PRINT("[LCD] db7400_cut_ver = %d\n", db7400_cut_ver);

}

static void init_lcm_registers(void)
{
#if 1
	switch (db7400_cut_ver) {
	case 5:
		dsi_set_cmdq_V3(lcm_initialization_setting_cut5_V3,
				sizeof(lcm_initialization_setting_cut5_V3) /
				sizeof(LCM_setting_table_V3), 1);
		break;

	case 6:
	case 7:
		dsi_set_cmdq_V3(lcm_initialization_setting_cut6_V3,
				sizeof(lcm_initialization_setting_cut6_V3) /
				sizeof(LCM_setting_table_V3), 1);
		break;

	default:
		LCM_PRINT("[LCD] error : LCM Driver IC version is unknown.\n");
		break;
	}

	LCM_PRINT("[LCD] db7400_cut_ver = %d\n", db7400_cut_ver);
#else
	dsi_set_cmdq_V3(lcm_initialization_setting_V3,
			sizeof(lcm_initialization_setting_V3) / sizeof(LCM_setting_table_V3), 1);
#endif
	MDELAY(120);
	dsi_set_cmdq_V3(lcm_initialization_setting_V3_Disp_On,
			sizeof(lcm_initialization_setting_V3_Disp_On) /
			sizeof(LCM_setting_table_V3), 1);
	LCM_PRINT("[LCD] init_lcm_registers\n");
}

static void init_lcm_registers_sleep(void)
{
	dsi_set_cmdq_V3(lcm_initialization_for_Disp_Off,
			sizeof(lcm_initialization_for_Disp_Off) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(30);
	dsi_set_cmdq_V3(lcm_initialization_for_Sleep_In,
			sizeof(lcm_initialization_for_Sleep_In) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(50);
	LCM_PRINT("[LCD] init_lcm_registers_sleep\n");
}

void init_lcm_registers_sleep_2nd(void)
{
#if 1
	if (db7400_cut_ver == 5) {
		dsi_set_cmdq_V3(lcm_initialization_for_sleep_in_post_cut5,
				sizeof(lcm_initialization_for_sleep_in_post_cut5) /
				sizeof(LCM_setting_table_V3), 1);
		MDELAY(90);
	} else if (db7400_cut_ver == 6) {
		dsi_set_cmdq_V3(lcm_initialization_for_sleep_in_post_cut6,
				sizeof(lcm_initialization_for_sleep_in_post_cut6) /
				sizeof(LCM_setting_table_V3), 1);
		MDELAY(110);
	} else if (db7400_cut_ver == 7) {
		dsi_set_cmdq_V3(lcm_initialization_for_sleep_in_post_cut6,
				sizeof(lcm_initialization_for_sleep_in_post_cut6) /
				sizeof(LCM_setting_table_V3), 1);
		MDELAY(110);
	} else
		LCM_PRINT("[LCD] error : db7400_cut_ver is unknown.\n");
#else
	dsi_set_cmdq_V3(lcm_initialization_for_sleep_in_post,
			sizeof(lcm_initialization_for_sleep_in_post) / sizeof(LCM_setting_table_V3),
			1);
#endif
	LCM_PRINT("[LCD] init_lcm_registers_sleep 2nd\n");
}

static void ctp_eint_pin_ctrl(int set)
{
	if (set == 0) {
		mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_DOWN);
		mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	} else if (set == 1) {
		mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
		mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
		mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	}
}

static void dsv_enp_en_ctrl(int set)
{
	if (set == 0)
		mt_set_gpio_out(GPIO_DSV_ENP_EN, GPIO_OUT_ZERO);
	else if (set == 1)
		mt_set_gpio_out(GPIO_DSV_ENP_EN, GPIO_OUT_ONE);
}

static void dsv_enn_en_ctrl(int set)
{
	if (set == 0)
		mt_set_gpio_out(GPIO_DSV_ENN_EN, GPIO_OUT_ZERO);
	else if (set == 1)
		mt_set_gpio_out(GPIO_DSV_ENN_EN, GPIO_OUT_ONE);
}

static void i2c0_sca_ctrl(int set)
{
	if (set == 0) {
		mt_set_gpio_mode(GPIO_I2C0_SCA_PIN, GPIO_I2C0_SCA_PIN_M_GPIO);
		mt_set_gpio_out(GPIO_I2C0_SCA_PIN, GPIO_OUT_ZERO);
	} else if (set == 1) {
		mt_set_gpio_mode(GPIO_I2C0_SCA_PIN, GPIO_I2C0_SCA_PIN_M_SCL);
	}
}

static void i2c0_sda_ctrl(int set)
{
	if (set == 0) {
		mt_set_gpio_mode(GPIO_I2C0_SDA_PIN, GPIO_I2C0_SDA_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_I2C0_SDA_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_select(GPIO_I2C0_SDA_PIN, GPIO_PULL_DOWN);
		mt_set_gpio_pull_enable(GPIO_I2C0_SDA_PIN, GPIO_PULL_ENABLE);
	} else if (set == 1) {
		mt_set_gpio_mode(GPIO_I2C0_SDA_PIN, GPIO_I2C0_SDA_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_I2C0_SDA_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_select(GPIO_I2C0_SDA_PIN, GPIO_PULL_UP);
		mt_set_gpio_pull_enable(GPIO_I2C0_SDA_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_mode(GPIO_I2C0_SDA_PIN, GPIO_I2C0_SDA_PIN_M_SDA);
	}
}

/* 1.8v LDO is always on */
static void ldo_1v8io_on(void)
{
#if 0
	mt6328_set_register_value(PMIC_RG_VRF18_1_VOSEL, 3);
	mt6328_set_register_value(PMIC_RG_VRF18_1_EN, 1);
#else
	/* hwPowerOn(MT6351_POWER_LDO_VGP3, VOL_1800, "LCD"); */
	pmic_set_register_value(PMIC_RG_VGP3_EN, 0x01);
#endif
}

/* 1.8v LDO is always on */
static void ldo_1v8io_off(void)
{
#if 0
	mt6328_set_register_value(PMIC_RG_VRF18_1_EN, 0);
#else
	/* hwPowerDown(MT6351_POWER_LDO_VGP3, "LCD"); */
	pmic_set_register_value(PMIC_RG_VGP3_EN, 0x00);
#endif
}

/* VGP1 3.0v LDO enable */
static void ldo_3v0_on(void)
{
	mt_set_gpio_mode(GPIO_TOUCH_EN, GPIO_TOUCH_RESET_M_GPIO);
	mt_set_gpio_dir(GPIO_TOUCH_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_TOUCH_EN, GPIO_OUT_ONE);
}

/* VGP1 3.0v LDO disable */
static void ldo_3v0_off(void)
{
	mt_set_gpio_mode(GPIO_TOUCH_EN, GPIO_TOUCH_RESET_M_GPIO);
	mt_set_gpio_dir(GPIO_TOUCH_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_TOUCH_EN, GPIO_OUT_ZERO);
}

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
		LCM_PRINT("[LCD] Reset High\n");
	} else {
		mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
		LCM_PRINT("[LCD] Reset Low\n");
	}
}

static void touch_reset_pin(int mode)
{
	mt_set_gpio_mode(GPIO_TOUCH_RESET, GPIO_TOUCH_RESET_M_GPIO);
	mt_set_gpio_dir(GPIO_TOUCH_RESET, GPIO_DIR_OUT);

	if (mode == 1)
		mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ONE);
	else if (mode == 0)
		mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ZERO);
}

static void lcm_init(void)
{
	ldo_1v8io_on();
	MDELAY(10);
	ldo_3v0_on();
	MDELAY(10);

	reset_lcd_module(1);
	MDELAY(5);
	reset_lcd_module(0);
	MDELAY(5);
	reset_lcd_module(1);
	MDELAY(50);

	touch_reset_pin(1);
	MDELAY(10);

	ldo_p5m5_dsv_5v5_on();
	MDELAY(10);

	init_lcm_registers();
	MDELAY(150);
	need_set_lcm_addr = 1;
	LCM_PRINT("[LCD] lcm_init\n");
}

static void lcm_suspend(void)
{
	init_lcm_registers_sleep();
	/* rt4832_dsv_toggle_ctrl(); */
	get_ver_info();

	i2c0_sca_ctrl(0);
	i2c0_sda_ctrl(0);

	dsv_enp_en_ctrl(0);
	dsv_enn_en_ctrl(0);

	ctp_eint_pin_ctrl(0);

	ldo_1v8io_off();
	ldo_3v0_off();
	ldo_p5m5_dsv_5v5_off();
	touch_reset_pin(0);
	reset_lcd_module(0);

	LCM_PRINT("[LCD] lcm_suspend\n");
}

static void lcm_resume_power(void)
{
	i2c0_sca_ctrl(1);
	i2c0_sda_ctrl(1);

	dsv_enp_en_ctrl(1);
	dsv_enn_en_ctrl(1);

	ctp_eint_pin_ctrl(1);

	ldo_1v8io_off();
	ldo_3v0_off();
	ldo_p5m5_dsv_5v5_off();
	touch_reset_pin(0);
	reset_lcd_module(0);
	MDELAY(20);

	LCM_PRINT("[LCD] lcm_resume_power\n");
}

static void lcm_resume(void)
{
	lcm_init();
	need_set_lcm_addr = 1;
	LCM_PRINT("[LCD] lcm_resume\n");
}

#if 0
static void lcm_esd_recover(void)
{
	lcm_suspend();
	lcm_resume();

	LCM_PRINT("[LCD] lcm_esd_recover\n");
}
#endif

/* --------------------------------------------------------------------------- */
/* Get LCM Driver Hooks */
/* --------------------------------------------------------------------------- */
LCM_DRIVER db7400_hd720_dsi_vdo_dongbu_drv = {
	.name = "db7400_hd720_dsi_vdo_dongbu",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.resume_power = lcm_resume_power,
	.suspend_2nd = init_lcm_registers_sleep_2nd,
};
