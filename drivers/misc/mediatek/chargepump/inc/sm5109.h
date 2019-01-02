#ifndef _LCD_BIAS_H
#define _LCM_BIAS_H

#define LCD_BIAS_VPOS_ADDR    0x00
#define LCD_BIAS_VNEG_ADDR    0x01
#define LCD_BIAS_APPS_ADDR    0x03

#ifdef BUILD_LK
#define LCD_BIAS_I2C_BUSNUM   2	/* for I2C channel 0 */
#define LCD_BIAS_I2C_ADDR       0x3E /*for I2C slave dev addr*/

#define LCD_BIAS_ST_MODE         0
#define LCD_BIAS_MAX_ST_MODE_SPEED 100  /* khz */

#define LCD_BIAS_PRINT printf

#else

#define LCD_BIAS_PRINT printk

#define SM5109_I2C_ID_NAME "I2C_LCD_BIAS"

#endif
#endif /* _LCM_BIAS_H */