#ifndef __RT4832_H__
#define __RT4832_H__

void rt4832_dsv_ctrl(int enable);
void rt4832_dsv_toggle_ctrl(void);
void rt4832_dsv_mode_change(int mode);
unsigned int get_cur_main_lcd_level(void);
extern int old_bl_level;
#endif
