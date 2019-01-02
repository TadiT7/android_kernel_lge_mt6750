#ifndef __RT4832_H__
#define __RT4832_H__

void rt4832_dsv_ctrl(int enable);
void rt4832_dsv_toggle_ctrl(void);
extern unsigned int get_cur_main_lcd_level(void);

#endif
