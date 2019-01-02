#ifndef __LM3632_H__
#define __LM3632_H__

void lm3632_dsv_ctrl(int enable);
#if defined(CONFIG_LGD_INCELL_LG4894_HD_SF3)
extern int old_bl_level;
#endif
void lm3632_dsv_toggle_ctrl(void);
void lm3632_dsv_mode_change(int mode);

#endif
