#ifndef __RT4832_H__
#define __RT4832_H__

#if defined(BUILD_LK)
void chargepump_DSV_on(void);
void chargepump_DSV_off(void);
#endif
void dw8767_dsv_ctrl(int enable);

#endif
