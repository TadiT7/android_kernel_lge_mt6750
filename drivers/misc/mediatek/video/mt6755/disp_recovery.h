/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/
#ifndef _DISP_RECOVERY_H_
#define _DISP_RECOVERY_H_

#define GPIO_EINT_MODE	0
#define GPIO_DSI_MODE	1

/* defined in mtkfb.c should move to mtkfb.h*/
extern unsigned int islcmconnected;
extern unsigned int mmsys_enable;

void primary_display_check_recovery_init(void);
void primary_display_esd_check_enable(int enable);
unsigned int need_wait_esd_eof(void);

#if defined(CONFIG_TCL_INCELL_FT8006M_HD_CV1) || defined(CONFIG_TOVIS_INCELL_TD4100_HD_LV7_SPR) || defined(CONFIG_TOVIS_INCELL_LG4894_HD_CV3)
void mtkfb_esd_recovery(void);
void LG_ESD_recovery(void);
#endif

#endif
