#ifndef __CUST_VIBRATOR_H__
#define __CUST_VIBRATOR_H__




#define CUST_VIBR_LIMIT
#define CUST_VIBR_VOL
/*----------------------------------------------------------------------------*/
struct vibrator_hw {
	int	vib_timer;
#ifdef CUST_VIBR_LIMIT
	int	vib_limit;
#endif
#ifdef CUST_VIBR_VOL
	int	vib_vol;
#endif
};
/*----------------------------------------------------------------------------*/
extern struct vibrator_hw *get_cust_vibrator_hw(void);
/*----------------------------------------------------------------------------*/
#endif
