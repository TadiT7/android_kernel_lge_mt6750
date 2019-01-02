#ifndef	__SOC_MTK_BOARD_LGE_H
#define	__SOC_MTK_BOARD_LGE_H

typedef enum {
	HW_REV_0	= 0,
	HW_REV_0_1,
	HW_REV_A,
	HW_REV_B,
	HW_REV_C,
#if defined(TARGET_MT6750_CV150) || defined(TARGET_MT6750_CV3)
	HW_REV_D,
	HW_REV_E,
#endif
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_MAX,
} hw_rev_type;

#ifdef CONFIG_LGE_DRAM_WR_BIAS_OFFSET
int lge_get_wr_bias_offset(void);
#endif

hw_rev_type lge_get_board_revno(void);
int lge_get_mtk_smpl(void);
#if defined CONFIG_LGD_INCELL_DB7400_HD_PH1
extern int lge_get_db7400_cut_ver(void);
#endif
#ifdef CONFIG_MTK_AAL_SUPPORT
extern int get_aal_support(void);
#endif

#if defined(CONFIG_PRE_SELF_DIAGNOSIS)
int lge_pre_self_diagnosis(char *drv_bus_code, int func_code, char *dev_code, char *drv_code, int errno);
int lge_pre_self_diagnosis_pass(char *dev_code);
#endif

#if defined(CONFIG_PRE_SELF_DIAGNOSIS)
int lge_pre_self_diagnosis(char *drv_bus_code, int func_code, char *dev_code, char *drv_code, int errno);

struct pre_selfd_platform_data {
    int (*set_values) (int r, int g, int b);
    int (*get_values) (int *r, int *g, int *b);
};
#endif

enum lge_laf_mode_type {
    LGE_LAF_MODE_NORMAL = 0,
    LGE_LAF_MODE_LAF,
};
enum lge_laf_mode_type lge_get_laf_mode(void);
int lge_get_bootreason(void);

char *lge_get_model_name(void);
#endif	/* __SOC_MTK_BOARD_LGE_H */
