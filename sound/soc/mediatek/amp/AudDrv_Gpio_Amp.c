#include "AudDrv_Gpio_Amp.h"
#include <sound/soc.h>

#define AMP_EN_PRINTK(fmt, arg...) \
    do { \
        printk("[AMP]: %s() "fmt"\n", __func__,##arg); \
    }while (0)

struct audio_gpio_attr spk_sw_gpios[PINCTRL_SPK_NUM] = {
        [PINCTRL_SPK_SW_DEFAULT] = {"aud_spk_sw_default", NULL},
        [PINCTRL_SPK_SW_OFF] = {"aud_spk_sw_off", NULL},
        [PINCTRL_SPK_SW_ON] = {"aud_spk_sw_on", NULL},
};

void Speaker_Switch_GPIO_On(struct pinctrl* pinctrl_spk_sw){
        AMP_EN_PRINTK("SPK SWT ON!!");
        pinctrl_select_state(pinctrl_spk_sw, spk_sw_gpios[PINCTRL_SPK_SW_ON].gpioctrl);
}
EXPORT_SYMBOL(Speaker_Switch_GPIO_On);

void Speaker_Switch_GPIO_Off(struct pinctrl* pinctrl_spk_sw){
        AMP_EN_PRINTK("SPK SWT OFF!!");
        pinctrl_select_state(pinctrl_spk_sw, spk_sw_gpios[PINCTRL_SPK_SW_OFF].gpioctrl);
}
EXPORT_SYMBOL(Speaker_Switch_GPIO_Off);

void Speaker_Switch_GPIO_Default(struct pinctrl* pinctrl_spk_sw){
        AMP_EN_PRINTK("SPK SWT Default!!");
        pinctrl_select_state(pinctrl_spk_sw, spk_sw_gpios[PINCTRL_SPK_SW_DEFAULT].gpioctrl);
}
EXPORT_SYMBOL(Speaker_Switch_GPIO_Default);

void Speaker_Switch_GPIO_Init(struct pinctrl** pinctrl_spk_sw, struct device* dev)
{
        int i;
        *pinctrl_spk_sw = devm_pinctrl_get(dev);
        AMP_EN_PRINTK("SPK SWT Init!!");
        if (IS_ERR(pinctrl_spk_sw)) {
                pr_err("%s: Cannot find pinctrl_spk_sw!\n", __func__);
        }
        for(i = 0; i < ARRAY_SIZE(spk_sw_gpios); i++){
                spk_sw_gpios[i].gpioctrl = pinctrl_lookup_state(*pinctrl_spk_sw,
                                spk_sw_gpios[i].name);
                if(IS_ERR(spk_sw_gpios[i].gpioctrl)){spk_sw_gpios[i].gpioctrl = NULL;
                        pr_err("%s: pinctrl_lookup_state %s fail %ld\n",
                                        __func__, spk_sw_gpios[i].name,
                                        PTR_ERR(spk_sw_gpios[i].gpioctrl));
                }
        }
};

EXPORT_SYMBOL(Speaker_Switch_GPIO_Init);



