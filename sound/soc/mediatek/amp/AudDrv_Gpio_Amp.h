#ifndef _AUDDRV_EXT_AMP_GPIO_H_
#define _AUDDRV_EXT_AMP_GPIO_H_

#include <linux/module.h>
#include <linux/device.h>
#include <sound/soc.h>

enum spk_sw_gpio_type {
        PINCTRL_SPK_SW_DEFAULT = 0,
        PINCTRL_SPK_SW_OFF,
        PINCTRL_SPK_SW_ON,
        PINCTRL_SPK_NUM
};

struct audio_gpio_attr {
        const char *name;
        struct pinctrl_state *gpioctrl;
};

void Speaker_Switch_GPIO_On(struct pinctrl* pinctrl_spk_sw);

void Speaker_Switch_GPIO_Off(struct pinctrl* pinctrl_spk_sw);

void Speaker_Switch_GPIO_Default(struct pinctrl* pinctrl_spk_sw);

void Speaker_Switch_GPIO_Init(struct pinctrl** pinctrl_spk_sw, struct device* dev);


#endif


