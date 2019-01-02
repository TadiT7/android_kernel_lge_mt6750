/*
 *  lge_battery_id.c
 *
 *  LGE Battery Charger Interface Driver
 *
 *  Copyright (C) 2011 LG Electronics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <soc/mediatek/lge/lge_battery_id.h>

#define MODULE_NAME "lge_battery_id"

static uint battery_id_info = BATT_ID_UNKNOWN;

#ifdef CONFIG_LGE_PM_BATTERY_ID_EMBEDDED_ADC
int battery_id_att = 0;
#endif

struct lge_battery_id_info {
	struct device          *dev;
	uint                    batt_info_from_cmdl;
	bool                    enabled;
	struct power_supply     psy_batt_id;
};

static enum power_supply_property lge_battery_id_battery_props[] = {
	POWER_SUPPLY_PROP_BATTERY_ID,
};

struct battery_list_type battery_list_info[] = {
	{
		.battery_id_num = BATT_ID_UNKNOWN,
		.battery_id_type = "UNKNOWN",
	},
#ifdef CONFIG_LGE_PM_BATTERY_ID_EMBEDDED_ADC
	{
		.battery_id_num = BATT_ID_LGC,
		.battery_id_type = "LGC",
	},
	{
		.battery_id_num = BATT_ID_TOCAD,
		.battery_id_type = "TOCAD",
	},
	{
		.battery_id_num = BATT_ID_ATL,
		.battery_id_type = "ATL",
	},
	{
		.battery_id_num = BATT_ID_BYD,
		.battery_id_type = "BYD",
	},
	{
		.battery_id_num = BATT_ID_LISHEN,
		.battery_id_type = "LISHEN",
	},
#else
	{
		.battery_id_num = BATT_ID_DS2704_N,
		.battery_id_type = "DS2704_N",
	},
	{
		.battery_id_num = BATT_ID_DS2704_L,
		.battery_id_type = "DS2704_L",
	},
	{
		.battery_id_num = BATT_ID_DS2704_C,
		.battery_id_type = "DS2704_C",
	},
	{
		.battery_id_num = BATT_ID_ISL6296_N,
		.battery_id_type = "ISL6296_N",
	},
	{
		.battery_id_num = BATT_ID_ISL6296_L,
		.battery_id_type = "ISL6296_L",
	},
	{
		.battery_id_num = BATT_ID_ISL6296_C,
		.battery_id_type = "ISL6296_C",
	},
	{
		.battery_id_num = BATT_ID_ISL6296A_N,
		.battery_id_type = "ISL6296A_N",
	},
	{
		.battery_id_num = BATT_ID_ISL6296A_L,
		.battery_id_type = "ISL6296A_L",
	},
	{
		.battery_id_num = BATT_ID_ISL6296A_C,
		.battery_id_type = "ISL6296A_C",
	},
	{
		.battery_id_num = BATT_ID_RA4301_VC0,
		.battery_id_type = "RA4301_VC0",
	},
	{
		.battery_id_num = BATT_ID_RA4301_VC1,
		.battery_id_type = "RA4301_VC1",
	},
	{
		.battery_id_num = BATT_ID_RA4301_VC2,
		.battery_id_type = "RA4301_VC2",
	},
	{
		.battery_id_num = BATT_ID_SW3800_VC0,
		.battery_id_type = "SW3800_VC0",
	},
	{
		.battery_id_num = BATT_ID_SW3800_VC1,
		.battery_id_type = "SW3800_VC1",
	},
	{
		.battery_id_num = BATT_ID_SW3800_VC2,
		.battery_id_type = "SW3800_VC2",
	},
#endif
};

struct battery_id_type battery_id_list[] = {
#ifdef CONFIG_LGE_PM_BATTERY_ID_EMBEDDED_ADC
	{
		.battery_id = BATT_ID_LGC,
		.battery_cell_type = LGC,
		.battery_type_name = "LGE_LGC",
	},
	{
		.battery_id = BATT_ID_TOCAD,
		.battery_cell_type = TOCAD,
		.battery_type_name = "LGE_TOCAD",
	},
	{
		.battery_id = BATT_ID_ATL,
		.battery_cell_type = ATL,
		.battery_type_name = "LGE_ATL",
	},
	{
		.battery_id = BATT_ID_BYD,
		.battery_cell_type = BYD,
		.battery_type_name = "LGE_BYD",
	},
	{
		.battery_id = BATT_ID_LISHEN,
		.battery_cell_type = LISHEN,
		.battery_type_name = "LGE_LISHEN",
	},
#else
	{
		.battery_id = BATT_ID_RA4301_VC0,
		.battery_cell_type = TCD_AAC,
		.battery_type_name = "LGE_BL51YF_Tocad_3000mAh",
	},
	{
		.battery_id = BATT_ID_RA4301_VC1,
		.battery_cell_type = LGC_LLL,
		.battery_type_name = "LGE_BL51YF_LGC_3000mAh",
	},
	{
		.battery_id = BATT_ID_SW3800_VC0,
		.battery_cell_type = LGC_LLL,
		.battery_type_name = "LGE_BL51YF_LGC_3000mAh",
	},
	{
		.battery_id = BATT_ID_SW3800_VC1,
		.battery_cell_type = TCD_AAC,
		.battery_type_name = "LGE_BL51YF_Tocad_3000mAh",
	},
	{
		.battery_id = BATT_ID_DS2704_L,
		.battery_cell_type = LGC_LLL,
		.battery_type_name = "itech_3000mah",
	},
	{
		.battery_id = BATT_ID_DS2704_C,
		.battery_cell_type = TCD_AAC,
		.battery_type_name = "itech_3000mah",
	},
	{
		.battery_id = BATT_ID_ISL6296_L,
		.battery_cell_type = TCD_AAC,
		.battery_type_name = "itech_3000mah",
	},
	{
		.battery_id = BATT_ID_ISL6296_C,
		.battery_cell_type = LGC_LLL,
		.battery_type_name = "itech_3000mah",
	},
#endif
};

static int __init battery_id_setup(char *batt_id)
{
        int i;

        for (i = 0; i < BATT_ID_LIST_MAX; i++) {
                if (strcmp(batt_id, battery_list_info[i].battery_id_type) == 0)
                        break;
        }
        if (i < BATT_ID_LIST_MAX)
                battery_id_info = battery_list_info[i].battery_id_num;

        return 0;
}

static bool is_battery_valid(uint batt_id)
{
#ifdef CONFIG_LGE_PM_BATTERY_ID_EMBEDDED_ADC
		return true;
#else
	if(batt_id == BATT_ID_DS2704_N || batt_id == BATT_ID_DS2704_L ||
		batt_id == BATT_ID_DS2704_C || batt_id == BATT_ID_ISL6296_N ||
		batt_id == BATT_ID_ISL6296_L || batt_id == BATT_ID_ISL6296_C ||
		batt_id == BATT_ID_RA4301_VC0 || batt_id == BATT_ID_RA4301_VC1 ||
		batt_id == BATT_ID_RA4301_VC2 || batt_id == BATT_ID_SW3800_VC0 ||
		batt_id == BATT_ID_SW3800_VC1 || batt_id == BATT_ID_SW3800_VC2 ||
		batt_id == BATT_ID_ISL6296A_N || batt_id == BATT_ID_ISL6296A_L ||
		batt_id == BATT_ID_ISL6296A_C)
		return true;
	else
		return false;
#endif
}

bool lge_battery_check(void)
{
	struct power_supply *psy;
	union power_supply_propval prop = {0,};
	uint battery_id;

	psy = power_supply_get_by_name("battery_id");
	if(psy) {
		psy->get_property(psy, POWER_SUPPLY_PROP_BATTERY_ID, &prop);
		battery_id = prop.intval;
	} else {
		pr_info("battery_id not found. use default battey \n");
		battery_id = BATT_ID_DEFAULT;
	}

	return is_battery_valid(battery_id);
}

#ifdef CONFIG_LGE_PM_BATTERY_ID_EMBEDDED_ADC
int lge_get_battery_id(void)
{
	pr_info("[LGE] Battery ID for Hidden Menu : %d\n", battery_id_att);
	return battery_id_att;
}
#endif

/*
 * TBD : This function should be more intelligent.
 * Should directly access battery id circuit via 1-Wired.
 */
static int lge_battery_id_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct lge_battery_id_info *info = container_of(psy,
			struct lge_battery_id_info,
			psy_batt_id);

	switch (psp) {
	case POWER_SUPPLY_PROP_BATTERY_ID:
		val->intval = info->batt_info_from_cmdl;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct of_device_id lge_battery_id_match_table[] = {
	{ .compatible = "lge,battery-id" },
	{}
};

static int lge_battery_id_dt_to_pdata(struct platform_device *pdev,
					struct lge_battery_id_info *pdata)
{
	struct device_node *node = pdev->dev.of_node;

	pdata->enabled = of_property_read_bool(node,
					"lge,restrict-mode-enabled");

	return 0;
}

static int lge_battery_id_probe(struct platform_device *pdev)
{
	struct lge_battery_id_info *info;
	/*struct power_supply *psy;*/
	int ret = 0;

	dev_info(&pdev->dev, "LGE Battery ID Checker started\n");

	info = kzalloc(sizeof(struct lge_battery_id_info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "memory error\n");
		return -ENOMEM;
	}

	ret = lge_battery_id_dt_to_pdata(pdev, info);
	if (ret)
		return -EIO;

	platform_set_drvdata(pdev, info);

	info->dev = &pdev->dev;
	info->batt_info_from_cmdl = battery_id_info;

	info->psy_batt_id.name		= "battery_id";
	info->psy_batt_id.type		= POWER_SUPPLY_TYPE_BATTERY;
	info->psy_batt_id.get_property	= lge_battery_id_get_property;
	info->psy_batt_id.properties	= lge_battery_id_battery_props;
	info->psy_batt_id.num_properties =
		ARRAY_SIZE(lge_battery_id_battery_props);

#ifdef CONFIG_LGE_PM_BATTERY_ID_EMBEDDED_ADC
	battery_id_att = info->batt_info_from_cmdl;
#endif

	ret = power_supply_register(&pdev->dev, &info->psy_batt_id);
	if (ret) {
		dev_err(&pdev->dev, "failed: power supply register\n");
		goto err_register;
	}

	return ret;
err_register:
	kfree(info);
	return ret;
}

static int lge_battery_id_remove(struct platform_device *pdev)
{
	struct lge_battery_id_info *info = platform_get_drvdata(pdev);

	power_supply_unregister(&info->psy_batt_id);
	kfree(info);

	return 0;
}

#if defined(CONFIG_PM)
static int lge_battery_id_suspend(struct device *dev)
{
	/*struct lge_battery_id_info *info = dev_get_drvdata(dev);*/

	return 0;
}

static int lge_battery_id_resume(struct device *dev)
{
	/*struct lge_battery_id_info *info = dev_get_drvdata(dev);*/

	return 0;
}

static const struct dev_pm_ops lge_battery_id_pm_ops = {
	.suspend	= lge_battery_id_suspend,
	.resume		= lge_battery_id_resume,
};
#endif

static struct platform_driver lge_battery_id_driver = {
	.driver = {
		.name   = MODULE_NAME,
		.owner  = THIS_MODULE,
#if defined(CONFIG_PM)
		.pm     = &lge_battery_id_pm_ops,
#endif
		.of_match_table = lge_battery_id_match_table,
	},
	.probe  = lge_battery_id_probe,
	.remove = lge_battery_id_remove,
};

static int __init lge_battery_id_init(void)
{
	__setup("lge.battid=", battery_id_setup);
	return platform_driver_register(&lge_battery_id_driver);
}

static void __exit lge_battery_id_exit(void)
{
	platform_driver_unregister(&lge_battery_id_driver);
}

module_init(lge_battery_id_init);
module_exit(lge_battery_id_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Cowboy");
MODULE_DESCRIPTION("LGE Battery ID Checker");
