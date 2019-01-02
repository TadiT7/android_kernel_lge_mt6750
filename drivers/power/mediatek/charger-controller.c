/* Copyright (C) 2017 LG Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt) "[CC] %s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/async.h>
#include <linux/wakelock.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#ifdef CONFIG_LGE_PM_USB_ID
#include <soc/mediatek/lge/lge_cable_id.h>
#include <soc/mediatek/lge/board_lge.h>
#endif
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
#include <soc/mediatek/lge/lge_battery_id.h>
#endif
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
#include <soc/mediatek/lge/lge_pseudo_batt.h>
#endif
#ifdef CONFIG_LGE_BOOT_MODE
#include <soc/mediatek/lge/lge_boot_mode.h>
#endif

#include <linux/power/charger-controller.h>

#define DEFAULT_BATT_CAPACITY		50
#define DEFAULT_BATT_VOLTAGE		4100
#define DEFAULT_BATT_TEMPERATURE	250

#define VOLTAGE_DIVIDE_UNIT		1000
#define CURRENT_DIVIDE_UNIT		1000
#define TEMP_DIVIDE_UNIT		10

#define DISABLE_FEATURES_CC
enum print_reason {
	PR_DEBUG        = BIT(0),
	PR_INFO         = BIT(1),
	PR_ERR          = BIT(2),
};

static unsigned int total_charging_time = 0;

static int cc_debug_mask = PR_INFO|PR_ERR|PR_DEBUG;
module_param_named(
	debug_mask, cc_debug_mask, int, S_IRUSR | S_IWUSR
);

#define pr_cc(reason, fmt, ...) \
	do { \
		if (cc_debug_mask & (reason)) \
			pr_info(fmt, ##__VA_ARGS__); \
		else \
			pr_debug(fmt, ##__VA_ARGS__); \
	} while (0)

enum {
	CC_BATT_TEMP_STATE_COLD,
	CC_BATT_TEMP_STATE_COOL,
	CC_BATT_TEMP_STATE_NORMAL,
	CC_BATT_TEMP_STATE_HIGH,
	CC_BATT_TEMP_STATE_OVERHEAT,
};

enum {
	STATUS_NONE = 0,
	STATUS_DISCON,
	STATUS_CONNECT,
};

const static char *batt_temp_state_str[] = {
	"Cold",
	"Cool",
	"Normal",
	"High",
	"Overheat",
};

enum {
	CC_BATT_VOLT_UNDER_4_0 = 0,
	CC_BATT_VOLT_OVER_4_0,
	CC_BATT_VOLT_DONTCARE,
};

#define CC_CHARGE_LIMIT_NAME_MAX 20
enum {
	CC_USBIN,		/* TA / USB */
	CC_WLCIN,		/* Wireless */
	CC_CHG_MAX,
	CC_CHG_ALL = 0xFF,
};

enum {
	CC_VOTER_PSY,		/* physical */
	CC_VOTER_USER,		/* userspace */
	CC_VOTER_SPEC,		/* battery specification */
	CC_VOTER_OTP,		/* battery otp */
	CC_VOTER_THERMAL,	/* thermal */
	CC_VOTER_BATTERY,	/* battery id */
	CC_VOTER_FB,		/* lcd */
	CC_VOTER_LLK,		/* store demo mode */
	CC_VOTER_SAFETY,	/* safety timer */
	CC_VOTER_CALL,		/* call */
	CC_VOTER_MAX,
};

struct chgctrl_limit_vote {
	int value;
	int active;
};

struct chgctrl_limit {
	char name[CC_CHARGE_LIMIT_NAME_MAX];
	struct mutex lock;

	struct chgctrl_limit_vote table[CC_VOTER_MAX];

	int active;

	int value;
	int voter;

	int (*cb)(void *);
};

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_SPEC
struct chgctrl_spec {
	/* temperature range */
	int tmin;
	int tmax;

	/* charge limit */
	int volt;
	int curr;
};
#endif

struct chgctrl {
	struct device *dev;
	struct power_supply psy;

	struct work_struct		notify_work;
	struct delayed_work		battery_health_work;

#ifdef CONFIG_LGE_PM_CHARGE_INFO
	struct delayed_work		charging_inform_work;
	struct wake_lock		information_lock;
#endif

	int psy_icl;
	int psy_fcc;
	int psy_vfloat;
	int psy_technology;

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_OTP
	struct delayed_work		otp_charging_work;
	int				otp_state_changed;

	/* values from device-tree */
	bool otp_v2;
	bool otp_v1_9;
	bool otp_for_sprint;
	int otp_fcc;
	int otp_vfloat;
	bool otp_ctrl_icl;
#endif

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_THERMAL
	/* values from device-tree */
	int thermal_fcc;
	bool thermal_ctrl_icl;
#endif

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_FB
	struct notifier_block fb_notify;
	int fb_blank;

	/* values from device-tree */
	int fb_fcc;
	bool fb_ctrl_icl;
#endif

#ifdef CONFIG_LGE_PM_LLK_MODE
	int llk_soc_max;
	int llk_soc_min;
#endif

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_CALL
	/* values from device-tree */
	int call_fcc;
	bool call_ctrl_icl;
	int call_chg_mode;
#endif

	struct chgctrl_limit icl[CC_CHG_MAX];
	struct chgctrl_limit fcc[CC_CHG_MAX];
	struct chgctrl_limit vfloat[CC_CHG_MAX];
	int present[CC_CHG_MAX];
	int online[CC_CHG_MAX];

	struct wake_lock chg_wake_lock;

	int status;

	int batt_temp_state;
	int batt_volt_state;

#ifdef CONFIG_LGE_PM_VZW_REQ
	int vzw_chg_mode;
#endif
#ifdef CONFIG_LGE_PM_LLK_MODE
	int store_demo_enabled;
#endif
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	unsigned int usb_current_max_enabled;
	int usb_current_max;
#endif
#ifdef CONFIG_LGE_PM_CHARGE_SAFETY_TIMER
	int safety_time;
	bool safety_timer_expired;
#endif
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_SPEC
	struct delayed_work spec_work;

	struct chgctrl_spec *spec;
	int spec_size;

	int spec_idx;
#endif
};

/* charger controller : internal functions */
static char *power_supply_type_str[] = {
	"Unknown", "Battery", "UPS", "Mains", "USB",
	"USB_DCP", "USB_CDP", "USB_ACA", "Wireless"
};

static char *usb_charger_source[] = {
	"usb",		/* for Qualcomm, MediaTek */
	"ac",		/* for MediaTek */
};

static char *wlc_charger_source[] = {
	"dc",		/* for Qualcomm */
	"wireless",	/* for MediaTek */
	"ext",		/* for Appsport */
};

static char *get_usb_type(void)
{
	struct power_supply *psy = NULL;
	union power_supply_propval val = {0, };
	int i;
	int rc;

	for (i = 0; i < ARRAY_SIZE(usb_charger_source); i++) {
		psy = power_supply_get_by_name(usb_charger_source[i]);
		if (!psy || !psy->get_property)
			continue;

		psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if (val.intval)
			break;

		psy = NULL;
	}

	if (!psy || !psy->get_property)
		return "None";

	rc = psy->get_property(psy, POWER_SUPPLY_PROP_FASTCHG, &val);
	if (!rc && val.intval)
		return "PEP";

	return power_supply_type_str[psy->type];
}

static int is_usb_online(void)
{
	struct power_supply *psy = NULL;
	union power_supply_propval val = {0, };
	int i;

	for (i = 0; i < ARRAY_SIZE(usb_charger_source); i++) {
		psy = power_supply_get_by_name(usb_charger_source[i]);
		if (!psy || !psy->get_property)
			continue;

		psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if (val.intval)
			break;

		psy = NULL;
	}

	if (!psy)
		return 0;

	return 1;
}

static bool is_usb_present(void)
{
	struct power_supply *psy = NULL;
	union power_supply_propval val = {0, };
	int i;

	for (i = 0; i < ARRAY_SIZE(usb_charger_source); i++) {
		psy = power_supply_get_by_name(usb_charger_source[i]);
		if (!psy || !psy->get_property)
			continue;

		psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT, &val);
		if (val.intval)
			break;

		psy = NULL;
	}

	if (!psy)
		return 0;

	return 1;
}

static int is_wireless_online(void)
{
	struct power_supply *psy = NULL;
	union power_supply_propval val = {0, };
	int i;

	for (i = 0; i < ARRAY_SIZE(wlc_charger_source); i++) {
		psy = power_supply_get_by_name(wlc_charger_source[i]);
		if (!psy || !psy->get_property)
			continue;

		psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if (val.intval)
			break;

		psy = NULL;
	}

	if (!psy)
		return 0;

	return 1;

}

static bool is_wireless_present(void)
{
	struct power_supply *psy = NULL;
	union power_supply_propval val = {0, };
	int i;

	for (i = 0; i < ARRAY_SIZE(wlc_charger_source); i++) {
		psy = power_supply_get_by_name(wlc_charger_source[i]);
		if (!psy || !psy->get_property)
			continue;

		psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT, &val);
		if (val.intval)
			break;

		psy = NULL;
	}

	if (!psy)
		return 0;

	return 1;
}

/* charger controller : voters */
static char *chgctrl_charger_str[] = {
	"usb",
	"wlc",
};

static char *chgctrl_voter_str[] = {
	"psy",
	"user",
	"spec",
	"otp",
	"thermal",
	"battery",
	"fb",
	"llk",
	"safety",
	"call",
};

static int chgctrl_update_limit(struct chgctrl_limit *limit)
{
	struct chgctrl_limit_vote *table = limit->table;
	int value = INT_MAX;
	int voter = CC_VOTER_MAX;
	int updated = 0;

	int i;

	mutex_lock(&limit->lock);

	for (i = 0; i < CC_VOTER_MAX; i++) {
		if (!table[i].active)
			continue;

		if (table[i].value < value) {
			value = table[i].value;
			voter = i;
		}
	}

	if (value != limit->value) {
		limit->value = value;
		limit->voter = voter;
		limit->active = ((voter == CC_VOTER_MAX) ? 0 : 1);
		updated = 1;

		if (limit->active) {
			pr_cc(PR_INFO, "%s limit %d by %s\n",
					limit->name, limit->value,
					chgctrl_voter_str[limit->voter]);
		} else {
			pr_cc(PR_INFO, "%s limit clear\n", limit->name);
		}
	}

	mutex_unlock(&limit->lock);

	return updated;
}
static int chgctrl_set_limit(struct chgctrl_limit* limit_table,
			     int chg, int voter, int value, int en)
{
	struct chgctrl_limit *limit = &limit_table[chg];
	bool changed = false;
	int rc;

	if (chg >= CC_CHG_MAX)
		return 0;
	if (voter >= CC_VOTER_MAX)
		return 0;

	if (value < 0)
		en = 0;

	mutex_lock(&limit->lock);

	if (limit->table[voter].value != value)
		changed = true;
	if (limit->table[voter].active != en)
		changed = true;

	/* nothing changed. return here */
	if (!changed) {
		mutex_unlock(&limit->lock);
		return 0;
	}

	limit->table[voter].value = value;
	limit->table[voter].active = en;

	if (en) {
		pr_cc(PR_DEBUG, "%s vote limit %d to %s\n",
				chgctrl_voter_str[voter],
				value, limit->name);
	} else {
		pr_cc(PR_DEBUG, "%s unvote limit to %s\n",
				chgctrl_voter_str[voter],
				limit->name);
	}

	mutex_unlock(&limit->lock);

	rc = chgctrl_update_limit(limit);

	if (rc && limit->cb)
		rc = limit->cb(limit->name);

	return rc;
}

static int chgctrl_input_current_limit(struct chgctrl *chgctrl,
				       int chg, int voter, int ma, int en)
{
	int rc = 0;

	if (chg != CC_CHG_ALL)
		return chgctrl_set_limit(chgctrl->icl, chg, voter, ma, en);

	for (chg = 0; chg < CC_CHG_MAX; chg++) {
		rc = chgctrl_set_limit(chgctrl->icl, chg, voter, ma, en);
	}

	return rc;
}

static int chgctrl_battery_current_limit(struct chgctrl *chgctrl,
					 int chg, int voter, int ma, int en)
{
	int rc = 0;

	if (chg != CC_CHG_ALL)
		return chgctrl_set_limit(chgctrl->fcc, chg, voter, ma, en);

	for (chg = 0; chg < CC_CHG_MAX; chg++) {
		rc = chgctrl_set_limit(chgctrl->fcc, chg, voter, ma, en);
	}

	return rc;
}

static int chgctrl_battery_voltage_limit(struct chgctrl *chgctrl,
					 int chg, int voter, int mv, int en)
{
	int rc = 0;

	if (chg != CC_CHG_ALL)
		return chgctrl_set_limit(chgctrl->vfloat, chg, voter, mv, en);

	for (chg = 0; chg < CC_CHG_MAX; chg++) {
		rc = chgctrl_set_limit(chgctrl->vfloat, chg, voter, mv, en);
	}

	return rc;

}

static void chgctrl_init_charge_limit(struct chgctrl *chgctrl,
				      char *name, struct chgctrl_limit *limit,
				      int (*cb)(void *))
{
	int i;

	for (i = 0; i < CC_CHG_MAX; i++) {
		snprintf(limit[i].name, CC_CHARGE_LIMIT_NAME_MAX, "%s_%s",
				chgctrl_charger_str[i], name);

		mutex_init(&limit[i].lock);

		limit[i].active = 0;

		limit[i].value = 0;
		limit[i].voter = 0;

		limit[i].cb = cb;
	}
}

static int chgctrl_is_charger_online(struct chgctrl *chgctrl)
{
	int i;

	for (i = 0; i < CC_CHG_MAX; i++) {
		if (chgctrl->online[i])
			return 1;
	}

	return 0;
}

static int chgctrl_is_charger_present(struct chgctrl *chgctrl)
{
	int i;

	for (i = 0; i < CC_CHG_MAX; i++) {
		if (chgctrl->present[i])
			return 1;
	}

	return 0;
}

static int chgctrl_get_effective_limit(struct chgctrl *chgctrl,
					       struct chgctrl_limit *limit)
{
	int charger;

	/* find effective charger (present) */
	for (charger = 0; charger < CC_CHG_MAX; charger++) {
		if (chgctrl->present[charger])
			break;
	}

	if (charger != CC_CHG_MAX)
		goto charger_found;

	/* find effective charger (online) */
	for (charger = 0; charger < CC_CHG_MAX; charger++) {
		if (chgctrl->online[charger])
			break;
	}

	/* use usbin as default */
	if (charger == CC_CHG_MAX)
		charger = CC_USBIN;

charger_found:
	if (!limit[charger].active)
		return -EINVAL;

	return limit[charger].value;
}

/* charger controller : get properties */
static struct power_supply* chgctrl_get_power_supply(void)
{
	static struct power_supply *psy;

	if (!psy)
		psy = power_supply_get_by_name("charger_controller");

	if (!psy || !psy->get_property)
		return NULL;

	return psy;
}

static int chgctrl_get_property_from_batt(enum power_supply_property psp,
				     union power_supply_propval *val)
{
	static struct power_supply *psy = NULL;

	if (!psy)
		psy = power_supply_get_by_name("battery");

	if (!psy || !psy->get_property)
		return -ENODEV;

	return psy->get_property(psy, psp, val);
}

static int chgctrl_get_property_from_usb(enum power_supply_property psp,
				    union power_supply_propval *val)
{
	static struct power_supply *psy = NULL;

	if (!psy)
		psy = power_supply_get_by_name("usb");

	if (!psy || !psy->get_property)
		return -ENODEV;

	return psy->get_property(psy, psp, val);
}

static int chgctrl_get_usb_adc(void)
{
	union power_supply_propval val = {0, };

	chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_ChargerVoltage, &val);

	return val.intval;
}

static int chgctrl_get_battery_temperature(void)
{
	union power_supply_propval val = {0,};
	int rc;

	rc = chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_TEMP, &val);
	if (rc)
		return DEFAULT_BATT_TEMPERATURE;

	return val.intval;
}

static int chgctrl_get_battery_capacity(void)
{
	union power_supply_propval val = {0, };
	int rc;

	rc = chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_CAPACITY, &val);
	if (rc)
		return DEFAULT_BATT_CAPACITY;

	return val.intval;
}

static int chgctrl_get_safety_timer_enable(void)
{
	union power_supply_propval val = {0, };
	int rc;

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	/* force disable safety timer when pseudo battey enabled */
	if (get_pseudo_batt_info(PSEUDO_BATT_MODE))
		return 0;
#endif

	rc = chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE, &val);
	if (rc)
		return 1;

	return val.intval;
}

static struct chgctrl* chgctrl_get_chgctrl(void)
{
	struct power_supply *psy = chgctrl_get_power_supply();

	if (!psy)
		return NULL;

	return container_of(psy, struct chgctrl, psy);
}

static int chgctrl_get_charging_step(struct chgctrl *chgctrl)
{
	int batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	int charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	batt_status = chgctrl->status;
	if (batt_status < 0)
		return charge_type;

	if (batt_status == POWER_SUPPLY_STATUS_FULL)
		charge_type = POWER_SUPPLY_CHARGE_TYPE_EOC;
	else if (batt_status == POWER_SUPPLY_STATUS_CHARGING)
		charge_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
	else
		charge_type = POWER_SUPPLY_CHARGE_TYPE_DISCHG;

	return charge_type;
}

/* charger controller : information */
#ifdef CONFIG_LGE_PM_CHARGE_INFO
static int cc_info_time = 60000;
module_param_named(
	cc_info_time, cc_info_time, int, S_IRUSR | S_IWUSR
);

#ifdef CONFIG_LGE_PM_USB_ID
static char *cable_type_str[] = {
	"NOT INIT", "0", "0", "0", "0", "0",
	"56K", "130K", "OPEN", "OPEN2", "NOT INIT",
	"910K", "NOT INIT"
};
#endif

static void charging_information(struct work_struct *work)
{
	struct chgctrl *chgctrl = container_of(work,
			struct chgctrl, charging_inform_work.work);
	union power_supply_propval val = {0, };
	int rc = 0;
	bool usb_present = is_usb_present();
	bool wireless_present = is_wireless_present();
	char *usb_type_name = get_usb_type();
	char *cable_type_name = "N/A";
	char *batt_mode = "Normal";
	int usbin_vol = chgctrl_get_usb_adc();
	int batt_soc = 0;
	int batt_vol = 0;
	int main_iusb_set = 0;
	int main_ibat_set = 0;
	int total_ibat_now = 0;
	int batt_temp = 0;
	int charge_type = 0;

	wake_lock(&chgctrl->information_lock);

	rc = chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_CAPACITY, &val);
	if (rc)
		goto reschedule;
	batt_soc = val.intval;

	rc = chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (rc)
		goto reschedule;
	batt_vol = div_s64(val.intval, VOLTAGE_DIVIDE_UNIT);

	rc = chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_TEMP, &val);
	if (rc)
		goto reschedule;
	batt_temp = div_s64(val.intval, TEMP_DIVIDE_UNIT);

	rc = chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (rc)
		goto reschedule;
	total_ibat_now = div_s64(val.intval, CURRENT_DIVIDE_UNIT);

	rc = chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &val);
	if (rc)
		goto reschedule;
	main_ibat_set = div_s64(val.intval, CURRENT_DIVIDE_UNIT);

	rc = chgctrl_get_property_from_usb(POWER_SUPPLY_PROP_CURRENT_MAX, &val);
	if (rc)
		goto reschedule;
	main_iusb_set = div_s64(val.intval, CURRENT_DIVIDE_UNIT);

	charge_type = chgctrl_get_charging_step(chgctrl);

	if (usb_present && (charge_type == POWER_SUPPLY_CHARGE_TYPE_FAST))
		total_charging_time += (cc_info_time/60000);
	else
		total_charging_time = 0;

#ifdef CONFIG_LGE_PM_USB_ID
	if (usb_present)
		cable_type_name = cable_type_str[lge_read_cable_type()];
#endif
#ifdef CONFIG_LGE_PM_CHARGE_SAFETY_TIMER
	if (chgctrl->safety_timer_expired)
		batt_mode = "Expired";
#endif
#ifdef CONFIG_LGE_PM_LLK_MODE
	if (chgctrl->store_demo_enabled)
		batt_mode = "Demo";
#endif
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	if (get_pseudo_batt_info(PSEUDO_BATT_MODE))
		batt_mode = "Fake";
#endif

	pr_cc(PR_INFO, "[CC-INFO] USB(%d, %s, %s, %dmV) WIRELESS(%d) BATT(%d%%, %dmV, %ddeg, %s) CHG(%dmA, %dmA, %dmA) CHG_T(%d/%dmin)\n",
			usb_present, usb_type_name, cable_type_name, usbin_vol,
			wireless_present,
			batt_soc, batt_vol, batt_temp, batt_mode,
			main_iusb_set, main_ibat_set, total_ibat_now,
			total_charging_time, chgctrl->safety_time);

reschedule:
	schedule_delayed_work(&chgctrl->charging_inform_work,
		round_jiffies_relative(msecs_to_jiffies(cc_info_time)));

	wake_unlock(&chgctrl->information_lock);
}
#endif

/* charger controller : userspace */
static int cc_iusb_limit = -1;
static int set_iusb_limit(const char *val, struct kernel_param *kp)
{
	struct chgctrl *chip = chgctrl_get_chgctrl();
	int ret;

	if (!chip) {
		pr_cc(PR_INFO, "not ready yet. ignore\n");
		return -ENODEV;
	}

	ret = param_set_int(val, kp);
	if (ret) {
		pr_cc(PR_ERR, "error setting value %d\n", ret);
		return ret;
	}
	pr_cc(PR_INFO, "Iusb to %dmA from userspace\n", cc_iusb_limit);

	chgctrl_input_current_limit(chip, CC_USBIN, CC_VOTER_USER,
			cc_iusb_limit, 1);

	return 0;
}
module_param_call(cc_iusb_limit, set_iusb_limit,
		param_get_int, &cc_iusb_limit, 0644);

static int cc_iwlc_limit = -1;
static int set_iwlc_limit(const char *val, struct kernel_param *kp)
{
	struct chgctrl *chip = chgctrl_get_chgctrl();
	int ret;

	if (!chip) {
		pr_cc(PR_INFO, "not ready yet. ignore\n");
		return -ENODEV;
	}

	ret = param_set_int(val, kp);
	if (ret) {
		pr_cc(PR_ERR, "error setting value %d\n", ret);
		return ret;
	}

	pr_cc(PR_INFO, "Iwlc to %dmA from userspace\n", cc_iwlc_limit);

	chgctrl_input_current_limit(chip, CC_WLCIN, CC_VOTER_USER,
			cc_iwlc_limit, 1);

	return 0;
}
module_param_call(cc_iwlc_limit, set_iwlc_limit,
		param_get_int, &cc_iwlc_limit, 0644);

static int cc_ibat_limit = -1;
static int set_ibat_limit(const char *val, struct kernel_param *kp)
{
	struct chgctrl *chip = chgctrl_get_chgctrl();
	int ret;

	if (!chip) {
		pr_cc(PR_INFO, "not ready yet. ignore\n");
		return -ENODEV;
	}

	ret = param_set_int(val, kp);
	if (ret) {
		pr_cc(PR_ERR, "error setting value %d\n", ret);
		return ret;
	}
	pr_cc(PR_INFO, "Ibat to %dmA from userspace\n", cc_ibat_limit);

	chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_USER,
			cc_ibat_limit, 1);

	return 0;
}
module_param_call(cc_ibat_limit, set_ibat_limit,
		param_get_int, &cc_ibat_limit, 0644);

static int cc_vbat_limit = -1;
static int set_vbat_limit(const char *val, struct kernel_param *kp)
{
	struct chgctrl *chip = chgctrl_get_chgctrl();
	int ret;

	if (!chip) {
		pr_cc(PR_INFO, "not ready yet. ignore\n");
		return -ENODEV;
	}

	ret = param_set_int(val, kp);
	if (ret) {
		pr_cc(PR_ERR, "error setting value %d\n", ret);
		return ret;
	}
	pr_cc(PR_INFO, "Vbat to %dmV from userspace\n", cc_vbat_limit);

	chgctrl_battery_voltage_limit(chip, CC_CHG_ALL, CC_VOTER_USER,
			cc_vbat_limit, 1);

	return 0;
}
module_param_call(cc_vbat_limit, set_vbat_limit,
		param_get_int, &cc_vbat_limit, 0644);

static int chgctrl_get_temp_state(struct chgctrl *chgctrl)
{
	struct temp_state {
		int state;
		int start;
		int release;
	};

	/* for otp ver.2 */
	const struct temp_state high_otp_v2[] = {
		{CC_BATT_TEMP_STATE_OVERHEAT,	550,	-30},
		{CC_BATT_TEMP_STATE_HIGH,	450,	-20},
	};
	const struct temp_state low_otp_v2[] = {
		{CC_BATT_TEMP_STATE_COLD,	0,	30},
		{CC_BATT_TEMP_STATE_COOL,	100,	20},
	};

	/* for otp ver.1_9 (Before ver2.0) */
	const struct temp_state high_otp_v1_9[] = {
		{CC_BATT_TEMP_STATE_OVERHEAT,	550,	-30},
		{CC_BATT_TEMP_STATE_HIGH,	450,	-20},
	};
	const struct temp_state low_otp_v1_9[] = {
		{CC_BATT_TEMP_STATE_COLD,	0,	30},
		{CC_BATT_TEMP_STATE_COOL,	100,	20},
	};

	/* for otp ver.1 */
	const struct temp_state high_otp_v1[] = {
		{CC_BATT_TEMP_STATE_OVERHEAT,	550,	-30},
		{CC_BATT_TEMP_STATE_HIGH,	450,	-20},
	};
	const struct temp_state low_otp_v1[] = {
		{CC_BATT_TEMP_STATE_COLD,	-100,	50},
	};

	/* for otp ver.1 (sprint) */
	const struct temp_state high_otp_v1_spr[] = {
		{CC_BATT_TEMP_STATE_OVERHEAT,	530,	-10},
		{CC_BATT_TEMP_STATE_HIGH,	450,	-10},
	};
	const struct temp_state low_otp_v1_spr[] = {
		{CC_BATT_TEMP_STATE_COLD,	-50,	10},
	};

	const struct temp_state *high;
	const struct temp_state *low;
	int high_size;
	int low_size;

	int cur_temp_state = chgctrl->batt_temp_state;
	int temp = chgctrl_get_battery_temperature();
	int i;

	/* select otp table */
	if (chgctrl->otp_v2) {
		high = high_otp_v2;
		low = low_otp_v2;

		high_size = ARRAY_SIZE(high_otp_v2);
		low_size = ARRAY_SIZE(low_otp_v2);
	} else {
		if (chgctrl->otp_for_sprint) {
			high = high_otp_v1_spr;
			low = low_otp_v1_spr;

			high_size = ARRAY_SIZE(high_otp_v1_spr);
			low_size = ARRAY_SIZE(low_otp_v1_spr);
		} else if (chgctrl->otp_v1_9) {
			high = high_otp_v1_9;
			low = low_otp_v1_9;

			high_size = ARRAY_SIZE(high_otp_v1_9);
			low_size = ARRAY_SIZE(low_otp_v1_9);
		} else {
			high = high_otp_v1;
			low = low_otp_v1;

			high_size = ARRAY_SIZE(high_otp_v1);
			low_size = ARRAY_SIZE(low_otp_v1);
		}
	}

	/* check high temperature */
	for (i = 0; i < high_size; i++) {
		if (temp > high[i].start)
			cur_temp_state = max(cur_temp_state, high[i].state);
		if (cur_temp_state != high[i].state)
			continue;
		if (temp < high[i].start + high[i].release)
			cur_temp_state--;
	}

	/* check low temperature */
	for (i = 0; i < low_size; i++) {
		if (temp < low[i].start)
			cur_temp_state = min(cur_temp_state, low[i].state);
		if (cur_temp_state != low[i].state)
			continue;
		if (temp > low[i].start + low[i].release)
			cur_temp_state++;
	}

	if (chgctrl->otp_v2 || chgctrl->otp_v1_9) {
		/* otp ver2.0 ver1.9 support cool state */
	} else {
		if (cur_temp_state == CC_BATT_TEMP_STATE_COOL)
			cur_temp_state = CC_BATT_TEMP_STATE_NORMAL;
	}

	return cur_temp_state;
}

static int chgctrl_get_volt_state(struct chgctrl *chgctrl)
{
	int cur_volt_state = chgctrl->batt_volt_state;
	union power_supply_propval val = {0, };

	/* update battery voltage */
	chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (val.intval <= 4000000)
		cur_volt_state = CC_BATT_VOLT_UNDER_4_0;
	else if (val.intval > 4000000)
		cur_volt_state = CC_BATT_VOLT_OVER_4_0;

	return cur_volt_state;
}

/* charger controller : over temperature protection */
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_OTP
static void otp_charging_work(struct work_struct *work)
{
	struct chgctrl *chip = container_of(work,
			struct chgctrl, otp_charging_work.work);
	int otp_fcc = 0;
	int otp_vfloat = 0;
	int active = 0;

	pr_cc(PR_INFO, "battery otp state = %s, changed = %d\n",
			batt_temp_state_str[chip->batt_temp_state],
			chip->otp_state_changed);

	if (!chip->otp_state_changed)
		return;

	if (chip->batt_temp_state != CC_BATT_TEMP_STATE_NORMAL)
		active = 1;

	otp_fcc = active ? chip->otp_fcc : 0;

	if (chip->batt_temp_state == CC_BATT_TEMP_STATE_OVERHEAT ||
			chip->batt_temp_state == CC_BATT_TEMP_STATE_COLD)
		otp_fcc = 0;

	if (chip->batt_temp_state == CC_BATT_TEMP_STATE_HIGH) {
		if (chip->batt_volt_state == CC_BATT_VOLT_OVER_4_0)
			otp_fcc = 0;

		/* do not decrease vfloat when battery voltage is high
		  to prevent battery is being dischaged by low vfloat */
		if (chip->batt_volt_state == CC_BATT_VOLT_UNDER_4_0)
			otp_vfloat = chip->otp_vfloat;
	}

	if (chip->otp_ctrl_icl) {
		chgctrl_input_current_limit(chip, CC_CHG_ALL, CC_VOTER_OTP,
				otp_fcc, otp_fcc ? 1 : 0);

		/* clear fcc limit when icl is limited */
		active = otp_fcc ? 0 : active;
	}
	chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_OTP,
			otp_fcc, active);

	/* otp ver.2 control vfloat to limit battery voltage */
	if (chip->otp_v2)
		chgctrl_battery_voltage_limit(chip, CC_CHG_ALL, CC_VOTER_OTP,
				otp_vfloat, otp_vfloat ? 1 : 0);

	chip->otp_state_changed = 0;

	return;
}
#endif

#ifdef CONFIG_LGE_PM_CHARGE_SAFETY_TIMER
static void chgctrl_safety_timer_limit(struct chgctrl *chip);
#endif

static int battery_health_polling = 0;
module_param_named(
	battery_health_polling, battery_health_polling, int, S_IRUSR | S_IWUSR
);

static void battery_health_work(struct work_struct *work)
{
	struct chgctrl *chgctrl = container_of(work,
			struct chgctrl, battery_health_work.work);
	int batt_temp_state;
	int batt_volt_state;

	batt_temp_state = chgctrl_get_temp_state(chgctrl);
	if (batt_temp_state != chgctrl->batt_temp_state) {
		pr_cc(PR_INFO, "batt_temp_state change %d -> %d\n",
				chgctrl->batt_temp_state, batt_temp_state);
		chgctrl->batt_temp_state = batt_temp_state;
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_OTP
		chgctrl->otp_state_changed = 1;
#endif
	}

	batt_volt_state = chgctrl_get_volt_state(chgctrl);
	if (batt_volt_state != chgctrl->batt_volt_state) {
		pr_cc(PR_INFO, "batt_volt_state change %d -> %d\n",
				chgctrl->batt_volt_state, batt_volt_state);
		chgctrl->batt_volt_state = batt_volt_state;
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_OTP
		chgctrl->otp_state_changed = 1;
#endif
	}

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_OTP
	/* trigger otp work */
	if (chgctrl->otp_state_changed &&
			chgctrl_is_charger_online(chgctrl)) {
		schedule_delayed_work(&chgctrl->otp_charging_work, 0);
	}
#endif

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_SPEC
	schedule_delayed_work(&chgctrl->spec_work, 0);
#endif

#ifdef CONFIG_LGE_PM_CHARGE_SAFETY_TIMER
	if (chgctrl_get_safety_timer_enable() && (chgctrl->safety_time != 0)) {
		chgctrl_safety_timer_limit(chgctrl);
	}
#endif

	if (battery_health_polling)
		schedule_delayed_work(&chgctrl->battery_health_work,
				msecs_to_jiffies(battery_health_polling));
}

/* charger controller : thermal */
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_THERMAL
int chgctrl_thermal_limit(int ma)
{
	struct chgctrl *chip = chgctrl_get_chgctrl();
	int active = 1;

	if (!chip) {
		pr_cc(PR_INFO, "not ready yet. ignore\n");
		return -ENODEV;
	}

	if (ma < 0)
		active = 0;

	if (chip->thermal_ctrl_icl)
		chgctrl_input_current_limit(chip, CC_CHG_ALL, CC_VOTER_THERMAL,
				ma, active);
	else
		chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_THERMAL,
				ma, active);

	return 0;
}
EXPORT_SYMBOL_GPL(chgctrl_thermal_limit);
#endif

/* charger controller : battery */
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
static int chgctrl_battery_id_limit(struct chgctrl *chip)
{
	static bool battery_id_checked = false;
	struct power_supply *psy;
	union power_supply_propval val;
	int rc;

	if (battery_id_checked)
		return 0;

	psy = power_supply_get_by_name("battery_id");
	if (!psy || !psy->get_property)
		return 0;

	rc = psy->get_property(psy, POWER_SUPPLY_PROP_BATTERY_ID, &val);
	if (rc)
		return 0;

	battery_id_checked = true;

	/* do nothing when battery id is present */
	if (val.intval)
		return 0;

	/* disable battery charging when battery id is invalid */
	chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_BATTERY,
			0, 1);
	return 0;
}
#endif

/* charger controller : framebuffer */
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_FB
static int chgctrl_fb_notifer_callback(struct notifier_block *nb,
				       unsigned long action, void *data)
{
	struct chgctrl *chip =
			container_of(nb, struct chgctrl, fb_notify);
	struct fb_event *evdata = (struct fb_event *)data;
	int fb_blank;
	int active = 0;

	if (!evdata || !evdata->data)
		return 0;

	fb_blank = *(int*)evdata->data;

	if (chip->fb_blank == fb_blank)
		return 0;

	/* filter unblank or not */
	if (chip->fb_blank != FB_BLANK_UNBLANK &&
			fb_blank != FB_BLANK_UNBLANK) {
		chip->fb_blank = fb_blank;
		return 0;
	}

	chip->fb_blank = fb_blank;
	pr_cc(PR_DEBUG, "fb_notify: %s\n",
			(fb_blank != FB_BLANK_UNBLANK ? "off" : "on"));

#ifdef CONFIG_LGE_BOOT_MODE
	/* apply fb limit only in normal boot */
	if (lge_get_boot_mode() != LGE_BOOT_MODE_NORMAL)
		return 0;
#endif

	/* active when display on */
	if (fb_blank == FB_BLANK_UNBLANK)
		active = 1;

	if (chip->fb_ctrl_icl) {
		chgctrl_input_current_limit(chip, CC_CHG_ALL, CC_VOTER_FB,
				chip->fb_fcc, active);
		return 0;
	}

	chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_FB,
				chip->fb_fcc, active);

	return 0;
}
#endif

/* charger controller : store demo mode */
#ifdef CONFIG_LGE_PM_LLK_MODE
static void chgctrl_llk_limit(struct chgctrl *chip)
{
	static int enabled = 0;
	static int capacity = 0;
	union power_supply_propval val = {0,};

	/* clear limits when disabling store demo mode */
	if (enabled != chip->store_demo_enabled &&
			!chip->store_demo_enabled) {
		chgctrl_input_current_limit(chip, CC_CHG_ALL, CC_VOTER_LLK, 0, 0);
		chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_LLK, 0, 0);
	}

	enabled = chip->store_demo_enabled;
	if (!enabled) {
		capacity = 0;
		return;
	}

	chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_CAPACITY, &val);
	if (capacity == val.intval)
		return;

	capacity = val.intval;

	/* limit battery charging */
	if (capacity >= chip->llk_soc_max)
		chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_LLK, 0, 1);
	if (capacity <= chip->llk_soc_min)
		chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_LLK, 0, 0);

	/* limit input current */
	if (capacity > chip->llk_soc_max)
		chgctrl_input_current_limit(chip, CC_CHG_ALL, CC_VOTER_LLK, 0, 1);
	else
		chgctrl_input_current_limit(chip, CC_CHG_ALL, CC_VOTER_LLK, 0, 0);
}
#endif

/* charger controller : safety timer */
#ifdef CONFIG_LGE_PM_CHARGE_SAFETY_TIMER
static void chgctrl_safety_timer_limit(struct chgctrl *chip)
{
	if (chip->safety_time == 0) {
		total_charging_time = 0;
		return;
	}

	/* limit battery charging */
	if (chgctrl_is_charger_online(chip) && (total_charging_time >= chip->safety_time)) {
		chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_SAFETY, 0, 1);
		chip->safety_timer_expired = true;
	}
	/* When charger remove, total_charging_time reset and IBAT recovery */
	if (!chgctrl_is_charger_online(chip)) {
		total_charging_time = 0;
		chip->safety_timer_expired = false;
		chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_SAFETY, 0, 0);
	}
}
#endif

/* charger controller : call */
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_CALL
static void chgctrl_call_limit(struct chgctrl *chip)
{
	if (chip->call_fcc < 0)
		return;

	if (chip->call_chg_mode == 0)
		return;

	switch (chip->call_chg_mode) {
	case STATUS_NONE:
		break;

	case STATUS_CONNECT:
		if (chip->call_ctrl_icl)
			chgctrl_input_current_limit(chip, CC_CHG_ALL, CC_VOTER_CALL, chip->call_fcc, 1);
		else
			chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_CALL, chip->call_fcc, 1);

		pr_cc(PR_INFO, "Call_chg_mode : %d Call_ON Decrease \n", chip->call_chg_mode);
		break;

	case STATUS_DISCON:
		if (chip->call_ctrl_icl)
			chgctrl_input_current_limit(chip, CC_CHG_ALL, CC_VOTER_CALL, 0, 0);
		else
			chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_CALL, 0, 0);

		pr_cc(PR_INFO, "Call_chg_mode : %d Call_OFF Recovery \n", chip->call_chg_mode);
		break;

	default:
		break;
	}
}
#endif

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_SPEC
static bool chgctrl_spec_is_index_valid(struct chgctrl *chip, int idx)
{
	if (idx < 0)
		return false;
	if (idx >= chip->spec_size)
		return false;

	return true;
}

static bool chgctrl_spec_in_hysterysis(struct chgctrl *chip,
		int next, int volt, int temp)
{
	struct chgctrl_spec *spec = chip->spec;
	int present = chip->spec_idx;

	/* if temperature spec is same, check only voltage */
	if (spec[next].tmin == spec[present].tmin &&
			spec[next].tmax == spec[present].tmax)
		goto check_voltage;

	if (temp > spec[next].tmax - 2)
		return true;
	if (temp < spec[next].tmin + 2)
		return true;

	return false;

check_voltage:
	if (volt > spec[next].volt - 200)
		return true;

	return false;
}

static void chgctrl_spec_work(struct work_struct *work)
{
	struct chgctrl *chip = container_of(work,
			struct chgctrl, spec_work.work);
	struct chgctrl_spec *spec = chip->spec;
	int spec_size = chip->spec_size;
	union power_supply_propval val;
	int volt, temp;
	int fcc = 0;
	int i;

	/* spec not exist. do not go further */
	if (!chip->spec)
		return;

	/* update battery data */
	chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_TEMP, &val);
	temp = val.intval / 10;
	chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	volt = val.intval / 1000;
	if (volt > 4400)
		volt = 4400;

	for (i = 0; i < spec_size; i++) {
		if (temp < spec[i].tmin || temp >= spec[i].tmax)
			continue;
		if (volt > spec[i].volt)
			continue;

		/* found spec */
		fcc = spec[i].curr;
		break;
	}

	/* same spec selected, ignore */
	if (i == chip->spec_idx)
		return;

	/* spec fcc first selected. update immediately */
	if (!chgctrl_spec_is_index_valid(chip, chip->spec_idx))
		goto update;

	/* fcc must be decreased. update immediately */
	if (fcc <= spec[chip->spec_idx].curr)
		goto update;

	/* charger not present. update immediately for next charging */
	if (!chgctrl_is_charger_present(chip))
		goto update;

	/* check hysterisis range */
	if (chgctrl_spec_in_hysterysis(chip, i, volt, temp))
		return;

update:
	/* update selected spec */
	chip->spec_idx = i;

	chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_SPEC, fcc, 1);
}
#endif

static int chgctrl_limit_changed_cb(void *args)
{
	struct chgctrl* chip = chgctrl_get_chgctrl();

	if (!chip)
		return -ENODEV;

	/* notify to battery */
	if (chgctrl_is_charger_online(chip))
		power_supply_changed(&chip->psy);
	return 0;
}

static int chgctrl_update_status(struct chgctrl *chip, int status)
{
	/* text should be sync with power_supply_show_property() */
	static char *status_text[] = {
		"Unknown", "Charging", "Discharging", "Not charging", "Full",
		"Cmd discharging"
	};

	if (chip->status == status)
		return 0;

	pr_cc(PR_INFO, "%s -> %s\n",
			status_text[chip->status], status_text[status]);
	chip->status = status;

	return 0;
}

static int update_pm_psy_status(struct chgctrl *chip)
{
	int usb_online;
	int wlc_online;

	/* update charger online */
	usb_online = is_usb_online();
	if (chip->online[CC_USBIN] != usb_online) {
		pr_cc(PR_DEBUG, "update usb_online %d -> %d\n",
				chip->online[CC_USBIN], usb_online);
		chip->online[CC_USBIN] = usb_online;
	}

	wlc_online = is_wireless_online();
	if (chip->online[CC_WLCIN] != wlc_online) {
		pr_cc(PR_DEBUG, "update wlc_online %d -> %d\n",
				chip->online[CC_WLCIN], wlc_online);
		chip->online[CC_WLCIN] = wlc_online;
	}

	pr_cc(PR_INFO, "online state %s=%d, %s=%d\n",
			chgctrl_charger_str[CC_USBIN], chip->online[CC_USBIN],
			chgctrl_charger_str[CC_WLCIN], chip->online[CC_WLCIN]);

#ifdef CONFIG_LGE_PM_VZW_REQ
	if (chip->online[CC_USBIN]) {
#ifdef CONFIG_LGE_PM_FLOATED_CHARGER_DETECT
		union power_supply_propval val = {0,};

		/* update incompatible charger for vzw */
		chgctrl_get_property_from_usb(POWER_SUPPLY_PROP_INCOMPATIBLE_CHG, &val);
		if (val.intval) {
			chip->vzw_chg_mode = VZW_INCOMPATIBLE_CHARGING;
		} else {
			chip->vzw_chg_mode = VZW_NORMAL_CHARGING;
		}
#endif
	} else {
		chip->vzw_chg_mode = VZW_NO_CHARGER;
	}
	pr_cc(PR_INFO, "vzw_chg_mode = %d\n", chip->vzw_chg_mode);
#endif

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_OTP
	/* update charger behavior */
	if (chgctrl_is_charger_online(chip))
		schedule_delayed_work(&chip->otp_charging_work,
				msecs_to_jiffies(100));
#endif

	return 0;
}

static void chgctrl_notify_worker(struct work_struct *work)
{
	struct chgctrl *chip = container_of(work,
			struct chgctrl, notify_work);

	update_pm_psy_status(chip);
}

static void changed_by_power_source(struct chgctrl *chip)
{
	cancel_work_sync(&chip->notify_work);
	schedule_work(&chip->notify_work);
}

static void changed_by_batt_psy(struct chgctrl *chip)
{
	union power_supply_propval val = {0,};

	/* update charger present. this will be faster than online */
	chip->present[CC_USBIN] = is_usb_present();
	chip->present[CC_WLCIN] = is_wireless_present();

	/* update battery status information */
	chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_STATUS, &val);
	if (!chgctrl_is_charger_present(chip) ||
			chip->status == POWER_SUPPLY_STATUS_DISCHARGING ||
			chip->status == POWER_SUPPLY_STATUS_FULL) {
		if (wake_lock_active(&chip->chg_wake_lock)) {
			pr_cc(PR_DEBUG, "chg_wake_unlocked\n");
			wake_unlock(&chip->chg_wake_lock);
		}
	} else {
		if (!wake_lock_active(&chip->chg_wake_lock)) {
			pr_cc(PR_DEBUG, "chg_wake_locked\n");
			wake_lock(&chip->chg_wake_lock);
		}
	}

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	chgctrl_battery_id_limit(chip);
#endif

#ifdef CONFIG_LGE_PM_LLK_MODE
	chgctrl_llk_limit(chip);
#endif

	/* need update battery health when polling mode */
	if (battery_health_polling)
		schedule_delayed_work(&chip->battery_health_work,
				msecs_to_jiffies(100));
}

int notify_charger_controller(struct power_supply *psy)
{
	struct power_supply *pst = chgctrl_get_power_supply();
	struct chgctrl *chip = chgctrl_get_chgctrl();

	if (!pst || !chip) {
		pr_cc(PR_INFO, "not ready yet. ignore\n");
		return -ENODEV;
	}

	pr_cc(PR_DEBUG, "notified by %s\n", psy->name);

	if (psy->type == POWER_SUPPLY_TYPE_BATTERY)
		changed_by_batt_psy(chip);
	else if (psy == pst)
		changed_by_power_source(chip);
	else if (psy->type != POWER_SUPPLY_TYPE_UNKNOWN)
		changed_by_power_source(chip);

	return 0;
}
EXPORT_SYMBOL_GPL(notify_charger_controller);

void chgctrl_property_override(enum power_supply_property prop,
				union power_supply_propval *val)
{
	struct chgctrl *chip = chgctrl_get_chgctrl();

	if (!chip) {
		pr_cc(PR_INFO, "not ready yet. ignore\n");
		return;
	}

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		chgctrl_update_status(chip, val->intval);

		if (!chgctrl_is_charger_present(chip))
			break;

		/* override only if status is fine */
		if (chip->status != POWER_SUPPLY_STATUS_CHARGING &&
				chip->status != POWER_SUPPLY_STATUS_FULL)
			break;

		/* show battery full when battery level is 100% */
		if (chgctrl_get_battery_capacity() >= 100)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
		if (get_pseudo_batt_info(PSEUDO_BATT_MODE))
			val->intval = 1;
#endif
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
		if (get_pseudo_batt_info(PSEUDO_BATT_MODE))
			val->intval = get_pseudo_batt_info(PSEUDO_BATT_CAPACITY);
#endif
		break;
	case POWER_SUPPLY_PROP_TEMP:
#ifdef CONFIG_MTK_SMART_BATTERY
		/* fall through */
	case POWER_SUPPLY_PROP_batt_temp:
#endif
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
		if (get_pseudo_batt_info(PSEUDO_BATT_MODE))
			val->intval = get_pseudo_batt_info(PSEUDO_BATT_TEMP) * 10;
#endif
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#ifdef CONFIG_MTK_SMART_BATTERY
	case POWER_SUPPLY_PROP_batt_vol:
#endif
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
		if (get_pseudo_batt_info(PSEUDO_BATT_MODE))
			val->intval = get_pseudo_batt_info(PSEUDO_BATT_VOLT);
#endif
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (chip->batt_temp_state == CC_BATT_TEMP_STATE_OVERHEAT)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (chip->batt_temp_state == CC_BATT_TEMP_STATE_COLD)
			val->intval = POWER_SUPPLY_HEALTH_COLD;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
#ifdef CONFIG_LGE_PM_CHARGE_SAFETY_TIMER
		if (chip->safety_timer_expired)
			val->intval = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
#endif
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		if (chip->psy_technology != POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
			val->intval = chip->psy_technology;
		break;
	default:
		break;
	}

	return;
}
EXPORT_SYMBOL_GPL(chgctrl_property_override);

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
int chgctrl_select_charging_current(int *icl, int *fcc)
{
	struct chgctrl *chip = chgctrl_get_chgctrl();
	int value;
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	int boost_icl = 0;
#endif

	if (!chip) {
		pr_cc(PR_INFO, "not ready. do not update\n");
		return -ENODEV;
	}

	/* update icl */
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	if (get_pseudo_batt_info(PSEUDO_BATT_MODE))
		boost_icl = 1;
#endif
	if (chip->usb_current_max_enabled)
		boost_icl = 1;

	if (boost_icl) {
		if (*icl < chip->usb_current_max * 100)
			*icl = chip->usb_current_max * 100;
		goto end_update_icl;
	}
#endif
	value = chgctrl_get_effective_limit(chip, chip->icl) * 100;
	if (value < 0)
		goto end_update_icl;

	if (value < *icl)
		*icl = value;
end_update_icl:

	/* update fcc */
	value = chgctrl_get_effective_limit(chip, chip->fcc) * 100;
	if (value < 0)
		goto end_update_fcc;

	if (value < *fcc)
		*fcc = value;
end_update_fcc:

	return 0;
}
EXPORT_SYMBOL_GPL(chgctrl_select_charging_current);

int chgctrl_select_cv(int *vfloat)
{
	struct chgctrl *chip = chgctrl_get_chgctrl();
	int value;

	if (!chip) {
		pr_cc(PR_INFO, "not ready. do not update\n");
		return -ENODEV;
	}

	value = chgctrl_get_effective_limit(chip, chip->vfloat) * 1000;
	if (value < 0)
		goto end_update_vfloat;

	if (value < *vfloat)
		*vfloat = value;
end_update_vfloat:

	return 0;
}
EXPORT_SYMBOL_GPL(chgctrl_select_cv);


int chgctrl_charging_enabled(void)
{
	struct chgctrl *chip = chgctrl_get_chgctrl();

	if (!chip)
		return 1;

	if (!chgctrl_get_effective_limit(chip, chip->icl))
		return 0;

	return 1;
}
EXPORT_SYMBOL_GPL(chgctrl_charging_enabled);

int chgctrl_battery_charging_enabled(void)
{
	struct chgctrl *chip = chgctrl_get_chgctrl();

	if (!chip)
		return 1;

	if (!chgctrl_get_effective_limit(chip, chip->fcc))
		return 0;

	return 1;
}
EXPORT_SYMBOL_GPL(chgctrl_battery_charging_enabled);

void chgctrl_update_battery_health(void)
{
	struct chgctrl *chip = chgctrl_get_chgctrl();

	if (!chip)
		return;

	/* if battery_health_polling is not zero,
	  updating health will be done in charger-controller */
	if (battery_health_polling)
		return;

	battery_health_work(&chip->battery_health_work.work);
}
EXPORT_SYMBOL_GPL(chgctrl_update_battery_health);
#endif

/* charger controller : power_supply class */
static char *chgctrl_supplied_to[] = {
	"battery",
};

static enum power_supply_property pm_power_props_chgctrl_pros[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_CALL
	POWER_SUPPLY_PROP_CALL_CHG_MODE,
#endif
#ifdef CONFIG_LGE_PM_LLK_MODE
	POWER_SUPPLY_PROP_STORE_DEMO_ENABLED,
#endif
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	POWER_SUPPLY_PROP_PSEUDO_BATT,
#endif
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	POWER_SUPPLY_PROP_USB_CURRENT_MAX,
#endif
#ifdef CONFIG_LGE_PM_VZW_REQ
	POWER_SUPPLY_PROP_VZW_CHG,
#endif
};

static int chgctrl_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	struct chgctrl *cc = container_of(psy, struct chgctrl, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		chgctrl_thermal_limit(val->intval);
		break;
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_CALL
	case POWER_SUPPLY_PROP_CALL_CHG_MODE:
		pr_cc(PR_INFO, "Set property call_chg_mode : %d\n", val->intval);
		cc->call_chg_mode = val->intval;
		chgctrl_call_limit(cc);
		break;
#endif
#ifdef CONFIG_LGE_PM_LLK_MODE
	case POWER_SUPPLY_PROP_STORE_DEMO_ENABLED:
		pr_cc(PR_INFO, "Set property store_demo_enabled : %d\n", val->intval);
		if (cc->store_demo_enabled != val->intval) {
			cc->store_demo_enabled = val->intval;
			chgctrl_llk_limit(cc);
		}
		break;
#endif
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	case POWER_SUPPLY_PROP_USB_CURRENT_MAX:
		if (val->intval)
			cc->usb_current_max_enabled = 1;
		else
			cc->usb_current_max_enabled = 0;
		break;
#endif
	default:
		pr_cc(PR_DEBUG, "Invalid property(%d)\n", psp);
		return -EINVAL;
	}

	return 0;
}

static int chgctrl_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	case POWER_SUPPLY_PROP_USB_CURRENT_MAX:
#endif
#ifdef CONFIG_LGE_PM_LLK_MODE
	case POWER_SUPPLY_PROP_STORE_DEMO_ENABLED:
#endif
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_CALL
	case POWER_SUPPLY_PROP_CALL_CHG_MODE:
#endif
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}

static int chgctrl_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct chgctrl *cc = container_of(psy, struct chgctrl, psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chgctrl_is_charger_present(cc);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = cc->status;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chgctrl_get_effective_limit(cc, cc->fcc);
		if (val->intval > 0)
			val->intval *= CURRENT_DIVIDE_UNIT;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = chgctrl_get_effective_limit(cc, cc->vfloat);
		if (val->intval > 0)
			val->intval *= VOLTAGE_DIVIDE_UNIT;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = chgctrl_get_effective_limit(cc, cc->icl);
		if (val->intval > 0)
			val->intval *= CURRENT_DIVIDE_UNIT;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chgctrl_is_charger_online(cc);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		if (cc->thermal_ctrl_icl)
			val->intval = chgctrl_get_effective_limit(cc, cc->icl);
		else
			val->intval = chgctrl_get_effective_limit(cc, cc->fcc);
		if (val->intval > 0)
			val->intval *= CURRENT_DIVIDE_UNIT;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = 1;
		if (!chgctrl_get_effective_limit(cc, cc->icl))
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = 1;
		if (!chgctrl_get_effective_limit(cc, cc->fcc))
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = chgctrl_get_charging_step(cc);
		break;
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_CALL
	case POWER_SUPPLY_PROP_CALL_CHG_MODE:
		val->intval = cc->call_chg_mode;
		break;
#endif
#ifdef CONFIG_LGE_PM_LLK_MODE
	case POWER_SUPPLY_PROP_STORE_DEMO_ENABLED:
		val->intval = cc->store_demo_enabled;
		break;
#endif
#ifdef CONFIG_LGE_PM_VZW_REQ
	case POWER_SUPPLY_PROP_VZW_CHG:
		val->intval = cc->vzw_chg_mode;
		break;
#endif
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	case POWER_SUPPLY_PROP_PSEUDO_BATT:
		val->intval = get_pseudo_batt_info(PSEUDO_BATT_MODE);
		break;
#endif
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	case POWER_SUPPLY_PROP_USB_CURRENT_MAX:
		val->intval = cc->usb_current_max_enabled;
		break;
#endif
	default:
		pr_cc(PR_INFO, "invalid property %d requested.\n", psp);
		return -EINVAL;
	}

	return rc;
}

static struct power_supply chgctrl_psy = {
	.name = "charger_controller",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.supplied_to = chgctrl_supplied_to,
	.num_supplicants = ARRAY_SIZE(chgctrl_supplied_to),
	.properties = pm_power_props_chgctrl_pros,
	.num_properties = ARRAY_SIZE(pm_power_props_chgctrl_pros),
	.get_property = chgctrl_get_property,
	.set_property = chgctrl_set_property,
	.property_is_writeable = chgctrl_property_is_writeable,
};

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
static ssize_t at_chg_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int r;
	bool b_chg_ok = false;
	int chg_type;
	union power_supply_propval val = {0,};

	chgctrl_get_property_from_batt(POWER_SUPPLY_PROP_CHARGE_TYPE, &val);
	chg_type = val.intval;

	if (chg_type != POWER_SUPPLY_CHARGE_TYPE_NONE) {
		b_chg_ok = true;
		r = snprintf(buf, 3, "%d\n", b_chg_ok);
		pr_cc(PR_INFO, "[Diag] true ! buf = %s, charging=1\n", buf);
	} else {
		b_chg_ok = false;
		r = snprintf(buf, 3, "%d\n", b_chg_ok);
		pr_cc(PR_INFO, "[Diag] false ! buf = %s, charging=0\n", buf);
	}

	return r;
}

static ssize_t at_chg_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;
	int intval;

	if (strncmp(buf, "0", 1) == 0) {
		/* stop charging */
		pr_cc(PR_INFO, "[Diag] stop charging start\n");
		intval = 0;

	} else if (strncmp(buf, "1", 1) == 0) {
		/* start charging */
		pr_cc(PR_INFO, "[Diag] start charging start\n");
		intval = 1;
	}

	if (ret)
		return -EINVAL;

	return 1;
}
static DEVICE_ATTR(at_charge, 0644, at_chg_status_show, at_chg_status_store);

static ssize_t at_chg_complete_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int guage_level = chgctrl_get_battery_capacity();

	if (guage_level == 100) {
		ret = snprintf(buf, 3, "%d\n", 0);
		pr_cc(PR_INFO, "[Diag] buf = %s, gauge==100\n", buf);
	} else {
		ret = snprintf(buf, 3, "%d\n", 1);
		pr_cc(PR_INFO, "[Diag] buf = %s, gauge<=100\n", buf);
	}

	return ret;
}

static ssize_t at_chg_complete_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;
	int intval;

	if (strncmp(buf, "0", 1) == 0) {
		/* charging not complete */
		pr_cc(PR_INFO, "[Diag] charging not complete start\n");
		intval = 1;
	} else if (strncmp(buf, "1", 1) == 0) {
		/* charging complete */
		pr_cc(PR_INFO, "[Diag] charging complete start\n");
		intval = 0;
	}

	if (ret)
		return -EINVAL;

	return 1;
}
static DEVICE_ATTR(at_chcomp, 0644, at_chg_complete_show, at_chg_complete_store);
#endif

static struct device_attribute *chgctrl_attrs[] = {
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	&dev_attr_at_charge,
	&dev_attr_at_chcomp,
#endif
	NULL,
};

static int chgctrl_create_files(struct device *dev,
				struct device_attribute **attrs)
{
	int rc = 0;

	if (!attrs)
		return -EINVAL;

	while (*attrs) {
		rc = device_create_file(dev, *attrs);
		if (rc) {
			pr_cc(PR_ERR, "create %s failed. %d\n",
				(*attrs)->attr.name, rc);
		}
		attrs++;
	}

	return rc;

}

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_SPEC
static int chgctrl_parse_dt_spec(struct chgctrl *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc = 0;
	int i;

	rc = of_property_read_u32(node, "lge,battery-spec-size",
			&chip->spec_size);
	if (rc)
		return rc;

	if (!chip->spec_size)
		return -EINVAL;

	chip->spec = devm_kmalloc(chip->dev,
			sizeof(struct chgctrl_spec) * chip->spec_size,
			GFP_KERNEL);
	if (!chip->spec)
		return -ENOMEM;

	for (i = 0; i < chip->spec_size; i++) {
		/* temperature (minimum) */
		rc = of_property_read_u32_index(node, "lge,battery-spec",
				(i * 4) + 0, &chip->spec[i].tmin);
		if (rc)
			goto err;

		/* temperature (maximum) */
		rc = of_property_read_u32_index(node, "lge,battery-spec",
				(i * 4) + 1, &chip->spec[i].tmax);
		if (rc)
			goto err;

		/* voltage (maximum) */
		rc = of_property_read_u32_index(node, "lge,battery-spec",
				(i * 4) + 2, &chip->spec[i].volt);
		if (rc)
			goto err;

		/* current (maximum) */
		rc = of_property_read_u32_index(node, "lge,battery-spec",
				(i * 4) + 3, &chip->spec[i].curr);
		if (rc)
			goto err;

	}

	return rc;

err:
	devm_kfree(chip->dev, chip->spec);
	chip->spec = NULL;

	return rc;
}
#endif

static int chgctrl_parse_dt(struct chgctrl *cc)
{
	struct device_node *node = cc->dev->of_node;
	int rc = 0;

	cc->psy_icl = 1800;
	rc = of_property_read_u32(node, "lge,icl", &cc->psy_icl);
	cc->psy_fcc = 3200;
	rc = of_property_read_u32(node, "lge,fcc", &cc->psy_fcc);
	cc->psy_vfloat = 4400;
	rc = of_property_read_u32(node, "lge,vfloat", &cc->psy_vfloat);

	rc = of_property_read_u32(node, "lge,technology", &cc->psy_technology);
	if (rc)
		cc->psy_technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_OTP
	/* otp */
	cc->otp_v2 = of_property_read_bool(node, "lge,otp_v2");
	cc->otp_v1_9 = of_property_read_bool(node, "lge,otp_v1_9");
	cc->otp_for_sprint = of_property_read_bool(node, "lge,otp_for_sprint");
	cc->otp_fcc = 450;
	rc = of_property_read_u32(node, "lge,otp_fcc", &cc->otp_fcc);

	cc->otp_vfloat = 4000;
	rc = of_property_read_u32(node, "lge,otp_vfloat", &cc->otp_vfloat);

	cc->otp_ctrl_icl = of_property_read_bool(node, "lge,otp_ctrl_icl");
#endif

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_THERMAL
	cc->thermal_ctrl_icl = of_property_read_bool(node, "lge,thermal_ctrl_icl");
#endif

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_FB
	/* framebuffer */
	cc->fb_fcc = -1;
	rc = of_property_read_u32(node, "lge,fb_fcc", &cc->fb_fcc);

	cc->fb_ctrl_icl = of_property_read_bool(node, "lge,fb_ctrl_icl");
#endif

#ifdef CONFIG_LGE_PM_LLK_MODE
	/* store demo mode */
	cc->llk_soc_max = 50;
	rc = of_property_read_u32(node, "lge,llk_soc_max", &cc->llk_soc_max);

	cc->llk_soc_min = 45;
	rc = of_property_read_u32(node, "lge,llk_soc_min", &cc->llk_soc_min);
#endif

#ifdef CONFIG_LGE_PM_CHARGE_SAFETY_TIMER
	cc->safety_time = 0;
	rc = of_property_read_u32(node, "lge,safety_time", &cc->safety_time);
#endif

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_CALL
	/* call */
	cc->call_fcc = -1;
	rc = of_property_read_u32(node, "lge,call_fcc", &cc->call_fcc);

	cc->call_ctrl_icl = of_property_read_bool(node, "lge,call_ctrl_icl");
#endif

#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	cc->usb_current_max = 900;
	rc = of_property_read_u32(node, "lge,usb_current_max",
			&cc->usb_current_max);
#endif

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_SPEC
	rc = chgctrl_parse_dt_spec(cc);
#endif

	return 0;
}

static int chgctrl_suspend(struct device *dev)
{
	return 0;
}

static int chgctrl_resume(struct device *dev)
{
	return 0;
}

static int chgctrl_probe(struct platform_device *pdev)
{
	struct chgctrl *chip = NULL;
	int ret = 0;

	chip = devm_kzalloc(&pdev->dev,
			sizeof(struct chgctrl), GFP_KERNEL);
	if (!chip) {
		pr_cc(PR_ERR, "failed to alloc memory\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, chip);

	chip->dev = &pdev->dev;

	ret = chgctrl_parse_dt(chip);
	if (ret)
		pr_cc(PR_ERR, "failed to parse dt\n");

	chgctrl_init_charge_limit(chip, "icl", chip->icl,
			chgctrl_limit_changed_cb);
	chgctrl_init_charge_limit(chip, "fcc", chip->fcc,
			chgctrl_limit_changed_cb);
	chgctrl_init_charge_limit(chip, "vfloat", chip->vfloat,
			chgctrl_limit_changed_cb);

	chgctrl_input_current_limit(chip, CC_CHG_ALL, CC_VOTER_PSY,
			chip->psy_icl, 1);
	chgctrl_battery_current_limit(chip, CC_CHG_ALL, CC_VOTER_PSY,
			chip->psy_fcc, 1);
	chgctrl_battery_voltage_limit(chip, CC_CHG_ALL, CC_VOTER_PSY,
			chip->psy_vfloat, 1);

	chip->batt_temp_state = CC_BATT_TEMP_STATE_NORMAL;
	chip->batt_volt_state = CC_BATT_VOLT_UNDER_4_0;
	if (chip->otp_v2)
		chip->batt_volt_state = CC_BATT_VOLT_DONTCARE;

#ifdef CONFIG_LGE_PM_LLK_MODE
	chip->store_demo_enabled = 0;
#endif
#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	chip->usb_current_max_enabled = 0;
#endif
#ifdef CONFIG_LGE_PM_CHARGE_SAFETY_TIMER
	chip->safety_timer_expired = false;
#endif
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_CALL
	chip->call_chg_mode = 0;
#endif
	wake_lock_init(&chip->chg_wake_lock,
			WAKE_LOCK_SUSPEND, "charging_wake_lock");
#ifdef CONFIG_LGE_PM_CHARGE_INFO
	wake_lock_init(&chip->information_lock,
			WAKE_LOCK_SUSPEND, "chgctrl_info");
#endif

	INIT_WORK(&chip->notify_work, chgctrl_notify_worker);
#ifdef CONFIG_LGE_PM_CHARGE_INFO
	INIT_DELAYED_WORK(&chip->charging_inform_work, charging_information);
#endif
	INIT_DELAYED_WORK(&chip->battery_health_work, battery_health_work);
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_OTP
	INIT_DELAYED_WORK(&chip->otp_charging_work, otp_charging_work);
#endif
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_SPEC
	chip->spec_idx = -1;
	INIT_DELAYED_WORK(&chip->spec_work, chgctrl_spec_work);
#endif

	chip->psy = chgctrl_psy;
	ret = power_supply_register(chip->dev, &chip->psy);
	if (ret < 0) {
		pr_cc(PR_ERR, "power_supply_register failed. %d\n", ret);
		goto error;
	}

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_FB
	chip->fb_notify.notifier_call = chgctrl_fb_notifer_callback;
	chip->fb_blank = FB_BLANK_POWERDOWN;
	fb_register_client(&chip->fb_notify);
#endif

	ret = chgctrl_create_files(chip->dev, chgctrl_attrs);
	if (ret)
		pr_cc(PR_ERR, "create file failed. %d\n", ret);

#ifdef CONFIG_LGE_PM_CHARGE_INFO
	schedule_delayed_work(&chip->charging_inform_work,
		round_jiffies_relative(msecs_to_jiffies(cc_info_time)));
#endif

	return 0;
error:
	wake_lock_destroy(&chip->chg_wake_lock);
#ifdef CONFIG_LGE_PM_CHARGE_INFO
	wake_lock_destroy(&chip->information_lock);
#endif

	return ret;
}

static int chgctrl_remove(struct platform_device *pdev)
{
	struct chgctrl *chip = platform_get_drvdata(pdev);
#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_OTP
	cancel_delayed_work(&chip->otp_charging_work);
#endif
#ifdef CONFIG_LGE_PM_CHARGE_INFO
	cancel_delayed_work(&chip->charging_inform_work);

	wake_lock_destroy(&chip->information_lock);
#endif
	wake_lock_destroy(&chip->chg_wake_lock);

#ifdef CONFIG_LGE_PM_CHARGE_SCENARIO_FB
	fb_unregister_client(&chip->fb_notify);
#endif
	power_supply_unregister(&chip->psy);

	return 0;
}

static const struct dev_pm_ops chgctrl_pm_ops = {
	.suspend = chgctrl_suspend,
	.resume = chgctrl_resume,
};

static struct of_device_id chgctrl_match_table[] = {
		{ .compatible = "lge,charger-controller", },
		{},
};

static struct platform_driver chgctrl_driver = {
	.probe = chgctrl_probe,
	.remove = chgctrl_remove,
	.driver = {
		.name = "lge,charger-controller",
		.owner = THIS_MODULE,
		.of_match_table = chgctrl_match_table,
		.pm = &chgctrl_pm_ops,
	},
};

static void async_chgctrl_init(void *data, async_cookie_t cookie)
{
	platform_driver_register(&chgctrl_driver);
	return;
}

static int __init chgctrl_init(void)
{
	async_schedule(async_chgctrl_init, NULL);

	return 0;
}

static void __exit chgctrl_exit(void)
{
	platform_driver_unregister(&chgctrl_driver);
}

module_init(chgctrl_init);
module_exit(chgctrl_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("charger IC driver current controller");
MODULE_VERSION("1.0");
