/*
* fan54005.c -- reference charger driver controlled by i2c
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/async.h>

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
#include "charging_hw_external_charger.h"
#endif

#define VENDOR_CODE (0x04)
#define PART_NUMBER (0x05)

#define CONTROL0_REG	(0x00)
#define TMR_RST_SHIFT	(7)
#define TMR_RST_MASK	(1U << TMR_RST_SHIFT)
#define EN_STAT_SHIFT	(6)
#define EN_STAT_MASK	(1U << EN_STAT_SHIFT)
#define STAT_SHIFT		(4)
#define STAT_MASK		(3U << STAT_SHIFT)
#define BOOST_SHIFT		(3)
#define BOOST_MASK		(1U << BOOST_SHIFT)
#define FAULT_SHIFT		(0)
#define FAULT_MASK		(7U << FAULT_SHIFT)

#define CONTROL1_REG 	(0x01)
#define IINLIM_SHIFT		(6)
#define IINLIM_MASK		(3U << IINLIM_SHIFT)
#define VLOWV_SHIFT		(4)
#define VLOWV_MASK		(3U << VLOWV_SHIFT)
#define TE_SHIFT		(3)
#define TE_MASK		(1U << TE_SHIFT)
#define CE_SHIFT		(2)
#define CE_MASK		(1U << CE_SHIFT)
#define HZ_MODE_SHIFT	(1)
#define HZ_MODE_MASK	(1U << HZ_MODE_SHIFT)
#define OPA_MODE_SHIFT	(0)
#define OPA_MODE_MASK	(1U << OPA_MODE_SHIFT)

#define OREG_REG		(0x02)
#define OREG_SHIFT		(2)
#define OREG_MASK		(0x3F << OREG_SHIFT)
#define OTG_PL_SHIFT	(1)
#define OTG_PL_MASK		(1U << OTG_PL_SHIFT)
#define OTG_EN_SHIFT	(0)
#define OTG_EN_MASK		(1U << OTG_EN_SHIFT)

#define IC_INFO_REG		(0x03)
#define VENDOR_SHIFT	(5)
#define VENDOR_MASK		(7U << VENDOR_SHIFT)
#define PN_SHIFT		(2)
#define PN_MASK			(7U << PN_SHIFT)
#define REV_SHIFT		(0)
#define REV_MASK		(3U << REV_SHIFT)

#define IBAT_REG		(0x04)
#define RESET_SHIFT		(7)
#define RESET_MASK		(1U << RESET_SHIFT)
#define IOCHARGE_SHIFT	(4)
#define IOCHARGE_MASK	(7U << IOCHARGE_SHIFT)
#define ITERM_SHIFT		(0)
#define ITERM_MASK		(7U << ITERM_SHIFT)

#define SP_CHARGER_REG	(0x05)
#define DIS_VREG_SHIFT	(6)
#define DIS_VREG_MASK	(1U << DIS_VREG_SHIFT)
#define IO_LEVEL_SHIFT	(5)
#define IO_LEVEL_MASK	(1U << IO_LEVEL_SHIFT)
#define SP_SHIFT		(4)
#define SP_MASK			(1U << SP_SHIFT)
#define EN_LEVEL_SHIFT	(3)
#define EN_LEVEL_MASK	(1U << EN_LEVEL_SHIFT)
#define VSP_SHIFT		(0)
#define VSP_MASK		(7U << VSP_SHIFT)

#define SAFETY_REG		(0x06)
#define ISAFE_SHIFT		(4)
#define ISAFE_MASK		(7U << ISAFE_SHIFT)
#define VSAFE_SHIFT		(0)
#define VSAFE_MASK		(0xF << VSAFE_SHIFT)

#define MONITOR_REG		(0x10)
#define ITERM_CMP_SHIFT	(7)
#define ITERM_CMP_MASK	(1U << ITERM_CMP_SHIFT)
#define VBAT_CMP_SHIFT	(6)
#define VBAT_CMP_MASK	(1U << VBAT_CMP_SHIFT)
#define LINCHG_SHIFT	(5)
#define LINCHG_MASK		(1U << LINCHG_SHIFT)
#define T_120_SHIFT		(4)
#define T_120_MASK		(1U << T_120_SHIFT)
#define ICHG_SHIFT		(3)
#define ICHG_MASK		(1U << ICHG_SHIFT)
#define IBUS_SHIFT		(2)
#define IBUS_MASK		(1U << IBUS_SHIFT)
#define VBUS_VALID_SHIFT	(1)
#define VBUS_VALID_MASK	(1U << VBUS_VALID_SHIFT)
#define CV_SHIFT		(0)
#define CV_MASK			(1U << CV_SHIFT)

/* Stat Description */
#define STAT_READY		(0x00)
#define STAT_CHARGE_IN_PROGRESS	(0x01)
#define STAT_CHARGE_DONE	(0x02)
#define STAT_FAULT		(0x03)

#define DUMP_SIZE 256

/* Fault Description */
enum {
	CHARGE_FAULT_NORMAL,
	CHARGE_FAULT_VBUS_OVP,
	CHARGE_FAULT_SLEEP_MODE,
	CHARGE_FAULT_POOR_INPUT_SOURCE,
	CHARGE_FAULT_BATTERY_OVP,
	CHARGE_FAULT_THERMAL_SHUTDOWN,
	CHARGE_FAULT_TIMER,
	CHARGE_FAULT_NO_BATTERY,
};

#define TMR_RST_MSECS	(10 * 1000)	/* 10 sec */

static const int iocharge_68ohm[] = {
	550, 650, 750, 850, 1050, 1150, 1350, 1450,
};
static const int iocharge_100ohm[] = {
	374, 442, 510, 578, 714, 782, 918, 986,
};
static const int iterm_68ohm[] = {
	49, 97, 146, 194, 243, 291, 340, 388,
};
static const int iterm_100ohm[] = {
	33, 66, 99, 132, 165, 198, 243, 164,
};
static const int iinlim[] = {
	100, 500, 800, 1450,
};
static const int vsp[] = {
	4213, 4293, 4373, 4453, 4533, 4613, 4693, 4773,
};
static const int isafe_68ohm[] = {
	550, 650, 750, 850, 1050, 1150, 1350, 1450,
};
static const int isafe_100ohm[] = {
	374, 442, 510, 578, 714, 782, 918, 986,
};

static const char *stat_str[] = {
	"ready",
	"charge in progress",
	"charge done",
	"fault",
};

static const char *charge_fault_str[] = {
	"None",
	"VBUS OVP",
	"Sleep Mode",
	"Poor Input Source",
	"Battery OVP",
	"Thermal Shutdown",
	"Timer Fault",
	"No Battery",
};

static const char *boost_fault_str[] = {
	"None",
	"Vbus > VBUSovp",
	"Over Current",
	"Vbat < UVLObst",
	"N/A",
	"Thermal Shutdown",
	"Timer Fault",
	"N/A",
};

struct fan54005_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct fan54005 {
	struct i2c_client *client;
	struct device *dev;

	struct power_supply psy;
	struct fan54005_regulator otg_vreg;
	struct mutex lock;
	struct delayed_work reset_timer;
	struct delayed_work reg_init;

	bool otg_boost_enabled;
	int te;

	/* device tree */
	bool is_slave;
	int irq_gpio;
	int rsense;
	int iterm_ma;
	int vsp_mv;
	int isafe_ma;
	int vsafe_mv;

	/* debugfs */
	struct dentry *debugfs;
	u32 debug_addr;

	/* dump buffer */
	char buffer[DUMP_SIZE];
};

const static int regs_to_dump[] = {
	CONTROL0_REG,
	CONTROL1_REG,
	OREG_REG,
	IC_INFO_REG,
	IBAT_REG,
	SP_CHARGER_REG,
	SAFETY_REG,
	MONITOR_REG,
};

static int fan54005_read(struct i2c_client *client, u8 reg, u8 *data)
{
	int rc;

	rc = i2c_smbus_read_byte_data(client, reg);
	if (rc < 0)
		dev_err(&client->dev, "failed to read. rc=%d\n", rc);

	*data = (rc & 0xFF);

	return rc < 0 ? rc : 0;
}

static int fan54005_write(struct i2c_client *client, u8 reg, u8 data)
{
	int rc;

	rc = i2c_smbus_write_byte_data(client, reg, data);
	if (rc < 0)
		dev_err(&client->dev, "failed to write. rc=%d\n", rc);

	return rc;
}

static int fan54005_masked_write(struct i2c_client *client, u8 reg, u8 data, u8 mask)
{
	struct fan54005 *chip = i2c_get_clientdata(client);
	u8 tmp;
	int rc = 0;

	mutex_lock(&chip->lock);

	rc = fan54005_read(client, reg, &tmp);
	if (rc < 0)
		goto out;

	/* set 1 to IBAT_REG bit 7 will reset charge parameters.
	  this bit alwayes read as 1 so clear bit here */
	if (reg == IBAT_REG)
		tmp &= 0x7F;

	tmp = (data & mask) | (tmp & (~mask));
	rc = fan54005_write(client, reg, tmp);

out:
	mutex_unlock(&chip->lock);
	return rc;
}

static int fan54005_tmr_rst(struct fan54005 *chip)
{
	return fan54005_masked_write(chip->client, CONTROL0_REG,
			(1U) << TMR_RST_SHIFT, TMR_RST_MASK);
}

static int fan54005_get_stat(struct fan54005 *chip)
{
	u8 data;
	int rc;

	rc = fan54005_read(chip->client, CONTROL0_REG, &data);
	if (rc)
		return rc;

	data &= STAT_MASK;
	data >>= STAT_SHIFT;

	return data;
}

static int fan54005_get_boost(struct fan54005 *chip)
{
	u8 data;
	int rc;

	rc = fan54005_read(chip->client, CONTROL0_REG, &data);
	if (rc)
		return rc;

	data &= BOOST_MASK;
	data >>= BOOST_SHIFT;

	return data;
}

static int fan54005_get_fault(struct fan54005 *chip)
{
	u8 data;
	int rc;

	rc = fan54005_read(chip->client, CONTROL0_REG, &data);
	if (rc)
		return rc;

	data &= FAULT_MASK;
	data >>= FAULT_SHIFT;

	return data;
}

static int fan54005_get_iinlim(struct fan54005 *chip)
{
	u8 data;
	int rc;

	rc = fan54005_read(chip->client, CONTROL1_REG, &data);
	if (rc)
		return rc;

	data = (data & IINLIM_MASK) >> IINLIM_SHIFT;

	return iinlim[data];
}

static int fan54005_set_iinlim(struct fan54005 *chip, int ma)
{
	u8 data;

	for (data = 0; data < ARRAY_SIZE(iinlim); data++) {
		if (ma <= iinlim[data])
			break;
	}
	if (data > 3)
		data = 3;

	return fan54005_masked_write(chip->client, CONTROL1_REG,
			data << IINLIM_SHIFT, IINLIM_MASK);
}

static int fan54005_get_te(struct fan54005 *chip)
{
	u8 data;
	int rc;

	rc = fan54005_read(chip->client, CONTROL1_REG, &data);
	if (rc)
		return rc;

	data &= TE_MASK;
	data >>= TE_SHIFT;

	return data;
}

static int fan54005_set_te(struct fan54005 *chip, int en)
{
	u8 data;

	if (en)
		data = 1;
	else
		data = 0;

	return fan54005_masked_write(chip->client, CONTROL1_REG,
			data << TE_SHIFT, TE_MASK);
}

static int fan54005_get_ce(struct fan54005 *chip)
{
	u8 data;
	int rc;

	rc = fan54005_read(chip->client, CONTROL1_REG, &data);
	if (rc)
		return rc;

	data &= CE_MASK;
	data >>= CE_SHIFT;

	return data;
}

static int fan54005_set_ce(struct fan54005 *chip, int en)
{
	u8 data = 0;

	if (en)
		data = 1;

	return fan54005_masked_write(chip->client, CONTROL1_REG,
			data << CE_SHIFT, CE_MASK);
}

static int fan54005_get_en_hiz(struct fan54005 *chip)
{
	u8 data;
	int rc;

	rc = fan54005_read(chip->client, CONTROL1_REG, &data);
	if (rc)
		return rc;

	data &= HZ_MODE_MASK;
	data >>= HZ_MODE_SHIFT;

	return data;
}

static int fan54005_set_en_hiz(struct fan54005 *chip, int en)
{
	u8 data = 0;

	if (en)
		data = 1;

	return fan54005_masked_write(chip->client, CONTROL1_REG,
			data << HZ_MODE_SHIFT, HZ_MODE_MASK);
}


static int fan54005_set_opa_mode(struct fan54005 *chip, int mode)
{
	u8 data;

	if (mode)
		data = 1;
	else
		data = 0;

	return fan54005_masked_write(chip->client, CONTROL1_REG,
			data << OPA_MODE_SHIFT, OPA_MODE_MASK);
}

static int fan54005_get_oreg(struct fan54005 *chip)
{
	u8 data;
	int mv;
	int rc;

	rc = fan54005_read(chip->client, OREG_REG, &data);
	if (rc)
		return rc;

	data = (data & OREG_MASK) >> OREG_SHIFT;

	mv = 3500 + (data * 20);
	if (mv > 4440)
		mv = 4440;

	return mv;
}

static int fan54005_set_oreg(struct fan54005 *chip, int mv)
{
	u8 data;

	if (mv < 3500)
		data = 0;

	data = (mv - 3500) / 20;
	if (mv < 3500)
		data = 0;
	if (mv > 4440)
		data = 0x2F;

	return fan54005_masked_write(chip->client, OREG_REG,
			data << OREG_SHIFT, OREG_MASK);
}

static int fan54005_get_vendor_code(struct fan54005 *chip)
{
	u8 data;
	int rc;

	rc = fan54005_read(chip->client, IC_INFO_REG, &data);
	if (rc)
		return rc;

	data &= VENDOR_MASK;
	data >>= VENDOR_SHIFT;

	return data;
}

static int fan54005_get_part_number(struct fan54005 *chip)
{
	u8 data;
	int rc;

	rc = fan54005_read(chip->client, IC_INFO_REG, &data);
	if (rc)
		return rc;

	data &= PN_MASK;
	data >>= PN_SHIFT;

	return data;
}

static int fan54005_get_revision(struct fan54005 *chip)
{
	u8 data;
	int rc;

	rc = fan54005_read(chip->client, IC_INFO_REG, &data);
	if (rc)
		return rc;

	data &= REV_MASK;
	data >>= REV_SHIFT;

	return data;
}

static int fan54005_get_io_level(struct fan54005 *chip)
{
	u8 data;
	int rc;

	rc = fan54005_read(chip->client, SP_CHARGER_REG, &data);
	if (rc)
		return rc;

	data &= IO_LEVEL_MASK;
	data >>= IO_LEVEL_SHIFT;

	return data;
}

static int fan54005_set_io_level(struct fan54005 *chip, int en)
{
	u8 data = 0;

	if (en)
		data = 1;

	return fan54005_masked_write(chip->client, SP_CHARGER_REG,
			data << IO_LEVEL_SHIFT, IO_LEVEL_MASK);
}

static int fan54005_get_iocharge(struct fan54005 *chip)
{
	const int *iocharge = iocharge_68ohm;
	u8 data;
	int rc;

	if (chip->rsense == 100)
		iocharge = iocharge_100ohm;

	rc = fan54005_read(chip->client, IBAT_REG, &data);
	if (rc)
		return rc;

	data = (data & IOCHARGE_MASK) >> IOCHARGE_SHIFT;

	return iocharge[data];
}

static int fan54005_set_iocharge(struct fan54005 *chip, int ma)
{
	const int *iocharge = iocharge_68ohm;
	u8 data;

	if (chip->rsense == 100)
		iocharge = iocharge_100ohm;

	for (data = 1; data < ARRAY_SIZE(iocharge_68ohm); data++) {
		if (ma < iocharge[data])
			break;
	}

	data--;

	return fan54005_masked_write(chip->client, IBAT_REG,
			data << IOCHARGE_SHIFT, IOCHARGE_MASK);
}

static int fan54005_get_iterm(struct fan54005 *chip)
{
	const int *iterm = iterm_68ohm;
	u8 data;
	int rc;

	if (chip->rsense == 100)
		iterm = iterm_100ohm;

	rc = fan54005_read(chip->client, IBAT_REG, &data);
	if (rc)
		return rc;

	data = (data & ITERM_MASK) >> ITERM_SHIFT;

	return iterm[data];
}

static int fan54005_set_iterm(struct fan54005 *chip, int ma)
{
	const int *iterm = iterm_68ohm;
	u8 data;

	if (chip->rsense == 100)
		iterm = iterm_100ohm;

	for (data = 0; data < ARRAY_SIZE(iterm_68ohm); data++) {
		if (ma <= iterm[data])
			break;
	}
	if (data > 7)
		data = 7;

	return fan54005_masked_write(chip->client, IBAT_REG,
			data << ITERM_SHIFT, ITERM_MASK);
}

static int fan54005_get_vsp(struct fan54005 *chip)
{
	u8 data;
	int rc;

	rc = fan54005_read(chip->client, SP_CHARGER_REG, &data);
	if (rc)
		return rc;

	data = (data & VSP_MASK) >> VSP_SHIFT;

	return vsp[data];
}

static int fan54005_set_vsp(struct fan54005 *chip, int mv)
{
	u8 data;

	for(data = 0; data < ARRAY_SIZE(vsp); data++) {
		if (mv <= vsp[data])
			break;
	}
	if (data > 7)
		data = 7;

	return fan54005_masked_write(chip->client, SP_CHARGER_REG,
			data << VSP_SHIFT, VSP_MASK);
}

static int fan54005_get_isafe(struct fan54005 *chip)
{
	const int *isafe = isafe_68ohm;
	u8 data;
	int rc;

	if (chip->rsense == 100)
		isafe = isafe_100ohm;

	rc = fan54005_read(chip->client, SAFETY_REG, &data);
	if (rc)
		return rc;

	data = (data & ISAFE_MASK) >> ISAFE_SHIFT;

	return isafe[data];
}

static int fan54005_get_vsafe(struct fan54005 *chip)
{
	u8 data;
	int rc;
	int mv;

	rc = fan54005_read(chip->client, SAFETY_REG, &data);
	if (rc)
		return rc;

	data = (data & VSAFE_MASK) >> VSAFE_SHIFT;
	mv = 4200 + (data * 20);
	if (mv > 4440)
		mv = 4440;

	return mv;
}

static int fan54005_set_isafe_vsafe(struct fan54005 *chip, int isafe_ma, int vsafe_mv)
{
	u8 data;
	u8 data_i;
	u8 data_v;

	const int *isafe = isafe_68ohm;

	if (chip->rsense == 100)
		isafe = isafe_100ohm;

	for (data_i = 0; data_i < ARRAY_SIZE(isafe_68ohm); data_i++) {
		if (isafe_ma <= isafe[data_i])
			break;
	}

	if (data_i > 7)
		data_i = 7;

	data_v = (vsafe_mv - 4200) / 20;
	if (vsafe_mv < 4200)
		data_v = 0;
	if (vsafe_mv > 4440)
		data_v = 0x0C;

	data = (data_i << ISAFE_SHIFT) | data_v;

	return fan54005_write(chip->client, SAFETY_REG, data);
}

static int fan54005_get_vbus_valid(struct fan54005 *chip)
{
	u8 data;
	int rc;

	rc = fan54005_read(chip->client, MONITOR_REG, &data);
	if (rc)
		return rc;

	data &= VBUS_VALID_MASK;
	data >>= VBUS_VALID_SHIFT;

	return data;
}

static int fan54005_dump_register(struct fan54005 *chip)
{
	u8 data;
	int rc;
	int i;
	char* buf = chip->buffer;

	buf[0] = '\0';

	snprintf(buf + strnlen(buf, DUMP_SIZE), DUMP_SIZE,
		 "fan54005_dump_register : ");

	for (i = 0; i < ARRAY_SIZE(regs_to_dump); i++) {
		rc = fan54005_read(chip->client, regs_to_dump[i], &data);
		if (rc) {
			dev_err(chip->dev, "dump_err: 0x%02x is error\n", regs_to_dump[i]);
			continue;
		}
		snprintf(buf + strnlen(buf, DUMP_SIZE), DUMP_SIZE, "0x%02x=0x%02x ", regs_to_dump[i], data);
	}
	dev_info(chip->dev, "%s\n", buf);

	return 0;
}

static int fan54005_online(struct fan54005 *chip)
{
	if (fan54005_get_vbus_valid(chip) > 0)
		return 1;

	return 0;
}

static void fan54005_reset_timer_work(struct work_struct *work)
{
	struct fan54005 *chip = container_of(to_delayed_work(work),
			struct fan54005, reset_timer);
	bool reschedule = false;

	fan54005_tmr_rst(chip);

	if (fan54005_get_vbus_valid(chip))
		reschedule = true;
	if (fan54005_get_boost(chip))
		reschedule = true;

	if (!reschedule) {
		dev_info(chip->dev, "stop kicking wdt\n");
		return;
	}

	schedule_delayed_work(&chip->reset_timer,
			msecs_to_jiffies(TMR_RST_MSECS));
}

static void fan54005_reg_init_work(struct work_struct *work)
{
	struct fan54005 *chip = container_of(to_delayed_work(work),
			struct fan54005, reg_init);
	int rc = 0;

	dev_info(chip->dev, "initializing registers\n");

	rc = fan54005_set_iterm(chip, chip->iterm_ma);
	if (rc)
		goto retry;

	rc = fan54005_set_vsp(chip, chip->vsp_mv);
	if (rc)
		goto retry;

	rc = fan54005_set_te(chip, chip->te);
	if (rc)
		goto retry;

	rc = fan54005_set_io_level(chip, 0);
	if (rc)
		goto retry;

	return;

retry:
	dev_info(chip->dev, "re-initializing registers after 1sec\n");
	schedule_delayed_work(&chip->reg_init, msecs_to_jiffies(1000));
}

static irqreturn_t fan54005_irq_handler(int irq, void *data)
{
	struct fan54005 *chip = data;
	int stat;
	int fault;
	int mode;

	stat = fan54005_get_stat(chip);
	if (stat < 0) {
		dev_err(chip->dev, "failed to get stat\n");
		return IRQ_HANDLED;
	}

	dev_info(chip->dev, "stat = %s\n", stat_str[stat]);
	if (stat == STAT_READY)
		return IRQ_HANDLED;

	if (stat != STAT_FAULT) {
		schedule_delayed_work(&chip->reset_timer,
				msecs_to_jiffies(TMR_RST_MSECS));
		return IRQ_HANDLED;
	}

	fault = fan54005_get_fault(chip);
	mode = fan54005_get_boost(chip);
	dev_err(chip->dev, "fault in %s mode = %s\n",
			mode ? "boost" : "charge",
			(mode ? boost_fault_str[fault] :
				charge_fault_str[fault]));

	/* re-init charge configuration */
	if (fault == CHARGE_FAULT_TIMER)
		schedule_delayed_work(&chip->reg_init, 0);

	return IRQ_HANDLED;
}

static enum power_supply_property fan54005_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
};

static int fan54005_status(struct fan54005 *chip)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;

	/* TODO : get status and convert to POWER_SUPPLY */
	if (!fan54005_get_vbus_valid(chip))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	switch (fan54005_get_stat(chip)) {
	case 0:
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case 1:
		status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case 2:
		status = POWER_SUPPLY_STATUS_FULL;
		break;
	case 3:
		status = POWER_SUPPLY_STATUS_UNKNOWN;
		dev_info(chip->dev, "chg_stat: fault\n");
		break;
	default:
		break;
	}

	return status;
}

static int fan54005_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct fan54005 *chip = container_of(psy, struct fan54005, psy);
	int rc = 0;

	switch(psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fan54005_status(chip);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = fan54005_online(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = fan54005_get_iocharge(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = fan54005_get_iinlim(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = fan54005_get_oreg(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = fan54005_get_en_hiz(chip);
		if (val->intval)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = fan54005_get_ce(chip);
		if (val->intval)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = fan54005_get_iterm(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGER_DUMP:
		val->intval = fan54005_dump_register(chip);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int fan54005_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct fan54005 *chip = container_of(psy, struct fan54005, psy);
	int rc = 0;

	switch(psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		fan54005_set_iocharge(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		fan54005_set_iinlim(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		fan54005_set_oreg(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		fan54005_set_en_hiz(chip, val->intval ? 0 : 1);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		fan54005_set_ce(chip, val->intval ? 0 : 1);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		chip->iterm_ma = val->intval / 1000;
		fan54005_set_iterm(chip, val->intval / 1000);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

int fan54005_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int rc = 0;

	switch(psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		rc = 1;
		break;
	default:
		break;
	}

	return rc;
}

static int fan54005_otg_regulator_enable(struct regulator_dev *rdev)
{
	struct fan54005 *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	chip->otg_boost_enabled = true;

	/* TODO : turn on boost */
	fan54005_set_en_hiz(chip, 0);
	fan54005_set_opa_mode(chip, 1);

	schedule_delayed_work(&chip->reset_timer, 0);

	return rc;
}

static int fan54005_otg_regulator_disable(struct regulator_dev *rdev)
{
	struct fan54005 *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	/* TODO : turn off boost */
	fan54005_set_opa_mode(chip, 0);
	cancel_delayed_work(&chip->reset_timer);

	chip->otg_boost_enabled = false;

	return rc;
}

static int fan54005_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct fan54005 *chip = rdev_get_drvdata(rdev);
	int enabled;

	enabled = fan54005_get_boost(chip);
	if (enabled < 0)
		enabled = 0;

	return enabled;
}

struct regulator_ops fan54005_otg_regulator_ops = {
	.enable		= fan54005_otg_regulator_enable,
	.disable	= fan54005_otg_regulator_disable,
	.is_enabled	= fan54005_otg_regulator_is_enable,
};

static int fan54005_regulator_init(struct fan54005 *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &fan54005_otg_regulator_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		chip->otg_vreg.rdev =
			devm_regulator_register(chip->dev, &chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev, "failed to register regulator, rc=%d\n", rc);
		}
	}

	return rc;
}

static int debugfs_get_data(void *data, u64 *val)
{
	struct fan54005 *chip = data;
	int rc;
	u8 temp;

	rc = fan54005_read(chip->client, chip->debug_addr, &temp);
	if (rc)
		return -EAGAIN;

	*val = temp;

	return 0;
}

static int debugfs_set_data(void *data, u64 val)
{
	struct fan54005 *chip = data;
	int rc;
	u8 temp;

	temp = (u8)val;
	rc = fan54005_write(chip->client, chip->debug_addr, temp);
	if (rc)
		return -EAGAIN;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(data_debugfs_ops,
	debugfs_get_data, debugfs_set_data, "0x%02llx\n");

static int dump_debugfs_show(struct seq_file *m, void *start)
{
	struct fan54005 *chip = m->private;
	u8 data;
	int rc;
	int i;

	for (i = 0; i < ARRAY_SIZE(regs_to_dump); i++) {
		rc = fan54005_read(chip->client, regs_to_dump[i], &data);
		if (rc) {
			seq_printf(m, "0x%02x=error\n", regs_to_dump[i]);
		}

		seq_printf(m, "0x%02x=0x%02x\n", regs_to_dump[i], data);
	}

	return 0;
}

static int dump_debugfs_open(struct inode *inode, struct file *file)
{
	struct fan54005 *chip = inode->i_private;

	return single_open(file, dump_debugfs_show, chip);
}

static const struct file_operations dump_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= dump_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int info_debugfs_show(struct seq_file *m, void *start)
{
	struct fan54005 *chip = m->private;
	int mode = fan54005_get_boost(chip);
	int fault = fan54005_get_fault(chip);

	seq_printf(m, "--- Chip ---\n");
	seq_printf(m, "Vendor Code = 0x%02x\n", fan54005_get_vendor_code(chip));
	seq_printf(m, "Part Number = 0x%02x\n", fan54005_get_part_number(chip));
	seq_printf(m, "Revision = 0x%02x\n", fan54005_get_revision(chip));
	seq_printf(m, "\n");
	seq_printf(m, "--- Enable ---\n");
	seq_printf(m, "(n)Charger Enable = %d\n", fan54005_get_ce(chip));
	seq_printf(m, "(n)Hi-Z = %d\n", fan54005_get_en_hiz(chip));
	seq_printf(m, "\n");

	/* Boost Mode */
	if (mode == 1) {
		seq_printf(m, "--- Boost Mode ---\n");
		seq_printf(m, "Fault = %s\n", boost_fault_str[fault]);
		return 0;
	}

	/* Charge Mode */
	seq_printf(m, "--- Charge Mode ---\n");
	seq_printf(m, "Valid Vbus = %s\n",
			fan54005_get_vbus_valid(chip) ? "Yes" : "No");
	seq_printf(m, "IO Level = %s\n",
			fan54005_get_io_level(chip) ? "Default" : "by IOCHARGE");
	seq_printf(m, "Input Current = %dmA\n", fan54005_get_iinlim(chip));
	seq_printf(m, "Charge Current = %dmA (safe = %dmA)\n",
			fan54005_get_iocharge(chip),
			fan54005_get_isafe(chip));
	seq_printf(m, "Float Voltage = %dmV (safe = %dmV)\n",
			fan54005_get_oreg(chip),
			fan54005_get_vsafe(chip));
	seq_printf(m, "Termination Current = %dmV\n", fan54005_get_iterm(chip));
	seq_printf(m, "Termination Enabled = %s\n",
			fan54005_get_te(chip) ? " Yes" : "No");
	seq_printf(m, "Vsp = %dmV\n", fan54005_get_vsp(chip));
	seq_printf(m, "Stat = %s\n", stat_str[fan54005_get_stat(chip)]);
	seq_printf(m, "Fault = %s\n", charge_fault_str[fault]);

	return 0;
}

static int info_debugfs_open(struct inode *inode, struct file *file)
{
	struct fan54005 *chip = inode->i_private;

	return single_open(file, info_debugfs_show, chip);
}

static const struct file_operations info_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= info_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int create_debugfs_entries(struct fan54005 *chip)
{
	struct dentry *ent;

	chip->debugfs = debugfs_create_dir(chip->psy.name, NULL);
	if (!chip->debugfs) {
		dev_err(chip->dev, "failed to create debugfs\n");
		return -ENODEV;
	}

	ent = debugfs_create_x32("addr", S_IFREG | S_IWUSR | S_IRUGO,
		chip->debugfs, &chip->debug_addr);
	if (!ent)
		dev_err(chip->dev, "failed to create addr debugfs\n");

	ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
		chip->debugfs, chip, &data_debugfs_ops);
	if (!ent)
		dev_err(chip->dev, "failed to create data debugfs\n");

	ent = debugfs_create_file("dump", S_IFREG | S_IRUGO,
		chip->debugfs, chip, &dump_debugfs_ops);
	if (!ent)
		dev_err(chip->dev, "failed to create dump debugfs\n");

	ent = debugfs_create_file("info", S_IFREG | S_IRUGO,
		chip->debugfs, chip, &info_debugfs_ops);
	if (!ent)
		dev_err(chip->dev, "failed to create info debugfs\n");

	return 0;
}

static int fan54005_hw_init(struct fan54005 *chip)
{
	int rc;

	rc = devm_gpio_request_one(chip->dev, chip->irq_gpio, GPIOF_DIR_IN,
				"fan54005");
	if (rc)
		dev_err(chip->dev, "failed to request gpio, rc=%d\n", rc);

	/* call isafe / vsafe twice to set safety bits */
	fan54005_set_isafe_vsafe(chip, chip->isafe_ma, chip->vsafe_mv);
	fan54005_set_isafe_vsafe(chip, chip->isafe_ma, chip->vsafe_mv);

	chip->te = fan54005_get_te(chip);

	schedule_delayed_work(&chip->reg_init, msecs_to_jiffies(0));
	schedule_delayed_work(&chip->reset_timer,
			msecs_to_jiffies(TMR_RST_MSECS));

	return rc;
}

static int fan54005_parse_dt(struct fan54005 *chip)
{
	struct device_node *np = chip->dev->of_node;
	int rc = 0;

	if (!np)
		return -ENODEV;

	chip->is_slave = of_property_read_bool(np, "slave-charger");
	chip->irq_gpio = of_get_named_gpio_flags(np, "irq-gpio", 0, NULL);
	if (!gpio_is_valid(chip->irq_gpio)) {
		dev_err(chip->dev, "failed to read irq-gpio\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "rsense", &chip->rsense);
	if (rc)
		chip->rsense = 68;

	rc = of_property_read_u32(np, "iterm", &chip->iterm_ma);
	if (rc)
		chip->iterm_ma = 388;

	rc = of_property_read_u32(np, "vsp", &chip->vsp_mv);
	if (rc)
		chip->vsp_mv = 4533;

	rc = of_property_read_u32(np, "isafe", &chip->isafe_ma);
	if (rc)
		chip->isafe_ma = 1450;

	rc = of_property_read_u32(np, "vsafe", &chip->vsafe_mv);
	if (rc)
		chip->vsafe_mv = 4400;

	return rc;
}

static int fan54005_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct fan54005 *chip;
	int rc;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);

	chip->otg_boost_enabled = false;
	mutex_init(&chip->lock);

	INIT_DELAYED_WORK(&chip->reset_timer, fan54005_reset_timer_work);
	INIT_DELAYED_WORK(&chip->reg_init, fan54005_reg_init_work);

	chip->psy.name = "fan54005";
	chip->psy.type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->psy.properties = fan54005_properties;
	chip->psy.num_properties = ARRAY_SIZE(fan54005_properties);
	chip->psy.get_property = fan54005_get_property;
	chip->psy.set_property = fan54005_set_property;
	chip->psy.property_is_writeable = fan54005_property_is_writeable;

	rc = fan54005_parse_dt(chip);
	if (rc)
		return rc;

	if (fan54005_get_vendor_code(chip) != VENDOR_CODE &&
			fan54005_get_part_number(chip) != PART_NUMBER) {
		dev_err(chip->dev, "do not probe. fan54005 not exist.\n");
		return -ENODEV;
	}

	if (!chip->is_slave) {
		rc = fan54005_regulator_init(chip);
		if (rc)
			return rc;
	}

	rc = fan54005_hw_init(chip);
	if (rc)
		return rc;

	rc = devm_request_threaded_irq(chip->dev, gpio_to_irq(chip->irq_gpio),
			NULL, fan54005_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "fan54005", chip);
	if (rc) {
		dev_err(chip->dev, "failed to request irq, rc=%d\n", rc);
		return rc;
	}

	rc = power_supply_register(chip->dev, &chip->psy);
	if (rc) {
		dev_err(chip->dev, "failed to register power_supply, rc=%d\n", rc);
		return rc;
	}

	create_debugfs_entries(chip);

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
	chr_control_register(&chip->psy);
#endif

	return rc;
}

static int fan54005_remove(struct i2c_client *client)
{
	struct fan54005 *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->psy);

	return 0;
}

static void fan54005_shutdown(struct i2c_client *client)
{
	struct fan54005 *chip = i2c_get_clientdata(client);

	if (chip->otg_boost_enabled) {
		/* TODO : force off boost here */
		fan54005_set_opa_mode(chip, 0);
		chip->otg_boost_enabled = false;
	}

	return;
}

static const struct of_device_id fan54005_of_match[] = {
	{
		.compatible = "fairchild,fan54005",
	},
};

static const struct i2c_device_id fan54005_i2c_id[] = {
	{
		.name = "fan54005",
		.driver_data = 0,
	},
};

static struct i2c_driver fan54005_driver = {
	.probe = fan54005_probe,
	.remove = fan54005_remove,
	.shutdown = fan54005_shutdown,
	.driver = {
		.name = "fan54005",
		.of_match_table = fan54005_of_match,
	},
	.id_table = fan54005_i2c_id,
};

static void async_fan54005_init(void *data, async_cookie_t cookie)
{
	i2c_add_driver(&fan54005_driver);
	return;
}

static int __init fan54005_init(void)
{
	return async_schedule(async_fan54005_init, NULL);
}

static void __exit fan54005_exit(void)
{
	i2c_del_driver(&fan54005_driver);
}

module_init(fan54005_init);
module_exit(fan54005_exit);
