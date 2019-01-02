/* drivers/input/misc/ak0991x-i2c.c - AK0991X compass driver
 *
 * Copyright (C) 2014 ASAHI KASEI MICRODEVICES CORPORATION.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*#define DEBUG*/
/*#define VERBOSE_DEBUG*/

#include "ak0991x.h"
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/types.h>
#include "ak0991x_input.h"

#include <asm/atomic.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <hwmsensor.h>
#include <cust_mag.h>

/***** I2C Tx/Rx operation ******************************************/
static int aki2c_rxdata(struct device *dev, unsigned char *rxdata,
			int length)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = rxdata,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxdata,
		},
	};

	ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msgs)) {
		dev_err(dev, "%s: transfer failed(size error).",
			__func__);
		return -ENXIO;
	}

	dev_vdbg(dev, "RxData: len=%02x, addr=%02x  data=%02x",
		 length, rxdata[0], rxdata[1]);
	return 0;
}

static int aki2c_txdata(struct device *dev, unsigned char *txdata,
			int length)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msg)) {
		dev_err(dev, "%s: transfer failed(size error).",
			__func__);
		return -ENXIO;
	}

	dev_vdbg(dev, "TxData: len=%02x, addr=%02x data=%02x",
		 length, txdata[0], txdata[1]);
	return 0;
}


static const struct ak0991x_bus_ops ak0991x_i2c_bops = {
	.bustype	= BUS_I2C,
	.rxdata		= aki2c_rxdata,
	.txdata		= aki2c_txdata,
};

/***** Probe/Remove function ****************************************/
static int ak0991x_i2c_probe(struct i2c_client *		client,
			     const struct i2c_device_id *	id)
{
	struct ak0991x_data *akm;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"%s: check_functionality failed.", __func__);
		return -ENODEV;
	}

	akm = ak0991x_probe(&client->dev, client->irq, &ak0991x_i2c_bops);
	if (IS_ERR(akm))
		return PTR_ERR(akm);

	/* Success */
	i2c_set_clientdata(client, akm);
	return 0;
}

static int ak0991x_i2c_remove(struct i2c_client *client)
{
	struct ak0991x_data *akm = i2c_get_clientdata(client);

	return ak0991x_remove(akm);
}

/***** Power management *********************************************/
static int ak0991x_i2c_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct ak0991x_data *akm = i2c_get_clientdata(i2c);

	return ak0991x_suspend(akm);
}

static int ak0991x_i2c_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct ak0991x_data *akm = i2c_get_clientdata(i2c);

	return ak0991x_resume(akm);
}

static const struct dev_pm_ops ak0991x_i2c_pops = {
	.suspend	= ak0991x_i2c_suspend,
	.resume		= ak0991x_i2c_resume,
};

/***** I2C interface ***********************************************/
static const struct i2c_device_id ak0991x_id[] = {
	{ AKM_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak0991x_id);

static struct i2c_driver ak0991x_i2c_driver = {
	.probe		= ak0991x_i2c_probe,
	.remove		= ak0991x_i2c_remove,
	.id_table	= ak0991x_id,
	.driver		= {
		.name	= AKM_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm	= &ak0991x_i2c_pops,
	},
};

#define AKM0991X_I2C_ADDRESS    0x18
static struct i2c_board_info __initdata i2c_akm09911 = { I2C_BOARD_INFO(AKM_DRIVER_NAME, (AKM0991X_I2C_ADDRESS >> 1)) };

static int akm_probe(struct platform_device *pdev)
{
	if (i2c_add_driver(&ak0991x_i2c_driver)) {
		printk("add driver error\n");
		return -1;
	}
	return 0;
}
static atomic_t dev_open_count;
static int akm_remove(struct platform_device *pdev)
{
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&ak0991x_i2c_driver);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id msensor_of_match[] = {
	{ .compatible = "mediatek,ak0991x", },
	{},
};
#endif

static struct platform_driver akm_sensor_driver = {
	.probe			= akm_probe,
	.remove			= akm_remove,
	.driver			= {
		.name		= MAG_PL_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = msensor_of_match,
#endif
	}
};

static int __init ak0991x_i2c_init(void)
{
	const char *name = "mediatek,ak0991x";
	// struct mag_hw *hw = get_cust_mag_hw();



	printk("%s: i2c_number=%d\n", __func__, hw->i2c_num);
	i2c_register_board_info(hw->i2c_num, &i2c_akm09911, 1);
	if (platform_driver_register(&akm_sensor_driver)) {
		printk("failed to register mag driver\n");
		return -ENODEV;
	}
	return 0;
}

static void __exit ak0991x_i2c_exit(void)
{
	platform_driver_unregister(&akm_sensor_driver);
}

module_init(ak0991x_i2c_init);
module_exit(ak0991x_i2c_exit);

MODULE_AUTHOR("Asahi Kasei Microdevices Corp. <multi-s@om.asahi-kasei.co.jp>");
MODULE_DESCRIPTION("AK0991X I2C compass driver");
MODULE_LICENSE("GPL");
