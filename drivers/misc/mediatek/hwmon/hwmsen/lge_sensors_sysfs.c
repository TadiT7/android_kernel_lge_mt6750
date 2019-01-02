/* Copyright (c) 2016, LG Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>

#include <lge_sensors_sysfs.h>

#define ROOT_PATH_NAME "lge_sensors"
#define MODULE_NAME "sensors-sysfs"

#define DEBUG_ENABLE

static struct sensors_sysfs_array *arr;
static int arr_cnt;

static int sensors_check_mandatory_path(int arr_num)
{
	static int mandatory_max = sizeof(sensors_mandatory_paths) / sizeof(sensors_mandatory_paths[0]);
	int i;

	for (i = 0; i < mandatory_max; i++) {
		if ((!strcmp(arr[arr_num].group, sensors_mandatory_paths[i][0])) &&
				(!strcmp(arr[arr_num].user_node, sensors_mandatory_paths[i][1]))) {
			return 1;
		}
	}

	return 0;
}

#ifdef CONFIG_OF
static int sensors_sysfs_parse_dt(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	int ret;
	int i;

	arr_cnt = of_property_count_strings(node, "sysfs,node") / 3;
	if (arr_cnt <= 0) {
		pr_err("%s : ERROR sysfs node isn't exist. arr_cnt %d \n", __func__, arr_cnt);
		return 0;
	}

	pr_info("%s : Total sysfs node is %d\n", __func__, arr_cnt);

	arr = kzalloc(arr_cnt * sizeof(struct sensors_sysfs_array), GFP_KERNEL);
	if (arr == NULL) {
		pr_err("%s : ERROR get sysfs array\n", __func__);
		return -1;
	}

	for (i = 0; i < arr_cnt ; i++) {
		ret = of_property_read_string_index(node, "sysfs,node", 3*i, &arr[i].group);
		if (ret < 0) {
			pr_err("%s : ERROR get [%d] group\n", __func__, i);
			goto err_get_array;
		}

		ret = of_property_read_string_index(node, "sysfs,node", 3*i+1, &arr[i].user_node);
		if (ret < 0) {
			pr_err("%s : ERROR get [%d] user_node\n", __func__, i);
			goto err_get_array;
		}

		ret = of_property_read_string_index(node, "sysfs,node", 3*i+2, &arr[i].kernel_node);
		if (ret < 0) {
			pr_err("%s : ERROR get [%d] kernel_node\n", __func__, i);
			goto err_get_array;
		}

		if (sensors_check_mandatory_path(i)) {
			if (!strcmp(arr[i].kernel_node, "NULL")) {
				pr_err("%s : ERROR get mandatory path %s\n", __func__, arr[i].user_node);
				continue;
				//goto err_get_array;
			}
			pr_info("%s : %s %s is mandatory\n", __func__,
					arr[i].group,
					arr[i].user_node);
		}
	}

#ifdef DEBUG_ENABLE
	for (i = 0; i < arr_cnt; i++) {
		pr_info("%s : get [%d] node is %s, %s, %s\n", __func__,
				i,
				arr[i].group,
				arr[i].user_node,
				arr[i].kernel_node);
	}
#endif
	return 0;

err_get_array:
	kzfree(arr);
	return -1;
}
#else
static int sensors_sysfs_parse_dt(struct platform_device *pdev)
{
	pr_err("%s : ERROR CONFIG_OF isn't set\n", __func__);
	return -1;
}
#endif

static int sensors_sysfs_make_path(void)
{
	struct proc_dir_entry *p;
	int i;
	int last_group_table_index = 0;

	struct parent_entry_table {
		char group_name[256];
		struct proc_dir_entry *parent;
	};

	struct parent_entry_table *group_table;

	/* Set Sensors Sysfs root directory */
	p = proc_mkdir(ROOT_PATH_NAME, NULL);
	if (p == NULL) {
		pr_err("%s : ERROR make root sysfs \n", __func__);
		return -ENOMEM;
	}

	pr_info("%s : alloc group_table size %d", __func__, arr_cnt * sizeof(struct parent_entry_table));
	group_table = kzalloc(arr_cnt * sizeof(struct parent_entry_table), GFP_KERNEL);
	if (group_table == NULL) {
		pr_err("%s : ERROR kzalloc group_table failed\n", __func__);
		return -1;
	}
	memset(group_table, 0, arr_cnt * sizeof(struct parent_entry_table));

	for (i = 0; i < arr_cnt; i++) {
		int j;
		int exist = 0;

		for (j = 0; j < last_group_table_index; j++) {
			if (!strcmp(arr[i].group, group_table[j].group_name)) {
				arr[i].parent = group_table[j].parent;
				exist = 1;
				break;
			}
		}

		if (exist) {
			pr_info("%s : exist group entry %s (%s)\n", __func__, arr[i].group, arr[i].user_node);
			continue;
		}

		pr_info("%s : mkdir group entry %s (%s)\n", __func__, arr[i].group, arr[i].user_node);
		arr[i].parent = proc_mkdir(arr[i].group, p);
		if (arr[i].parent == NULL) {
			pr_err("%s : ERROR make %s group of %s node\n", __func__, arr[i].group, arr[i].user_node);
			continue;//return -ENOMEM;
		}

		strncpy(group_table[last_group_table_index].group_name, arr[i].group, 255);
		group_table[last_group_table_index].parent = arr[i].parent;
		last_group_table_index++;
	}

	/* Set Sensors Sysfs Path */
	for (i = 0; i < arr_cnt; i++) {
		struct proc_dir_entry *entry_ptr;

		if (!strcmp(arr[i].kernel_node, "NULL")) {
			pr_info("%s : %s user node didn't have kernel node\n", __func__, arr[i].user_node);
			continue;
		}

#ifdef DEBUG_ENABLE
		pr_info("%s : make [%d] sysfs path(%s, %s, %s)\n", __func__,
				i,
				arr[i].group,
				arr[i].user_node,
				arr[i].kernel_node);
#endif
		entry_ptr = proc_symlink(arr[i].user_node, arr[i].parent, arr[i].kernel_node);
		if (entry_ptr == NULL) {
			pr_err("%s : ERROR symlink [%d] sysfs path(%s, %s, %s)\n", __func__,
					i,
					arr[i].group,
					arr[i].user_node,
					arr[i].kernel_node);
			continue;//return -ENOMEM;
		}

	}

	pr_info("%s : free group_table", __func__);

	kzfree(group_table);
	return 0;

//err_make_path:
	//kzfree(group_table);
	//return -1;
}

static int sensors_read_sysfs_path(void)
{
	int i;

	arr_cnt = sizeof(default_sensors_sysfs_path) / sizeof(default_sensors_sysfs_path[0]);

	pr_info("%s : Total sysfs node is %d\n", __func__, arr_cnt);

	arr = kzalloc(arr_cnt * sizeof(struct sensors_sysfs_array), GFP_KERNEL);
	if (arr == NULL) {
		pr_err("%s : ERROR get sysfs array\n", __func__);
		return -1;
	}

	for (i = 0; i < arr_cnt; i++) {
		arr[i].group = default_sensors_sysfs_path[i][0];
		arr[i].user_node = default_sensors_sysfs_path[i][1];
		arr[i].kernel_node = default_sensors_sysfs_path[i][2];

		if (arr[i].kernel_node == NULL) {
			pr_err("%s : ERROR get [%d] kernel_node\n", __func__, i);
			goto err_get_array;
		}

		if (sensors_check_mandatory_path(i)) {
			if (!strcmp(arr[i].kernel_node, "NULL")) {
				pr_err("%s : ERROR get mandatory path %s\n", __func__, arr[i].user_node);
				continue;
				//goto err_get_array;
			}
			pr_info("%s : %s %s is mandatory\n", __func__, arr[i].group, arr[i].user_node);
		}
	}
#ifdef DEBUG_ENABLE
	for (i = 0; i < arr_cnt; i++) {
		pr_info("%s : get [%d] node is %s, %s, %s\n", __func__,
				i,
				arr[i].group,
				arr[i].user_node,
				arr[i].kernel_node);
	}
#endif
	return 0;

err_get_array:
	kzfree(arr);
	return -1;
}

static int sensors_sysfs_probe(struct platform_device *pdev)
{
	int ret;

	if (pdev->dev.of_node) {
		ret = sensors_sysfs_parse_dt(pdev);
		if (ret < 0) {
			pr_err("%s : ERROR parse DT\n", __func__);
			return ret;
		}
	} else {
		ret = sensors_read_sysfs_path();
		if (ret < 0) {
			pr_err("%s : ERROR read sysfs path\n", __func__);
			return ret;
		}
	}

	ret = sensors_sysfs_make_path();
	if (ret < 0) {
		pr_err("%s : ERROR make sysfs path\n", __func__);
		return ret;
	}

	pr_info("%s : Success sysfs Init\n", __func__);

	return ret;
}

static int sensors_sysfs_remove(struct platform_device *pdev)
{
	if (arr == NULL) {
		return 0;
	}

	// TODO: remove_proc_entry
	//pr_info("%s : remove entry %s\n", __func__, ROOT_PATH_NAME);
	//remove_proc_entry(ROOT_PATH_NAME, NULL);

	kzfree(arr);
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id sensors_sysfs_match_table[] = {
	{ .compatible = "lge,sensors-sysfs" },
	{ },
};
#endif

static struct platform_driver sensors_sysfs_driver = {
	.probe = sensors_sysfs_probe,
	.remove = sensors_sysfs_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sensors_sysfs_match_table,
#endif
	},
};

static int __init sensors_sysfs_init(void)
{
	return platform_driver_register(&sensors_sysfs_driver);
}

static void __exit sensors_sysfs_exit(void)
{
	platform_driver_unregister(&sensors_sysfs_driver);
}

late_initcall(sensors_sysfs_init);
module_exit(sensors_sysfs_exit);
MODULE_DESCRIPTION("LGE Sensors sysfs driver");
MODULE_LICENSE("GPL v2");
