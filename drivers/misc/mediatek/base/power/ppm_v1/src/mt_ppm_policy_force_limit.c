/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>

#include "mt_ppm_internal.h"


#define Cluster0_fix_freq_game	3	/* 689 Mhz */
#define Cluster1_fix_freq_game	5	/* 871 Mhz */

/* #define GAME_MODE_FIX_CORE */
#ifdef GAME_MODE_FIX_CORE
#define Cluster0_fix_core_game	4
#define Cluster1_fix_core_game	0
#endif

static enum ppm_power_state ppm_forcelimit_get_power_state_cb(enum ppm_power_state cur_state);
static void ppm_forcelimit_update_limit_cb(enum ppm_power_state new_state);
static void ppm_forcelimit_status_change_cb(bool enable);
static void ppm_forcelimit_mode_change_cb(enum ppm_mode mode);

/* other members will init by ppm_main */
static struct ppm_policy_data forcelimit_policy = {
	.name			= __stringify(PPM_POLICY_FORCE_LIMIT),
	.lock			= __MUTEX_INITIALIZER(forcelimit_policy.lock),
	.policy			= PPM_POLICY_FORCE_LIMIT,
	.priority		= PPM_POLICY_PRIO_USER_SPECIFY_BASE,
	.get_power_state_cb	= ppm_forcelimit_get_power_state_cb,
	.update_limit_cb	= ppm_forcelimit_update_limit_cb,
	.status_change_cb	= ppm_forcelimit_status_change_cb,
	.mode_change_cb		= ppm_forcelimit_mode_change_cb,
};

struct ppm_forcelimit_data forcelimit_data = {
	.is_freq_limited_by_user = false,
	.is_core_limited_by_user = false,
	.is_fixed_for_game	= false,
};


/* MUST in lock */
bool ppm_forcelimit_is_policy_active(void)
{
	if (!forcelimit_data.is_freq_limited_by_user && !forcelimit_data.is_core_limited_by_user)
		return false;
	else
		return true;
}

static enum ppm_power_state ppm_forcelimit_get_power_state_cb(enum ppm_power_state cur_state)
{
	if (forcelimit_data.is_core_limited_by_user)
		return ppm_judge_state_by_force_limit(cur_state, forcelimit_data);
	else
		return cur_state;
}

static void ppm_forcelimit_update_limit_cb(enum ppm_power_state new_state)
{
	unsigned int i;
	struct ppm_policy_req *req = &forcelimit_policy.req;

	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_ver("@%s: forcelimit policy update limit for new state = %s\n",
		__func__, ppm_get_power_state_name(new_state));

	if (forcelimit_data.is_freq_limited_by_user || forcelimit_data.is_core_limited_by_user) {
		ppm_hica_set_default_limit_by_state(new_state, &forcelimit_policy);

		for (i = 0; i < req->cluster_num; i++) {
			req->limit[i].min_cpu_core = (forcelimit_data.limit[i].min_core_num == -1)
				? req->limit[i].min_cpu_core
				: forcelimit_data.limit[i].min_core_num;
			req->limit[i].max_cpu_core = (forcelimit_data.limit[i].max_core_num == -1)
				? req->limit[i].max_cpu_core
				: forcelimit_data.limit[i].max_core_num;
			req->limit[i].min_cpufreq_idx = (forcelimit_data.limit[i].min_freq_idx == -1)
				? req->limit[i].min_cpufreq_idx
				: forcelimit_data.limit[i].min_freq_idx;
			req->limit[i].max_cpufreq_idx = (forcelimit_data.limit[i].max_freq_idx == -1)
				? req->limit[i].max_cpufreq_idx
				: forcelimit_data.limit[i].max_freq_idx;
		}

		ppm_limit_check_for_force_limit(new_state, req, forcelimit_data);

		/* error check */
		for (i = 0; i < req->cluster_num; i++) {
			if (req->limit[i].max_cpu_core < req->limit[i].min_cpu_core)
				req->limit[i].min_cpu_core = req->limit[i].max_cpu_core;
			if (req->limit[i].max_cpufreq_idx > req->limit[i].min_cpufreq_idx)
				req->limit[i].min_cpufreq_idx = req->limit[i].max_cpufreq_idx;
		}
	}

	FUNC_EXIT(FUNC_LV_POLICY);
}

static void ppm_forcelimit_status_change_cb(bool enable)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_ver("@%s: forcelimit policy status changed to %d\n", __func__, enable);

	FUNC_EXIT(FUNC_LV_POLICY);
}

static void ppm_forcelimit_mode_change_cb(enum ppm_mode mode)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_ver("@%s: ppm mode changed to %d\n", __func__, mode);

	FUNC_EXIT(FUNC_LV_POLICY);
}

#if	0
static int ppm_forcelimit_min_cpu_core_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < forcelimit_policy.req.cluster_num; i++) {
		seq_printf(m, "cluster %d: min_core_num = %d, max_core_num = %d\n",
			i, forcelimit_data.limit[i].min_core_num, forcelimit_data.limit[i].max_core_num);
	}

	return 0;
}

static ssize_t ppm_forcelimit_min_cpu_core_proc_write(struct file *file, const char __user *buffer,
					size_t count,	loff_t *pos)
{
	int id, min_core, i;
	bool is_limit = false;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d", &id, &min_core) == 2) {
		if (id < 0 || id >= ppm_main_info.cluster_num) {
			ppm_err("@%s: Invalid cluster id: %d\n", __func__, id);
			goto out;
		}

		if (min_core != -1 && min_core < (int)get_cluster_min_cpu_core(id)) {
			ppm_err("@%s: Invalid input! min_core = %d\n", __func__, min_core);
			goto out;
		}

		ppm_lock(&forcelimit_policy.lock);

		if (!forcelimit_policy.is_enabled) {
			ppm_warn("@%s: forcelimit policy is not enabled!\n", __func__);
			ppm_unlock(&forcelimit_policy.lock);
			goto out;
		}

		/* error check */
		if (forcelimit_data.limit[id].max_core_num != -1
			&& min_core != -1
			&& min_core > forcelimit_data.limit[id].max_core_num) {
			ppm_warn("@%s: min_core(%d) > max_core(%d), sync to max core!\n",
				__func__, min_core, forcelimit_data.limit[id].max_core_num);
			min_core = forcelimit_data.limit[id].max_core_num;
		}

		if (min_core != forcelimit_data.limit[id].min_core_num) {
			forcelimit_data.limit[id].min_core_num = min_core;
			ppm_dbg(USER_LIMIT, "force limit min_core_num = %d for cluster %d\n", min_core, id);
		}

		/* check is core limited or not */
		for (i = 0; i < forcelimit_policy.req.cluster_num; i++) {
			if (forcelimit_data.limit[i].min_core_num != -1
				|| forcelimit_data.limit[i].max_core_num != -1) {
				is_limit = true;
				break;
			}
		}
		forcelimit_data.is_core_limited_by_user = is_limit;

		forcelimit_policy.is_activated = ppm_forcelimit_is_policy_active();

		ppm_unlock(&forcelimit_policy.lock);
		ppm_task_wakeup();
	} else
		ppm_err("@%s: Invalid input!\n", __func__);

out:
	free_page((unsigned long)buf);
	return count;
}

static int ppm_forcelimit_max_cpu_core_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < forcelimit_policy.req.cluster_num; i++) {
		seq_printf(m, "cluster %d: min_core_num = %d, max_core_num = %d\n",
			i, forcelimit_data.limit[i].min_core_num, forcelimit_data.limit[i].max_core_num);
	}

	return 0;
}

static ssize_t ppm_forcelimit_max_cpu_core_proc_write(struct file *file, const char __user *buffer,
					size_t count,	loff_t *pos)
{
	int id, max_core, i;
	bool is_limit = false;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d", &id, &max_core) == 2) {
		if (id < 0 || id >= ppm_main_info.cluster_num) {
			ppm_err("@%s: Invalid cluster id: %d\n", __func__, id);
			goto out;
		}

		if (max_core != -1 && max_core > (int)get_cluster_max_cpu_core(id)) {
			ppm_err("@%s: Invalid input! max_core = %d\n", __func__, max_core);
			goto out;
		}

#ifdef PPM_IC_SEGMENT_CHECK
		if (!max_core) {
			if ((id == 0 && ppm_main_info.fix_state_by_segment == PPM_POWER_STATE_LL_ONLY)
				|| (id == 1 && ppm_main_info.fix_state_by_segment == PPM_POWER_STATE_L_ONLY)) {
				ppm_err("@%s: Cannot disable cluster %d due to fix_state_by_segment is %s\n",
					__func__, id, ppm_get_power_state_name(ppm_main_info.fix_state_by_segment));
				goto out;
			}
		}
#endif

		ppm_lock(&forcelimit_policy.lock);

		if (!forcelimit_policy.is_enabled) {
			ppm_warn("@%s: forcelimit policy is not enabled!\n", __func__);
			ppm_unlock(&forcelimit_policy.lock);
			goto out;
		}

		/* error check */
		if (forcelimit_data.limit[id].min_core_num != -1
			&& max_core != -1
			&& max_core < forcelimit_data.limit[id].min_core_num) {
			ppm_warn("@%s: max_core(%d) < min_core(%d), overwrite min core!\n",
				__func__, max_core, forcelimit_data.limit[id].min_core_num);
			forcelimit_data.limit[id].min_core_num = max_core;
		}

		if (max_core != forcelimit_data.limit[id].max_core_num) {
			forcelimit_data.limit[id].max_core_num = max_core;
			ppm_dbg(USER_LIMIT, "force limit max_core_num = %d for cluster %d\n", max_core, id);
		}

		/* check is core limited or not */
		for (i = 0; i < forcelimit_policy.req.cluster_num; i++) {
			if (forcelimit_data.limit[i].min_core_num != -1
				|| forcelimit_data.limit[i].max_core_num != -1) {
				is_limit = true;
				break;
			}
		}
		forcelimit_data.is_core_limited_by_user = is_limit;

		forcelimit_policy.is_activated = ppm_forcelimit_is_policy_active();

		ppm_unlock(&forcelimit_policy.lock);
		ppm_task_wakeup();
	} else
		ppm_err("@%s: Invalid input!\n", __func__);

out:
	free_page((unsigned long)buf);
	return count;
}

static int ppm_forcelimit_min_cpu_freq_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < forcelimit_policy.req.cluster_num; i++) {
		seq_printf(m, "cluster %d: min_freq_idx = %d, max_freq_idx = %d\n",
			i, forcelimit_data.limit[i].min_freq_idx, forcelimit_data.limit[i].max_freq_idx);
	}

	return 0;
}

static ssize_t ppm_forcelimit_min_cpu_freq_proc_write(struct file *file, const char __user *buffer,
					size_t count, loff_t *pos)
{
	int id, min_freq, idx, i;
	bool is_limit = false;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d", &id, &min_freq) == 2) {
		if (id < 0 || id >= ppm_main_info.cluster_num) {
			ppm_err("@%s: Invalid cluster id: %d\n", __func__, id);
			goto out;
		}

		ppm_lock(&forcelimit_policy.lock);

		if (!forcelimit_policy.is_enabled) {
			ppm_warn("@%s: forcelimit policy is not enabled!\n", __func__);
			ppm_unlock(&forcelimit_policy.lock);
			goto out;
		}

		idx = (min_freq == -1) ? -1 : ppm_main_freq_to_idx(id, min_freq, CPUFREQ_RELATION_L);

		/* error check, sync to max idx if min freq > max freq */
		if (forcelimit_data.limit[id].max_freq_idx != -1
			&& idx != -1
			&& idx < forcelimit_data.limit[id].max_freq_idx)
			idx = forcelimit_data.limit[id].max_freq_idx;

		if (idx != forcelimit_data.limit[id].min_freq_idx) {
			forcelimit_data.limit[id].min_freq_idx = idx;
			ppm_dbg(USER_LIMIT, "force limit min_freq = %d KHz(idx = %d) for cluster %d\n",
				min_freq, idx, id);
		}

		/* check is freq limited or not */
		for (i = 0; i < forcelimit_policy.req.cluster_num; i++) {
			if (forcelimit_data.limit[i].min_freq_idx != -1
				|| forcelimit_data.limit[i].max_freq_idx != -1) {
				is_limit = true;
				break;
			}
		}
		forcelimit_data.is_freq_limited_by_user = is_limit;

		forcelimit_policy.is_activated = ppm_forcelimit_is_policy_active();

		ppm_unlock(&forcelimit_policy.lock);
		ppm_task_wakeup();
	} else
		ppm_err("@%s: Invalid input!\n", __func__);

out:
	free_page((unsigned long)buf);
	return count;
}

static int ppm_forcelimit_max_cpu_freq_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < forcelimit_policy.req.cluster_num; i++) {
		seq_printf(m, "cluster %d: min_freq_idx = %d, max_freq_idx = %d\n",
			i, forcelimit_data.limit[i].min_freq_idx, forcelimit_data.limit[i].max_freq_idx);
	}

	return 0;
}

static ssize_t ppm_forcelimit_max_cpu_freq_proc_write(struct file *file, const char __user *buffer,
					size_t count, loff_t *pos)
{
	int id, max_freq, idx, i;
	bool is_limit = false;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d", &id, &max_freq) == 2) {
		if (id < 0 || id >= ppm_main_info.cluster_num) {
			ppm_err("@%s: Invalid cluster id: %d\n", __func__, id);
			goto out;
		}

		ppm_lock(&forcelimit_policy.lock);

		if (!forcelimit_policy.is_enabled) {
			ppm_warn("@%s: forcelimit policy is not enabled!\n", __func__);
			ppm_unlock(&forcelimit_policy.lock);
			goto out;
		}

		idx = (max_freq == -1) ? -1 : ppm_main_freq_to_idx(id, max_freq, CPUFREQ_RELATION_H);

		/* error check, sync to max idx if max freq < min freq */
		if (forcelimit_data.limit[id].min_freq_idx != -1
			&& idx != -1
			&& idx > forcelimit_data.limit[id].min_freq_idx)
			forcelimit_data.limit[id].min_freq_idx = idx;

		if (idx != forcelimit_data.limit[id].max_freq_idx) {
			forcelimit_data.limit[id].max_freq_idx = idx;
			ppm_dbg(USER_LIMIT, "force limit max_freq = %d KHz(idx = %d) for cluster %d\n",
				max_freq, idx, id);
		}

		/* check is freq limited or not */
		for (i = 0; i < forcelimit_policy.req.cluster_num; i++) {
			if (forcelimit_data.limit[i].min_freq_idx != -1
				|| forcelimit_data.limit[i].max_freq_idx != -1) {
				is_limit = true;
				break;
			}
		}
		forcelimit_data.is_freq_limited_by_user = is_limit;

		forcelimit_policy.is_activated = ppm_forcelimit_is_policy_active();

		ppm_unlock(&forcelimit_policy.lock);
		ppm_task_wakeup();
	} else
		ppm_err("@%s: Invalid input!\n", __func__);

out:
	free_page((unsigned long)buf);
	return count;
}
#endif

static int ppm_forcelimit_fix_game_proc_show(struct seq_file *m, void *v)
{
	#if 1
	int i;

	for (i = 0; i < forcelimit_policy.req.cluster_num; i++) {
		seq_printf(m, "cluster %d: min_freq_idx = %d, max_freq_idx = %d\n",
			i, forcelimit_data.limit[i].min_freq_idx, forcelimit_data.limit[i].max_freq_idx);
	}
	#endif

	ppm_info("@%s: Game Mode: %d LL: (%d)(%d)(%d)(%d) L: (%d)(%d)(%d)(%d)\n", __func__,
		forcelimit_data.is_fixed_for_game,
		forcelimit_data.limit[0].min_freq_idx, forcelimit_data.limit[0].max_freq_idx,
		forcelimit_data.limit[0].min_core_num, forcelimit_data.limit[0].max_core_num,
		forcelimit_data.limit[1].min_freq_idx, forcelimit_data.limit[1].max_freq_idx,
		forcelimit_data.limit[1].min_core_num, forcelimit_data.limit[1].max_core_num);

	return 0;
}

static ssize_t ppm_forcelimit_fix_game_proc_write(struct file *file, const char __user *buffer,
					size_t count, loff_t *pos)
{
	unsigned int enabled;
	bool is_clear	= true;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf || !forcelimit_data.limit)
		return -EINVAL;

	if (!kstrtouint(buf, 10, &enabled))	{
		ppm_info("Request Game Mode Set: %d\n", enabled);
	} else {
		ppm_err(" Fail to parsing game mode\n");
		goto out;
	}

	/* if enable is set, freq. will fixed	*/
	/* if not, freq. should will be free	*/
	if (enabled)
		is_clear	= false;
	else
		is_clear	= true;

	ppm_lock(&forcelimit_policy.lock);

	if (!forcelimit_policy.is_enabled) {
		ppm_warn("@%s: forcelimit policy is not enabled!\n", __func__);
		ppm_unlock(&forcelimit_policy.lock);
		goto out;
	}

	if (is_clear) {
		forcelimit_data.is_freq_limited_by_user = false;
		forcelimit_data.is_fixed_for_game = false;

		forcelimit_data.limit[0].min_freq_idx	= -1;	//get_cluster_min_cpufreq_idx(0);
		forcelimit_data.limit[0].max_freq_idx	= -1;	//get_cluster_max_cpufreq_idx(0);

		forcelimit_data.limit[1].min_freq_idx	= -1;	//get_cluster_min_cpufreq_idx(1);
		forcelimit_data.limit[1].max_freq_idx	= -1;	//get_cluster_max_cpufreq_idx(1);

		forcelimit_data.is_core_limited_by_user	= false;

		forcelimit_data.limit[0].min_core_num	= 0;	//get_cluster_min_cpu_core(0);
		forcelimit_data.limit[0].max_core_num	= 4;	//get_cluster_max_cpu_core(0);

		forcelimit_data.limit[1].min_core_num	= 0;	//get_cluster_min_cpu_core(1);
		forcelimit_data.limit[1].max_core_num	= 4;	//get_cluster_max_cpu_core(1);
	} else {
		forcelimit_data.is_freq_limited_by_user = true;
		forcelimit_data.is_fixed_for_game = true;

		forcelimit_data.limit[0].min_freq_idx	= get_cluster_min_cpufreq_idx(0);
		forcelimit_data.limit[0].max_freq_idx	= Cluster0_fix_freq_game;

		forcelimit_data.limit[1].min_freq_idx	= get_cluster_min_cpufreq_idx(1);
		forcelimit_data.limit[1].max_freq_idx	= Cluster1_fix_freq_game;

		forcelimit_data.is_core_limited_by_user	= true;

		forcelimit_data.limit[0].min_core_num	= 0;	//get_cluster_min_cpu_core(0);
		#ifdef GAME_MODE_FIX_CORE
		forcelimit_data.limit[0].max_core_num	= Cluster0_fix_core_game;
		#else
		forcelimit_data.limit[0].max_core_num	= 4;	//Cluster0_fix_core_game;
		#endif

		forcelimit_data.limit[1].min_core_num	= 0;	//get_cluster_min_cpu_core(1);
		#ifdef GAME_MODE_FIX_CORE
		forcelimit_data.limit[1].max_core_num	= Cluster1_fix_core_game;
		#else
		forcelimit_data.limit[1].max_core_num	= 4;	//Cluster1_fix_core_game;
		#endif
	}

	forcelimit_policy.is_activated = ppm_forcelimit_is_policy_active();
	ppm_unlock(&forcelimit_policy.lock);
	ppm_task_wakeup();

out:
	free_page((unsigned long)buf);
	return count;
}

#if	0
PROC_FOPS_RW(forcelimit_min_cpu_core);
PROC_FOPS_RW(forcelimit_max_cpu_core);
PROC_FOPS_RW(forcelimit_min_cpu_freq);
PROC_FOPS_RW(forcelimit_max_cpu_freq);
#endif
PROC_FOPS_RW(forcelimit_fix_game);


static int __init ppm_forcelimit_policy_init(void)
{
	int i, ret = 0;

	struct pentry {
		const char *name;
		const struct file_operations *fops;
	};

	const struct pentry entries[] = {
		#if	0
		PROC_ENTRY(forcelimit_min_cpu_core),
		PROC_ENTRY(forcelimit_max_cpu_core),
		PROC_ENTRY(forcelimit_min_cpu_freq),
		PROC_ENTRY(forcelimit_max_cpu_freq),
		#endif
		PROC_ENTRY(forcelimit_fix_game),
	};

	FUNC_ENTER(FUNC_LV_POLICY);

	/* create procfs */
	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create(entries[i].name, S_IRUGO | S_IWUSR | S_IWGRP, policy_dir, entries[i].fops)) {
			ppm_err("%s(), create /proc/ppm/policy/%s failed\n", __func__, entries[i].name);
			ret = -EINVAL;
			goto out;
		}
	}

	forcelimit_data.limit = kcalloc(ppm_main_info.cluster_num, sizeof(*forcelimit_data.limit), GFP_KERNEL);
	if (!forcelimit_data.limit) {
		ret = -ENOMEM;
		goto out;
	}

	/* init forcelimit_data */
	for_each_ppm_clusters(i) {
		forcelimit_data.limit[i].min_freq_idx = -1;
		forcelimit_data.limit[i].max_freq_idx = -1;
		forcelimit_data.limit[i].min_core_num = -1;
		forcelimit_data.limit[i].max_core_num = -1;
	}

	if (ppm_main_register_policy(&forcelimit_policy)) {
		ppm_err("@%s: forcelimit policy register failed\n", __func__);
		kfree(forcelimit_data.limit);
		ret = -EINVAL;
		goto out;
	}

	ppm_info("@%s: register %s done!\n", __func__, forcelimit_policy.name);

out:
	FUNC_EXIT(FUNC_LV_POLICY);

	return ret;
}

static void __exit ppm_forcelimit_policy_exit(void)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	kfree(forcelimit_data.limit);

	ppm_main_unregister_policy(&forcelimit_policy);

	FUNC_EXIT(FUNC_LV_POLICY);
}

module_init(ppm_forcelimit_policy_init);
module_exit(ppm_forcelimit_policy_exit);

