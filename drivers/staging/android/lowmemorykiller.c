/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_score_adj values will get killed. Specify
 * the minimum oom_score_adj values in
 * /sys/module/lowmemorykiller/parameters/adj and the number of free pages in
 * /sys/module/lowmemorykiller/parameters/minfree. Both files take a comma
 * separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill
 * processes with a oom_score_adj value of 8 or higher when the free memory
 * drops below 4096 pages and kill processes with a oom_score_adj value of 0 or
 * higher when the free memory drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/swap.h>
#include <linux/rcupdate.h>
#include <linux/notifier.h>
#include <linux/freezer.h>
#include <linux/cpu.h>
#include <linux/fs.h>
#include <linux/sched/rt.h>

#ifdef CONFIG_HSWAP
#include <linux/delay.h>
#include <linux/kthread.h>
#include "../../block/zram/zram_drv.h"
#endif

#ifdef CONFIG_HIGHMEM
#define _ZONE ZONE_HIGHMEM
#else
#define _ZONE ZONE_NORMAL
#endif

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MT_ENG_BUILD)
#include <mt-plat/aee.h>
#include <disp_assert_layer.h>
static uint32_t in_lowmem;
#endif

#ifdef CONFIG_HIGHMEM
#include <linux/highmem.h>
#endif

#ifdef CONFIG_MTK_ION
#include "mtk/ion_drv.h"
#endif

#ifdef CONFIG_MTK_GPU_SUPPORT
#include <mt-plat/mtk_gpu_utility.h>
#endif

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
#define CONVERT_ADJ(x) ((x * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE)
#define REVERT_ADJ(x)  (x * (-OOM_DISABLE + 1) / OOM_SCORE_ADJ_MAX)
#else
#define CONVERT_ADJ(x) (x)
#define REVERT_ADJ(x) (x)
#endif /* CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES */

static int lmk_fast_run = 0;
static short lowmem_debug_adj = CONVERT_ADJ(0);
#ifdef CONFIG_MT_ENG_BUILD
#ifdef CONFIG_MTK_AEE_FEATURE
static short lowmem_kernel_warn_adj = CONVERT_ADJ(0);
#endif
#define output_expect(x) likely(x)
static uint32_t enable_candidate_log = 1;
#define LMK_LOG_BUF_SIZE 500
static uint8_t lmk_log_buf[LMK_LOG_BUF_SIZE];
#else
#define output_expect(x) unlikely(x)
static uint32_t enable_candidate_log;
#endif
static DEFINE_SPINLOCK(lowmem_shrink_lock);

#define CREATE_TRACE_POINTS
#include "trace/lowmemorykiller.h"

static uint32_t lowmem_debug_level = 1;
static short lowmem_adj[9] = {
	0,
	1,
	6,
	12,
};
static int lowmem_adj_size = 9;
int lowmem_minfree[9] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};
static int lowmem_minfree_size = 9;

#ifdef CONFIG_HIGHMEM
static int total_low_ratio = 1;
#endif

static int lmk_kill_cnt = 0;
#ifdef CONFIG_HSWAP
static int lmk_reclaim_cnt = 0;
#endif

static struct task_struct *lowmem_deathpending;
static unsigned long lowmem_deathpending_timeout;

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			pr_info(x);			\
	} while (0)

static int
task_notify_func(struct notifier_block *self, unsigned long val, void *data);

static struct notifier_block task_nb = {
	.notifier_call	= task_notify_func,
};

static int
task_notify_func(struct notifier_block *self, unsigned long val, void *data)
{
	struct task_struct *task = data;

	if (task == lowmem_deathpending)
		lowmem_deathpending = NULL;

	return NOTIFY_DONE;
}

static unsigned long lowmem_count(struct shrinker *s,
				  struct shrink_control *sc)
{
#ifdef CONFIG_FREEZER
	/* Do not allow LMK to work when system is freezing */
	if (pm_freezing)
		return 0;
#endif
	return global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);
}

int can_use_cma_pages(gfp_t gfp_mask)
{
	int can_use = 0;
	int mtype = gfpflags_to_migratetype(gfp_mask);
	int i = 0;
	int *mtype_fallbacks = get_migratetype_fallbacks(mtype);

	if (is_migrate_cma(mtype)) {
		can_use = 1;
	} else {
		for (i = 0;; i++) {
			int fallbacktype = mtype_fallbacks[i];

			if (is_migrate_cma(fallbacktype)) {
				can_use = 1;
				break;
			}

			if (fallbacktype == MIGRATE_RESERVE)
				break;
		}
	}
	return can_use;
}

void tune_lmk_zone_param(struct zonelist *zonelist, int classzone_idx,
					int *other_free, int *other_file,
					int use_cma_pages)
{
	struct zone *zone;
	struct zoneref *zoneref;
	int zone_idx;

	for_each_zone_zonelist(zone, zoneref, zonelist, MAX_NR_ZONES) {
		zone_idx = zonelist_zone_idx(zoneref);
		if (zone_idx == ZONE_MOVABLE) {
			if (!use_cma_pages && other_free)
				*other_free -=
				    zone_page_state(zone, NR_FREE_CMA_PAGES);
			continue;
		}

		if (zone_idx > classzone_idx) {
			if (other_free != NULL)
				*other_free -= zone_page_state(zone,
							       NR_FREE_PAGES);
			if (other_file != NULL)
				*other_file -= zone_page_state(zone,
							       NR_FILE_PAGES)
					- zone_page_state(zone, NR_SHMEM)
					- zone_page_state(zone, NR_SWAPCACHE);
		} else if (zone_idx < classzone_idx) {
			if (zone_watermark_ok(zone, 0, 0, classzone_idx, 0) &&
			    other_free) {
				if (!use_cma_pages) {
					*other_free -= min(
					  zone->lowmem_reserve[classzone_idx] +
					  zone_page_state(
					    zone, NR_FREE_CMA_PAGES),
					  zone_page_state(
					    zone, NR_FREE_PAGES));
				} else {
					*other_free -=
					  zone->lowmem_reserve[classzone_idx];
				}
			} else {
				if (other_free)
					*other_free -=
					  zone_page_state(zone, NR_FREE_PAGES);
			}
		}
	}
}

#ifdef CONFIG_HIGHMEM
void adjust_gfp_mask(gfp_t *gfp_mask)
{
	struct zone *preferred_zone;
	struct zonelist *zonelist;
	enum zone_type high_zoneidx;

	if (current_is_kswapd()) {
		zonelist = node_zonelist(0, *gfp_mask);
		high_zoneidx = gfp_zone(*gfp_mask);
		first_zones_zonelist(zonelist, high_zoneidx, NULL,
				&preferred_zone);

		if (high_zoneidx == ZONE_NORMAL) {
			if (zone_watermark_ok_safe(preferred_zone, 0,
					high_wmark_pages(preferred_zone), 0,
					0))
				*gfp_mask |= __GFP_HIGHMEM;
		} else if (high_zoneidx == ZONE_HIGHMEM) {
			*gfp_mask |= __GFP_HIGHMEM;
		}
	}
}
#else
void adjust_gfp_mask(gfp_t *unused)
{
}
#endif

void tune_lmk_param(int *other_free, int *other_file, struct shrink_control *sc)
{
	gfp_t gfp_mask;
	struct zone *preferred_zone;
	struct zonelist *zonelist;
	enum zone_type high_zoneidx, classzone_idx;
	unsigned long balance_gap;
	int use_cma_pages;

	gfp_mask = sc->gfp_mask;
	adjust_gfp_mask(&gfp_mask);

	zonelist = node_zonelist(0, gfp_mask);
	high_zoneidx = gfp_zone(gfp_mask);
	first_zones_zonelist(zonelist, high_zoneidx, NULL, &preferred_zone);
	classzone_idx = zone_idx(preferred_zone);
	use_cma_pages = can_use_cma_pages(gfp_mask);

	balance_gap = min(low_wmark_pages(preferred_zone),
			  (preferred_zone->present_pages +
			   KSWAPD_ZONE_BALANCE_GAP_RATIO-1) /
			   KSWAPD_ZONE_BALANCE_GAP_RATIO);

	if (likely(current_is_kswapd() && zone_watermark_ok(preferred_zone, 0,
			  high_wmark_pages(preferred_zone) + SWAP_CLUSTER_MAX +
			  balance_gap, 0, 0))) {
		if (lmk_fast_run)
			tune_lmk_zone_param(zonelist, classzone_idx, other_free,
				       other_file, use_cma_pages);
		else
			tune_lmk_zone_param(zonelist, classzone_idx, other_free,
				       NULL, use_cma_pages);

		if (zone_watermark_ok(preferred_zone, 0, 0, _ZONE, 0)) {
			if (!use_cma_pages) {
				*other_free -= min(
				  preferred_zone->lowmem_reserve[_ZONE]
				  + zone_page_state(
				    preferred_zone, NR_FREE_CMA_PAGES),
				  zone_page_state(
				    preferred_zone, NR_FREE_PAGES));
			} else {
				*other_free -=
				  preferred_zone->lowmem_reserve[_ZONE];
			}
		} else {
			*other_free -= zone_page_state(preferred_zone,
						      NR_FREE_PAGES);
		}

		lowmem_print(4, "lowmem_shrink of kswapd tunning for highmem "
			     "ofree %d, %d\n", *other_free, *other_file);
	} else {
		tune_lmk_zone_param(zonelist, classzone_idx, other_free,
			       other_file, use_cma_pages);

		if (!use_cma_pages) {
			*other_free -=
			  zone_page_state(preferred_zone, NR_FREE_CMA_PAGES);
		}

		lowmem_print(4, "lowmem_shrink tunning for others ofree %d, "
			     "%d\n", *other_free, *other_file);
	}
}

#ifdef CONFIG_HSWAP
static int test_task_flag(struct task_struct *p, int flag)
{
	struct task_struct *t;

	for_each_thread(p, t) {
		task_lock(t);
		if (test_tsk_thread_flag(t, flag)) {
			task_unlock(t);
			return 1;
		}
		task_unlock(t);
	}

	return 0;
}

static bool reclaim_task_is_ok(int selected_task_anon_size)
{
	int free_size = zram0_free_size() - get_lowest_prio_swapper_space_nrpages();

	if (selected_task_anon_size < free_size)
		return true;

	return false;
}

#define OOM_SCORE_SERVICE_B_ADJ 800
#define OOM_SCORE_CACHED_APP_MIN_ADJ 900

static DEFINE_MUTEX(reclaim_mutex);

static struct completion reclaim_completion;
static struct task_struct *selected_task;

#define RESET_TIME 3600000 /* activity top time reset time(msec) */
static int reset_task_time_thread(void *p)
{
	struct task_struct *tsk;

	while (1) {
		struct task_struct *p;

		rcu_read_lock();
		for_each_process(tsk) {
			if (tsk->flags & PF_KTHREAD)
				continue;

			/* if task no longer has any memory ignore it */
			if (test_task_flag(tsk, TIF_MEMDIE))
				continue;

			if (tsk->exit_state || !tsk->mm)
				continue;

			p = find_lock_task_mm(tsk);
			if (!p)
				continue;

			if (p->signal->top_time)
				p->signal->top_time =
					(p->signal->top_time * 3) / 4;

			task_unlock(p);
		}
		rcu_read_unlock();
		msleep(RESET_TIME);
	}
	return 0;
}

static int reclaim_task_thread(void *p)
{
	int selected_tasksize;
	struct reclaim_param rp;

	init_completion(&reclaim_completion);

	while (1) {
		wait_for_completion(&reclaim_completion);

		mutex_lock(&reclaim_mutex);
		if (!selected_task)
			goto reclaim_end;

		lowmem_print(3, "hswap: scheduled reclaim task '%s'(%d), adj%hd\n",
				selected_task->comm, selected_task->pid,
				selected_task->signal->oom_score_adj);

		task_lock(selected_task);
		if (selected_task->exit_state || !selected_task->mm) {
			task_unlock(selected_task);
			put_task_struct(selected_task);
			goto reclaim_end;
		}

		selected_tasksize = get_mm_rss(selected_task->mm);
		if (!selected_tasksize) {
			task_unlock(selected_task);
			put_task_struct(selected_task);
			goto reclaim_end;
		}

		task_unlock(selected_task);

		rp = reclaim_task_file_anon(selected_task, selected_tasksize);
		lowmem_print(3, "Reclaimed '%s' (%d), adj %hd,\n" \
				"   nr_reclaimed %d\n",
			     selected_task->comm, selected_task->pid,
			     selected_task->signal->oom_score_adj,
			     rp.nr_reclaimed);
		++lmk_reclaim_cnt;

		put_task_struct(selected_task);

reclaim_end:
		selected_task = NULL;

		init_completion(&reclaim_completion);
		mutex_unlock(&reclaim_mutex);
	}

	return 0;
}

#define RECLAIM_TASK_CNT 100
struct task_struct* reclaim_task[RECLAIM_TASK_CNT];

static struct task_struct *find_suitable_reclaim(int reclaim_cnt,
		int *rss_size)
{
	struct task_struct *selected = NULL;
	int selected_tasksize = 0;
	int tasksize, anonsize;
	long selected_top_time = -1;
	int i = 0;

	for (i = 0; i < reclaim_cnt; i++) {
		struct task_struct *p;

		p = reclaim_task[i];

		task_lock(p);
		if (p->exit_state || !p->mm) {
			task_unlock(p);
			continue;
		}

		tasksize = get_mm_rss(p->mm);
		anonsize = get_mm_counter(p->mm, MM_ANONPAGES);
		task_unlock(p);

		if (!tasksize)
			continue;

		if (!reclaim_task_is_ok(anonsize))
			continue;

		if (selected_tasksize > tasksize)
			continue;

		selected_top_time = p->signal->top_time;
		selected_tasksize = tasksize;
		selected = p;
	}

	*rss_size = selected_tasksize;

	return selected;
}

void reclaim_arr_free(int reclaim_cnt)
{
	int i;

	for (i = 0; i < reclaim_cnt; i++)
		reclaim_task[i] = NULL;
}
#endif

/*
 * It's reasonable to grant the dying task an even higher priority to
 * be sure it will be scheduled sooner and free the desired pmem.
 * It was suggested using SCHED_RR:1 (the lowest RT priority),
 * so that this task won't interfere with any running RT task.
 */
static void boost_dying_task_prio(struct task_struct *p)
{
	if (!rt_task(p)) {
		struct sched_param param;
		param.sched_priority = 1;
		sched_setscheduler_nocheck(p, SCHED_RR, &param);
	}
}

static unsigned long lowmem_scan(struct shrinker *s, struct shrink_control *sc)
{
	struct task_struct *tsk;
	struct task_struct *selected = NULL;
	unsigned long rem = 0;
	int tasksize;
	int i;
	short min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int minfree = 0;
	int selected_tasksize = 0;
	short selected_oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free = global_page_state(NR_FREE_PAGES);
	int other_file;
#ifdef CONFIG_HSWAP
	long selected_top_time = -1, cur_top_time;
	int reclaimed_cnt = 0, reclaim_cnt = 0;
	int hswap_tasksize = 0;
	int swapsize = 0, selected_swapsize = 0;
#endif

	int print_extra_info = 0;
	static unsigned long lowmem_print_extra_info_timeout;
#if defined(CONFIG_SWAP) && defined(CONFIG_MTK_GMO_RAM_OPTIMIZE)
	int to_be_aggressive = 0;
	unsigned long swap_pages = 0;
#endif
	bool in_cpu_hotplugging = false;

#ifdef CONFIG_MT_ENG_BUILD
	/* dump memory info when framework low memory*/
	int pid_dump = -1; /* process which need to be dump */
	/* int pid_sec_mem = -1; */
	int max_mem = 0;
	static int pid_flm_warn = -1;
	static unsigned long flm_warn_timeout;
	int log_offset = 0, log_ret;
#endif /* CONFIG_MT_ENG_BUILD*/
	/*
	* If we already have a death outstanding, then
	* bail out right away; indicating to vmscan
	* that we have nothing further to offer on
	* this pass.
	*
	*/
	if (lowmem_deathpending &&
	    time_before_eq(jiffies, lowmem_deathpending_timeout))
		return SHRINK_STOP;

	/* Check whether it is in cpu_hotplugging */
	in_cpu_hotplugging = cpu_hotplugging();

	/* Subtract CMA free pages from other_free if this is an unmovable page allocation */
	if (IS_ENABLED(CONFIG_CMA))
		if (!(sc->gfp_mask & __GFP_MOVABLE))
			other_free -= global_page_state(NR_FREE_CMA_PAGES);

#ifdef CONFIG_MIGRATE_HIGHORDER
	other_free -= global_page_state(NR_FREE_HIGHORDER_PAGES);
#endif

	if (!spin_trylock(&lowmem_shrink_lock)) {
		lowmem_print(4, "lowmem_shrink lock failed\n");
		return SHRINK_STOP;
	}

#ifdef CONFIG_HSWAP
	if (!mutex_trylock(&reclaim_mutex)) {
		spin_unlock(&lowmem_shrink_lock);
		return 0;
	}
	mutex_unlock(&reclaim_mutex);
#endif

	/* Let other_free be positive or zero */
	if (other_free < 0) {
		/* lowmem_print(1, "Original other_free [%d] is too low!\n", other_free); */
		other_free = 0;
	}

#if defined(CONFIG_64BIT) && defined(CONFIG_SWAP)
	/* Halve other_free if there is less free swap */
	if (vm_swap_full()) {
		lowmem_print(3, "Halve other_free %d\n", other_free);
		other_free >>= 1;
	}
#endif

	if (global_page_state(NR_SHMEM) + total_swapcache_pages() <
		global_page_state(NR_FILE_PAGES))
		other_file = global_page_state(NR_FILE_PAGES) -
			global_page_state(NR_SHMEM) -\
			total_swapcache_pages();
	else
		other_file = 0;

	tune_lmk_param(&other_free, &other_file, sc);

#if defined(CONFIG_SWAP) && defined(CONFIG_MTK_GMO_RAM_OPTIMIZE)
	swap_pages = atomic_long_read(&nr_swap_pages);
	/* More than 1/2 swap usage */
	if (swap_pages * 2 < total_swap_pages)
		to_be_aggressive++;
	/* More than 3/4 swap usage */
	if (swap_pages * 4 < total_swap_pages)
		to_be_aggressive++;
#endif

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
	for (i = 0; i < array_size; i++) {
		minfree = lowmem_minfree[i];
		if (other_free < minfree && other_file < minfree) {
#if defined(CONFIG_SWAP) && defined(CONFIG_MTK_GMO_RAM_OPTIMIZE)
			if (to_be_aggressive != 0 && i > 3) {
				i -= to_be_aggressive;
				if (i < 3)
					i = 3;
			}
#endif
			min_score_adj = lowmem_adj[i];
			break;
		}
	}

	/* If in CPU hotplugging, let LMK be more aggressive */
	if (in_cpu_hotplugging) {
		pr_alert("Aggressive LMK during CPU hotplug!\n");
		min_score_adj = 0;
	}

	lowmem_print(3, "lowmem_scan %lu, %x, ofree %d %d, ma %hd\n",
			sc->nr_to_scan, sc->gfp_mask, other_free,
			other_file, min_score_adj);

	if (min_score_adj == OOM_SCORE_ADJ_MAX + 1) {
		lowmem_print(5, "lowmem_scan %lu, %x, return 0\n",
			     sc->nr_to_scan, sc->gfp_mask);

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MT_ENG_BUILD)
		/*
		* disable indication if low memory
		*/
		if (in_lowmem) {
			in_lowmem = 0;
			/* DAL_LowMemoryOff(); */
			lowmem_print(1, "LowMemoryOff\n");
		}
#endif
		spin_unlock(&lowmem_shrink_lock);
		return 0;
	}

	selected_oom_score_adj = min_score_adj;

	/* add debug log */
	if (output_expect(enable_candidate_log)) {
		if (min_score_adj <= lowmem_debug_adj) {
			if (time_after_eq(jiffies, lowmem_print_extra_info_timeout)) {
				lowmem_print_extra_info_timeout = jiffies + HZ;
				print_extra_info = 1;
			}
		}

		if (print_extra_info) {
			lowmem_print(1, "Free memory other_free: %d, other_file:%d pages\n", other_free, other_file);
#ifdef CONFIG_MT_ENG_BUILD
			log_offset = snprintf(lmk_log_buf, LMK_LOG_BUF_SIZE, "%s",
#else
			lowmem_print(1,
#endif
#ifdef CONFIG_ZRAM
					"<lmk>  pid  adj  score_adj     rss   rswap name\n");
#else
					"<lmk>  pid  adj  score_adj     rss name\n");
#endif
		}
	}

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		short oom_score_adj;

		if (tsk->flags & PF_KTHREAD)
			continue;

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

#ifdef CONFIG_MT_ENG_BUILD
		if (p->signal->flags & SIGNAL_GROUP_COREDUMP) {
			task_unlock(p);
			continue;
		}
#endif

		if (test_tsk_thread_flag(p, TIF_MEMDIE) &&
		    time_before_eq(jiffies, lowmem_deathpending_timeout)) {
#ifdef CONFIG_MT_ENG_BUILD
			static pid_t last_dying_pid;

			if (last_dying_pid != p->pid) {
				lowmem_print(1, "lowmem_shrink return directly, due to  %d (%s) is dying\n",
					p->pid, p->comm);
				last_dying_pid = p->pid;
			}
#endif
			task_unlock(p);
#ifdef CONFIG_HSWAP
			goto end_lmk;
#else
			rcu_read_unlock();
			spin_unlock(&lowmem_shrink_lock);
			return SHRINK_STOP;
#endif
		}
		oom_score_adj = p->signal->oom_score_adj;

		if (output_expect(enable_candidate_log)) {
			if (print_extra_info) {
#ifdef CONFIG_MT_ENG_BUILD
log_again:
				log_ret = snprintf(lmk_log_buf+log_offset, LMK_LOG_BUF_SIZE-log_offset,
#else
				lowmem_print(1,
#endif
#ifdef CONFIG_ZRAM
						"<lmk>%5d%5d%11d%8lu%8lu %s\n", p->pid,
						REVERT_ADJ(oom_score_adj), oom_score_adj,
						get_mm_rss(p->mm),
						get_mm_counter(p->mm, MM_SWAPENTS), p->comm);
#else /* CONFIG_ZRAM */
						"<lmk>%5d%5d%11d%8lu %s\n", p->pid,
						REVERT_ADJ(oom_score_adj), oom_score_adj,
						get_mm_rss(p->mm), p->comm);
#endif

#ifdef CONFIG_MT_ENG_BUILD
				if ((log_offset + log_ret) >= LMK_LOG_BUF_SIZE || log_ret < 0) {
					*(lmk_log_buf + log_offset) = '\0';
					lowmem_print(1, "\n%s", lmk_log_buf);
					/* pr_err("lmk log overflow log_offset:%d\n", log_offset); */
					log_offset = 0;
					memset(lmk_log_buf, 0x0, LMK_LOG_BUF_SIZE);
					goto log_again;
				} else
					log_offset += log_ret;
#endif
			}
		}

#ifdef CONFIG_MT_ENG_BUILD
		tasksize = get_mm_rss(p->mm);
#ifdef CONFIG_ZRAM
		tasksize += get_mm_counter(p->mm, MM_SWAPENTS);
#endif
		/*
		* dump memory info when framework low memory:
		* record the first two pid which consumed most memory.
		*/
		if (tasksize > max_mem) {
			max_mem = tasksize;
			/* pid_sec_mem = pid_dump; */
			pid_dump = p->pid;
		}

		if (p->pid == pid_flm_warn &&
			time_before_eq(jiffies, flm_warn_timeout)) {
			task_unlock(p);
			continue;
		}
#endif

#ifdef CONFIG_HSWAP
		cur_top_time = p->signal->top_time;

		if (p->signal->reclaimed)
			reclaimed_cnt++;

		if (oom_score_adj >= OOM_SCORE_SERVICE_B_ADJ &&
				!p->signal->reclaimed) {
			if (reclaim_cnt < RECLAIM_TASK_CNT)
				reclaim_task[reclaim_cnt++] = p;
		}

		if (min_score_adj > OOM_SCORE_SERVICE_B_ADJ) {
			if (oom_score_adj <= OOM_SCORE_SERVICE_B_ADJ) {
				task_unlock(p);
				continue;
			}
		} else {
#endif
			if (oom_score_adj < min_score_adj) {
				task_unlock(p);
				continue;
			}
#ifdef CONFIG_HSWAP
		}
#endif

#ifndef CONFIG_MT_ENG_BUILD
		tasksize = get_mm_rss(p->mm);
#ifdef CONFIG_ZRAM
#ifndef CONFIG_HSWAP
		tasksize += get_mm_counter(p->mm, MM_SWAPENTS);
#endif
#endif
#endif
#ifdef CONFIG_HSWAP
		swapsize = get_mm_counter(p->mm, MM_SWAPENTS);
#endif
		task_unlock(p);
		if (tasksize <= 0)
			continue;
		if (selected) {
#ifdef CONFIG_HSWAP
			if (min_score_adj <= OOM_SCORE_SERVICE_B_ADJ) {
				if (oom_score_adj < selected_oom_score_adj)
					continue;
				if (oom_score_adj == selected_oom_score_adj &&
						tasksize <= selected_tasksize)
					continue;
			} else {
				if (selected_top_time >= 0  &&
						selected_top_time < cur_top_time)
					continue;
				if (selected_top_time == cur_top_time) {
					if (tasksize <= selected_tasksize)
						continue;
				}
			}
#else
			if (oom_score_adj < selected_oom_score_adj)
				continue;
			if (oom_score_adj == selected_oom_score_adj &&
			    tasksize <= selected_tasksize)
				continue;
#endif
		}
#ifdef CONFIG_MTK_GMO_RAM_OPTIMIZE
		/*
		* if cached > 30MB, don't kill ub:secureRandom while its adj is 9
		*/
		if (!strcmp(p->comm, "ub:secureRandom") &&
			(REVERT_ADJ(oom_score_adj) == 9) && (other_file > 30*256)) {
			pr_info("select but ignore '%s' (%d), oom_score_adj %d, oom_adj %d, size %d, to kill, cache %ldkB is below limit %ldkB",
							p->comm, p->pid,
							oom_score_adj, REVERT_ADJ(oom_score_adj),
							tasksize,
							other_file * (long)(PAGE_SIZE / 1024),
							minfree * (long)(PAGE_SIZE / 1024));
		    continue;
		}
#endif
		selected = p;
		selected_tasksize = tasksize;
#ifdef CONFIG_HSWAP
		selected_swapsize = swapsize;
		selected_top_time = cur_top_time;
#endif
		selected_oom_score_adj = oom_score_adj;
		lowmem_print(2, "select '%s' (%d), adj %d, score_adj %hd, size %d, to kill\n",
			     p->comm, p->pid, REVERT_ADJ(oom_score_adj), oom_score_adj, tasksize);
	}

#ifdef CONFIG_MT_ENG_BUILD
	if (log_offset > 0)
		lowmem_print(1, "\n%s", lmk_log_buf);
#endif

	if (selected) {
		long cache_size = other_file * (long)(PAGE_SIZE / 1024);
		long cache_limit = minfree * (long)(PAGE_SIZE / 1024);
		long free = other_free * (long)(PAGE_SIZE / 1024);
		trace_lowmemory_kill(selected, cache_size, cache_limit, free);
#ifdef CONFIG_HSWAP
		if (min_score_adj < OOM_SCORE_SERVICE_B_ADJ)
			goto kill;
		else if (!reclaim_cnt &&
				(min_score_adj > OOM_SCORE_CACHED_APP_MIN_ADJ)) {
			rem = SHRINK_STOP;
			goto end_lmk;
		}

		if (reclaim_cnt && selected_task == NULL && mutex_trylock(&reclaim_mutex)) {
			selected_task = find_suitable_reclaim(reclaim_cnt, &hswap_tasksize);
			if (selected_task) {
				unsigned long flags;

				if (lock_task_sighand(selected_task, &flags)) {
					selected_task->signal->reclaimed = 1;
					unlock_task_sighand(selected_task, &flags);
				}
				get_task_struct(selected_task);
				complete(&reclaim_completion);
				rem += hswap_tasksize;
				lowmem_print(1, "Reclaiming '%s' (%d), adj %hd, top time = %ld\n" \
						"   to free %ldkB on behalf of '%s' (%d) because\n" \
						"   cache %ldkB is below limit %ldkB for oom_score_adj %hd\n" \
						"   Free memory is %ldkB above reserved.\n",
						selected_task->comm, selected_task->pid,
						selected_task->signal->oom_score_adj, selected_task->signal->top_time,
						hswap_tasksize * (long)(PAGE_SIZE / 1024),
						current->comm, current->pid,
						other_file * (long)(PAGE_SIZE / 1024),
						minfree * (long)(PAGE_SIZE / 1024),
						min_score_adj,
						other_free * (long)(PAGE_SIZE / 1024));
				lowmem_print(3, "reclaimed cnt = %d, reclaim cont = %d, min oom score= %hd\n",
						reclaimed_cnt, reclaim_cnt, min_score_adj);
				mutex_unlock(&reclaim_mutex);
				goto end_lmk;
			} else {
				mutex_unlock(&reclaim_mutex);
			}
		}
		selected_tasksize += selected_swapsize;
kill:
#endif
#ifndef CONFIG_HSWAP
		lowmem_print(1, "Killing '%s' (%d), adj %d, score_adj %hd,\n"
#else
		lowmem_print(1, "Killing '%s' (%d), adj %d, score_adj %hd, reclaim_cnt %d\n"
#endif
				"   to free %ldkB on behalf of '%s' (%d) because\n"
				"   cache %ldkB is below limit %ldkB for oom_score_adj %hd\n"
				"   Free memory is %ldkB above reserved\n"
#if defined(CONFIG_SWAP) && defined(CONFIG_MTK_GMO_RAM_OPTIMIZE)
				"   swapfree %lukB of SwapTatal %lukB(decrease %d level)\n"
#endif
				, selected->comm, selected->pid,
				REVERT_ADJ(selected_oom_score_adj),
				selected_oom_score_adj,
#ifdef CONFIG_HSWAP
				 reclaim_cnt,
#endif
				selected_tasksize * (long)(PAGE_SIZE / 1024),
				current->comm, current->pid,
				cache_size, cache_limit,
				min_score_adj,
				free
#if defined(CONFIG_SWAP) && defined(CONFIG_MTK_GMO_RAM_OPTIMIZE)
				, swap_pages * 4, total_swap_pages * 4, to_be_aggressive
#endif
				);
		lowmem_deathpending = selected;
		lowmem_deathpending_timeout = jiffies + HZ;
		set_tsk_thread_flag(selected, TIF_MEMDIE);

		if (output_expect(enable_candidate_log)) {
			if (print_extra_info) {
				show_free_areas(0);
			#ifdef CONFIG_MTK_ION
				/* Show ION status */
				ion_mm_heap_memory_detail();
			#endif
			#ifdef CONFIG_MTK_GPU_SUPPORT
				if (mtk_dump_gpu_memory_usage() == false)
					lowmem_print(1, "mtk_dump_gpu_memory_usage not support\n");
			#endif
			}
		}

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MT_ENG_BUILD)
		/*
		* when kill adj=0 process trigger kernel warning, only in MTK internal eng load
		*/
		if ((selected_oom_score_adj <= lowmem_kernel_warn_adj) && /*lowmem_kernel_warn_adj=16 for test*/
			time_after_eq(jiffies, flm_warn_timeout)) {
			if (pid_dump != pid_flm_warn) {
				#define MSG_SIZE_TO_AEE 70
				char msg_to_aee[MSG_SIZE_TO_AEE];

				lowmem_print(1, "low memory trigger kernel warning\n");
				snprintf(msg_to_aee, MSG_SIZE_TO_AEE,
						"please contact AP/AF memory module owner[pid:%d]\n", pid_dump);
				aee_kernel_warning_api("LMK", 0, DB_OPT_DEFAULT |
					DB_OPT_DUMPSYS_ACTIVITY |
					DB_OPT_LOW_MEMORY_KILLER |
					DB_OPT_PID_MEMORY_INFO | /* smaps and hprof*/
					DB_OPT_PROCESS_COREDUMP |
					DB_OPT_DUMPSYS_SURFACEFLINGER |
					DB_OPT_DUMPSYS_GFXINFO |
					DB_OPT_DUMPSYS_PROCSTATS,
					"Framework low memory\nCRDISPATCH_KEY:FLM_APAF", msg_to_aee);

				if (pid_dump == selected->pid) {/*select 1st time, filter it*/
					/* pid_dump = pid_sec_mem; */
					pid_flm_warn = pid_dump;
					flm_warn_timeout = jiffies + 60*HZ;
					lowmem_deathpending = NULL;
					lowmem_print(1, "'%s' (%d) max RSS, not kill\n",
								selected->comm, selected->pid);
					send_sig(SIGSTOP, selected, 0);
					rcu_read_unlock();
					spin_unlock(&lowmem_shrink_lock);
					return rem;
				}
			} else {
				lowmem_print(1, "pid_flm_warn:%d, select '%s' (%d)\n",
								pid_flm_warn, selected->comm, selected->pid);
				pid_flm_warn = -1; /*reset*/
			}
		}
#endif

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MT_ENG_BUILD)
		/*
		* show an indication if low memory
		*/
		if (!in_lowmem && selected_oom_score_adj <= lowmem_debug_adj) {
			in_lowmem = 1;
			/* DAL_LowMemoryOn();*/
			lowmem_print(1, "LowMemoryOn\n");
			/* aee_kernel_warning(module_name, lowmem_warning);*/
		}
#endif

		//Improve the priority of killed process can accelerate the process to die,
		//and the process memory would be released quickly
		boost_dying_task_prio(selected);

		send_sig(SIGKILL, selected, 0);
		rem += selected_tasksize;
#ifdef CONFIG_HSWAP
		lowmem_print(3, "reclaimed cnt = %d, reclaim cont = %d, min oom score= %hd\n",
				reclaimed_cnt, reclaim_cnt, min_score_adj);
#endif
		++lmk_kill_cnt;
	}

	lowmem_print(4, "lowmem_scan %lu, %x, return %lu\n",
		     sc->nr_to_scan, sc->gfp_mask, rem);
#ifdef CONFIG_HSWAP
end_lmk:
	reclaim_arr_free(reclaim_cnt);
#endif
	rcu_read_unlock();
	spin_unlock(&lowmem_shrink_lock);
	return rem;
}

static struct shrinker lowmem_shrinker = {
	.scan_objects = lowmem_scan,
	.count_objects = lowmem_count,
	.seeks = DEFAULT_SEEKS * 16
};

static int __init lowmem_init(void)
{
#ifdef CONFIG_HIGHMEM
	unsigned long normal_pages;
#endif
#ifdef CONFIG_HSWAP
	struct task_struct *reclaim_tsk;
	struct task_struct *reset_top_time_tsk;
#endif

#ifdef CONFIG_ZRAM
	vm_swappiness = 100;
#endif

	task_free_register(&task_nb);
	register_shrinker(&lowmem_shrinker);

#ifdef CONFIG_HIGHMEM
	normal_pages = totalram_pages - totalhigh_pages;
	total_low_ratio = (totalram_pages + normal_pages - 1) / normal_pages;
	pr_err("[LMK]total_low_ratio[%d] - totalram_pages[%lu] - totalhigh_pages[%lu]\n",
			total_low_ratio, totalram_pages, totalhigh_pages);
#endif
#ifdef CONFIG_HSWAP
	reclaim_tsk = kthread_run(reclaim_task_thread, NULL, "reclaim_task");
	reset_top_time_tsk = kthread_run(reset_task_time_thread, NULL, "reset_task");
#endif

	return 0;
}

static void __exit lowmem_exit(void)
{
	unregister_shrinker(&lowmem_shrinker);
	task_free_unregister(&task_nb);
}

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static short lowmem_oom_adj_to_oom_score_adj(short oom_adj)
{
	if (oom_adj == OOM_ADJUST_MAX)
		return OOM_SCORE_ADJ_MAX;
	else
		return (oom_adj * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE;
}

static void lowmem_autodetect_oom_adj_values(void)
{
	int i;
	short oom_adj;
	short oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;

	if (array_size <= 0)
		return;

	oom_adj = lowmem_adj[array_size - 1];
	if (oom_adj > OOM_ADJUST_MAX)
		return;

	oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
	if (oom_score_adj <= OOM_ADJUST_MAX)
		return;

	lowmem_print(1, "lowmem_shrink: convert oom_adj to oom_score_adj:\n");
	for (i = 0; i < array_size; i++) {
		oom_adj = lowmem_adj[i];
		oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
		lowmem_adj[i] = oom_score_adj;
		lowmem_print(1, "oom_adj %d => oom_score_adj %d\n",
			     oom_adj, oom_score_adj);
	}
}

static int lowmem_adj_array_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_array_ops.set(val, kp);

	/* HACK: Autodetect oom_adj values in lowmem_adj array */
	lowmem_autodetect_oom_adj_values();

	return ret;
}

static int lowmem_adj_array_get(char *buffer, const struct kernel_param *kp)
{
	return param_array_ops.get(buffer, kp);
}

static void lowmem_adj_array_free(void *arg)
{
	param_array_ops.free(arg);
}

static struct kernel_param_ops lowmem_adj_array_ops = {
	.set = lowmem_adj_array_set,
	.get = lowmem_adj_array_get,
	.free = lowmem_adj_array_free,
};

static const struct kparam_array __param_arr_adj = {
	.max = ARRAY_SIZE(lowmem_adj),
	.num = &lowmem_adj_size,
	.ops = &param_ops_short,
	.elemsize = sizeof(lowmem_adj[0]),
	.elem = lowmem_adj,
};
#endif

/*
 * get_min_free_pages
 * returns the low memory killer watermark of the given pid,
 * When the system free memory is lower than the watermark, the LMK (low memory
 * killer) may try to kill processes.
 */
int get_min_free_pages(pid_t pid)
{
	struct task_struct *p;
	int target_oom_adj = 0;
	int i = 0;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;

	for_each_process(p) {
		/* search pid */
		if (p->pid == pid) {
			task_lock(p);
			target_oom_adj = p->signal->oom_score_adj;
			task_unlock(p);
			/* get min_free value of the pid */
			for (i = array_size - 1; i >= 0; i--) {
				if (target_oom_adj >= lowmem_adj[i]) {
					pr_debug("pid: %d, target_oom_adj = %d, lowmem_adj[%d] = %d, lowmem_minfree[%d] = %d\n",
							pid, target_oom_adj, i, lowmem_adj[i], i,
							lowmem_minfree[i]);
					return lowmem_minfree[i];
				}
			}
			goto out;
		}
	}

out:
	lowmem_print(3, "[%s]pid: %d, adj: %d, lowmem_minfree = 0\n",
			__func__, pid, p->signal->oom_score_adj);
	return 0;
}
EXPORT_SYMBOL(get_min_free_pages);

/* Query LMK minfree settings */
/* To query default value, you can input index with value -1. */
size_t query_lmk_minfree(int index)
{
	int which;

	/* Invalid input index, return default value */
	if (index < 0)
		return lowmem_minfree[2];

	/* Find a corresponding output */
	which = 5;
	do {
		if (lowmem_adj[which] <= index)
			break;
	} while (--which >= 0);

	/* Fix underflow bug */
	which = (which < 0) ? 0 : which;

	return lowmem_minfree[which];
}
EXPORT_SYMBOL(query_lmk_minfree);

module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
module_param_cb(adj, &lowmem_adj_array_ops,
		.arr = &__param_arr_adj, S_IRUGO | S_IWUSR);
__MODULE_PARM_TYPE(adj, "array of short");
#else
module_param_array_named(adj, lowmem_adj, short, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
#endif
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);
module_param_named(lmk_fast_run, lmk_fast_run, int, S_IRUGO | S_IWUSR);

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static int debug_adj_set(const char *val, const struct kernel_param *kp)
{
	const int ret = param_set_uint(val, kp);

	lowmem_debug_adj = lowmem_oom_adj_to_oom_score_adj(lowmem_debug_adj);
	return ret;
}

static struct kernel_param_ops debug_adj_ops = {
	.set = &debug_adj_set,
	.get = &param_get_uint,
};

module_param_cb(debug_adj, &debug_adj_ops, &lowmem_debug_adj, S_IRUGO | S_IWUSR);
__MODULE_PARM_TYPE(debug_adj, short);

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MT_ENG_BUILD)
static int flm_warn_adj_set(const char *val, const struct kernel_param *kp)
{
	const int ret = param_set_uint(val, kp);

	lowmem_kernel_warn_adj = lowmem_oom_adj_to_oom_score_adj(lowmem_kernel_warn_adj);
	return ret;
}

static struct kernel_param_ops flm_warn_adj_ops = {
	.set = &flm_warn_adj_set,
	.get = &param_get_uint,
};
module_param_cb(flm_warn_adj, &flm_warn_adj_ops, &lowmem_kernel_warn_adj, S_IRUGO | S_IWUSR);
#endif
#else
module_param_named(debug_adj, lowmem_debug_adj, short, S_IRUGO | S_IWUSR);
#endif
module_param_named(candidate_log, enable_candidate_log, uint, S_IRUGO | S_IWUSR);

module_param_named(lmk_kill_cnt, lmk_kill_cnt, int, S_IRUGO);
#ifdef CONFIG_HSWAP
module_param_named(lmk_reclaim_cnt, lmk_reclaim_cnt, int, S_IRUGO);
#endif

late_initcall(lowmem_init);
module_exit(lowmem_exit);

MODULE_LICENSE("GPL");

