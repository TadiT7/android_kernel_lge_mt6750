/*
 * lge_etrs.c
 *
 * Copyright (C) 2017 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/debugfs.h>

#include <soc/mediatek/lge/board_lge.h>
#include <soc/mediatek/lge/lge_etrs.h>

#ifdef CONFIG_OF_RESERVED_MEM
#include <linux/memblock.h>
#include <linux/of_reserved_mem.h>
#endif

#if defined(CONFIG_LGE_HANDLE_PANIC)
#include <linux/reboot.h>
#include <soc/mediatek/lge/lge_handle_panic.h>
static int crash_handle_status = 0;
#endif

#define PREFIX_MAX		32
#define LOG_LINE_MAX		(1024 - PREFIX_MAX)

static DEFINE_SPINLOCK(lge_ramconsole_lock);

#ifndef ALIGN
#define ALIGN(x, a) __ALIGN_MASK(x, (typeof(x))(a) - 1)
#define __ALIGN_MASK(x, mask) (((x) + (mask)) & ~(mask))
#endif

#define RAMCONSOLE_SIG (0x4352544C) /* LTRC */
struct lge_ramconsole_struct* lge_ramconsole;
phys_addr_t ramconsole_base;

struct trace_data_metadata_info g_metadata_info;
static size_t trace_data_avail_size;

struct trace_data_info g_head_data;
struct dentry *lge_etrs_dir = NULL;
struct dentry *trace_log_dbgfs = NULL;
struct dentry *trace_data_dbgfs = NULL;

static size_t ramconsole_buffer_start_add(struct ramconsole_zone *zone, size_t a)
{
	int old;
	int new;

	do {
		old = atomic_read(&zone->buffer->start);
		new = old + a;
		while (unlikely(new >= zone->buffer_size))
			new -= zone->buffer_size;
	} while (atomic_cmpxchg(&zone->buffer->start, old, new) != old);

	return old;
}

static void ramconsole_buffer_size_add(struct ramconsole_zone *zone, size_t a)
{
	size_t old;
	size_t new;

	if (atomic_read(&zone->buffer->size) == zone->buffer_size)
		return;

	do {
		old = atomic_read(&zone->buffer->size);
		new = old + a;
		if (new > zone->buffer_size)
			new = zone->buffer_size;
	} while (atomic_cmpxchg(&zone->buffer->size, old, new) != old);
}

static void notrace ramconsole_update(struct ramconsole_zone *zone,
	const void *s, unsigned int start, unsigned int count)
{
	struct ramconsole_buffer *buffer = zone->buffer;
	memcpy(buffer->data + start, s, count);
//	ramconsole_update_ecc(zone, start, count);
}

int notrace ramconsole_write(struct ramconsole_zone *zone,
							 const void *s, unsigned int count)
{
	int rem;
	int c = count;
	size_t start;

	if (unlikely(c > zone->buffer_size)) {
		s += c - zone->buffer_size;
		c = zone->buffer_size;
	}

	ramconsole_buffer_size_add(zone, c);

	start = ramconsole_buffer_start_add(zone, c);

	rem = zone->buffer_size - start;
	if (unlikely(rem < c)) {
		ramconsole_update(zone, s, start, rem);
		s += rem;
		c -= rem;
		start = 0;
	}
	ramconsole_update(zone, s, start, c);

//	ramconsole_update_header_ecc(zone);

	return count;
}

static size_t print_time(struct timespec time,
						 struct tm tmresult, char *buf)
{
	if (!buf)
		return snprintf(NULL, 0,
				"[%02d-%02d %02d:%02d:%02d.%03lu] ",
				tmresult.tm_mon+1,
				tmresult.tm_mday,
				tmresult.tm_hour,
				tmresult.tm_min,
				tmresult.tm_sec,
						(unsigned long) time.tv_nsec/1000000);

	return sprintf(buf,
			"[%02d-%02d %02d:%02d:%02d.%03lu] ",
			tmresult.tm_mon+1,
			tmresult.tm_mday,
			tmresult.tm_hour,
			tmresult.tm_min,
			tmresult.tm_sec,
		   (unsigned long) time.tv_nsec/1000000);
}

void lge_ramconsole_log(const char *fmt, ...)
{
	va_list args;
	static char textbuf[LOG_LINE_MAX];
	char *text = textbuf;
	size_t text_len = 0;
	struct timespec time;
	struct tm tmresult;
	unsigned long flags;

#if defined(CONFIG_LGE_HANDLE_PANIC)
	if (!crash_handle_status)
		return;
#endif
	spin_lock_irqsave(&lge_ramconsole_lock, flags);
	va_start(args, fmt);

	time = __current_kernel_time();
	time_to_tm(time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1),
			   &tmresult);

	text_len = print_time(time, tmresult, text);

	text_len += vscnprintf(text + text_len, sizeof(textbuf), fmt, args);

	if (lge_ramconsole && lge_ramconsole->trace_log)
		ramconsole_write(lge_ramconsole->trace_log, text, text_len);
	va_end(args);
	spin_unlock_irqrestore(&lge_ramconsole_lock, flags);
}

void trace_data_metadata_write(struct trace_data_info *trace_data_info)
{
	void *data_info;
	struct trace_data_metadata_info *metadata_info = &g_metadata_info;

	size_t start = 0;

	data_info = (void *)trace_data_info;
	metadata_info->metadata_count++;

	if (metadata_info->metadata_count > MAX_TRACE_DATA) {
		metadata_info->metadata_count--;
		pr_err("%s: metadata zone is already full. Increase MAX_TRACE_DATA to add more tracing data!!\n", __func__);
		return;
	}

	start = atomic_read(&metadata_info->metadata_current_offset);
	if (unlikely(start >= metadata_info->metadata_end_offset)) {
		pr_err("%s: unexpected behavior!!\n", __func__);
		return;
	}

#ifdef DEBUG_TRACE_DATA
	/* notify their metadata offset to each trace data info 
	 * so that they can update metadata info themselves
	 */
	trace_data_info->metadata_offset = start;
#endif
	ramconsole_update(lge_ramconsole->trace_data, data_info, start, sizeof(struct trace_data_info)); 


	start += sizeof(struct trace_data_info);
	atomic_set(&metadata_info->metadata_current_offset, start);
}

void trace_data_timestamp_write(struct trace_data_info *trace_data_info)
{
	struct trace_data_info *data_info;
	size_t start = 0;
/*
	static char textbuf[TRACE_DATA_TIMESTAMP_LEN];
	char *text = textbuf;
	size_t text_len = 0;
	struct timespec time;
	struct tm tmresult;
*/
	u64 k_time;

	data_info = trace_data_info;
	start = atomic_read(&data_info->timestamp_current_offset);

/*
	time = __current_kernel_time();
	time_to_tm(time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1),
			   &tmresult);

	text_len = print_time(time, tmresult, text);
	text[text_len] = '\n';
*/
	k_time = ktime_to_us(ktime_get());
/*	pr_err("%s: check current time stamp : %llu", __func__, k_time); */

	if (unlikely(start >= data_info->timestamp_end_offset)) {
		pr_err("%s: unexpected behavior!!\n", __func__);
		return;
	}

/*
	ramconsole_update(lge_ramconsole->trace_data, text, start, text_len+1);

	start += TRACE_DATA_TIMESTAMP_LEN;
*/
	ramconsole_update(lge_ramconsole->trace_data, &k_time, start, sizeof(k_time));

	start += sizeof(k_time);

	/* implement ring buffer */
	if (start >= data_info->timestamp_end_offset)
		start = data_info->timestamp_start_offset;

	atomic_set(&data_info->timestamp_current_offset, start);
}

static size_t trace_data_buffer_start_add(struct trace_data_info *trace_data_info, size_t a)
{
	struct trace_data_info *data_info;
	size_t start = a;

	data_info = trace_data_info;
	atomic_set(&data_info->data_current_offset, start);
	/* pr_err("%s: check trace data offset: %d", __func__, (int)data_info->data_current_offset); */
	return start;
}

void trace_data_write(struct trace_data_info *trace_data_info, const void *data_value, ...)
{
	struct trace_data_info *data_info;
	uint8_t value_type;
   	size_t start = 0;
	int v_size;
	va_list args;
	int i;
	void *second_value = NULL;

#if defined(CONFIG_LGE_HANDLE_PANIC)
	if (!crash_handle_status)
		return;
#endif

	if (!trace_data_info) {
		pr_err("%s: data_start_offset is null, check if trace data is initialized.", __func__);
		return;
	}

	data_info = trace_data_info;
	value_type = data_info->data_value_type;

	switch (value_type) {
	case FORMAT_TYPE_INT:
		v_size = sizeof(int);
		break;
	case FORMAT_TYPE_UINT:
		v_size = sizeof(unsigned int);
		break;
	case FORMAT_TYPE_LONG:
		v_size = sizeof(long);
		break;
	case FORMAT_TYPE_ULONG:
		v_size = sizeof(unsigned long);
		break;
	case FORMAT_TYPE_LONG_LONG:
		v_size = sizeof(long long);
		break;
	case FORMAT_TYPE_FLOAT:
		v_size = sizeof(float);
		break;
	case FORMAT_TYPE_CHAR:
		v_size = sizeof(char);
		break;
	default:
		v_size = sizeof(int);
	}

	start = atomic_read(&data_info->data_current_offset);

	if (unlikely(start >= data_info->data_end_offset)){
		pr_err("%s: unexpected behavior!! start=%d end_offset=%d\n\n", __func__, start, (int)data_info->data_end_offset);
		return;
	}

	if (start + (v_size * (data_info->array_size - 1)) >= data_info->data_end_offset) {
		/* implement ring buffer */
		/* pr_err("%s: implement ring buffer.", __func__); */
		start = data_info->data_start_offset;
	}

	ramconsole_update(lge_ramconsole->trace_data, data_value, start, v_size);

	start += v_size;

	/* support data array */
	if (data_info->array_size > 1) {
		va_start(args, data_value);
		for (i = 1; i < data_info->array_size ; i++) {
			second_value = va_arg(args, void*);

			if (unlikely(start >= data_info->data_end_offset)){
				pr_err("%s: unexpected behavior..start=%d end_offset=%d\n", __func__, start, (int)data_info->data_end_offset);
				return;
			}
			if (second_value) {
				ramconsole_update(lge_ramconsole->trace_data, second_value, start, v_size);
				start += v_size;
			} else {
				/* just skip offset if array data is not enough */
				start += v_size;
			}
		}
		va_end(args);
	}

	/* implement ring buffer */
	if (start >= data_info->data_end_offset) {
		/* pr_err("%s: implement ring buffer..", __func__); */
		start = data_info->data_start_offset;
	}

	trace_data_buffer_start_add(trace_data_info, start);

	/* add trace data timestamp */
	trace_data_timestamp_write(data_info);

#ifdef DEBUG_TRACE_DATA
	/* update trace data metadata */
	ramconsole_update(lge_ramconsole->trace_data, (void *)data_info, data_info->metadata_offset, sizeof(struct trace_data_info));
#endif
	
}


struct trace_data_info *lge_ramconsole_init_trace_data(const char* data_name, uint8_t data_value_type, uint16_t data_count, uint32_t array_size)
{
	struct trace_data_info *new;
	int check_offset;
	uint8_t v_size;
	size_t temp_offset;
	size_t avail_size;
	size_t text_len;
	struct trace_data_info *data_info = &g_head_data;

#if defined(CONFIG_LGE_HANDLE_PANIC)
	if (!crash_handle_status)
		return NULL;
#endif
	new = kmalloc(sizeof(struct trace_data_info), GFP_KERNEL);
	if (!new) {
		pr_err("%s: can't allocate memory!", __func__);
		return NULL;
	}

	avail_size = trace_data_avail_size;

	new->data_start_offset = atomic_read(&lge_ramconsole->trace_data->buffer->next_data_start_offset);
	atomic_set(&new->data_current_offset, new->data_start_offset);

	new->data_value_type = (uint32_t)data_value_type;
	new->data_count = (uint32_t)data_count;
	new->array_size = (uint32_t)array_size;

	memset(new->data_name, 0, sizeof(new->data_name));
	text_len = scnprintf(new->data_name, sizeof(new->data_name), "%s", data_name);

	switch (data_value_type) {
	case FORMAT_TYPE_INT:
		v_size = sizeof(int);
		break;
	case FORMAT_TYPE_UINT:
		v_size = sizeof(unsigned int);
		break;
	case FORMAT_TYPE_LONG:
		v_size = sizeof(long);
		break;
	case FORMAT_TYPE_ULONG:
		v_size = sizeof(unsigned long);
		break;
	case FORMAT_TYPE_LONG_LONG:
		v_size = sizeof(long long);
		break;
	case FORMAT_TYPE_FLOAT:
		v_size = sizeof(float);
		break;
	case FORMAT_TYPE_CHAR:
		v_size = sizeof(char);
		break;
	default:
		v_size = sizeof(int);
	}
	new->data_end_offset = new->data_start_offset + (v_size * data_count * array_size);

	/* consider each data timestamp */
	new->timestamp_start_offset = new->data_end_offset;
	atomic_set(&new->timestamp_current_offset, new->data_end_offset);
/*	new->timestamp_end_offset = new->timestamp_start_offset + (TRACE_DATA_TIMESTAMP_LEN * data_count); */
	new->timestamp_end_offset = new->timestamp_start_offset + (sizeof(u64) * data_count);
	temp_offset = ALIGN(new->timestamp_end_offset, 4); /* 4bytes align for memcpy*/ 

	/* exception handling for considering max buffer size */
	if (temp_offset >= avail_size) {
		pr_err("%s: can't allocate memory, since zone is full now!", __func__);
		kfree(new);
		return NULL;
	}

	/* set next data start offset */
	atomic_set(&lge_ramconsole->trace_data->buffer->next_data_start_offset, temp_offset);

	/* 
	 * <data_start_offset>
	 * |data_1|data_2|...|
	 * <data_end_offset(=time_stamp_start_offset)>
	 * |data_1_timestamp|data_2_timestamp|...|
	 * <timestamp_end_offset(=next_data_start_offset)>
	 */

	/* atomic_set(&lge_ramconsole->trace_data->buffer->next_data_start_offset, new->data_end_offset); */
	check_offset = atomic_read(&lge_ramconsole->trace_data->buffer->next_data_start_offset);

	/* pr_err("%s: next_data_start_offset is %d", __func__, check_offset); */

	/* add a metadata to the ramconsole for this trace data */
	trace_data_metadata_write(new);

	list_add_tail(&new->data_list, &data_info->data_list);

	return new;
}

static void ramconsole_init_trace_data_metadata(struct ramconsole_zone *trace_zone)
{
	struct ramconsole_zone *trace_data = trace_zone;
	struct trace_data_metadata_info *metadata_info = &g_metadata_info;

	atomic_set(&metadata_info->metadata_current_offset, 0);
	metadata_info->metadata_end_offset = 0;
	metadata_info->metadata_count = 0;
	metadata_info->metadata_max = MAX_TRACE_DATA;
	
	metadata_info->metadata_end_offset = sizeof(struct trace_data_info) * MAX_TRACE_DATA;
	/* pr_err("%s: sizeof trace_data_info: %d\n", __func__, sizeof(struct trace_data_info)); */

	atomic_set(&trace_data->buffer->next_data_start_offset, metadata_info->metadata_end_offset);

	trace_data_avail_size = lge_ramconsole->trace_data->buffer_size - sizeof(struct ramconsole_buffer);
	pr_err("%s: available buffer size for tracing data: %d\n", __func__, (int)trace_data_avail_size);
}

static void *ramconsole_vmap(phys_addr_t start, size_t size,
							 unsigned int memtype)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);

	if (memtype)
		prot = pgprot_noncached(PAGE_KERNEL);
	else
		prot = pgprot_writecombine(PAGE_KERNEL);

	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %u pages\n",
		       __func__, page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vmap(pages, page_count, VM_MAP, prot);
	kfree(pages);

	return vaddr;
}

static int ramconsole_buffer_map(phys_addr_t start, size_t size,
								 struct ramconsole_zone *zone, unsigned int memtype)
{
	zone->paddr = start;
	zone->size = size;

	if (pfn_valid(start >> PAGE_SHIFT))
		zone->vaddr = ramconsole_vmap(start, size, memtype);
	else {
		pr_err("%s: Failed to map : start address is not valid pfn\n", __func__);
	}

	if (!zone->vaddr) {
		pr_err("%s: Failed to map 0x%llx pages at 0x%llx\n", __func__,
			(unsigned long long)size, (unsigned long long)start);
		return -ENOMEM;
	}

	zone->buffer = zone->vaddr + offset_in_page(start);
	zone->buffer_size = size - sizeof(struct ramconsole_buffer);

	return 0;
}

void ramconsole_buffer_init(struct ramconsole_zone *zone)
{
	atomic_set(&zone->buffer->start, 0);
	atomic_set(&zone->buffer->size, 0);
	atomic_set(&zone->buffer->next_data_start_offset, 0);
	zone->buffer->data_start_vaddr =  zone->buffer->data;
}

static void ramconsole_buffer_clear(struct ramconsole_zone *zone)
{
	struct ramconsole_buffer *buffer = zone->buffer;
	memset(buffer->data, 0, zone->buffer_size);
}

static int ramconsole_post_init(struct ramconsole_zone *zone, u32 sig)
{
	/* int ret; */

	/* ret = persistent_ram_init_ecc(prz, ecc_info); */
	/* if (ret) */
	/* 	return ret; */

	ramconsole_buffer_clear(zone);

	sig ^= RAMCONSOLE_SIG;

	zone->buffer->sig = sig;
	ramconsole_buffer_init(zone);

	return 0;
}

void ramconsole_free(struct ramconsole_zone *zone)
{
	if (!zone)
		return;

	if (zone->vaddr) {
		if (pfn_valid(zone->paddr >> PAGE_SHIFT)) {
			vunmap(zone->vaddr);
		}
		zone->vaddr = NULL;
	}
	kfree(zone);
}

struct ramconsole_zone *ramconsole_new(phys_addr_t paddr,
									   size_t sz, u32 sig, unsigned int memtype)
{
	int ret = -ENOMEM;
	struct ramconsole_zone *zone;

	zone = kzalloc(sizeof(struct ramconsole_zone), GFP_KERNEL);
	if (!zone)
		goto err;

	ret = ramconsole_buffer_map(paddr, sz, zone, memtype);
	if (ret)
		goto err;

	ret = ramconsole_post_init(zone, sig);
	if (ret)
		goto err;

	return zone;
err:
	ramconsole_free(zone);
	return ERR_PTR(ret);
}

int ramconsole_init_items(struct ramconsole_zone **zone, phys_addr_t *paddr,
								  size_t sz, u32 sig, unsigned int memtype)
{
	if (!sz)
		return 0;

	if (*paddr + sz - lge_ramconsole->phys_addr > lge_ramconsole->size)
		return -ENOMEM;

	*zone = ramconsole_new(*paddr, sz, sig, memtype);
	if (IS_ERR(*zone)) {
		int err = PTR_ERR(zone);
		return err;
	}

	ramconsole_buffer_init(*zone);

	*paddr += sz;
	return 0;
}

static int trace_log_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t trace_log_debug_read(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	void *data_start_vaddr = NULL;
	size_t buffer_size = 0;

	data_start_vaddr = lge_ramconsole->trace_log->buffer->data;
	buffer_size = atomic_read(&lge_ramconsole->trace_log->buffer->size);

	if (data_start_vaddr && buffer_size >= 0)
		return simple_read_from_buffer(ubuf, count, ppos, data_start_vaddr, buffer_size);
	else
		return 0;
}

static const struct file_operations trace_log_debug_fops = {
	.read = trace_log_debug_read,
	.open = trace_log_debug_open,
};

static int trace_data_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t trace_data_debug_read(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	struct trace_data_info *tmp;
	struct list_head *pos;
	struct trace_data_info *data_info = &g_head_data;
	struct trace_data_metadata_info *metadata_info = &g_metadata_info;

	char *trace_data_buffer;

	ssize_t len, ret = 0;
	size_t buffer_size = 0;
	u64 k_time;

	void *trace_data_root_vaddr = lge_ramconsole->trace_data->buffer->data;
	void *timestamp_cur = NULL;
	void *data_cur = NULL;
	uint8_t data_value_type;
	uint8_t v_size;
	uint32_t array_size = 1;

	int i, j;

	if (!metadata_info)
		return 0;

	buffer_size = PAGE_SIZE * 4; /* temp size */
	trace_data_buffer = kmalloc(buffer_size, GFP_KERNEL);

	if (!trace_data_buffer) {
		pr_err("%s: memory alloc failed!", __func__);
		return -ENOMEM;
	}

	memset(trace_data_buffer, 0, buffer_size);

	len = scnprintf(trace_data_buffer, buffer_size, "######## Trace Data List ########\n");

	list_for_each(pos, &data_info->data_list) {
		tmp = list_entry(pos, struct trace_data_info, data_list);
		if (tmp) {
			len += scnprintf(trace_data_buffer + len, buffer_size - len, "%s\n", tmp->data_name);
			pr_err("%s: trace_data_name: %s, length: %d", __func__, tmp->data_name, strlen(tmp->data_name));
			/* get timestamp info*/
			timestamp_cur = trace_data_root_vaddr + tmp->timestamp_start_offset;
			/* get data info*/
			data_cur = trace_data_root_vaddr + tmp->data_start_offset;
			array_size = tmp->array_size;
			data_value_type = tmp->data_value_type;

			for (i = 0; i < tmp->data_count; i++) {
				k_time = *(u64 *)timestamp_cur;
				len += scnprintf(trace_data_buffer + len, buffer_size - len, "[%18llu] ", k_time);
				timestamp_cur += sizeof(k_time);

				for (j = 0; j < array_size; j++) {
					switch (data_value_type) {
					case FORMAT_TYPE_INT:
						len += scnprintf(trace_data_buffer + len, buffer_size - len, "%d ", *(int *)data_cur);
						v_size = sizeof(int);
						break;
					case FORMAT_TYPE_UINT:
						len += scnprintf(trace_data_buffer + len, buffer_size - len, "%u ", *(unsigned int *)data_cur);
						v_size = sizeof(unsigned int);
						break;
					case FORMAT_TYPE_LONG:
						len += scnprintf(trace_data_buffer + len, buffer_size - len, "%ld ", *(long *)data_cur);
						v_size = sizeof(long);
						break;
					case FORMAT_TYPE_ULONG:
						len += scnprintf(trace_data_buffer + len, buffer_size - len, "%lu ", *(unsigned long *)data_cur);
						v_size = sizeof(unsigned long);
						break;
					case FORMAT_TYPE_LONG_LONG:
						len += scnprintf(trace_data_buffer + len, buffer_size - len, "%lld ", *(long long *)data_cur);
						v_size = sizeof(long long);
						break;

					case FORMAT_TYPE_FLOAT: /* currently just memcpy for float */
						memcpy(trace_data_buffer + len, data_cur, sizeof(float));
						len += sizeof(float);
						v_size = sizeof(float);
						break;

					case FORMAT_TYPE_CHAR:
						len += scnprintf(trace_data_buffer + len, buffer_size - len, "%c ", *(char *)data_cur);
						v_size = sizeof(char);
						break;
					default:
						len += scnprintf(trace_data_buffer + len, buffer_size - len, "%d ", *(int *)data_cur);
						v_size = sizeof(int);
					}
					data_cur += v_size;
				}
				len += scnprintf(trace_data_buffer + len, buffer_size - len, "\n");
			}

		} /* end of..if (tmp) */
	}
	/* if len > buffer_size ? */
	ret = simple_read_from_buffer(ubuf, count, ppos, trace_data_buffer, len);

	kfree(trace_data_buffer);
	return ret;
}

static const struct file_operations trace_data_debug_fops = {
	.read = trace_data_debug_read,
	.open = trace_data_debug_open,
};

#if defined(CONFIG_LGE_HANDLE_PANIC)
static void lge_ramconsole_free_page(unsigned long mem_addr, unsigned long size)
{
	unsigned long pfn_start, pfn_end, pfn_idx;

	pfn_start = mem_addr >> PAGE_SHIFT;
	pfn_end = (mem_addr + size) >> PAGE_SHIFT;

	for (pfn_idx = pfn_start; pfn_idx < pfn_end; pfn_idx++) {
		free_reserved_page(pfn_to_page(pfn_idx));
	}
}

static void lge_ramconsole_reserve_cleanup(unsigned long addr, unsigned long size)
{
	pr_info("%s: reserved-memory free[@0x%lx+@0x%lx)\n", __func__, addr, size);
	if (addr == 0 || size == 0)
		return;

	memblock_free(addr, size);

	lge_ramconsole_free_page(addr, size);
}
#endif

static int __init lge_ramconsole_init(void)
{
	struct device_node *node;
	struct resource ramconsole_res;
	int trace_data_size;
	int trace_log_size;
	int err = -ENODEV;
	struct trace_data_info *data_info = &g_head_data;

	lge_ramconsole = kzalloc(sizeof(struct lge_ramconsole_struct), GFP_KERNEL);
	if (!lge_ramconsole)
		return -ENOMEM;
	node = of_find_compatible_node(NULL, NULL, "lge,ramconsole");
	if (!node)
		return -ENODEV;

	err = of_property_read_u32(node, "trace_data_size", &trace_data_size);
	if (err)
		return err;

	err = of_property_read_u32(node, "trace_log_size", &trace_log_size);
	if (err)
		return err;

	node = of_find_compatible_node(NULL, NULL, "lge,ramconsole_mem");
	if (!node)
		return -ENODEV;

	if (of_address_to_resource(node, 0, &ramconsole_res))
		return -ENODEV;

	lge_ramconsole->phys_addr = ramconsole_base = ramconsole_res.start;
	lge_ramconsole->size = (ramconsole_res.end - ramconsole_res.start) + 1;

#if defined(CONFIG_LGE_HANDLE_PANIC)
	crash_handle_status = lge_get_crash_handle_status();
	if (!crash_handle_status) {
		pr_err("%s: LGE crash handler disabled => disable ETRS", __func__);
		lge_ramconsole_reserve_cleanup(lge_ramconsole->phys_addr, lge_ramconsole->size);
		kfree(lge_ramconsole);
		return -ENOMEM;
	}
#endif
	pr_err("%s: ETRS enabled.", __func__);
	ramconsole_init_items(&lge_ramconsole->trace_data, &ramconsole_base, trace_data_size, 0, 1);
	ramconsole_init_items(&lge_ramconsole->trace_log, &ramconsole_base, trace_log_size, 0, 1);

	/* add metadata zone for trace_data */
	ramconsole_init_trace_data_metadata(lge_ramconsole->trace_data);

	INIT_LIST_HEAD(&data_info->data_list);

	/* make debugfs */
	lge_etrs_dir = debugfs_create_dir("lge_etrs", NULL);
	if (lge_etrs_dir) {
		trace_log_dbgfs = debugfs_create_file("trace_log", S_IFREG | S_IRUGO, lge_etrs_dir, (void *)0, &trace_log_debug_fops);
		trace_data_dbgfs = debugfs_create_file("trace_data", S_IFREG | S_IRUGO, lge_etrs_dir, (void *)0, &trace_data_debug_fops);

		if ((!trace_log_dbgfs) || (!trace_data_dbgfs)) {
			pr_err("%s: unable to create lge_etrs debugfs file\n", __func__);
			debugfs_remove(lge_etrs_dir);
		}
	}

	return 0;
}

static void __exit lge_ramconsole_exit(void)
{
	return;
}

early_initcall(lge_ramconsole_init);

MODULE_DESCRIPTION("LGE ETRS driver");
MODULE_AUTHOR("Fred Cho <fred.cho@lge.com>");
MODULE_LICENSE("GPL");
