/*
 * lge_etrs.h
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

#ifndef __LGE_ETRS_H__
#define __LGE_ETRS_H__

#define LGE_RAM_CONSOLE_PHY_ADDR	0x0
#define LGE_RAM_CONSOLE_SIZE	(0 * SZ_1K)

/* for debugging */
/* #define DEBUG_TRACE_DATA */

struct ramconsole_buffer {
	uint32_t    sig;
	atomic_t    start;
	atomic_t    size;
	atomic_t    next_data_start_offset;
	void        *data_start_vaddr;
	uint8_t     data[0];
};

struct ramconsole_zone {
	phys_addr_t paddr;
	size_t size;
	void *vaddr;
	struct ramconsole_buffer *buffer;
	size_t buffer_size;
};

struct lge_ramconsole_struct {
	struct ramconsole_zone *trace_data;
	struct ramconsole_zone *trace_log;
	phys_addr_t phys_addr;
	size_t size;
};

#define TRACE_DATA_NAME_LEN 32
#define TRACE_DATA_TIMESTAMP_LEN 25
#define MAX_TRACE_DATA 100

enum trace_data_format_type {
	FORMAT_TYPE_INT,
	FORMAT_TYPE_UINT,
	FORMAT_TYPE_LONG,
	FORMAT_TYPE_ULONG,
	FORMAT_TYPE_LONG_LONG,
	FORMAT_TYPE_FLOAT,
	FORMAT_TYPE_CHAR,
	FORMAT_TYPE_SIZE_T
};

struct trace_data_info {
	/* char data_name[TRACE_DATA_NAME_LEN]; */
	char data_name[TRACE_DATA_NAME_LEN];
	size_t data_start_offset;
	size_t timestamp_start_offset;
	uint32_t data_value_type;
	uint32_t data_count;
	size_t data_end_offset;
	atomic_t data_current_offset;
	size_t timestamp_end_offset;
	atomic_t timestamp_current_offset;
	uint32_t array_size;
#ifdef DEBUG_TRACE_DATA
	size_t metadata_offset;
#endif
	struct list_head data_list;
};

struct trace_data_metadata_info {
	atomic_t metadata_current_offset;
	size_t metadata_end_offset;
	size_t metadata_max;
	size_t metadata_count;
};


extern struct trace_data_info *lge_ramconsole_init_trace_data(const char* item_name, uint8_t item_value_type, uint16_t item_count, uint32_t array_size);

extern void trace_data_write(struct trace_data_info *trace_data_info, const void *data_value, ...);

#define CAT( A, B ) A ## B
#define SELECT( NAME, NUM ) CAT( NAME ## _, NUM )

#define GET_COUNT( _1, _2, _3, _4, _5, _6 /* ad nauseam */, COUNT, ... ) COUNT
#define VA_SIZE( ... ) GET_COUNT( __VA_ARGS__, 6, 5, 4, 3, 2, 1 )

#define VA_SELECT( NAME, ... ) SELECT( NAME, VA_SIZE(__VA_ARGS__) )(__VA_ARGS__)

#define INIT_TRACE_DATA( ... ) VA_SELECT( INIT_TRACE_DATA, __VA_ARGS__ )
#define INIT_TRACE_DATA_3( data, t, c ) (data = lge_ramconsole_init_trace_data(#data, t, c, 1))
#define INIT_TRACE_DATA_4( data, t, c, a_size ) (data = lge_ramconsole_init_trace_data(#data, t, c, a_size))

#define TRACE_WRITE(data, var, ...)					\
	(trace_data_write(data, var, ##__VA_ARGS__))

extern void lge_ramconsole_log(const char* fmt, ...);
#endif
