/* production_test.c
 *
 * Copyright (C) 2015 LGE.
 *
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
#define TS_MODULE "[prd]"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_td4100.h"
#include "touch_td4100_prd.h"

static char line[50000] = { 0 };

static const int retry_count = 30;


/* static u16 LowerImage[ROW_SIZE][COL_SIZE]; */
/* static u16 UpperImage[ROW_SIZE][COL_SIZE]; */


/* static void log_file_size_check(struct device *dev) */
void log_file_size_check(struct device *dev)
{
	char *fname = NULL;
	struct file *file;
	loff_t file_size = 0;
	int i = 0;
	char buf1[128] = { 0 };
	char buf2[128] = { 0 };
	mm_segment_t old_fs = get_fs();
	int ret = 0;
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	boot_mode = touch_boot_mode_check(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
		fname = "/sdcard/touch_self_test.txt";
		break;
	case MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case MINIOS_MFTS_FOLDER:
	case MINIOS_MFTS_FLAT:
	case MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}

	if (fname) {
		file = filp_open(fname, O_RDONLY, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
		goto error;
	}

	if (IS_ERR(file)) {
		TOUCH_I("%s : ERR(%ld) Open file error [%s]\n", __func__, PTR_ERR(file), fname);
		goto error;
	}

	file_size = vfs_llseek(file, 0, SEEK_END);
	TOUCH_I("%s : [%s] file_size = %lld\n", __func__, fname, file_size);

	filp_close(file, 0);

	if (file_size > MAX_LOG_FILE_SIZE) {
		TOUCH_I("%s : [%s] file_size(%lld) > MAX_LOG_FILE_SIZE(%d)\n",
			__func__, fname, file_size, MAX_LOG_FILE_SIZE);

		for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
			if (i == 0)
				sprintf(buf1, "%s", fname);
			else
				sprintf(buf1, "%s.%d", fname, i);

			ret = sys_access(buf1, 0);

			if (ret == 0) {
				TOUCH_I("%s : file [%s] exist\n", __func__, buf1);

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					if (sys_unlink(buf1) < 0) {
						TOUCH_E("%s : failed to remove file [%s]\n",
							__func__, buf1);
						goto error;
					}

					TOUCH_I("%s : remove file [%s]\n", __func__, buf1);
				} else {
					sprintf(buf2, "%s.%d", fname, (i + 1));

					if (sys_rename(buf1, buf2) < 0) {
						TOUCH_E("%s : failed to rename file [%s] -> [%s]\n",
							__func__, buf1, buf2);
						goto error;
					}

					TOUCH_I("%s : rename file [%s] -> [%s]\n",
						__func__, buf1, buf2);
				}
			} else {
				TOUCH_I("%s : file [%s] does not exist (ret = %d)\n",
					__func__, buf1, ret);
			}
		}
	}

error:
	set_fs(old_fs);
}

static void write_file(struct device *dev, char *data, int write_time)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = { 0 };
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	boot_mode = touch_boot_mode_check(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
		fname = "/sdcard/touch_self_test.txt";
		break;
	case MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case MINIOS_MFTS_FOLDER:
	case MINIOS_MFTS_FLAT:
	case MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}

	if (fname) {
		fd = sys_open(fname, O_WRONLY | O_CREAT | O_APPEND, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
		set_fs(old_fs);
		return;
	}

	if (fd >= 0) {
		if (write_time == TIME_INFO_WRITE) {
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
			snprintf(time_string, 64,
				 "\n[%02d-%02d %02d:%02d:%02d.%03lu]\n",
				 my_date.tm_mon + 1,
				 my_date.tm_mday, my_date.tm_hour,
				 my_date.tm_min, my_date.tm_sec,
				 (unsigned long)my_time.tv_nsec / 1000000);
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	} else {
		TOUCH_I("File open failed\n");
	}
	set_fs(old_fs);
}

static int sdcard_spec_file_read(struct device *dev)
{
	int ret = 0;
	int fd = 0;
	char *path[2] = { "/mnt/sdcard/sf3_limit.txt",
		"/mnt/sdcard/sf3_limit_mfts.txt"
	};
	int path_idx = 0;

	mm_segment_t old_fs = get_fs();

	if (touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER)
		path_idx = 1;
	else
		path_idx = 0;
	set_fs(KERNEL_DS);
	fd = sys_open(path[path_idx], O_RDONLY, 0);
	if (fd >= 0) {
		sys_read(fd, line, sizeof(line));
		sys_close(fd);
		TOUCH_I("%s file existing\n", path[path_idx]);
		ret = 1;
	}
	set_fs(old_fs);

	return ret;
}

static int spec_file_read(struct device *dev)
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fwlimit = NULL;
	const char *path[2] = { ts->panel_spec,
		ts->panel_spec_mfts
	};
	int path_idx = 0;

	if (touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER)
		path_idx = 1;
	else
		path_idx = 0;

	if (ts->panel_spec == NULL || ts->panel_spec_mfts == NULL) {
		TOUCH_I("panel_spec_file name is null\n");
		ret = -1;
		goto error;
	}

	if (request_firmware(&fwlimit, path[path_idx], dev) < 0) {
		TOUCH_I("request ihex is failed in normal mode\n");
		ret = -1;
		goto error;
	}

	if (fwlimit->data == NULL) {
		ret = -1;
		TOUCH_I("fwlimit->data is NULL\n");
		goto error;
	}

	strlcpy(line, fwlimit->data, sizeof(line));

error:
	if (fwlimit)
		release_firmware(fwlimit);

	return ret;
}

/* static int synaptics_get_limit(struct device *dev, char *breakpoint, */
int synaptics_get_limit(struct device *dev, char *breakpoint, u16(*buf)[COL_SIZE])
{
	int p = 0;
	int q = 0;
	int r = 0;
	int cipher = 1;
	int ret = 0;
	char *found;
	int boot_mode = 0;
	int file_exist = 0;
	int tx_num = 0;
	int rx_num = 0;


	if (breakpoint == NULL) {
		ret = -1;
		goto error;
	}

	boot_mode = touch_boot_mode_check(dev);
	if (boot_mode > MINIOS_MFTS_CURVED || boot_mode < TOUCH_NORMAL_BOOT) {
		ret = -1;
		goto error;
	}

	file_exist = sdcard_spec_file_read(dev);
	if (!file_exist) {
		ret = spec_file_read(dev);
		if (ret == -1)
			goto error;
	}

	if (line == NULL) {
		ret = -1;
		goto error;
	}

	found = strnstr(line, breakpoint, sizeof(line));
	if (found != NULL) {
		q = found - line;
	} else {
		TOUCH_I("failed to find breakpoint. The panel_spec_file is wrong\n");
		ret = -1;
		goto error;
	}

	memset(buf, 0, ROW_SIZE * COL_SIZE * 2);

	while (1) {
		if (line[q] == ',') {
			cipher = 1;
			for (p = 1; (line[q - p] >= '0') && (line[q - p] <= '9'); p++) {
				buf[tx_num][rx_num] += ((line[q - p] - '0') * cipher);
				cipher *= 10;
			}
			r++;
			if (r % (int)COL_SIZE == 0) {
				rx_num = 0;
				tx_num++;
			} else {
				rx_num++;
			}
		}
		q++;
		if (r == (int)ROW_SIZE * (int)COL_SIZE) {
			TOUCH_I("panel_spec_file scanning is success\n");
			break;
		}
	}

	if (ret == 0) {
		ret = -1;
		goto error;

	} else {
		TOUCH_I("panel_spec_file scanning is success\n");
		return ret;
	}

error:
	return ret;
}

static int switchPage(struct device *dev, u8 page)
{
	int ret = 0;
	u8 data = 0;
	int count = 0;

	do {
		synaptics_write(dev, PAGE_SELECT_REG, &page, sizeof(page));
		msleep(20);
		synaptics_read(dev, PAGE_SELECT_REG, &data, sizeof(page));
		count++;
	} while ((data != page) && (count < retry_count));

	if (count >= retry_count) {
		TOUCH_I("Timeout -- Page switch fail !\n");
		ret = -EAGAIN;
	}

	return ret;
}

static void RspSetReportType(struct device *dev, u8 setValue)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	synaptics_write(dev, d->f54.dsc.data_base, &setValue, sizeof(setValue));
}

static int RspSetCommandType(struct device *dev, u8 setValue)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	u8 data = 0;
	int count = 0;

	synaptics_write(dev, d->f54.dsc.command_base, &setValue, sizeof(setValue));

	do {
		touch_msleep(10);
		synaptics_read(dev, d->f54.dsc.command_base, &data, sizeof(data));
		count++;
	} while (data != 0x00 && (count < retry_count));

	if (count >= retry_count) {
		TOUCH_I("Timeout -- Command set fail !\n");
		ret = -EAGAIN;
	}

	return ret;
}

void RspResetReportAddress(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 data = 0;
	u8 addr = 0;

	addr = d->f54.dsc.data_base + 1;
	synaptics_write(dev, addr, &data, 1);
	addr++;
	synaptics_write(dev, addr, &data, 1);
}

void RspReadImageReport(struct device *dev, int16_t *buf)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	synaptics_read(dev, d->f54.dsc.data_base + 3, (u8 *) buf, ROW_SIZE * COL_SIZE * 2);
}


/* static void firmware_version_log(struct device *dev) */
void firmware_version_log(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	unsigned char buffer[LOG_BUF_SIZE] = { 0, };
	int boot_mode = 0;

	boot_mode = touch_boot_mode_check(dev);
	if (boot_mode >= MINIOS_MFTS_FOLDER)
		ret = synaptics_ic_info(dev);

	ret = snprintf(buffer, LOG_BUF_SIZE, "======== Firmware Info ========\n");
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret, "version : v%d.%02d\n",
			d->ic_info.version.major, d->ic_info.version.minor);

	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"product id : %s\n", d->ic_info.product_id);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret, "===============================\n\n");

	write_file(dev, buffer, TIME_INFO_SKIP);
}

static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	int16_t *delta = NULL;
	int i = 0;
	int j = 0;

	mutex_lock(&ts->lock);
	delta = kzalloc(sizeof(int16_t) * (COL_SIZE * ROW_SIZE), GFP_KERNEL);

	ret = switchPage(dev, ANALOG_PAGE);
	if (ret < 0)
		goto error;

	RspSetReportType(dev, 0x2);
	RspResetReportAddress(dev);
	ret = RspSetCommandType(dev, 0x1);
	if (ret < 0)
		goto error;

	RspReadImageReport(dev, delta);

	for (i = 0; i < ROW_SIZE; i++) {
		char log_buf[LOG_BUF_SIZE] = { 0, };
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "[%2d]  ", i);
		for (j = 0; j < COL_SIZE; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret, "%5d ",
					delta[(ROW_SIZE * (COL_SIZE - 1) + i) - (ROW_SIZE * j)]);
			log_ret += snprintf(log_buf + log_ret,
					    LOG_BUF_SIZE - log_ret, "%5d ",
					    delta[(ROW_SIZE * (COL_SIZE - 1) + i) -
						  (ROW_SIZE * j)]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	switchPage(dev, DEFAULT_PAGE);
	if (delta != NULL)
		kfree(delta);

	mutex_unlock(&ts->lock);
	return ret;
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int16_t *rawdata = NULL;
	int ret = 0;
	/* u8 data = 0x0; */
	int i = 0;
	int j = 0;

	mutex_lock(&ts->lock);

	rawdata = kzalloc(sizeof(int16_t) * (COL_SIZE * ROW_SIZE), GFP_KERNEL);

	ret = switchPage(dev, ANALOG_PAGE);
	if (ret < 0)
		goto error;

	RspSetReportType(dev, 0x3);
	RspResetReportAddress(dev);

	/* Apply ForceCal. */
	/*
	   synaptics_read(dev, d->f54.dsc.command_base, &data, sizeof(data));
	   data = data | 0x02;
	   ret = RspSetCommandType(dev, data);
	   if (ret < 0)
	   goto error;
	 */

	ret = RspSetCommandType(dev, 0x1);
	if (ret < 0)
		goto error;

	RspReadImageReport(dev, rawdata);

	for (i = 0; i < ROW_SIZE; i++) {
		char log_buf[LOG_BUF_SIZE] = { 0, };
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "[%2d]  ", i);
		for (j = 0; j < COL_SIZE; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret, "%5d ",
					rawdata[(ROW_SIZE * (COL_SIZE - 1) + i) - (ROW_SIZE * j)]);
			log_ret += snprintf(log_buf + log_ret,
					    LOG_BUF_SIZE - log_ret, "%5d ",
					    rawdata[(ROW_SIZE * (COL_SIZE - 1) + i) -
						    (ROW_SIZE * j)]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	switchPage(dev, DEFAULT_PAGE);
	if (rawdata != NULL)
		kfree(rawdata);

	mutex_unlock(&ts->lock);

	return ret;
}

static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);

static struct attribute *prd_attribute_list[] = {
	&touch_attr_delta.attr,
	&touch_attr_rawdata.attr,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
};

int td4100_prd_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &prd_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	return ret;
}
