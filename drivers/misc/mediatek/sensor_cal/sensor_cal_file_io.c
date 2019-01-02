#include "sensor_cal_file_io.h"
#include <hwmsensor.h> 
#include <linux/fs.h>
#include <linux/syscalls.h>

#define SENSOR_CAL_TAG					"SENSOR_CAL"
#define SENSOR_CAL_ERR(fmt, args...)    pr_err(SENSOR_CAL_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define SENSOR_CAL_LOG(fmt, args...)    pr_debug(SENSOR_CAL_TAG fmt, ##args)

#define PROFILE ID_BASE -1 
#define DATALEN 10
#define MSGLEN  128

static char msg[MSGLEN]; //only for cal file
static char hist[MSGLEN]; //for sensor logging

int sensor_calibration_read(int sensor, int* cal);
int sensor_calibration_save(int sensor, int* cal);
int sensor_history_save(int sensor, const char* __event, const char* __msg);

static char* hist_msg(int sensor, const char* __event, const  char* __msg)
{
	struct timespec tp;
	struct tm tm;
	tp = __current_kernel_time();
	time_to_tm(tp.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &tm);
	if(__event && __msg)
	{
		memset(hist, 0x0, MSGLEN);
		sprintf(hist, "[%02d-%02d]%02d:%02d:%02d.%.03lu : sensor_type(%d) event(%s) : ", 
				tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, (unsigned long)tp.tv_nsec/1000000, sensor, __event);
		strcat(hist, __msg);
		return hist;
	}
	return (char*)0;
}
static int proxy_msg(int sensor, int* cal)
{
	int pCal;
	int ret;
	int res = sensor_calibration_read(sensor,&pCal);
	if(res)
		ret = sprintf(msg, "%d 0", *cal);
	else
		ret = sprintf(msg, "%d %d", *cal, pCal);
	return ret;
}
static int accel_gyro_msg(int sensor, int* cal)
{
	int pCal[3];
	int ret;
	int res = sensor_calibration_read(sensor,pCal);
	if(res)
		ret = sprintf(msg, "%d %d %d 0 0 0",cal[0], cal[1], cal[2]);
	else
		ret = sprintf(msg, "%d %d %d %d %d %d", cal[0], cal[1], cal[2], pCal[0], pCal[1], pCal[2]);
	return ret;

}
static char* cur_msg(void)
{
	return msg;
}
static char* sensor_msg(int sensor, int* cal)
{
	int ret = 0, res = 0;

	memset(msg, 0x0, MSGLEN);
	
	if(sensor==ID_PROXIMITY)
		ret = proxy_msg(sensor, cal); 
	else if(sensor==ID_ACCELEROMETER || sensor == ID_GYROSCOPE)
		ret = accel_gyro_msg(sensor, cal);
	else
		res = 1;
	
	msg[ret++] = '\n';
	msg[ret] = '\0';
	
	return (res) ? (char*)0 : msg;
}
static char* get_path(int sensor)
{
	char* fname = NULL;
	switch(sensor)
	{
	case PROFILE:
		fname = SENSOR_HISTORY_FILE;
		break;
	case ID_PROXIMITY:
		fname = SENSOR_PROXIMITY_CAL_FILE;
		break;
	case ID_ACCELEROMETER:
		fname = SENSOR_ACCEL_CAL_FILE;
		break;
	case ID_GYROSCOPE:
		fname = SENSOR_GYRO_CAL_FILE;
		break;
	default:
		SENSOR_CAL_LOG("Undefined sensor type\n");
		break;
	}
	return fname;
}

static int __sensor_read(const char* fname, const int sensor, int* cal)
{
	int fd, res=0;
	char buf[MSGLEN];
	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);
	fd = sys_open(fname, O_RDONLY, 0);

	if(fd < 0)
		goto cal_read_error;
	
	res = sys_read(fd, buf , MSGLEN);
	if(res <= 0)
		goto cal_read_error;

	switch(sensor)
	{
    case ID_PROXIMITY:
	 	sscanf(buf,"%d",cal);
		break;
	case ID_ACCELEROMETER:
	case ID_GYROSCOPE:
		sscanf(buf,"%d %d %d",&cal[0], &cal[1], &cal[2]);
		break;
	default:
		break;
	}
cal_read_error:	
	sys_close(fd);
	set_fs(old_fs);
	return (res <=0) ? -EINVAL : 0;
}

int __sensor_write(const char* fname, const char* msg, int flag )
{
	int fd, res=0;
	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);
	
	fd = sys_open(fname, flag, 0666);
	if(fd < 0)
	{
		goto cal_write_error;
	}
	res = sys_write(fd, msg, strlen(msg));
	if(res <= 0)
		goto cal_write_error;

	sys_fsync(fd);
cal_write_error:	
	sys_close(fd);
	set_fs(old_fs);
	return (res <= 0) ? -EINVAL : 0;
}
static int sensor_read(const int sensor, int* cal )
{
	return  __sensor_read( get_path(sensor), sensor, cal); 
}
static int sensor_write( const int sensor, const char* msg )
{
	return  __sensor_write(get_path(sensor), msg, O_WRONLY|O_CREAT|S_IROTH); 
}
int sensor_history_save(int sensor, const char* __event, const char* __msg )
{
	return __sensor_write (get_path(PROFILE), hist_msg(sensor, __event, __msg), O_WRONLY|O_CREAT|S_IROTH|O_APPEND );
}
EXPORT_SYMBOL(sensor_history_save);
int sensor_calibration_save(int sensor, int* cal)
{
	return sensor_write( sensor, sensor_msg(sensor, cal)) || sensor_history_save(sensor, "calibration", cur_msg() ); 
}
EXPORT_SYMBOL(sensor_calibration_save);

int sensor_calibration_read(int sensor, int *cal_read)
{
	return sensor_read(sensor, cal_read);
}
EXPORT_SYMBOL(sensor_calibration_read);
