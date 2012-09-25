/*
 * ASUS ram dump.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <asm/io.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>

MODULE_DESCRIPTION("Asus ram dump");
MODULE_LICENSE("GPL");

/***********kernel log ramdump test *****/
#define IRAM_CMD_ADDRESS 0x4001F000
#define IRAM_KERNEL_LOG_BUFFER	0x40020000
#define DATA_LOGS		"/data/logs"
#define DATA_LOGS_RAMDUMP	"/data/logs/ramdump"
#define DATA_MEDIA_RAMDUMP	"/data/media/ramdump"


struct delayed_work ramdump_work;

static char rd_log_file[256];
static char rd_kernel_time[256];

struct timespec ts;
struct rtc_time tm;

static void ramdump_get_time(void){
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	sprintf(rd_kernel_time, "%d-%02d-%02d-%02d%02d%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
}

static int ramdump_log_filename(void){
	struct file *fp;
	int err;
	mm_segment_t old_fs;
	fp = filp_open(DATA_LOGS , O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	if (PTR_ERR(fp) == -ENOENT) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		err = sys_mkdir(DATA_LOGS,0777);
		if (err < 0) {
			set_fs(old_fs);
			return -ENOENT;
		}
		set_fs(old_fs);
		strcpy(rd_log_file, DATA_LOGS_RAMDUMP);
	} else {
		filp_close(fp,NULL);
		strcpy(rd_log_file, DATA_LOGS_RAMDUMP);
	}
	return 0;

}

static void ramdump_work_function(struct work_struct *dat){
	void __iomem *cmd_addr;
	void __iomem *test_addr;
	char *p;
	char temp[1024];
	char cmd[32];
	struct file *fp;
	int i;
	mm_segment_t old_fs;

	cmd_addr = ioremap(IRAM_CMD_ADDRESS,8);
	test_addr = ioremap(IRAM_KERNEL_LOG_BUFFER,8);

	strncpy(cmd, cmd_addr, 10);
	//printk(KERN_ERR "ramdump: cmd = %s\n", cmd);

	if(!strncmp(cmd, "kernel panic", 9)){
		printk(KERN_INFO "ramdump starting\n");

		if (ramdump_log_filename() < 0){
			printk(KERN_ERR "%s folder doesn't exist, and create fail !\n", DATA_LOGS);
			return ;
		}

		ramdump_get_time();
		strcat(rd_log_file, rd_kernel_time);
		strcat(rd_log_file,".log");

		old_fs = get_fs();
		set_fs(KERNEL_DS);

		p = (char *) test_addr;
		fp = filp_open(rd_log_file , O_APPEND | O_RDWR | O_CREAT, S_IRWXU|S_IRWXG|S_IRWXO);
		if (PTR_ERR(fp) == -ENOENT){
			set_fs(old_fs);
			return ;
		}

		for (i = 0; i < 128; i++){
			memcpy(temp, p + (1024 * i), 1024);
			vfs_write(fp, temp, 1024, &fp->f_pos);
		}
		memset(cmd_addr, 0, 12);
		memset(test_addr, 0, 128*1024);

		filp_close(fp,NULL);
		set_fs(old_fs);
		printk(KERN_INFO "ramdump file: %s\n", rd_log_file);
	}
	printk(KERN_INFO "rd: finish\n");
}

static int __init rd_init(void){
	INIT_DELAYED_WORK_DEFERRABLE(&ramdump_work, ramdump_work_function);
	schedule_delayed_work(&ramdump_work, 15*HZ);
	return 0;
}

static void __exit rd_exit(void){

}


module_init(rd_init);
module_exit(rd_exit);


