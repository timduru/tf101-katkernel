/*
 *
 * Touch Screen Serial Driver for EETI Controller
 *
 * Copyright (C) 2000-2011  eGalax_eMPIA Technology Inc.
 * All Rights Reserved.
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
 */

#define RELEASE_DATE "2011/05/04"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/serio.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
	#include <linux/earlysuspend.h>
	static struct early_suspend egalax_early_suspend;
#endif

#define DRIVER_DESC	"egalax touch screen serial driver"
#define SERIO_EGALAX	0xEF

// Global define to enable function
#define _ENABLE_DBG_LEVEL
#define _IDLE_MODE_SUPPORT
//#define _CONSTANT_TOUCH_SUPPORT

static int global_major = 0; // dynamic major by default 
static int global_minor = 0;

#define MAX_EVENTS		600
#define MAX_PACKET_LEN		64
#define FIFO_SIZE		PAGE_SIZE
#define MAX_SUPPORT_POINT	4

// ioctl command
#define EGALAX_IOC_MAGIC	0x72
#define	EGALAX_IOCWAKEUP	_IO(EGALAX_IOC_MAGIC, 1)
#define EGALAX_IOC_MAXNR	1

#define CONSTANT_ENABLE_TIME	750 // ms
#define CONST_TOUCH_DIST	10 // 2048 range

// running mode
#define MODE_STOP	0
#define MODE_WORKING	1
#define MODE_IDLE	2
#define MODE_SUSPEND	3

struct point_data {
	short Status;
	short X;
	short Y;
};

struct _egalax_serial {
	struct serio *serio;
	unsigned char packet_len;
	unsigned char bytes_recved;
	unsigned char msg_in;
	unsigned char point_in;
	unsigned char data[MAX_PACKET_LEN];
	unsigned char downCnt;
	unsigned char work_state;
#ifdef _CONSTANT_TOUCH_SUPPORT
	unsigned char enableConstTouch; 
	struct hrtimer const_hrtimer;
#endif
#ifdef _IDLE_MODE_SUPPORT
	struct timer_list idle_timer;
#endif
};

struct egalax_char_dev
{
	int OpenCnts;
	struct cdev cdev;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	struct kfifo* pCharKFiFo;
	spinlock_t CharFiFoLock;
#else
	struct kfifo CharKFiFo;
#endif
	unsigned char *pFiFoBuf;
	struct semaphore sem;
	wait_queue_head_t fifo_inq;
};

static struct _egalax_serial *p_egalax_serial_dev = NULL;	// allocated in egalax_serial_connect
static struct egalax_char_dev *p_char_dev = NULL;	// allocated in init_module
static atomic_t egalax_char_available = ATOMIC_INIT(1);
static atomic_t wait_command_ack = ATOMIC_INIT(0);
static struct class *egalax_class;
static struct input_dev *input_dev = NULL;
static struct point_data PointBuf[MAX_SUPPORT_POINT];
static char DevPhys[] = {"ttyHS/serio/input0"};

#define DBG_MODULE	0x00000001
#define DBG_CDEV	0x00000002
#define DBG_PROC	0x00000004
#define DBG_POINT	0x00000008
#define DBG_INT		0x00000010
#define DBG_PARSER	0x00000020
#define DBG_SUSP	0x00000040
#define DBG_INPUT	0x00000080 // 1 for EETI report. 0 for default report
#define DBG_CONST	0x00000100
#define DBG_IDLE	0x00000200
#define DBG_WAKEUP	0x00000400
static unsigned int DbgLevel = DBG_MODULE|DBG_SUSP;

#ifdef _ENABLE_DBG_LEVEL
	#define PROC_FS_NAME	"egalax_dbg"
	#define PROC_FS_MAX_LEN	8
	static struct proc_dir_entry *dbgProcFile;
#endif

#if(1)
	#define EGALAX_DBG(level, fmt, args...)  { if( (level&DbgLevel)>0 ) \
					printk( KERN_DEBUG "[egalax_serial]: " fmt, ## args); }
#else
	#define EGALAX_DBG(level, fmt, args...)
#endif

#define IDLE_INTERVAL	5 // second
// additional define global vaiable
int egalax_suspend_result = 0;

static int wakeup_controller(void)
{
	unsigned char buf[] = {0x0A, 0x01, 0x41};
	int i, ret=-1, waitCnt, retry=3, val;
	
	EGALAX_DBG(DBG_WAKEUP, " Run wakeup_controller\n");

	while(0 < retry--)
	{
		atomic_set(&wait_command_ack, 0);

		for(i=0; i<sizeof(buf); i++)
			serio_write(p_egalax_serial_dev->serio, buf[i]);

		waitCnt = 20;
		while( !(val=atomic_read(&wait_command_ack)) )
		{
			if(0 >= waitCnt--)
				break;
			mdelay(2);
		}

		if(val>0)
		{
			ret = 0;
			break;
		}
	}
	
	EGALAX_DBG(DBG_WAKEUP, " Wakeup_controller done with result:%d\n", ret);

	return ret;
}

static int egalax_cdev_open(struct inode *inode, struct file *filp)
{
	struct egalax_char_dev *cdev;

       if(p_egalax_serial_dev == NULL){
	     printk("[egalax_serial]: No EETI touch device!\n");
	     return -ENODEV;	
	}
	     
	   
	cdev = container_of(inode->i_cdev, struct egalax_char_dev, cdev);
	if( cdev == NULL )
	{
        	EGALAX_DBG(DBG_CDEV, " No such char device node \n");
		return -ENODEV;
	}
	
	if( !atomic_dec_and_test(&egalax_char_available) )
	{
		atomic_inc(&egalax_char_available);
		return -EBUSY; /* already open */
	}

	cdev->OpenCnts++;
	filp->private_data = cdev;// Used by the read and write metheds

#ifdef _IDLE_MODE_SUPPORT
	// check and wakeup controller if necessary
	del_timer_sync(&p_egalax_serial_dev->idle_timer);
	if( p_egalax_serial_dev->work_state == MODE_IDLE )
		wakeup_controller();
#endif

	EGALAX_DBG(DBG_CDEV, " CDev open done!\n");
	try_module_get(THIS_MODULE);
	return 0;
}

static int egalax_cdev_release(struct inode *inode, struct file *filp)
{
	struct egalax_char_dev *cdev; // device information

	cdev = container_of(inode->i_cdev, struct egalax_char_dev, cdev);
        if( cdev == NULL )
        {
                EGALAX_DBG(DBG_CDEV, " No such char device node \n");
                return -ENODEV;
        }

	atomic_inc(&egalax_char_available); /* release the device */

	filp->private_data = NULL;
	cdev->OpenCnts--;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	kfifo_reset( cdev->pCharKFiFo );
#else
	kfifo_reset( &cdev->CharKFiFo );
#endif

#ifdef _IDLE_MODE_SUPPORT
	mod_timer(&p_egalax_serial_dev->idle_timer,  jiffies+HZ*IDLE_INTERVAL);
#endif

	EGALAX_DBG(DBG_CDEV, " CDev release done!\n");
	module_put(THIS_MODULE);
	return 0;
}

#define MAX_READ_BUF_LEN	50
static char fifo_read_buf[MAX_READ_BUF_LEN];
static ssize_t egalax_cdev_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	int read_cnt, ret, fifoLen;
	struct egalax_char_dev *cdev = file->private_data;
	
	if( down_interruptible(&cdev->sem) )
		return -ERESTARTSYS;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	fifoLen = kfifo_len(cdev->pCharKFiFo);
#else
	fifoLen = kfifo_len(&cdev->CharKFiFo);
#endif

	while( fifoLen<1 ) /* nothing to read */
	{
		up(&cdev->sem); /* release the lock */
		if( file->f_flags & O_NONBLOCK )
			return -EAGAIN;

	#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
		if( wait_event_interruptible(cdev->fifo_inq, kfifo_len( cdev->pCharKFiFo )>0) )
	#else
		if( wait_event_interruptible(cdev->fifo_inq, kfifo_len( &cdev->CharKFiFo )>0) )
	#endif
		{
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
		}

		if( down_interruptible(&cdev->sem) )
			return -ERESTARTSYS;
	}

	if(count > MAX_READ_BUF_LEN)
		count = MAX_READ_BUF_LEN;

	EGALAX_DBG(DBG_CDEV, " \"%s\" reading: real fifo data\n", current->comm);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	read_cnt = kfifo_get(cdev->pCharKFiFo, fifo_read_buf, count);
#else
	read_cnt = kfifo_out(&cdev->CharKFiFo, fifo_read_buf, count);
#endif

	ret = copy_to_user(buf, fifo_read_buf, read_cnt)?-EFAULT:read_cnt;

	up(&cdev->sem);
	
	return ret;
}

static ssize_t egalax_cdev_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	struct egalax_char_dev *cdev = file->private_data;
	int ret=0, written=0;
	unsigned char c;

	if( down_interruptible(&cdev->sem) )
		return -ERESTARTSYS;
	
	if(!(p_egalax_serial_dev->serio)) 
	{
		ret = -ENODEV;
		goto out;
	}

	if(count > MAX_PACKET_LEN)
		count = MAX_PACKET_LEN;

	while(count--) 
	{
		if(get_user(c, buf++)) 
		{
			ret = -EFAULT;
			goto out;
		}
		if(serio_write(p_egalax_serial_dev->serio, c)) 
		{
			ret = -EIO;
			goto out;
		}
		written++;
	};

out:
	up(&cdev->sem);
	EGALAX_DBG(DBG_CDEV, " Serio writing %d bytes.\n", written);

	if(ret!=0)
		return ret;
	else
		return written;
}

#ifdef _ENABLE_DBG_LEVEL
static int egalax_proc_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data )
{
	int ret;
	
	EGALAX_DBG(DBG_PROC, " \"%s\" call proc_read\n", current->comm);
	
	if(offset > 0)  /* we have finished to read, return 0 */
		ret  = 0;
	else 
		ret = sprintf(buffer, "Debug Level: 0x%08X\nRelease Date: %s\n", DbgLevel, RELEASE_DATE);

	return ret;
}

static int egalax_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char procfs_buffer_size = 0; 
	int i;
	unsigned char procfs_buf[PROC_FS_MAX_LEN+1] = {0};

	EGALAX_DBG(DBG_PROC, " \"%s\" call proc_write\n", current->comm);

	procfs_buffer_size = count;
	if(procfs_buffer_size > PROC_FS_MAX_LEN ) 
		procfs_buffer_size = PROC_FS_MAX_LEN+1;
	
	if( copy_from_user(procfs_buf, buffer, procfs_buffer_size) ) 
	{
		EGALAX_DBG(DBG_PROC, " proc_write faied at copy_from_user\n");
		return -EFAULT;
	}

	DbgLevel = 0;
	for(i=0; i<procfs_buffer_size-1; i++)
	{
		if( procfs_buf[i]>='0' && procfs_buf[i]<='9' )
			DbgLevel |= (procfs_buf[i]-'0');
		else if( procfs_buf[i]>='A' && procfs_buf[i]<='F' )
			DbgLevel |= (procfs_buf[i]-'A'+10);
		else if( procfs_buf[i]>='a' && procfs_buf[i]<='f' )
			DbgLevel |= (procfs_buf[i]-'a'+10);
		
		if(i!=procfs_buffer_size-2)
			DbgLevel <<= 4;
	}

	DbgLevel = DbgLevel&0xFFFFFFFF;

	EGALAX_DBG(DBG_PROC, " Switch Debug Level to 0x%08X\n", DbgLevel);

	return count; // procfs_buffer_size;
}
#endif // #ifdef _ENABLE_DBG_LEVEL

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int egalax_cdev_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long args)
{	
	//struct egalax_char_dev *cdev = file->private_data;
	int ret=0;

	if(_IOC_TYPE(cmd) != EGALAX_IOC_MAGIC)
		return -ENOTTY;
	if(_IOC_NR(cmd) > EGALAX_IOC_MAXNR)
		return -ENOTTY;

	if(_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void __user*)args, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void __user*)args, _IOC_SIZE(cmd));

	if(ret)
		return -EFAULT;

	EGALAX_DBG(DBG_CDEV, " Handle device ioctl command\n");
	switch (cmd)
	{
		case EGALAX_IOCWAKEUP:
			ret = wakeup_controller();
			break;
		default:
			ret = -ENOTTY;
			break;
	}

	return ret;
}
#endif

static unsigned int egalax_cdev_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct egalax_char_dev *cdev = filp->private_data;
	unsigned int mask = 0;
	int fifoLen;
	
	down(&cdev->sem);
	poll_wait(filp, &cdev->fifo_inq,  wait);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	fifoLen = kfifo_len(cdev->pCharKFiFo);
#else
	fifoLen = kfifo_len(&cdev->CharKFiFo);
#endif

	if( fifoLen > 0 )
		mask |= POLLIN | POLLRDNORM;    /* readable */
	if( (FIFO_SIZE - fifoLen) > 0 )
		mask |= POLLOUT | POLLWRNORM;   /* writable */

	up(&cdev->sem);
	return mask;
}

#ifdef _CONSTANT_TOUCH_SUPPORT
static void setConstTouchTimer(struct _egalax_serial *p_egalax_serial, char state)
{
	if(state)
	{
		p_egalax_serial->enableConstTouch = 1;
		hrtimer_start(&p_egalax_serial->const_hrtimer, ktime_set(0, CONSTANT_ENABLE_TIME*1000000), HRTIMER_MODE_REL); 
	}
	else
	{
		p_egalax_serial->enableConstTouch = 0;
		hrtimer_cancel(&p_egalax_serial->const_hrtimer);
	}
	EGALAX_DBG(DBG_CONST, " set ConstTouch Timer with state=%d\n", state);
}
#endif

bool checkPointPacket(unsigned char *buf)
{
	if((buf[0]&0x80)==0x80 && (buf[1]&0x80)==0 && (buf[2]&0x80)==0 && (buf[3]&0x80)==0 && (buf[4]&0x80)==0 && buf[5]>='A' && buf[5]<('A'+MAX_SUPPORT_POINT) )
		return true;
	else
		return false;
}

static int LastUpdateID = 0;
static void ProcessReport(struct _egalax_serial *p_egalax_serial)
{
	int i, cnt_down=0, cnt_up=0;
	short X=0, Y=0, ContactID=0, Status=0;
	unsigned char *buf = p_egalax_serial->data;
	bool bNeedReport = false;
	int lastDownCnt=0;

	if(!checkPointPacket(buf))
	{
		EGALAX_DBG(DBG_POINT, " Get UART Point data error [%02X][%02X][%02X][%02X][%02X][%02X]\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
		return;
	}

	Status = buf[0]&0x01;
	ContactID = buf[5]-'A';
	X = ((buf[1]&0x7F)<<7) + (buf[2]&0x7F);
	Y = 2047-(((buf[3]&0x7F)<<7) + (buf[4]&0x7F));

#ifdef _CONSTANT_TOUCH_SUPPORT
	if(p_egalax_serial->downCnt<=1 && ContactID==0 && Status>0)
	{
		if(PointBuf[ContactID].Status!=1)
			setConstTouchTimer(p_egalax_serial, 1);
		else
		{
			if( p_egalax_serial->enableConstTouch ) 
			{ 
				if(abs(PointBuf[ContactID].X-X)<CONST_TOUCH_DIST && abs(PointBuf[ContactID].Y-Y)<CONST_TOUCH_DIST )
				{
					EGALAX_DBG(DBG_CONST, " Skip point because of constant touch\n");
					return;
				}
				else
					setConstTouchTimer(p_egalax_serial, 0);
			}
		}
	}
	else
		setConstTouchTimer(p_egalax_serial, 0);
#endif

	lastDownCnt = p_egalax_serial->downCnt;
	PointBuf[ContactID].X = X;
	PointBuf[ContactID].Y = Y;
	if(PointBuf[ContactID].Status!=Status)
	{
		if(Status)
			p_egalax_serial->downCnt++;
		else if( PointBuf[ContactID].Status>0 )
			p_egalax_serial->downCnt--;
		
		PointBuf[ContactID].Status = Status;
		bNeedReport = true;
	}

	EGALAX_DBG(DBG_POINT, " Get Point[%d] Update: Status=%d X=%d Y=%d DownCnt=%d\n", ContactID, Status, X, Y, p_egalax_serial->downCnt);

	// Send point report
	if( bNeedReport || (ContactID <= LastUpdateID) )
	{
		for(i=0; i<MAX_SUPPORT_POINT; i++)
		{
			if(PointBuf[i].Status >= 0)
			{
				input_report_abs(input_dev, ABS_MT_TRACKING_ID, i);
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, PointBuf[i].Status);
				input_report_abs(input_dev, ABS_MT_POSITION_X, PointBuf[i].X);
				input_report_abs(input_dev, ABS_MT_POSITION_Y, PointBuf[i].Y);
				input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 0);
				input_mt_sync(input_dev);

				if(PointBuf[i].Status == 0)
				{
					PointBuf[i].Status--;
					cnt_up++;
				}
				else
					cnt_down++;
			}
		}

		input_sync(input_dev);
		EGALAX_DBG(DBG_POINT, " Input sync point data done! (Down:%d Up:%d)\n", cnt_down, cnt_up);
	}

	LastUpdateID = ContactID;
}

static irqreturn_t egalax_serial_interrupt(struct serio *serio, unsigned char data, unsigned int flags)
{
	struct _egalax_serial *egalax_serial = (struct _egalax_serial *)serio_get_drvdata(serio);

	EGALAX_DBG(DBG_INT, " Interrupt trigger with Data: 0x%02X\n", data);

	if(egalax_serial == NULL)
		return IRQ_HANDLED;

	if(egalax_serial->work_state == MODE_STOP || egalax_serial->work_state == MODE_SUSPEND)
		return IRQ_HANDLED;

#ifdef _IDLE_MODE_SUPPORT
	if(egalax_serial->work_state == MODE_IDLE)
		egalax_serial->work_state = MODE_WORKING;
#endif

	if(egalax_serial->point_in==0 && data&0x80 && egalax_serial->bytes_recved==0)
	{
		egalax_serial->data[0] = data;
		egalax_serial->point_in = 1;
		egalax_serial->packet_len = 6;
		egalax_serial->bytes_recved = 1;
	}
	else if(egalax_serial->msg_in==0 && data==0x0A && egalax_serial->bytes_recved==0 )
	{
		egalax_serial->data[0] = data;
		egalax_serial->msg_in = 1;
		egalax_serial->packet_len = 0;
		egalax_serial->bytes_recved = 1;

		atomic_set(&wait_command_ack, 1);
	}
	else if(egalax_serial->point_in || egalax_serial->msg_in)
	{
		if(egalax_serial->msg_in && egalax_serial->packet_len==0)
		{
			if(data > MAX_PACKET_LEN-2)
			{
				EGALAX_DBG(DBG_PARSER, " Recv command data is too long:%d\n", data);
				egalax_serial->msg_in = 0;
				egalax_serial->point_in = 0;
				egalax_serial->bytes_recved = 0;
				egalax_serial->packet_len = 0;
			}
			else
			{
				egalax_serial->data[egalax_serial->bytes_recved++] = data;
				egalax_serial->packet_len = data+2;
			}
		}
		else
		{
			egalax_serial->data[egalax_serial->bytes_recved++] = data;
			if( egalax_serial->bytes_recved == egalax_serial->packet_len )
			{
				if(egalax_serial->msg_in && egalax_serial->point_in)
				{
					EGALAX_DBG(DBG_PARSER, " Unclear state, msg & point are true\n");
				}
				else if(egalax_serial->msg_in)
				{
					// handle command here
					EGALAX_DBG(DBG_PARSER, " Get vendor command data\n");

					if( p_char_dev->OpenCnts>0 ) // If someone reading now! put the data into the buffer!
					{
					#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
						kfifo_put(p_char_dev->pCharKFiFo, egalax_serial->data, egalax_serial->bytes_recved);
					#else
						kfifo_in(&p_char_dev->CharKFiFo, egalax_serial->data, egalax_serial->bytes_recved);
					#endif
					 	wake_up_interruptible( &p_char_dev->fifo_inq );
					}

					egalax_serial->msg_in = 0;
				}
				else if(egalax_serial->point_in)
				{
					// handle point here
					ProcessReport(egalax_serial);
					egalax_serial->point_in = 0;
				}
				egalax_serial->bytes_recved = 0;
				egalax_serial->packet_len = 0;
			#ifdef _IDLE_MODE_SUPPORT
				if( p_char_dev->OpenCnts<=0 )
					mod_timer(&egalax_serial->idle_timer,  jiffies+HZ*IDLE_INTERVAL);
			#endif
			}
		}
	}
	else
	{
		EGALAX_DBG(DBG_PARSER, " Unclear state, data from serial driver : 0x%02x\n", data);
		egalax_serial->msg_in = 0;
		egalax_serial->point_in = 0;
		egalax_serial->packet_len = 0;
		egalax_serial->bytes_recved = 0;
	}
	
	return IRQ_HANDLED;
}

#ifdef _CONSTANT_TOUCH_SUPPORT
static enum hrtimer_restart egalax_const_hrtimer_handle(struct hrtimer *handle)
{
	struct _egalax_serial *egalax_serial = container_of(handle, struct _egalax_serial, const_hrtimer);

	egalax_serial->enableConstTouch = 0;
	EGALAX_DBG(DBG_CONST, " Disable const touch because timeout\n");

	return HRTIMER_NORESTART;
}
#endif

#ifdef _IDLE_MODE_SUPPORT
static void egalax_idle_timer_routine(unsigned long data)
{
	struct _egalax_serial *egalax_serial = (struct _egalax_serial *)data;
	unsigned char buf[] = {0x0A, 0x04, 0x36, 0x3F, 0x01, 0x00}; //50ms
	int i, ret=0;

	if(egalax_serial->work_state == MODE_WORKING)
	{
		for(i=0; i<sizeof(buf); i++)
		{
			if(serio_write(egalax_serial->serio, buf[i]))
			{ 
				ret = -1;
				break;
			}
		}

		if(ret==0)
		{
			egalax_serial->work_state = MODE_IDLE;
			EGALAX_DBG(DBG_IDLE, " Set controller to idle mode\n");
		}
		else
			EGALAX_DBG(DBG_IDLE, " Try to set controller to idle failed:%d\n", ret);
	}
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void egalax_serial_early_suspend(struct early_suspend *handler)
{
	unsigned char buf[] = {0x0A, 0x03, 0x36, 0x3F, 0x02};
	short i;

	EGALAX_DBG(DBG_SUSP, " Enter early_suspend\n");

	if(!p_egalax_serial_dev || !(p_egalax_serial_dev->serio)) 
		goto fail_suspend;

      if(p_egalax_serial_dev->work_state == MODE_SUSPEND){
	    egalax_suspend_result = 0;
	    return;
	}

#ifdef _CONSTANT_TOUCH_SUPPORT
	hrtimer_cancel(&p_egalax_serial_dev->const_hrtimer);
#endif

#ifdef _IDLE_MODE_SUPPORT
	del_timer_sync(&p_egalax_serial_dev->idle_timer);

	if(p_egalax_serial_dev->work_state == MODE_IDLE)
	{
		EGALAX_DBG((DBG_SUSP|DBG_IDLE), " Wakeup controller from idle mode before entering suspend\n");

		if(wakeup_controller()!=0)
			goto fail_suspend;

		EGALAX_DBG((DBG_SUSP|DBG_IDLE), " Device return to working\n");
	}
#endif

	for(i=0; i<(sizeof(buf)/sizeof(unsigned char)); i++)
	{
		if(serio_write(p_egalax_serial_dev->serio, buf[i])) 
			goto fail_suspend;
	}
	
	p_egalax_serial_dev->work_state = MODE_SUSPEND;
	EGALAX_DBG(DBG_SUSP, " Early_suspend done\n");
	egalax_suspend_result = 0;
	return;

fail_suspend:
	p_egalax_serial_dev->work_state = MODE_SUSPEND;
	EGALAX_DBG(DBG_SUSP, " Early_suspend failed\n");
	egalax_suspend_result = -1;
	return;
}

static void egalax_serial_early_resume(struct early_suspend *handler)
{
	short i;

	EGALAX_DBG(DBG_SUSP, " Enter early_resume\n");

	if(!p_egalax_serial_dev || !(p_egalax_serial_dev->serio)) 
		goto fail_resume;

        if(p_egalax_serial_dev->work_state == MODE_WORKING){
	    return;
	}

	// re-init parameter
	p_egalax_serial_dev->msg_in = 0;
	p_egalax_serial_dev->point_in = 0;
	p_egalax_serial_dev->packet_len = 0;
	p_egalax_serial_dev->bytes_recved = 0;
	p_egalax_serial_dev->downCnt=0;
#ifdef _CONSTANT_TOUCH_SUPPORT
	p_egalax_serial_dev->enableConstTouch = 0;
#endif
	for(i=0; i<MAX_SUPPORT_POINT; i++)
	{
		PointBuf[i].Status = -1;
		PointBuf[i].X = PointBuf[i].Y = 0;
	}

	p_egalax_serial_dev->work_state = MODE_WORKING;

	if(wakeup_controller()!=0)
		goto fail_resume;

	EGALAX_DBG(DBG_SUSP, " Early_resume done\n");
	return;

fail_resume:
	p_egalax_serial_dev->work_state = MODE_SUSPEND;
	EGALAX_DBG(DBG_SUSP, " Early_resume failed\n");
	return;
}
#endif

static struct input_dev * allocate_Input_Dev(void)
{
	printk("JN101 touch: allocate touch devices\n");
	int ret;
	struct input_dev *pInputDev=NULL;

	pInputDev = input_allocate_device();
	if(pInputDev == NULL)
	{
		EGALAX_DBG(DBG_MODULE, " Failed to allocate input device\n");
		return NULL;//-ENOMEM;
	}

	pInputDev->name = "eGalax_Serial";
	pInputDev->phys = DevPhys;
	pInputDev->id.bustype = BUS_RS232;
	pInputDev->id.vendor = 0x0EEF;
	pInputDev->id.product = 0x0030;
	pInputDev->id.version = 0x0001;
	
//	set_bit(EV_ABS, pInputDev->evbit);
	set_bit(BTN_TOUCH, pInputDev->keybit);
	input_set_abs_params(pInputDev, ABS_MT_TRACKING_ID, 0, MAX_SUPPORT_POINT, 0, 0);
	input_set_abs_params(pInputDev, ABS_MT_POSITION_X, 0, 2047, 0, 0);
	input_set_abs_params(pInputDev, ABS_MT_POSITION_Y, 0, 2047, 0, 0);
	input_set_abs_params(pInputDev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(pInputDev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	__set_bit(EV_ABS, pInputDev->evbit);
	__set_bit(EV_SYN, pInputDev->evbit);
	__set_bit(EV_KEY, pInputDev->evbit);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
	input_set_events_per_packet(pInputDev, MAX_EVENTS);
#endif

	ret = input_register_device(pInputDev);
	if(ret) 
	{
		EGALAX_DBG(DBG_MODULE, " Unable to register input device.\n");
		input_free_device(pInputDev);
		pInputDev = NULL;
	}
	
	return pInputDev;
}

static void egalax_serial_disconnect(struct serio *serio)
{
	struct _egalax_serial *egalax_serial = (struct _egalax_serial *)serio_get_drvdata(serio);
	
	EGALAX_DBG(DBG_MODULE, " Start serial disconnect ...\n");

	egalax_serial->work_state = MODE_STOP;

#ifdef _CONSTANT_TOUCH_SUPPORT
	hrtimer_cancel(&egalax_serial->const_hrtimer);
#endif

#ifdef _IDLE_MODE_SUPPORT
	del_timer_sync(&egalax_serial->idle_timer);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&egalax_early_suspend);
#endif

	serio_close(serio);
	serio_set_drvdata(serio, NULL);

	kfree(egalax_serial);
	p_egalax_serial_dev = NULL;

	EGALAX_DBG(DBG_MODULE, " Serial disconnect done!\n");

	return;
}

static int egalax_serial_connect(struct serio *serio, struct serio_driver *drv)
{
	int ret, i;

	EGALAX_DBG(DBG_MODULE, " Start serial connect ...\n");
	p_egalax_serial_dev = (struct _egalax_serial *)kzalloc(sizeof(struct _egalax_serial), GFP_KERNEL);
	if (!p_egalax_serial_dev) 
	{
		EGALAX_DBG(DBG_MODULE, " Request memory failed\n");
		ret = -ENOMEM;
		goto fail;
	}
	memset(p_egalax_serial_dev, 0, sizeof(struct _egalax_serial));

	p_egalax_serial_dev->serio = serio;
	serio_set_drvdata(serio, p_egalax_serial_dev);
	ret = serio_open(serio, drv);
	if(ret)
		goto fail1;

	p_egalax_serial_dev->work_state = MODE_WORKING;

	// setup timer
#ifdef _CONSTANT_TOUCH_SUPPORT
	hrtimer_init(&p_egalax_serial_dev->const_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL); // constant touch timer
	p_egalax_serial_dev->const_hrtimer.function = egalax_const_hrtimer_handle;
	p_egalax_serial_dev->enableConstTouch = 0;
#endif

#ifdef _IDLE_MODE_SUPPORT
	setup_timer(&p_egalax_serial_dev->idle_timer, egalax_idle_timer_routine, (unsigned long)p_egalax_serial_dev);
	mod_timer(&p_egalax_serial_dev->idle_timer,  jiffies+HZ*IDLE_INTERVAL*10);
#endif

	for(i=0; i<MAX_SUPPORT_POINT;i++)
	{
		PointBuf[i].Status = -1;
		PointBuf[i].X = PointBuf[i].Y = 0;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	egalax_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN; //EARLY_SUSPEND_LEVEL_STOP_DRAWING-10;
	egalax_early_suspend.suspend = egalax_serial_early_suspend;
	egalax_early_suspend.resume = egalax_serial_early_resume;
	register_early_suspend(&egalax_early_suspend);
	EGALAX_DBG(DBG_MODULE, " Register early_suspend done\n");
#endif

	EGALAX_DBG(DBG_MODULE, " Serial connect done!\n");
	return 0;

fail1:	
	//serio_close(serio);
	serio_set_drvdata(serio, NULL);
fail:
	kfree(p_egalax_serial_dev);
	p_egalax_serial_dev = NULL;
	return ret;
}

static struct serio_device_id egalax_serio_idtable[] = {
	{
		.type	= SERIO_RS232,
		.proto	= SERIO_EGALAX,
		.id	= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ 0 }
};

MODULE_DEVICE_TABLE(serio, egalax_serio_idtable);

static struct serio_driver egalax_serio_drv = {
	.driver		= {
		.name	= "egalax",
	},
	.description	= DRIVER_DESC,
	.id_table	= egalax_serio_idtable,
	.interrupt	= egalax_serial_interrupt,
	.connect	= egalax_serial_connect,
	.disconnect	= egalax_serial_disconnect,
};

static const struct file_operations egalax_cdev_fops = {
	.owner	= THIS_MODULE,
	.read	= egalax_cdev_read,
	.write	= egalax_cdev_write,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
	.ioctl	= egalax_cdev_ioctl,
#endif
	.poll	= egalax_cdev_poll,
	.open	= egalax_cdev_open,
	.release= egalax_cdev_release,
};

static void egalax_serial_ts_exit(void)
{
	dev_t devno = MKDEV(global_major, global_minor);
	
	EGALAX_DBG(DBG_MODULE, " Exit driver ...\n");

	if(p_char_dev)
	{
		if( p_char_dev->pFiFoBuf )
			kfree(p_char_dev->pFiFoBuf);
	
		cdev_del(&p_char_dev->cdev);
		kfree(p_char_dev);
		p_char_dev = NULL;
	}

	unregister_chrdev_region( devno, 1);

	if(!IS_ERR(egalax_class))
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
		class_device_destroy(egalax_class, devno);
#else
		device_destroy(egalax_class, devno);
#endif 
		class_destroy(egalax_class);
	}

	if(input_dev)
	{
		input_unregister_device(input_dev);
		input_dev = NULL;
	}

	serio_unregister_driver(&egalax_serio_drv);

#ifdef _ENABLE_DBG_LEVEL
	remove_proc_entry(PROC_FS_NAME, NULL);
#endif

	EGALAX_DBG(DBG_MODULE, " Exit driver done!\n");
}

static struct egalax_char_dev* setup_chardev(dev_t dev)
{
	struct egalax_char_dev *pCharDev;
	int result;

	pCharDev = kmalloc(1*sizeof(struct egalax_char_dev), GFP_KERNEL);
	if(!pCharDev) 
		goto fail_cdev;
	memset(pCharDev, 0, sizeof(struct egalax_char_dev));

	pCharDev->pFiFoBuf = kmalloc(sizeof(unsigned char)*FIFO_SIZE, GFP_KERNEL);
	if(!pCharDev->pFiFoBuf)
		goto fail_fifobuf;
	memset(pCharDev->pFiFoBuf, 0, sizeof(unsigned char)*FIFO_SIZE);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	spin_lock_init( &pCharDev->CharFiFoLock );
	pCharDev->pCharKFiFo = kfifo_init(pCharDev->pFiFoBuf, FIFO_SIZE, GFP_KERNEL, &pCharDev->CharFiFoLock);
	if( pCharDev->pCharKFiFo==NULL )
		goto fail_kfifo;
#else
	kfifo_init(&pCharDev->CharKFiFo, pCharDev->pFiFoBuf, FIFO_SIZE);
	if( !kfifo_initialized(&pCharDev->CharKFiFo) )
		goto fail_kfifo;
#endif
	
	pCharDev->OpenCnts = 0;
	cdev_init(&pCharDev->cdev, &egalax_cdev_fops);
	pCharDev->cdev.owner = THIS_MODULE;
	sema_init(&pCharDev->sem, 1);
	init_waitqueue_head(&pCharDev->fifo_inq);

	result = cdev_add(&pCharDev->cdev, dev, 1);
	if(result)
	{
		EGALAX_DBG(DBG_MODULE, " Failed at cdev added\n");
		goto fail_kfifo;
	}

	return pCharDev; 

fail_kfifo:
	kfree(pCharDev->pFiFoBuf);
fail_fifobuf:
	kfree(pCharDev);
fail_cdev:
	return NULL;
}

static int egalax_serial_ts_init(void)
{
	int result;
	dev_t devno = 0;

	EGALAX_DBG(DBG_MODULE, " Driver init ...\n");

	// Asking for a dynamic major unless directed otherwise at load time.
	if(global_major) 
	{
		devno = MKDEV(global_major, global_minor);
		result = register_chrdev_region(devno, 1, "egalax_serial");
	} 
	else 
	{
		result = alloc_chrdev_region(&devno, global_minor, 1, "egalax_serial");
		global_major = MAJOR(devno);
	}

	if (result < 0)
	{
		EGALAX_DBG(DBG_MODULE, " Cdev can't get major number\n");
		return 0;
	}

	// allocate the character device
	p_char_dev = setup_chardev(devno);
	if(!p_char_dev) 
	{
		result = -ENOMEM;
		goto fail;
	}

	egalax_class = class_create(THIS_MODULE, "egalax_serial");
	if(IS_ERR(egalax_class))
	{
		EGALAX_DBG(DBG_MODULE, " Failed in creating class.\n");
		result = -EFAULT;
		goto fail;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
	class_device_create(egalax_class, NULL, devno, NULL, "egalax_serial");
#else
	device_create(egalax_class, NULL, devno, NULL, "egalax_serial");
#endif
	EGALAX_DBG(DBG_MODULE, " Register egalax_serial cdev, major: %d \n",global_major);

	result = serio_register_driver(&egalax_serio_drv);
	if(result!=0)
		goto fail;

	input_dev = allocate_Input_Dev();
	if(input_dev==NULL)
	{
		EGALAX_DBG(DBG_MODULE, " allocate_Input_Dev failed\n");
		result = -EINVAL; 
		goto fail;
	}

#ifdef _ENABLE_DBG_LEVEL
	dbgProcFile = create_proc_entry(PROC_FS_NAME, 0666, NULL);
	if (dbgProcFile == NULL) 
	{
		remove_proc_entry(PROC_FS_NAME, NULL);
		EGALAX_DBG(DBG_MODULE, " Could not initialize /proc/%s\n", PROC_FS_NAME);
	}
	else
	{
		dbgProcFile->read_proc = egalax_proc_read;
		dbgProcFile->write_proc = egalax_proc_write;
		EGALAX_DBG(DBG_MODULE, " /proc/%s created\n", PROC_FS_NAME);
	}
#endif // #ifdef _ENABLE_DBG_LEVEL

	EGALAX_DBG(DBG_MODULE, " Driver init done!\n");
	return 0;

fail:	
	egalax_serial_ts_exit();
	return result;
}

 int egalax_disable(void)
{
      if(p_egalax_serial_dev == NULL){
	     printk("[egalax_serial]: No EETI touch device!\n");
	     return  -ENODEV;	
	}
      printk("[egalax_serial]: egalax_disable +\n");
	egalax_serial_early_suspend(NULL);
      printk("[egalax_serial]: egalax_disable -\n");

	return egalax_suspend_result;
}
EXPORT_SYMBOL(egalax_disable);
 int egalax_enable(void)
{
      if(p_egalax_serial_dev == NULL){
	     printk("[egalax_serial]: No EETI touch device!\n");
	     return -ENODEV;	
	}

	printk("[egalax_serial]: egalax_enable+\n");
	egalax_serial_early_resume(NULL);
	printk("[egalax_serial]: egalax_enable-\n");
	return 0;
}
 EXPORT_SYMBOL(egalax_enable);


module_init(egalax_serial_ts_init);
module_exit(egalax_serial_ts_exit);

MODULE_AUTHOR("EETI <touch_fae@eeti.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
