/*
 * drivers/rtc/rtc-tps6586x.c
 *
 * RTC driver for TI TPS6586x
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/tps6586x.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>

#define RTC_CTRL	0xc0
#define POR_RESET_N	BIT(7)
#define OSC_SRC_SEL	BIT(6)
#define RTC_ENABLE	BIT(5)	/* enables alarm */
#define RTC_BUF_ENABLE	BIT(4)	/* 32 KHz buffer enable */
#define PRE_BYPASS	BIT(3)	/* 0=1KHz or 1=32KHz updates */
#define CL_SEL_MASK	(BIT(2)|BIT(1))
#define CL_SEL_POS	1
#define RTC_ALARM1_HI	0xc1
#define RTC_COUNT4	0xc6
#define RTC_COUNT4_DUMMYREAD 0xc5  /* start a PMU RTC access by reading the register prior to the RTC_COUNT4 */
#define ALM1_VALID_RANGE_IN_SEC 0x3FFF /*only 14-bits width in second*/
struct tps6586x_rtc  *ptps6586x_rtc_dev;

struct tps6586x_rtc {
	unsigned long		epoch_start;
	int			irq;
	struct rtc_device	*rtc;
	bool			irq_en;
};

static int tps6586x_rtc_read_time(struct device *dev, struct rtc_time *tm);
static int tps6586x_rtc_set_time(struct device *dev, struct rtc_time *tm);
static ssize_t store_rtc_time(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	#define YEAR_OFFSET (0)
	#define YEAR_SIZE (4)
	#define MON_OFFSET (YEAR_SIZE+1)
	#define MON_SIZE (2)
	#define MDAY_OFFSET (MON_OFFSET + MON_SIZE+1)
	#define MDAY_SIZE (2)
       int ret=count;
	int err=0;
	struct rtc_time tm;
	char year[10]={0};
	char mon[10]={0};
	char mday[10]={0};

	memcpy(year,buf,YEAR_SIZE);
	memcpy(mon,buf+MON_OFFSET,MON_SIZE);
	memcpy(mday,buf+MDAY_OFFSET ,MDAY_SIZE);
	printk("year=%s mon=%s day=%s\n",year,mon,mday);

	tm.tm_year=simple_strtol(year, NULL, 0);
	tm.tm_year-=1900;
	tm.tm_mon=simple_strtol(mon, NULL, 0);
	tm.tm_mon-=1;
	tm.tm_mday=simple_strtol(mday, NULL, 0);
	tm.tm_hour=0;
	tm.tm_min=0;
	tm.tm_sec=0;
	err = rtc_valid_tm(&tm);
	if (err != 0){
		printk("store_rtc_time: not vaild time\n");
		goto fail;
	}
	err = mutex_lock_interruptible(&ptps6586x_rtc_dev->rtc->ops_lock);
	if (err)
		goto fail;
	tps6586x_rtc_set_time(ptps6586x_rtc_dev->rtc->dev.parent,&tm);
	mutex_unlock(&ptps6586x_rtc_dev->rtc->ops_lock);
fail:
	return ret;
}
static ssize_t show_rtc_time(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct rtc_time tm;
	int err=0;

	printk("show_rtc_time\n");
	err = mutex_lock_interruptible(&ptps6586x_rtc_dev->rtc->ops_lock);
	if (err)
		goto fail;
	tps6586x_rtc_read_time(ptps6586x_rtc_dev->rtc->dev.parent, &tm);
	mutex_unlock(&ptps6586x_rtc_dev->rtc->ops_lock);
	return sprintf(buf, "%d/%d/%d\n", tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday);
fail:
	return 0;
}
static DEVICE_ATTR(rtc_time, S_IWUSR | S_IRUGO, show_rtc_time,store_rtc_time);

static struct attribute *tps_rtc_attributes[] = {
	&dev_attr_rtc_time.attr,
	NULL
};
static const struct attribute_group tps_rtc_group = {
	.attrs = tps_rtc_attributes,
};
static inline struct device *to_tps6586x_dev(struct device *dev)
{
	return dev->parent;
}

static int tps6586x_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	unsigned long long ticks = 0;
	unsigned long seconds;
	u8 buff[6];
	int err;
	int i;

	err = tps6586x_reads(tps_dev, RTC_COUNT4_DUMMYREAD, sizeof(buff), buff);
	if (err < 0) {
		dev_err(dev, "failed to read counter\n");
		return err;
	}

	for (i = 1; i < sizeof(buff); i++) {
		ticks <<= 8;
		ticks |= buff[i];
	}

	seconds = ticks >> 10;

	seconds += rtc->epoch_start;
	rtc_time_to_tm(seconds, tm);
	return rtc_valid_tm(tm);
}

static int tps6586x_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	unsigned long long ticks;
	unsigned long seconds;
	u8 buff[5];
	int err;

	rtc_tm_to_time(tm, &seconds);

	if (WARN_ON(seconds < rtc->epoch_start)) {
		dev_err(dev, "requested time unsupported\n");
		return -EINVAL;
	}

	seconds -= rtc->epoch_start;

	ticks = (unsigned long long)seconds << 10;
	buff[0] = (ticks >> 32) & 0xff;
	buff[1] = (ticks >> 24) & 0xff;
	buff[2] = (ticks >> 16) & 0xff;
	buff[3] = (ticks >> 8) & 0xff;
	buff[4] = ticks & 0xff;

	err = tps6586x_clr_bits(tps_dev, RTC_CTRL, RTC_ENABLE);
	if (err < 0) {
		dev_err(dev, "failed to clear RTC_ENABLE\n");
		return err;
	}

	err = tps6586x_writes(tps_dev, RTC_COUNT4, sizeof(buff), buff);
	if (err < 0) {
		dev_err(dev, "failed to program new time\n");
		return err;
	}

	err = tps6586x_set_bits(tps_dev, RTC_CTRL, RTC_ENABLE);
	if (err < 0) {
		dev_err(dev, "failed to set RTC_ENABLE\n");
		return err;
	}

	return 0;
}

static int tps6586x_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	unsigned long seconds;
	unsigned long ticks;
	unsigned long rtc_current_time;
	unsigned long long rticks = 0;
	u8 buff[3];
	u8 rbuff[6];
	int err;
	int i;

	if (rtc->irq == -1)
		return -EIO;

	rtc_tm_to_time(&alrm->time, &seconds);

	if (WARN_ON(alrm->enabled && (seconds < rtc->epoch_start))) {
		dev_err(dev, "can't set alarm to requested time\n");
		return -EINVAL;
	}

	if (alrm->enabled && !rtc->irq_en) {
		enable_irq(rtc->irq);
		rtc->irq_en = true;
	} else if (!alrm->enabled && rtc->irq_en) {
		disable_irq(rtc->irq);
		rtc->irq_en = false;
	}

	seconds -= rtc->epoch_start;

	err = tps6586x_reads(tps_dev, RTC_COUNT4_DUMMYREAD, sizeof(rbuff), rbuff);
	if (err < 0) {
		dev_err(dev, "failed to read counter\n");
		return err;
	}

	for (i = 1; i < sizeof(rbuff); i++) {
		rticks <<= 8;
		rticks |= rbuff[i];
	}

	rtc_current_time = rticks >> 10;
	if ((seconds - rtc_current_time) > ALM1_VALID_RANGE_IN_SEC)
		seconds = rtc_current_time - 1;

	ticks = (unsigned long long)seconds << 10;

	buff[0] = (ticks >> 16) & 0xff;
	buff[1] = (ticks >> 8) & 0xff;
	buff[2] = ticks & 0xff;

	err = tps6586x_writes(tps_dev, RTC_ALARM1_HI, sizeof(buff), buff);
	if (err)
		dev_err(tps_dev, "unable to program alarm\n");

	return err;
}

static int tps6586x_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	unsigned long ticks;
	unsigned long seconds;
	u8 buff[3];
	int err;

	err = tps6586x_reads(tps_dev, RTC_ALARM1_HI, sizeof(buff), buff);
	if (err)
		return err;

	ticks = (buff[0] << 16) | (buff[1] << 8) | buff[2];
	seconds = ticks >> 10;
	seconds += rtc->epoch_start;

	rtc_time_to_tm(seconds, &alrm->time);

	return 0;
}

static int tps6586x_rtc_alarm_irq_enable(struct device *dev,
					 unsigned int enabled)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	u8 buff;
	int err;

	if (rtc->irq == -1)
		return -EIO;

	err = tps6586x_read(tps_dev, RTC_CTRL, &buff);
	if (err < 0) {
		dev_err(dev, "failed to read RTC_CTRL\n");
		return err;
	}

	if ((enabled && (buff & RTC_ENABLE)) ||
	    (!enabled && !(buff & RTC_ENABLE)))
		return 0;

	if (enabled) {
		err = tps6586x_set_bits(tps_dev, RTC_CTRL, RTC_ENABLE);
		if (err < 0) {
			dev_err(dev, "failed to set RTC_ENABLE\n");
			return err;
		}

		if (!rtc->irq_en) {
			enable_irq(rtc->irq);
			rtc->irq_en = true;
		}
	} else {
		err = tps6586x_clr_bits(tps_dev, RTC_CTRL, RTC_ENABLE);
		if (err < 0) {
			dev_err(dev, "failed to clear RTC_ENABLE\n");
			return err;
		}

		if (rtc->irq_en) {
			disable_irq(rtc->irq);
			rtc->irq_en = false;
		}
	}

	return 0;
}

static const struct rtc_class_ops tps6586x_rtc_ops = {
	.read_time	= tps6586x_rtc_read_time,
	.set_time	= tps6586x_rtc_set_time,
	.set_alarm	= tps6586x_rtc_set_alarm,
	.read_alarm	= tps6586x_rtc_read_alarm,
	.alarm_irq_enable = tps6586x_rtc_alarm_irq_enable,
};

static irqreturn_t tps6586x_rtc_irq(int irq, void *data)
{
	struct device *dev = data;
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);

	rtc_update_irq(rtc->rtc, 1, RTC_IRQF | RTC_AF);
	return IRQ_HANDLED;
}

static int __devinit tps6586x_rtc_probe(struct platform_device *pdev)
{
	struct tps6586x_rtc_platform_data *pdata = pdev->dev.platform_data;
	struct device *tps_dev = to_tps6586x_dev(&pdev->dev);
	struct tps6586x_rtc *rtc;
	int err;
	struct tps6586x_epoch_start *epoch;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform_data specified\n");
		return -EINVAL;
	}

	rtc = kzalloc(sizeof(*rtc), GFP_KERNEL);

	if (!rtc)
		return -ENOMEM;

	rtc->irq = -1;

	if (pdata->irq < 0)
		dev_warn(&pdev->dev, "no IRQ specified, wakeup is disabled\n");

	epoch = &pdata->start;
	rtc->epoch_start = mktime(epoch->year, epoch->month, epoch->day,
				  epoch->hour, epoch->min, epoch->sec);

	dev_set_drvdata(&pdev->dev, rtc);

	device_init_wakeup(&pdev->dev, 1);

	rtc->rtc = rtc_device_register("tps6586x-rtc", &pdev->dev,
				       &tps6586x_rtc_ops, THIS_MODULE);

	if (IS_ERR(rtc->rtc)) {
		err = PTR_ERR(rtc->rtc);
		goto fail;
	}

	/* 1 kHz tick mode, enable tick counting */
	err = tps6586x_update(tps_dev, RTC_CTRL,
		RTC_ENABLE | OSC_SRC_SEL | ((pdata->cl_sel << CL_SEL_POS) &
					    CL_SEL_MASK),
		RTC_ENABLE | OSC_SRC_SEL | PRE_BYPASS | CL_SEL_MASK);
	if (err < 0) {
		dev_err(&pdev->dev, "unable to start counter\n");
		goto fail;
	}

	if (pdata && (pdata->irq >= 0)) {
		rtc->irq = pdata->irq;
		err = request_threaded_irq(pdata->irq, NULL, tps6586x_rtc_irq,
					   IRQF_ONESHOT, "tps6586x-rtc",
					   &pdev->dev);
		if (err) {
			dev_warn(&pdev->dev, "unable to request IRQ(%d)\n", rtc->irq);
			rtc->irq = -1;
		} else {
			enable_irq_wake(rtc->irq);
			disable_irq(rtc->irq);
		}
	}
	ptps6586x_rtc_dev=rtc;
	if (sysfs_create_group(&pdev->dev.kobj, &tps_rtc_group )) {
		printk(" tps6586x_rtc_probe : Not able to create the sysfs\n");
	}
	return 0;

fail:
	if (!IS_ERR_OR_NULL(rtc->rtc))
		rtc_device_unregister(rtc->rtc);
	device_init_wakeup(&pdev->dev, 0);
	kfree(rtc);
	return err;
}

static int __devexit tps6586x_rtc_remove(struct platform_device *pdev)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(&pdev->dev);

	if (rtc->irq != -1)
		free_irq(rtc->irq, rtc);
	rtc_device_unregister(rtc->rtc);
	kfree(rtc);
	return 0;
}

static struct platform_driver tps6586x_rtc_driver = {
	.driver	= {
		.name	= "tps6586x-rtc",
		.owner	= THIS_MODULE,
	},
	.probe	= tps6586x_rtc_probe,
	.remove	= __devexit_p(tps6586x_rtc_remove),
};

static int __init tps6586x_rtc_init(void)
{
	return platform_driver_register(&tps6586x_rtc_driver);
}
module_init(tps6586x_rtc_init);

static void __exit tps6586x_rtc_exit(void)
{
	platform_driver_unregister(&tps6586x_rtc_driver);
}
module_exit(tps6586x_rtc_exit);

MODULE_DESCRIPTION("TI TPS6586x RTC driver");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtc-tps6586x");
