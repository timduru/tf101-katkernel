/*
 *  Headset device detection driver.
 *
 * Copyright (C) 2011 ASUSTek Corporation.
 *
 * Authors:
 *  Jason Cheng <jason4_cheng@asus.com>
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
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/timer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <asm/string.h>
#include <mach/board-ventana-misc.h>
#include <sound/soc.h>
#include "../codecs/codec_param.h"
#include "../gpio-names.h"

MODULE_DESCRIPTION("Headset detection driver");
MODULE_LICENSE("GPL");

/*----------------------------------------------------------------------------
** FUNCTION DECLARATION
**----------------------------------------------------------------------------*/
static int   __init     	headset_init(void);
static void __exit    headset_exit(void);
static irqreturn_t   	detect_irq_handler(int irq, void *dev_id);
static void 		detection_work(struct work_struct *work);
static int               	jack_config_gpio(void);
static irqreturn_t   	lineout_irq_handler(int irq, void *dev_id);
static void 		lineout_work_queue(struct work_struct *work);
static int               	lineout_config_gpio(void);
static void 		detection_work(struct work_struct *work);
static int               	btn_config_gpio(void);
int 			hs_micbias_power(int on);
/*----------------------------------------------------------------------------
** GLOBAL VARIABLES
**----------------------------------------------------------------------------*/
#define JACK_GPIO		(TEGRA_GPIO_PW2) //178
#define LINEOUT_GPIO	(TEGRA_GPIO_PK5) //85
#define HOOK_GPIO		(TEGRA_GPIO_PX1) //185
#define ON	1
#define OFF	0

enum{
	NO_DEVICE = 0,
	HEADSET_WITH_MIC = 1,
	HEADSET_WITHOUT_MIC = 2,
};


struct headset_data {
	struct switch_dev sdev;
	struct input_dev *input;
	unsigned int irq;
	struct hrtimer timer;
	ktime_t debouncing_time;
};

static struct headset_data *hs_data;
bool headset_alive = false;
EXPORT_SYMBOL(headset_alive);
bool lineout_alive;
EXPORT_SYMBOL(lineout_alive);

static struct workqueue_struct *g_detection_work_queue;
static DECLARE_WORK(g_detection_work, detection_work);

struct work_struct headset_work;
struct work_struct lineout_work;
extern struct snd_soc_codec *global_codec;

extern bool need_spk;

extern struct wm8903_parameters audio_params[];

static ssize_t headset_name_show(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs_data->sdev)){
	case NO_DEVICE:{
		return sprintf(buf, "%s\n", "No Device");
		}
	case HEADSET_WITH_MIC:{
		return sprintf(buf, "%s\n", "HEADSET");
		}
	case HEADSET_WITHOUT_MIC:{
		return sprintf(buf, "%s\n", "HEADPHONE");
		}
	}
	return -EINVAL;
}

static ssize_t headset_state_show(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs_data->sdev)){
	case NO_DEVICE:
		return sprintf(buf, "%d\n", 0);
	case HEADSET_WITH_MIC:
		return sprintf(buf, "%d\n", 1);
	case HEADSET_WITHOUT_MIC:
		return sprintf(buf, "%d\n", 2);
	}
	return -EINVAL;
}

static void insert_headset(void)
{
	if(gpio_get_value(HOOK_GPIO)){
		printk("%s: headphone\n", __func__);
		switch_set_state(&hs_data->sdev, HEADSET_WITHOUT_MIC);
		hs_micbias_power(OFF);
		headset_alive = false;
	}else{
		printk("%s: headset\n", __func__);
		switch_set_state(&hs_data->sdev, HEADSET_WITH_MIC);
		hs_micbias_power(ON);
		headset_alive = true;
	}
	hs_data->debouncing_time = ktime_set(0, 20000000);  /* 20 ms */
}

static void remove_headset(void)
{
	switch_set_state(&hs_data->sdev, NO_DEVICE);
	hs_data->debouncing_time = ktime_set(0, 100000000);  /* 100 ms */
	headset_alive = false;
}

static void detection_work(struct work_struct *work)
{
	unsigned long irq_flags;
	int cable_in1;
	int mic_in = 0;

	hs_micbias_power(ON);
	/* Disable headset interrupt while detecting.*/
	local_irq_save(irq_flags);
	disable_irq(hs_data->irq);
	local_irq_restore(irq_flags);

	/* Delay 500ms for pin stable. */
	msleep(500);

	/* Restore IRQs */
	local_irq_save(irq_flags);
	enable_irq(hs_data->irq);
	local_irq_restore(irq_flags);

	if (gpio_get_value(JACK_GPIO) != 0) {
		/* Headset not plugged in */
		if (switch_get_state(&hs_data->sdev) == HEADSET_WITH_MIC || switch_get_state(&hs_data->sdev) == HEADSET_WITHOUT_MIC)
			remove_headset();
		goto exit_micbias_off;
	}

	cable_in1 = gpio_get_value(JACK_GPIO);
	mic_in  = gpio_get_value(HOOK_GPIO);
	if (cable_in1 == 0) {
	    printk("HOOK_GPIO value: %d\n", mic_in);
		if(switch_get_state(&hs_data->sdev) == NO_DEVICE)
			insert_headset();
		else if ( mic_in == 1)
			goto exit_micbias_off;
	} else{
		printk("HEADSET: Jack-in GPIO is low, but not a headset \n");
		goto exit_micbias_off;
	}

	return;

exit_micbias_off:
	hs_micbias_power(OFF);
	return;
}

static enum hrtimer_restart detect_event_timer_func(struct hrtimer *data)
{
	queue_work(g_detection_work_queue, &g_detection_work);
	return HRTIMER_NORESTART;
}

/**********************************************************
**  Function: Jack detection-in gpio configuration function
**  Parameter: none
**  Return value: if sucess, then returns 0
**
************************************************************/
static int jack_config_gpio()
{
	int ret;

	printk("HEADSET: Config Jack-in detection gpio\n");
	hs_micbias_power(ON);
	tegra_gpio_enable(JACK_GPIO);
	ret = gpio_request(JACK_GPIO, "h2w_detect");
	ret = gpio_direction_input(JACK_GPIO);

	hs_data->irq = gpio_to_irq(JACK_GPIO);
	ret = request_irq(hs_data->irq, detect_irq_handler,
			  IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "h2w_detect", NULL);

	ret = irq_set_irq_wake(hs_data->irq, 1);

	msleep(1);
	if (gpio_get_value(JACK_GPIO) == 0){
		insert_headset();
	}else {
		hs_micbias_power(OFF);
		headset_alive = false;
		remove_headset();
	}

	return 0;
}

/**********************************************************
**  Function: Headset Hook Key Detection interrupt handler
**  Parameter: irq
**  Return value: IRQ_HANDLED
**  High: Hook button pressed
************************************************************/
static int btn_config_gpio()
{
	int ret;

	printk("HEADSET: Config Headset Button detection gpio\n");

	tegra_gpio_enable(HOOK_GPIO);
	ret = gpio_request(HOOK_GPIO, "btn_INT");
	ret = gpio_direction_input(HOOK_GPIO);

	return 0;
}

static void lineout_work_queue(struct work_struct *work)
{
	msleep(300);

	if (gpio_get_value(LINEOUT_GPIO) == 0){
		printk("LINEOUT: LineOut inserted\n");
		lineout_alive = true;
		snd_soc_write(global_codec, 0x10, 0x0000); /* MIXSPK Disable*/
		snd_soc_write(global_codec, 0x11, 0x0000); /* SPK Disable*/
		snd_soc_write(global_codec, 0x76, 0x0000); /* Mute the speaker */

	}else if(gpio_get_value(LINEOUT_GPIO) && need_spk){
		printk("LINEOUT: LineOut removed\n");
		lineout_alive = false;
		snd_soc_write(global_codec, 0x10, 0x0003); /* MIXSPK Enable*/
		if(ASUSGetProjectID() == 101){
		snd_soc_write(global_codec, 0x3E, audio_params[TF101].analog_speaker_volume | 0x80); /* SPKL Volume: 4dB*/
		snd_soc_write(global_codec, 0x3F, audio_params[TF101].analog_speaker_volume | 0x80); /* SPKR Volume: 4dB*/
		}else if(ASUSGetProjectID() == 102){
		snd_soc_write(global_codec, 0x3E, audio_params[SL101].analog_speaker_volume | 0x80); /* SPKL Volume: 0dB*/
		snd_soc_write(global_codec, 0x3F,audio_params[SL101].analog_speaker_volume | 0x80); /* SPKR Volume: 0dB*/
		}
		snd_soc_write(global_codec, 0x11, 0x0003); /* SPK Enable*/
		snd_soc_write(global_codec, 0x76, 0x0033); /* GPIO3 configure: EN_SPK*/
	}

}

/**********************************************************
**  Function: LineOut Detection configuration function
**  Parameter: none
**  Return value: IRQ_HANDLED
**
************************************************************/
static int lineout_config_gpio()
{
	int ret;

	printk("HEADSET: Config LineOut detection gpio\n");

	tegra_gpio_enable(LINEOUT_GPIO);
	ret = gpio_request(LINEOUT_GPIO, "lineout_int");
	ret = gpio_direction_input(LINEOUT_GPIO);
	ret = request_irq(gpio_to_irq(LINEOUT_GPIO), &lineout_irq_handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "lineout_int", 0);

	if (gpio_get_value(LINEOUT_GPIO) == 0)
		lineout_alive = true;
	else
		lineout_alive = false;

	return 0;
}

/**********************************************************
**  Function: LineOut detection interrupt handler
**  Parameter: dedicated irq
**  Return value: if sucess, then returns IRQ_HANDLED
**
************************************************************/
static irqreturn_t lineout_irq_handler(int irq, void *dev_id)
{
	schedule_work(&lineout_work);
	return IRQ_HANDLED;
}

/**********************************************************
**  Function: Headset jack-in detection interrupt handler
**  Parameter: dedicated irq
**  Return value: if sucess, then returns IRQ_HANDLED
**
************************************************************/
static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	do {
		value1 = gpio_get_value(JACK_GPIO);
		irq_set_irq_type(hs_data->irq, value1 ?
				IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
		value2 = gpio_get_value(JACK_GPIO);
	} while (value1 != value2 && retry_limit-- > 0);

	if ((switch_get_state(&hs_data->sdev) == NO_DEVICE) ^ value2)
		hrtimer_start(&hs_data->timer, hs_data->debouncing_time, HRTIMER_MODE_REL);

	return IRQ_HANDLED;
}

int hs_micbias_power(int on)
{
	static int nLastVregStatus = -1;
	unsigned int CtrlReg = 0;

    int ret;

	if(on && nLastVregStatus!=ON){
		printk("HEADSET: Turn on micbias power\n");
		nLastVregStatus = ON;

		/* Mic Bias enable */
		CtrlReg = (0x1<<0) | (0x1<<1);
		ret=snd_soc_write(global_codec, 0x06, CtrlReg);

		/* Enable analog inputs */
		CtrlReg = (0x1<<1) | (0x1<<0);
		ret=snd_soc_write(global_codec, 0x0C, CtrlReg);

	}else if(!on && nLastVregStatus!=OFF){
		printk("HEADSET: Turn off micbias power\n");
		nLastVregStatus = OFF;

		/* Mic Bias disable */
		CtrlReg = (0x0<<0) | (0x0<<1);
		ret=snd_soc_write(global_codec, 0x06, CtrlReg);

		/* Disable analog inputs */
		CtrlReg = (0x0<<1) | (0x0<<0);
		ret=snd_soc_write(global_codec, 0x0C, CtrlReg);
	}
	return 0;
}
EXPORT_SYMBOL(hs_micbias_power);

/**********************************************************
**  Function: Headset driver init function
**  Parameter: none
**  Return value: none
**
************************************************************/
static int __init headset_init(void)
{

	int ret;

	printk(KERN_INFO "%s+ #####\n", __func__);

	hs_data = kzalloc(sizeof(struct headset_data), GFP_KERNEL);
	if (!hs_data){
		printk(KERN_INFO "%s- hs_data is NULL !#####\n", __func__);
		return -ENOMEM;
	}

	hs_data->debouncing_time = ktime_set(0, 100000000);  /* 100 ms */
	hs_data->sdev.name = "h2w";
	hs_data->sdev.print_name = headset_name_show;
	hs_data->sdev.print_state = headset_state_show;

	ret = switch_dev_register(&hs_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	g_detection_work_queue = create_workqueue("detection");

	hrtimer_init(&hs_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hs_data->timer.function = detect_event_timer_func;

	printk("HEADSET: Headset detection mode\n");
	btn_config_gpio();/*Config hook detection GPIO*/
	jack_config_gpio();/*Config jack detection GPIO*/

	printk("%s:ProjectID()=%d\n", __FUNCTION__, ASUSGetProjectID());
	if (ASUSGetProjectID() == 101){
		INIT_WORK(&lineout_work, lineout_work_queue);
		lineout_config_gpio();
	}
	printk(KERN_INFO "%s- #####\n", __func__);
	return 0;

err_switch_dev_register:
	printk(KERN_INFO "%s- Failed to register driver !#####\n", __func__);

	return ret;
}

/**********************************************************
**  Function: Headset driver exit function
**  Parameter: none
**  Return value: none
**
************************************************************/
static void __exit headset_exit(void)
{
	printk("HEADSET: Headset exit\n");
	if (switch_get_state(&hs_data->sdev))
		remove_headset();
	gpio_free(JACK_GPIO);
	gpio_free(HOOK_GPIO);

	if (ASUSGetProjectID() == 101){
		gpio_free(LINEOUT_GPIO);
	}

	free_irq(hs_data->irq, 0);
	destroy_workqueue(g_detection_work_queue);
	switch_dev_unregister(&hs_data->sdev);
}

module_init(headset_init);
module_exit(headset_exit);
