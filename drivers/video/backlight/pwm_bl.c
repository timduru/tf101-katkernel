/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>

#include <linux/delay.h>
#include <mach/dc.h>
#include <linux/gpio.h>
/* LCD_BL_EN, GPIO_PD4 */
#define ventana_bl_enb		28

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	int			(*notify)(struct device *,
					  int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
};

/* For power saving, the mapping between brightness & duty is modified as Non-Linear. */
static unsigned int asus_remapped_brightness[] = {
	0, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550,
	2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2550, 2805,
	2805, 2805, 2805, 2805, 3060, 3060, 3060, 3060, 3060, 3315, 3315, 3315, 3315, 3315, 3570, 3570,
	3570, 3570, 3570, 3825, 3825, 3825, 3825, 3825, 4080, 4080, 4080, 4080, 4080, 4335, 4335, 4335,
	4335, 4335, 4590, 4590, 4590, 4590, 4590, 4845, 4845, 4845, 4845, 4845, 5100, 5100, 5100, 5100,
	5100, 5355, 5355, 5355, 5355, 5610, 5610, 5610, 5610, 5865, 5865, 5865, 5865, 6120, 6120, 6120,
	6120, 6375, 6375, 6375, 6375, 6630, 6630, 6630, 6630, 6885, 6885, 6885, 6885, 7140, 7140, 7140,
	7140, 7395, 7395, 7395, 7395, 7650, 7650, 7650, 7650, 7905, 7905, 7905, 7905, 8160, 8160, 8160,
	8160, 8415, 8415, 8415, 8415, 8670, 8670, 8670, 8670, 8925, 8925, 8925, 8925, 9180, 9180, 9180,
	9180, 9435, 9435, 9435, 9435, 9690, 9690, 9690, 9690, 9945, 9945, 9945, 9945, 10200, 10200, 10200,
	10200, 10455, 10455, 10455, 10710, 10710, 10710, 10965, 10965, 10965, 11220, 11220, 11220, 11475, 11475, 11475,
	11730, 11730, 11730, 11985, 11985, 11985, 12240, 12240, 12240, 12495, 12495, 12495, 12750, 12750, 12750, 13005,
	13005, 13005, 13260, 13260, 13260, 13515, 13515, 13770, 13770, 14025, 14025, 14280, 14280, 14535, 14535, 14790,
	14790, 15045, 15045, 15300, 15300, 15555, 15555, 15810, 15810, 16065, 16065, 16320, 16575, 16830, 17085, 17340,
	17595, 17850, 18105, 18360, 18615, 18870, 19125, 19380, 19635, 19890, 20145, 20400, 20655, 20910, 21165, 21420,
	21675, 21930, 22185, 22440, 22695, 22950, 23205, 23460, 23715, 23970, 24225, 24480, 24735, 24990, 25245, 25500,
};

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;

	if (bl->props.power != FB_BLANK_UNBLANK) {
		printk("Can't update brightness 'cause of \"bl->props.power != FB_BLANK_UNBLANK\"\n");
		brightness = 0;
	}
	if (bl->props.fb_blank != FB_BLANK_UNBLANK) {
		printk("Can't update brightness 'cause of \"bl->props.fb_blank != FB_BLANK_UNBLANK\"\n");
		brightness = 0;
	}

	if (brightness == 0) {
		if (pb->notify) {
			/* ventana_backlight_notify(); */
			brightness = pb->notify(pb->dev, brightness);
		}

		/* HSD: TP7= 0 ms~ */
		msleep(5);

		printk("Disp: brightness= 0\n");
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
	} else {
		if(b_dc0_enabled) {
			/* period: pwm period in nsec; max: the max brightness, 255. */
			printk("Disp: brightness= %d(--> %d); PWM freq= %d Hz\n", brightness, asus_remapped_brightness[brightness], 1000000000/pb->period);
			pwm_config(pb->pwm, asus_remapped_brightness[brightness] * (pb->period / 100) / max, pb->period);
			pwm_enable(pb->pwm);

			if (gpio_get_value(ventana_bl_enb) == 0) {
				/* HSD: TP6= 10ms~ */
				msleep(10);
			}

			if (pb->notify) {
				/* ventana_backlight_notify(); */
				brightness = pb->notify(pb->dev, brightness);
			}
		}
	}
	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
};

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	int ret;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = kzalloc(sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pb->period = data->pwm_period_ns;
	pb->notify = data->notify;
	pb->check_fb = data->check_fb;
	pb->lth_brightness = data->lth_brightness *
		(data->pwm_period_ns / data->max_brightness);
	pb->dev = &pdev->dev;

	pb->pwm = pwm_request(data->pwm_id, "backlight");
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for backlight\n");
		ret = PTR_ERR(pb->pwm);
		goto err_pwm;
	} else
		dev_dbg(&pdev->dev, "got pwm for backlight\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
	pwm_free(pb->pwm);
err_pwm:
	kfree(pb);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	pwm_free(pb->pwm);
	kfree(pb);
	if (data->exit)
		data->exit(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM
static int pwm_backlight_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	if (pb->notify)
		pb->notify(pb->dev, 0);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	return 0;
}

static int pwm_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define pwm_backlight_suspend	NULL
#define pwm_backlight_resume	NULL
#endif

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name	= "pwm-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
	.suspend	= pwm_backlight_suspend,
	.resume		= pwm_backlight_resume,
};

static int __init pwm_backlight_init(void)
{
	return platform_driver_register(&pwm_backlight_driver);
}
module_init(pwm_backlight_init);

static void __exit pwm_backlight_exit(void)
{
	platform_driver_unregister(&pwm_backlight_driver);
}
module_exit(pwm_backlight_exit);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

