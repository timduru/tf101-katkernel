/*
 * tegra_soc_voodoo_sound.c -- sysfs ALSA controls for tegra SoC
 *
 * Copyright (c) 2010-2011, Françs Simond aka supercurio on twitter & XDA
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

/*
 * ChangeLog:
 * 2012/07/22 - Timduru: Initial import of supercurio HC patches into 2.6.39.4 JB Kernel tree
 * 2012/07/28 - Timduru: Modifications to get it to compile with TeamEos3 Roach JB kernel tree
 * 2012/08/02 - Timduru: Modifications to get it to compile with Guevor JB #696
 * 2012/08/02 - Timduru: Add Module compile support option
 * 2012/08/03 - Timduru: Cleanup
 *
 *
*/


#include <../../../include/asm-generic/int-ll64.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

#include "voodoo_defs.h"
#include <sound/soc.h>
#include "../codecs/codec_param.h"
#include "../codecs/wm8903.h"

extern struct snd_soc_codec *global_codec;
extern struct wm8903_parameters audio_params[];


MODULE_DESCRIPTION("Voodoo sound for Eee Pad Transformer");
MODULE_AUTHOR("supercurio");
MODULE_LICENSE("GPL");


void write_hpvol(struct snd_soc_codec *codec, unsigned short level)
{
	unsigned short val;

	val = level;
	val |= WM8903_HPOUTLZC;
	snd_soc_write(codec, WM8903_ANALOGUE_OUT1_LEFT, val);

	val |= WM8903_HPOUTVU;
	snd_soc_write(codec, WM8903_ANALOGUE_OUT1_RIGHT, val);
}

void update_hpvol(struct snd_soc_codec *codec)
{
	unsigned short val;
	short steps;
	unsigned short hp_level_old;

	// read previous level
	val = snd_soc_read(codec, WM8903_ANALOGUE_OUT1_LEFT);
	val &= ~(WM8903_HPOUTVU_MASK);
	val &= ~(WM8903_HPOUTLZC_MASK);
	val &= ~(WM8903_HPL_MUTE_MASK);
	hp_level_old = val;

	// calculate number of steps for volume fade
	steps = audio_params[DEVICEID].analog_headset_volume - hp_level_old;

	while (steps != 0) {
		if (audio_params[DEVICEID].analog_headset_volume < hp_level_old)
			steps++;
		else
			steps--;

		write_hpvol(codec,
			    audio_params[DEVICEID].analog_headset_volume - steps);

		if (steps != 0)
			udelay(1000);
	}
}

static ssize_t headphone_amplifier_level_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	return sprintf(buf, "%u\n", audio_params[DEVICEID].analog_headset_volume);
}

static ssize_t headphone_amplifier_level_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t size)
{
	unsigned short vol;
	if (sscanf(buf, "%hu", &vol) == 1) {

		// hard limit to 62 because 63 introduces distortions
		if (vol > 62)
			vol = 62;

		audio_params[DEVICEID].analog_headset_volume = vol;

		update_hpvol(global_codec);
	}
	return size;
}

static ssize_t voodoo_sound_version(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", VOODOO_SOUND_VERSION);
}

static ssize_t voodoo_sound_hardware(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "WM%X\n",
		       snd_soc_read(global_codec, WM8903_SW_RESET_AND_ID));
}

static DEVICE_ATTR(headphone_amplifier_level, S_IRUGO | S_IWUGO,
		   headphone_amplifier_level_show,
		   headphone_amplifier_level_store);

static DEVICE_ATTR(version, S_IRUGO, voodoo_sound_version, NULL);

static DEVICE_ATTR(hardware, S_IRUGO, voodoo_sound_hardware, NULL);

#ifdef MODULE
static DEVICE_ATTR(module, 0,
		   NULL,
		   NULL);
#endif

static struct attribute *voodoo_sound_attributes[] = {
	&dev_attr_headphone_amplifier_level.attr,
	&dev_attr_hardware.attr,
	&dev_attr_version.attr,
#ifdef MODULE
	&dev_attr_module.attr,
#endif
	NULL
};

static struct attribute_group voodoo_sound_group = {
	.attrs = voodoo_sound_attributes,
};

static struct miscdevice voodoo_sound_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "voodoo_sound",
};

static int __init voodoo_sound_init(void)
{
	printk("Voodoo sound: initializing driver v%d\n", VOODOO_SOUND_VERSION);

	misc_register(&voodoo_sound_device);
	if (sysfs_create_group(&voodoo_sound_device.this_device->kobj,
			       &voodoo_sound_group) < 0) {
		printk("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for (%s)!\n",
		       voodoo_sound_device.name);
	}
	return 0;
}

void __exit voodoo_sound_exit(void)
{
	printk("Voodoo sound: removing driver v%d\n", VOODOO_SOUND_VERSION);

	sysfs_remove_group(&voodoo_sound_device.this_device->kobj,
			   &voodoo_sound_group);
	misc_deregister(&voodoo_sound_device);
}

module_init(voodoo_sound_init);
module_exit(voodoo_sound_exit);
