/*
 * drivers/power/bq20z45_battery.c
 *
 * Gas Gauge driver for TI's BQ20Z45
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#include <linux/miscdevice.h>
#include <mach/gpio.h>
#include "../../arch/arm/mach-tegra/gpio-names.h"
#include "../../arch/arm/mach-tegra/wakeups-t2.h"
#include <linux/delay.h>

#define GPIOPIN_CHARGER_ENABLE                TEGRA_GPIO_PR6
#define SMBUS_RETRY                                     (3)
#define GPIOPIN_LOW_BATTERY_DETECT	  TEGRA_GPIO_PW3
#define GPIOPIN_BATTERY_DETECT	         TEGRA_GPIO_PI3
#define BATTERY_POLLING_RATE                    (60)
#define EP101_RESERVE_CAPACITY                             (6920)
#define EP101_FULL_CAPACITY                                   (8326)
#define EP102_RESERVE_CAPACITY                             (9000)
#define EP102_FULL_CAPACITY                                   (12000)
#define EP103_RESERVE_CAPACITY                             (9500)
#define EP103_FULL_CAPACITY                                   (12600)
#define LIMIT_IC_EN (0)
#define LIMIT_IC_DIS (1) /*default set 1 to disable limit*/
#define TEMP_KELVIN_TO_CELCIUS                             (2731)
#ifdef CONFIG_ASUS_CHARGER_MODE
int ready_to_polling=0;
#else
int ready_to_polling=1;
#endif
int reboot_test_tool_installed=0;
bool check_rvsd_process=0;
int exit_charging_mode=0;
EXPORT_SYMBOL(ready_to_polling);
extern int asusec_is_battery_full_callback(int full);
extern  int mxt_enable(void);
extern  int mxt_disable(void);
enum {
       REG_MANUFACTURER_DATA,
	REG_STATE_OF_HEALTH,
	REG_TEMPERATURE,
	REG_VOLTAGE,
	REG_CURRENT,
	REG_TIME_TO_EMPTY,
	REG_TIME_TO_FULL,
	REG_STATUS,
	REG_CAPACITY,
	REG_SERIAL_NUMBER,
	REG_MAX
};
typedef enum {
	Charger_Type_Battery = 0,
	Charger_Type_AC,
	Charger_Type_USB,
	Charger_Type_Num,
	Charger_Type_Force32 = 0x7FFFFFFF
} Charger_Type;

#define BATTERY_MANUFACTURER_SIZE	12
#define BATTERY_NAME_SIZE		8

/* manufacturer access defines */
#define MANUFACTURER_ACCESS_STATUS	0x0006
#define MANUFACTURER_ACCESS_SLEEP	0x0011

/* battery status value bits */
#define BATTERY_CHARGING 		0x40
#define BATTERY_FULL_CHARGED		0x20
#define BATTERY_FULL_DISCHARGED 	0x10
#define BQ20Z45_DATA(_psp, _addr, _min_value, _max_value)	\
	{							\
		.psp = POWER_SUPPLY_PROP_##_psp,		\
		.addr = _addr,					\
		.min_value = _min_value,			\
		.max_value = _max_value,			\
	}

static struct bq20z45_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} bq20z45_data[] = {
       [REG_MANUFACTURER_DATA]= BQ20Z45_DATA(PRESENT, 0, 0, 65535),
       [REG_STATE_OF_HEALTH]= BQ20Z45_DATA(HEALTH, 0, 0, 65535),
	[REG_TEMPERATURE]       = BQ20Z45_DATA(TEMP, 0x08, 0, 65535),
	[REG_VOLTAGE]           = BQ20Z45_DATA(VOLTAGE_NOW, 0x09, 0, 20000),
	[REG_CURRENT]           = BQ20Z45_DATA(CURRENT_NOW, 0x0A, -32768, 32767),
	[REG_TIME_TO_EMPTY]     = BQ20Z45_DATA(TIME_TO_EMPTY_AVG, 0x12, 0, 65535),
	[REG_TIME_TO_FULL]      = BQ20Z45_DATA(TIME_TO_FULL_AVG, 0x13, 0, 65535),
	[REG_STATUS]            = BQ20Z45_DATA(STATUS, 0x16, 0, 65535),
	[REG_CAPACITY]          = BQ20Z45_DATA(CAPACITY, 0x0d, 0, 100),//battery HW request
	[REG_SERIAL_NUMBER]     = BQ20Z45_DATA(SERIAL_NUMBER, 0x1C, 0, 65535),
};

static enum power_supply_property bq20z45_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,

};
static int bq20z45_get_fc_bit(int *fc);
extern unsigned  get_usb_cable_status(void);
static void bq20418_charger_control(struct work_struct* work);
void charge_ic_enable(bool enable);
EXPORT_SYMBOL(charge_ic_enable);
int Configure_Charger_pin(void);
int bq20z45_check_alarm(int battery_status);
int  check_rvsd(void);
#define USB_NO_Cable 0
#define USB_DETECT_CABLE 1
#define USB_SHIFT 0
#define AC_SHIFT 1
#define USB_Cable ((1 << (USB_SHIFT)) | (USB_DETECT_CABLE))
#define USB_AC_Adapter ((1 << (AC_SHIFT)) | (USB_DETECT_CABLE))
#define USB_CALBE_DETECT_MASK (USB_Cable  | USB_DETECT_CABLE)
unsigned battery_cable_status=0;
unsigned battery_docking_status=0;
unsigned battery_driver_ready=0;
static int ac_on ;
static int usb_on ;
static int pseudo_suspend_charging_on=0 ;
#define PMU_GPIO_CHARGER_CONTROL_PIN (3)
#define CHARGER_STATE_ENABLED  (1)
#define CHARGER_STATE_DISABLED (2)
#define CHARGER_STATE_UNKNOW   (3)
void check_cabe_type(void)
{
      if(battery_cable_status == USB_AC_Adapter){
		ac_on = 1;
		usb_on = 0;
	}
	else if(battery_cable_status  == USB_Cable){
		usb_on = 1;
		/*
		 if(battery_docking_status)
			ac_on = 1;
		 else
		 */
		ac_on = 0;
	}
	/*
	else if(battery_docking_status){
		 ac_on = 1 ;
	        usb_on = 0;
	}
	*/
	else{
		ac_on = 0;
		usb_on = 0;
	}
}
static int bq20z45_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

static enum power_supply_property power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int bq20z45_get_psp(int reg_offset, enum power_supply_property psp,union power_supply_propval *val);

static int power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int ret=0;
	switch (psp) {

	case POWER_SUPPLY_PROP_ONLINE:
			if(psy->type == POWER_SUPPLY_TYPE_MAINS &&  ac_on )
				val->intval = 1;
			else if (psy->type == POWER_SUPPLY_TYPE_USB && usb_on)
				val->intval = 1;
			else if (psy->type == POWER_SUPPLY_TYPE_DOCK_AC&& battery_docking_status)
				val->intval = 1;
			else
				val->intval = 0;
		break;

	default:
		return -EINVAL;
	}
	return ret;
}

static char *supply_list[] = {
	"battery",
	"ac",
#ifndef REMOVE_USB_POWER_SUPPLY
	"usb",
#endif
};

static struct power_supply bq20z45_supply[] = {
	{
	.name		= "battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= bq20z45_properties,
	.num_properties	= ARRAY_SIZE(bq20z45_properties),
	.get_property	= bq20z45_get_property,
       },
	{
		.name                   = "ac",
		.type                     = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property =power_get_property,
	},
#ifndef REMOVE_USB_POWER_SUPPLY
	{
		.name                    = "usb",
		.type                      = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties =power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property =power_get_property,
	},
#endif
	{
		.name                    = "docking_ac",
		.type                      = POWER_SUPPLY_TYPE_DOCK_AC,
		.properties =power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property =power_get_property,
	},
};

static struct bq20z45_device_info {
	struct i2c_client	*client;
        struct delayed_work battery_stress_test;
	struct delayed_work thermal_stress_test;
	struct delayed_work pmu_stress_test;
	struct delayed_work status_poll_work ;
	struct delayed_work charger_pad_dock_detect ;
	  struct delayed_work charger_control;
	int smbus_status;
	int battery_present;
	int low_battery_present;
	unsigned int old_capacity;
	unsigned int cap_err;
	unsigned int old_temperature;
	unsigned int temp_err;
	int gpio_battery_detect;
	int gpio_low_battery_detect;
	int irq_battery_detect;
	int irq_low_battery_detect;
	spinlock_t		lock;
	struct miscdevice battery_misc;
	unsigned int prj_id;
	unsigned int reserver_capacity;
	unsigned int full_capacity;
	struct wake_lock low_battery_wake_lock;
	struct wake_lock cable_event_wake_lock;
} *bq20z45_device;

int bq20z45_smbus_read_data(int reg_offset,int byte)
{
     s32 ret=-EINVAL;
     int count=0;
	do{
		if(byte)
			ret=i2c_smbus_read_byte_data(bq20z45_device->client,bq20z45_data[reg_offset].addr);
		else
			ret=i2c_smbus_read_word_data(bq20z45_device->client,bq20z45_data[reg_offset].addr);

	}while((ret<0)&& (++count<=SMBUS_RETRY));
      return ret;
}

int bq20z45_smbus_write_data(int reg_offset,int byte, unsigned int value)
{
     s32 ret=-EINVAL;
     int count=0;

     do{
	    if(byte){
                  ret=i2c_smbus_write_byte_data(bq20z45_device->client,bq20z45_data[reg_offset].addr,value&0xFF);
	}
	    else{
	          ret=i2c_smbus_write_word_data(bq20z45_device->client,bq20z45_data[reg_offset].addr,value&0xFFFF);
		}

	}while((ret<0)&& (++count<=SMBUS_RETRY));
      return ret;
}
struct workqueue_struct *battery_work_queue=NULL;
static atomic_t device_count;
static ssize_t show_battery_smbus_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", bq20z45_device->smbus_status);
}
static DEVICE_ATTR(battery_smbus, S_IWUSR | S_IRUGO, show_battery_smbus_status,NULL);

static struct attribute *battery_smbus_attributes[] = {

	&dev_attr_battery_smbus.attr,
	NULL
};

static const struct attribute_group battery_smbus_group = {
	.attrs = battery_smbus_attributes,
};
static void battery_status_poll(struct work_struct *work)
{
       struct bq20z45_device_info *battery_device =container_of(work, struct bq20z45_device_info,status_poll_work.work);
	  //printk("battery_status_poll\n");
	//kobject_uevent(&bq20z45_supply.dev->kobj, KOBJ_CHANGE);
	power_supply_changed(&bq20z45_supply[Charger_Type_Battery]);

	/* Schedule next poll */
       bq20z45_device->battery_present =!(gpio_get_value(bq20z45_device->gpio_battery_detect));
	if(ready_to_polling &&  bq20z45_device->battery_present)
		queue_delayed_work(battery_work_queue, &battery_device->status_poll_work,BATTERY_POLLING_RATE*HZ);
}


static irqreturn_t battery_detect_isr(int irq, void *dev_id)
{
	bq20z45_device->battery_present =!(gpio_get_value(bq20z45_device->gpio_battery_detect));
	printk("battery_detect_isr battery %s \n",bq20z45_device->battery_present?"instered":"removed" );
	if(ready_to_polling && bq20z45_device->battery_present)
		queue_delayed_work(battery_work_queue, &bq20z45_device->status_poll_work, BATTERY_POLLING_RATE*HZ);
	return IRQ_HANDLED;
}

static irqreturn_t low_battery_detect_isr(int irq, void *dev_id)
{
	bq20z45_device->low_battery_present=!(gpio_get_value(bq20z45_device->gpio_low_battery_detect));
	printk("low_battery_detect_isr battery is %x\n",bq20z45_device->low_battery_present );
	cancel_delayed_work(&bq20z45_device->status_poll_work);
	cancel_delayed_work(&bq20z45_device->charger_control);
	queue_delayed_work(battery_work_queue,&bq20z45_device->status_poll_work,3*HZ);
	return IRQ_HANDLED;
}

void setup_detect_irq(void )
{
       s32 ret=0;

       bq20z45_device->gpio_battery_detect=GPIOPIN_BATTERY_DETECT;
	bq20z45_device->gpio_low_battery_detect=GPIOPIN_LOW_BATTERY_DETECT;
	 bq20z45_device->battery_present=0;
	bq20z45_device->low_battery_present=0;
	 tegra_gpio_enable(bq20z45_device->gpio_battery_detect);
       ret = gpio_request(bq20z45_device->gpio_battery_detect, "battery_detect");
	if (ret < 0) {
		printk("request battery_detect gpio failed\n");
		bq20z45_device->gpio_battery_detect = -1;
		goto setup_low_bat_irq;
	}

	bq20z45_device->irq_battery_detect = gpio_to_irq(bq20z45_device->gpio_battery_detect);
	if (bq20z45_device->irq_battery_detect < 0) {
		printk("invalid battery_detect GPIO\n");
		bq20z45_device->gpio_battery_detect = -1;
		bq20z45_device->irq_battery_detect = -1;
		goto setup_low_bat_irq;
	}
	//gpio_get_value(button->gpio)
	ret = gpio_direction_input(bq20z45_device->gpio_battery_detect);
	if (ret < 0) {
			printk("failed to configure GPIO\n");
			gpio_free(bq20z45_device->gpio_battery_detect);
			bq20z45_device->gpio_battery_detect= -1;
			goto setup_low_bat_irq;
	}


	ret = request_irq(bq20z45_device->irq_battery_detect , battery_detect_isr,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,"bq20z45-battery (detect)", NULL);
	if (ret < 0) {
            printk("failed to request  battery_detect irq\n");
	}
	 bq20z45_device->battery_present=!(gpio_get_value(bq20z45_device->gpio_battery_detect));
	printk("setup_irq: battery_present =%x   \n",bq20z45_device->battery_present);
 setup_low_bat_irq:
	 tegra_gpio_enable(bq20z45_device->gpio_low_battery_detect);
       ret = gpio_request(bq20z45_device->gpio_low_battery_detect, "low_battery_detect");
	if (ret < 0) {
		printk("request low_battery_detect gpio failed\n");
		bq20z45_device->gpio_low_battery_detect = -1;
		goto exit;
	}

	bq20z45_device->irq_low_battery_detect = gpio_to_irq(bq20z45_device->gpio_low_battery_detect);
	if (bq20z45_device->irq_low_battery_detect< 0) {
		printk("invalid low_battery_detect gpio\n");
		bq20z45_device->gpio_low_battery_detect = -1;
		bq20z45_device->irq_low_battery_detect = -1;
		goto exit;
	}

	ret = gpio_direction_input(bq20z45_device->gpio_low_battery_detect);
	if (ret < 0) {
			printk("failed to configure low_battery_detect  gpio\n");
			gpio_free(bq20z45_device->gpio_battery_detect);
			bq20z45_device->gpio_low_battery_detect= -1;
			goto exit;
	}
	ret = request_irq(bq20z45_device->irq_low_battery_detect , low_battery_detect_isr,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,"bq20z45-battery (low battery)", NULL);
       bq20z45_device->low_battery_present=!(gpio_get_value(bq20z45_device->gpio_low_battery_detect));
	if (ret < 0) {
            printk("failed to request  low)battery_detect irq\n");
	}
	enable_irq_wake(bq20z45_device->irq_low_battery_detect);
printk("setup_irq:low_battery_present=%x \n",bq20z45_device->low_battery_present);
exit:

	return;
}

static irqreturn_t charger_pad_dock_interrupt(int irq, void *dev_id)
{
    printk(KERN_INFO"charger_pad_dock_interrupt\n");
   // schedule_delayed_work(&bq20z45_device->charger_pad_dock_detect, 0.1*HZ);
   queue_delayed_work(battery_work_queue,&bq20z45_device->charger_pad_dock_detect, 0.2*HZ);
    return IRQ_HANDLED;
}

static void charger_pad_dock_detection(struct work_struct *w)
{
	int dock_in_value = 0;
	int charger_pad_dock_value = 0;

	dock_in_value = gpio_get_value(TEGRA_GPIO_PX5);
	charger_pad_dock_value = gpio_get_value(TEGRA_GPIO_PS1);
	printk(KERN_INFO"charger_pad_dock_detection:dock_in_value=%u  charger_pad_dock_value =%u\n",dock_in_value,charger_pad_dock_value );

	if(dock_in_value == 0 && charger_pad_dock_value ==0)
		battery_docking_status=true;
	else
		battery_docking_status=false;

	bq20418_charger_control(NULL);

}

static int bq20z45_get_health(enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;	/* Write to ManufacturerAccess with
	* ManufacturerAccess command and then
	* read the status */

	/*ret = i2c_smbus_write_word_data(bq20z45_device->client,
	bq20z45_data[REG_MANUFACTURER_DATA].addr,
	MANUFACTURER_ACCESS_STATUS);*/
	ret =bq20z45_smbus_write_data(REG_MANUFACTURER_DATA,0,MANUFACTURER_ACCESS_STATUS);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,"%s: i2c write for battery presence "
			"failed\n", __func__);
		return -EINVAL;
		}
	/*ret = i2c_smbus_read_word_data(bq20z45_device->client,bq20z45_data[REG_MANUFACTURER_DATA].addr);*/
	ret=bq20z45_smbus_read_data(REG_MANUFACTURER_DATA,0);

	if (ret < 0) {
		     dev_err(&bq20z45_device->client->dev,"%s: i2c read for battery presence "
			"failed\n", __func__);
		     return -EINVAL;
		}

	if (ret >= bq20z45_data[REG_MANUFACTURER_DATA].min_value && ret <= bq20z45_data[REG_MANUFACTURER_DATA].max_value) {
		/* Mask the upper nibble of 2nd byte and
		* lower byte of response then
		* shift the result by 8 to get status*/
		ret &= 0x0F00;
		ret >>= 8;
		if (psp == POWER_SUPPLY_PROP_PRESENT) {
			static char *present_text[] = {"Removed","Exist"};
			if (ret == 0x0F)				/* battery removed */
				val->intval = 0;
			else
				val->intval = 1;
			// printk("bq20z45_get_psp POWER_SUPPLY_PROP_PRESENT  %s\n",present_text[val->intval]);
		}
		else if (psp == POWER_SUPPLY_PROP_HEALTH) {
			static char *health_text[] = {"Unknown", "Good", "Overheat", "Dead", "Over voltage","Unspecified failure", "Cold"};
			if (ret == 0x09)
				val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			else if (ret == 0x0B)
				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
			else if (ret == 0x0C)
				val->intval = POWER_SUPPLY_HEALTH_DEAD;
			else
				val->intval = POWER_SUPPLY_HEALTH_GOOD;	
			// printk("bq20z45_get_psp POWER_SUPPLY_PROP_HEALTH %s\n",health_text[val->intval]);
		}

		} else {
		        val->intval = 0;
		}
	return 0;

}
static int bq20z45_get_chargingCurrent(void)
{
	s32 ret,count=0;
 	#define REG_CHARGING_CURRENT (0x14)
	if(check_rvsd_process)
		return 0xF;
	do{
		ret=i2c_smbus_read_word_data(bq20z45_device->client,REG_CHARGING_CURRENT);
	}while((ret<0)&& (++count<=SMBUS_RETRY));

	if (ret < 0) {
		printk(KERN_ERR"%s: i2c read for  REG_CHARGING_CURRENT failed");
		return 0;
	}
	if(!(ret>0 && ret <=65535)){
		printk("warnning:charging current over sepc ret=%u\n",ret);
		return 0;/*0 means over tempurate*/
	}
	printk("charging current ret=%u\n",ret);
	return ret;
}

static int bq20z45_get_chargingVoltage(void)
{
       s32 ret=0,count=0;
       #define REG_CHARGING_VOLTAGE (0x15)
	if(check_rvsd_process)
		return 0xF;
       do{
	    ret=i2c_smbus_read_word_data(bq20z45_device->client,REG_CHARGING_VOLTAGE);
	}while((ret<0)&& (++count<=SMBUS_RETRY));
	if (ret < 0) {
		printk(KERN_ERR"%s: i2c read for REG_CHARGING_VOLTAG failed ");
		return 0;
	}
	if(!(ret>0 && ret <=65535)){
		printk("warnning:charging voltage over sepc ret=%u\n",ret);
		return 0;/*0 means over tempurate*/
	}
	printk("charging voltage ret=%u\n",ret);
	return ret;
}

static int bq20z45_get_psp(int reg_offset, enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;
	/*ret = i2c_smbus_read_word_data(bq20z45_device->client,
		bq20z45_data[reg_offset].addr);*/
	ret=bq20z45_smbus_read_data(reg_offset,0);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c read for %d failed\n", __func__, reg_offset);

		if(psp == POWER_SUPPLY_PROP_TEMP && (++bq20z45_device->temp_err<=3) &&(bq20z45_device->old_temperature!=0xFF)){
			val->intval=bq20z45_device->old_temperature;
			printk("read battery's tempurate fail use old temperature=%u bq20z45_device->temp_err=%u\n",val->intval,bq20z45_device->temp_err);
			return 0;
		}
		else
		return -EINVAL;
	}
	      bq20z45_device->smbus_status=1;
	if (ret >= bq20z45_data[reg_offset].min_value &&
	    ret <= bq20z45_data[reg_offset].max_value) {
		val->intval = ret;
		if (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW) {
			printk("bq20z45_get_psp voltage_now =%u\n",val->intval );//4
			}
		if (psp == POWER_SUPPLY_PROP_STATUS) {
			/* mask the upper byte and then find the
			 * actual status */
                   static char *status_text[] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};

			if (!(ret & BATTERY_CHARGING) && (ac_on ||battery_docking_status))//DSG
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else if (ret & BATTERY_FULL_CHARGED)//fc
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else if (ret & BATTERY_FULL_DISCHARGED)//fd
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

				if(ac_on  || battery_docking_status || ( pseudo_suspend_charging_on &&usb_on )){
					if ((ret &BATTERY_FULL_CHARGED) /*||bq20z45_check_alarm(ret)*/|| !bq20z45_get_chargingCurrent()  || !bq20z45_get_chargingVoltage())
						charge_ic_enable(false);
					else
						charge_ic_enable(true);
					}
			else
				charge_ic_enable(false);
                       printk("bq20z45_get_psp val->intval =%s ret =%x charging=%x\n",status_text[val->intval] ,ret ,!(ret & BATTERY_CHARGING) );//4
		}
		else if (psp == POWER_SUPPLY_PROP_TEMP) {
			  ret -=TEMP_KELVIN_TO_CELCIUS;
			  bq20z45_device->old_temperature=val->intval = ret;
			 bq20z45_device->temp_err=0;
			  printk("bq20z45_get_psp  temp=%u\n",ret );
		}
	} else {
		val->intval = 0;
		if (psp == POWER_SUPPLY_PROP_STATUS){
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			printk("Error:bq20z45_get_psp POWER_SUPPLY_STATUS_UNKNOWN ret=%x  addr=%x %x %x %x\n",ret,bq20z45_data[reg_offset].addr,reg_offset,bq20z45_data[reg_offset].min_value,bq20z45_data[reg_offset].max_value);
		}

	}

	return 0;
}
static int bq20z45_get_capacity(union power_supply_propval *val)
{
	s32 ret;
	s32 temp_capacity;

	ret =bq20z45_smbus_read_data(REG_CAPACITY,1);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev, "%s: i2c read for %d "
			"failed bq20z45_device->cap_err=%u\n", __func__, REG_CAPACITY,bq20z45_device->cap_err);
		if(bq20z45_device->cap_err>5 || (bq20z45_device->old_capacity==0xFF))
		return -EINVAL;
		else{
			val->intval=bq20z45_device->old_capacity;
			bq20z45_device->cap_err++;
			printk("read capacity fail, use old capacity=%u bq20z45_device->cap_err=%u\n",val->intval,bq20z45_device->cap_err);
			return 0;
		}
	}
        /* bq20z45 spec says that this can be >100 %
         * even if max value is 100 % */

	temp_capacity = ((ret >= 100) ? 100 : ret);

	 /* lose 28%,59%,79% */
	if(temp_capacity >60 && temp_capacity <80)
		temp_capacity-=1;
	else if(temp_capacity >30&& temp_capacity <=60)
		temp_capacity-=2;
	else if(temp_capacity >=0&& temp_capacity <=30)
		temp_capacity-=3;

	/*Re-check capacity to avoid  that temp_capacity <0*/
	temp_capacity = ((temp_capacity <0) ? 0 : temp_capacity);
	val->intval=temp_capacity;


	bq20z45_device->old_capacity=val->intval;
	bq20z45_device->cap_err=0;
	printk("bq20z45_get_capacity val->intval=%u ret=%u\n",val->intval,ret,bq20z45_device->low_battery_present?"low battery event":" ");
	return 0;
}
static int bq20z45_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	u8 count;

	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_HEALTH:
			if (bq20z45_get_health(psp, val))
				goto error;
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			if (bq20z45_get_capacity(val))
				goto error;
			break;

		case POWER_SUPPLY_PROP_STATUS:
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		case POWER_SUPPLY_PROP_CURRENT_NOW:
		case POWER_SUPPLY_PROP_TEMP:
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		case POWER_SUPPLY_PROP_SERIAL_NUMBER:
			for (count = 0; count < REG_MAX; count++) {
				if (psp == bq20z45_data[count].psp)
					break;
			}

			if (bq20z45_get_psp(count, psp, val))
				return -EINVAL;//return -EINVAL;
			break;

		default:
			dev_err(&bq20z45_device->client->dev,
				"%s: INVALID property psp=%u\n", __func__,psp);
			return -EINVAL;
	}

	return 0;

	error:

	return -EINVAL;
}

#include "stress_test.c"
void docking_callback(void )
{
       printk("========================================================\n") ;
	printk("docking_callback  px5=%x ps1=%x \n",gpio_get_value(TEGRA_GPIO_PX5),gpio_get_value(TEGRA_GPIO_PS1) );
       printk("========================================================\n") ;
      if(!gpio_get_value(TEGRA_GPIO_PX5) && !gpio_get_value(TEGRA_GPIO_PS1)){
		battery_docking_status=true;
		enable_irq(gpio_to_irq(TEGRA_GPIO_PS1));
	}else{
		battery_docking_status=false;
		disable_irq(gpio_to_irq(TEGRA_GPIO_PS1));
	}

     if(battery_work_queue ){
		 cancel_delayed_work(&bq20z45_device->status_poll_work);
		 cancel_delayed_work(&bq20z45_device->charger_control);
		 queue_delayed_work(battery_work_queue,&bq20z45_device->charger_control,0);
	}
}
EXPORT_SYMBOL(docking_callback);
void   register_docking_charging_irq(void)
{
       int rc;
	tegra_gpio_enable(TEGRA_GPIO_PS1);
	rc = gpio_request(TEGRA_GPIO_PS1,"dock_charging");
	if (rc< 0)
		printk(KERN_ERR"TEGRA_GPIO_PS1 GPIO%d request fault!%d\n",TEGRA_GPIO_PS1,rc);
	rc= gpio_direction_input(TEGRA_GPIO_PS1);
	if (rc)
	        printk(KERN_ERR"gpio_direction_input failed for input TEGRA_GPIO_PS1=%d\n", TEGRA_GPIO_PS1);
	rc= request_irq(gpio_to_irq(TEGRA_GPIO_PS1), charger_pad_dock_interrupt, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "dock_charging" , NULL);
	if (rc < 0)
		printk(KERN_ERR"Could not register for TEGRA_GPIO_PS1 interrupt, irq = %d, rc = %d\n", gpio_to_irq(TEGRA_GPIO_PS1), rc);

	if(!gpio_get_value(TEGRA_GPIO_PX5) && !gpio_get_value(TEGRA_GPIO_PS1)){
		battery_docking_status=true;
		enable_irq(gpio_to_irq(TEGRA_GPIO_PS1));
	}else
		disable_irq(gpio_to_irq(TEGRA_GPIO_PS1));
	printk(" register_docking_charging_irq  px5=%x ps1=%x \n",gpio_get_value(TEGRA_GPIO_PX5),gpio_get_value(TEGRA_GPIO_PS1) );
}
void  setup_5v_chg_en_pin(void)
{
       int rc;
	rc = gpio_request(TEGRA_GPIO_PS5,"5v_chg_en");
	if (rc< 0)
		printk(KERN_ERR"TEGRA_GPIO_PS5 GPIO %d request fault!%d\n",TEGRA_GPIO_PS5,rc);
	rc= gpio_direction_output(TEGRA_GPIO_PS5,LIMIT_IC_DIS);
	tegra_gpio_enable(TEGRA_GPIO_PS5);
}
extern unsigned int ASUSGetProjectID( void );
#define VALUE_ARRY_SIZE (7)
unsigned short value[VALUE_ARRY_SIZE]={0};

void set_value(unsigned short buffer[])
{
	int i=0,ret=0;
	printk("bq20z45 set_value\n");
	while(i < (VALUE_ARRY_SIZE-1)){
		value[i]=buffer[i];
		i++;
	}
	value[VALUE_ARRY_SIZE-1]=0xFF;
	i=0;
	do{
		ret=check_rvsd();
	}while(ret && ++i<=3);
}
EXPORT_SYMBOL(set_value);
int  check_rvsd(void)
{
	#define	PF_STATUS_REGISTER	(0x53)
	#define	RSVD_BIT	(1<<14)
	unsigned short PF_Status=0;
	int ret=0;

	check_rvsd_process=1;
	if (value[VALUE_ARRY_SIZE-1]!=0xFF){
		printk("check_rvsd value not ready\n");
		return ret;
	}
	printk("check_rvsd  %x %x %x %x %x \n",value[0],value[1],value[2],value[3],value[4],value[5]);

	ret=i2c_smbus_write_word_data(bq20z45_device->client,0x0,value[0]);
	if(ret <0){
		printk("check_rvsd write fail 0\n");
		return ret;
	}

	ret=i2c_smbus_write_word_data(bq20z45_device->client,0x0,value[1]);
	if(ret <0){
		printk("check_rvsd write  fail 1\n");
		return ret;
	}

	PF_Status=i2c_smbus_read_word_data(bq20z45_device->client,PF_STATUS_REGISTER);
	if(ret <0){
		printk("check_rvsd read PF_STATUS_REGISTER fail\n");
		return ret;
	}

	//check PF status
	if(PF_Status == RSVD_BIT && ac_on ){
		// clear PF status
		charge_ic_enable(true);
		msleep(5000);
		ret=i2c_smbus_write_word_data(bq20z45_device->client,0x0,value[2]);
		if(ret <0){
			printk("check_rvsd write  fail 2\n");
			return ret;
		}

		ret=i2c_smbus_write_word_data(bq20z45_device->client,0x0,value[3]);
		if(ret <0){
			printk("check_rvsd write  fail 3\n");
			return ret;
		}

		ret=i2c_smbus_write_word_data(bq20z45_device->client,0x0,value[4]);
		if(ret <0){
			printk("check_rvsd write  fail 4\n");
			return ret;
		}
	}

	ret=i2c_smbus_write_word_data(bq20z45_device->client,0x0,value[5]);
	if(ret <0)
		printk("check_rvsd write  fail 5\n");

	check_rvsd_process=0;
	return ret;
}
static int bq20z45_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc;
	int i=0;
	printk("bq20z45_probe: client->addr=%x boot_reason=%x\n",client->addr,boot_reason);
	bq20z45_device = kzalloc(sizeof(*bq20z45_device), GFP_KERNEL);
	if (!bq20z45_device) {
		return -ENOMEM;
	}
	memset(bq20z45_device, 0, sizeof(*bq20z45_device));
       if (!Configure_Charger_pin()){
		pr_debug("bq20z45_probe: gpio open fail, disable charger controller\n");
	}
	bq20z45_device->client = client;
	i2c_set_clientdata(client, bq20z45_device);
       bq20z45_device->smbus_status=0;
       bq20z45_device->cap_err=0;
       bq20z45_device->temp_err=0;
	bq20z45_device->old_capacity=0xFF;
       bq20z45_device->old_temperature=0xFF;
	bq20z45_device->prj_id=ASUSGetProjectID();
	bq20z45_device->low_battery_present=0;
	if(bq20z45_device->prj_id==101){
		bq20z45_device->reserver_capacity = EP101_RESERVE_CAPACITY;
		bq20z45_device->full_capacity         = EP101_FULL_CAPACITY;
	}else if (bq20z45_device->prj_id==102){
		bq20z45_device->reserver_capacity = EP102_RESERVE_CAPACITY;
		bq20z45_device->full_capacity         = EP102_FULL_CAPACITY;
	}else{
		bq20z45_device->reserver_capacity = EP103_RESERVE_CAPACITY;
		bq20z45_device->full_capacity         = EP103_FULL_CAPACITY;
	}
	for (i = 0; i < ARRAY_SIZE(bq20z45_supply); i++) {
		rc = power_supply_register(&client->dev, &bq20z45_supply[i]);
		if (rc) {
			printk(KERN_ERR "Failed to register power supply\n");
			while (i--)
				power_supply_unregister(&bq20z45_supply[i]);
			kfree(bq20z45_device);
			return rc;
		}
	}

	dev_info(&bq20z45_device->client->dev,"%s: battery driver registered\n", client->name);
       spin_lock_init(&bq20z45_device->lock);
	INIT_DELAYED_WORK(&bq20z45_device->status_poll_work, battery_status_poll) ;
	INIT_DELAYED_WORK(&bq20z45_device->charger_control, bq20418_charger_control) ;
       INIT_DELAYED_WORK(&bq20z45_device->battery_stress_test,  battery_strees_test) ;
        battery_work_queue = create_singlethread_workqueue("battery_workqueue");
       INIT_DELAYED_WORK(&bq20z45_device->charger_pad_dock_detect,charger_pad_dock_detection);
	/* Register sysfs hooks */
	if (sysfs_create_group(&client->dev.kobj, &battery_smbus_group )) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
	}

	 register_docking_charging_irq();

	battery_cable_status = get_usb_cable_status();
	/*if(battery_cable_status==USB_AC_Adapter || battery_docking_status ){
		 cancel_delayed_work(&bq20z45_device->status_poll_work);
		 cancel_delayed_work(&bq20z45_device->charger_control);
		 bq20418_charger_control(NULL);
		}
	 else{
			charge_ic_enable(false);
			cancel_delayed_work(&bq20z45_device->status_poll_work);
			cancel_delayed_work(&bq20z45_device->charger_control);
		  #ifndef CONFIG_ASUS_CHARGER_MODE
			queue_delayed_work(battery_work_queue,&bq20z45_device->status_poll_work,5*HZ);
		  #endif
		}*/
	 cancel_delayed_work(&bq20z45_device->status_poll_work);
	 cancel_delayed_work(&bq20z45_device->charger_control);
	 bq20418_charger_control(NULL);
	 setup_5v_chg_en_pin();
	 setup_detect_irq();
        bq20z45_device->battery_misc.minor	= MISC_DYNAMIC_MINOR;
	 bq20z45_device->battery_misc.name	= "battery";
	 bq20z45_device->battery_misc.fops  	= &battery_fops;
               rc=misc_register(&bq20z45_device->battery_misc);
	 printk(KERN_INFO "battery register misc device for I2C stress test rc=%x\n", rc);

	 wake_lock_init(&bq20z45_device->low_battery_wake_lock, WAKE_LOCK_SUSPEND, "low_battery_detection");
	 wake_lock_init(&bq20z45_device->cable_event_wake_lock, WAKE_LOCK_SUSPEND, "battery_cable_event");
	 battery_driver_ready=1;
	return 0;
}

static int bq20z45_remove(struct i2c_client *client)
{
	struct bq20z45_device_info *bq20z45_device;
       int i=0;
	bq20z45_device = i2c_get_clientdata(client);

	for (i = 0; i < ARRAY_SIZE(bq20z45_supply); i++) {
		power_supply_unregister(&bq20z45_supply[i]);
	}
	if (bq20z45_device) {
		wake_lock_destroy(&bq20z45_device->low_battery_wake_lock);
		kfree(bq20z45_device);
		bq20z45_device = NULL;
	}

	return 0;
}

#if defined (CONFIG_PM)
static int bq20z45_suspend(struct i2c_client *client,
	pm_message_t state)
{
	//s32 ret;
	//struct bq20z45_device_info *bq20z45_device;
	//bq20z45_device = i2c_get_clientdata(client);
	cancel_delayed_work_sync(&bq20z45_device->status_poll_work);
	cancel_delayed_work_sync(&bq20z45_device->charger_control);
       flush_workqueue(battery_work_queue);
	/* write to manufacture access with sleep command */
	/*ret = i2c_smbus_write_word_data(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr,
		MANUFACTURER_ACCESS_SLEEP);
	if(bq20z45_device->battery_present ){
		ret=bq20z45_smbus_write_data(REG_MANUFACTURER_DATA,0,MANUFACTURER_ACCESS_SLEEP);
		if (ret < 0 ) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c write for %d failed\n",
				__func__, MANUFACTURER_ACCESS_SLEEP);
			return -EINVAL;
		}
	}else{
	printk("bq20z45_suspend: no battery");
	}
    */
	return 0;
}

/* any smbus transaction will wake up bq20z45 */
extern unsigned long wake_status;
extern unsigned long temp_wake_status;
static int bq20z45_resume(struct i2c_client *client)
{
      #define LOW_BATTERY_WAKESOURCE_PAD TEGRA_WAKE_GPIO_PW3

	bq20z45_device->battery_present =!(gpio_get_value(bq20z45_device->gpio_battery_detect));
	cancel_delayed_work(&bq20z45_device->status_poll_work);
	if(!gpio_get_value(TEGRA_GPIO_PX5) && !gpio_get_value(TEGRA_GPIO_PS1)){
		battery_docking_status=true;
		enable_irq(gpio_to_irq(TEGRA_GPIO_PS1));
	}else{
		battery_docking_status=false;
		disable_irq(gpio_to_irq(TEGRA_GPIO_PS1));
	}

	if(temp_wake_status&TEGRA_WAKE_USB1_VBUS){
		temp_wake_status&=(~TEGRA_WAKE_USB1_VBUS);
		battery_cable_status = get_usb_cable_status();
		check_cabe_type();
		if ( ac_on )
			power_supply_changed(&bq20z45_supply[Charger_Type_AC]);
		#ifndef REMOVE_USB_POWER_SUPPLY
		if ( usb_on )
			power_supply_changed(&bq20z45_supply[Charger_Type_USB]);
		#endif
	}

	if(ready_to_polling){
		if(wake_status&LOW_BATTERY_WAKESOURCE_PAD){
			queue_delayed_work(battery_work_queue,&bq20z45_device->status_poll_work,3*HZ);
			wake_lock_timeout(&bq20z45_device->low_battery_wake_lock, 6*HZ);
		}
		else
			queue_delayed_work(battery_work_queue,&bq20z45_device->status_poll_work,5*HZ);
	}
	printk("bq20z45_resume wake_status= %x", wake_status);

	return 0;
}
#endif

static const struct i2c_device_id bq20z45_id[] = {
	{ "bq20z45-battery", 0 },
	{},
};

static struct i2c_driver bq20z45_battery_driver = {
	.probe		= bq20z45_probe,
	.remove 	= bq20z45_remove,
#if defined (CONFIG_PM)
	.suspend	= bq20z45_suspend,
	.resume 	= bq20z45_resume,
#endif
	.id_table	= bq20z45_id,
	.driver = {
		.name	= "bq20z45-battery",
	},
};

 //#ifdef  SUPPORT_CHARGE_CONTROL
int bq20z45_check_alarm(int battery_status)
{
	#define BATTERY_STATUS_TCA			(1<<14)
	#define BATTERY_STATUS_OTA			(1<<12)
	if( battery_status & BATTERY_STATUS_TCA ){
		printk("bq20z45_check_alarm BATTERY_STATUS_TCA,Terminate Charge Alarm\n");//4
		return 1;
	}
	if( battery_status & BATTERY_STATUS_OTA  ){
		printk("bq20z45_check_alarm BATTERY_STATUS_OTA, Over Temperature Alarm \n");
		return 1;
	}
	return 0;
}
static int bq20z45_get_fc_bit(int *fc)
{
	s32 ret;
	#define BATTERY_STATUS_FC				(1<<5)

	/*ret = i2c_smbus_read_word_data(bq20z45_device->client,
		bq20z45_data[REG_STATUS].addr);*/
	ret=bq20z45_smbus_read_data(REG_STATUS,0);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c read for %d failed\n", __func__, REG_STATUS);
		return -EINVAL;
	}

	if (ret >= bq20z45_data[REG_STATUS].min_value &&
		ret <= bq20z45_data[REG_STATUS].max_value) {
			*fc= ret&BATTERY_STATUS_FC;
			printk("bq20z45_get_fc_bit fc=%x\n",*fc);//4
	}

	return ret;
}

int Configure_Charger_pin(void)
{
	int ret=0;

	ret = gpio_request(GPIOPIN_CHARGER_ENABLE, "charger_enable");
	if (ret < 0) {
		printk("request charger_enable gpio failed\n");
		goto Cleanup;
	}

	ret = gpio_direction_output(GPIOPIN_CHARGER_ENABLE,1/*default disable*/);
	if (ret < 0) {
			printk(" Configure_Charger_pin failed to configure GPIO TEGRA_GPIO_PR6\n");
			gpio_free(GPIOPIN_CHARGER_ENABLE);
			goto Cleanup;
	}
	tegra_gpio_enable(GPIOPIN_CHARGER_ENABLE);

	return 1;
Cleanup:
    return 0;
}
void charge_ic_enable(bool enable)
{
	int output_high=0;
	static int old_state=0xFF;
	int new_state=0xFF;

	if(enable ){
		output_high=false;
		new_state=CHARGER_STATE_ENABLED;
	}
	else{
		output_high=true;
		new_state=CHARGER_STATE_DISABLED;
	}
    if ( (enable) && (old_state!=new_state) )
		asusec_is_battery_full_callback(false);
    // printk("charge_ic_enable  enable=%x battery_cable_status =%u new_state=%x 0ld_state=%x output_high=%s\n",enable,battery_cable_status ,new_state,old_state,output_high?"HIGH":"LOW");
	//if(old_state!=new_state)
	{
		gpio_set_value(GPIOPIN_CHARGER_ENABLE,output_high);
		old_state=new_state;
	}
    if ( (!enable) && (old_state!=new_state) )
		asusec_is_battery_full_callback(true);
}

unsigned int  get_charge_ic_state(void )
{

	int  state=gpio_get_value(GPIOPIN_CHARGER_ENABLE);
	return (unsigned int)!state;
}
static void bq20418_charger_control(struct work_struct* work)
{
	int fc=1;
	int baytery_status=0;

	check_cabe_type();
	if( ready_to_polling ){
		if(ac_on ||( pseudo_suspend_charging_on && usb_on )/*||usb_on*/){
			baytery_status=bq20z45_get_fc_bit(&fc);
		if(fc /*||bq20z45_check_alarm(baytery_status)*/||!bq20z45_get_chargingCurrent()|| !bq20z45_get_chargingVoltage())
			charge_ic_enable(false);
		else
			charge_ic_enable(true);
		}
		else
			charge_ic_enable(false);

              if(battery_work_queue ){
			cancel_delayed_work(&bq20z45_device->status_poll_work);
			queue_delayed_work(battery_work_queue,&bq20z45_device->status_poll_work,HZ*3);
		}

	}
}
/*
 * 0000: no cable
 * 0001: have cable
 * 0001: USB cable
 * 0010: no this stage
 * 0011: AC apdater
 * 0100: Car Mount only
 * 0101: Car Mount+USB Cable
 * 0110: no this stage
 * 0111: Car Mount+AC apdater
 * 1XXX: Car Mount has power
 * 0XXX: Car mount does not have power
 */

void battery_callback(unsigned usb_cable_state)
{
	int old_cable_status=battery_cable_status;
	printk("========================================================\n") ;
	printk("battery_callback  usb_cable_state =%x\n",usb_cable_state ) ;
	printk("========================================================\n") ;
	battery_cable_status = usb_cable_state;
	if(!battery_driver_ready)
		return;

	wake_lock_timeout(&bq20z45_device->cable_event_wake_lock, 2*HZ);
	if(! battery_cable_status){
		if ( old_cable_status == USB_AC_Adapter){
			power_supply_changed(&bq20z45_supply[Charger_Type_AC]);
		}
		#ifndef REMOVE_USB_POWER_SUPPLY
		else if ( old_cable_status == USB_Cable){
			power_supply_changed(&bq20z45_supply[Charger_Type_USB]);
		}
		#endif
	}else if ( battery_cable_status == USB_Cable){
		power_supply_changed(&bq20z45_supply[Charger_Type_USB]);
	}else if ( battery_cable_status == USB_AC_Adapter){
		power_supply_changed(&bq20z45_supply[Charger_Type_AC]);
	}

         if(battery_work_queue ){
		 cancel_delayed_work(&bq20z45_device->status_poll_work);
		 cancel_delayed_work(&bq20z45_device->charger_control);
		 queue_delayed_work(battery_work_queue,&bq20z45_device->charger_control,0);
	}

}
EXPORT_SYMBOL(battery_callback);
void pseudo_suspend_charging_mode_en(int enable)
{
	printk("en_pseudo_suspend_charging_mode enable=%u usb_on=%u\n", enable,usb_on);
	if(  enable && usb_on)
		gpio_set_value(TEGRA_GPIO_PS5, LIMIT_IC_EN);
	else
		gpio_set_value(TEGRA_GPIO_PS5, LIMIT_IC_DIS);
	charge_ic_enable(enable);
}
int battery_charger_callback(unsigned int enable)
{
    check_cabe_type();
    printk("battery_charger_callback  usb_on=%u enable=%u \n",usb_on,enable);
    if ( (!usb_on) && (!pseudo_suspend_charging_on) )
		return 0;
    if(enable){
		pseudo_suspend_charging_on=true;
		pseudo_suspend_charging_mode_en(true);
    }else{
		pseudo_suspend_charging_on=false;
		pseudo_suspend_charging_mode_en(false);
    }
    return 0;
}
EXPORT_SYMBOL(battery_charger_callback);
//#endif

static int __init bq20z45_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq20z45_battery_driver);
	if (ret)
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c_add_driver failed\n", __func__);

	return ret;
}
module_init(bq20z45_battery_init);

static void __exit bq20z45_battery_exit(void)
{
	i2c_del_driver(&bq20z45_battery_driver);
}
module_exit(bq20z45_battery_exit);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("BQ20z45 battery monitor driver");
MODULE_LICENSE("GPL");
