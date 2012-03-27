/* 
 * ASUS EC driver.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <asm/gpio.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/power_supply.h>
#include <mach/board-ventana-misc.h>

#include "asusec.h"
#include "elan_i2c_asus.h"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/*
 * functions declaration
 */
static int asusec_i2c_write_data(struct i2c_client *client, u16 data);
static int asusec_i2c_read_data(struct i2c_client *client);
static void asusec_reset_dock(void);
static int asusec_is_init_running(void);
static int asusec_chip_init(struct i2c_client *client);
static void asusec_work_function(struct work_struct *dat);
static void asusec_dock_init_work_function(struct work_struct *dat);
static void asusec_fw_update_work_function(struct work_struct *dat);
static int __devinit asusec_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int __devexit asusec_remove(struct i2c_client *client);
static int asusec_kp_key_mapping(int x);
static ssize_t asusec_show_dock(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_info_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_store_led(struct device *class,
		struct device_attribute *attr,const char *buf, size_t count);
static ssize_t asusec_store_ec_wakeup(struct device *class,
		struct device_attribute *attr,const char *buf, size_t count);
static ssize_t asusec_show_drain(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_show_dock_battery(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_show_dock_battery_all(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_show_dock_control_flag(struct device *class,
		struct device_attribute *attr,char *buf);


static int asusec_keypad_get_response(struct i2c_client *client, int res);
static int asusec_keypad_enable(struct i2c_client *client);
static int asusec_touchpad_get_response(struct i2c_client *client, int res);
static int asusec_touchpad_enable(struct i2c_client *client);
static int asusec_touchpad_disable(struct i2c_client *client);
//static int asusec_touchpad_reset(struct i2c_client *client);
static int asusec_suspend(struct i2c_client *client, pm_message_t mesg);
static int asusec_resume(struct i2c_client *client);
static int asusec_open(struct inode *inode, struct file *flip);
static int asusec_release(struct inode *inode, struct file *flip);
static long asusec_ioctl(struct file *flip, unsigned int cmd, unsigned long arg);
static ssize_t ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
static ssize_t ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);
static void BuffPush(char data);
static int asusec_input_device_create(struct i2c_client *client);
static ssize_t asusec_switch_name(struct switch_dev *sdev, char *buf);
static ssize_t asusec_switch_state(struct switch_dev *sdev, char *buf);
static int asusec_event(struct input_dev *dev, unsigned int type, unsigned int code, int value);	
static int asusec_dock_battery_get_capacity(union power_supply_propval *val);
static int asusec_dock_battery_get_status(union power_supply_propval *val);
static int asusec_dock_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val);
int asusec_close_keyboard(void);

/*
 * global variable
 */
static char host_to_ec_buffer[EC_BUFF_LEN];
static char ec_to_host_buffer[EC_BUFF_LEN];
static int h2ec_count;
static int buff_in_ptr;	  // point to the next free place
static int buff_out_ptr;	  // points to the first data

struct i2c_client dockram_client;
static struct class *asusec_class;
static struct device *asusec_device ;
struct asusec_chip *ec_chip;

struct cdev *asusec_cdev ;
static dev_t asusec_dev ;
static int asusec_major = 0 ;
static int asusec_minor = 0 ;

static struct workqueue_struct *asusec_wq;
struct delayed_work asusec_stress_work;

static const struct i2c_device_id asusec_id[] = {
	{"asusec", 0},
	{}
};

static enum power_supply_property asusec_dock_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
};

static struct power_supply asusec_power_supply[] = {
	{
		.name		= "dock_battery",
		.type		= POWER_SUPPLY_TYPE_DOCK_BATTERY,
		.properties	= asusec_dock_properties,
		.num_properties	= ARRAY_SIZE(asusec_dock_properties),
		.get_property	= asusec_dock_battery_get_property,
	},
};

MODULE_DEVICE_TABLE(i2c, asusec_id);

struct file_operations asusec_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = asusec_ioctl,
	.open = asusec_open,
	.write = ec_write,
	.read = ec_read,
	.release = asusec_release,
};

static struct i2c_driver asusec_driver = {
	.class	= I2C_CLASS_HWMON,
	.driver	 = {
		.name = "asusec",
		.owner = THIS_MODULE,
	},
	.probe	 = asusec_probe,
	.remove	 = __devexit_p(asusec_remove),
	.suspend = asusec_suspend,
	.resume = asusec_resume,
	.id_table = asusec_id,
};


static DEVICE_ATTR(ec_status, S_IWUSR | S_IRUGO, asusec_show,NULL);
static DEVICE_ATTR(ec_info, S_IWUSR | S_IRUGO, asusec_info_show,NULL);
static DEVICE_ATTR(ec_dock, S_IWUSR | S_IRUGO, asusec_show_dock,NULL);
static DEVICE_ATTR(ec_dock_led, S_IWUSR | S_IRUGO, NULL,asusec_store_led);
static DEVICE_ATTR(ec_wakeup, S_IWUSR | S_IRUGO, NULL,asusec_store_ec_wakeup);
static DEVICE_ATTR(ec_dock_discharge, S_IWUSR | S_IRUGO, asusec_show_drain,NULL);
static DEVICE_ATTR(ec_dock_battery, S_IWUSR | S_IRUGO, asusec_show_dock_battery,NULL);
static DEVICE_ATTR(ec_dock_battery_all, S_IWUSR | S_IRUGO, asusec_show_dock_battery_all,NULL);
static DEVICE_ATTR(ec_dock_control_flag, S_IWUSR | S_IRUGO, asusec_show_dock_control_flag,NULL);

static struct attribute *asusec_smbus_attributes[] = {
	&dev_attr_ec_status.attr,
	&dev_attr_ec_info.attr,
	&dev_attr_ec_dock.attr,
	&dev_attr_ec_dock_led.attr,
	&dev_attr_ec_wakeup.attr,
	&dev_attr_ec_dock_discharge.attr,
	&dev_attr_ec_dock_battery.attr,
	&dev_attr_ec_dock_battery_all.attr,
	&dev_attr_ec_dock_control_flag.attr,
NULL
};


static const struct attribute_group asusec_smbus_group = {
	.attrs = asusec_smbus_attributes,
};

static int asusec_kp_sci_table[]={0, KEY_SLEEP, KEY_WLAN, KEY_BLUETOOTH, 
		ASUSEC_KEY_TOUCHPAD, KEY_BRIGHTNESSDOWN, KEY_BRIGHTNESSUP, ASUSEC_KEY_AUTOBRIGHT, 
		KEY_CAMERA, -9, -10, -11, 
		-12, -13, -14, -15, 
		KEY_WWW, ASUSEC_KEY_SETTING, KEY_PREVIOUSSONG, KEY_PLAYPAUSE, 
		KEY_NEXTSONG, KEY_MUTE, KEY_VOLUMEDOWN, KEY_VOLUMEUP};

static void asusec_dockram_init(struct i2c_client *client){
	dockram_client.adapter = client->adapter;
	dockram_client.addr = 0x1b;
	dockram_client.detected = client->detected; 
	dockram_client.dev = client->dev;
	dockram_client.driver = client->driver;
	dockram_client.flags = client->flags;
	dockram_client.irq = client->irq;
	strcpy(dockram_client.name,client->name);
}

static int asusec_dockram_write_data(int cmd, int length)
{
	int ret = 0;

	if (ec_chip->dock_in == 0){
		return -1;
	}

	ret = i2c_smbus_write_i2c_block_data(&dockram_client, cmd, length, ec_chip->i2c_dm_data);
	if (ret < 0) {
		ASUSEC_ERR("Fail to read dockram data, status %d\n", ret);
	}
	return ret;
}

static int asusec_dockram_read_data(int cmd)
{
	int ret = 0;

	if (ec_chip->dock_in == 0){
		return -1;
	}
	
	ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_data);
	if (ret < 0) {
		ASUSEC_ERR("Fail to read dockram data, status %d\n", ret);
	}
	return ret;
}

static int asusec_i2c_write_data(struct i2c_client *client, u16 data)
{
	int ret = 0;

	if (ec_chip->dock_in == 0){
		return -1;
	}

	ret = i2c_smbus_write_word_data(client, 0x64, data);
	if (ret < 0) {
		ASUSEC_ERR("Fail to write data, status %d\n", ret);
	}
	return ret;
}

static int asusec_i2c_read_data(struct i2c_client *client)
{
	int ret = 0;

	if (ec_chip->dock_in == 0){
		return -1;
	}

	ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, ec_chip->i2c_data);
	ASUSEC_I2C_DATA(ec_chip->i2c_data, ec_chip->index);
	if (ret < 0) {
		ASUSEC_ERR("Fail to read data, status %d\n", ret);
	}
	return ret;
}

static int asusec_keypad_get_response(struct i2c_client *client, int res)
{
	int retry = ASUSEC_RETRY_COUNT;

	while(retry-- > 0){
		asusec_i2c_read_data(client);		
		ASUSEC_I2C_DATA(ec_chip->i2c_data, ec_chip->index);
		if ((ec_chip->i2c_data[1] & ASUSEC_OBF_MASK) && 
			(!(ec_chip->i2c_data[1] & ASUSEC_AUX_MASK))){ 
			if (ec_chip->i2c_data[2]  == res){
				goto get_asusec_keypad_i2c;
			}
		}		
		msleep(CONVERSION_TIME_MS/5);
	}
	return -1;

get_asusec_keypad_i2c:
	return 0;	

}

static int asusec_keypad_enable(struct i2c_client *client)
{
	int retry = ASUSEC_RETRY_COUNT;

	while(retry-- > 0){
		asusec_i2c_write_data(client, 0xF400);		
		if(!asusec_keypad_get_response(client, ASUSEC_PS2_ACK)){
			goto keypad_enable_ok;
		}
	}
	ASUSEC_ERR("fail to enable keypad");
	return -1;

keypad_enable_ok:
	return 0;
}

static int asusec_keypad_disable(struct i2c_client *client)
{	
	int retry = ASUSEC_RETRY_COUNT;	

	while(retry-- > 0){
		asusec_i2c_write_data(client, 0xF500);
		if(!asusec_keypad_get_response(client, ASUSEC_PS2_ACK)){
			goto keypad_disable_ok;
		}
	}

	ASUSEC_ERR("fail to disable keypad");
	return -1;

keypad_disable_ok:
	return 0;
}

static void asusec_keypad_led_on(struct work_struct *dat)
{	
	ec_chip->kbc_value = 1;
	ASUSEC_INFO("send led cmd 1\n");
	asusec_i2c_write_data(ec_chip->client, 0xED00);
}


static void asusec_keypad_led_off(struct work_struct *dat)
{	
	ec_chip->kbc_value = 0;
	ASUSEC_INFO("send led cmd 1\n");
	asusec_i2c_write_data(ec_chip->client, 0xED00);
}


static int asusec_touchpad_get_response(struct i2c_client *client, int res)
{
	int retry = ASUSEC_RETRY_COUNT;

	msleep(CONVERSION_TIME_MS);
	while(retry-- > 0){
		asusec_i2c_read_data(client);
		ASUSEC_I2C_DATA(ec_chip->i2c_data, ec_chip->index);
		if ((ec_chip->i2c_data[1] & ASUSEC_OBF_MASK) && 
			(ec_chip->i2c_data[1] & ASUSEC_AUX_MASK)){ 
			if (ec_chip->i2c_data[2] == res){
				goto get_asusec_touchpad_i2c;
			}
		}		
		msleep(CONVERSION_TIME_MS/5);
	}

	ASUSEC_ERR("fail to get touchpad response");
	return -1;

get_asusec_touchpad_i2c:
	return 0;	

}

static int asusec_touchpad_enable(struct i2c_client *client)
{
	ec_chip->tp_wait_ack = 1;		
	asusec_i2c_write_data(client, 0xF4D4);
	return 0;
}

static int asusec_touchpad_disable(struct i2c_client *client)
{	
	int retry = 5;	

	while(retry-- > 0){
		asusec_i2c_write_data(client, 0xF5D4);
		if(!asusec_touchpad_get_response(client, ASUSEC_PS2_ACK)){
			goto touchpad_disable_ok;
		}
	}

	ASUSEC_ERR("fail to disable touchpad");
	return -1;

touchpad_disable_ok:
	return 0;
}
/*
static int asusec_touchpad_reset(struct i2c_client *client)
{

	int retry = ASUSEC_RETRY_COUNT;

	while(retry-- > 0){
		asusec_i2c_write_data(client, 0xFFD4);
		msleep(CONVERSION_TIME_MS*5);
		if(asusec_touchpad_get_response(client, ASUSEC_PS2_ACK)){
			continue;
		}	
		if((ec_chip->i2c_data[3] == 0xAA) &&
			(ec_chip->i2c_data[4] == 0x00)){
			goto touchpad_reset_ok;
		}
	}

	ASUSEC_ERR("fail to reset touchpad");
	return -1;

touchpad_reset_ok:
	return 0;
}
*/

static void asusec_fw_clear_buf(void){
	int i;

	for (i = 0; i < 64; i++){
		i2c_smbus_read_byte_data(&dockram_client, 0);
	}
}

static void asusec_fw_reset_ec_op(void){
	char i2c_data[32];
	int i;
	int r_data[32];

	asusec_fw_clear_buf();
	
	i2c_data[0] = 0x01;
	i2c_data[1] = 0x21;
	for (i = 0; i < i2c_data[0]+1 ; i++){
		i2c_smbus_write_byte_data(&dockram_client, i2c_data[i],0);
	}
	msleep(CONVERSION_TIME_MS*4);
}

static void asusec_fw_address_set_op(void){
	char i2c_data[32];
	int i;
	int r_data[32];

	asusec_fw_clear_buf();

	i2c_data[0] = 0x05;
	i2c_data[1] = 0xa0;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x00;
	i2c_data[4] = 0x02;
	i2c_data[5] = 0x00;
	for (i = 0; i < i2c_data[0]+1 ; i++){
		i2c_smbus_write_byte_data(&dockram_client, i2c_data[i],0);
	}
	msleep(CONVERSION_TIME_MS*4);
}

static void asusec_fw_enter_op(void){
	char i2c_data[32];
	int i;
	int r_data[32];

	asusec_fw_clear_buf();

	i2c_data[0] = 0x05;
	i2c_data[1] = 0x10;
	i2c_data[2] = 0x55;
	i2c_data[3] = 0xaa;
	i2c_data[4] = 0xcd;
	i2c_data[5] = 0xbe;
	for (i = 0; i < i2c_data[0]+1 ; i++){
		i2c_smbus_write_byte_data(&dockram_client, i2c_data[i],0);
	}
	msleep(CONVERSION_TIME_MS*4);
}

static int asusec_fw_cmp_id(void){
	char i2c_data[32];
	int i;
	int r_data[32];
	int ret_val = 0;

	asusec_fw_clear_buf();

	i2c_data[0] = 0x01;
	i2c_data[1] = 0xC0;
	for (i = 0; i < i2c_data[0]+1 ; i++){
		i2c_smbus_write_byte_data(&dockram_client, i2c_data[i],0);
	}
	msleep(CONVERSION_TIME_MS*10);

	for (i = 0; i < 5; i++){
		r_data[i] = i2c_smbus_read_byte_data(&dockram_client, 0);
	}

	for (i = 0; i < 5; i++){
		ASUSEC_NOTICE("r_data[%d] = 0x%x\n", i, r_data[i]);
	}

	if (r_data[0] == 0xfa &&
		r_data[1] == 0xf0 &&
		r_data[2] == 0x12 &&
		r_data[3] == 0xef &&
		r_data[4] == 0x12){
		ret_val = 0;
	} else {
		ret_val = 1;
	}

	return ret_val;
}

static void asusec_fw_reset(void){

	if (asusec_fw_cmp_id() == 0){
		asusec_fw_enter_op();
		asusec_fw_address_set_op();
		asusec_fw_reset_ec_op();
		asusec_fw_clear_buf();
		if (ec_chip->re_init == 0){
			queue_delayed_work(asusec_wq, &ec_chip->asusec_dock_init_work, HZ/2);
			ec_chip->re_init = 1;
		}
	}
}
static int asusec_i2c_test(struct i2c_client *client){
	return asusec_i2c_write_data(client, 0x0000);
}

static void asusec_reset_dock(void){
	ec_chip->dock_init = 0;
	ASUSEC_NOTICE("send EC_Request\n");	
	gpio_set_value(TEGRA_GPIO_PS3, 0);
	msleep(CONVERSION_TIME_MS);
	gpio_set_value(TEGRA_GPIO_PS3, 1);		
}
static int asusec_is_init_running(void){
	int ret_val;
	
	mutex_lock(&ec_chip->dock_init_lock);
	ret_val = ec_chip->dock_init;
	ec_chip->dock_init = 1;
	mutex_unlock(&ec_chip->dock_init_lock);
	return ret_val;
}

static void asusec_clear_i2c_buffer(struct i2c_client *client){
	int i;
	for ( i=0; i<8; i++){
		asusec_i2c_read_data(client);
	}
}
static int asusec_chip_init(struct i2c_client *client)
{
	int ret_val = 0;
	int i;

	if(asusec_is_init_running()){
		return 0;
	}	

	wake_lock(&ec_chip->wake_lock);
	strcpy(ec_chip->ec_model_name, " \n");
	strcpy(ec_chip->ec_version, " \n");
	disable_irq_nosync(client->irq);

	ec_chip->op_mode = 0;

	for ( i = 0; i < 10; i++){
		ret_val = asusec_i2c_test(client);
		if (ret_val < 0)
			msleep(300);
		else
			break;
	}
	if(ret_val < 0){
		goto fail_to_access_ec;
	}	

	for ( i=0; i<8; i++){
		asusec_i2c_read_data(client);
	}
	
	if (asusec_dockram_read_data(0x01) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_model_name, &ec_chip->i2c_dm_data[1]);
	ASUSEC_NOTICE("Model Name: %s\n", ec_chip->ec_model_name);
	
	if (asusec_dockram_read_data(0x02) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_version, &ec_chip->i2c_dm_data[1]);
	ASUSEC_NOTICE("EC-FW Version: %s\n", ec_chip->ec_version);
	
	if (asusec_dockram_read_data(0x03) < 0){
		goto fail_to_access_ec;
	}
	ASUSEC_INFO("EC-Coding Format: %s\n", ec_chip->i2c_dm_data);
	
	if (asusec_dockram_read_data(0x04) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->dock_pid, &ec_chip->i2c_dm_data[1]);
	ASUSEC_INFO("PID Version: %s\n", ec_chip->dock_pid);

	if(asusec_input_device_create(client)){
		goto fail_to_access_ec;
	}

	if (ASUSGetProjectID()==101){
		msleep(750);
		asusec_clear_i2c_buffer(client);
		asusec_touchpad_disable(client);
	}

	asusec_keypad_disable(client);
	
#if TOUCHPAD_ELAN
#if TOUCHPAD_MODE
	if (ASUSGetProjectID()==101){
		asusec_clear_i2c_buffer(client);
		if ((!elantech_detect(ec_chip)) && (!elantech_init(ec_chip))){
		    ec_chip->touchpad_member = ELANTOUCHPAD;
		} else {
			ec_chip->touchpad_member = -1;
		}
	}
#endif
#endif

	ASUSEC_NOTICE("touchpad and keyboard init\n");
	ec_chip->d_index = 0;

	asusec_keypad_enable(client);
	asusec_clear_i2c_buffer(client);
	
	enable_irq(client->irq);
	ec_chip->init_success = 1;

	if ((ASUSGetProjectID()==101) && ec_chip->tp_enable){
		asusec_touchpad_enable(client);
	}
	if ((ASUSGetProjectID()==102) && !gpio_get_value(TEGRA_GPIO_PS4)){
		asusec_close_keyboard();
	}

	ec_chip->status = 1;
	wake_unlock(&ec_chip->wake_lock);
	return 0;

fail_to_access_ec:
	if (asusec_dockram_read_data(0x00) < 0){
		ASUSEC_NOTICE("No EC detected\n");
		ec_chip->dock_in = 0;
	} else {
		ASUSEC_NOTICE("Need EC FW update\n");
		asusec_fw_reset();
	}
	enable_irq(client->irq);
	wake_unlock(&ec_chip->wake_lock);
	return -1;

}


static irqreturn_t asusec_interrupt_handler(int irq, void *dev_id){

	int gpio = irq_to_gpio(irq);

	if (gpio == TEGRA_GPIO_PS2){
		disable_irq_nosync(irq);
		if (ec_chip->op_mode){
			queue_delayed_work(asusec_wq, &ec_chip->asusec_fw_update_work, 0);			
		}
		else{
			if (ec_chip->suspend_state){
				ec_chip->wakeup_lcd = 1;
				ec_chip->ap_wake_wakeup = 1;
			}
			queue_delayed_work(asusec_wq, &ec_chip->asusec_work, 0);
		}
	}
	else if (gpio == TEGRA_GPIO_PX5){
		ec_chip->dock_in = 0;
		ec_chip->dock_det++;
		queue_delayed_work(asusec_wq, &ec_chip->asusec_dock_init_work, 0);
	}
	return IRQ_HANDLED;	
}

static int asusec_irq_dock_in(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = TEGRA_GPIO_PX5;
	unsigned irq = gpio_to_irq(TEGRA_GPIO_PX5);
	const char* label = "asusec_dock_in" ; 

	ASUSEC_INFO("gpio = %d, irq = %d\n", gpio, irq);
	ASUSEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	tegra_gpio_enable(gpio);
	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSEC_ERR("gpio_request failed for input %d\n", gpio);		
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSEC_ERR("gpio_direction_input failed for input %d\n", gpio);			
		goto err_gpio_direction_input_failed;
	}
	ASUSEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, asusec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
	if (rc < 0) {
		ASUSEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);	
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}	
	ASUSEC_INFO("request irq = %d, rc = %d\n", irq, rc);

	if (gpio_get_value(gpio)){
		ASUSEC_NOTICE("No dock detected\n");
		ec_chip->dock_in = 0;
	} else{
		ASUSEC_NOTICE("Dock detected\n");
		ec_chip->dock_in = 1;
	}

	return 0 ;

err_gpio_request_irq_fail :	
	gpio_free(gpio);
err_gpio_direction_input_failed:
	return rc;
}


#if ASUSEC_INTERRUPT_DRIVEN
static int asusec_irq(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = irq_to_gpio(client->irq);
	const char* label = "asusec_input" ; 

	ASUSEC_INFO("gpio = %d, irq = %d\n", gpio, client->irq);
	ASUSEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	tegra_gpio_enable(gpio);
	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSEC_ERR("gpio_request failed for input %d\n", gpio);		
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSEC_ERR("gpio_direction_input failed for input %d\n", gpio);			
		goto err_gpio_direction_input_failed;
	}
	ASUSEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(client->irq, asusec_interrupt_handler,/*IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_HIGH|*/IRQF_TRIGGER_LOW, label, client);
	if (rc < 0) {
		ASUSEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, client->irq, rc);	
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}	
	enable_irq_wake(client->irq);
	ASUSEC_INFO("request irq = %d, rc = %d\n", client->irq, rc);	

	return 0 ;

err_gpio_request_irq_fail :	
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;
}
#endif

static int asusec_irq_ec_request(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = TEGRA_GPIO_PS3;
	const char* label = "asusec_request" ; 

	ASUSEC_INFO("gpio = %d, irq = %d\n", gpio, client->irq);
	ASUSEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	tegra_gpio_enable(gpio);
	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSEC_ERR("gpio_request failed for input %d\n", gpio);		
		goto err_exit;
	}

	rc = gpio_direction_output(gpio, 1) ;
	if (rc) {
		ASUSEC_ERR("gpio_direction_output failed for input %d\n", gpio);			
		goto err_exit;
	}
	ASUSEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));
	
	return 0 ;

err_exit:
	return rc;
}


static int asusec_kp_key_mapping(int x)
{
	switch (x){
		case ASUSEC_KEYPAD_ESC:
			return KEY_BACK; 

		case ASUSEC_KEYPAD_KEY_WAVE:
			return KEY_GRAVE;

		case ASUSEC_KEYPAD_KEY_1:
			return KEY_1;

		case ASUSEC_KEYPAD_KEY_2:
			return KEY_2;

		case ASUSEC_KEYPAD_KEY_3:
			return KEY_3;

		case ASUSEC_KEYPAD_KEY_4:
			return KEY_4;

		case ASUSEC_KEYPAD_KEY_5:
			return KEY_5;

		case ASUSEC_KEYPAD_KEY_6:
			return KEY_6;

		case ASUSEC_KEYPAD_KEY_7:
			return KEY_7;

		case ASUSEC_KEYPAD_KEY_8:
			return KEY_8;

		case ASUSEC_KEYPAD_KEY_9:
			return KEY_9;

		case ASUSEC_KEYPAD_KEY_0:
			return KEY_0;

		case ASUSEC_KEYPAD_KEY_MINUS:
			return KEY_MINUS;

		case ASUSEC_KEYPAD_KEY_EQUAL:
			return KEY_EQUAL;

		case ASUSEC_KEYPAD_KEY_BACKSPACE:
			return KEY_BACKSPACE;

		case ASUSEC_KEYPAD_KEY_TAB:
			return KEY_TAB;

		case ASUSEC_KEYPAD_KEY_Q:
			return KEY_Q;

		case ASUSEC_KEYPAD_KEY_W:
			return KEY_W;

		case ASUSEC_KEYPAD_KEY_E:
			return KEY_E;

		case ASUSEC_KEYPAD_KEY_R:
			return KEY_R;

		case ASUSEC_KEYPAD_KEY_T:
			return KEY_T;

		case ASUSEC_KEYPAD_KEY_Y:
			return KEY_Y;

		case ASUSEC_KEYPAD_KEY_U:
			return KEY_U;

		case ASUSEC_KEYPAD_KEY_I:
			return KEY_I;

		case ASUSEC_KEYPAD_KEY_O:
			return KEY_O;

		case ASUSEC_KEYPAD_KEY_P:
			return KEY_P;

		case ASUSEC_KEYPAD_KEY_LEFTBRACE:
			return KEY_LEFTBRACE;

		case ASUSEC_KEYPAD_KEY_RIGHTBRACE:
			return KEY_RIGHTBRACE;

		case ASUSEC_KEYPAD_KEY_BACKSLASH:
			return KEY_BACKSLASH;

		case ASUSEC_KEYPAD_KEY_CAPSLOCK:
			return KEY_CAPSLOCK;

		case ASUSEC_KEYPAD_KEY_A:
			return KEY_A;

		case ASUSEC_KEYPAD_KEY_S:
			return KEY_S;

		case ASUSEC_KEYPAD_KEY_D:
			return KEY_D;

		case ASUSEC_KEYPAD_KEY_F:
			return KEY_F;

		case ASUSEC_KEYPAD_KEY_G:
			return KEY_G;

		case ASUSEC_KEYPAD_KEY_H:
			return KEY_H;

		case ASUSEC_KEYPAD_KEY_J:
			return KEY_J;

		case ASUSEC_KEYPAD_KEY_K:
			return KEY_K;

		case ASUSEC_KEYPAD_KEY_L:
			return KEY_L;

		case ASUSEC_KEYPAD_KEY_SEMICOLON:
			return KEY_SEMICOLON;

		case ASUSEC_KEYPAD_KEY_APOSTROPHE:
			return KEY_APOSTROPHE;

		case ASUSEC_KEYPAD_KEY_ENTER:
			return KEY_ENTER;

		case ASUSEC_KEYPAD_KEY_LEFTSHIFT:
			return KEY_LEFTSHIFT;

		case ASUSEC_KEYPAD_KEY_Z:
			return KEY_Z;

		case ASUSEC_KEYPAD_KEY_X:
			return KEY_X;

		case ASUSEC_KEYPAD_KEY_C:
			return KEY_C;

		case ASUSEC_KEYPAD_KEY_V:
			return KEY_V;

		case ASUSEC_KEYPAD_KEY_B:
			return KEY_B;

		case ASUSEC_KEYPAD_KEY_N:
			return KEY_N;

		case ASUSEC_KEYPAD_KEY_M:
			return KEY_M;

		case ASUSEC_KEYPAD_KEY_COMMA:
			return KEY_COMMA;

		case ASUSEC_KEYPAD_KEY_DOT:
			return KEY_DOT;

		case ASUSEC_KEYPAD_KEY_SLASH:
			return KEY_SLASH;

		case ASUSEC_KEYPAD_KEY_RIGHTSHIFT:
			return KEY_RIGHTSHIFT;

		case ASUSEC_KEYPAD_KEY_LEFT:
			return KEY_LEFT;

		case ASUSEC_KEYPAD_KEY_RIGHT:
			return KEY_RIGHT;

		case ASUSEC_KEYPAD_KEY_UP:
			return KEY_UP;

		case ASUSEC_KEYPAD_KEY_DOWN:
			return KEY_DOWN;

		case ASUSEC_KEYPAD_RIGHTWIN:
			return KEY_SEARCH;

		case ASUSEC_KEYPAD_LEFTCTRL:
			return KEY_LEFTCTRL;

		case ASUSEC_KEYPAD_LEFTWIN:
			return KEY_HOMEPAGE;

		case ASUSEC_KEYPAD_LEFTALT:
			return KEY_LEFTALT;

		case ASUSEC_KEYPAD_KEY_SPACE:
			return KEY_SPACE;

		case ASUSEC_KEYPAD_RIGHTALT:
			return KEY_RIGHTALT;

		case ASUSEC_KEYPAD_WINAPP:
			return KEY_MENU;

		case ASUSEC_KEYPAD_RIGHTCTRL:
			return KEY_RIGHTCTRL;

		case ASUSEC_KEYPAD_HOME:	
			return KEY_HOME;

		case ASUSEC_KEYPAD_PAGEUP:
			return KEY_PAGEUP;

		case ASUSEC_KEYPAD_PAGEDOWN:
			return KEY_PAGEDOWN;

		case ASUSEC_KEYPAD_END:
			return KEY_END;

		//--- JP keys
		case ASUSEC_YEN:
			return KEY_YEN;
			
		case ASUSEC_RO:
			return KEY_RO;
			
		case ASUSEC_MUHENKAN:
			return KEY_MUHENKAN;
			
		case ASUSEC_HENKAN:
			return KEY_HENKAN;
			
		case ASUSEC_HIRAGANA_KATAKANA:
			return KEY_KATAKANAHIRAGANA;			
			
		//--- UK keys
		case ASUSEC_EUROPE_2:
			return KEY_102ND;
			
		default:
			return -1;
	}
}

static void asusec_reset_counter(unsigned long data){
	ec_chip->d_index = 0;
}

static int asusec_tp_control(int arg){

	int ret_val = 0;	
	
	if(arg == ASUSEC_TP_ON){
		if (ec_chip->tp_enable == 0){
			ec_chip->tp_wait_ack = 1;
			ec_chip->tp_enable = 1;
			asusec_i2c_write_data(ec_chip->client, 0xF4D4);
			ec_chip->d_index = 0;
		}
		ret_val = 0;
	} else if (arg == ASUSEC_TP_OFF){
		ec_chip->tp_wait_ack = 1;
		ec_chip->tp_enable = 0;
		asusec_i2c_write_data(ec_chip->client, 0xF5D4);
		ec_chip->d_index = 0;
		ret_val = 0;
	} else
		ret_val = -ENOTTY;
	
	return ret_val;

}
#if (!TOUCHPAD_MODE)
static void asusec_tp_rel(void){

	ec_chip->touchpad_data.x_sign = (ec_chip->ec_data[0] & X_SIGN_MASK) ? 1:0;
	ec_chip->touchpad_data.y_sign = (ec_chip->ec_data[0] & Y_SIGN_MASK) ? 1:0;
	ec_chip->touchpad_data.left_btn = (ec_chip->ec_data[0] & LEFT_BTN_MASK) ? 1:0;
	ec_chip->touchpad_data.right_btn = (ec_chip->ec_data[0] & RIGHT_BTN_MASK) ? 1:0;
	ec_chip->touchpad_data.delta_x = 
		(ec_chip->touchpad_data.x_sign) ? (ec_chip->ec_data[1] - 0xff):ec_chip->ec_data[1];
	ec_chip->touchpad_data.delta_y = 
		(ec_chip->touchpad_data.y_sign) ? (ec_chip->ec_data[2] - 0xff):ec_chip->ec_data[2];

	input_report_rel(ec_chip->indev, REL_X, ec_chip->touchpad_data.delta_x);
	input_report_rel(ec_chip->indev, REL_Y, (-1) * ec_chip->touchpad_data.delta_y);
	input_report_key(ec_chip->indev, BTN_LEFT, ec_chip->touchpad_data.left_btn);
	input_report_key(ec_chip->indev, KEY_BACK, ec_chip->touchpad_data.right_btn);				
	input_sync(ec_chip->indev);
	
}
#endif

#if TOUCHPAD_MODE
static void asusec_tp_abs(void){
	unsigned char SA1,A1,B1,SB1,C1,D1;
	static unsigned char SA1_O=0,A1_O=0,B1_O=0,SB1_O=0,C1_O=0,D1_O=0;
	static int Null_data_times = 0;
	
	if ((ec_chip->tp_enable) && (ec_chip->touchpad_member == ELANTOUCHPAD)){
		SA1= ec_chip->ec_data[0];
		A1 = ec_chip->ec_data[1];
		B1 = ec_chip->ec_data[2];
		SB1= ec_chip->ec_data[3];
		C1 = ec_chip->ec_data[4];
		D1 = ec_chip->ec_data[5];
		ASUSEC_INFO("SA1=0x%x A1=0x%x B1=0x%x SB1=0x%x C1=0x%x D1=0x%x \n",SA1,A1,B1,SB1,C1,D1);
		if ( (SA1 == 0xC4) && (A1 == 0xFF) && (B1 == 0xFF) && 
		     (SB1 == 0x02) && (C1 == 0xFF) && (D1 == 0xFF)){
			Null_data_times ++;
			goto asusec_tp_abs_end;
		}

		if(!(SA1 == SA1_O && A1 == A1_O && B1 == B1_O && 
		   SB1 == SB1_O && C1 == C1_O && D1 == D1_O)) {
			elantech_report_absolute_to_related(ec_chip, &Null_data_times);
		}
		
asusec_tp_abs_end:
		SA1_O = SA1;
		A1_O = A1;
		B1_O = B1;
		SB1_O = SB1;
		C1_O = C1;
		D1_O = D1;
	}

}
#endif

static void asusec_touchpad_processing(void){
	int i;
	int length = 0;
	int tp_start = 0;
	ASUSEC_I2C_DATA(ec_chip->i2c_data,ec_chip->index);

#if TOUCHPAD_MODE
	length = ec_chip->i2c_data[0];
	if (ec_chip->tp_wait_ack){
		ec_chip->tp_wait_ack = 0;
		tp_start = 1;
		ec_chip->d_index = 0;
	} else {
		tp_start = 0;
	}
		
	for( i = tp_start; i < length - 1 ; i++){
		ec_chip->ec_data[ec_chip->d_index] = ec_chip->i2c_data[i+2];
		ec_chip->d_index++;
		if (ec_chip->d_index == 6){			
			asusec_tp_abs();
			ec_chip->d_index = 0;
		}
	}
	
	
	if (ec_chip->d_index)
		mod_timer(&ec_chip->asusec_timer,jiffies+(HZ * 1/20));
#else
	length = ec_chip->i2c_data[0];
	for( i = 0; i < length -1 ; i++){
		ec_chip->ec_data[ec_chip->d_index] = ec_chip->i2c_data[i+2];
		ec_chip->d_index++;
		if (ec_chip->d_index == 3){
			asusec_tp_rel();
			ec_chip->d_index = 0;
		}
	}	
#endif
}

static void asusec_kp_wake(void){
	ASUSEC_NOTICE("ASUSEC WAKE\n");
	if (asusec_input_device_create(ec_chip->client)){
		return ;
	}
	input_report_key(ec_chip->indev, KEY_MENU, 1);
	input_sync(ec_chip->indev);
	input_report_key(ec_chip->indev, KEY_MENU, 0);
	input_sync(ec_chip->indev);
}

static void asusec_kp_smi(void){
	if (ec_chip->i2c_data[2] == ASUSEC_SMI_HANDSHAKING){
		ASUSEC_NOTICE("ASUSEC_SMI_HANDSHAKING\n");
		asusec_chip_init(ec_chip->client);
	} else if (ec_chip->i2c_data[2] == ASUSEC_SMI_RESET){
		ASUSEC_NOTICE("ASUSEC_SMI_RESET\n");
		ec_chip->init_success = 0;
		asusec_dock_init_work_function(NULL);
	} else if (ec_chip->i2c_data[2] == ASUSEC_SMI_WAKE){
		asusec_kp_wake();
		ASUSEC_NOTICE("ASUSEC_SMI_WAKE\n");
	}
}

static void asusec_kp_kbc(void){
	if (ec_chip->i2c_data[2] == ASUSEC_PS2_ACK){
		if (ec_chip->kbc_value == 0){
			ASUSEC_INFO("send led cmd 2\n");
			asusec_i2c_write_data(ec_chip->client, 0x0000);
		} else {
			ASUSEC_INFO("send led cmd 2\n");
			asusec_i2c_write_data(ec_chip->client, 0x0400);
		}
	}
}
static void asusec_kp_sci(void){
	int ec_signal = ec_chip->i2c_data[2];
	
	ec_chip->keypad_data.input_keycode = asusec_kp_sci_table[ec_signal];
	if(ec_chip->keypad_data.input_keycode > 0){
		ASUSEC_INFO("input_keycode = 0x%x\n", ec_chip->keypad_data.input_keycode);
		
		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 1);
		input_sync(ec_chip->indev); 
		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 0);
		input_sync(ec_chip->indev); 
		
	}else{				
		ASUSEC_INFO("Unknown ec_signal = 0x%x\n", ec_signal);
	}
}
static void asusec_kp_key(void){
	int scancode = 0;
	
	if (ec_chip->i2c_data[2] == ASUSEC_KEYPAD_KEY_EXTEND){		// data is an extended data
		ec_chip->keypad_data.extend = 1;
		ec_chip->bc = 3;
	}else{
		ec_chip->keypad_data.extend = 0;
		ec_chip->bc = 2;
	}
	if(ec_chip->i2c_data[ec_chip->bc] == ASUSEC_KEYPAD_KEY_BREAK){ // the data is a break signal
		ec_chip->keypad_data.value = 0;	
		ec_chip->bc++;
	}else{
		ec_chip->keypad_data.value = 1;
	}
	
	if (ec_chip->keypad_data.extend == 1){
		scancode = ((ASUSEC_KEYPAD_KEY_EXTEND << 8) | ec_chip->i2c_data[ec_chip->bc]);
	} else {
		scancode = ec_chip->i2c_data[ec_chip->bc];
	}
	if (ec_chip->i2c_data[0] == 6){								// left shift DOWN + note2 keys
		if ((ec_chip->i2c_data[2] == 0xE0) &&
			(ec_chip->i2c_data[3] == 0xF0) &&
			(ec_chip->i2c_data[4] == 0x12)){
			scancode = ec_chip->i2c_data[5] << 8 | ec_chip->i2c_data[6];
			ec_chip->keypad_data.value = 1;
		}
		else if ((ec_chip->i2c_data[2] == 0xE0) &&				// right shift DOWN + note2 keys
			(ec_chip->i2c_data[3] == 0xF0) &&
			(ec_chip->i2c_data[4] == 0x59)){
			scancode = ec_chip->i2c_data[5] << 8 | ec_chip->i2c_data[6];
			ec_chip->keypad_data.value = 1;
		}
	}
	ASUSEC_INFO("scancode = 0x%x\n", scancode);
	ec_chip->keypad_data.input_keycode = asusec_kp_key_mapping(scancode);
	if(ec_chip->keypad_data.input_keycode > 0){
		ASUSEC_INFO("input_keycode = 0x%x, input_value = %d\n", 
				ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
		
		input_report_key(ec_chip->indev, 
			ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
		input_sync(ec_chip->indev); 
		
	}else{				
		ASUSEC_INFO("Unknown scancode = 0x%x\n", scancode);
	}

}

static void asusec_keypad_processing(void){
		
	ASUSEC_I2C_DATA(ec_chip->i2c_data,ec_chip->index);	
	if (ec_chip->i2c_data[1] & ASUSEC_KBC_MASK)
		asusec_kp_kbc();
	else if (ec_chip->i2c_data[1] & ASUSEC_SCI_MASK)	// ec data is a signal
		asusec_kp_sci();
	else												// ec data is a scan code
		asusec_kp_key();
}

static void asusec_stresstest_work_function(struct work_struct *dat)
{
	asusec_i2c_read_data(ec_chip->client);
	if (ec_chip->i2c_data[1] & ASUSEC_OBF_MASK){		// ec data is valid
		if (ec_chip->i2c_data[1] & ASUSEC_AUX_MASK){	// ec data is from touchpad
			asusec_touchpad_processing();
		}else{		// ec data is from keyboard
			asusec_keypad_processing();
		}
	}

	queue_delayed_work(asusec_wq, &asusec_stress_work, HZ/ec_chip->polling_rate);
}
#if CALLBACK_READY
extern void docking_callback(void );
#endif
static void asusec_dock_init_work_function(struct work_struct *dat)
{
	int gpio = TEGRA_GPIO_PX5;
	int irq = gpio_to_irq(gpio);
	int i = 0;
	int d_counter = 0;
	int gpio_state = 0;
	ASUSEC_INFO("Dock-init function\n");

	wake_lock(&ec_chip->wake_lock_init);
	if (ASUSGetProjectID()==101){
		ASUSEC_NOTICE("EP101 dock-init\n");
		if (ec_chip->dock_det){
			gpio_state = gpio_get_value(gpio);
			for(i = 0; i < 40; i++){
				msleep(50);
				if (gpio_state == gpio_get_value(gpio)){
					d_counter++;
				} else {
					gpio_state = gpio_get_value(gpio);
					d_counter = 0;
				}
				if (d_counter > 4){
					break;
				}
			}
#if CALLBACK_READY
			docking_callback();
#endif
			ec_chip->dock_det--;
			ec_chip->re_init = 0;
		}
		
		mutex_lock(&ec_chip->input_lock);
		if (gpio_get_value(gpio)){
			ASUSEC_NOTICE("No dock detected\n");
			ec_chip->dock_in = 0;
			ec_chip->init_success = 0;
			ec_chip->tp_enable = 1;
			if (ec_chip->indev){
				input_unregister_device(ec_chip->indev);
				ec_chip->indev = NULL;
			}
			if (ec_chip->private->abs_dev){
				input_unregister_device(ec_chip->private->abs_dev);
				ec_chip->private->abs_dev = NULL;
			}
		} else{
			ASUSEC_NOTICE("Dock detected\n");
			ec_chip->dock_in = 1;
			if (gpio_get_value(TEGRA_GPIO_PS4) || (!ec_chip->status)){
				if (ec_chip->init_success == 0){
					msleep(400);
					asusec_reset_dock();
					msleep(200);
					asusec_chip_init(ec_chip->client);
				}
			} else {
				ASUSEC_NOTICE("Keyboard is closed\n");
			}
		}
		switch_set_state(&ec_chip->dock_sdev, ec_chip->dock_in ? 10 : 0);
		mutex_unlock(&ec_chip->input_lock);
	}
	else if (ASUSGetProjectID()==102){
		ASUSEC_NOTICE("EP102 dock-init\n");
		ec_chip->dock_in = 1;
		if (gpio_get_value(TEGRA_GPIO_PS4) || (!ec_chip->status)){
			if(ec_chip->init_success == 0){
				msleep(400);
				asusec_reset_dock();
				msleep(200);
				asusec_chip_init(ec_chip->client);
			}
		} else {
			ASUSEC_NOTICE("Keyboard is closed\n");
		}
	}
	wake_unlock(&ec_chip->wake_lock_init);
}

static void asusec_fw_update_work_function(struct work_struct *dat)
{
	int smbus_data;
	int gpio = TEGRA_GPIO_PS2;
	int irq = gpio_to_irq(gpio);
	
	mutex_lock(&ec_chip->lock);
	smbus_data = i2c_smbus_read_byte_data(&dockram_client, 0);
	enable_irq(irq);
	//ASUSEC_INFO("read byte = 0x%x\n", smbus_data);
	BuffPush(smbus_data);		
	//ASUSEC_INFO("gpio TEGRA_GPIO_PS2 value = %d\n", gpio_get_value(TEGRA_GPIO_PS2));
	mutex_unlock(&ec_chip->lock);
}

static void asusec_work_function(struct work_struct *dat)
{
	int gpio = TEGRA_GPIO_PS2;
	int irq = gpio_to_irq(gpio);
	int ret_val = 0;

	if (ec_chip->wakeup_lcd){
		if (gpio_get_value(TEGRA_GPIO_PS4)){
			ec_chip->wakeup_lcd = 0;
			if (ASUSGetProjectID()==101){
				ec_chip->dock_in = gpio_get_value(TEGRA_GPIO_PX5) ? 0 : 1;
			} else if (ASUSGetProjectID()==102){
				ec_chip->dock_in = 1;
			}
			wake_lock_timeout(&ec_chip->wake_lock, 3*HZ);
			msleep(500);
		}
	}

	ret_val = asusec_i2c_read_data(ec_chip->client);
	enable_irq(irq);

	if (ret_val < 0){
		return ;
	}

	if (ec_chip->i2c_data[1] & ASUSEC_OBF_MASK){		// ec data is valid
		if (ec_chip->i2c_data[1] & ASUSEC_SMI_MASK){	// ec data is from touchpad
			asusec_kp_smi();
			return ;
		}
	}	

	mutex_lock(&ec_chip->input_lock);
	if (ec_chip->indev == NULL || ec_chip->init_success == 0){
		mutex_unlock(&ec_chip->input_lock);
		return;
	}
	if (ec_chip->i2c_data[1] & ASUSEC_OBF_MASK){		// ec data is valid
		if (ec_chip->i2c_data[1] & ASUSEC_AUX_MASK){	// ec data is from touchpad
			if (ec_chip->private->abs_dev)
				asusec_touchpad_processing();
		}else{		// ec data is from keyboard
			asusec_keypad_processing();
		}
	}
	mutex_unlock(&ec_chip->input_lock);
#if (!ASUSEC_INTERRUPT_DRIVEN)
	queue_delayed_work(asusec_wq, &ec_chip->asusec_work, HZ/ASUSEC_POLLING_RATE);
#endif
}

static void asusec_keypad_set_input_params(struct input_dev *dev)
{
	int i = 0;
	set_bit(EV_KEY, dev->evbit);	
	for ( i = 0; i < 246; i++)
		set_bit(i,dev->keybit);

	input_set_capability(dev, EV_LED, LED_CAPSL);
}

static void asusec_touchpad_set_input_params(struct input_dev *dev)
{
	set_bit(EV_KEY, dev->evbit);
	set_bit(BTN_LEFT, dev->keybit);
	set_bit(BTN_RIGHT, dev->keybit);

	set_bit(EV_REL, dev->evbit);
	set_bit(REL_X, dev->relbit);
	set_bit(REL_Y, dev->relbit);

	set_bit(EV_SYN, dev->evbit);
	set_bit(EV_ABS, dev->evbit);
	set_bit(BTN_TOUCH, dev->keybit);
	set_bit(BTN_2, dev->keybit);
	set_bit(ABS_TOOL_WIDTH, dev->absbit);

	input_set_abs_params(dev, ABS_X, XMIN_NOMINAL, XMAX_NOMINAL, 0, 0);
	input_set_abs_params(dev, ABS_Y, YMIN_NOMINAL, YMAX_NOMINAL, 0, 0);
	input_set_abs_params(dev, ABS_HAT0X, XMIN_NOMINAL, XMAX_NOMINAL, 0, 0);
	input_set_abs_params(dev, ABS_HAT0Y, YMIN_NOMINAL, YMAX_NOMINAL, 0, 0);
	input_set_abs_params(dev, ABS_HAT1X, XMIN_NOMINAL, XMAX_NOMINAL, 0, 0);
	input_set_abs_params(dev, ABS_HAT1Y, YMIN_NOMINAL, YMAX_NOMINAL, 0, 0);

	input_set_abs_params(dev, ABS_MT_POSITION_X, XMIN_NOMINAL, XMAX_NOMINAL, 0, 0);	
	input_set_abs_params(dev, ABS_MT_POSITION_Y, YMIN_NOMINAL, YMAX_NOMINAL, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	input_set_abs_params(dev, ABS_PRESSURE, 0, 255, 0, 0);
}

static int asusec_input_device_create(struct i2c_client *client){
	int err = 0;
	
	if (ec_chip->indev){
		return 0;
	}
	ec_chip->indev = input_allocate_device();
	if (!ec_chip->indev) {
		ASUSEC_ERR("input_dev allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->indev->name = "asusec";
	ec_chip->indev->phys = "/dev/input/asusec";
	ec_chip->indev->dev.parent = &client->dev;
	ec_chip->indev->event = asusec_event;
	
	asusec_keypad_set_input_params(ec_chip->indev);
	//asusec_touchpad_set_input_params(ec_chip->indev);
	err = input_register_device(ec_chip->indev);
	if (err) {
		ASUSEC_ERR("input registration fails\n");
		goto exit_input_free;
	}
	return 0;

exit_input_free:
	input_free_device(ec_chip->indev);
	ec_chip->indev = NULL;
exit:
	return err;

}

static int __devinit asusec_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;

	ASUSEC_INFO("asusec probe\n");
	err = sysfs_create_group(&client->dev.kobj, &asusec_smbus_group);
	if (err) {
		ASUSEC_ERR("Unable to create the sysfs\n");
		goto exit;
	}

	ec_chip = kzalloc(sizeof (struct asusec_chip), GFP_KERNEL);
	if (!ec_chip) {
		ASUSEC_ERR("Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}
	ec_chip->private = kzalloc(sizeof(struct elantech_data), GFP_KERNEL);
	if (!ec_chip->private) {
		ASUSEC_ERR("Memory allocation (elantech_data) fails\n");
		err = -ENOMEM;
		goto exit;
	}
		
	i2c_set_clientdata(client, ec_chip);
	ec_chip->client = client;
	ec_chip->client->driver = &asusec_driver;				
	ec_chip->client->flags = 1;

	mutex_init(&ec_chip->lock);
	mutex_init(&ec_chip->kbc_lock);
	mutex_init(&ec_chip->input_lock);
	mutex_init(&ec_chip->dock_init_lock);

	init_timer(&ec_chip->asusec_timer);
	ec_chip->asusec_timer.function = asusec_reset_counter;

	wake_lock_init(&ec_chip->wake_lock, WAKE_LOCK_SUSPEND, "asusec_wake");
	wake_lock_init(&ec_chip->wake_lock_init, WAKE_LOCK_SUSPEND, "asusec_wake_init");

	ec_chip->status = 0;
	ec_chip->dock_det = 0;
	ec_chip->dock_in = 0;
	ec_chip->dock_init = 0;
	ec_chip->d_index = 0;
	ec_chip->suspend_state = 0;
	ec_chip->init_success = 0;
	ec_chip->wakeup_lcd = 0;
	ec_chip->tp_wait_ack = 0;
	ec_chip->tp_enable = 1;
	ec_chip->re_init = 0;
	ec_chip->ec_wakeup = 0;
	ec_chip->indev = NULL;
	ec_chip->private->abs_dev = NULL;
	asusec_dockram_init(client);
	
	cdev_add(asusec_cdev,asusec_dev,1) ;

	ec_chip->dock_sdev.name = DOCK_SDEV_NAME;
	ec_chip->dock_sdev.print_name = asusec_switch_name;
	ec_chip->dock_sdev.print_state = asusec_switch_state;
	if(switch_dev_register(&ec_chip->dock_sdev) < 0){
		ASUSEC_ERR("switch_dev_register for dock failed!\n");
		goto exit;
	}
	switch_set_state(&ec_chip->dock_sdev, 0);

	err = power_supply_register(&client->dev, &asusec_power_supply[0]);
	if (err){
		ASUSEC_ERR("fail to register power supply for dock\n");
		goto exit;
	}
	
	asusec_wq = create_singlethread_workqueue("asusec_wq");
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusec_work, asusec_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusec_dock_init_work, asusec_dock_init_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusec_fw_update_work, asusec_fw_update_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusec_led_on_work, asusec_keypad_led_on);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusec_led_off_work, asusec_keypad_led_off);
	INIT_DELAYED_WORK_DEFERRABLE(&asusec_stress_work, asusec_stresstest_work_function);
	
	asusec_irq_dock_in(client);
	asusec_irq_ec_request(client);
#if ASUSEC_INTERRUPT_DRIVEN
	asusec_irq(client);
#endif	

	queue_delayed_work(asusec_wq, &ec_chip->asusec_dock_init_work, 0);

#if (!ASUSEC_INTERRUPT_DRIVEN)
	if (ec_chip->status){
		queue_delayed_work(asusec_wq, &(ec_chip->asusec_work), HZ/ASUSEC_POLLING_RATE);
	}
#endif	

	return 0;

exit:
	return err;
}

static int __devexit asusec_remove(struct i2c_client *client)
{
	struct asusec_chip *chip = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s()\n", __func__);
	input_unregister_device(chip->indev);
	kfree(chip);
	return 0;
}

static ssize_t asusec_info_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "Model Name: %s, EC-FW Version: %s\n", 
			ec_chip->ec_model_name, ec_chip->ec_version);
}

static ssize_t asusec_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", ec_chip->status);
}

static ssize_t asusec_show_dock(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "dock detect = %d\n", ec_chip->dock_in);
}

static ssize_t asusec_store_led(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		if (buf[0] == '0')
			queue_delayed_work(asusec_wq, &ec_chip->asusec_led_off_work, 0);
		else 
			queue_delayed_work(asusec_wq, &ec_chip->asusec_led_on_work, 0);
	}
	
	return 0 ;
}

static ssize_t asusec_store_ec_wakeup(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	if (buf[0] == '0'){
		ec_chip->ec_wakeup = 0;
		ASUSEC_NOTICE("Set EC shutdown when PAD in LP0\n");
	}
	else{
		ec_chip->ec_wakeup = 1;
		ASUSEC_NOTICE("Keep EC active when PAD in LP0\n");
	}
		
	return 0 ;
}

static ssize_t asusec_show_drain(struct device *class,struct device_attribute *attr,char *buf)
{
	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		asusec_dockram_read_data(0x0A);

		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x8;
		asusec_dockram_write_data(0x0A,9);
		ASUSEC_NOTICE("discharging 15 seconds\n");
		return sprintf(buf, "discharging 15 seconds\n");
	}	

	return 0;
}

static ssize_t asusec_show_dock_battery(struct device *class,struct device_attribute *attr,char *buf)
{
	int bat_percentage = 0;
	int ret_val = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		ret_val = asusec_dockram_read_data(0x14);

		if (ret_val < 0)
			return sprintf(buf, "-1\n");
		else{
			bat_percentage = (ec_chip->i2c_dm_data[14] << 8 )| ec_chip->i2c_dm_data[13];
			return sprintf(buf, "%d\n", bat_percentage);
		}
	}

	return sprintf(buf, "-1\n");
}

static ssize_t asusec_show_dock_battery_all(struct device *class,struct device_attribute *attr,char *buf)
{
	int i = 0;
	char temp_buf[64];
	int ret_val = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		ret_val = asusec_dockram_read_data(0x14);

		if (ret_val < 0)
			return sprintf(buf, "fail to get dock-battery info\n");
		else{
			sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
			strcpy(buf, temp_buf);
			for (i = 1; i < 17; i++){
				sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
				strcat(buf, temp_buf);
			}
			return strlen(buf);
		}
	}

	return sprintf(buf, "fail to get dock-battery info\n");
}

static ssize_t asusec_show_dock_control_flag(struct device *class,struct device_attribute *attr,char *buf)
{
	int i = 0;
	char temp_buf[64];
	int ret_val = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		ret_val = asusec_dockram_read_data(0x0A);

		if (ret_val < 0)
			return sprintf(buf, "fail to get control-flag info\n");
		else{
			sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
			strcpy(buf, temp_buf);
			for (i = 1; i < 9; i++){
				sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
				strcat(buf, temp_buf);
			}
			return strlen(buf);
		}
	}

	return sprintf(buf, "fail to get control-flag info\n");
}


static void asusec_dock_info_update(void){
	int ret_val = 0;
	char *envp[3];
	char name_buf[64];
	char state_buf[64];
	int env_offset = 0;
	int length = 0;

	if (ec_chip->ap_wake_wakeup && (gpio_get_value(TEGRA_GPIO_PX5) == 0)){
		ec_chip->ap_wake_wakeup = 0;
		ret_val = asusec_i2c_test(ec_chip->client);
		if((ret_val >= 0) && (asusec_dockram_read_data(0x04) >= 0)){
			strcpy(ec_chip->dock_pid, &ec_chip->i2c_dm_data[1]);
			ASUSEC_NOTICE("PID Version: %s\n", ec_chip->dock_pid);
		}
		if ((ret_val >= 0) && (asusec_dockram_read_data(0x02) >= 0)){
			if (ec_chip->dock_in &&
				strncmp(ec_chip->ec_version, &ec_chip->i2c_dm_data[1], 11)){

				strcpy(ec_chip->ec_version, &ec_chip->i2c_dm_data[1]);
				ASUSEC_NOTICE("EC-FW Version: %s\n", ec_chip->ec_version);

				length = strlen(ec_chip->ec_version);
				ec_chip->ec_version[length] = NULL;
				snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s", ec_chip->ec_version);
				envp[env_offset++] = name_buf;

				snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%s", "10");
				envp[env_offset++] = state_buf;

				envp[env_offset] = NULL;
				kobject_uevent_env(&ec_chip->dock_sdev.dev->kobj, KOBJ_CHANGE, envp);
			}
		}
	}
}

static void asusec_dock_status_check(void){
	if ((ec_chip->ec_version[6] <= '0') &&
	    (ec_chip->ec_version[7] <= '2') &&
	    (ec_chip->ec_version[8] <= '0') &&
	    (ec_chip->ec_version[9] <= '9') &&
	    (ec_chip->ec_version[10] == NULL)){
		ec_chip->init_success = 0;
		wake_lock(&ec_chip->wake_lock_init);
		queue_delayed_work(asusec_wq, &ec_chip->asusec_dock_init_work, 0);
	} else if (gpio_get_value(TEGRA_GPIO_PX5) && ec_chip->indev){
		ec_chip->init_success = 0;
		wake_lock(&ec_chip->wake_lock_init);
		queue_delayed_work(asusec_wq, &ec_chip->asusec_dock_init_work, 0);
	} else if (strcmp(ec_chip->dock_pid, "PCBA-EP101") == 0){
		ec_chip->init_success = 0;
		wake_lock(&ec_chip->wake_lock_init);
		queue_delayed_work(asusec_wq, &ec_chip->asusec_dock_init_work, 0);
	}

}

static int asusec_suspend(struct i2c_client *client, pm_message_t mesg){
	printk("asusec_suspend+\n");
	printk("asusec_suspend-\n");
	return 0;
}

static int asusec_resume(struct i2c_client *client){

	printk("asusec_resume+\n");

	ec_chip->suspend_state = 0;
	if (ASUSGetProjectID()==101){
		asusec_dock_info_update();
		asusec_dock_status_check();
	} else if (ASUSGetProjectID()==102){
		ec_chip->init_success = 0;
		wake_lock(&ec_chip->wake_lock_init);
		queue_delayed_work(asusec_wq, &ec_chip->asusec_dock_init_work, 0);
	}

	printk("asusec_resume-\n");
	return 0;	
}

static int asusec_set_wakeup_cmd(void){
	int ret_val = 0;

	if (ec_chip->dock_in){
		ret_val = asusec_i2c_test(ec_chip->client);
		if(ret_val >= 0){
			asusec_dockram_read_data(0x0A);
			ec_chip->i2c_dm_data[0] = 8;
			if (ec_chip->ec_wakeup){
				ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x80;
			} else {
				ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0x7F;
			}
			asusec_dockram_write_data(0x0A,9);
		}
	}
	return 0;
}
static ssize_t asusec_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asusec_switch_state(struct switch_dev *sdev, char *buf)
{
	if (ASUSGetProjectID() == 101) {
		return sprintf(buf, "%s\n", (ec_chip->dock_in && ec_chip->init_success ? "10" : "0"));
	} else {
		return sprintf(buf, "%s\n", "0");
	}
}

static int asusec_open(struct inode *inode, struct file *flip){
	ASUSEC_NOTICE(" ");
	return 0;
}
static int asusec_release(struct inode *inode, struct file *flip){
	ASUSEC_NOTICE(" ");
	return 0;
}
static long asusec_ioctl(struct file *flip,
					unsigned int cmd, unsigned long arg){
	int err = 1;
	char *envp[3];
	char name_buf[64];
	int env_offset = 0;
	int length = 0;

	if (_IOC_TYPE(cmd) != ASUSEC_IOC_MAGIC)
	 return -ENOTTY;
	if (_IOC_NR(cmd) > ASUSEC_IOC_MAXNR)
	return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	 switch (cmd) {
        case ASUSEC_POLLING_DATA:
			if (arg == ASUSEC_IOCTL_HEAVY){
				ASUSEC_NOTICE("heavy polling\n");
				ec_chip->polling_rate = 80;
				queue_delayed_work(asusec_wq, &asusec_stress_work, HZ/ec_chip->polling_rate);
			}
			else if (arg == ASUSEC_IOCTL_NORMAL){
				ASUSEC_NOTICE("normal polling\n");
				ec_chip->polling_rate = 10;
				queue_delayed_work(asusec_wq, &asusec_stress_work, HZ/ec_chip->polling_rate);
			}
			else if  (arg == ASUSEC_IOCTL_END){
				ASUSEC_NOTICE("polling end\n");
		    	cancel_delayed_work_sync(&asusec_stress_work) ;
			}
			else
				return -ENOTTY;
			break;
		case ASUSEC_FW_UPDATE:
			if (ec_chip->dock_in){
				ASUSEC_NOTICE("ASUSEC_FW_UPDATE\n");
				buff_in_ptr = 0;
				buff_out_ptr = 0;
				h2ec_count = 0;
				ec_chip->suspend_state = 0;
				ec_chip->status = 0;
				asusec_reset_dock();
				wake_lock_timeout(&ec_chip->wake_lock, 3*60*HZ);
				msleep(3000);
				ec_chip->op_mode = 1;
				ec_chip->i2c_dm_data[0] = 0x02;
				ec_chip->i2c_dm_data[1] = 0x55;
				ec_chip->i2c_dm_data[2] = 0xAA;
				asusec_dockram_write_data(0x40,3);
				ec_chip->init_success = 0;
				msleep(1000);
			} else {
				ASUSEC_NOTICE("No dock detected\n");
				return -1;
			}
			break;
		case ASUSEC_INIT:
			msleep(500);
			ec_chip->status = 0;
			queue_delayed_work(asusec_wq, &ec_chip->asusec_dock_init_work, 0);
			msleep(2500);
			ASUSEC_NOTICE("ASUSEC_INIT - EC version: %s\n", ec_chip->ec_version);
			length = strlen(ec_chip->ec_version);
			ec_chip->ec_version[length] = NULL;
			snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s", ec_chip->ec_version);
			envp[env_offset++] = name_buf;
			envp[env_offset] = NULL;
			kobject_uevent_env(&ec_chip->dock_sdev.dev->kobj, KOBJ_CHANGE, envp);
			break;
		case ASUSEC_TP_CONTROL:
			ASUSEC_NOTICE("ASUSEC_TP_CONTROL\n");
			if ((ec_chip->op_mode == 0) && ec_chip->dock_in && ec_chip->init_success){				
				err = asusec_tp_control(arg);
				return err;
			}
			else
				return -ENOTTY;
		case ASUSEC_EC_WAKEUP:
			ASUSEC_NOTICE("ASUSEC_EC_WAKEUP, arg = %d\n", arg);
			if (arg == ASUSEC_EC_OFF){
				ec_chip->ec_wakeup = 0;
				ASUSEC_NOTICE("Set EC shutdown when PAD in LP0\n");
				return asusec_set_wakeup_cmd();
			}
			else if (arg == ASUSEC_EC_ON){
				ec_chip->ec_wakeup = 1;
				ASUSEC_NOTICE("Keep EC active when PAD in LP0\n");
				return asusec_set_wakeup_cmd();
			}
			else {
				ASUSEC_ERR("Unknown argument");
				return -ENOTTY;
			}
		case ASUSEC_FW_DUMMY:
			ASUSEC_NOTICE("ASUSEC_FW_DUMMY\n");
			ec_chip->i2c_dm_data[0] = 0x02;
			ec_chip->i2c_dm_data[1] = 0x55;
			ec_chip->i2c_dm_data[2] = 0xAA;
			asusec_dockram_write_data(0x40,3);
			return 0;
        default: /* redundant, as cmd was checked against MAXNR */
            return -ENOTTY;
	}
    return 0;
}

static int BuffDataSize(void)
{   
    int in = buff_in_ptr;
    int out = buff_out_ptr;

    if (in >= out)
    {
        return (in - out);
    }
    else
    {
        return ((EC_BUFF_LEN - out) + in);
    }
}
static void BuffPush(char data)
{

    if (BuffDataSize() >= (EC_BUFF_LEN -1)) 
    {
        ASUSEC_ERR("Error: EC work-buf overflow \n");
        return;
    }

	//ASUSEC_INFO("ec_to_host_buffer[%d] = 0x%x\n", buff_in_ptr, data);
    ec_to_host_buffer[buff_in_ptr] = data;
    buff_in_ptr++;
    if (buff_in_ptr >= EC_BUFF_LEN) 
    {
        buff_in_ptr = 0;
    }    
}

static char BuffGet(void)
{
    char c = (char)0;

    if (BuffDataSize() != 0) 
    {
        c = (char) ec_to_host_buffer[buff_out_ptr];        
        buff_out_ptr++;
         if (buff_out_ptr >= EC_BUFF_LEN) 
         {
             buff_out_ptr = 0;
         }
    }
    return c;
}

static ssize_t ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int i = 0;
    int ret;
    char tmp_buf[EC_BUFF_LEN];
	static int f_counter = 0;
	static int total_buf = 0;

	mutex_lock(&ec_chip->lock);
	mutex_unlock(&ec_chip->lock);
	
    //ASUSEC_INFO("ec_read (%d)\n",count);
    //ASUSEC_INFO("buff_in_ptr = %d, buff_out_ptr = %d\n",buff_in_ptr, buff_out_ptr);
    
    while ((BuffDataSize() > 0) && count)
    {
        tmp_buf[i] = BuffGet();
        //ASUSEC_INFO("tmp_buf[%d] = 0x%x, total_buf = %d\n", i, tmp_buf[i], total_buf);
        count--;
        i++;
		f_counter = 0;
		total_buf++;
    }	

    ret = copy_to_user(buf, tmp_buf, i);
    if (ret == 0)
    {
        ret = i; // No error. Return the number of byte read.
    }


    return ret;
}

static ssize_t ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int err;    
    int i;

    //ASUSEC_INFO("ec_write (%d)\n",count);

    if (h2ec_count > 0) 
    {                   /* There is still data in the buffer that */
        return -EBUSY;  /* was not sent to the EC */
    }
    if (count > EC_BUFF_LEN) 
    {
        return -EINVAL; /* data size is too big */
    }
    
    err = copy_from_user(host_to_ec_buffer, buf, count);
    if (err)
    {
        ASUSEC_ERR("ec_write copy error\n");
        return err;
    }

    h2ec_count = count;
    //ASUSEC_INFO("ec_write (%d)\n",count);
    for (i = 0; i < count ; i++) 
    {
		//ASUSEC_INFO("smbus write, i = %d, data = 0x%x\n", i, host_to_ec_buffer[i]);
		i2c_smbus_write_byte_data(&dockram_client, host_to_ec_buffer[i],0);
    }
    h2ec_count = 0;
    //ASUSEC_INFO("ec_write done\n");
    return count;

}

static int asusec_event(struct input_dev *dev, unsigned int type, unsigned int code, int value){
	ASUSEC_INFO("type = 0x%x, code = 0x%x, value = 0x%x\n", type, code, value);
	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		if ((type == EV_LED) && (code == LED_CAPSL)){
			if(value == 0){
				queue_delayed_work(asusec_wq, &ec_chip->asusec_led_off_work, 0);
				return 0;
			} else {
				queue_delayed_work(asusec_wq, &ec_chip->asusec_led_on_work, 0);
				return 0;
			}
		}
	}
	return -ENOTTY;		
}

static int asusec_dock_battery_get_capacity(union power_supply_propval *val){
	int bat_percentage = 0;
	int ret_val = 0;

	val->intval = -1;
	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		ret_val = asusec_dockram_read_data(0x14);

		if (ret_val < 0){
			return -1;
		}
		else {
			bat_percentage = (ec_chip->i2c_dm_data[14] << 8 )| ec_chip->i2c_dm_data[13];
			val->intval = bat_percentage;
			return 0;
		}
	}
	return -1;
}

static int asusec_dock_battery_get_status(union power_supply_propval *val){
	int bat_percentage = 0;
	int ret_val = 0;

	val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		ret_val = asusec_dockram_read_data(0x0A);

		if (ret_val < 0){
			return -1;
		}
		else {
			if (ec_chip->i2c_dm_data[1] & 0x4)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			return 0;
		}
	}
	return -1;
}

static int asusec_dock_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_CAPACITY:
			if(asusec_dock_battery_get_capacity(val) < 0)
				goto error;
			break;
		case POWER_SUPPLY_PROP_STATUS:
			if(asusec_dock_battery_get_status(val) < 0)
				goto error;
			break;
		default:
			return -EINVAL;
	}
	return 0;

error:
	return -EINVAL;
}

int asusec_dock_resume(void){
	ASUSEC_NOTICE("keyboard opened, op_mode = %d\n", ec_chip->op_mode);
	if (ec_chip->op_mode == 0){
		ASUSEC_NOTICE("keyboard opened\n");
		if (ec_chip->suspend_state == 0){
			wake_lock(&ec_chip->wake_lock_init);
			queue_delayed_work(asusec_wq, &ec_chip->asusec_dock_init_work, 0);
		}
	}
	return 0;

}
EXPORT_SYMBOL(asusec_dock_resume);

int asusec_open_keyboard(void){
	ASUSEC_NOTICE("keyboard opened, op_mode = %d\n", ec_chip->op_mode);
	if ((ec_chip->suspend_state == 0) && (ec_chip->op_mode == 0)){
		ASUSEC_NOTICE("keyboard opened\n");
		ec_chip->init_success = 0;
		wake_lock(&ec_chip->wake_lock_init);
		queue_delayed_work(asusec_wq, &ec_chip->asusec_dock_init_work, 0);
	}
	return 0;
	
}
EXPORT_SYMBOL(asusec_open_keyboard);


int asusec_close_keyboard(void){
	int ret_val;

	if (ec_chip->status == 0){
		return 0;
	} else if ((ec_chip->suspend_state == 0) && (ec_chip->op_mode == 0)){
		ASUSEC_NOTICE("keyboard closed\n");
		if (ec_chip->dock_in){
			ret_val = asusec_i2c_test(ec_chip->client);
			if(ret_val < 0){
				goto fail_to_access_ec;
			}
		
			asusec_dockram_read_data(0x0A);

			ec_chip->i2c_dm_data[0] = 8;
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xDF;
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x22;
			asusec_dockram_write_data(0x0A,9);
		}
	
fail_to_access_ec:	
		ec_chip->init_success = 0;
	}
	return 0;
	
}
EXPORT_SYMBOL(asusec_close_keyboard);


int asusec_suspend_hub_callback(void){
	int ret_val;

	printk("asusec_suspend_hub_callback+\n");
	ASUSEC_NOTICE("suspend\n");
	if (ec_chip->dock_in){
		ret_val = asusec_i2c_test(ec_chip->client);
		if(ret_val < 0){
			goto fail_to_access_ec;
		}
		
		asusec_dockram_read_data(0x0A);

		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xDF;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x22;
		if (ec_chip->ec_wakeup){
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x80;
		} else {
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0x7F;
		}
		asusec_dockram_write_data(0x0A,9);	
	}
	
fail_to_access_ec:		
	flush_workqueue(asusec_wq);
	ec_chip->suspend_state = 1;
	ec_chip->dock_det = 0;
	printk("asusec_suspend_hub_callback-\n");
	return 0;
	
}
EXPORT_SYMBOL(asusec_suspend_hub_callback);


int asusec_is_ac_over_10v_callback(void){

	int ret_val;

	ASUSEC_NOTICE("access dockram\n");
	if (ec_chip->dock_in){
		ret_val = asusec_i2c_test(ec_chip->client);
		if(ret_val < 0){
			goto fail_to_access_ec;
		}	
		asusec_dockram_read_data(0x0A);
		ASUSEC_NOTICE("byte[1] = 0x%x\n", ec_chip->i2c_dm_data[1]);

		return ec_chip->i2c_dm_data[1] & 0x20;
	}
		
fail_to_access_ec:	
	ASUSEC_NOTICE("dock doesn't exist or fail to access ec\n");
	return -1;
}
EXPORT_SYMBOL(asusec_is_ac_over_10v_callback);

int asusec_is_battery_full_callback(int full){

	int ret_val;

	ASUSEC_NOTICE("access dockram\n");
	if (ec_chip->dock_in){
		msleep(500);
		ret_val = asusec_i2c_test(ec_chip->client);
		if(ret_val < 0){
			goto fail_to_access_ec;
		}
		asusec_dockram_read_data(0x0A);

		ec_chip->i2c_dm_data[0] = 8;
		if (full){
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x10;
		} else{
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xEF;
		}
		ret_val = asusec_dockram_write_data(0x0A,9);
		return ret_val;
	}
	
fail_to_access_ec:	
	ASUSEC_NOTICE("dock doesn't exist or fail to access ec\n");
	return -1;
}
EXPORT_SYMBOL(asusec_is_battery_full_callback);

int asusec_dock_battery_callback(void){

	int bat_percentage = 0;
	int ret_val = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		ret_val = asusec_dockram_read_data(0x14);

		if (ret_val < 0)
			return -1;
		else{
			bat_percentage = (ec_chip->i2c_dm_data[14] << 8 )| ec_chip->i2c_dm_data[13];
			return bat_percentage;
		}
	}
	return -1;
}
EXPORT_SYMBOL(asusec_dock_battery_callback);


static int __init asusec_init(void)
{
	int err_code = 0;	

	if (asusec_major) {
		asusec_dev = MKDEV(asusec_major, asusec_minor);
		err_code = register_chrdev_region(asusec_dev, 1, "asusec");
	} else {
		err_code = alloc_chrdev_region(&asusec_dev, asusec_minor, 1,"asusec");
		asusec_major = MAJOR(asusec_dev);
	}

	ASUSEC_NOTICE("cdev_alloc\n") ;
	asusec_cdev = cdev_alloc() ;
	asusec_cdev->owner = THIS_MODULE ;
	asusec_cdev->ops = &asusec_fops ;
		
	err_code=i2c_add_driver(&asusec_driver);
	if(err_code){
		ASUSEC_ERR("i2c_add_driver fail\n") ;
		goto i2c_add_driver_fail ;
	}
	asusec_class = class_create(THIS_MODULE, "asusec");
	if(asusec_class <= 0){
		ASUSEC_ERR("asusec_class create fail\n");
		err_code = -1;
		goto class_create_fail ;
	}
	asusec_device = device_create(asusec_class, NULL, MKDEV(asusec_major, asusec_minor), NULL, "asusec" );
	if(asusec_device <= 0){
		ASUSEC_ERR("asusec_device create fail\n");
		err_code = -1;
		goto device_create_fail ;
	}

	ASUSEC_INFO("return value %d\n", err_code) ;

	return 0;

device_create_fail :
	class_destroy(asusec_class) ;	
class_create_fail :
	i2c_del_driver(&asusec_driver);	
i2c_add_driver_fail :

	return err_code;

}

static void __exit asusec_exit(void)
{
	device_destroy(asusec_class,MKDEV(asusec_major, asusec_minor)) ;
	class_destroy(asusec_class) ;
	i2c_del_driver(&asusec_driver);
	unregister_chrdev_region(asusec_dev, 1);
	switch_dev_unregister(&ec_chip->dock_sdev);
}

module_init(asusec_init);
module_exit(asusec_exit);
