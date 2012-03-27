#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/earlysuspend.h>
#include <linux/freezer.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/slab.h>

#include "elan_i2c_asus.h"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

static int elan_i2c_asus_cmd(struct i2c_client *client,unsigned char *param, int command)
{
  
	u16 asus_ec_cmd;
	int ret;
	int retry = ELAN_RETRY_COUNT;
	int i;
	int retry_data_count;
	u8 i2c_data[16];
	int index;

	ELAN_INFO("command = 0x%x\n",command);
	asus_ec_cmd = (((command & 0x00ff) << 8) | 0xD4);
	ret = 0;
	ret = i2c_smbus_write_word_data(client, 0x64, asus_ec_cmd);
	if (ret < 0) {
		ELAN_ERR("Wirte to device fails status %x\n",ret);
		return ret;
	}
	msleep(CONVERSION_TIME_MS);	
	
	while(retry-- > 0){
		ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, i2c_data);
		if (ret < 0) {
			ELAN_ERR("Fail to read data, status %d\n", ret);
			return ret;
		}
		ASUSEC_I2C_DATA(i2c_data, index);
		if ((i2c_data[1] & ASUSEC_OBF_MASK) && 
			(i2c_data[1] & ASUSEC_AUX_MASK)){ 
			if (i2c_data[2] == PSMOUSE_RET_ACK){
				break;
			}
			else if (i2c_data[2] == PSMOUSE_RET_NAK){
				goto fail_elan_touchpad_i2c;
			}
		}		
		msleep(CONVERSION_TIME_MS/5);
	}
	
	retry_data_count = (command & 0x0f00) >> 8;
	for(i=1; i <= retry_data_count; i++){
		param[i-1] = i2c_data[i+2];	  
	}
	  
	return 0;
	
fail_elan_touchpad_i2c:	
	ELAN_ERR("fail to get touchpad response");
	return -1;
  
}

/*
 * Interpret complete data packets and report absolute mode input events for
 * hardware version 2. (6 byte packets)
 */
void elantech_report_absolute_to_related(struct asusec_chip *ec_chip, int *Null_data_times)
{
	struct elantech_data *etd;
	struct input_dev *dev;
	unsigned char *packet;
	unsigned int fingers;
	int left_button, right_button;
	int x, y;
	int last_fingers;
	int i;

	etd = (struct elantech_data *) ec_chip->private;
	dev = etd->abs_dev;
 	packet = ec_chip->ec_data;

	// Report multitouch events for fingers.
	fingers = (packet[0] & 0xc0) >> 6;
	x = ((packet[1] & 0x0f) << 8) | packet[2];
	y = etd->ymax - (((packet[4] & 0x0f) << 8) | packet[5]);
	//printk("fingers=%d, x=%d, y=%d, packet=%02x,%02x,%02x,%02x,%02x,%02x\n", fingers, x, y, packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]);

	last_fingers = etd->fingers;

	switch (fingers) {
		case 0:
			// No fingers down.
			etd->fingers = 0;
			break;

		case 1:
			// One finger down.
			etd->fingers = 1;
			etd->pos[0].x = x;
			etd->pos[0].y = y;
			break;

		case 2:
			// Two fingers down.
			// Wait to get data from both fingers.
			if (etd->fingers != 2) {
				etd->fingers = 2;
				etd->pos[0].x = -1;
				etd->pos[1].x = -1;
			}
			if ((packet[0] & 0x0c) == 0x04) {
				etd->pos[0].x = x;
				etd->pos[0].y = y;
			} else {
				etd->pos[1].x = x;
				etd->pos[1].y = y;
			}
			if (etd->pos[0].x < 0 || etd->pos[1].x < 0)
				return;
			break;

		case 3:
			// Three or more fingers down.
			// Wait for at least one finger to go up.
			return;
	}

	// Send finger reports.
	if (etd->fingers) {
		for (i = 0; i < etd->fingers; i++) {
			input_report_abs(dev, ABS_MT_POSITION_X, etd->pos[i].x);
			input_report_abs(dev, ABS_MT_POSITION_Y, etd->pos[i].y);
			input_mt_sync(dev);
		}
	} else if (last_fingers) {
		input_mt_sync(dev);
	}

	// Send button press / release events.
	left_button = (packet[0] & 0x01);
	if (left_button != etd->left_button) {
		input_report_key(dev, BTN_LEFT, left_button);
		etd->left_button = left_button;
	}

	right_button = (packet[0] & 0x02) >> 1;
	if (right_button != etd->right_button) {
		input_report_key(dev, BTN_RIGHT, right_button);
		etd->right_button = right_button;
	}
	
	input_sync(dev);
}

/*
 * Put the touchpad into absolute mode
 */
 
static int elantech_set_absolute_mode(struct asusec_chip *ec_chip)
{
	
	struct i2c_client *client;
	unsigned char reg_10 = 0x03;	
		
	ELAN_INFO("elantech_set_absolute_mode 2\n");
	client = ec_chip->client;
	
	if ((!elan_i2c_asus_cmd(client, NULL, ETP_PS2_CUSTOM_COMMAND)) &&
	    (!elan_i2c_asus_cmd(client, NULL, ETP_REGISTER_RW)) &&
	    (!elan_i2c_asus_cmd(client, NULL, ETP_PS2_CUSTOM_COMMAND)) &&
	    (!elan_i2c_asus_cmd(client, NULL, 0x0010)) &&
	    (!elan_i2c_asus_cmd(client, NULL, ETP_PS2_CUSTOM_COMMAND)) &&
	    (!elan_i2c_asus_cmd(client, NULL, reg_10)) &&
	    (!elan_i2c_asus_cmd(client, NULL, PSMOUSE_CMD_SETSCALE11))) {
		
		return 0;
	}
	return -1; 
}


/*
 * Set the appropriate event bits for the input subsystem
 */
static int elantech_set_input_rel_params(struct asusec_chip *ec_chip)
{
	struct elantech_data *etd;
	unsigned char param[3];
	int ret;
			 
	if ((!elan_i2c_asus_cmd(ec_chip->client, NULL, ETP_PS2_CUSTOM_COMMAND)) &&
	    (!elan_i2c_asus_cmd(ec_chip->client, NULL, 0x0000)) &&
	    (!elan_i2c_asus_cmd(ec_chip->client, param, PSMOUSE_CMD_GETINFO))){
		
		etd = ec_chip->private;
		if(etd->abs_dev){
			return 0;
		}
		
		etd->xmax = (0x0F & param[0]) << 8 | param[1];
		etd->ymax = (0xF0 & param[0]) << 4 | param[2];		
		
		etd->abs_dev = input_allocate_device();
		ELAN_INFO("1 elantech_touchscreen=%p\n",etd->abs_dev);
		if (etd->abs_dev != NULL){
			ELAN_INFO("2 elantech_touchscreen=%p\n",etd->abs_dev);
			etd->abs_dev->name = "elantech_touchscreen";
			
			etd->abs_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_SYN);
			etd->abs_dev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_RIGHT);
		
			set_bit(EV_SYN, etd->abs_dev->evbit);
			set_bit(EV_KEY, etd->abs_dev->evbit);
			set_bit(EV_ABS, etd->abs_dev->evbit);
			input_set_abs_params(etd->abs_dev, ABS_MT_POSITION_X, 0, etd->xmax, 0, 0);
			input_set_abs_params(etd->abs_dev, ABS_MT_POSITION_Y, 0, etd->ymax, 0, 0);

			ret=input_register_device(etd->abs_dev);
			if (ret) {
			      ELAN_ERR("Unable to register %s input device\n", etd->abs_dev->name);		  
			}
		}
		return 0;
	}
	return -1;
  
}


/*
 * Use magic knock to detect Elantech touchpad
 */
int elantech_detect(struct asusec_chip *ec_chip)
{
	struct i2c_client *client;
	unsigned char param[3];
	ELAN_INFO("2.6.2X-Elan-touchpad-2010-11-27\n");

	client = ec_chip->client;
	
	if (elan_i2c_asus_cmd(client,  NULL, PSMOUSE_CMD_DISABLE) ||
	    elan_i2c_asus_cmd(client,  NULL, PSMOUSE_CMD_SETSCALE11) ||
	    elan_i2c_asus_cmd(client,  NULL, PSMOUSE_CMD_SETSCALE11) ||
	    elan_i2c_asus_cmd(client,  NULL, PSMOUSE_CMD_SETSCALE11) ||
	    elan_i2c_asus_cmd(client, param, PSMOUSE_CMD_GETINFO)) {
		ELAN_ERR("sending Elantech magic knock failed.\n");
		return -1;
	}

	/*
	 * Report this in case there are Elantech models that use a different
	 * set of magic numbers
	 */
	if (param[0] != 0x3c ||param[1] != 0x03 || param[2]!= 0x00) {
		ELAN_ERR("unexpected magic knock result 0x%02x, 0x%02x, 0x%02x.\n",
			param[0], param[1],param[2]);
		return -1;
	}
	
	return 0;
}

/*
 * Initialize the touchpad and create sysfs entries
 */
int elantech_init(struct asusec_chip *ec_chip)
{  
	ELAN_INFO("Elan et1059 elantech_init\n");
	
	if (elantech_set_absolute_mode(ec_chip)){
		ELAN_ERR("failed to put touchpad into absolute mode.\n");
		return -1;
	}
	if (elantech_set_input_rel_params(ec_chip)){
		ELAN_ERR("failed to elantech_set_input_rel_params.\n");
		return -1;
	}
	//elan_i2c_asus_cmd(ec_chip->client,  NULL, PSMOUSE_CMD_ENABLE);
	return 0;
}
