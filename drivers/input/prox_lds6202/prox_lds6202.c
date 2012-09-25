/*
 * A sensor driver for the proximity LDS6202.
 *
 * PureTouch.* proximity Sensor IC Driver LDS6202.
 *
 * Copyright (c) 2010, ASUSTek Corporation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>

/*#define DEBUG           1*/
#define VERBOSE_DEBUG   1

#ifdef CONFIG_RIL
#define SET_RIL_3G
#endif

MODULE_DESCRIPTION("IDT Proximity Sensor Driver LDS6202");
MODULE_LICENSE("GPL");

/*----------------------------------------------------------------------------
** Debug Utility
**----------------------------------------------------------------------------*/
#define PROX_SENSOR_DEBUG			1

#if PROX_SENSOR_DEBUG
#define PROX_SENSOR_INFO(format, arg...)	\
	printk(KERN_INFO "PROX_SENSOR: [%s] " format , __FUNCTION__ , ## arg)
#else
#define PROX_SENSOR_INFO(format, arg...)
#endif

#define PROX_SENSOR_ERR(format, arg...)	\
	printk(KERN_ERR "PROX_SENSOR: [%s] " format , __FUNCTION__ , ## arg)

#undef DUMP_REG
#define PROX_INT				48		/* TEGRA_GPIO_PG0 */
#define PROX_RESET				50		/* TEGRA_GPIO_PG2 */
#define PROX_REG_SEN_STATUS	0x46	/* PROXIMITY SENSOR STATUS */

/*----------------------------------------------------------------------------
** Global Variable
**----------------------------------------------------------------------------*/
struct prox_lds6202_data{
	struct attribute_group attrs;
	int status;
	int obj_detect;
};

struct prox_lds6202_data *prox_data;
struct i2c_client *prox_client;
static dev_t prox_lds6202_dev;
struct cdev *prox_lds6202_cdev;
static struct class *prox_lds6202_class;
static struct device *prox_lds6202_class_device;
static int prox_lds6202_major = 0;
static int prox_lds6202_minor = 0;
struct work_struct pxy_work;
static struct workqueue_struct *prox_wq;
bool prox_active =false;
static struct switch_dev proxi_sdev;
static int proxi_in = 0;

/*----------------------------------------------------------------------------
** FUNCTION DECLARATION
**----------------------------------------------------------------------------*/
static int __devinit prox_lds6202_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit prox_lds6202_remove(struct i2c_client *client);
static int prox_lds6202_suspend(struct i2c_client *client, pm_message_t mesg);
static int prox_lds6202_resume(struct i2c_client *client);
static u16 prox_lds6202_read_reg(struct i2c_client *client, u16 reg);
static bool prox_lds6202_write_reg(struct i2c_client* client, u16 reg, u16 val);
static int __init prox_lds6202_init(void);
static void __exit prox_lds6202_exit(void);
static irqreturn_t prox_lds6202_interrupt_handler(int irq, void *dev_id);
static void prox_lds6202_work_function(struct work_struct *dat);
static int prox_lds6202_config_irq(struct i2c_client *client);
static int	init_prox_sensor(void);
int prox_lds6202_enable(void);
int prox_lds6202_disable(void);
extern void proxi_request_ril(int state);

/*----------------------------------------------------------------------------
** I2C Driver Structure
**----------------------------------------------------------------------------*/
static const struct i2c_device_id prox_lds6202_id[] = {
	{"prox_lds6202", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, prox_lds6202_id);

static struct i2c_driver prox_lds6202_driver = {
	.driver = {
		.name	= "prox_lds6202",
		.owner	= THIS_MODULE,
	},
	.probe		= prox_lds6202_probe,
	.remove		= __devexit_p(prox_lds6202_remove),
	.resume         = prox_lds6202_resume,
	.suspend        = prox_lds6202_suspend,
	.id_table	= prox_lds6202_id,
};

static ssize_t show_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	prox_data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", prox_data->status);
}

static ssize_t show_obj_detect(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	prox_data = i2c_get_clientdata(client);
	if (prox_active)
	return sprintf(buf, "%d\n", prox_data->obj_detect);
	else
		return sprintf(buf, "-1\n");
}

DEVICE_ATTR(prox_status, 0755, show_status, NULL);
DEVICE_ATTR(obj_detect, 0755, show_obj_detect, NULL);

static struct attribute *prox_lds6202_attr[] = {
	&dev_attr_prox_status.attr,
	&dev_attr_obj_detect.attr,
	NULL
};

/**********************************************************
**  Function: proximity driver I2C read operation
**  Parameter: I2C client, register read
**  Return value: if sucess, then returns the value of the register
**                      otherwise returns error code
************************************************************/
static u16 prox_lds6202_read_reg(struct i2c_client *client, u16 reg)
{
	struct i2c_msg msg[3];
	char data[2];
	u8 buf1[2];
	u16 result;
	int ret = 0;

	buf1[0] = (u8)reg >> 8;	/* Register Address(MSB) */
	buf1[1] = (u8)reg;			/* Register Address(LSB) */

	/* Write register */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf1;

	/* Read data */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = (u8 *)data;

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret < 0){
		dev_err(&client->dev, "Failed to read 0x%x: %d\n", reg, ret);
		return false;
	}

	result =  (data[1] & 0xff) | (u16)(data[0] << 8);

	return result;

}

/**********************************************************
**  Function: proximity driver I2C write operation
**  Parameter: I2C client, register read and the value written
**  Return value: if sucess, then returns 0
**                      otherwise returns error value
************************************************************/
static bool prox_lds6202_write_reg(struct i2c_client* client, u16 reg, u16 val)
{
	int err;
	struct i2c_msg msg[1];
	char data[4];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 4;
	msg->buf = data;

	data[0] = (u8)(reg >> 8);	/* Register Address(MSB) */
	data[1] = (u8)reg;			/* Register Address(LSB) */
	data[2] = (u8)(val >> 8);	/* Data Value(MSB) */
	data[3] = (u8)val;			/* Data Value(LSB) */

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return false;

	return true;

}


/**********************************************************
**  Function: Called to interrupt handler for reading register data
**  Parameter: work struct
**  Return value: none
************************************************************/
static void prox_lds6202_work_function(struct work_struct *dat)
{
	struct i2c_client *client;
	u16 status;
	int int_pxy;
#if VERBOSE_DEBUG
	int rc;
#endif
	client = prox_client;

#if VERBOSE_DEBUG
	rc = gpio_get_value(PROX_INT);
	PROX_SENSOR_INFO("The INT_PROX# state is %d\n",rc);
#endif

	status = prox_lds6202_read_reg(client, PROX_REG_SEN_STATUS);
	prox_data->obj_detect=status;


#ifdef SET_RIL_3G
	if (status == 1)
		proxi_request_ril(1);
	else
		proxi_request_ril(0);
#endif

	return ;

}

/**********************************************************
**  Function: Proximity interrupt handler for INT1
**  Parameter: dedicated irq
**  Return value: if sucess, then returns IRQ_HANDLED
**
************************************************************/
static irqreturn_t prox_lds6202_interrupt_handler(int irq, void *dev_id)
{
	queue_work(prox_wq, &pxy_work);

	return IRQ_HANDLED;
}

/**********************************************************
**  Function: Capacitive touch driver configure the external gpio pin
**  Parameter: dedicated irq
**  Return value: if sucess, then returns 0
**
************************************************************/
static int prox_lds6202_config_irq(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = irq_to_gpio(client->irq);
	const char* label = "prox_lds6126_input";

	PROX_SENSOR_INFO("INT configuration, GPIO = %d, IRQ = %d\n", gpio, client->irq);

	rc = gpio_request(gpio, label);
	if(rc){
		PROX_SENSOR_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	if(rc){
		PROX_SENSOR_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	PROX_SENSOR_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(client->irq, prox_lds6202_interrupt_handler, IRQF_TRIGGER_FALLING, label, client);
	if(rc < 0){
		PROX_SENSOR_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, client->irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
	PROX_SENSOR_INFO("request IRQ = %d, rc = %d\n", client->irq, rc);
	return 0 ;

err_gpio_request_irq_fail :
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;
}

/**********************************************************
**  Function: Capacitive touch driver device init: read the HelloPacket
**  Parameter: none
**  Return value: if sucess, then returns positive number
**                      otherwise returns negative number
************************************************************/
static int init_prox_sensor()
{
	struct i2c_client *client ;
	int ret;
	u8 read_status;
	u8 read2byte[2];
#if VERBOSE_DEBUG
	u16 temp;
#endif
	client = prox_client;

	PROX_SENSOR_INFO("Proximity sensor initialization\n");

#if VERBOSE_DEBUG
	temp = prox_lds6202_read_reg(client, 0x01f);
	PROX_SENSOR_INFO("Manufacturer ID(0x1F) is %x\n",temp);
#endif

	/* COLD RESET */
	prox_lds6202_write_reg(client, 0x0000, 0x0000);
	/* TOUCH DISABLE */
	prox_lds6202_write_reg(client, 0x0040, 0x0030);
	/* DCM CONFIG Disable */
	prox_lds6202_write_reg(client, 0x000A, 0x0000);
	/* PSEL, Enable */
	prox_lds6202_write_reg(client, 0x0041, 0x0001);
	/* INTERRUPT */
	prox_lds6202_write_reg(client, 0x0042, 0x0002);
	/* SELC_Unit Configuration */
	prox_lds6202_write_reg(client, 0x004E, 0x0000);
		/* Ambient Config */
	prox_lds6202_write_reg(client, 0x0051, 0x0A1F);
	/* Recalib Config*/
	prox_lds6202_write_reg(client, 0x0052, 0x07FF);
	/* Long Touch */
	prox_lds6202_write_reg(client, 0x0053, 0x07FF);
	/*PROXIMITY SENSOR*/
	prox_lds6202_write_reg(client, 0x0039, 0x5401);
	/* Scroll set */
	prox_lds6202_write_reg(client, 0x0074, 0x0000);
	/* Power on set  */
	prox_lds6202_write_reg(client, 0x0038, 0x0006);
	/* CHANNEL 0 SELC*/
	prox_lds6202_write_reg(client, 0x0030, 0x0080);
	/*PROXIMITY SENSOR STATUS*/
//	prox_lds6202_write_reg(client, 0x0046, 0x0000);
	/* CHANNEL 0 THRESHOLD */
	prox_lds6202_write_reg(client, 0x005F, 0x0001);
	/* CHANNEL 0 THRESHOLD */
	prox_lds6202_write_reg(client, 0x0060, 0x00FF);
	/* TOUCH ENABLE */
	prox_lds6202_write_reg(client, 0x0040, 0x8030);
	/* SOFT RESET */
	prox_lds6202_write_reg(client, 0x0001, 0x0000);

	PROX_SENSOR_INFO("PROX sensor initialization is DONE\n");

	return ret;

}

/**********************************************************
**  Function: Called to I2C slave detection for proximity driver
**  Parameter: I2C client
**  Return value: if success, returns 0
**
************************************************************/
static int __devinit prox_lds6202_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	bool ret;
	prox_client = client;

	PROX_SENSOR_INFO("\n");

	prox_data = kzalloc(sizeof(struct prox_lds6202_data), GFP_KERNEL);
	if (!prox_data){
		PROX_SENSOR_ERR("Memory allocation fails\n");
		err = -ENOMEM;
		goto out;
	}
	/* Touch data processing workqueue initialization */
	INIT_WORK(&pxy_work, prox_lds6202_work_function);

	i2c_set_clientdata(prox_client, prox_data);
	prox_client->flags = 0;
	strlcpy(prox_client->name, "prox_lds6202", I2C_NAME_SIZE);
	prox_data->status=0;	

	ret = prox_lds6202_read_reg(prox_client,0x40);
	if (!ret){
		PROX_SENSOR_ERR("Register read fails, no device\n");
		err = -ENOMEM;
		goto out;
	}

	cdev_add(prox_lds6202_cdev,prox_lds6202_dev,1);

	init_prox_sensor();

	/*Configure IRQ Pin*/
	prox_lds6202_config_irq(client);

	/* Register sysfs hooks */
	prox_data->attrs.attrs = prox_lds6202_attr;
	err = sysfs_create_group(&client->dev.kobj, &prox_data->attrs);
	if(err){
		dev_err(&client->dev, "Not able to create the sysfs\n");
		goto out;
	}

	prox_data->status=1;	
	prox_lds6202_disable();
	
	return 0;

out:
	return err;
}

/**********************************************************
**  Function: Called to remove devices from the adapter
**  Parameter: I2C client
**  Return value: if success, returns 0
**
************************************************************/
static int __devexit prox_lds6202_remove(struct i2c_client *client)
{
	struct prox_lds6202_data *prox_data = i2c_get_clientdata(client);

	PROX_SENSOR_INFO("\n");
	sysfs_remove_group(&client->dev.kobj, &prox_data->attrs);
	kfree(prox_data);

	return 0;
}

int prox_lds6202_open(struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t prox_lds6202_write(struct file *flip, const char __user *buf, size_t count, loff_t *offs) {
    char cmd;

    if (copy_from_user(&cmd, buf, sizeof(char))) return -EFAULT;

    if (cmd == '0')
        prox_lds6202_disable();
    else if (cmd == '1')
        prox_lds6202_enable();

    return count;
}

struct file_operations prox_lds6202_fops = {
	.owner =    THIS_MODULE,
	.open =     prox_lds6202_open,
    .write =    prox_lds6202_write,
};

/**********************************************************
**  Function: Suspend function: configure gpio and flush workqueue
**  Parameter:
**  Return value: none
**
************************************************************/
static int prox_lds6202_suspend(struct i2c_client *client, pm_message_t mesg)
{
	PROX_SENSOR_INFO("\n");
	printk("prox_lds6202_suspend+");
	cancel_work_sync(&pxy_work);
	flush_workqueue(prox_wq);
	printk("prox_lds6202_suspend-");
	return 0;
}

/**********************************************************
**  Function: Resume function: re-config interrupt pin setting
**  Parameter:
**  Return value: none
**
************************************************************/
static int prox_lds6202_resume(struct i2c_client *client)
{
	PROX_SENSOR_INFO("\n");
	printk("prox_lds6202_resume+");
	queue_work(prox_wq, &pxy_work);
	printk("prox_lds6202_resume-");
	return 0;
}

/**********************************************************
**  Function: Device enable function: called by RIL
**  Parameter:
**  Return value: none
**
************************************************************/
int prox_lds6202_enable()
{
	struct i2c_client *client;
	client = prox_client;

	PROX_SENSOR_INFO("Call by RIL and enable proximity sensor\n");
	prox_lds6202_write_reg(client, 0x0002, 0x0000);

	prox_active=true;
}
EXPORT_SYMBOL(prox_lds6202_enable);

/**********************************************************
**  Function: Device disable function: called by RIL
**  Parameter:
**  Return value: none
**
************************************************************/
int prox_lds6202_disable()
{
	struct i2c_client *client;
	client = prox_client;

	PROX_SENSOR_INFO("Call by RIL and disable proximity sensor\n");
	prox_lds6202_write_reg(client, 0x0002, 0x0001);

	prox_active=false;
}
EXPORT_SYMBOL(prox_lds6202_disable);

/**********************************************************
**  Function: Proximity driver initialize
**  Parameter: none
**  Return value: if sucess, then returns 0
**                      otherwise returns error code
************************************************************/
static int __init prox_lds6202_init(void)
{
	int rc;
	const char* label = "prox_lds6126_output";

	PROX_SENSOR_INFO("Init, proximity sensor is on I2C-GEN2\n");

	prox_wq = create_singlethread_workqueue("prox_wq");
	if(!prox_wq)
		return -ENOMEM;

	if(prox_lds6202_major){
			prox_lds6202_dev = MKDEV(prox_lds6202_major, prox_lds6202_minor);
			rc = register_chrdev_region(prox_lds6202_dev, 1, "prox_lds6202");
	}else{
            rc = alloc_chrdev_region(&prox_lds6202_dev, prox_lds6202_minor, 1,"prox_lds6202");
            prox_lds6202_major = MAJOR(prox_lds6202_dev);
	}
	if(rc < 0){
            PROX_SENSOR_ERR("can't get major %d\n", prox_lds6202_major);
            return rc;
	}
	PROX_SENSOR_INFO("cdev_alloc\n");
	prox_lds6202_cdev = cdev_alloc();
	prox_lds6202_cdev->owner = THIS_MODULE;
	prox_lds6202_cdev->ops = &prox_lds6202_fops;

	prox_lds6202_class = class_create(THIS_MODULE, "prox_lds6202");
	prox_lds6202_class_device = device_create(prox_lds6202_class, NULL, MKDEV(prox_lds6202_major, prox_lds6202_minor), NULL, "prox_lds6202" );

	rc = i2c_add_driver(&prox_lds6202_driver);
	if(rc)
		PROX_SENSOR_ERR("i2c_add_driver fail\n");

	tegra_gpio_enable(PROX_INT);
	gpio_request(PROX_INT, label);
	rc = gpio_request(PROX_RESET, label);
	if(rc)
		PROX_SENSOR_ERR("gpio_request failed for input %d\n", PROX_RESET);
	gpio_direction_output(PROX_RESET,1);
	rc = gpio_get_value(PROX_RESET);
	PROX_SENSOR_INFO("The RESET_PROX state is %d(High: ON)\n",rc);

	return 0;

}

/**********************************************************
**  Function: Proximity driver exit function
**  Parameter: none
**  Return value: none
**
************************************************************/
static void __exit prox_lds6202_exit(void)
{
	PROX_SENSOR_INFO("\n");

	i2c_del_driver(&prox_lds6202_driver);
	if (prox_wq)
		destroy_workqueue(prox_wq);

	cdev_del(prox_lds6202_cdev);
	unregister_chrdev_region(prox_lds6202_dev, 1);
	class_destroy(prox_lds6202_class);
}

module_init(prox_lds6202_init);
module_exit(prox_lds6202_exit);

