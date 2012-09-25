/*
 * Lite-On AL3000A Digital Ambient Light Sensor
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>

#include <asm/gpio.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/delay.h>

/*#define DEBUG           1*/
/*#define VERBOSE_DEBUG   1*/

#undef DUMP_REG

#define AL3000A_REG_CONFIGURATION 0x00
#define AL3000A_REG_TIMING_CONTROL 0x01
#define AL3000A_REG_ALS_CONTROL  0x02
#define AL3000A_REG_INTERRUPT_STATUS 0x03
#define AL3000A_REG_DATA 0x05
#define AL3000A_REG_ALS_WINDOW 0x08

#define AL3000A_MAX_REGS AL3000A_REG_ALS_WINDOW

#define AL3000A_MODE_POWER_UP 0
#define AL3000A_MODE_POWER_DOWN 2
#define AL3000A_MODE_RESET 3

#define AL3000A_OPERATION_ACTIVE 0
#define AL3000A_OPERATION_IDLE 3

#define MAX_CONVERSION_TRIAL 5
#define MAX_CONVERION_TIMEOUT 1000
#define CONVERSION_DONE_POLL_TIME 10

#define AL3000A_IOC_MAGIC 0xF3
#define AL3000A_IOC_MAXNR 2
#define AL3000A_POLL_DATA _IOR(AL3000A_IOC_MAGIC,2,int )

#define AL3000A_IOCTL_START_HEAVY 2
#define AL3000A_IOCTL_START_NORMAL 1
#define AL3000A_IOCTL_END 0

#define START_NORMAL	(HZ)
#define START_HEAVY	(HZ)

#define LIGHT_IRQ_GPIO	TEGRA_GPIO_PZ2

extern unsigned int ASUSGetProjectID( void );

static bool al3000a_init_fail = false;
static int poll_mode=0;
struct delayed_work al3000a_poll_data_work;
static struct workqueue_struct *sensor_work_queue;
struct i2c_client *al3000a_client;
/*
static int lux_table[64]={1,1,1,2,2,2,3,4,4,5,6,7,9,11,13,16,19,22,27,32,39,46,56,67,80,96,
                          116,139,167,200,240,289,346,416,499,599,720,864,1037,1245,1495,
                          1795,2154,2586,3105,3728,4475,5372,6449,7743,9295,11159,13396,
                          16082,19307,23178,27826,33405,40103,48144,57797,69386,83298,
                          100000};
*/

/* default array is lux_ep101_table */
static int lux_table[64]={
1,1,1,2,2,2,3,4,
4,5,10,20,50,70,100,150,
200,250,300,350,400,500,600,700,
900,1100,1400,1500,1500,1500,1500,1500,
1500,1500,1500,1500,1500,1500,1500,1500,
1500,1795,2154,2586,3105,3728,4475,5372,
6449,7743,9295,11159,13396,16082,19307,23178,
27826,33405,40103,48144,57797,69386,83298,100000};

/* 200 , 300 , 400 , 500 , 600 , 700 , 900 , 1100 , 1400 are tested */
static int lux_ep101_table[64]={
1,1,1,2,2,2,3,4,
4,5,10,20,50,70,100,150,
200,250,300,350,400,500,600,700,
900,1100,1400,1500,1500,1500,1500,1500,
1500,1500,1500,1500,1500,1500,1500,1500,
1500,1795,2154,2586,3105,3728,4475,5372,
6449,7743,9295,11159,13396,16082,19307,23178,
27826,33405,40103,48144,57797,69386,83298,100000};

/* 100 , 200 , 300 , 400 , 500 , 600 , 700 , 900 , 1100 , 1400 are tested */
static int lux_reduce_ep101_table[64]={
1,1,1,2,2,2,3,4,
4,10,20,50,70,100,150,200,
250,300,350,400,500,600,700,900,
1100,1400,1500,1500,1500,1500,1500,1500,
1500,1500,1500,1500,1500,1500,1500,1500,
1500,1795,2154,2586,3105,3728,4475,5372,
6449,7743,9295,11159,13396,16082,19307,23178,
27826,33405,40103,48144,57797,69386,83298,100000};

/* 100 - 1500 are tested */
static int lux_ep102_table[64]={
1,3,5,10,20,30,40,60,
80,100,125,150,175,200,230,270,
300,350,400,500,600,700,900,1100,
1200,1400,1500,1500,1500,1500,1500,1500,
1500,1500,1500,1500,1500,1500,1500,1500,
1500,1500,1500,1500,1500,1500,1500,1500,
1500,1500,1500,1500,1500,1500,1500,1500,
1500,1500,1500,1500,1500,1500,1500,1500};

struct al3000a_data {
	struct device           *hwmon_dev;
	struct attribute_group  attrs;
	struct i2c_client *client;
	struct mutex            lock;
	struct work_struct work;
	struct miscdevice misc_dev;
	int                      lux;
	u8                      reg_cache[AL3000A_MAX_REGS];
	int                     eoc_gpio;
	int                     eoc_irq;
	int 		        status;
};


static int al3000a_probe(struct i2c_client *client,
			   const struct i2c_device_id *id);
static int al3000a_remove(struct i2c_client *client);
static int al3000a_suspend(struct i2c_client *client, pm_message_t mesg);
static int al3000a_resume(struct i2c_client *client);

static const struct i2c_device_id al3000a_id[] = {
	{"al3000a", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, al3000a);

static struct i2c_driver al3000a_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "al3000a",
	},
	.probe		= al3000a_probe,
	.remove		= al3000a_remove,
	.resume         = al3000a_resume,
	.suspend        = al3000a_suspend,
	.id_table	= al3000a_id,
};
int al3000a_open(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__);
	return 0;          /* success */
}


int al3000a_release(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__);
	return 0;          /* success */
}

int al3000a_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 1;

	if (_IOC_TYPE(cmd) != AL3000A_IOC_MAGIC)
	return -ENOTTY;
	if (_IOC_NR(cmd) > AL3000A_IOC_MAXNR)
	return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
	err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
	err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	switch (cmd) {
		case AL3000A_POLL_DATA:
			if (arg == AL3000A_IOCTL_START_HEAVY){
				printk("light sensor heavey\n");
			poll_mode = START_HEAVY;
			queue_delayed_work(sensor_work_queue, &al3000a_poll_data_work, poll_mode);
			}
			else if (arg == AL3000A_IOCTL_START_NORMAL){
				printk("light sensor normal\n");
				poll_mode = START_NORMAL;
				queue_delayed_work(sensor_work_queue, &al3000a_poll_data_work, poll_mode);
			}
			else if  (arg == AL3000A_IOCTL_END){
			printk("light sensor end\n");
			cancel_delayed_work_sync(&al3000a_poll_data_work);
			}
			else
	return -ENOTTY;
	break;
	default: /* redundant, as cmd was checked against MAXNR */
	return -ENOTTY;
		}
	return 0;
}

	struct file_operations al3000a_fops = {
		.owner =    THIS_MODULE,
		.unlocked_ioctl =	al3000a_ioctl,
		.open =		al3000a_open,
		.release =	al3000a_release,
		};

static bool al3000a_write_data(struct i2c_client *client,
		u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 w_data[2];
	int ret = 0;

	struct al3000a_data *data = i2c_get_clientdata(client);

	w_data[0] = reg;
	w_data[1] = val;

	dev_vdbg(&client->dev,"%s(): Writing Reg 0x%x to value 0x%x\n",
			__func__,w_data[0], w_data[1]);
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = w_data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Write to device fails status %x\n", ret);
		return false;
	}
	data->reg_cache[reg] = val;
	return true;
}

static bool al3000a_read_data(struct i2c_client *client,
		u8 reg, u8 length, u8 * buffer)
{
	struct i2c_msg msg[2];
	u8 w_data[2];
	int ret = 0;

	w_data[0] = reg;

	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_NOSTART;	/* set repeated start and write */
	msg[0].len = 1;
	msg[0].buf = w_data;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = length;
	msg[1].buf = buffer;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Read from device fails.\n");
		return false;
	}
	return true;
}

static void  al3000a_poll_data(struct work_struct * work)
{
	bool status;
	u8 count;
	status = al3000a_read_data(al3000a_client, AL3000A_REG_DATA, 1, &count);
	if( !status)
		printk("Read light sensor data fail\n");

	if(poll_mode ==0)
		msleep(5);

	queue_delayed_work(sensor_work_queue, &al3000a_poll_data_work, poll_mode);
}


void al3000a_dump_reg(struct i2c_client *client)
{
	u8 value, offset;

	for(offset=AL3000A_REG_CONFIGURATION; offset<=AL3000A_MAX_REGS; offset++){
		al3000a_read_data(client, offset, sizeof(value), &value);
		printk("al3000a:%s:[0x%02X]=0x%02X\n", __FUNCTION__, offset, value);
	}
	return;
}

static bool al3000a_read_lux(struct i2c_client *client,
		 int * lux)
{
	struct al3000a_data *data = i2c_get_clientdata(client);
	u32 timeout_ms = MAX_CONVERION_TIMEOUT;
	u8 count;
	u8 read_status;
	bool status;
	int state;

	dev_vdbg(&client->dev, "%s()\n", __func__);
	status = al3000a_write_data(client, AL3000A_REG_CONFIGURATION, 0x00);
	if (!status) {
		dev_err(&client->dev, "Error in setting active mode\n");
		return false;
	}

	while (timeout_ms) {
		msleep(CONVERSION_DONE_POLL_TIME);
		state = (gpio_get_value(data->eoc_gpio) ? 1 : 0);
		if (!state)
			break;
		timeout_ms -= CONVERSION_DONE_POLL_TIME;
	}
	if (!timeout_ms) {
		dev_err(&client->dev, "Conversion timeout happend\n");
		return false;
	}
	status = al3000a_read_data(client, AL3000A_REG_INTERRUPT_STATUS, 1, &read_status);
	if (!status) {
		dev_err(&client->dev, "Error in reading interrupt status\n");
		return false;
	}

	if (read_status)
		status = al3000a_read_data(client, AL3000A_REG_DATA, 1, &count);
	if (!status || !read_status) {
		dev_err(&client->dev, "Read data fails\n");
		return false;
	}
	status = al3000a_write_data(client, AL3000A_REG_CONFIGURATION, 0x03);
	if (!status) {
		dev_err(&client->dev, "Error in setting idlemode\n");
		return false;
	}
	count = count & 0x3F;
	*lux = lux_table[count];
	data->lux = *lux;

	return true;
}

static int al3000a_Init(struct i2c_client *client)
{
	int project_id = ASUSGetProjectID();
	int i=0;
	printk("ASUSGetProjectID() = %d \n", project_id);
	switch(project_id){
		case 101:
		for(i=0;i<64;i++){
			lux_table[i] = lux_reduce_ep101_table[i];
		}
		break;
		case 102:
		for(i=0;i<64;i++){
			lux_table[i] = lux_ep102_table[i];
		}
		break;
		default:
		for(i=0;i<64;i++){
			lux_table[i] = lux_reduce_ep101_table[i];
		}
	}

	struct al3000a_data *data = i2c_get_clientdata(client);
	u8 buffer[1];
	bool status;

	dev_vdbg(&client->dev, "%s()\n", __func__);

	/* Write the reset mode */
	status = al3000a_write_data(client, AL3000A_REG_CONFIGURATION, 0x0F );
	if (!status) {
		dev_err(&client->dev, "Error in setting reset mode\n");
		return -ENODEV;
	}
	status = al3000a_write_data(client, AL3000A_REG_CONFIGURATION, 0x00 );
	if (!status) {
		dev_err(&client->dev, "Error in setting power up and active mode\n");
		return -ENODEV;
	}
	/* Get light  data and store in the device data */
	status = al3000a_read_data(client, AL3000A_REG_DATA, 1, buffer);
	if (!status) {
		dev_err(&client->dev, "Not able to read light data\n");
		return -ENODEV;
	}
	data->lux = lux_table[buffer[0] & 0x3F];

	return 0;
}

static ssize_t show_show_reg(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	if(al3000a_init_fail == true){
		return sprintf(buf, "%d \n", -1);
	}
	struct i2c_client *client = to_i2c_client(dev);
	struct al3000a_data *data = i2c_get_clientdata(client);
	u8 count;

	al3000a_read_data(al3000a_client, AL3000A_REG_DATA, 1, &count);
	count = count & 0x3F;
	return sprintf(buf, "%d \n", count);
}

static ssize_t show_show_lux(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	if(al3000a_init_fail == true){
		printk("Light sensor: al3000a hardware chip fail !!!! \n");
		return sprintf(buf, "%d \n", -1);
	}
	struct i2c_client *client = to_i2c_client(dev);
	struct al3000a_data *data = i2c_get_clientdata(client);
	u8 raw_data;

	if(al3000a_read_data(al3000a_client, AL3000A_REG_DATA, 1, &raw_data)){
		return sprintf(buf, "%d \n", lux_table[raw_data & 0x3F]);
	}
	else
		return sprintf(buf, "%d \n", data->lux);
}

static ssize_t show_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct al3000a_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->status);
}

static SENSOR_DEVICE_ATTR(show_reg, 0755, show_show_reg, NULL, 1);
static SENSOR_DEVICE_ATTR(show_lux, 0755, show_show_lux, NULL, 1);
static SENSOR_DEVICE_ATTR(lightsensor_status, 0755, show_status, NULL, 2);

static struct attribute *al3000a_attr[] = {
	&sensor_dev_attr_show_reg.dev_attr.attr,
	&sensor_dev_attr_show_lux.dev_attr.attr,
	&sensor_dev_attr_lightsensor_status.dev_attr.attr,
	NULL
};

static irqreturn_t al3000a_interrupt_handler(int irq, void *dev_id)
{
	//printk("Light sensor: %s\n", __FUNCTION__);
	struct al3000a_data * data= dev_id;

	schedule_work(&data->work);

	return IRQ_HANDLED;
}

static void al3000a_work(struct work_struct *work)
{
	struct al3000a_data *data = container_of(work, struct al3000a_data, work);
	u8 count;
	bool status;

	dev_vdbg(&data->client->dev, "%s()\n", __func__);

	status = al3000a_read_data(data->client, AL3000A_REG_DATA, 1, &count);
	if (!status) {
		dev_err(&data->client->dev, "Read data fails\n");
	}
	else{
		count = count & 0x3F;
		data->lux = lux_table[count];
		//printk("lux value is : %d \n", data->lux);
	}
}

static int al3000a_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct al3000a_data *data;
	int err;

	dev_dbg(&client->dev, "%s()\n", __func__);

	data = kzalloc(sizeof (struct al3000a_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);
	INIT_WORK(&data->work, al3000a_work);
	data->status = 0;
	data->client = client;
	data->eoc_irq = client->irq;
	data->eoc_gpio = irq_to_gpio(client->irq);
	al3000a_client= data->client;

	tegra_gpio_enable(data->eoc_gpio);
	err = gpio_request(data->eoc_gpio, "al3000a");
	if (err < 0) {
		dev_err(&client->dev, "failed to request GPIO %d, error %d\n",
			data->eoc_gpio, err);
		goto exit_free;
	}

	err = gpio_direction_input(data->eoc_gpio);
	if (err < 0) {
		dev_err(&client->dev, "Failed to configure input direction for"
			" GPIO %d, error %d\n", data->eoc_gpio, err);
		gpio_free(data->eoc_gpio);
		goto exit_gpio;
	}

	err=request_irq(data->eoc_irq, al3000a_interrupt_handler,
		IRQF_TRIGGER_FALLING | IRQF_DISABLED , "al3000a", data);
	if(err){
		printk("AL3000A : %s request_irq fail\n", __FUNCTION__);
		goto exit_gpio;
	}
	err = al3000a_Init(client);
	if (err < 0) {
		al3000a_init_fail = true;
		printk("Light sensor: al3000a chip init fail !!!! \n");
		//dev_err(&client->dev, "AL3000A  initialization fails\n");
		//goto exit_int;
	}

	dev_info(&client->dev, "%s chip found Gpio %d\n", client->name,
		 data->eoc_gpio);
#ifdef DUMP_REG
	al3000a_dump_reg(client);
#endif

sensor_work_queue = create_singlethread_workqueue("i2c_lightsensor_wq");
	if(!sensor_work_queue){
		pr_err("al3000a_probe: Unable to create workqueue");
		goto exit_int;
		}

	INIT_DELAYED_WORK(&al3000a_poll_data_work, al3000a_poll_data);
	data->misc_dev.minor  = MISC_DYNAMIC_MINOR;
	data->misc_dev.name = "lightsensor";
	data->misc_dev.fops = &al3000a_fops;
	err = misc_register(&data->misc_dev);
		if (err) {
			pr_err("tegra_acc_probe: Unable to register %s\misc device\n", data->misc_dev.name);
		goto exit_int;
			}
	/* Register sysfs hooks */
	data->attrs.attrs = al3000a_attr;
	err = sysfs_create_group(&client->dev.kobj, &data->attrs);
	if (err) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		goto exit_free;
	}

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		dev_err(&client->dev, "hwmon registration fails\n");
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}
	data->status = 1;
	pr_info("%s()\n", __func__);
printk("Light sensor: probe succeed\n");

	return 0;

exit_remove:
	sysfs_remove_group(&client->dev.kobj, &data->attrs);
exit_int:
	free_irq(data->eoc_irq, data);
exit_gpio:
	gpio_free(data->eoc_gpio);
exit_free:
	kfree(data);
exit:
	return err;
}

static int al3000a_remove(struct i2c_client *client)
{
	struct al3000a_data *data = i2c_get_clientdata(client);

	misc_deregister(&data->misc_dev);
	dev_dbg(&client->dev, "%s()\n", __func__);
	pr_info("%s()\n", __func__);
	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &data->attrs);
	gpio_free(data->eoc_gpio);
	kfree(data);
	return 0;
}

static int al3000a_suspend(struct i2c_client *client, pm_message_t mesg)
{
	if(al3000a_init_fail == true){
		return 0;
	}
	struct al3000a_data *data = i2c_get_clientdata(client);
	bool status;

	dev_dbg(&client->dev, "%s()\n", __func__);
	pr_info("%s()\n", __func__);

	mutex_lock(&data->lock);
	status = al3000a_write_data(client, AL3000A_REG_CONFIGURATION,  0x0B );
	if (!status) {
		dev_err(&client->dev, "Error in setting power down and idle mode\n");
		mutex_unlock(&data->lock);
		return 0;
	}
	mutex_unlock(&data->lock);
	return 0;
}
static int al3000a_resume(struct i2c_client *client)
{
	if(al3000a_init_fail == true){
		return 0;
	}
	struct al3000a_data *data = i2c_get_clientdata(client);
	bool status;

	dev_dbg(&client->dev, "%s()\n", __func__);
	pr_info("%s()\n", __func__);

	mutex_lock(&data->lock);
	status = al3000a_write_data(client, AL3000A_REG_CONFIGURATION, 0x00);
	if (!status) {
		dev_err(&client->dev, "Error in setting power up mode\n");
		mutex_unlock(&data->lock);
		return 0;
	}
	mutex_unlock(&data->lock);
	return 0;
}

static int __init al3000a_init(void)
{
	pr_info("%s()\n", __func__);
	return i2c_add_driver(&al3000a_driver);
}

static void __exit al3000a_exit(void)
{
	pr_info("%s()\n", __func__);
	i2c_del_driver(&al3000a_driver);
}

module_init(al3000a_init);
module_exit(al3000a_exit);
