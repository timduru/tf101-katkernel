#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <mach/board-ventana-misc.h>
#include <../pm-irq.h>

#include "ril.h"
#include "proximity.h"
#include "sim.h"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

static struct class *ril_class;
static struct devices *dev;
static struct delayed_work workq;
static dev_t ril_dev;
static int ril_major = 0;
static int ril_minor = 0;
static int proxi_out = 0;
static int is3G = 0;
static int modem_detect_count = 0;

static ssize_t show_power_state(struct device *class, struct device_attribute *attr, char *buf);
static ssize_t store_power_state(struct device *class, struct attribute *attr, const char *buf, size_t count);
static ssize_t show_reset_state(struct device *class, struct device_attribute *attr, char *buf);
static ssize_t store_reset_state(struct device *class, struct attribute *attr, const char *buf, size_t count);
static ssize_t show_proxi_state(struct device *class, struct device_attribute *attr, char *buf);
static ssize_t store_proxi_state(struct device *class, struct attribute *attr, const char *buf, size_t count);
static ssize_t show_sar3g_state(struct device *class, struct device_attribute *attr, char *buf);
static ssize_t store_sar3g_state(struct device *class, struct attribute *attr, const char *buf, size_t count);
static ssize_t show_3g_available(struct device *class, struct device_attribute *attr, char *buf);

static int ril_probe(struct platform_device *pdev);
static void ril_shutdown(struct platform_device *pdev);

static struct platform_device ril_device = {
	.name = "ril",
};

struct platform_driver ril_driver = {
	.probe      = ril_probe,
	.shutdown   = ril_shutdown,
	.driver     = {
		.name   = "ril",
		.owner  = THIS_MODULE,
	},
};

static struct device_attribute ril_device_attr[] = {
		__ATTR(3g_power, 00644, show_power_state, store_power_state),
		__ATTR(3g_reset, 00644, show_reset_state, store_reset_state),
		__ATTR(proxi_out, 00644, show_proxi_state, store_proxi_state),
		__ATTR(sar_det_3g, 00644, show_sar3g_state, store_sar3g_state),
		__ATTR(3g_available, 00644, show_3g_available, NULL),
		__ATTR_NULL,
};

static int ril_probe(struct platform_device *pdev)
{
	RIL_INFO("RIL Probe\n");
	return 0;
}

static void ril_shutdown(struct platform_device *pdev)
{
	RIL_INFO("RIL Shutdown\n");

	gpio_set_value(GPIO_3G_Reset_PIN, 0);

	mdelay(10);

	gpio_set_value(GPIO_3G_Power_PIN, 0);
}

static void store_gpio(const char *buf, int gpio, char *name)
{
	int state = 0;
	sscanf(buf, "%d", &state);

	if (state != 0)
		state = 1;

	gpio_set_value(gpio, state);
	RIL_INFO("Set %s state to %d\n", name, state);
}

static ssize_t show_power_state(struct device *class, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(GPIO_3G_Power_PIN));
}

static ssize_t store_power_state(struct device *class, struct attribute *attr, const char *buf, size_t count)
{
	store_gpio(buf, GPIO_3G_Power_PIN, "GPIO_3G_Power_PIN");
	return count;
}

static ssize_t show_reset_state(struct device *class, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(GPIO_3G_Reset_PIN));
}

static ssize_t store_reset_state(struct device *class, struct attribute *attr, const char *buf, size_t count)
{
	store_gpio(buf, GPIO_3G_Reset_PIN, "GPIO_3G_Reset_PIN");
	return count;
}

static ssize_t show_proxi_state(struct device *class, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", proxi_out);
}

static ssize_t store_proxi_state(struct device *class, struct attribute *attr, const char *buf, size_t count)
{
	int state = 0;
	sscanf(buf, "%d", &state);
	proxi_out = state;
	ril_request_proxi(state);
	return count;
}

static ssize_t show_sar3g_state(struct device *class, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(GPIO_SAR_DET_3G));
}

static ssize_t store_sar3g_state(struct device *class, struct attribute *attr, const char *buf, size_t count)
{
	store_gpio(buf, GPIO_SAR_DET_3G, "GPIO_SAR_DET_3G");
	return count;
}

static ssize_t show_3g_available(struct device *class, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ASUS3GAvailable());
}

static int add_ril_files(void)
{
	int rc = 0, i = 0;

	rc = alloc_chrdev_region(&ril_dev, ril_minor, 1, "ril");
	ril_major = MAJOR(ril_dev);
	RIL_INFO("rc = %d ril_major = %d\n", rc, ril_major);

	ril_class = class_create(THIS_MODULE, "ril");
	if(ril_class <= 0){
		RIL_ERR("ril_class create fail\n");
		rc = -1;
		goto failed ;
	}
	dev = device_create(ril_class, NULL, MKDEV(ril_major, ril_minor), NULL, "ril_files");
	if(dev <= 0){
		RIL_ERR("dev create fail\n");
		rc = -1;
		goto failed ;
	}

	for(i=0; i<(ARRAY_SIZE(ril_device_attr)-1); i++) {
		rc = device_create_file(dev, &ril_device_attr[i]);
		if (rc) {
			RIL_ERR("create file of [%d] failed, err = %d\n", i, rc);
			goto failed_remove_files;
		}
	}

	RIL_INFO("add_ril_dev success\n");
	return 0;

failed_remove_files:
	while (i--)
		device_remove_file(dev, &ril_device_attr[i]);
failed:
	return rc;
}

void set_wakeup_source(void)
{
	int sim_irq = gpio_to_irq(GPIO_SIM_PIN);
	int mod_irq = gpio_to_irq(GPIO_MODEM_WAKEUP);

	tegra_pm_irq_set_wake_type(sim_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	enable_irq_wake(sim_irq);

	tegra_pm_irq_set_wake_type(mod_irq, IRQF_TRIGGER_FALLING);
	enable_irq_wake(mod_irq);
}

static int init_input_gpio(void)
{
	int rc = 0 ;
	RIL_INFO("init_input_gpio start\n");

	/* request GPIO_SIM_PIN */
	tegra_gpio_enable(GPIO_SIM_PIN);
	rc = gpio_request(GPIO_SIM_PIN, NAME_SIM);
	if (rc) {
		RIL_ERR("gpio_request failed for GPIO_SIM_PIN, rc = %d\n", rc);
		goto sim_pin_request_failed;
	}

	if (is3G) {
		rc = gpio_direction_input(GPIO_SIM_PIN);
		if (rc < 0) {
			RIL_ERR("gpio_direction_input failed for GPIO_SIM_PIN %d\n", GPIO_SIM_PIN);
			goto sim_pin_direction_failed;
		}
	} else {
		rc = gpio_direction_output(GPIO_SIM_PIN, 0);
		if (rc < 0) {
			RIL_ERR("gpio_direction_output failed for GPIO_SIM_PIN %d\n", GPIO_SIM_PIN);
			goto sim_pin_direction_failed;
		}
	}

	/* request GPIO_MODEM_WAKEUP */
	tegra_gpio_enable(GPIO_MODEM_WAKEUP);

	rc = gpio_request(GPIO_MODEM_WAKEUP, "modem_wakeup");
	if (rc) {
		RIL_ERR("gpio_request failed for GPIO_MODEM_WAKEUP, rc = %d\n", rc);
		goto modem_wakeup_request_failed;
	}

	if (is3G) {
		rc = gpio_direction_input(GPIO_MODEM_WAKEUP);
		if (rc < 0) {
			RIL_ERR("gpio_direction_input failed for GPIO_MODEM_WAKEUP\n");
			goto modem_wakeup_direction_failed;
		}
	} else {
		rc = gpio_direction_output(GPIO_MODEM_WAKEUP, 0);
		if (rc < 0) {
			RIL_ERR("gpio_direction_output failed for GPIO_MODEM_WAKEUP\n");
			goto modem_wakeup_direction_failed;
		}
	}

	if (is3G) {
		RIL_INFO("GPIO_SIM_PIN: state = %d\n", gpio_get_value(GPIO_SIM_PIN));
		RIL_INFO("GPIO_MODEM_WAKEUP: state = %d\n", gpio_get_value(GPIO_MODEM_WAKEUP));
	}
	RIL_INFO("init_input_gpio success\n");
	return 0;

modem_wakeup_direction_failed:
	gpio_free(GPIO_MODEM_WAKEUP);
modem_wakeup_request_failed:
sim_pin_direction_failed:
	gpio_free(GPIO_SIM_PIN);
sim_pin_request_failed:
	return rc;
}

static int detect_id(void)
{
	struct file *fv = NULL;
	struct file *fp = NULL;

	fv = filp_open(USB_VID_PATH, O_RDONLY, 0);
	fp = filp_open(USB_PID_PATH, O_RDONLY, 0);

	if (!IS_ERR(fv) && !IS_ERR(fp)) {
		filp_close(fv, NULL);
		filp_close(fp, NULL);
		RIL_INFO("open file success\n");
		return 1;
	}

	if (!IS_ERR(fv))
		filp_close(fv, NULL);
	if (!IS_ERR(fp))
		filp_close(fp, NULL);

	RIL_INFO("open file failed\n");
	return 0;
}

static void detect_modem(void)
{
	modem_detect_count++;
	if (detect_id() > 0) {
		RIL_INFO("USB find the modem\n");
	} else {
		RIL_ERR("USB cannot find the modem, reset GPIO_ULPI_Reset_PIN\n");
		RIL_ERR("modem_detect_count = %d\n", modem_detect_count);
		if (modem_detect_count <= 3) {
			gpio_set_value(GPIO_ULPI_Reset_PIN, 0);
			mdelay(50);
			gpio_set_value(GPIO_ULPI_Reset_PIN, 1);
			schedule_delayed_work(&workq, 4 * HZ);
		} else {
			gpio_set_value(GPIO_3G_Power_PIN, 0);
			gpio_set_value(GPIO_3G_Reset_PIN, 0);
		}
	}
}

static int init_output_gpio(void)
{
	int rc = 0;
	RIL_INFO("init_output_gpio start\n");

	/* request GPIO_3G_Power_PIN to enable 3G power */
	tegra_gpio_enable(GPIO_3G_Power_PIN);
	rc = gpio_request(GPIO_3G_Power_PIN, "3G_power");
	if (rc < 0) {
		RIL_ERR("gpio_request failed for GPIO_3G_Power_PIN, rc = %d\n", rc);
		goto power_request_failed;
	}
	rc = gpio_direction_output(GPIO_3G_Power_PIN, is3G?1:0);
	if (rc < 0) {
		RIL_ERR("gpio_direction_output failed for GPIO_3G_Power_PIN\n");
		goto power_direction_failed;
	}

	mdelay(20);

	/* request GPIO_3G_Reset_PIN to enable 3G Reset */
	tegra_gpio_enable(GPIO_3G_Reset_PIN);
	rc = gpio_request(GPIO_3G_Reset_PIN, "3G_Reset");
	if (rc < 0) {
		RIL_ERR("gpio_request failed for GPIO_3G_Reset_PIN, rc = %d\n", rc);
		goto reset_request_failed;
	}
	rc = gpio_direction_output(GPIO_3G_Reset_PIN, is3G?1:0);
	if (rc < 0) {
		RIL_ERR("gpio_direction_output failed for GPIO_3G_Reset_PIN\n");
		goto reset_direction_failed;
	}

	/* request GPIO_SAR_DET_3G */
	tegra_gpio_enable(GPIO_SAR_DET_3G);
	rc = gpio_request(GPIO_SAR_DET_3G, "sar_det_3G");
	if (rc < 0) {
		RIL_ERR("gpio_request failed for GPIO_SAR_DET_3G, rc = %d\n", rc);
		goto sar_request_failed;
	}
	rc = gpio_direction_output(GPIO_SAR_DET_3G, 0);
	if (rc < 0) {
		RIL_ERR("gpio_direction_output failed for GPIO_SAR_DET_3G\n");
		goto sar_direction_failed;
	}

	if (is3G) {
		/* request GPIO_ULPI_Reset_PIN */
		tegra_gpio_enable(GPIO_ULPI_Reset_PIN);
		rc = gpio_request(GPIO_ULPI_Reset_PIN, "ULPI_Reset");
		if (rc == -16) {
			RIL_INFO("GPIO_ULPI_Reset_PIN has been requested\n");
		} else if (rc < 0) {
			RIL_ERR("gpio_request failed for GPIO_ULPI_Reset_PIN, rc = %d\n", rc);
			goto ulpi_request_failed;
		} else {
			/* enable phy */
			rc = gpio_direction_output(GPIO_ULPI_Reset_PIN, 1);
			if (rc < 0) {
				RIL_ERR("gpio_direction_output failed for GPIO_ULPI_Reset_PIN\n");
				goto ulpi_direction_failed;
			}
		}
		RIL_INFO("GPIO_ULPI_Reset_PIN: state = %d\n", gpio_get_value(GPIO_ULPI_Reset_PIN));

		INIT_DELAYED_WORK(&workq, detect_modem);
		schedule_delayed_work(&workq, 4*HZ);
	}

	RIL_INFO("GPIO_3G_Power_PIN: state = %d\n", gpio_get_value(GPIO_3G_Power_PIN));
	RIL_INFO("GPIO_3G_Reset_PIN: state = %d\n", gpio_get_value(GPIO_3G_Power_PIN));
	RIL_INFO("GPIO_SAR_DET_3G: state = %d\n", gpio_get_value(GPIO_SAR_DET_3G));
	RIL_INFO("init_output_gpio success\n");
	return 0;

ulpi_direction_failed:
	gpio_free(GPIO_ULPI_Reset_PIN);
ulpi_request_failed:
sar_direction_failed:
	gpio_free(GPIO_SAR_DET_3G);
sar_request_failed:
reset_direction_failed:
	gpio_free(GPIO_3G_Reset_PIN);
reset_request_failed:
power_direction_failed:
	gpio_free(GPIO_3G_Power_PIN);
power_request_failed:
	return rc;
}

static void free_gpios(void)
{
	gpio_free(GPIO_3G_Power_PIN);
	gpio_free(GPIO_3G_Reset_PIN);
	gpio_free(GPIO_SAR_DET_3G);
	gpio_free(GPIO_SIM_PIN);
	gpio_free(GPIO_MODEM_WAKEUP);
}

static void free_ril_files(void)
{
	int i;
	for(i=0; i<(ARRAY_SIZE(ril_device_attr)-1); i++) {
		device_remove_file(dev, &ril_device_attr[i]);
	}
	device_destroy(ril_class, MKDEV(ril_major, ril_minor));
	class_destroy(ril_class);
	unregister_chrdev_region(ril_dev, 1);
}

static int __init ril_init(void)
{
	int rc = 0;
	RIL_INFO("asus ril init\n");

	is3G = ASUS3GAvailable();

	rc = platform_device_register(&ril_device);
	if(rc) {
		RIL_ERR("platform_device_register failed\n");
		goto device_failed;
	}

	rc = platform_driver_register(&ril_driver);
	if(rc) {
		RIL_ERR("platform_driver_register failed\n");
		goto driver_failed;
	}

	rc = init_output_gpio();
	if(rc < 0) {
		RIL_INFO("init_output_gpio function fail\n");
		goto failed;
	}

	rc = init_input_gpio();
	if(rc < 0) {
		RIL_INFO("init_input_gpio function fail\n");
		goto failed;
	}

	if (!is3G) {
		RIL_INFO("Wifi-Only version, return!\n");
		return 0;
	}

	set_wakeup_source();

	init_proximity();

	rc = init_sim_hot_plug();
	if(rc < 0) {
		RIL_INFO("init_sim_hot_plug function fail\n");
		goto failed;
	}

	/* create sysfs */
	rc = add_ril_files();
	if(rc < 0)
		goto failed;
	RIL_INFO("ril_init completely\n");
	return 0;

failed:
	platform_driver_unregister(&ril_driver);
driver_failed:
	platform_device_unregister(&ril_device);
device_failed:
	RIL_ERR("ril_init failed\n");
	return rc;
}

static void __exit ril_exit(void)
{
	RIL_INFO("asus ril exit\n");
	if (is3G) {
		free_ril_files();
		free_sim_hot_plug();
		free_proximity();
	}
	free_gpios();
	platform_driver_unregister(&ril_driver);
	platform_device_unregister(&ril_device);
}

module_init(ril_init);
module_exit(ril_exit);

