#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/switch.h>

#include "ril.h"
#include "sim.h"

/* struct declaration */
static struct delayed_work workq;
static struct switch_dev sim_sdev;
static int sim_state;

static ssize_t print_sim_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", NAME_SIM);
}

static ssize_t print_sim_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", sim_state);
}

static irqreturn_t sim_interrupt_handler(int irq, void *dev_id)
{
	schedule_delayed_work(&workq, 0.1 * HZ);
	return IRQ_HANDLED;
}

static void sim_irq_work(void)
{
	sim_state = gpio_get_value(GPIO_SIM_PIN);

	RIL_INFO("sim state = %d\n", sim_state);
	switch_set_state(&sim_sdev, sim_state);
}
int init_sim_hot_plug(void)
{
	int rc = 0;
	int sim_irq = gpio_to_irq(GPIO_SIM_PIN);
	sim_state = gpio_get_value(GPIO_SIM_PIN);

	RIL_INFO("GPIO = %d , irq = %d, state = %d\n", GPIO_SIM_PIN, sim_irq, sim_state);

	rc = request_irq(sim_irq, sim_interrupt_handler,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, NAME_SIM, NULL);
	if (rc < 0) {
		RIL_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", NAME_SIM, sim_irq, rc);
		rc = -EIO;
		goto failed;
	}

	/* init work queue */
	INIT_DELAYED_WORK(&workq, sim_irq_work);

	/* register switch class*/
	sim_sdev.name = NAME_SIM;
	sim_sdev.print_name = print_sim_name;
	sim_sdev.print_state = print_sim_state;
	rc = switch_dev_register(&sim_sdev);
	/*
	 * Because switch_dev_register will initiate sdev.state to 0,
	 * sdev.state will be initiated after switch_dev_register.
	 */
	sim_sdev.state = sim_state;

	if (rc < 0) {
		RIL_ERR("Could not register switch device, rc = %d\n", rc);
		goto failed;
	}

	RIL_INFO("request irq and switch class successfully\n");
	return 0;

failed:
	gpio_free(GPIO_SIM_PIN);
	return rc;
}

void free_sim_hot_plug(void)
{
	switch_dev_unregister(&sim_sdev);
}

