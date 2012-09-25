#include <linux/module.h>
#include <linux/switch.h>

#include "ril.h"
#include "proximity.h"

static struct switch_dev proxi_sdev;
static int proxi_in = 0;

static ssize_t print_proxi_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", "proxi_in");
}

static ssize_t print_proxi_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", proxi_in);
}

void ril_request_proxi(int state)
{
#ifdef CONFIG_PROX
	//should call proximity function
	RIL_INFO("state = %d\n", state);
	if (state) {
		prox_lds6202_enable();
	} else {
		prox_lds6202_disable();
	}
#else
	RIL_INFO("proximity is not configured\n");
#endif
}

void proxi_request_ril(int state)
{
	//be called by proximity
	RIL_INFO("state = %d\n", state);

	if (state != 0)
		state = 1;

	proxi_in = state;

	switch_set_state(&proxi_sdev, proxi_in);
}
EXPORT_SYMBOL(proxi_request_ril);

int init_proximity(void)
{
	int rc = 0;
	/* register switch class*/
	proxi_sdev.name = "proximity";
	proxi_sdev.print_name = print_proxi_name;
	proxi_sdev.print_state = print_proxi_state;
	rc = switch_dev_register(&proxi_sdev);

	if (rc < 0) {
		RIL_ERR("Could not register switch device, rc = %d\n", rc);
		goto failed;
	}

	RIL_INFO("register switch class successfully\n");
	return 0;

failed:
	return rc;
}

void free_proximity(void)
{
	switch_dev_unregister(&proxi_sdev);
}

