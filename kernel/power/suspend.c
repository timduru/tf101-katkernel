/*
 * kernel/power/suspend.c - Suspend to RAM and standby functionality.
 *
 * Copyright (c) 2003 Patrick Mochel
 * Copyright (c) 2003 Open Source Development Lab
 * Copyright (c) 2009 Rafael J. Wysocki <rjw@sisk.pl>, Novell Inc.
 *
 * This file is released under the GPLv2.
 */

#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/cpu.h>
#include <linux/syscalls.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <trace/events/power.h>

#include "power.h"

#include <mach/iomap.h>

#define TIMER_PTV	0x0
#define TIMER_EN	(1 << 31)
#define TIMER_PERIODIC	(1 << 30)

#define TIMER_PCR	0x4
#define TIMER_PCR_INTR	(1 << 30)

#define WDT_EN		(1 << 5)
#define WDT_SEL_TMR1	(0 << 4)
#define WDT_SYS_RST	(1 << 2)


static void __iomem *watchdog_timer  = IO_ADDRESS(TEGRA_TMR1_BASE);
static void __iomem *watchdog_source = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
 void watchdog_enable(int sec)
{
	u32 val;
       printk("watchdog_enable\n");
	val = sec* 1000000ul;
	val |= (TIMER_EN | TIMER_PERIODIC);
	writel(val, watchdog_timer + TIMER_PTV);

	val = WDT_EN | WDT_SEL_TMR1 | WDT_SYS_RST;
	writel(val, watchdog_source);
}

void watchdog_disable(void)
{
       printk("watchdog_disable\n");
	writel(0, watchdog_source);
	writel(0, watchdog_timer + TIMER_PTV);
	writel(TIMER_PCR_INTR, watchdog_timer + TIMER_PCR);
}
static void disable_nonboot_cpus_timeout(unsigned long data)
{
	printk(KERN_EMERG "**** disable_nonboot_cpus_timeout\n");
	watchdog_disable();
	BUG();
}
int suspend_enter_flag=0;
extern struct timer_list suspend_timer;
extern  void suspend_worker_timeout(unsigned long data);

const char *const pm_states[PM_SUSPEND_MAX] = {
#ifdef CONFIG_EARLYSUSPEND
	[PM_SUSPEND_ON]		= "on",
#endif
	[PM_SUSPEND_STANDBY]	= "standby",
	[PM_SUSPEND_MEM]	= "mem",
};

static const struct platform_suspend_ops *suspend_ops;

/**
 *	suspend_set_ops - Set the global suspend method table.
 *	@ops:	Pointer to ops structure.
 */
void suspend_set_ops(const struct platform_suspend_ops *ops)
{
	mutex_lock(&pm_mutex);
	suspend_ops = ops;
	mutex_unlock(&pm_mutex);
}

bool valid_state(suspend_state_t state)
{
	/*
	 * All states need lowlevel support and need to be valid to the lowlevel
	 * implementation, no valid callback implies that none are valid.
	 */
	return suspend_ops && suspend_ops->valid && suspend_ops->valid(state);
}

/**
 * suspend_valid_only_mem - generic memory-only valid callback
 *
 * Platform drivers that implement mem suspend only and only need
 * to check for that in their .valid callback can use this instead
 * of rolling their own .valid callback.
 */
int suspend_valid_only_mem(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}

static int suspend_test(int level)
{
#ifdef CONFIG_PM_DEBUG
	if (pm_test_level == level) {
		printk(KERN_INFO "suspend debug: Waiting for 5 seconds.\n");
		mdelay(5000);
		return 1;
	}
#endif /* !CONFIG_PM_DEBUG */
	return 0;
}

/**
 *	suspend_prepare - Do prep work before entering low-power state.
 *
 *	This is common code that is called for each state that we're entering.
 *	Run suspend notifiers, allocate a console and stop all processes.
 */
extern int pm_notifier_call_chain2(unsigned long val);
static int suspend_prepare(void)
{
	int error;

	if (!suspend_ops || !suspend_ops->enter)
		return -EPERM;

	pm_prepare_console();
	error = pm_notifier_call_chain2(PM_SUSPEND_PREPARE);
	if (error){
		printk("suspend_prepare fail error=%d\n",error);
		goto Finish;
	}
	error = usermodehelper_disable();
	if (error)
		goto Finish;
	watchdog_disable();
	del_timer_sync(&suspend_timer);
	destroy_timer_on_stack(&suspend_timer);
	init_timer_on_stack(&suspend_timer);
	suspend_timer.expires = jiffies + HZ * 42;
	suspend_timer.function = suspend_worker_timeout;
	add_timer(&suspend_timer);
	watchdog_enable(44);

	error = suspend_freeze_processes();
	watchdog_disable();
	del_timer_sync(&suspend_timer);
	destroy_timer_on_stack(&suspend_timer);
	init_timer_on_stack(&suspend_timer);
	suspend_timer.expires = jiffies + HZ * 9;
	suspend_timer.function = suspend_worker_timeout;
	add_timer(&suspend_timer);
	watchdog_enable(11);

	if (!error)
		return 0;

	suspend_thaw_processes();
	usermodehelper_enable();
 Finish:
	pm_notifier_call_chain2(PM_POST_SUSPEND);
	pm_restore_console();
	return error;
}

/* default implementation */
void __attribute__ ((weak)) arch_suspend_disable_irqs(void)
{
	local_irq_disable();
}

/* default implementation */
void __attribute__ ((weak)) arch_suspend_enable_irqs(void)
{
	local_irq_enable();
}

/**
 *	suspend_enter - enter the desired system sleep state.
 *	@state:		state to enter
 *
 *	This function should be called after devices have been suspended.
 */
static int suspend_enter(suspend_state_t state)
{
	int error;
	struct timer_list timer;
	init_timer_on_stack(&timer);
	timer.expires = jiffies + HZ * 6;
	timer.function = disable_nonboot_cpus_timeout;

	if (suspend_ops->prepare) {
		error = suspend_ops->prepare();
		if (error)
			goto Platform_finish;
	}

	error = dpm_suspend_noirq(PMSG_SUSPEND);
	if (error) {
		printk(KERN_ERR "PM: Some devices failed to power down\n");
		goto Platform_finish;
	}

	if (suspend_ops->prepare_late) {
		error = suspend_ops->prepare_late();
		if (error)
			goto Platform_wake;
	}

	if (suspend_test(TEST_PLATFORM))
		goto Platform_wake;
	add_timer(&timer);
	suspend_enter_flag=1;
	error = disable_nonboot_cpus();
	suspend_enter_flag=0;
	del_timer_sync(&timer);
	destroy_timer_on_stack(&timer);
	if (error || suspend_test(TEST_CPUS))
		goto Enable_cpus;

	arch_suspend_disable_irqs();
	BUG_ON(!irqs_disabled());

	error = sysdev_suspend(PMSG_SUSPEND);
	if (!error) {
		error = syscore_suspend();
		if (error)
			sysdev_resume();
	}
	if (!error) {
		if (!(suspend_test(TEST_CORE) || pm_wakeup_pending())) {
			error = suspend_ops->enter(state);
			events_check_enabled = false;
		}
		syscore_resume();
		sysdev_resume();
	}

	arch_suspend_enable_irqs();
	BUG_ON(irqs_disabled());

 Enable_cpus:
	enable_nonboot_cpus();

 Platform_wake:
	if (suspend_ops->wake)
		suspend_ops->wake();

	dpm_resume_noirq(PMSG_RESUME);

 Platform_finish:
	if (suspend_ops->finish)
		suspend_ops->finish();

	return error;
}

/**
 *	suspend_devices_and_enter - suspend devices and enter the desired system
 *				    sleep state.
 *	@state:		  state to enter
 */
int suspend_devices_and_enter(suspend_state_t state)
{
	int error;

	if (!suspend_ops)
		return -ENOSYS;

	trace_machine_suspend(state);
	if (suspend_ops->begin) {
		error = suspend_ops->begin(state);
		if (error)
			goto Close;
	}
	suspend_console();
	suspend_test_start();
	error = dpm_suspend_start(PMSG_SUSPEND);
	if (error) {
		printk(KERN_ERR "PM: Some devices failed to suspend\n");
		goto Recover_platform;
	}
	suspend_test_finish("suspend devices");
	if (suspend_test(TEST_DEVICES))
		goto Recover_platform;

	suspend_enter(state);

 Resume_devices:
	suspend_test_start();
	dpm_resume_end(PMSG_RESUME);
	suspend_test_finish("resume devices");
        printk("PM: resume_console+\n");
	resume_console();
	printk("PM: resume_console-\n");
 Close:
	if (suspend_ops->end)
		suspend_ops->end();
	trace_machine_suspend(PWR_EVENT_EXIT);
	return error;

 Recover_platform:
	if (suspend_ops->recover)
		suspend_ops->recover();
	goto Resume_devices;
}

/**
 *	suspend_finish - Do final work before exiting suspend sequence.
 *
 *	Call platform code to clean up, restart processes, and free the
 *	console that we've allocated. This is not called for suspend-to-disk.
 */
static void suspend_finish(void)
{
	printk("suspend_finish+\n");
	suspend_thaw_processes();
	usermodehelper_enable();
	pm_notifier_call_chain2(PM_POST_SUSPEND);
	pm_restore_console();
	printk("suspend_finish-\n");
}

/**
 *	enter_state - Do common work of entering low-power state.
 *	@state:		pm_state structure for state we're entering.
 *
 *	Make sure we're the only ones trying to enter a sleep state. Fail
 *	if someone has beat us to it, since we don't want anything weird to
 *	happen when we wake up.
 *	Then, do the setup for suspend, enter the state, and cleaup (after
 *	we've woken up).
 */
int enter_state(suspend_state_t state)
{
	int error;

	if (!valid_state(state))
		return -ENODEV;

	if (!mutex_trylock(&pm_mutex))
		return -EBUSY;

	printk(KERN_INFO "PM: Syncing filesystems ... ");
	sys_sync();
	printk("done.\n");

	pr_debug("PM: Preparing system for %s sleep\n", pm_states[state]);
	error = suspend_prepare();
	if (error)
		goto Unlock;

	if (suspend_test(TEST_FREEZER))
		goto Finish;

	pr_debug("PM: Entering %s sleep\n", pm_states[state]);
	pm_restrict_gfp_mask();
	error = suspend_devices_and_enter(state);
	pm_restore_gfp_mask();

 Finish:
	pr_debug("PM: Finishing wakeup.\n");
	suspend_finish();
 Unlock:
	mutex_unlock(&pm_mutex);
	return error;
}

/**
 *	pm_suspend - Externally visible function for suspending system.
 *	@state:		Enumerated value of state to enter.
 *
 *	Determine whether or not value is within range, get state
 *	structure, and enter (above).
 */
int pm_suspend(suspend_state_t state)
{
	if (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX)
		return enter_state(state);
	return -EINVAL;
}
EXPORT_SYMBOL(pm_suspend);
