#ifndef _RIL_H
#define _RIL_H

#include "../../arch/arm/mach-tegra/gpio-names.h"

/* DEBUG */
#define RIL_DEBUG 1

#if RIL_DEBUG
#define RIL_INFO(format, arg...) \
	printk(KERN_INFO "RIL: [%s] " format , __FUNCTION__ , ## arg)
#else
#define RIL_INFO(format, arg...)
#endif

#define RIL_ERR(format, arg...) \
	printk(KERN_ERR "RIL: [%s] " format , __FUNCTION__ , ## arg)

/* GPIO NUM */
#define GPIO_3G_Power_PIN    TEGRA_GPIO_PR7
#define GPIO_Enable_RF_PIN   TEGRA_GPIO_PD1
#define GPIO_3G_Reset_PIN    TEGRA_GPIO_PD0
#define GPIO_SAR_DET_3G      TEGRA_GPIO_PG1
#define GPIO_SIM_PIN         TEGRA_GPIO_PC7
#define GPIO_MODEM_WAKEUP    TEGRA_GPIO_PQ6
#define GPIO_ULPI_Reset_PIN  TEGRA_GPIO_PV1

#define USB_VID_PATH         "/sys/devices/platform/tegra-ehci.1/usb1/1-1/idVendor"
#define USB_PID_PATH         "/sys/devices/platform/tegra-ehci.1/usb1/1-1/idProduct"

#define NAME_SIM "sim_plug"

#endif
