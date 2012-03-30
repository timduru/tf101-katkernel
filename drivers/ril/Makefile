obj-$(CONFIG_RIL)	+= ril.o
obj-$(CONFIG_RIL)	+= proximity.o
obj-$(CONFIG_RIL)	+= sim.o

ifeq ($(CONFIG_SENSORS_PROX_LDS6202), y)
EXTRA_CFLAGS = -DCONFIG_PROX
endif