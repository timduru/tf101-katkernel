
# Kernel makefile for motions sensors
#
#

# MPU
obj-$(CONFIG_MPU_SENSORS_MPU3050)	+= mpu3050.o
mpu3050-objs += mpuirq.o \
	slaveirq.o \
	mpu-dev.o \
	mpu-i2c.o \
	mlsl-kernel.o \
	mlos-kernel.o \
	$(MLLITE_DIR)mldl_cfg.o

#
# Accel options
#
ifdef CONFIG_MPU_SENSORS_KXTF9
mpu3050-objs += $(MLLITE_DIR)accel/kxtf9.o
endif

#
# Compass options
#
ifdef CONFIG_MPU_SENSORS_AK8975
mpu3050-objs += $(MLLITE_DIR)compass/ak8975.o
endif

ifdef CONFIG_MPU_SENSORS_AMI304
mpu3050-objs += $(MLLITE_DIR)compass/ami30x.o
endif

#
# Pressure options
#
ifdef CONFIG_MPU_SENSORS_BMA085
mpu3050-objs += $(MLLITE_DIR)pressure/bma085.o
endif

EXTRA_CFLAGS += -I$(M)/$(MLLITE_DIR) \
                -I$(M)/../../include \
		-Idrivers/misc/mpu3050 \
                -Iinclude/linux

obj-$(CONFIG_MPU_SENSORS_MPU6000)+= mpu6000.o
mpu6000-objs += mpuirq.o \
	slaveirq.o \
	mpu-dev.o \
	mpu-i2c.o \
	mlsl-kernel.o \
	mlos-kernel.o \
	$(MLLITE_DIR)mldl_cfg.o \
	$(MLLITE_DIR)accel/mantis.o

ifdef CONFIG_MPU_SENSORS_AK8975
mpu6000-objs += $(MLLITE_DIR)compass/ak8975.o
endif

ifdef CONFIG_MPU_SENSORS_MPU6000
EXTRA_CFLAGS += -DM_HW
endif

obj-$(CONFIG_MPU_SENSORS_TIMERIRQ)+= timerirq.o

ifdef CONFIG_MPU_SENSORS_DEBUG
EXTRA_CFLAGS += -DDEBUG
endif

