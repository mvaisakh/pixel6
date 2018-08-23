lwis-objs := lwis_device.o
lwis-objs += lwis_clock.o
lwis-objs += lwis_dt.o
lwis-objs += lwis_gpio.o
lwis-objs += lwis_regulator.o
lwis-objs += sensors/lwis_sensor.o
lwis-objs += sensors/lwis_sensor_imx362.o

obj-$(CONFIG_LWIS) += lwis.o

subdir-ccflags-$(CONFIG_LWIS) += -Idrivers/media/platform/google/lwis
subdir-ccflags-$(CONFIG_LWIS) += -Idrivers/media/platform/google/lwis/sensors
