lwis-objs := lwis_device.o

obj-$(CONFIG_LWIS) += lwis.o

subdir-ccflags-$(CONFIG_LWIS) += -Idrivers/media/platform/google/lwis
