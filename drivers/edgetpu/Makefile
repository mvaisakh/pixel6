# SPDX-License-Identifier: GPL-2.0
#
# Makefile for EdgeTPU framework and dependent drivers.
#

obj-$(CONFIG_ABROLHOS)		+= abrolhos.o

edgetpu-fw-objs := edgetpu-firmware-util.o edgetpu-shared-fw.o edgetpu-firmware.o
edgetpu-objs	:= edgetpu-core.o edgetpu-mailbox.o edgetpu-kci.o edgetpu-device-group.o edgetpu-telemetry.o edgetpu-mapping.o edgetpu-map-dmabuf.o edgetpu-async.o $(edgetpu-fw-objs)

abrolhos-objs	:= abrolhos-device.o abrolhos-firmware.o edgetpu-direct.o abrolhos-platform.o abrolhos-iommu.o $(edgetpu-objs)

KBUILD_OPTIONS += CONFIG_ABROLHOS=m

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) $(@)
