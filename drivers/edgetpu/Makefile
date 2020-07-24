# SPDX-License-Identifier: GPL-2.0
#
# Makefile for EdgeTPU framework and dependent drivers.
#

obj-$(CONFIG_ABROLHOS)		+= abrolhos.o

GIT_PATH=$(srctree)
ifeq ($(shell git --git-dir=$(GIT_PATH)/.git rev-parse --is-inside-work-tree),true)
	GIT_REPO_STATE=$(shell (git --git-dir=$(GIT_PATH)/.git --work-tree=$(GIT_PATH) status --porcelain | grep -q .) && echo -dirty)
	ccflags-y       += -DGIT_REPO_TAG=\"$(shell git --git-dir=$(GIT_PATH)/.git rev-parse --short HEAD)$(GIT_REPO_STATE)\"
else
	ccflags-y       += -DGIT_REPO_TAG=\"Not\ a\ git\ repository\"
endif

edgetpu-fw-objs := edgetpu-firmware-util.o edgetpu-shared-fw.o edgetpu-firmware.o
edgetpu-objs	:= edgetpu-core.o edgetpu-mailbox.o edgetpu-kci.o edgetpu-device-group.o edgetpu-telemetry.o edgetpu-mapping.o edgetpu-dmabuf.o edgetpu-async.o edgetpu-pm.o $(edgetpu-fw-objs)

abrolhos-objs	:= abrolhos-device.o abrolhos-firmware.o edgetpu-direct.o abrolhos-platform.o abrolhos-iommu.o edgetpu-thermal.o abrolhos-pm.o $(edgetpu-objs)

KBUILD_OPTIONS += CONFIG_ABROLHOS=m

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) $(@)
