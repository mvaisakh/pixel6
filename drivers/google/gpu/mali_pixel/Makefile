# SPDX-License-Identifier: GPL-2.0
#
# Copyright 2021 Google LLC
#

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += CONFIG_MALI_MEMORY_GROUP_MANAGER=y
KBUILD_OPTIONS += CONFIG_MALI_PRIORITY_CONTROL_MANAGER=y

KBUILD_OPTIONS += $(KBUILD_EXTRA) # Extra config if any

src:=$(if $(patsubst /%,,$(src)),$(srctree)/$(src),$(src))

CONFIG_MALI_MEMORY_GROUP_MANAGER ?= y
CONFIG_MALI_PRIORITY_CONTROL_MANAGER ?= y

DEFINES += \
	-DCONFIG_MALI_MEMORY_GROUP_MANAGER=$(CONFIG_MALI_MEMORY_GROUP_MANAGER) \
	-DCONFIG_MALI_PRIORITY_CONTROL_MANAGER=$(CONFIG_MALI_PRIORITY_CONTROL_MANAGER)

# Use our defines when compiling, and include mali platform module headers
ccflags-y += $(DEFINES) -I$(src)/../common/include

mali_pixel-objs :=
ifeq ($(CONFIG_MALI_MEMORY_GROUP_MANAGER),m)
	mali_pixel-objs += memory_group_manager.o
endif
ifeq ($(CONFIG_MALI_PRIORITY_CONTROL_MANAGER),m)
	mali_pixel-objs += priority_control_manager.o
endif

# Add kernel module target if any of our config options is enabled
ifneq ($(mali_pixel-objs),)
	obj-y += mali_pixel.o
	mali_pixel-objs += mali_pixel_mod.o
endif

