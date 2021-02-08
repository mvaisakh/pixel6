obj-$(CONFIG_TOUCHSCREEN_TBN)		+= touch_bus_negotiator.o
obj-$(CONFIG_TOUCHSCREEN_HEATMAP)	+= heatmap.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_TBN=m
KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_HEATMAP=m
EXTRA_CFLAGS	+= -DDYNAMIC_DEBUG_MODULE

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	$(KBUILD_OPTIONS) \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	$(@)
