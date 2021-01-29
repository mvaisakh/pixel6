KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += CONFIG_TOUCHSCREEN_FTS=m
EXTRA_CFLAGS=-I$(KERNEL_SRC)/../google-modules/display

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" $(@)
