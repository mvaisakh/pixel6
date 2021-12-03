obj-$(CONFIG_TOUCHSCREEN_SEC_TS)   += sec_touch.o
sec_touch-objs   += sec_ts.o sec_ts_fw.o sec_ts_fn.o sec_cmd.o \
	sec_ts_only_vendor.o

KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_SEC_TS=m
EXTRA_CFLAGS	+= -DDYNAMIC_DEBUG_MODULE
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_TBN
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_HEATMAP
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_OFFLOAD
EXTRA_CFLAGS	+= -I$(srctree)/drivers/google/display
EXTRA_CFLAGS	+= -I$(srctree)/drivers/google/touch/common
EXTRA_CFLAGS	+= -I$(srctree)/drivers/google/touch/common/include
