ifeq ($(CONFIG_SAMSUNG_PRODUCT_SHIP), y)
  obj-$(CONFIG_TOUCHSCREEN_SEC_TS)   += sec_ts.o sec_ts_fw.o sec_ts_fn.o sec_cmd.o
else
  obj-$(CONFIG_TOUCHSCREEN_SEC_TS)   += sec_ts.o sec_ts_fw.o sec_ts_fn.o sec_cmd.o sec_ts_only_vendor.o
endif

KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_SEC_TS=m
ccflags-y	+= -DDYNAMIC_DEBUG_MODULE
ccflags-y	+= -DCONFIG_TOUCHSCREEN_TBN
ccflags-y	+= -DCONFIG_TOUCHSCREEN_HEATMAP
ccflags-y	+= -DCONFIG_TOUCHSCREEN_OFFLOAD
ccflags-y	+= -I$(srctree)/drivers/google/display
ccflags-y	+= -I$(srctree)/drivers/google/touch/common
ccflags-y	+= -I$(srctree)/drivers/google/touch/common/include
