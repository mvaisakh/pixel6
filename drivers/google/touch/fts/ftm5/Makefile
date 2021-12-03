# FTM5 support
obj-$(CONFIG_TOUCHSCREEN_FTS)	+= ftm5.o
ftm5-objs = fts.o fts_proc.o
ftm5-objs += \
	fts_lib/ftsCompensation.o fts_lib/ftsCore.o fts_lib/ftsError.o \
	fts_lib/ftsFrame.o fts_lib/ftsIO.o fts_lib/ftsTest.o fts_lib/ftsTime.o \
	fts_lib/ftsTool.o fts_lib/ftsFlash.o fts_lib/ftsGesture.o

KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_FTS=m
EXTRA_CFLAGS	+= -DDYNAMIC_DEBUG_MODULE
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_TBN
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_HEATMAP
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_OFFLOAD
EXTRA_CFLAGS	+= -I$(srctree)/drivers/google/display
EXTRA_CFLAGS	+= -I$(srctree)/drivers/google/touch/common
EXTRA_CFLAGS	+= -I$(srctree)/drivers/google/touch/common/include
