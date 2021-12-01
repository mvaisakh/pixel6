KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_FTS=m
ccflags-y	+= -DDYNAMIC_DEBUG_MODULE
ccflags-y	+= -DCONFIG_TOUCHSCREEN_TBN
ccflags-y	+= -DCONFIG_TOUCHSCREEN_HEATMAP
ccflags-y	+= -DCONFIG_TOUCHSCREEN_OFFLOAD
ccflags-y	+= -I$(srctree)/drivers/google/display
ccflags-y	+= -I$(srctree)/drivers/google/touch/common
ccflags-y	+= -I$(srctree)/drivers/google/touch/common/include

obj-$(CONFIG_TOUCHSCREEN_FTS)   += ftm5.o
ftm5-objs = fts.o fts_proc.o
ftm5-objs += \
        fts_lib/ftsCompensation.o fts_lib/ftsCore.o fts_lib/ftsError.o \
        fts_lib/ftsFrame.o fts_lib/ftsIO.o fts_lib/ftsTest.o fts_lib/ftsTime.o \
        fts_lib/ftsTool.o fts_lib/ftsFlash.o fts_lib/ftsGesture.o
