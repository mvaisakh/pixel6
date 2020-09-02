obj-$(CONFIG_GOOGLE_BMS)	+= google-bms.o
google-bms-objs += gbms_storage.o google_bms.o google_eeprom.o
obj-$(CONFIG_GOOGLE_CPM)	+= google_cpm.o
obj-$(CONFIG_GOOGLE_CHARGER)	+= google-charger.o
google-charger-objs += google_charger.o google_dc_pps.o
obj-$(CONFIG_GOOGLE_BATTERY)	+= google-battery.o
google-battery-objs += google_battery.o google_ttf.o
obj-$(CONFIG_USB_OVERHEAT_MITIGATION)	+= overheat_mitigation.o
obj-$(CONFIG_CHARGER_P9221)	+= p9221_charger.o p9221_chip.o
obj-$(CONFIG_CHARGER_MAX77729)	+= max77729_charger.o
obj-$(CONFIG_CHARGER_MAX77759)	+= max77759_charger.o
obj-$(CONFIG_PMIC_MAX77729)	+= max77729_pmic.o
obj-$(CONFIG_MAXQ_MAX77759)	+= max77759_maxq.o
obj-$(CONFIG_UIC_MAX77729)	+= max77729_uic.o
obj-$(CONFIG_BATTERY_MAX1720X)  += max1720x-battery.o
max1720x-battery-objs += max1720x_battery.o max_m5.o
obj-$(CONFIG_PCA9468)	+= pca9468_charger.o
obj-$(CONFIG_MAX20339)		+= max20339.o
obj-$(CONFIG_PMIC_VOTER_COMPAT)	+= pmic-voter-compat.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += CONFIG_GOOGLE_BMS=m \
		  CONFIG_GOOGLE_CPM=m \
		  CONFIG_GOOGLE_CHARGER=m \
		  CONFIG_GOOGLE_BATTERY=m \
		  CONFIG_GOOGLE_BEE=m \
		  CONFIG_USB_OVERHEAT_MITIGATION=m \
		  CONFIG_PMIC_VOTER_COMPAT=m \
		  CONFIG_CHARGER_P9221=m \
		  CONFIG_CHARGER_MAX77729=m \
		  CONFIG_CHARGER_MAX77759=m \
		  CONFIG_PMIC_MAX77729=m \
		  CONFIG_MAXQ_MAX77759=m \
		  CONFIG_UIC_MAX77729=m \
		  CONFIG_BATTERY_MAX1720X=m \
		  CONFIG_PCA9468=m \
		  CONFIG_MAX20339=m

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) $(@)
