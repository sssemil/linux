EXTRA_CFLAGS 	+= -Idrivers/hisi/mntn/blackbox/platform_lpm3
#LOCAL_C_INCLUDES += vendor/hisi/ap/libc_sec/include
#LOCAL_CFLAGS += -Ivendor/hisi/ap/libc_sec/include

subdir-ccflags-y              += -Wall -Werror

ifeq ($(TARGET_BOARD_PLATFORM), hi3660)
EXTRA_CFLAGS += -DHISEE_POWER_MANAGE_COUNT
endif

ifeq ($(TARGET_BOARD_PLATFORM), kirin970)
EXTRA_CFLAGS += -DHISEE_POWER_MANAGE_PRO
endif

ifeq ($(chip_type), cs)
EXTRA_CFLAGS += -DHISEE_CHIPTEST_SUPPORT_CASDKEY
endif

obj-$(CONFIG_HISI_HISEE)  += hisi_hisee.o
obj-$(CONFIG_HISI_HISEE)  += hisi_hisee_power.o
obj-$(CONFIG_HISI_HISEE)  += hisi_hisee_upgrade.o
obj-$(CONFIG_HISI_HISEE)  += hisi_hisee_chip_test.o
obj-$(CONFIG_HISI_HISEE)  += hisi_hisee_fs.o
obj-$(CONFIG_HISI_HISEE)  += hisi_hisee_autotest.o
