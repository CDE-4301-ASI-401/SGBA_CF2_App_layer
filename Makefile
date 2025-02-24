CRAZYFLIE_BASE := $(PWD)/crazyflie-firmware
OOT_CONFIG := $(CRAZYFLIE_BASE)/build/.config
include $(CRAZYFLIE_BASE)/tools/make/oot.mk

EXTRA_CFLAGS += -Wno-unused-variable -Wno-unused-function
