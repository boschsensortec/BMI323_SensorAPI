COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= mcu_alternate_config_tilt_orient.c

API_LOCATION ?= ../..

COMMON_LOCATION ?= ..

C_SRCS += \
$(API_LOCATION)/bmi3.c \
$(API_LOCATION)/bmi323.c \
$(COMMON_LOCATION)/common/common.c

INCLUDEPATHS += \
$(API_LOCATION) \
$(COMMON_LOCATION)/common

TARGET = MCU_APP20

include $(COINES_INSTALL_PATH)/coines.mk