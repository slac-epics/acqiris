TOP := ../..
include $(TOP)/configure/CONFIG

ifeq ($(T_A),linux-x86_64)
ACQIRIS_PKG_DIR := /reg/g/pcds/package/external/acqiris_4.2a
ACQIRIS_PKG_INCLUDE := $(ACQIRIS_PKG_DIR)/include
ACQIRIS_PKG_LIB := $(ACQIRIS_PKG_DIR)/lib64
ACQIRIS_KVERSION := 0x20000

LIBRARY := acqiris_support
DBD += acqiris_support.dbd
endif

ifeq ($(T_A),linux-x86)
ACQIRIS_PKG_DIR := /reg/g/pcds/package/external/acqiris_3.3a
ACQIRIS_PKG_INCLUDE := $(ACQIRIS_PKG_DIR)/include
ACQIRIS_PKG_LIB := $(ACQIRIS_PKG_DIR)/lib
ACQIRIS_KVERSION := 0x10006

LIBRARY := acqiris_support
DBD += acqiris_support.dbd
endif

LIBSRCS += acqiris_dev_li.cpp
LIBSRCS += acqiris_dev_lo.cpp
LIBSRCS += acqiris_dev_ai.cpp
LIBSRCS += acqiris_dev_ao.cpp
LIBSRCS += acqiris_dev_mbbo.cpp
LIBSRCS += acqiris_dev_wf.cpp
LIBSRCS += acqiris_dev_init.cpp
LIBSRCS += acqiris_drv_li.cpp
LIBSRCS += acqiris_drv_lo.cpp
LIBSRCS += acqiris_drv_ai.cpp
LIBSRCS += acqiris_drv_ao.cpp
LIBSRCS += acqiris_drv_mbbo.cpp
LIBSRCS += acqiris_drv_wf.cpp
LIBSRCS += acqiris_daq.cpp
LIBSRCS += acqiris_drv_init.cpp

USR_CFLAGS += -I$(ACQIRIS_PKG_INCLUDE) -D_ACQIRIS -D_LINUX -DACQIRIS_KVERSION=$(ACQIRIS_KVERSION)
USR_CPPFLAGS += -I$(ACQIRIS_PKG_INCLUDE) -D_ACQIRIS -D_LINUX -DACQIRIS_KVERSION=$(ACQIRIS_KVERSION)
acqiris_support_LDFLAGS += -L$(ACQIRIS_PKG_LIB) -lAqDrv4

include $(TOP)/configure/RULES
