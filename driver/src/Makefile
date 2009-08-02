TOP := ../..
include $(TOP)/configure/CONFIG

# Note: cannot build for 64bits because we don't have the Acqiris driver
ifeq ($(T_A),linux-x86)
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

ifndef ACQIRISDIR
ACQIRISDIR := $(EPICS_TOP)/../acqiris
endif
ACQIRIS_INCLUDE := $(ACQIRISDIR)/usr/include
ACQIRIS_LIB := $(ACQIRISDIR)/usr/lib
USR_CFLAGS += -I$(ACQIRIS_INCLUDE) -D_ACQIRIS -D_LINUX
USR_CPPFLAGS += -I$(ACQIRIS_INCLUDE) -D_ACQIRIS -D_LINUX
acqiris_support_LDFLAGS += -L$(ACQIRIS_LIB) -lAqDrv4

include $(TOP)/configure/RULES
