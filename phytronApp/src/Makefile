TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
# Build the IOC application


LIBRARY_IOC += phytronAxisMotor

DBD += phytronSupport.dbd

# The following are compiled and added to the support library
phytronAxisMotor_SRCS += phytronAxisMotor.cpp phytronIoCtrl.cpp

INC += phytronAxisMotor.h phytronIoCtrl.h

phytronAxisMotor_LIBS += motor
phytronAxisMotor_LIBS += asyn
phytronAxisMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

# Test with PHYIOC
PROD_IOC = PHYIOC
PHYIOC_SRCS += PHYIOC_registerRecordDeviceDriver.cpp SoftIocMain.cpp

PHYIOC_LIBS += motor
PHYIOC_LIBS += asyn 
PHYIOC_LIBS += phytronAxisMotor 
PHYIOC_LIBS += $(EPICS_BASE_IOC_LIBS)

PHYIOC_DBD += base.dbd asyn.dbd motorSupport.dbd
PHYIOC_DBD += drvAsynIPPort.dbd phytronSupport.dbd

DBD += PHYIOC.dbd

#===========================

include $(TOP)/configure/RULES

