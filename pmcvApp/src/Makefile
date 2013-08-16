TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += pmcv

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install pmcv.dbd into <top>/dbd
DBD += pmcv.dbd

# specify all source files to be compiled and added to the library
pmcv_SRCS += main.cpp PMCV.cpp 
#pmcv_SRCS += $(MOTOR)/motorApp/MotorSrc/asynMotorController.cpp
#pmcv_SRCS += $(MOTOR)/motorApp/MotorSrc/asynMotorAxis.cpp
pmcv_SRCS += asynMotorController.cpp
pmcv_SRCS += asynMotorAxis.cpp

mc100_LIBS += motor asyn
mc100_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

