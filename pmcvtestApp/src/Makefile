TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = pmcvtest
# pmcvtest.dbd will be created and installed
DBD += pmcvtest.dbd

# pmcvtest.dbd will be made up from these files:
pmcvtest_DBD += base.dbd

# Include dbd files from all support applications:
pmcvtest_DBD += drvAsynIPPort.dbd motorRecord.dbd motorSupport.dbd asyn.dbd pmcv.dbd asSupport.dbd

# Add all the support libraries needed by this IOC
pmcvtest_LIBS += asyn pmcv motor autosave

# pmcvtest_registerRecordDeviceDriver.cpp derives from pmcvtest.dbd
pmcvtest_SRCS += pmcvtest_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
pmcvtest_SRCS_DEFAULT += pmcvtestMain.cpp
pmcvtest_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#pmcvtest_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
pmcvtest_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

