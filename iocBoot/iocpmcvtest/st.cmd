#!../../bin/linux-x86/pmcvtest

< envPaths

## Register all support components
dbLoadDatabase("../../dbd/pmcvtest.dbd",0,0)
pmcvtest_registerRecordDeviceDriver(pdbbase) 

# Get the PMCV port from the environment variable settings (see st.sh)
epicsEnvSet("PMCV_IP" "$(PMCV_IP=10.0.0.21)")
epicsEnvSet("PMCV_TCP_PORT" "$(PMCV_TCP_PORT=1024)")
epicsEnvSet("P" "$(P=MLL:)")
epicsEnvSet("R" "$(R=PMCV:)")
epicsEnvSet("PMCVPORT" "PMCV")
epicsEnvSet("AS_PREFIX", "$(P)$(R)")
epicsEnvShow "PMCV_IP"
epicsEnvShow "PMCV_TCP_PORT"

# tcp/ip option:
drvAsynIPPortConfigure("IP1" ,"$(PMCV_IP):$(PMCV_TCP_PORT)",0,0,0)

# or the serial option:
#drvAsynSerialPortConfigure("IP1", "/dev/ttyS0", 0, 0, 0)
#asynSetOption("IP1", -1,"baud",xx)
#asynSetOption("IP1", -1,"bits",8)
#asynSetOption("IP1", -1,"parity","none")
#asynSetOption("IP1", -1,"stop",1)

#PMCVCreateController(portName, PMCVPortName, numAxes, movingPollPeriod, idlePollPeriod)
PMCVCreateController("$(PMCVPORT)", "IP1", 3, 20, 100)

#asynSetTraceMask("$(PMCVPORT)", -1, 0x3)
#asynSetTraceMask("$(PMCVPORT)", -1, 0xFF)
#asynSetTraceMask("IP1", -1, 0xFF)

##asynSetTraceIOMask("$(PMCVPORT)", -1, 255)
##asynSetTraceIOMask("IP1", -1, 255)

dbLoadRecords("$(TOP)/db/pmcv_general.db", "P=$(P),R=$(R),PORT=$(PMCVPORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(TOP)/db/pmcv_axis.db", "P=$(P),R=$(R)axis1:,PORT=$(PMCVPORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(TOP)/db/pmcv_axis.db", "P=$(P),R=$(R)axis2:,PORT=$(PMCVPORT),ADDR=1,TIMEOUT=1")
dbLoadRecords("$(TOP)/db/pmcv_axis.db", "P=$(P),R=$(R)axis3:,PORT=$(PMCVPORT),ADDR=2,TIMEOUT=1")

dbLoadTemplate("pmcv.sub")

cd $(TOP)/iocBoot/$(IOC)
< save_restore.cmd 

iocInit()

create_monitor_set("auto_positions.req", 5, "P=$(AS_PREFIX)")
create_monitor_set("auto_settings.req", 30, "P=$(AS_PREFIX)")

# PMCVsetEncMode(portName axis spi_mode quad_mode frequency swap_ab noise_filter)
# spi_mode             SPI mode off (0) on (1)
# quad_mode            Quad mode off (0) on (1)
# frequency            110KHz (0) 440KHz (1) 1.8MHz (2) 7MHz (3)
# swap_ab              Normal quadrature signal (0) swap A/B (1)
# noise_filter         Enable digital noise filter (1) disable (0)
# SPI off, quad on, 7MHz, swap A/B quad, enable noise filter:
PMCVsetEncMode(PMCV, 0, 0, 1, 3, 1, 1)
PMCVsetEncMode(PMCV, 1, 0, 1, 3, 1, 1)
PMCVsetEncMode(PMCV, 2, 0, 1, 3, 0, 1)

#asynReport 3 $(PMCVPORT)

