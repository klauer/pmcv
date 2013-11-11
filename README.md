PiezoMotor PMCM51 EPICS Driver
==============================

Motor record driver ("model-3" type) for the PiezoMotor PMCM51 4-axis microstepping piezo driver.

Requirements
------------

Though it may work on other versions, the driver was tested on these:

1. EPICS base 3.14.12.3 http://www.aps.anl.gov/epics/
2. asyn 4-18 http://www.aps.anl.gov/epics/modules/soft/asyn/
3. motor record 6-7 http://www.aps.anl.gov/bcda/synApps/motor/

Optional
--------

1. EDM http://ics-web.sns.ornl.gov/edm/log/getLatest.php
2. Autosave http://www.aps.anl.gov/bcda/synApps/autosave/autosave.html

Installation
------------

1. Install EPICS
    1. If using a Debian-based system (e.g., Ubuntu), use the packages here http://epics.nsls2.bnl.gov/debian/
    2. If no packages are available for your distribution, build from source
2. Edit configure/RELEASE
    1. Point the directories listed in there to the appropriate places
    2. If using the Debian packages, everything can be pointed to /usr/lib/epics
3. Edit iocBoot/iocpmcvtest/st.cmd
    1. Change the shebang on the top of the script if your architecture is different than linux-x86:
        #!../../bin/linux-x86/pmcvtest
        (check if the environment variable EPICS_HOST_ARCH is set, or perhaps `uname -a`, or ask someone if
         you don't know)
    2. The following lines set the prefix to all of the additional (i.e., non-motor record) PVs (with $(P)$(R)):
        ```
        epicsEnvSet("P", "$(P=MLL:)")
        epicsEnvSet("R", "$(R=PMCV:)")
        ```
       Set the second quoted strings appropriately.
    3. The following line sets the autosave prefix for your PMCV PVs:
        ```
        # Autosave prefix
        epicsEnvSet("AS_PREFIX", "$(P)$(R)")
        ```
       Set the second quoted string appropriately.
    4. The following line sets the IP address of the device:
        ```
        epicsEnvSet("PMCV_IP" "$(PMCV_IP=10.0.0.21)")
        epicsEnvSet("PMCV_TCP_PORT" "$(PMCV_TCP_PORT=1024)")
        ```
        The port should remain the same, but change the IP address.
    5. If necessary, you can change the rate at which the controller is polled for positions and such:
        ```
        #PMCVCreateController(portName, PMCVPortName, numAxes, movingPollPeriod, idlePollPeriod)

        PMCVCreateController("$(PMCV_PORT)", "$(ASYN_PORT)", 1, 50, 100)
        ```
        The moving and idle poll periods are both in milliseconds. The former rate is used when an axis is in motion, the latter otherwise.
    6. The controller has additional features outside of the standard motor record. Add these additional PVs per controller:
       `dbLoadRecords("$(TOP)/db/pmcv_general.db", "P=$(P),R=$(R),PORT=$(PMCVPORT),ADDR=0,TIMEOUT=1")`
       and per-axis: ` bLoadRecords("$(TOP)/db/pmcv_axis.db", "P=$(P),R=$(R)axis1:,PORT=$(PMCVPORT),ADDR=0,TIMEOUT=1")`
    7. Set-up the encoder settings for each axis on the controllers (see the example in the file -- PMCVsetEncMode).
    8. If using autosave, add lines in auto_positions.req and auto_settings.req for each motor. If not using autosave, comment create_monitor_set lines.

4.  Edit iocBoot/iocpmcvtest/pmcv.sub
    Modify the lines so that there is one motor.db line per axis. Each will be named $(P)$(M).
5. Go to the top directory and `make`
6. If all goes well:
    ```
    $ cd iocBoot/iocpmcvtest
    $ chmod +x st.cmd
    $ ./st.cmd
    ```

7. Run EDM:
    ```
    $ export EDMDATAFILES=$TOP/op/edl:$EDMDATAFILES
    $ edm -x -m "P=MLL:,R=PMCV:,M1=pmcv1,M2=pmcv2,M3=pmcv3,M4=pmcv4" pmcv_general
    ```

Note
----

No, I cannot remember why this is named PMCV. (TODO rename it someday)
