/*
FILENAME... PMCVMotorDriver.cpp
USAGE...    


*/

#include "PMCV.h"

asynStatus pmcv_frequency_index_to_enum(int frequency, PMCVAxis::encoderFrequency *freq) {
  if (!freq)
    return asynError;

  switch (frequency) {
  case 0: *freq = PMCVAxis::enc_110khz; break;
  case 1: *freq = PMCVAxis::enc_440khz; break;
  case 2: *freq = PMCVAxis::enc_1_8mhz; break;
  case 3: *freq = PMCVAxis::enc_7mhz; break;
  default:
    fprintf(stderr, "%s:%s: Invalid frequency index %d\n",
            driverName, __FUNCTION__, frequency);
    return asynError;
  }
  return asynSuccess;
}

const char *pmcv_encoder_frequency_to_string(PMCVAxis::encoderFrequency frequency, int &index) {
  switch (frequency) {
  case (PMCVAxis::enc_110khz): index=0; return "110KHz";
  case (PMCVAxis::enc_440khz): index=1; return "440KHz";
  case (PMCVAxis::enc_1_8mhz): index=2; return "1.8MHz";
  case (PMCVAxis::enc_7mhz): index=3; return "7MHz";
  default:
    return "invalid frequency";
  }
}

const char *pmcv_waveform_res_to_string(PMCVAxis::waveformResolution res) {
  switch (res) {
  case (PMCVAxis::rhombf_32):  return "rhombf 32";
  case (PMCVAxis::rhombf_64):  return "rhombf 64";
  case (PMCVAxis::rhombf_128): return "rhombf 128";
  case (PMCVAxis::rhombf_256): return "rhombf 256";
  case (PMCVAxis::rhomb_32):   return "rhomb 32";
  case (PMCVAxis::rhomb_64):   return "rhomb 64";
  case (PMCVAxis::rhomb_128):  return "rhomb 128";
  case (PMCVAxis::rhomb_256):  return "rhomb 256";
  case (PMCVAxis::omega_32):   return "omega 32";
  case (PMCVAxis::omega_64):   return "omega 64";
  case (PMCVAxis::omega_128):  return "omega 128";
  case (PMCVAxis::omega_256_): return "omega 256";
  case (PMCVAxis::omega_256):  return "omega 256";
  case (PMCVAxis::omega_512):  return "omega 512";
  case (PMCVAxis::omega_1024): return "omega 1024";
  case (PMCVAxis::omega_2048): return "omega 2048";
  default:
    return "invalid res";
  }
}

static ELLLIST PMCVList;
static int PMCVListInitialized = 0;

bool addToList(const char *portName, PMCVController *drv) {
    if (!PMCVListInitialized) {
        PMCVListInitialized = 1;
        ellInit(&PMCVList);
    } else if (findByPortName(portName) != NULL) {
        fprintf(stderr, "ERROR: Re-using portName=%s\n", portName);
        return false;
    }

    PMCVNode *pNode = (PMCVNode*)calloc(1, sizeof(PMCVNode));
    pNode->portName = epicsStrDup(portName);
    pNode->pController = drv;
    ellAdd(&PMCVList, (ELLNODE*)pNode);
    return true;
}

PMCVController* findByPortName(const char *portName) {
    PMCVNode *pNode;

    // Find this 
    if (!PMCVListInitialized) {
        printf("%s:%s: ERROR, PMCV list not initialized\n",
            driverName, __FUNCTION__);
        return NULL;
    }

    pNode = (PMCVNode*)ellFirst(&PMCVList);
    while(pNode) {
        if (!strcmp(pNode->portName, portName)) {
            return pNode->pController;
        }
        pNode = (PMCVNode*)ellNext((ELLNODE*)pNode);
    }

    printf("%s: PMCV on port %s not found\n",
        driverName, portName);
    return NULL;
}

///// PMCVCreateController
//
/** Creates a new PMCVController object.
  * Configuration command, called directly or from iocsh
  * \param[in] type              The type of the controller [Use GCS for fully GCS-compatible controllers] (GCS, E-755, ...)
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] PMCVPortName      The name of the drvAsynIPPPort that was created previously to connect to the PMCV controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between moving polls
  * \param[in] idlePollPeriod    The time in ms between idle polls
  */
extern "C" int PMCVCreateController(const char *portName, const char *PMCVPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  if (!portName || !PMCVPortName)
    return asynError;
  new PMCVController(portName, PMCVPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}


/** Code for iocsh registration */
static const iocshArg PMCVCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg PMCVCreateControllerArg1 = {"PMCV port name", iocshArgString};
static const iocshArg PMCVCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg PMCVCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg PMCVCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const PMCVCreateControllerArgs[] = {&PMCVCreateControllerArg0,
                                                               &PMCVCreateControllerArg1,
                                                               &PMCVCreateControllerArg2,
                                                               &PMCVCreateControllerArg3,
                                                               &PMCVCreateControllerArg4
                                                            };
static const iocshFuncDef PMCVCreateControllerDef = {"PMCVCreateController", 5, PMCVCreateControllerArgs};
static void PMCVCreateControllerCallFunc(const iocshArgBuf *args)
{
  PMCVCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}


PMCVAxis *PMCVGetAxis(const char *portName, int axis) {
  if (!portName) {
    fprintf(stderr, "%s: must specify PMCV port name\n",
            driverName);
    return NULL;
  }

  PMCVController* controller=findByPortName(portName);
  if (!controller) {
    fprintf(stderr, "%s: invalid port name\n",
            driverName);
    return NULL;
  }

  PMCVAxis* pAxis=(PMCVAxis*)controller->getAxis(axis);
  if (!pAxis) {
    fprintf(stderr, "%s:%s: Invalid axis number %d (numAxes=%d)\n",
            driverName, portName, axis, controller->getAxisCount());
    return NULL;
  }
  return pAxis;
}

//////////PMCVsetEncMode
/** Set the encoder settings on an axis
  * Configuration command, called directly or from iocsh
 * \param[in] portName             The port name
 * \param[in] axis                 The axis to configure
 * \param[in] spi_mode             SPI mode off (0) on (1)
 * \param[in] quad_mode            Quad mode off (0) on (1)
 * \param[in] frequency            110KHz (0) 440KHz (1) 1.8MHz (2) 7MHz (3)
 * \param[in] swap_ab              Normal quadrature signal (0) swap A/B (1)
 * \param[in] noise_filter         Enable digital noise filter (1) disable (0)
  */
extern "C" int
PMCVsetEncMode(const char* portName, int axis, int spi_mode, int quad_mode, 
                int freq, int swap_ab, int noise_filter)
{
  PMCVAxis *pAxis = PMCVGetAxis(portName, axis);
  if (!pAxis)
    return asynError;
 
  PMCVAxis::encoderFrequency frequency;
  if (pmcv_frequency_index_to_enum(freq, &frequency) != asynSuccess) {
    return asynError;
  }
   
  if (pAxis->setEncMode(spi_mode, quad_mode, frequency, swap_ab, noise_filter) != asynSuccess) {
    fprintf(stderr, "%s:%s:%s: Unable to set encoder mode for axis %d\n",
            driverName, portName, __FUNCTION__, axis);
    return asynError;
  } else {
    return asynSuccess;
  }


}

/** Code for iocsh registration */
static const iocshArg PMCVsetEncModeArg0 = {"portName", iocshArgString};
static const iocshArg PMCVsetEncModeArg1 = {"axis", iocshArgInt};
static const iocshArg PMCVsetEncModeArg2 = {"spi_mode", iocshArgInt};
static const iocshArg PMCVsetEncModeArg3 = {"quad_mode", iocshArgInt};
static const iocshArg PMCVsetEncModeArg4 = {"frequency", iocshArgInt};
static const iocshArg PMCVsetEncModeArg5 = {"swap_ab", iocshArgInt};
static const iocshArg PMCVsetEncModeArg6 = {"noise_filter", iocshArgInt};
static const iocshArg * const PMCVsetEncModeArgs[] = {
  &PMCVsetEncModeArg0,
  &PMCVsetEncModeArg1,
  &PMCVsetEncModeArg2,
  &PMCVsetEncModeArg3,
  &PMCVsetEncModeArg4,
  &PMCVsetEncModeArg5,
  &PMCVsetEncModeArg6,
  };
static const iocshFuncDef PMCVsetEncModeDef = {"PMCVsetEncMode", 7, PMCVsetEncModeArgs};
static void PMCVsetEncModeCallFunc(const iocshArgBuf *args)
{
  PMCVsetEncMode(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival, args[5].ival, args[6].ival);
}


//////////PMCVsetWPC
/** Set waveforms per count manually
  * Configuration command, called directly or from iocsh
 * \param[in] portName             The port name
 * \param[in] axis                 The axis to configure
 * \param[in] wpc                  Waveforms per count
  */
extern "C" int
PMCVsetWPC(const char* portName, int axis, int wpc)
{
  PMCVAxis *pAxis = PMCVGetAxis(portName, axis);
  if (!pAxis)
    return asynError;

  return pAxis->setWPC(wpc, false);
}

/** Code for iocsh registration */
static const iocshArg PMCVsetWPCArg0 = {"portName", iocshArgString};
static const iocshArg PMCVsetWPCArg1 = {"axis", iocshArgInt};
static const iocshArg PMCVsetWPCArg2 = {"wpc", iocshArgInt};
static const iocshArg * const PMCVsetWPCArgs[] = {
  &PMCVsetWPCArg0,
  &PMCVsetWPCArg1,
  &PMCVsetWPCArg2,
  };
static const iocshFuncDef PMCVsetWPCDef = {"PMCVsetWPC", 3, PMCVsetWPCArgs};
static void PMCVsetWPCCallFunc(const iocshArgBuf *args)
{
  PMCVsetWPC(args[0].sval, args[1].ival, args[2].ival);
}

//////////PMCVsetSpeedRamp
/** Set speed ramp (Hz at 1wfm from target)
  * Configuration command, called directly or from iocsh
 * \param[in] portName             The port name
 * \param[in] axis                 The axis to configure
 * \param[in] rampUp               Ramp up Hz
 * \param[in] rampDown             Ramp down Hz
  */
extern "C" int
PMCVsetSpeedRamp(const char* portName, int axis, int rampUp, int rampDown)
{
  PMCVAxis *pAxis = PMCVGetAxis(portName, axis);
  if (!pAxis)
    return asynError;

  return pAxis->setSpeedRamp(rampUp, rampDown);
}

/** Code for iocsh registration */
static const iocshArg PMCVsetSpeedRampArg0 = {"portName", iocshArgString};
static const iocshArg PMCVsetSpeedRampArg1 = {"axis", iocshArgInt};
static const iocshArg PMCVsetSpeedRampArg2 = {"rampUp", iocshArgInt};
static const iocshArg PMCVsetSpeedRampArg3 = {"rampDown", iocshArgInt};
static const iocshArg * const PMCVsetSpeedRampArgs[] = {
  &PMCVsetSpeedRampArg0,
  &PMCVsetSpeedRampArg1,
  &PMCVsetSpeedRampArg2,
  &PMCVsetSpeedRampArg3,
  };
static const iocshFuncDef PMCVsetSpeedRampDef = {"PMCVsetSpeedRamp", 4, PMCVsetSpeedRampArgs};
static void PMCVsetSpeedRampCallFunc(const iocshArgBuf *args)
{
  PMCVsetSpeedRamp(args[0].sval, args[1].ival, args[2].ival, args[3].ival);
}


//////////PMCVsetDZ
/** Set dZ for axis pair
  * Configuration command, called directly or from iocsh
 * \param[in] portName             The port name
 * \param[in] axis                 The first axis of the pair
 * \param[in] dz                   pos0 + dZ = pos1
  */
extern "C" int
PMCVsetDZ(const char* portName, int axis, int dz)
{
  PMCVAxis *pAxis = PMCVGetAxis(portName, axis);
  if (!pAxis)
    return asynError;
 
  return pAxis->setDz(dz);
}

/** Code for iocsh registration */
static const iocshArg PMCVsetDZArg0 = {"portName", iocshArgString};
static const iocshArg PMCVsetDZArg1 = {"axis", iocshArgInt};
static const iocshArg PMCVsetDZArg2 = {"dz", iocshArgInt};
static const iocshArg * const PMCVsetDZArgs[] = {
  &PMCVsetDZArg0,
  &PMCVsetDZArg1,
  &PMCVsetDZArg2,
  };
static const iocshFuncDef PMCVsetDZDef = {"PMCVsetDZ", 3, PMCVsetDZArgs};
static void PMCVsetDZCallFunc(const iocshArgBuf *args)
{
  PMCVsetDZ(args[0].sval, args[1].ival, args[2].ival);
}



//////////PMCVflashCommand
/** Run a command relating to the controller's flash memory
  * Configuration command, called directly or from iocsh
 * \param[in] portName             The port name
 * \param[in] axis                 The first axis of the pair
 * \param[in] command              Save to flash (0), restore from flash (1), restore defaults (2)
  */
extern "C" int
PMCVflashCommand(const char* portName, int axis, int command)
{
  PMCVAxis *pAxis = PMCVGetAxis(portName, axis);
  if (!pAxis)
    return asynError;

  switch (command) {
  case 0: return pAxis->flashSaveSettings();
  case 1: return pAxis->flashRestoreSettings();
  case 2: return pAxis->flashRestoreDefaults();
  default:
    fprintf(stderr, "%s:%s:%s: Invalid command index %d\n",
            driverName, portName, __FUNCTION__, command);
    return asynError;
  }
}

/** Code for iocsh registration */
static const iocshArg PMCVflashCommandArg0 = {"portName", iocshArgString};
static const iocshArg PMCVflashCommandArg1 = {"axis", iocshArgInt};
static const iocshArg PMCVflashCommandArg2 = {"command", iocshArgInt};
static const iocshArg * const PMCVflashCommandArgs[] = {
  &PMCVflashCommandArg0,
  &PMCVflashCommandArg1,
  &PMCVflashCommandArg2,
  };
static const iocshFuncDef PMCVflashCommandDef = {"PMCVflashCommand", 3, PMCVflashCommandArgs};
static void PMCVflashCommandCallFunc(const iocshArgBuf *args)
{
  PMCVflashCommand(args[0].sval, args[1].ival, args[2].ival);
}



//////////PMCVsetIndexMode
/** Set index mode parameters
  * Configuration command, called directly or from iocsh
 * \param[in] portName             The port name
 * \param[in] axis                 The first axis of the pair
 * \param[in] position_reset       Index mode position reset
 * \param[in] stop                 Index mode stop
  */
extern "C" int
PMCVsetIndexMode(const char* portName, int axis, int position_reset, int stop)
{
  PMCVAxis *pAxis = PMCVGetAxis(portName, axis);
  if (!pAxis)
    return asynError;

  return pAxis->setIndexMode(position_reset, stop);
}

/** Code for iocsh registration */
static const iocshArg PMCVsetIndexModeArg0 = {"portName", iocshArgString};
static const iocshArg PMCVsetIndexModeArg1 = {"axis", iocshArgInt};
static const iocshArg PMCVsetIndexModeArg2 = {"position_reset", iocshArgInt};
static const iocshArg PMCVsetIndexModeArg3 = {"stop", iocshArgInt};
static const iocshArg * const PMCVsetIndexModeArgs[] = {
  &PMCVsetIndexModeArg0,
  &PMCVsetIndexModeArg1,
  &PMCVsetIndexModeArg2,
  &PMCVsetIndexModeArg3,
  };
static const iocshFuncDef PMCVsetIndexModeDef = {"PMCVsetIndexMode", 4, PMCVsetIndexModeArgs};
static void PMCVsetIndexModeCallFunc(const iocshArgBuf *args)
{
  PMCVsetIndexMode(args[0].sval, args[1].ival, args[2].ival, args[3].ival);
}


//////////PMCVsetSync
/** Set sync parameters for an axis
  * Configuration command, called directly or from iocsh
 * \param[in] portName             The port name
 * \param[in] axis                 The first axis of the pair
 * \param[in] parallel             Parallel mode
 * \param[in] normal               Normal mode
 * \param[in] sync                 Encoder synchronized mode
  */
extern "C" int
PMCVsetSync(const char* portName, int axis, int parallel, int normal, int sync)
{
  PMCVAxis *pAxis = PMCVGetAxis(portName, axis);
  if (!pAxis)
    return asynError;

  return pAxis->setSync((bool)parallel, (bool)normal, (bool)sync);
}

/** Code for iocsh registration */
static const iocshArg PMCVsetSyncArg0 = {"portName", iocshArgString};
static const iocshArg PMCVsetSyncArg1 = {"axis", iocshArgInt};
static const iocshArg PMCVsetSyncArg2 = {"parallel", iocshArgInt};
static const iocshArg PMCVsetSyncArg3 = {"normal", iocshArgInt};
static const iocshArg PMCVsetSyncArg4 = {"sync", iocshArgInt};
static const iocshArg * const PMCVsetSyncArgs[] = {
  &PMCVsetSyncArg0,
  &PMCVsetSyncArg1,
  &PMCVsetSyncArg2,
  &PMCVsetSyncArg3,
  &PMCVsetSyncArg4,
  };
static const iocshFuncDef PMCVsetSyncDef = {"PMCVsetSync", 5, PMCVsetSyncArgs};
static void PMCVsetSyncCallFunc(const iocshArgBuf *args)
{
  PMCVsetSync(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival);
}


//////////PMCVsetPark
/** Park/unpark an axis
  * Configuration command, called directly or from iocsh
 * \param[in] portName             The port name
 * \param[in] axis                 The first axis of the pair
 * \param[in] park                 Park the axis (1) unpark (0)
  */
extern "C" int
PMCVsetPark(const char* portName, int axis, int park)
{
  PMCVAxis *pAxis = PMCVGetAxis(portName, axis);
  if (!pAxis)
    return asynError;

  return pAxis->setSyncPark((bool)park);
}

/** Code for iocsh registration */
static const iocshArg PMCVsetParkArg0 = {"portName", iocshArgString};
static const iocshArg PMCVsetParkArg1 = {"axis", iocshArgInt};
static const iocshArg PMCVsetParkArg2 = {"park", iocshArgInt};
static const iocshArg * const PMCVsetParkArgs[] = {
  &PMCVsetParkArg0,
  &PMCVsetParkArg1,
  &PMCVsetParkArg2,
  };
static const iocshFuncDef PMCVsetParkDef = {"PMCVsetPark", 3, PMCVsetParkArgs};
static void PMCVsetParkCallFunc(const iocshArgBuf *args)
{
  PMCVsetPark(args[0].sval, args[1].ival, args[2].ival);
}


//////////PMCVsetSpeedLimits
/** Set speed limits for an axis (max is 2200Hz)
  * Configuration command, called directly or from iocsh
 * \param[in] portName             The port name
 * \param[in] axis                 The first axis of the pair
 * \param[in] minHz                Minimum Hz
 * \param[in] maxHz                Maximum Hz
  */
extern "C" int
PMCVsetSpeedLimits(const char* portName, int axis, int minHz, int maxHz)
{
  PMCVAxis *pAxis = PMCVGetAxis(portName, axis);
  if (!pAxis)
    return asynError;

  return pAxis->setSpeedLimits((uint)minHz, (uint)maxHz);
}

/** Code for iocsh registration */
static const iocshArg PMCVsetSpeedLimitsArg0 = {"portName", iocshArgString};
static const iocshArg PMCVsetSpeedLimitsArg1 = {"axis", iocshArgInt};
static const iocshArg PMCVsetSpeedLimitsArg2 = {"minHz", iocshArgInt};
static const iocshArg PMCVsetSpeedLimitsArg3 = {"maxHz", iocshArgInt};
static const iocshArg * const PMCVsetSpeedLimitsArgs[] = {
  &PMCVsetSpeedLimitsArg0,
  &PMCVsetSpeedLimitsArg1,
  &PMCVsetSpeedLimitsArg2,
  &PMCVsetSpeedLimitsArg3,
  };
static const iocshFuncDef PMCVsetSpeedLimitsDef = {"PMCVsetSpeedLimits", 4, PMCVsetSpeedLimitsArgs};
static void PMCVsetSpeedLimitsCallFunc(const iocshArgBuf *args)
{
  PMCVsetSpeedLimits(args[0].sval, args[1].ival, args[2].ival, args[3].ival);
}


//////////PMCVsetWaveformResolution
/*
  * Configuration command, called directly or from iocsh
  * \param[in] portName             The port name
  * \param[in] axis                 The first axis of the pair
  * \param[in] wfmres               Waveform resolution (see below)
  * \param[in] stop                 Stop motor

  wfmres:
    0x0F / 0x0E / 0x0D / 0x0C = Omega 2048/1024/512/256 (used in target mode)
    0x0B / 0x0A / 0x09 / 0x08 = Omega 256/128/64p/32    (used in target mode. 0x0B is same as 0x0C)
    0x07 / 0x06 / 0x05 / 0x04 = Rhomb 256/128/64/32     (faster than Omega)
    0x03 / 0x02 / 0x01 / 0x00 = RhombF 256/128/64/32    (stronger than Omega)
  */
extern "C" int
PMCVsetWaveformResolution(const char* portName, int axis, int wfmres, int stop)
{
  PMCVAxis *pAxis = PMCVGetAxis(portName, axis);
  if (!pAxis)
    return asynError;

  if (wfmres > 0x0F || wfmres < 0) {
    fprintf(stderr, "%s:%s: Invalid wfmres 0x%x. See docs.\n",
            driverName, portName, (uchar)wfmres);
  }

  int run;
  if (stop != 0)
    run = 0;
  else
    run = 1;

  return pAxis->setWfmRes(run, (PMCVAxis::waveformResolution)wfmres);
}

/** Code for iocsh registration */
static const iocshArg PMCVsetWaveformResolutionArg0 = {"portName", iocshArgString};
static const iocshArg PMCVsetWaveformResolutionArg1 = {"axis", iocshArgInt};
static const iocshArg PMCVsetWaveformResolutionArg2 = {"wfmres", iocshArgInt};
static const iocshArg PMCVsetWaveformResolutionArg3 = {"stop", iocshArgInt};
static const iocshArg * const PMCVsetWaveformResolutionArgs[] = {
  &PMCVsetWaveformResolutionArg0,
  &PMCVsetWaveformResolutionArg1,
  &PMCVsetWaveformResolutionArg2,
  &PMCVsetWaveformResolutionArg3,
  };
static const iocshFuncDef PMCVsetWaveformResolutionDef = {"PMCVsetWaveformResolution", 4, PMCVsetWaveformResolutionArgs};
static void PMCVsetWaveformResolutionCallFunc(const iocshArgBuf *args)
{
  PMCVsetWaveformResolution(args[0].sval, args[1].ival, args[2].ival, args[3].ival);
}


//////////PMCVsetMicrodelay
/** Set microstep delay (in usec)
  * Configuration command, called directly or from iocsh
  * \param[in] portName             The port name
  * \param[in] axis                 The first axis of the pair
  * \param[in] delay                Micro step delay (in us, converted to ticks automatically)
  */
extern "C" int
PMCVsetMicrodelay(const char* portName, int axis, double delay)
{
  PMCVAxis *pAxis = PMCVGetAxis(portName, axis);
  if (!pAxis)
    return asynError;

  return pAxis->setMicroDelay(delay);
}

/** Code for iocsh registration */
static const iocshArg PMCVsetMicrodelayArg0 = {"portName", iocshArgString};
static const iocshArg PMCVsetMicrodelayArg1 = {"axis", iocshArgInt};
static const iocshArg PMCVsetMicrodelayArg2 = {"delay", iocshArgDouble};
static const iocshArg * const PMCVsetMicrodelayArgs[] = {
  &PMCVsetMicrodelayArg0,
  &PMCVsetMicrodelayArg1,
  &PMCVsetMicrodelayArg2,
  };
static const iocshFuncDef PMCVsetMicrodelayDef = {"PMCVsetMicroDelay", 3, PMCVsetMicrodelayArgs};
static void PMCVsetMicrodelayCallFunc(const iocshArgBuf *args)
{
  PMCVsetMicrodelay(args[0].sval, args[1].ival, args[2].dval);
}


//////////PMCVclearBoxErrors
/** Update box's comm status and clear errors
  * Configuration command, called directly or from iocsh
  * \param[in] portName             The port name
  */
extern "C" int
PMCVclearBoxErrors(const char* portName)
{
  if (!portName) {
    fprintf(stderr, "%s: must specify PMCV port name\n",
            driverName);
    return asynError;
  }

  PMCVController* controller=findByPortName(portName);
  if (!controller) {
    fprintf(stderr, "%s: invalid port name\n",
            driverName);
    return asynError;
  }

  return controller->clearBoxErrors();
}

/** Code for iocsh registration */
static const iocshArg PMCVclearBoxErrorsArg0 = {"portName", iocshArgString};
static const iocshArg * const PMCVclearBoxErrorsArgs[] = {
  &PMCVclearBoxErrorsArg0,
  };
static const iocshFuncDef PMCVclearBoxErrorsDef = {"PMCVclearBoxErrors", 1, PMCVclearBoxErrorsArgs};
static void PMCVclearBoxErrorsCallFunc(const iocshArgBuf *args)
{
  PMCVclearBoxErrors(args[0].sval);
}



/***********************************************************************/
static void PMCVMotorRegister(void)
{
  iocshRegister(&PMCVCreateControllerDef, PMCVCreateControllerCallFunc);
  iocshRegister(&PMCVsetEncModeDef, PMCVsetEncModeCallFunc);
  iocshRegister(&PMCVsetWPCDef, PMCVsetWPCCallFunc);
  iocshRegister(&PMCVsetSpeedRampDef, PMCVsetSpeedRampCallFunc);
  iocshRegister(&PMCVsetDZDef, PMCVsetDZCallFunc);
  iocshRegister(&PMCVflashCommandDef, PMCVflashCommandCallFunc);
  iocshRegister(&PMCVsetIndexModeDef, PMCVsetIndexModeCallFunc);
  iocshRegister(&PMCVsetSyncDef, PMCVsetSyncCallFunc);
  iocshRegister(&PMCVsetParkDef, PMCVsetParkCallFunc);
  iocshRegister(&PMCVsetSpeedLimitsDef, PMCVsetSpeedLimitsCallFunc);
  iocshRegister(&PMCVsetMicrodelayDef, PMCVsetMicrodelayCallFunc);
  iocshRegister(&PMCVsetWaveformResolutionDef, PMCVsetWaveformResolutionCallFunc);
    iocshRegister(&PMCVclearBoxErrorsDef, PMCVclearBoxErrorsCallFunc);
}

extern "C" {
epicsExportRegistrar(PMCVMotorRegister);
}


