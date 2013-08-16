#include "PMCV.h"

void PMCVController::fullUpdate() {
  readBoxStatus();
  readBoxFirmware();
}

void PMCVAxis::fullUpdate() {
  if (axisNo_ == 0) {
    pc_->fullUpdate();
  }

//TODO: move some of these to initialization -- don't need to be read continuously
//
  bool spi, quad, swap_ab, noise_filter, sync_mode, parallel;
  encoderFrequency frequency;
  if (readEncMode(spi, quad, frequency, swap_ab, noise_filter, sync_mode, parallel) == asynSuccess) {
    int freq_index;
    pc_->setStringParam(axisNo_, pc_->encFrequencyStr_, pmcv_encoder_frequency_to_string(frequency, freq_index));
    setIntegerParam(pc_->encFrequency_, freq_index);
    setIntegerParam(pc_->encParallel_, parallel);
    setIntegerParam(pc_->encNoiseFilter_, noise_filter);
    setIntegerParam(pc_->encSyncMode_, sync_mode);
    setIntegerParam(pc_->encSwapAb_, swap_ab);
    setIntegerParam(pc_->encQuadMode_, quad);
    setIntegerParam(pc_->encSpiMode_, spi);
  }


  bool silent, park, stop_index, pos_reset, target_mode;
  if (readDrvMode(parallel, silent, park, stop_index, pos_reset, target_mode, sync_mode) == asynSuccess) {
    setIntegerParam(pc_->drvParallel_, parallel);
    setIntegerParam(pc_->drvSilent_, silent);
    setIntegerParam(pc_->drvPark_, park);
    setIntegerParam(pc_->drvStopIndex_, stop_index);
    setIntegerParam(pc_->drvPosReset_, pos_reset);
    setIntegerParam(pc_->drvTargetMode_, target_mode);
    setIntegerParam(pc_->drvSyncMode_, sync_mode);
  }

  if (readEncLimits(limitA_, limitB_) == asynSuccess) {
    setDoubleParam(pc_->motorLowLimit_, limitA_);
    setDoubleParam(pc_->motorHighLimit_, limitB_);
  }

  epicsInt32 dz;
  if (readDz(dz) == asynSuccess) {
    setIntegerParam(pc_->syncDz_, dz);
  }

  double us_delay;
  if (readMicroDelay(us_delay) == asynSuccess) {
    setDoubleParam(pc_->microDelay_, us_delay);
  }

  bool run;
  waveformResolution res;
  if (readWfmRes(run, res) == asynSuccess) {
    pc_->setStringParam(axisNo_, pc_->waveformResStr_, 
                        pmcv_waveform_res_to_string(res));
    setIntegerParam(pc_->waveformRes_, res);
  }

  epicsUInt16 rampUp, rampDown;
  if (readSpeedRamps(rampUp, rampDown) == asynSuccess) {
    setIntegerParam(pc_->speedRampUp_, rampUp);
    setIntegerParam(pc_->speedRampDown_, rampDown);
  }

  epicsUInt16 minHz, maxHz;
  if (readSpeedLimits(minHz, maxHz) == asynSuccess) {
    setDoubleParam(pc_->motorVelBase_, minHz);
    setIntegerParam(pc_->minHz_, minHz);
    setIntegerParam(pc_->maxHz_, maxHz);
  }

  epicsUInt32 wpc;
  if (readWPC(wpc) == asynSuccess) {
    setIntegerParam(pc_->wpc_, wpc);
  }

  /*if (setWPC(10) != asynSuccess)
    printf("readWPC() failed\n");
  if (setSpeedRamp(PMCV_DEFAULT_SPEED_RAMP, PMCV_DEFAULT_SPEED_RAMP) != asynSuccess)
    printf("setSpeedRamp() failed\n");
  if (setSpeedLimits(PMCV_DEFAULT_MIN_HZ, PMCV_DEFAULT_MAX_HZ) != asynSuccess)
    printf("setSpeedLimits() failed\n");*/

  /*if (flashSaveSettings() != asynSuccess)
    printf("flashSaveSettings() failed\n");
  else
    printf("save settings\n");

  if (readPosition() != asynSuccess)
    printf("readPosition() failed\n");
  if (readFlashPos() != asynSuccess)
    printf("readFlashPos() failed\n");

*/

}




asynStatus PMCVAxis::poll(bool *moving) {
  *moving = false;

  epicsInt32 position;
  if (readPosition(position) != asynSuccess) {
    statusFailed_++;
    if (statusFailed_ > PMCV_STATUS_FAILED_THRESHOLD) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
        "%s:%s: ERROR: axis %d reached status response failed threshold (%d); exiting\n",
        driverName, __FUNCTION__, axisNo_, PMCV_STATUS_FAILED_THRESHOLD);
      exit(1);
    }

  } else {
    statusFailed_ = 0;
    setDoubleParam(pc_->motorPosition_, (double)position);
    setDoubleParam(pc_->motorEncoderPosition_, (double)position);
  }

  //printf("read position %d\n", position);

  pollNumber_++;
  if (pollNumber_ > PMCV_FULL_UPDATE_RATE) {
    fullUpdate();
    pollNumber_ = 0;
  }

  epicsInt32 readpos;
  if (readTarget(readpos) == asynSuccess) {
    //
  }

  readStatus2();
  setIntegerParam(pc_->motorStatusMoving_, moving_);
  setIntegerParam(pc_->motorStatusDone_, !moving_);
  /*if (axisNo_ == 0)
    printf("limit a %d limit b %d\n", limitA_, limitB_);*/
  setIntegerParam(pc_->motorStatusLowLimit_, hitLimitA_);
  setIntegerParam(pc_->motorStatusHighLimit_, hitLimitB_);
  setIntegerParam(pc_->motorStatusDirection_, direction_);
  setIntegerParam(pc_->motorStatusCommsError_, (comBuffErr_ || comFrameErr_));

  if (moving)
    *moving = moving_;

  callParamCallbacks();
  return asynSuccess;
}

/** Creates a new PMCVAxis object.
  * \param[in] controller         The PMCV controller
  * \param[in] axis_num           The axis number (1-based)
  */
PMCVAxis::PMCVAxis(PMCVController *controller, int axis_num)
  :  asynMotorAxis((asynMotorController*)controller, axis_num)
{
  id_ = axis_num;
  pc_ = controller;
  encoderPos_ = 0.0;
  statusFailed_ = 0;
  pollNumber_ = 0;
  setIntegerParam(pc_->motorStatusHasEncoder_, 1);

  asynStatus status = setSyncPark(PMCV_SYNC_PARK_DEFAULT);
  if (status != asynSuccess) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
      "%s:axis %d: Unable to initialize (unpark, ackmode, normal operation): %s\n",
      __FUNCTION__, axisNo_, pasynUser_->errorMessage);
  }

  char firmware_id, version;
  if (readFirmware(firmware_id, version) == asynSuccess) {
    setIntegerParam(pc_->firmwareId_, firmware_id);
    setIntegerParam(pc_->firmwareVersion_, version);
  }
}

asynStatus PMCVAxis::readPosition(epicsInt32 &position) {
  uchar buf[4];
  asynStatus ret = pc_->command(PMCV_BUILD_AXIS_READ_POS(axisNo_), (char*)buf, 4);
  
  if (ret != asynSuccess)
    return ret;

  if (pmcv_decode_int32(&position, (char*)buf)) {
    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
      "%s:%s: axis %d position %d\n",
      driverName, __FUNCTION__, axisNo_, position);
  }

  return asynSuccess;
}


asynStatus PMCVAxis::stop() {
  // TODO: ack returned?
  return pc_->commandByAxis(axisNo_, PMCV_CMD_STOP, NULL, 0);
}

asynStatus PMCVAxis::setTarget(epicsInt32 target, bool move) {
  const int output_length = 5;
  char output[output_length];

  output[0] = PMCV_CMD_SET_TARGET;
  if (!pmcv_encode_int32(target, &output[1])) {
    return asynError;
  }

  asynStatus ret = pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);

  if (ret == asynSuccess) {
    return targetModeOn();
  }

  return ret;

}

asynStatus PMCVAxis::readStatus1() {
  const int input_length = 1;
  char input[input_length];
  const char output = PMCV_CMD_READ_STATUS1;

  asynStatus ret = pc_->commandByAxis(axisNo_, output, input, input_length);

  if (ret != asynSuccess)
    return ret;

  updateStatus((uchar)input[0]);
  return asynSuccess;
}

void PMCVAxis::updateStatus(uchar status) {
  anyError_ = ((status & 0x80) != 0);
  targetLimit_ = ((status & 0x40) != 0);
  hitLimitB_ = ((status & 0x20) == 0);
  hitLimitA_ = ((status & 0x10) == 0);
  indexPass_ = ((status & 0x08) != 0);
  targetPass_ = ((status & 0x04) != 0);
  direction_ = ((status & 0x02) != 0);
  moving_ = ((status & 0x01) != 0);

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: axis %d: anyError %d targetLimit %d limitB %d limitA %d indexPass %d targetPass %d direction %d moving %d\n",
    driverName, __FUNCTION__, axisNo_, anyError_, targetLimit_, limitB_, limitA_, indexPass_, targetPass_, direction_, moving_);

  
}

asynStatus PMCVAxis::readStatus2() {
  const int input_length = 3;
  char input[input_length];
  const char output = PMCV_CMD_READ_STATUS2;

  asynStatus ret = pc_->commandByAxis(axisNo_, output, input, input_length);

  if (ret != asynSuccess)
    return ret;

  updateStatus((uchar)input[0]);

  uchar status = input[1];
  flashErr_ = ((status & 0x80) != 0);
  syncErr_ = ((status & 0x40) != 0);
  comBuffErr_ = ((status & 0x20) != 0);
  comFrameErr_ = ((status & 0x10) != 0);
  encErr_ = ((status & 0x08) != 0);
  drvErr_ = ((status & 0x04) != 0);
  overheat_ = ((status & 0x02) != 0);
  cmdErr_ = ((status & 0x01) != 0);

  status = input[2];
  firmErr_ = ((status & 0x02) != 0);
  resetErr_ = ((status & 0x01) != 0);

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: axis %d: firmErr %d resetErr %d flashErr %d syncErr %d comBuffErr %d comFrameErr %d encErr %d drvErr %d overheat %d cmdErr %d \n",
    driverName, __FUNCTION__, axisNo_, firmErr_, resetErr_, flashErr_, syncErr_, comBuffErr_, comFrameErr_, encErr_, drvErr_, overheat_, cmdErr_);
  return asynSuccess;
}

asynStatus PMCVAxis::targetModeOn() {
  return pc_->commandByAxis(axisNo_, PMCV_CMD_TARGET_MODE_ON, NULL, 0, 1);
}

asynStatus PMCVAxis::readTarget(epicsInt32 &target) {
  uchar buf[4];
  asynStatus ret = pc_->commandByAxis(axisNo_, PMCV_CMD_READ_TARGET, (char*)buf, 4);

  if (ret != asynSuccess)
    return ret;

  if (pmcv_decode_int32(&target, (char*)buf)) {
    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
      "%s:%s: axis %d target position %d\n",
      driverName, __FUNCTION__, axisNo_, target);
    return asynSuccess;
  } else {
    return asynError;
  }

}

asynStatus PMCVAxis::setIndexMode(char flags) {
  const int output_length = 2;
  char output[output_length];

  output[0] = PMCV_CMD_SET_INDEXMODE;
  output[1] = flags;

  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);
}

asynStatus PMCVAxis::setIndexMode(bool position_reset, bool stop) {
  char flags=0;
  if (position_reset)
    flags |= PMCV_SET_INDEX_MODE_POS_RESET;

  if (stop)
    flags |= PMCV_SET_INDEX_MODE_STOP_INDEX;

  return setIndexMode(flags);
}

asynStatus PMCVAxis::readDrvMode(char flags) {
  const int input_length = 1;
  char input[input_length];
  const char output = PMCV_CMD_READ_DRVMODE;

  asynStatus ret = pc_->commandByAxis(axisNo_, output, input, input_length);

  if (ret == asynSuccess)
    flags = input[0];

  return ret;
}

asynStatus PMCVAxis::readDrvMode(bool &parallel, bool &silent, bool &park, bool &stop_index, bool &pos_reset, bool &target_mode, bool &sync_mode) {
  char flags = 0;
  asynStatus ret = readDrvMode(flags);
  if (ret != asynSuccess)
    return ret;

  parallel = ((flags & PMCV_DRV_MODE_PARALLEL) != 0);
  silent = ((flags & PMCV_DRV_MODE_SILENT) != 0);
  park = ((flags & PMCV_DRV_MODE_PARK) != 0);
  stop_index = ((flags & PMCV_DRV_MODE_STOP_INDEX) != 0);
  pos_reset = ((flags & PMCV_DRV_MODE_POS_RESET) != 0);
  target_mode = ((flags & PMCV_DRV_MODE_TARGET_MODE) != 0);
  sync_mode = ((flags & PMCV_DRV_MODE_SYNC_MODE) != 0);

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: axis %d: parallel %d silent %d park %d stop_index %d pos_reset %d target_mode %d sync_mode %d\n",
    driverName, __FUNCTION__, axisNo_, parallel, silent, park, stop_index, pos_reset, target_mode, sync_mode);

  return asynSuccess;
}

asynStatus PMCVAxis::setSync(bool parallel, bool normal, bool sync) {
  uchar flags = 0;
  if (parallel)
    flags |= PMCV_SYNC_PARK_PARALLEL;
  if (normal)
    flags |= PMCV_SYNC_PARK_NORMAL;
  if (sync)
    flags |= PMCV_SYNC_PARK_SYNC;

  return setSyncPark(flags);
}

asynStatus PMCVAxis::setPark(bool park) {
  uchar flags;
  if (park) {
    flags = PMCV_SYNC_PARK_PARK;
    fprintf(stderr, "Park axis %d\n", axisNo_);
  } else {
    flags = PMCV_SYNC_PARK_UNPARK;
    fprintf(stderr, "Unpark axis %d\n", axisNo_);
  }
  return setSyncPark(flags);
}


asynStatus PMCVAxis::setSyncPark(bool ack_mode, bool silent_mode, bool parallel, bool unpark, bool park, bool normal, bool sync) {
  // only changes are updated, which is why park/unpark are different bits
  char flags = 0;
  if (ack_mode) 
    flags |= PMCV_SYNC_PARK_ACKMODE;
  else if (silent_mode)
    flags |= PMCV_SYNC_PARK_SILENT;

  if (parallel)
    flags |= PMCV_SYNC_PARK_PARALLEL;

  if (park)
    flags |= PMCV_SYNC_PARK_PARK;
  else if (unpark)
    flags |= PMCV_SYNC_PARK_UNPARK;

  if (normal)
    flags |= PMCV_SYNC_PARK_NORMAL;
  if (sync)
    flags |= PMCV_SYNC_PARK_SYNC;
  return setSyncPark(flags);
}

asynStatus PMCVAxis::setSyncPark(char flags) {
  const int output_length = 2;
  char output[output_length];

  output[0] = PMCV_CMD_SET_SYNC_PARK;
  output[1] = flags;

  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);
}

asynStatus PMCVAxis::setEncLimits(epicsInt32 low, epicsInt32 high) {
  const int output_length = 9; // 1 + 2x4
  char output[output_length];

  output[0] = PMCV_CMD_SET_ENCLIMITS;

  //output[1, 2, 3, 4] = limit A
  //output[5, 6, 7, 8] = limit B
  if (!pmcv_encode_int32(low, &output[1]))
    return asynError;
  if (!pmcv_encode_int32(high, &output[5]))
    return asynError;

  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);
}

asynStatus PMCVAxis::readEncLimits(epicsInt32 &limit_a, epicsInt32 &limit_b) {
  const int input_length = 8;
  char input[input_length];

  asynStatus ret = pc_->commandByAxis(axisNo_, PMCV_CMD_READ_ENC_LIMITS, input, input_length);

  if (ret != asynSuccess)
    return ret;

  //input[0, 1, 2, 3] = limit A
  //input[4, 5, 6, 7] = limit B
 
  pmcv_decode_int32(&limit_a, &input[0]);
  pmcv_decode_int32(&limit_b, &input[4]);

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: axis %d: low limit %d high limit %d\n",
    driverName, __FUNCTION__, axisNo_, limit_a, limit_b);

  return asynSuccess;
}

asynStatus PMCVAxis::setWPC(epicsUInt32 wpc, bool is_calculated_wpc) {
  const int output_length = 1 + 4;
  char output[output_length];

  output[0] = PMCV_CMD_SET_WPC;

  if (is_calculated_wpc)
    wpc >>= 16; // divide by 65536

  //output[1, 2, 3, 4] = wpc
  if (!pmcv_encode_uint32(wpc, &output[1]))
    return asynError;

  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);
}

asynStatus PMCVAxis::readWPC(epicsUInt32 &wpc) {
  const int input_length = 4;
  char input[input_length];

  asynStatus ret = pc_->commandByAxis(axisNo_, PMCV_CMD_READ_WPC, input, input_length);

  if (ret != asynSuccess)
    return ret;

  if (pmcv_decode_uint32(&wpc, input)) {
    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
      "%s:%s: axis %d: wpc %d per 65536 counts (calculated wpc: %d)\n",
      driverName, __FUNCTION__, axisNo_, wpc, wpc << 16);
    return asynSuccess;
  } else {
    return asynError;
  }
}

asynStatus PMCVAxis::setSpeedRamp(epicsUInt16 rampUp, epicsUInt16 rampDown) {
  const int output_length = 2 + 2 + 1;
  char output[output_length];

  // output[1, 2] rampUp
  // output[3, 4] rampDown
  output[0] = PMCV_CMD_SET_SPEED_RAMP;
  if (!pmcv_encode_uint16(rampUp, &output[1])) {
    return asynError;
  }
  if (!pmcv_encode_uint16(rampDown, &output[3])) {
    return asynError;
  }

  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);
}

asynStatus PMCVAxis::readSpeedRamps(epicsUInt16 &rampUp, epicsUInt16 &rampDown) {
  const int input_length = 2+2;
  char input[input_length];

  asynStatus ret = pc_->commandByAxis(axisNo_, PMCV_CMD_READ_SPEED_RAMPS, input, input_length);

  if (ret != asynSuccess)
    return ret;

  // rampUp   input[0, 1]  (14-bit, unsigned)
  // rampDown input[2, 3]
  ;
  if (!pmcv_decode_uint16(&rampUp, &input[0]))
    return asynError;
  if (!pmcv_decode_uint16(&rampDown, &input[2]))
    return asynError;

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: axis %d: ramp up (unused): %d Hz ramp down: %d Hz at 1wfm from target\n",
    driverName, __FUNCTION__, axisNo_, rampUp, rampDown);

  return asynSuccess;
}

asynStatus PMCVAxis::setSpeedLimits(epicsUInt16 minHz, epicsUInt16 maxHz) {
  const int output_length = 2 + 2 + 1;
  char output[output_length];

  if (minHz > PMCV_MAX_SPEED_HZ || maxHz > PMCV_MAX_SPEED_HZ || minHz > maxHz) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
      "%s:%s: axis %d: invalid speed limits: %d Hz %d Hz (max is %d)\n",
      driverName, __FUNCTION__, axisNo_, minHz, maxHz, PMCV_MAX_SPEED_HZ);
    return asynError;
  }

  // output[1, 2] minHz
  // output[3, 4] maxHz
  output[0] = PMCV_CMD_SET_SPEED_LIMITS;
  if (!pmcv_encode_uint16(minHz, &output[1])) {
    return asynError;
  }
  if (!pmcv_encode_uint16(maxHz, &output[3])) {
    return asynError;
  }

  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);
}

asynStatus PMCVAxis::readSpeedLimits(epicsUInt16 &minHz, epicsUInt16 &maxHz) {
  // default 10, 2200Hz
  const int input_length = 2+2;
  char input[input_length];

  asynStatus ret = pc_->commandByAxis(axisNo_, PMCV_CMD_READ_SPEED_LIMITS, input, input_length);

  if (ret != asynSuccess)
    return ret;

  // minHz input[0, 1]  (14-bit, unsigned)
  // maxHz input[2, 3]
  if (!pmcv_decode_uint16(&minHz, &input[0]))
    return asynError;
  if (!pmcv_decode_uint16(&maxHz, &input[2]))
    return asynError;

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: axis %d: min: %d Hz max: %d Hz\n",
    driverName, __FUNCTION__, axisNo_, minHz, maxHz);

  return asynSuccess;
}

asynStatus PMCVAxis::setPosition(epicsInt32 position) {
  const int output_length = 5;
  char output[output_length];

  output[0] = PMCV_CMD_SET_POS;
  if (!pmcv_encode_int32(position, &output[1])) {
    return asynError;
  }

  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);
}

asynStatus PMCVAxis::setDz(epicsInt32 dz) {
  // sync mode: (applies to first of axis pair)
  // position 0 + dZ = position 1 (when perfectly aligned)
  //
  const int output_length = 5;
  char output[output_length];

  output[0] = PMCV_CMD_SET_DZ;
  if (!pmcv_encode_int32(dz, &output[1])) {
    return asynError;
  }

  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);
}

asynStatus PMCVAxis::readDz(epicsInt32 &dz) {
  // sync mode:
  // position 0 + dZ = position 1 (when perfectly aligned)
  uchar buf[4];
  asynStatus ret = pc_->commandByAxis(axisNo_, PMCV_CMD_READ_DZ, (char*)buf, 4);
  
  if (ret != asynSuccess)
    return ret;

  if (pmcv_decode_int32(&dz, (char*)buf)) {
    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
      "%s:%s: axis %d dZ %d\n",
      driverName, __FUNCTION__, axisNo_, dz);
  }

  return asynSuccess;
}

asynStatus PMCVAxis::stopRestore() {
  // TODO: ack returned?
  return pc_->commandByAxis(axisNo_, PMCV_CMD_STOP_RESTORE, NULL, 0);
}

asynStatus PMCVAxis::setEncMode(bool spi, bool quad, encoderFrequency frequency, bool swap_ab, bool noise_filter) {
  uchar flags = frequency;
  if (spi)
    flags |= PMCV_ENC_MODE_SPI_MODE;
  else if (quad)
    flags |= PMCV_ENC_MODE_QUAD_MODE;

  if (swap_ab)
    flags |= PMCV_ENC_MODE_SWAP_AB;
  if (noise_filter)
    flags |= PMCV_ENC_MODE_NOISE_FILTER;

  return setEncMode(flags);
}

asynStatus PMCVAxis::setEncMode(uchar flags) {
  const int output_length = 3;
  char output[output_length];

  output[0] = PMCV_CMD_SET_ENC_MODE;
  output[1] = flags;
  output[2] = 0;

  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);

}

asynStatus PMCVAxis::readEncMode(char &flags) {
  const int input_length = 2;
  char input[input_length];

  asynStatus ret = pc_->commandByAxis(axisNo_, PMCV_CMD_READ_ENC_MODE, input, input_length);

  if (ret == asynSuccess)
    flags = input[0];

  return ret;
}

asynStatus PMCVAxis::readEncMode(bool &spi, bool &quad, encoderFrequency &frequency, bool &swap_ab, bool &noise_filter, bool &sync_mode, bool &parallel) {
  char status;
  asynStatus result;
  if ((result = readEncMode(status)) != asynSuccess)
    return result;

  parallel = (status & PMCV_ENC_MODE_PARALLEL) != 0;
  noise_filter = (status & PMCV_ENC_MODE_NOISE_FILTER) != 0;
  swap_ab = (status & PMCV_ENC_MODE_SWAP_AB) != 0;
  quad = (status & PMCV_ENC_MODE_QUAD_MODE) != 0;
  spi = (status & PMCV_ENC_MODE_SPI_MODE) != 0;
  sync_mode = (status & PMCV_ENC_MODE_SYNC_MODE) != 0;

  //bool quad_b = (status & PMCV_ENC_MODE_QUAD_B) != 0;
  //bool quad_a = (status & PMCV_ENC_MODE_QUAD_A) != 0;
  frequency = (encoderFrequency)(status & PMCV_ENC_FREQ_MASK);

  int freq_index;
  const char *freq_string = pmcv_encoder_frequency_to_string(frequency, freq_index);

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: axis %d: parallel %d noise_filter %d swap_ab %d quad_mode %d spi_mode %d sync_mode %d frequency: %s\n",
    driverName, __FUNCTION__, axisNo_, parallel, noise_filter, swap_ab, quad, spi, sync_mode, freq_string);
  return asynSuccess;
}

asynStatus PMCVAxis::setWfmRes(bool run, waveformResolution res) {
  char flags = 0;

  if (res < 0 || res > 0x0F)
    return asynError;

  if (run)
    flags |= PMCV_WFM_RES_RUN;
  
  flags |= res;

  return setWfmRes(flags);
}

asynStatus PMCVAxis::setWfmRes(char flags) {
  const int output_length = 2;
  char output[output_length];

  output[0] = PMCV_CMD_SET_WFM_RES;
  output[1] = flags;

  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);
}

asynStatus PMCVAxis::readWfmRes(bool &run, waveformResolution &res) {
  const int input_length = 1;
  char input[input_length];

  asynStatus ret = pc_->commandByAxis(axisNo_, PMCV_CMD_READ_WFM_RES, input, input_length);

  if (ret != asynSuccess)
    return ret;

  uchar status = input[0];
  run = (status & PMCV_WFM_RES_RUN) != 0;
  res = (waveformResolution)(status & PMCV_WFM_RES_MASK);

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: axis %d: run %d waveformRes: %s (%x)\n",
    driverName, __FUNCTION__, axisNo_, run, PMCV_WFM_RES_STRINGS[res],
    (uchar)res);

  return asynSuccess;
}

asynStatus PMCVAxis::runSteps(epicsUInt32 steps, bool direction, bool run) {
  const int output_length = 5;
  char output[output_length];

  output[0] = PMCV_CMD_RUN_STEPS;
  if (!pmcv_encode_uint32(steps, &output[1])) {
    return asynError;
  }

  // output[1, 2, 3, 4] = steps
  output[4] &= PMCV_RUN_STEPS_MSB_MASK;

  if (direction)
    output[4] |= PMCV_RUN_STEPS_DIRECTION;

  if (run)
    output[4] |= PMCV_RUN_STEPS_RUN;

  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0);
}

asynStatus PMCVAxis::readSteps(epicsUInt32 &steps) {
  const int input_length = 4;
  char input[input_length];

  asynStatus ret = pc_->commandByAxis(axisNo_, PMCV_CMD_READ_STEPS, input, input_length);

  if (ret != asynSuccess)
    return ret;

  if (pmcv_decode_uint32(&steps, input)) {
    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
      "%s:%s: axis %d: steps remaining: %d\n",
      driverName, __FUNCTION__, axisNo_, steps);
    return asynSuccess;
  } else {
    return asynError;
  }
}

asynStatus PMCVAxis::setMicroDelay(double us_delay) {
  epicsUInt32 ticks = us_delay / PMCV_US_PER_TICK;
  return setMicroDelay(ticks);
}

asynStatus PMCVAxis::setMicroDelay(epicsUInt32 ticks) {
  const int output_length = 5;
  char output[output_length];

  output[0] = PMCV_CMD_SET_MICRO_DELAY;
  if (!pmcv_encode_uint32(ticks, &output[1])) {
    return asynError;
  }

  // output[1, 2, 3, 4] = ticks
  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);
}

asynStatus PMCVAxis::readMicroDelay(double &us_delay) {
  epicsUInt32 ticks;
  asynStatus ret = readMicroDelay(ticks);
  if (ret == asynSuccess)
    us_delay = ticks * PMCV_US_PER_TICK;

  return ret;
}

asynStatus PMCVAxis::readMicroDelay(epicsUInt32 &ticks) {
  const int input_length = 4;
  char input[input_length];

  asynStatus ret = pc_->commandByAxis(axisNo_, PMCV_CMD_READ_MICRO_DELAY, input, input_length);

  if (ret != asynSuccess)
    return ret;

  double us;
  if (pmcv_decode_uint32(&ticks, input)) {
    us = (double)ticks * PMCV_US_PER_TICK;
    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
      "%s:%s: axis %d: micro delay: %d ticks %g us\n",
      driverName, __FUNCTION__, axisNo_, ticks, us);
    return asynSuccess;
  } else {
    return asynError;
  }
}

asynStatus PMCVAxis::setWfmFreq(epicsUInt16 frequency) {
  // NOTE: not implemented on the controller side
  const int output_length = 2 + 1;
  char output[output_length];

  // output[1, 2] frequency
  output[0] = PMCV_CMD_SET_WFM_FREQ;
  if (!pmcv_encode_uint16(frequency, &output[1])) {
    return asynError;
  }

  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);
}

asynStatus PMCVAxis::noOperation() {
  const int input_length = 1;
  char input[input_length];
  const char output = PMCV_CMD_NO_OPERATION;

  asynStatus ret = pc_->commandByAxis(axisNo_, output, input, input_length);

  if (ret != asynSuccess)
    return ret;

  if (input[0] == PMCV_BEL)
    return asynSuccess;
  else
    return asynError;
}

asynStatus PMCVAxis::clearErrors() {
  return pc_->commandByAxis(axisNo_, PMCV_CMD_CLEAR_ERRORS, NULL, 0, 1);
}


asynStatus PMCVAxis::setUserbyte(epicsUInt32 userbyte) {
  const int output_length = 1 + 4;
  char output[output_length];

  output[0] = PMCV_CMD_SET_USERBYTE;
  //output[1, 2, 3, 4] = userbyte
  if (!pmcv_encode_uint32(userbyte, &output[1]))
    return asynError;

  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);
}

asynStatus PMCVAxis::readUserbyte(epicsUInt32 &userbyte) {
  const int input_length = 4;
  char input[input_length];

  asynStatus ret = pc_->commandByAxis(axisNo_, PMCV_CMD_READ_USERBYTE, input, input_length);

  if (ret != asynSuccess)
    return ret;

  if (pmcv_decode_uint32(&userbyte, input)) {
    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
      "%s:%s: axis %d: userbyte %d\n",
      driverName, __FUNCTION__, axisNo_, userbyte);
    return asynSuccess;
  } else {
    return asynError;
  }
}

asynStatus PMCVAxis::readFlashPos(epicsUInt16 &checksum, epicsInt32 &position) {
  const int input_length = PMCV_FLASHPOS_LENGTH; //8
  uchar input[input_length];

  asynStatus ret = pc_->commandByAxis(axisNo_, PMCV_CMD_READ_FLASH_POS, (char*)input, input_length);

  if (ret != asynSuccess)
    return ret;

  if (input[PMCV_FLASHPOS_CHECK1_BYTE] != PMCV_FLASHPOS_CHECK1_VALUE) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
      "%s:%s: check values %x %x differ from expected %x %x\n",
      driverName, __FUNCTION__, 
      input[PMCV_FLASHPOS_CHECK1_BYTE], input[PMCV_FLASHPOS_CHECK2_BYTE], 
      PMCV_FLASHPOS_CHECK1_VALUE, PMCV_FLASHPOS_CHECK2_VALUE);
    return asynError;
  }

  if (!pmcv_decode_uint16(&checksum, (char*)&input[PMCV_FLASHPOS_CHECKSUM_START]))
    return asynError;

  asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER,
    "%s:%s: checksum %x%x\n",
    driverName, __FUNCTION__, 
    input[PMCV_FLASHPOS_CHECKSUM_START], input[PMCV_FLASHPOS_CHECKSUM_START+1]);

  if (pmcv_decode_int32(&position, (char*)&input[PMCV_FLASHPOS_POSITION_START])) {
    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
      "%s:%s: axis %d flash position %d\n",
      driverName, __FUNCTION__, axisNo_, position);
  }
 
  /*asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: axis %d packet %x %x %x %x / %x %x / %x %x\n",
    driverName, __FUNCTION__, axisNo_, 
    input[0], input[1], input[2], input[3], input[4], input[5], input[6], input[7]);*/

  return asynSuccess;
}

asynStatus PMCVAxis::flashSaveSettings() {
  return flashCmd(PMCV_FLASH_SAVE_SETTINGS);
}

asynStatus PMCVAxis::flashRestoreSettings() {
  return flashCmd(PMCV_FLASH_RESTORE_SAVED);
}

asynStatus PMCVAxis::flashRestoreDefaults() {
  return flashCmd(PMCV_FLASH_RESTORE_DEFAULT);
}

asynStatus PMCVAxis::flashCmd(char flags) {
  // (applies to first of axis pair)

  const int output_length = 2;
  char output[output_length];

  output[0] = PMCV_CMD_FLASH_CMD;
  output[1] = flags;

  // must be first command after an address. by setting the controller's
  // current address to the initial value, it will force the set address
  // command.
  pc_->currentAddr_ = -1;
  return pc_->commandByAxis(axisNo_, output, output_length, NULL, 0, 1);
}

asynStatus PMCVAxis::readFirmware(char &firmware_id, char &version) {
  const int input_length = 2;
  char input[input_length];

  asynStatus ret = pc_->commandByAxis(axisNo_, PMCV_CMD_READ_FIRMWARE, input, input_length);

  if (ret != asynSuccess)
    return ret;

  if (input[PMCV_FIRMWARE_ID] != PMCV_FIRMWARE_ID_MAGIC) {
    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
      "%s:%s: Firmware id magic mismatch: %x != %x (expected)\n", 
      driverName, __FUNCTION__, input[PMCV_FIRMWARE_ID], PMCV_FIRMWARE_ID_MAGIC);
    return asynError;
  }

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: Firmware id: %x version: %x\n", 
    driverName, __FUNCTION__, input[PMCV_FIRMWARE_ID], input[PMCV_FIRMWARE_VERSION]);

  firmware_id = input[PMCV_FIRMWARE_ID];
  version = input[PMCV_FIRMWARE_VERSION];
  return asynSuccess;
}


/** Creates a new PMCVController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] PMCVPortName     The name of the drvAsynIPPPort that was created previously to connect to the PMCV controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
PMCVController::PMCVController(const char *portName, const char *PMCVPortName, int numAxes,
                             int movingPollPeriod, int idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_PMCV_PARAMS,
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask,
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  PMCVAxis *pAxis;

  ackMode_ = true;
  currentAddr_ = -1;
  if (!addToList(portName, this)) {
    printf("%s:%s: Init failed", driverName, portName);
    return;
  }

  movingPollPeriod_ = movingPollPeriod;
  idlePollPeriod_ = idlePollPeriod;

  createParam(PMCV_STATUS_STRING           , asynParamOctet,   &statusParam_);
  createParam(PMCV_BOX_FIRMWARE_STRING     , asynParamInt32,   &boxFirmwareParam_);
  createParam(PMCV_VOLTAGE_ERROR_STRING    , asynParamInt32,   &voltageErrorParam_);
  createParam(PMCV_SERIAL_ERROR_STRING     , asynParamInt32,   &serialErrorParam_);
  createParam(PMCV_HARDWARE_ERROR_STRING   , asynParamInt32,   &hardwareErrorParam_);
  createParam(PMCV_ANYBUS_ERROR_STRING     , asynParamInt32,   &anybusErrorParam_);
  createParam(PMCV_RESET_STATUS_STRING     , asynParamInt32,   &resetStatusParam_);
  createParam(PMCV_ERROR_STRING            , asynParamInt32,   &errorParam_);
  createParam(PMCV_EXT_COMM_STRING         , asynParamInt32,   &externalCommErrorParam_);
  createParam(PMCV_INT_ERR_STRING          , asynParamInt32,   &internalErrorParam_);
  createParam(PMCV_LOST_RESPONSE_STRING    , asynParamInt32,   &lostResponseBytesParam_);
  createParam(PMCV_ENC_PARALLEL_STRING     , asynParamInt32,   &encParallel_);
  createParam(PMCV_ENC_NOISE_FILTER_STRING , asynParamInt32,   &encNoiseFilter_);
  createParam(PMCV_ENC_SYNC_MODE_STRING    , asynParamInt32,   &encSyncMode_);
  createParam(PMCV_ENC_SWAP_AB_STRING      , asynParamInt32,   &encSwapAb_);
  createParam(PMCV_ENC_QUAD_MODE_STRING    , asynParamInt32,   &encQuadMode_);
  createParam(PMCV_ENC_SPI_MODE_STRING     , asynParamInt32,   &encSpiMode_);
  createParam(PMCV_ENC_FREQUENCY_STR_STRING, asynParamOctet,   &encFrequencyStr_);
  createParam(PMCV_ENC_FREQUENCY_STRING    , asynParamInt32,   &encFrequency_);
  createParam(PMCV_DRV_PARALLEL_STRING     , asynParamInt32,   &drvParallel_);
  createParam(PMCV_DRV_SILENT_STRING       , asynParamInt32,   &drvSilent_);
  createParam(PMCV_DRV_PARK_STRING         , asynParamInt32,   &drvPark_);
  createParam(PMCV_DRV_STOP_INDEX_STRING   , asynParamInt32,   &drvStopIndex_);
  createParam(PMCV_DRV_POS_RESET_STRING    , asynParamInt32,   &drvPosReset_);
  createParam(PMCV_DRV_TARGET_MODE_STRING  , asynParamInt32,   &drvTargetMode_);
  createParam(PMCV_DRV_SYNC_MODE_STRING    , asynParamInt32,   &drvSyncMode_);
  createParam(PMCV_SYNC_DZ_STRING          , asynParamInt32,   &syncDz_);
  createParam(PMCV_MICRO_DELAY_STRING      , asynParamFloat64, &microDelay_);
  createParam(PMCV_WAVEFORM_RES_STRING     , asynParamInt32,   &waveformRes_);
  createParam(PMCV_WAVEFORM_RES_STR_STRING , asynParamOctet,   &waveformResStr_);
  createParam(PMCV_SPEED_RAMP_UP           , asynParamInt32,   &speedRampUp_);
  createParam(PMCV_SPEED_RAMP_DOWN         , asynParamInt32,   &speedRampDown_);
  createParam(PMCV_WPC_STRING              , asynParamInt32,   &wpc_);
  createParam(PMCV_FIRMWARE_ID_STRING      , asynParamInt32,   &firmwareId_);
  createParam(PMCV_FIRMWARE_VERSION_STRING , asynParamInt32,   &firmwareVersion_);
  createParam(PMCV_CLEAR_BOX_ERRORS_STRING , asynParamInt32,   &clearBoxErrors_);
  createParam(PMCV_MIN_HZ_STRING           , asynParamInt32,   &minHz_);
  createParam(PMCV_MAX_HZ_STRING           , asynParamInt32,   &maxHz_);

  for (int i=0; i < PMCV_VOLTAGE_COUNT; i++) {
    createParam(PMCV_VOLTAGE_STRINGS[i],  asynParamInt32, &voltageParams_[i]);
  }

  for (int i=0; i < PMCV_BUTTON_COUNT; i++) {
    createParam(PMCV_BUTTON_STRINGS[i],   asynParamInt32, &buttonParams_[i]);
  }

  setStringParam(statusParam_, "");
  timeout_ = PMCV_TIMEOUT;

  /* Connect to PMCV controller */
  status = pasynOctetSyncIO->connect(PMCVPortName, 0, &pasynUser_, NULL);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
      "%s:%s: cannot connect to PMCV controller\n",
      driverName, __FUNCTION__);
  }

  status = pasynOctetSyncIO->setInputEos(pasynUser_, NULL, 0);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
      "%s: Unable to set input EOS on %s: %s\n",
      __FUNCTION__, PMCVPortName, pasynUser_->errorMessage);
    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
      "%s: OK to ignore if processEos is disabled\n",
      __FUNCTION__);
  }

  status = pasynOctetSyncIO->setOutputEos(pasynUser_, NULL, 0);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
      "%s: Unable to set output EOS on %s: %s\n",
      __FUNCTION__, PMCVPortName, pasynUser_->errorMessage);
    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
      "%s: OK to ignore if processEos is disabled\n",
      __FUNCTION__);
  }

  // Create the axis objects
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new PMCVAxis(this, axis);
  }


  callParamCallbacks();

  // TODO: fix
  movingPollPeriod = 50;
  idlePollPeriod = 250;
  printf("Moving poll period: %d idle poll period: %d\n", movingPollPeriod, idlePollPeriod);
  startPoller(((double)movingPollPeriod)/1000., ((double)idlePollPeriod)/1000., 2);
}


int
PMCVController::check_error() {
  /*
        asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
          "%s:%s: %s\n",
          driverName, __FUNCTION__, err_str);
  */
  return asynSuccess;
}

asynStatus PMCVAxis::stop(double acceleration)
{
  return stop();
}

asynStatus PMCVAxis::moveVelocity(double min_velocity, double max_velocity, double acceleration) {
  // TODO
  return asynError;
}

asynStatus PMCVAxis::home(double min_velocity, double max_velocity, double acceleration, int forwards) {
  // TODO
  return asynError;
}

asynStatus PMCVAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
  
  asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: axis %d: move to %g\n",
    driverName, __FUNCTION__, id_, position);
  printf("%s:%s: axis %d: move to %g\n",
    driverName, __FUNCTION__, id_, position);

  return setTarget((epicsInt32)position, true);
}

bool pmcv_decode_uint16(epicsUInt16 *position, char* buffer) {
  // position from controller, should be endian-safe
  if (!position || !buffer)
    return false;

  if (is_big_endian()) {
    // untested
    union {
      epicsUInt16 i;
      char c[2];
    } reversed;
    
    reversed.c[0] = buffer[1];
    reversed.c[1] = buffer[0];

    *position = reversed.i;
  } else {
    memcpy(position, buffer, sizeof(epicsUInt16));
  }

  return true;
}

bool pmcv_decode_uint32(epicsUInt32 *position, char* buffer) {
  // position from controller, should be endian-safe
  if (!position || !buffer)
    return false;

  if (is_big_endian()) {
    // untested
    union {
      epicsUInt32 i;
      char c[4];
    } reversed;
    
    for (int i=0; i < 4; i++) {
      reversed.c[i] = buffer[3 - i];
    }

    *position = reversed.i;
  } else {
    memcpy(position, buffer, sizeof(epicsUInt32));
  }

  return true;
}

bool pmcv_encode_2byte(char* buffer) {
  if (!buffer)
    return false;

  uchar B0=0, B1=1;
  char encoded[2];

  if (is_big_endian()) {
    // note: untested
    B0=1;
    B1=0;
  }

  // 16 - 2 = 14 bit number
  encoded[B0] = buffer[B0] & ~(0x80);
  encoded[B1] = (((0x80 & buffer[B0]) >> 7) | (buffer[B1] << 1)) & ~(0x80);

  for (int i=0; i < 2; i++) {
    encoded[i] &= ~(0x80); //clear MSb
  }

  memcpy(buffer, encoded, 2);
  return true;
}

bool pmcv_encode_4byte(uchar* buffer) {
  if (!buffer)
    return false;

  uchar B0=0, B1=1, B2=2, B3=3;
  uchar encoded[4];

  if (is_big_endian()) {
    // note: untested
    B0=3;
    B1=2;
    B2=1;
    B3=0;
  }

  // 28-bit address. this means each byte's MSB is zeroed out
  // in effect, this shifts around the bits in a confusing
  // fashion that can only be explained concisely on pen and paper...
  encoded[B0] = buffer[B0]; // LSB
  encoded[B1] = ((0x80 & buffer[B0]) >> 7) | (buffer[B1] << 1);
  encoded[B2] = ((0xC0 & buffer[B1]) >> 6) | (buffer[B2] << 2);
  encoded[B3] = ((0xE0 & buffer[B2]) >> 5) | (buffer[B3] << 3); // MSB

  for (int i=0; i < 4; i++) {
    encoded[i] &= ~(0x80); //clear MSb
  }

  memcpy(buffer, encoded, 4);

  return true;
}

bool pmcv_encode_uint16(epicsUInt16 position, char* buffer) {
  // position -> controller, endian-safety untested
  if (!buffer)
    return false;

  memcpy(buffer, &position, sizeof(epicsUInt16));
  return pmcv_encode_2byte(buffer);
}

bool pmcv_encode_uint32(epicsUInt32 position, char* buffer) {
  // position -> controller, endian-safety untested
  if (!buffer)
    return false;

  memcpy(buffer, &position, sizeof(epicsUInt32));
  return pmcv_encode_4byte((uchar*)buffer);
}

bool pmcv_decode_int32(epicsInt32 *position, char* buffer) {
  // position from controller, should be endian-safe
  if (!position || !buffer)
    return false;

  if (is_big_endian()) {
    // untested
    union {
      epicsInt32 i;
      char c[4];
    } reversed;
    
    for (int i=0; i < 4; i++) {
      reversed.c[i] = buffer[3 - i];
    }

    *position = reversed.i;
  } else {
    memcpy(position, buffer, sizeof(epicsInt32));
  }

  return true;
}

bool pmcv_encode_int32(epicsInt32 position, char* buffer) {
  // position -> controller, endian-safety untested
  if (!buffer)
    return false;

  memcpy(buffer, &position, sizeof(epicsInt32));
  return pmcv_encode_4byte((uchar*)buffer);
}

int is_big_endian(void) {
  union {
    epicsUInt32 i;
    char c[4];
  } bint = { 0x01000000 };

  return bint.c[0] == 1; 
}

bool PMCVController::updateStatus(char status_byte) {
  bool errors=((status_byte & PMCV_STATUS_ERROR_MASK) != 0);
 
  int buttons[PMCV_BUTTON_COUNT];
  buttons[0] = (status_byte & PMCV_STATUS_BUTTON_0) ? 1 : 0;
  buttons[1] = (status_byte & PMCV_STATUS_BUTTON_1) ? 1 : 0;
  buttons[2] = (status_byte & PMCV_STATUS_BUTTON_2) ? 1 : 0;

  for (int i=0; i < PMCV_BUTTON_COUNT; i++) {
    setIntegerParam(buttonParams_[i], buttons[i]);
  }

  setIntegerParam(voltageErrorParam_,  ((status_byte & PMCV_STATUS_VOLT_ERROR) ? 1 : 0));
  setIntegerParam(serialErrorParam_,   ((status_byte & PMCV_STATUS_SERIAL_ERROR) ? 1 : 0));
  setIntegerParam(hardwareErrorParam_, ((status_byte & PMCV_STATUS_HARDWARE_ERROR) ? 1 : 0));
  setIntegerParam(anybusErrorParam_,   ((status_byte & PMCV_STATUS_ANYBUS_ERROR) ? 1 : 0));
  setIntegerParam(resetStatusParam_,   ((status_byte & PMCV_STATUS_RESET) ? 1 : 0));

  setIntegerParam(errorParam_, (errors ? 1 : 0));

  if (errors) {
    asynPrint(pasynUser_, ASYN_TRACE_FLOW | ASYN_TRACE_ERROR,
        "%s:%s: Errors reported: voltage: %d serial: %d hardware: %d anybus: %d reset status: %d\n",
        driverName, __FUNCTION__, 
        ((status_byte & PMCV_STATUS_VOLT_ERROR) ? 1 : 0),
        ((status_byte & PMCV_STATUS_SERIAL_ERROR) ? 1 : 0),
        ((status_byte & PMCV_STATUS_HARDWARE_ERROR) ? 1 : 0),
        ((status_byte & PMCV_STATUS_ANYBUS_ERROR) ? 1 : 0),
        ((status_byte & PMCV_STATUS_RESET) ? 1 : 0)
      );
  }

  callParamCallbacks();
  return errors;
}

asynStatus PMCVController::readBoxStatus() {
  uchar buf[1];

  asynStatus ret = command(PMCV_CMD_READ_BOX_STATUS, (char*)buf, 1);
  if (ret != asynSuccess)
    return ret;

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: Firmware id: %x\n", 
    driverName, __FUNCTION__, buf[PMCV_FIRMWARE_ID]);

  setIntegerParam(boxFirmwareParam_, buf[PMCV_FIRMWARE_ID]);

  char status_byte = buf[0];
  updateStatus(status_byte);

  return asynSuccess;
}

asynStatus PMCVController::clearBoxErrors() {
  uchar buf[5];

  asynStatus ret = command(PMCV_CMD_CLEAR_BOX_ERRORS, (char*)buf, 5);
  if (ret != asynSuccess)
    return ret;

  updateStatus(buf[PMCV_STATUS_BYTE]);

  setIntegerParam(externalCommErrorParam_, (buf[PMCV_EXT_BYTE] & PMCV_EXT_ERROR_MASK));
  setIntegerParam(internalErrorParam_, buf[PMCV_INT_BYTE]);
  // TODO: "bytes 3, 4: rcon=0 or else an unexpected reset occured" <-- what does this mean?
  return asynSuccess;
}

asynStatus PMCVController::readBoxFirmware() {
  uchar input[7];

  asynStatus ret = command(PMCV_CMD_READ_BOX_FIRMWARE, (char*)input, 7);
  if (ret != asynSuccess)
    return ret;

  if (input[PMCV_FIRMWARE_ID] != PMCV_FIRMWARE_ID_MAGIC) {
    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
      "%s:%s: Firmware id magic mismatch: %x != %x (expected)\n", 
      driverName, __FUNCTION__, input[PMCV_FIRMWARE_ID], PMCV_FIRMWARE_ID_MAGIC);
    return asynError;
  }

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: Firmware id: %x version: %x\n", 
    driverName, __FUNCTION__, input[PMCV_FIRMWARE_ID], input[PMCV_FIRMWARE_VERSION]);

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: 48V: %d 24V: %d 5V sensor: %d\n", 
    driverName, __FUNCTION__, 
    input[PMCV_FIRMWARE_48V], input[PMCV_FIRMWARE_24V], input[PMCV_FIRMWARE_5V_SENSOR]);

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: 5V: %d 3V: %d\n", 
    driverName, __FUNCTION__, 
    input[PMCV_FIRMWARE_5V], input[PMCV_FIRMWARE_3V]);
 
  voltages[PMCV_VOLTAGES_48V] = (uint)(input[PMCV_FIRMWARE_48V]);
  voltages[PMCV_VOLTAGES_24V] = (uint)(input[PMCV_FIRMWARE_24V]);
  voltages[PMCV_VOLTAGES_5V_SENSOR] = (uint)(input[PMCV_FIRMWARE_5V_SENSOR]);
  voltages[PMCV_VOLTAGES_5V] = (uint)(input[PMCV_FIRMWARE_5V]);
  voltages[PMCV_VOLTAGES_3V] = (uint)(input[PMCV_FIRMWARE_3V]);

  for (int i=0; i < PMCV_VOLTAGE_COUNT; i++) {
    setIntegerParam(voltageParams_[i], voltages[i]);
  }

  return asynSuccess;
}

asynStatus PMCVController::commandByAxis(int axis, char* command, int cmd_length, char *in_buffer, uint expected_length, uint ack_amount) {
  if (axis < 0 || axis > PMCV_AXIS_COUNT) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
      "%s:%s: invalid axis specified: %d\n",
      driverName, __FUNCTION__, axis);
    return asynError;
  }
  // axis 0 = addr 2, axis 1 = addr 3, etc.
  return commandByAddr(axis + 2, command, cmd_length, in_buffer, expected_length, ack_amount);
}

asynStatus PMCVController::commandByAxis(int axis, char command, char *in_buffer, uint expected_length, uint ack_amount) {
  if (axis < 0 || axis > PMCV_AXIS_COUNT) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
      "%s:%s: invalid axis specified: %d\n",
      driverName, __FUNCTION__, axis);
    return asynError;
  }
  // axis 0 = addr 2, axis 1 = addr 3, etc.
  return commandByAddr(axis + 2, command, in_buffer, expected_length, ack_amount);
}

asynStatus PMCVController::commandByAddr(int addr, char *command, int cmd_length, char *in_buffer, uint expected_length, uint ack_amount) {
  int nread;

  for (int i=0; i < cmd_length; i++) {
    assert(!PMCV_IS_ADDRESS_COMMAND(command[i]));
  }

  if (addr == currentAddr_) {
    return writeRead(command, cmd_length, in_buffer, expected_length, nread, ack_amount);
  } else {
    char *out_buffer = new char[cmd_length + 1];
    out_buffer[0] = PMCV_BUILD_ADDR_SET(addr);
    memcpy(&out_buffer[1], command, cmd_length);

    if (addr >= 2 && addr <= 5)
        // additional ack will be sent by the box
        ack_amount++;
    asynStatus ret = writeRead(out_buffer, cmd_length + 1, in_buffer, expected_length, nread, ack_amount);

    delete [] out_buffer;
    return ret;
  }
}

asynStatus PMCVController::commandByAddr(int addr, char command, char *in_buffer, uint expected_length, uint ack_amount) {
  int nread;

  assert(!PMCV_IS_ADDRESS_COMMAND(command));

  if (addr == currentAddr_) {
    char out_buffer[1] = { command };
    return writeRead(out_buffer, 1, in_buffer, expected_length, nread, ack_amount);
  } else {
    char out_buffer[2] = { PMCV_BUILD_ADDR_SET(addr), command };
    return writeRead(out_buffer, 2, in_buffer, expected_length, nread, ack_amount+1);
  }
}

asynStatus PMCVController::command(char command, char *in_buffer, uint expected_length, uint ack_amount) {
  int nread;
  char out_buffer[1] = { command };
  return writeRead(out_buffer, 1, in_buffer, expected_length, nread, ack_amount);
}

asynStatus PMCVController::writeRead(char *out_buffer, size_t out_length, char *in_buffer, uint expected_length, int &read, uint ack_amount) {
  size_t nwrite;
  size_t nread;
  asynStatus status;
  int eomReason;
  char buffer[PMCV_BUFFER_SIZE];

  asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER,
    "%s:%s: Write (%db)\n",
    driverName, __FUNCTION__, out_length);

  for (uint i=0; i < out_length; i++) {
    if (PMCV_IS_ADDRESS_COMMAND(out_buffer[i])) {
      currentAddr_ = PMCV_ADDR_FROM_COMMAND(out_buffer[i]);
    }
  }

  if (ack_amount > 0 && ackMode_) {
    expected_length += ack_amount;
  }

  lock();

  if (expected_length > 0) {
    status = pasynOctetSyncIO->writeRead(pasynUser_,
                                         out_buffer, out_length,
                                         buffer, expected_length,
                                         timeout_, &nwrite, &nread, &eomReason);
  } else {
    status = pasynOctetSyncIO->write(pasynUser_,
                                     out_buffer, out_length,
                                     timeout_, &nwrite);
  }

  unlock();

  /*if (nwrite > 0) {
    printf("write: ");
    for (uint i=0; i < nwrite; i++) {
      printf("%x ", (uchar)out_buffer[i]);
    }
  }

  if (nread > 0) {
    printf("read: ");
    for (uint i=0; i < nread; i++) {
      printf("%x ", (uchar)buffer[i]);
    }
  }

  if (nwrite > 0 || nread > 0)
    printf("\n");*/

  if (nwrite != out_length) {
    asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER|ASYN_TRACE_ERROR,
      "%s:%s: wrote (%db) expected %d\n",
      driverName, __FUNCTION__, nwrite, out_length);
    return asynError;
  }
  if (nread != expected_length) {
    asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER|ASYN_TRACE_ERROR,
      "%s:%s: Read (%db) expected %d (+ack amount %d) = %d\n",
      driverName, __FUNCTION__, nread, expected_length - ack_amount, ack_amount, expected_length);
    return asynError;
  }

  if (ack_amount > 0 && expected_length >= 1) {
    for (uint i=0; i < ack_amount; i++) {
      if (buffer[i] == PMCV_ACK) {
        // good
      } else if (buffer[i] == PMCV_NAK) {
        status = asynError;
      } else if (buffer[i] == PMCV_BEL) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
          "%s:%s: BEL received for command %x\n",
          driverName, __FUNCTION__, (uchar)out_buffer[i]);
      }
    }
  }

  if (in_buffer)
    memcpy(in_buffer, &buffer[ack_amount], nread - ack_amount);

  return status;
}

/* 
void position_test() {
  uchar buf[4];
  int position=10000000;
  pmcv_encode_int32(position, (char*)buf);
  printf("position=%d: %x %x %x %x\n", position, buf[0], buf[1], buf[2], buf[3]);
  position *= -1;
  pmcv_encode_int32(position, (char*)buf);
  printf("position=%d: %x %x %x %x\n", position, buf[0], buf[1], buf[2], buf[3]);
  uchar buf[4];
  for (int position=0; position <= 2000; position += 100) {
    pmcv_encode_int32(position, (char*)buf);
    printf("position=%d: %x %x %x %x\n", position, buf[0], buf[1], buf[2], buf[3]);
  }

  pmcv_encode_int32(200, (char*)buf);
  printf("%x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);

  pmcv_encode_int32(300, (char*)buf);
  printf("%x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);

  union {
    epicsInt32 i;
    char c[4];
  } pos;

  pos.c[0] = 0x0;
  pos.c[1] = 0x0;
  pos.c[2] = 0x33;
  pos.c[3] = 0x2;
  //pmcv_decode_int32(&pos, (char*)pos.c);
  printf("%x %x %x %x = %d\n", pos.c[0], pos.c[1], pos.c[2], pos.c[3], pos.i);

  pos.c[0] = 0x2;
  pos.c[1] = 0x33;
  pos.c[2] = 0x0;
  pos.c[3] = 0x0;

  //pmcv_decode_int32(&pos, (char*)pos.c);
  printf("%x %x %x %x = %d\n", pos.c[0], pos.c[1], pos.c[2], pos.c[3], pos.i);
}
*/

asynStatus PMCVController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  const char *paramName = "(unset)";
  PMCVAxis *pAxis;
  int axis;
  asynStatus ret = asynSuccess;

  pAxis = (PMCVAxis*)asynMotorController::getAxis(pasynUser);
  if (!pAxis) return asynError;
  axis = pAxis->axisNumber();

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  if (function == motorHighLimit_ || function == motorLowLimit_) {
    epicsInt32 limitA, limitB;
    if (pAxis->readEncLimits(limitA, limitB) == asynSuccess) {
      if (function == motorHighLimit_)
        limitB = (epicsInt32)value;
      else
        limitA = (epicsInt32)value;

      ret = pAxis->setEncLimits(limitA, limitB);
    }
  } else if (function == microDelay_) {
    ret = pAxis->setMicroDelay(value);
  } else {

  }
 
  if (ret != asynSuccess)
    return ret;

  return asynMotorController::writeFloat64(pasynUser, value);
}


asynStatus PMCVController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  const char *paramName = "(unset)";
  PMCVAxis *pAxis;
  int axis;

  pAxis = (PMCVAxis*)asynMotorController::getAxis(pasynUser);
  if (!pAxis) return asynError;
  axis = pAxis->axisNumber();

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  asynStatus ret = asynSuccess;
  if (function == clearBoxErrors_ && value == 1) {
    printf("Clear box errors...\n");
    ret = clearBoxErrors();
  } else if (function == drvPark_) {
    ret = pAxis->setPark(value != 0);
  } else if (function == drvStopIndex_ || function == drvPosReset_) {
    epicsInt32 pos_reset, stop_index;

    if (function == drvStopIndex_) {
      getIntegerParam(drvPosReset_, &pos_reset);
      stop_index = value;
    } else {
      pos_reset = value;
      getIntegerParam(drvStopIndex_, &stop_index);
    }
    ret = pAxis->setIndexMode(pos_reset, stop_index);

  } else if (function == speedRampUp_ || function == speedRampDown_) {
    epicsInt32 ramp_up, ramp_down;

    if (function == speedRampUp_) {
      ramp_up = value;
      getIntegerParam(speedRampDown_, &ramp_down);
    } else {
      getIntegerParam(speedRampUp_, &ramp_up);
      ramp_down = value;
    }

    if (ramp_up < 0 || ramp_down < 0) {
      ret = asynError;
    } else if ((epicsUInt16)ramp_up != ramp_up || (epicsUInt16)ramp_down != ramp_down) {
      ret = asynError;
    } else {
      ret = pAxis->setSpeedRamp(ramp_up, ramp_down);
    }

  } else if (function == maxHz_ || function == minHz_) {
    epicsInt32 max_hz, min_hz;

    if (function == maxHz_) {
      getIntegerParam(minHz_, &min_hz);
      max_hz = value;
    } else {
      min_hz = value;
      getIntegerParam(maxHz_, &max_hz);
    }

    if (max_hz < 0 || min_hz < 0) {
      ret = asynError;
    } else if ((epicsUInt16)max_hz != max_hz || (epicsUInt16)min_hz != min_hz) {
      ret = asynError;
    } else {
      ret = pAxis->setSpeedLimits(min_hz, max_hz);
    }

  } else if (function == syncDz_) {
    ret = pAxis->setWPC(value);
  } else if (function == waveformRes_) {
    if ((uchar)value > 0x0F || value < 0) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
        "%s:%s: Invalid wfmres 0x%x. See docs.\n",
        driverName, portName, (uchar)value);
      ret = asynError;
    } else {
      ret = pAxis->setWfmRes(true, (PMCVAxis::waveformResolution)value);
    }
  } else if (function == drvParallel_ || function == drvSyncMode_) {
    epicsInt32 parallel, sync;
    if (function == drvParallel_) {
      getIntegerParam(axis, drvParallel_, &parallel);
      sync = value;
    } else {
      getIntegerParam(axis, drvSyncMode_, &sync);
      parallel = value;
    }
    bool normal = !(parallel || sync); // TODO: correct?
    ret = pAxis->setSync(parallel, normal, sync);
  } else if (function == drvTargetMode_) {
    ret = pAxis->targetModeOn();
  } else if (function == wpc_) {
    if (value > 0)
      ret = pAxis->setWPC((epicsUInt32)value);
    else
      ret = asynError;
  }

  if (ret != asynSuccess)
    return ret;

  ret = asynMotorController::writeInt32(pasynUser, value);
  
  if (function == encNoiseFilter_ ||
      function == encSwapAb_      ||
      function == encQuadMode_    ||
      function == encSpiMode_     ||
      function == encFrequency_) {
    return pAxis->updateEncModeFromParams();
  }

  return ret;
}

asynStatus PMCVAxis::updateEncModeFromParams() {
  epicsInt32 spi, quad, swap_ab, noise_filter; //, sync_mode, parallel;
  epicsInt32 frequency;

  //pc_->getIntegerParam(axisNo_, pc_->encParallel_   , &parallel); <-- R/O
  //pc_->getIntegerParam(axisNo_, pc_->encSyncMode_   , &sync_mode); <-- R/O
  pc_->getIntegerParam(axisNo_, pc_->encNoiseFilter_, &noise_filter);
  pc_->getIntegerParam(axisNo_, pc_->encSwapAb_     , &swap_ab);
  pc_->getIntegerParam(axisNo_, pc_->encQuadMode_   , &quad);
  pc_->getIntegerParam(axisNo_, pc_->encSpiMode_    , &spi);
  pc_->getIntegerParam(axisNo_, pc_->encFrequency_  , &frequency);

  encoderFrequency freq;
  if (pmcv_frequency_index_to_enum(frequency, &freq) != asynSuccess) {
    return asynError;
  }

  return setEncMode(spi, quad, freq, swap_ab, noise_filter);
}

