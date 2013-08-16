#ifndef _PMCV_H
#define _PMCV_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cmath>

#include <iocsh.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsExport.h>

#include <asynOctetSyncIO.h>
#include "asynMotorController.h"
#include "asynMotorAxis.h"

#ifndef uint
typedef unsigned int uint;
typedef unsigned char uchar;
#endif

#define PMCV_BUFFER_SIZE          60
#define PMCV_ACK                  0x06 // simple acknowledgement
#define PMCV_BEL                  0x07 // unexpected/modified parameter (but acknowledged otherwise)
#define PMCV_NAK                  0x15 // not performed

#define PMCV_TIMEOUT              0.5
#define PMCV_UNIT_SCALE           1.0e6

#define PMCV_AXIS_COUNT           4
#define PMCV_FULL_UPDATE_RATE     10 // every n polls, will query the rest of the axis information

/* Asyn parameter strings */
#define PMCV_STATUS_STRING         "PMCV_STATUS"
#define PMCV_ERROR_STRING          "PMCV_ERROR"
#define PMCV_VOLTAGE_ERROR_STRING  "PMCV_VOLTAGE_ERROR"
#define PMCV_SERIAL_ERROR_STRING   "PMCV_SERIAL_ERROR"
#define PMCV_HARDWARE_ERROR_STRING "PMCV_HARDWARE_ERROR"
#define PMCV_ANYBUS_ERROR_STRING   "PMCV_ANYBUS_ERROR"
#define PMCV_RESET_STATUS_STRING   "PMCV_RESET_STATUS"
#define PMCV_BOX_FIRMWARE_STRING   "PMCV_BOX_FIRMWARE"

#define PMCV_EXT_COMM_STRING       "PMCV_EXT_COMM_ERROR"
#define PMCV_INT_ERR_STRING        "PMCV_INT_COMM_ERROR"
#define PMCV_LOST_RESPONSE_STRING  "PMCV_LOST_RESPONSE"

#define PMCV_ENC_PARALLEL_STRING      "PMCV_ENC_PARALLEL"
#define PMCV_ENC_NOISE_FILTER_STRING  "PMCV_ENC_NOISE_FILTER"
#define PMCV_ENC_SYNC_MODE_STRING     "PMCV_ENC_SYNC_MODE"
#define PMCV_ENC_SWAP_AB_STRING       "PMCV_ENC_SWAP_AB"
#define PMCV_ENC_QUAD_MODE_STRING     "PMCV_ENC_QUAD_MODE"
#define PMCV_ENC_SPI_MODE_STRING      "PMCV_ENC_SPI_MODE"
#define PMCV_ENC_FREQUENCY_STR_STRING "PMCV_ENC_FREQUENCY_STR"
#define PMCV_ENC_FREQUENCY_STRING     "PMCV_ENC_FREQUENCY"

#define PMCV_DRV_PARALLEL_STRING      "PMCV_DRV_PARALLEL"
#define PMCV_DRV_SILENT_STRING        "PMCV_DRV_SILENT"
#define PMCV_DRV_PARK_STRING          "PMCV_DRV_PARK"
#define PMCV_DRV_STOP_INDEX_STRING    "PMCV_DRV_STOP_INDEX"
#define PMCV_DRV_POS_RESET_STRING     "PMCV_DRV_POS_RESET"
#define PMCV_DRV_TARGET_MODE_STRING   "PMCV_DRV_TARGET_MODE"
#define PMCV_DRV_SYNC_MODE_STRING     "PMCV_DRV_SYNC_MODE"

#define PMCV_MIN_HZ_STRING            "PMCV_MIN_HZ"
#define PMCV_MAX_HZ_STRING            "PMCV_MAX_HZ"
#define PMCV_SYNC_DZ_STRING           "PMCV_SYNC_DZ"
#define PMCV_MICRO_DELAY_STRING       "PMCV_MICRO_DELAY"

#define PMCV_WAVEFORM_RES_STRING      "PMCV_WAVEFORM_RES"
#define PMCV_WAVEFORM_RES_STR_STRING  "PMCV_WAVEFORM_RES_STR"
#define PMCV_SPEED_RAMP_UP            "PMCV_SPEED_RAMP_UP"
#define PMCV_SPEED_RAMP_DOWN          "PMCV_SPEED_RAMP_DOWN"

#define PMCV_WPC_STRING               "PMCV_WPC"
#define PMCV_FIRMWARE_ID_STRING       "PMCV_FIRMWARE_ID"
#define PMCV_FIRMWARE_VERSION_STRING  "PMCV_FIRMWARE_VERSION"
#define PMCV_CLEAR_BOX_ERRORS_STRING  "PMCV_CLEAR_BOX_ERRORS"

static const char *PMCV_BUTTON_STRINGS[] = { 
  "PMCV_BUTTON_1",
  "PMCV_BUTTON_2",
  "PMCV_BUTTON_3",
};

static const char *PMCV_VOLTAGE_STRINGS[] = { 
  "PMCV_48V",
  "PMCV_24V",
  "PMCV_5V_SENSOR",
  "PMCV_5V",
  "PMCV_3V"
};

#define PMCV_DEFAULT_MIN_HZ          40
#define PMCV_DEFAULT_MAX_HZ          2200
#define PMCV_MAX_SPEED_HZ            2200
#define PMCV_DEFAULT_SPEED_RAMP      40

/* Commands */
#define PMCV_CMD_READ_BOX_STATUS     0x87
#define PMCV_CMD_CLEAR_BOX_ERRORS    0x84
#define PMCV_CMD_READ_BOX_FIRMWARE   0x85

#define PMCV_CMD_STOP                0x00
#define PMCV_CMD_READ_POS            0x01
#define PMCV_CMD_SET_TARGET          0x02
#define PMCV_CMD_READ_STATUS1        0x03
#define PMCV_CMD_TARGET_MODE_ON      0x04
#define PMCV_CMD_READ_TARGET         0x05
#define PMCV_CMD_SET_INDEXMODE       0x06
#define PMCV_CMD_READ_DRVMODE        0x07
#define PMCV_CMD_SET_SYNC_PARK       0x08
#define PMCV_CMD_SET_ENCLIMITS       0x0a
#define PMCV_CMD_READ_ENC_LIMITS     0x0b
#define PMCV_CMD_SET_WPC             0x0c
#define PMCV_CMD_READ_WPC            0x0d
#define PMCV_CMD_SET_SPEED_RAMP      0x0e
#define PMCV_CMD_READ_SPEED_RAMPS    0x0f
#define PMCV_CMD_SET_SPEED_LIMITS    0x10
#define PMCV_CMD_READ_SPEED_LIMITS   0x11
#define PMCV_CMD_SET_POS             0x12
#define PMCV_CMD_SET_DZ              0x14
#define PMCV_CMD_READ_DZ             0x15
#define PMCV_CMD_STOP_RESTORE        0x16
#define PMCV_CMD_SET_ENC_MODE        0x1a
#define PMCV_CMD_READ_ENC_MODE       0x1b
#define PMCV_CMD_SET_WFM_RES         0x1e
#define PMCV_CMD_READ_WFM_RES        0x1f
#define PMCV_CMD_RUN_STEPS           0x20
#define PMCV_CMD_READ_STEPS          0x21
#define PMCV_CMD_SET_MICRO_DELAY     0x22
#define PMCV_CMD_READ_MICRO_DELAY    0x23
#define PMCV_CMD_SET_WFM_FREQ        0x24
#define PMCV_CMD_NO_OPERATION        0x26
#define PMCV_CMD_CLEAR_ERRORS        0x2e
#define PMCV_CMD_READ_STATUS2        0x2f
#define PMCV_CMD_SET_USERBYTE        0x30
#define PMCV_CMD_READ_USERBYTE       0x31
#define PMCV_CMD_READ_FLASH_POS      0x37
#define PMCV_CMD_FLASH_CMD           0x38
#define PMCV_CMD_READ_FIRMWARE       0x3d

// build address
#define PMCV_BUILD_ADDRESS(addr, cc)         (0x80 | (((addr) & 0x1F) << 2) | ((cc) & 0x3))
#define PMCV_BUILD_BROADCAST(addr, cc)       PMCV_BUILD_ADDRESS(0, cc)
#define PMCV_BUILD_AXIS(axis, cc)            PMCV_BUILD_ADDRESS(axis + 2, cc) // 0-based axes

#define PMCV_BUILD_AXIS_SET(axis)                 PMCV_BUILD_AXIS(axis, 0)
#define PMCV_BUILD_AXIS_READ_POS(axis)            PMCV_BUILD_AXIS(axis, 1)
#define PMCV_BUILD_AXIS_SET_TARGET(axis)          PMCV_BUILD_AXIS(axis, 2)
#define PMCV_BUILD_AXIS_READ_STATUS(axis)         PMCV_BUILD_AXIS(axis, 3)

#define PMCV_BUILD_ADDR_SET(addr)                 PMCV_BUILD_ADDRESS(addr, 0)
#define PMCV_BUILD_ADDR_READ_POS(addr)            PMCV_BUILD_ADDRESS(addr, 1)
#define PMCV_BUILD_ADDR_SET_TARGET(addr)          PMCV_BUILD_ADDRESS(addr, 2)
#define PMCV_BUILD_ADDR_READ_STATUS(addr)         PMCV_BUILD_ADDRESS(addr, 3)

#define PMCV_IS_ADDRESS_COMMAND(byte)        ((0x80 & (byte)) == 0x80)
#define PMCV_ADDR_FROM_COMMAND(byte)         (((byte) >> 2) & 0x1F)

int is_big_endian(void);

bool pmcv_encode_2byte(char *buffer);
bool pmcv_encode_uint16(epicsUInt16 position, char* buffer);
bool pmcv_decode_uint16(epicsUInt16 *position, char* buffer);

bool pmcv_encode_4byte(char *buffer);
bool pmcv_encode_uint32(epicsUInt32 position, char* buffer);
bool pmcv_decode_uint32(epicsUInt32 *position, char* buffer);
bool pmcv_encode_int32(epicsInt32 position, char* buffer);
bool pmcv_decode_int32(epicsInt32 *position, char* buffer);

// readBoxStatus bits
#define PMCV_STATUS_BYTE             0
#define PMCV_STATUS_BUTTON_0         (1 << 0)
#define PMCV_STATUS_BUTTON_1         (1 << 1)
#define PMCV_STATUS_BUTTON_2         (1 << 2)
#define PMCV_STATUS_VOLT_ERROR       (1 << 3)
#define PMCV_STATUS_SERIAL_ERROR     (1 << 4)
#define PMCV_STATUS_HARDWARE_ERROR         (1 << 5)
#define PMCV_STATUS_ANYBUS_ERROR     (1 << 6)
#define PMCV_STATUS_RESET            (1 << 7)
#define PMCV_STATUS_ERROR_MASK       0x78
#define PMCV_BUTTON_COUNT           3

/* clearBoxErrors response */
// external comm status (byte 1)
#define PMCV_EXT_BYTE                    1
#define PMCV_EXT_LOST_AXIS_BYTES_MASK    0x0F
#define PMCV_EXT_ERROR_MASK              0xF0   // TODO this order could be reversed
#define PMCV_EXT_FRAME_ERROR             (1 << 5)
#define PMCV_EXT_RX_BUFFER_ERROR         (1 << 6)
#define PMCV_EXT_RX_OVERFLOW             (1 << 7)

// internal comm status (byte 2)
#define PMCV_INT_BYTE                 2
#define PMCV_INT_3V                   (1 << 0)   // TODO this order could be reversed
#define PMCV_INT_5V                   (1 << 1)
#define PMCV_INT_5V_SENSOR            (1 << 2)
#define PMCV_INT_24V                  (1 << 3)
#define PMCV_INT_48V                  (1 << 4)
#define PMCV_INT_RX_FRAME_ERROR       (1 << 5)
#define PMCV_INT_RX_BUFFER_ERROR      (1 << 6)
#define PMCV_INT_RX_OVERFLOW          (1 << 7)

// bytes 3, 4 = 0 or there was an unexpected reset
#define PMCV_UNEXPECTED_RESET         0

/* readBoxFirmware response */
#define PMCV_FIRMWARE_ID_MAGIC       0x56
#define PMCV_FIRMWARE_ID             0
#define PMCV_FIRMWARE_VERSION        1
#define PMCV_FIRMWARE_48V            2
#define PMCV_FIRMWARE_24V            3
#define PMCV_FIRMWARE_5V_SENSOR      4
#define PMCV_FIRMWARE_5V             5
#define PMCV_FIRMWARE_3V             6
#define PMCV_NOMINAL_VOLTAGE         193

// PMC voltages
#define PMCV_VOLTAGES_48V            0
#define PMCV_VOLTAGES_24V            1
#define PMCV_VOLTAGES_5V_SENSOR      2
#define PMCV_VOLTAGES_5V             3
#define PMCV_VOLTAGES_3V             4
#define PMCV_VOLTAGE_COUNT           5

// setSyncPark flags
#define PMCV_SYNC_PARK_ACKMODE       0x40
#define PMCV_SYNC_PARK_SILENT        0x20
#define PMCV_SYNC_PARK_PARALLEL      0x10
#define PMCV_SYNC_PARK_UNPARK        0x08
#define PMCV_SYNC_PARK_PARK          0x04
#define PMCV_SYNC_PARK_NORMAL        0x02
#define PMCV_SYNC_PARK_SYNC          0x01

#define PMCV_SYNC_PARK_DEFAULT       (PMCV_SYNC_PARK_NORMAL |  \
                                      PMCV_SYNC_PARK_ACKMODE | \
                                      PMCV_SYNC_PARK_UNPARK)
// setEncMode flags
#define PMCV_ENC_MODE_PARALLEL      0x80 // read-only
#define PMCV_ENC_MODE_NOISE_FILTER  0x40
#define PMCV_ENC_MODE_SWAP_AB       0x20
#define PMCV_ENC_MODE_QUAD_B        0x10
#define PMCV_ENC_MODE_QUAD_A        0x08
#define PMCV_ENC_MODE_QUAD_MODE     0x04
#define PMCV_ENC_MODE_SPI_MODE      0x02
#define PMCV_ENC_MODE_SYNC_MODE     0x01 // read-only

#define PMCV_ENC_110KHZ             0
#define PMCV_ENC_440KHZ             PMCV_ENC_MODE_QUAD_A
#define PMCV_ENC_1_8MHZ             PMCV_ENC_MODE_QUAD_B
#define PMCV_ENC_7MHZ               (PMCV_ENC_MODE_QUAD_A|PMCV_ENC_MODE_QUAD_B)

#define PMCV_ENC_FREQ_MASK          PMCV_ENC_7MHZ


// setIndexMode flags
#define PMCV_SET_INDEX_MODE_POS_RESET      0x04 // set position = 0 if index occurs
#define PMCV_SET_INDEX_MODE_STOP_INDEX     0x08 // stop if sensor index occurs

// readDrvMode flags
#define PMCV_DRV_MODE_PARALLEL             0x80
#define PMCV_DRV_MODE_SILENT               0x40
#define PMCV_DRV_MODE_PARK                 0x20
//#define PMCV_DRV_MODE_UNUSED               0x10
#define PMCV_DRV_MODE_STOP_INDEX           0x08
#define PMCV_DRV_MODE_POS_RESET            0x04
#define PMCV_DRV_MODE_TARGET_MODE          0x02
#define PMCV_DRV_MODE_SYNC_MODE            0x01

// readWfmRes flags
#define PMCV_WFM_RES_RUN                  0x20

#define PMCV_WFM_RES_WFM1                 0x08
#define PMCV_WFM_RES_WFM0                 0x04
#define PMCV_WFM_RES_RES1                 0x02
#define PMCV_WFM_RES_RES0                 0x01
#define PMCV_WFM_RES_MASK                 0x0F

#define PMCV_OMEGA_2048                   0x0F
#define PMCV_OMEGA_1024                   0x0E
#define PMCV_OMEGA_512                    0x0D
#define PMCV_OMEGA_256                    0x0C
#define PMCV_OMEGA_256_                   0x0B // same as above
#define PMCV_OMEGA_128                    0x0A
#define PMCV_OMEGA_64                     0x09
#define PMCV_OMEGA_32                     0x08

#define PMCV_RHOMB_256                    0x07 // same as above
#define PMCV_RHOMB_128                    0x06
#define PMCV_RHOMB_64                     0x05
#define PMCV_RHOMB_32                     0x04

#define PMCV_RHOMBF_256                   0x03 // same as above
#define PMCV_RHOMBF_128                   0x02
#define PMCV_RHOMBF_64                    0x01
#define PMCV_RHOMBF_32                    0x00

static const char *PMCV_WFM_RES_STRINGS[] = { 
  "PMCV_RHOMBF_32",
  "PMCV_RHOMBF_64",
  "PMCV_RHOMBF_128",
  "PMCV_RHOMBF_256",
  "PMCV_RHOMB_32",
  "PMCV_RHOMB_64",
  "PMCV_RHOMB_128",
  "PMCV_RHOMB_256",
  "PMCV_OMEGA_32",
  "PMCV_OMEGA_64",
  "PMCV_OMEGA_128",
  "PMCV_OMEGA_256_",
  "PMCV_OMEGA_256",
  "PMCV_OMEGA_512",
  "PMCV_OMEGA_1024",
  "PMCV_OMEGA_2048",
};

// runSteps flags
#define PMCV_RUN_STEPS_RUN             0x40
#define PMCV_RUN_STEPS_DIRECTION       0x20
#define PMCV_RUN_STEPS_MSB_MASK        0x07

// microDelay
#define PMCV_US_PER_TICK               0.2286

// flashCmd flags
#define PMCV_FLASH_RESTORE_SAVED             0x01
#define PMCV_FLASH_RESTORE_DEFAULT           0x02
#define PMCV_FLASH_SAVE_SETTINGS             0x04

// flashPos (8 byte response)
#define PMCV_FLASHPOS_LENGTH                 8 // bytes total
#define PMCV_FLASHPOS_POSITION_START         0 // 4 bytes
#define PMCV_FLASHPOS_CHECK1_BYTE            4
#define PMCV_FLASHPOS_CHECK2_BYTE            5
#define PMCV_FLASHPOS_CHECKSUM_START         6 // 2 bytes

#define PMCV_FLASHPOS_CHECK1_VALUE           0xAA
#define PMCV_FLASHPOS_CHECK2_VALUE           0x55

static const char* driverName = "PMCV";
class PMCVController;

class PMCVAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  PMCVAxis(PMCVController *pC, int axis);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  int axisNumber() { return axisNo_; }

  enum encoderFrequency {
    enc_110khz = 0,
    enc_440khz = PMCV_ENC_MODE_QUAD_A,
    enc_1_8mhz = PMCV_ENC_MODE_QUAD_B,
    enc_7mhz = (PMCV_ENC_MODE_QUAD_A|PMCV_ENC_MODE_QUAD_B),
  };

  enum waveformResolution {
    rhombf_32 = PMCV_RHOMBF_32,
    rhombf_64 = PMCV_RHOMBF_64,
    rhombf_128 = PMCV_RHOMBF_128,
    rhombf_256 = PMCV_RHOMBF_256,
    rhomb_32 = PMCV_RHOMB_32,
    rhomb_64 = PMCV_RHOMB_64,
    rhomb_128 = PMCV_RHOMB_128,
    rhomb_256 = PMCV_RHOMB_256,
    omega_32 = PMCV_OMEGA_32,
    omega_64 = PMCV_OMEGA_64,
    omega_128 = PMCV_OMEGA_128,
    omega_256_ = PMCV_OMEGA_256_,
    omega_256 = PMCV_OMEGA_256,
    omega_512 = PMCV_OMEGA_512,
    omega_1024 = PMCV_OMEGA_1024,
    omega_2048 = PMCV_OMEGA_2048,
  };

  /* And these are specific to this class: */
  asynStatus stop();

  asynStatus setPosition(epicsInt32 position);
  asynStatus readPosition(epicsInt32 &position);

  asynStatus readTarget(epicsInt32 &target);
  asynStatus setTarget(epicsInt32 target, bool move=true);

  asynStatus readStatus1();
  asynStatus readStatus2();

  asynStatus targetModeOn();
  asynStatus setIndexMode(char flags);
  asynStatus setIndexMode(bool position_reset, bool stop);

  asynStatus readDrvMode(char flags);
  asynStatus readDrvMode(bool &parallel, bool &silent, bool &park, bool &stop_index, bool &pos_reset, bool &target_mode, bool &sync_mode);

  asynStatus setSyncPark(bool ack_mode, bool silent_mode, bool parallel, bool unpark, bool park, bool normal, bool sync);
  asynStatus setSyncPark(char flags);
  asynStatus setPark(bool park); // uses setSyncPark
  asynStatus setSync(bool parallel, bool normal, bool sync); // uses setSyncPark

  asynStatus setEncLimits(epicsInt32 low, epicsInt32 high);
  asynStatus readEncLimits(epicsInt32 &limit_a, epicsInt32 &limit_b);

  asynStatus setWPC(epicsUInt32 wpc, bool is_calculated_wpc=false);
  asynStatus readWPC(epicsUInt32 &wpc);

  asynStatus setSpeedRamp(epicsUInt16 rampUp, epicsUInt16 rampDown);
  asynStatus readSpeedRamps(epicsUInt16 &rampUp, epicsUInt16 &rampDown);

  asynStatus setSpeedLimits(epicsUInt16 minHz, epicsUInt16 maxHz);
  asynStatus readSpeedLimits(epicsUInt16 &minHz, epicsUInt16 &maxHz);

  asynStatus setDz(epicsInt32 dz);
  asynStatus readDz(epicsInt32 &dz);

  asynStatus stopRestore();
  asynStatus setEncMode(unsigned char flags);
  asynStatus setEncMode(bool spi, bool quad, encoderFrequency frequency, bool swap_ab, bool noise_filter);
  asynStatus readEncMode(char &flags);
  asynStatus readEncMode(bool &spi, bool &quad, encoderFrequency &frequency, bool &swap_ab, bool &noise_filter, bool &sync_mode, bool &parallel);
  asynStatus updateEncModeFromParams();

  asynStatus setWfmRes(char flags);
  asynStatus setWfmRes(bool run, waveformResolution res);
  asynStatus readWfmRes(bool &run, waveformResolution &res);

  asynStatus runSteps(epicsUInt32 steps, bool direction, bool run=true);
  asynStatus readSteps(epicsUInt32 &steps);

  asynStatus setMicroDelay(double us_delay);
  asynStatus setMicroDelay(epicsUInt32 ticks);
  asynStatus readMicroDelay(epicsUInt32 &ticks);
  asynStatus readMicroDelay(double &us_delay);
  
  asynStatus setWfmFreq(epicsUInt16 frequency);
  asynStatus noOperation();
  asynStatus clearErrors();

  asynStatus setUserbyte(epicsUInt32 userbyte);
  asynStatus readUserbyte(epicsUInt32 &userbyte);

  asynStatus readFlashPos(epicsUInt16 &checksum, epicsInt32 &position);
  asynStatus flashCmd(char flags);
  asynStatus flashSaveSettings();
  asynStatus flashRestoreSettings();
  asynStatus flashRestoreDefaults();

  asynStatus readFirmware(char &firmware_id, char &version);

  void fullUpdate();
  void updateStatus(unsigned char status_byte);

  inline bool isFlagSet(uint flag) { return (flags_ & flag) == flag; }
  inline void setFlag(uint flag)   { flags_ |= flag; }
  inline void clearFlag(uint flag) { flags_ &= ~flag; }
  inline void setFlag(uint flag, bool set) {
    if (set)
      flags_ |= flag;
    else
      flags_ &= ~flag;
  }

private:
  friend class PMCVController;
  PMCVController *pc_;      /**< Pointer to the asynMotorController to which this axis belongs.
                              *   Abbreviated because it is used very frequently */
  int encoderPos_;   /**< Cached copy of the encoder position */
  uint flags_;       /**< Cached copy of the current flags */

  int axisNum_;         // according to asyn
  int id_;              // axis num according to controller
  int statusFailed_;    // number of consecutive times status queries have failed
  int pollNumber_;

  // From readStatus1 (or readStatus2):
  bool anyError_;
  bool targetLimit_;
  bool hitLimitA_, hitLimitB_;
  bool indexPass_;
  bool targetPass_;
  bool direction_;
  bool moving_;

  epicsInt32 limitA_, limitB_;

  // From readStatus2:
  bool flashErr_;
  bool syncErr_;
  bool comBuffErr_;
  bool comFrameErr_;
  bool encErr_;
  bool drvErr_;
  bool overheat_;
  bool cmdErr_;
  bool firmErr_;
  bool resetErr_;

};

// if status queries fail (n) times in a row, restart the ioc
#define PMCV_STATUS_FAILED_THRESHOLD 100

class PMCVController : public asynMotorController {
public:
  PMCVController(const char *portName, const char *PMCVPortName, int numAxes, int movingPollPeriod, int idlePollPeriod);
  PMCVAxis* getAxis(int axisNo) {
    return (PMCVAxis*)asynMotorController::getAxis(axisNo);
  }

  int getAxisCount() { return numAxes_; }

  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus command(char command, char *in_buffer, uint expected_length, uint ack_amount=0);
  asynStatus commandByAxis(int axis, char command, char *in_buffer, uint expected_length, uint ack_amount=0);
  asynStatus commandByAxis(int axis, char *command, int cmd_length, char *in_buffer, uint expected_length, uint ack_amount=0);
  asynStatus commandByAddr(int addr, char command, char *in_buffer, uint expected_length, uint ack_amount=0);
  asynStatus commandByAddr(int addr, char *command, int cmd_length, char *in_buffer, uint expected_length, uint ack_amount=0);
  asynStatus writeRead(char *out_buffer, size_t out_length, char *in_buffer, uint expected_length, int &read, uint ack_amount=0);

  asynStatus readBoxFirmware();
  asynStatus clearBoxErrors();
  asynStatus readBoxStatus();
  bool updateStatus(char status_byte);
  void fullUpdate();

  uint voltages[5];

protected:
#define FIRST_PMCV_PARAM statusParam_
  int statusParam_;
  int voltageErrorParam_;
  int serialErrorParam_;
  int hardwareErrorParam_;
  int anybusErrorParam_;
  int resetStatusParam_;
  int lostResponseBytesParam_;

  int voltageParams_[PMCV_VOLTAGE_COUNT];
  int buttonParams_[PMCV_BUTTON_COUNT];
  int internalErrorParam_;
  int externalCommErrorParam_;
  int boxFirmwareParam_;

  int encParallel_;
  int encNoiseFilter_;
  int encSyncMode_;
  int encSwapAb_;
  int encQuadMode_;
  int encSpiMode_;
  int encFrequency_;
  int encFrequencyStr_;
                    
  int drvParallel_;
  int drvSilent_;
  int drvPark_;
  int drvStopIndex_;
  int drvPosReset_;
  int drvTargetMode_;
  int drvSyncMode_;
                    
  int syncDz_;
  int microDelay_;
                    
  int waveformRes_;
  int waveformResStr_;
  int speedRampUp_;
  int speedRampDown_;

  int minHz_;
  int maxHz_;
                    
  int wpc_;
  int firmwareId_;
  int firmwareVersion_;
  int clearBoxErrors_;

  int errorParam_;
#define LAST_PMCV_PARAM errorParam_
#define NUM_PMCV_PARAMS (&LAST_PMCV_PARAM - &FIRST_PMCV_PARAM + 1)

  int check_error();
  double timeout_;
  int currentAddr_;

  bool ackMode_;

private:
  friend class PMCVAxis;
  asynUser *pasynUser_;

};

/* Use the following structure and functions to manage multiple instances
 * of the driver */
typedef struct PMCVNode {
    ELLNODE node;
    const char *portName;
    PMCVController *pController;
} PMCVNode;

bool addToList(const char *portName, PMCVController *drv);
PMCVController* findByPortName(const char *portName);

const char *pmcv_encoder_frequency_to_string(PMCVAxis::encoderFrequency frequency, int &index);
const char *pmcv_waveform_res_to_string(PMCVAxis::waveformResolution res);
asynStatus pmcv_frequency_index_to_enum(int frequency, PMCVAxis::encoderFrequency *freq);

#endif
