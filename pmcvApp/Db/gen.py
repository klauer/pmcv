format_ = """record(%(rtyp)s, "$(P)$(R)%(pvname)s")
{
    field(DTYP, "%(dtyp)s")
    field(INP,  "@asyn($(PORT),$(ADDR),$(TIMEOUT))%(param)s")
    field(PINI, "%(init)d")
    field(SCAN, "I/O Intr")
}
"""

import re

def make_pvname(name):
    name = name.lower()
    if name.startswith('pmcv_'):
        name = name[len('pmcv_'):]
    if name.endswith('_string'):
        name = name[:-len('_string')]
    def do_upper(m):
        return m.group(0).upper()[1]
    return re.sub('_([a-z])', do_upper, name)

def do_params(params, dtyp='', rtyp='', init=1):
    for param in params:
        pvname = make_pvname(param)
        print(format_ % locals())

bi_params = [
    "PMCV_ERROR",
    "PMCV_VOLTAGE_ERROR",
    "PMCV_SERIAL_ERROR",
    "PMCV_HARDWARE_ERROR",
    "PMCV_ANYBUS_ERROR",
    "PMCV_RESET_STATUS",

    "PMCV_EXT_COMM_ERROR",
    "PMCV_EXT_COMM_ERROR",
    "PMCV_ENC_PARALLEL_STRING",
    "PMCV_ENC_NOISE_FILTER_STRING",
    "PMCV_ENC_SYNC_MODE_STRING",
    "PMCV_ENC_SWAP_AB_STRING",
    "PMCV_ENC_QUAD_MODE_STRING",
    "PMCV_ENC_SPI_MODE_STRING",

    "PMCV_DRV_PARALLEL_STRING",
    "PMCV_DRV_SILENT_STRING",
    "PMCV_DRV_PARK_STRING",
    "PMCV_DRV_STOP_INDEX_STRING",
    "PMCV_DRV_POS_RESET_STRING",
    "PMCV_DRV_TARGET_MODE_STRING",
    "PMCV_DRV_SYNC_MODE_STRING",

    "PMCV_BUTTON_1",
    "PMCV_BUTTON_2",
    "PMCV_BUTTON_3",
]

int_params = [
    "PMCV_FIRMWARE",

    "PMCV_LOST_RESPONSE",

    "PMCV_SYNC_DZ_STRING",

    "PMCV_SPEED_RAMP_UP",
    "PMCV_SPEED_RAMP_DOWN",

    "PMCV_WPC_STRING",
    "PMCV_FIRMWARE_ID_STRING",
    "PMCV_FIRMWARE_VERSION_STRING",

    "PMCV_48V",
    "PMCV_24V",
    "PMCV_5V_SENSOR",
    "PMCV_5V",
    "PMCV_3V",
]

float_params = ["PMCV_MICRO_DELAY_STRING",]
octet_params = ["PMCV_ENC_FREQUENCY_STRING", "PMCV_WAVEFORM_RES_STRING",
    "PMCV_STATUS_STRING",
]


do_params(octet_params, dtyp='asynOctetRead', rtyp='stringin')
do_params(float_params, dtyp='asynFloat64Read', rtyp='ai')
do_params(bi_params, dtyp='asynInt32', rtyp='bi')
do_params(int_params, dtyp='asynInt32', rtyp='longin')
# createParam(PMCV_MICRO_DELAY_STRING     , asynParamFloat64, &microDelay_);
# createParam(PMCV_ENC_FREQUENCY_STRING   , asynParamOctet, &encFrequency_);
# createParam(PMCV_WAVEFORM_RES_STRING    , asynParamOctet, &waveformRes_);

