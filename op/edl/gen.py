from __future__ import print_function
header = """
4 0 1
beginScreenProperties
major 4
minor 0
release 1
x 162
y 362
w 473
h 586
font "helvetica-medium-r-18.0"
ctlFont "helvetica-medium-r-18.0"
btnFont "helvetica-medium-r-18.0"
fgColor index 14
bgColor index 0
textColor index 14
ctlFgColor1 index 14
ctlFgColor2 index 0
ctlBgColor1 index 0
ctlBgColor2 index 14
topShadowColor index 0
botShadowColor index 14
showGrid
snapToGrid
endScreenProperties

# (Rectangle)
object activeRectangleClass
beginObjectProperties
major 4
minor 0
release 0
x 10
y 10
w 450
h 30
lineColor index 14
fill
fillColor index 27
endObjectProperties

# (Static Text)
object activeXTextClass
beginObjectProperties
major 4
minor 1
release 0
x 140
y 20
w 202
h 21
font "helvetica-medium-r-18.0"
fgColor index 0
bgColor index 0
useDisplayBg
value {
  "%(title)s"
}
autoSize
endObjectProperties
"""


text = """
# (Static Text)
object activeXTextClass
beginObjectProperties
major 4
minor 1
release 0
x 80
y %(y)d
w 42
h 21
font "helvetica-medium-r-18.0"
fgColor index 14
bgColor index 0
useDisplayBg
value {
  "%(desc)s"
}
autoSize
endObjectProperties

# (Text Monitor)
object activeXTextDspClass:noedit
beginObjectProperties
major 4
minor 5
release 0
x 10
y %(y)d
w 60
h 21
controlPv "%(pvname)s"
font "helvetica-medium-r-18.0"
fgColor index 14
bgColor index 0
useDisplayBg
autoHeight
limitsFromDb
nullColor index 0
useHexPrefix
newPos
objType "monitors"
endObjectProperties
"""

toggle_text = """
# (Button)
object activeButtonClass
beginObjectProperties
major 4
minor 0
release 0
x 140
y %(y)d
w 70
h 30
fgColor index 14
onColor index 0
offColor index 0
inconsistentColor index 14
topShadowColor index 0
botShadowColor index 14
controlPv "%(pvname)s"
onLabel "On"
offLabel "Off"
labelType "literal"
3d
font "helvetica-medium-r-18.0"
objType "controls"
controlBitPos 0
endObjectProperties

# (Text Monitor)
object activeXTextDspClass:noedit
beginObjectProperties
major 4
minor 5
release 0
x 230
y %(y)d
w 60
h 21
controlPv "%(pvname)s_R"
font "helvetica-medium-r-18.0"
fgColor index 14
bgColor index 0
useDisplayBg
autoHeight
limitsFromDb
nullColor index 0
useHexPrefix
newPos
objType "monitors"
endObjectProperties

# (Static Text)
object activeXTextClass
beginObjectProperties
major 4
minor 1
release 0
x 10
y %(y)d
w 94
h 21
font "helvetica-medium-r-18.0"
fgColor index 14
bgColor index 0
useDisplayBg
value {
  "%(desc)s"
}
autoSize
endObjectProperties
"""

textbox_text = """
# (Text Monitor)
object activeXTextDspClass:noedit
beginObjectProperties
major 4
minor 5
release 0
x 230
y %(y)d
w 60
h 21
controlPv "%(pvname)s_R"
font "helvetica-medium-r-18.0"
fgColor index 14
bgColor index 0
useDisplayBg
autoHeight
limitsFromDb
nullColor index 0
useHexPrefix
newPos
objType "monitors"
endObjectProperties

# (Text Control)
object activeXTextDspClass
beginObjectProperties
major 4
minor 5
release 0
x 140
y %(y)d
w 70
h 21
controlPv "%(pvname)s"
font "helvetica-medium-r-18.0"
fgColor index 14
bgColor index 0
useDisplayBg
editable
autoHeight
limitsFromDb
nullColor index 0
useHexPrefix
newPos
objType "controls"
endObjectProperties

# (Static Text)
object activeXTextClass
beginObjectProperties
major 4
minor 1
release 0
x 10
y %(y)d
w 73
h 21
font "helvetica-medium-r-18.0"
fgColor index 14
bgColor index 0
useDisplayBg
value {
  "%(desc)s"
}
autoSize
endObjectProperties
"""

general = """$(P)$(R)anybusError_R
$(P)$(R)button_1_R
$(P)$(R)button_2_R
$(P)$(R)button_3_R
$(P)$(R)error_R
$(P)$(R)extCommError_R
$(P)$(R)hardwareError_R
$(P)$(R)intCommError_R
$(P)$(R)resetStatus_R
$(P)$(R)serialError_R
$(P)$(R)voltageError_R
$(P)$(R)24v_R
$(P)$(R)3v_R
$(P)$(R)48v_R
$(P)$(R)5vSensor_R
$(P)$(R)5v_R
$(P)$(R)boxFirmware_R
$(P)$(R)lostResponse_R""".split('\n')

axis = """$(P)$(R)$(A)microDelay_R
$(P)$(R)$(A)drvParallel_R
$(P)$(R)$(A)drvPark_R
$(P)$(R)$(A)drvPosReset_R
$(P)$(R)$(A)drvSilent_R
$(P)$(R)$(A)drvStopIndex_R
$(P)$(R)$(A)drvSyncMode_R
$(P)$(R)$(A)drvTargetMode_R
$(P)$(R)$(A)encNoiseFilter_R
$(P)$(R)$(A)encParallel_R
$(P)$(R)$(A)encQuadMode_R
$(P)$(R)$(A)encSpiMode_R
$(P)$(R)$(A)encSwapAb_R
$(P)$(R)$(A)encSyncMode_R
$(P)$(R)$(A)firmwareId_R
$(P)$(R)$(A)firmwareVersion_R
$(P)$(R)$(A)lostResponse_R
$(P)$(R)$(A)speedRampDown_R
$(P)$(R)$(A)speedRampUp_R
$(P)$(R)$(A)syncDz_R
$(P)$(R)$(A)wpc_R
$(P)$(R)$(A)encFrequency_R
$(P)$(R)$(A)status_R
$(P)$(R)$(A)waveformRes_R""".split('\n')

modifiable = """$(P)$(R)$(A)microDelay_R
$(P)$(R)$(A)drvParallel_R
$(P)$(R)$(A)drvPark_R
$(P)$(R)$(A)drvPosReset_R
$(P)$(R)$(A)drvSilent_R
$(P)$(R)$(A)drvStopIndex_R
$(P)$(R)$(A)drvSyncMode_R
$(P)$(R)$(A)drvTargetMode_R
$(P)$(R)$(A)encNoiseFilter_R
$(P)$(R)$(A)encParallel_R
$(P)$(R)$(A)encQuadMode_R
$(P)$(R)$(A)encSpiMode_R
$(P)$(R)$(A)encSwapAb_R
$(P)$(R)$(A)speedRampDown_R
$(P)$(R)$(A)speedRampUp_R
$(P)$(R)$(A)syncDz_R
$(P)$(R)$(A)wpc_R
""".split('\n')

togglable = """$(P)$(R)$(A)drvPark_R
$(P)$(R)$(A)drvPosReset_R
$(P)$(R)$(A)drvSilent_R
$(P)$(R)$(A)drvStopIndex_R
$(P)$(R)$(A)drvSyncMode_R
$(P)$(R)$(A)drvTargetMode_R
$(P)$(R)$(A)encNoiseFilter_R
$(P)$(R)$(A)encParallel_R
$(P)$(R)$(A)encQuadMode_R
$(P)$(R)$(A)encSpiMode_R
$(P)$(R)$(A)encSwapAb_R""".split('\n')
list_ = axis

if list_ == general:
    title = 'PiezoMotor Controller V - General'
else:
    title = 'PiezoMotor Controller V - $(A)'

#print(header % { 'title' : title })

def do_entry(pvname, y):
    desc = pvname

    # too lazy for regexes
    if desc.endswith('_R'):
        desc = desc[:-2]
    desc = desc.replace('_', ' ').replace("$(P)", "").replace("$(R)", "").replace('$(A)', '')
    if pvname not in modifiable:
      #out = read_only % locals()
      return
    else:
      if pvname in togglable:
        pvname = pvname[:-2]
        out = toggle_text % locals()
      else:
        pvname = pvname[:-2]
        out = textbox_text % locals()

    print(out)

y = 300
for info in list_:
    do_entry(info, y)
    y += 40

