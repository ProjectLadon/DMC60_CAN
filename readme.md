# ROS Interface for the Digilent DMC60 Motor Controller

This package provides an interface node for controlling a Digilent DMC60C motor controller over CAN using the socketcan_bridge package.

# Dependencies

## ROS Packages
* [ros_canopen][https://github.com/ros-industrial/ros_canopen]
* [ros_control][https://github.com/ros-controls/ros_control]
* [roslint][https://github.com/ros/roslint]
* [filters][https://github.com/ros/filters]
* [pluginlib][https://github.com/ros/pluginlib]
* [class_loader][https://github.com/ros/class_loader]
* [cmake_modules][https://github.com/ros/cmake_modules]

## System Dependencies
* [muparser][https://github.com/beltoforion/muparser]
* [tinyXML2][https://github.com/leethomason/tinyxml2]

# Startup Sequence
The first thing this node does is attempt to enumerate the devices attached to the CAN bus.

After enumeration is complete, this module shall set the parameters on each hardware device as described in the parameters section.

If all enumerated devices have unique device numbers, they will be published (and start accepting commands) immediately upon enumeration.

# Topics
All topics use the device number as the motor ID. Therefore, the structure of topic names is ``/<node_name>/<motor_id>/<topic_name>`` or ``/<node_name>/<motor_id>/<category_name>/<topic_name>``.

## Subscribed Topics

### ModeSelect (UInt8)
| Control Mode | Mode Name | Description |
| --- | --- | --- |
| 0 | modeVoltage | Open Loop Voltage Control (Duty Cycle) |
| 1 | modeVelocity | Closed Loop Velocity Control |
| 2 | modePosition | Closed Loop Position Control |
| 3 | modeCurrent | Closed Loop Current Control |
| 4 | modeVComp | Voltage Compensation Mode |
| 5 | modeFollower | Slave Follower Mode |
| 6-14 | modeReserved | Reserved for future use |
| 15 | modeNoDrive | No Drive (output disabled) |

All devices shall be set to No Drive until messages to the contrary are received on this topic.

### BrakeModeOverride (Bool)

### BrakeModeOverrideSetting (Bool)

### MotorControlProfileSlot (UInt8)

### ReverseFeedbackSensor (Bool)

### ReverseMotor (Bool)

### DisableForwardLimit (Bool)

### DisableReverseLimit (Bool)

### EnableLimitOverride (Bool)

### Target (Int32)
Target value. Meaning depends on selected mode.

### VoltageRamp (UInt16)

## Published Topics
These are all of the topics published for each motor enumerated on startup.

### GeneralStatus (UInt64)
Raw general status message

#### GeneralStatus/AppliedDutyCycle (Int16)
Output duty cycle applied to the H-bridge

#### GeneralStatus/FaultStatus (UInt32)
Fault status bytes, raw value

#### GeneralStatus/FwdLimitPin (Bool)
True when forward limit pin is high, false otherwise

#### GeneralStatus/FwdLimitHit (Bool)
True when forward limit is active, false otherwise

#### GeneralStatus/FwdLimitDisabled (Bool)
True when forward limit is disabled, false when forward limit is enabled

#### GeneralStatus/FwdLimitNormalClosed (Bool)
True when forward limit switch is normally closed, false when normally open

#### GeneralStatus/RevLimitPin (Bool)
True when the reverse limit pin is high, false otherwise

#### GeneralStatus/RevLimitHit (Bool)
True when the reverse limit is active, false otherwise

#### GeneralStatus/RevLimitDisabled (Bool)
True when the reverse limit is disabled, false when reverse limit is enabled

#### GeneralStatus/RevLimitNormalClosed (Bool)
True when reverse limit switch is normally closed, false when normally open

#### GeneralStatus/FwdRevLimitOverride (Bool)
True when limit switch enable state is being overridden by the control frame, false otherwise

#### GeneralStatus/FwdLimitDisableOvrd (Bool)
True when forward limit switch is disabled by the control frame, false when forward limit switch is enabled by the control frame

#### GeneralStatus/RevLimitDisableOvrd (Bool)
True when reverse limit switch is disabled by the control frame, false when reverse limit switch is enabled by the control frame

#### GeneralStatus/SoftFwdLimitHit (Bool)
True when forward soft limit is active, false otherwise

#### GeneralStatus/SoftFwdLimitEnabled (Bool)
True when soft forward limit is enabled, false when disabled

#### GeneralStatus/SoftRevLimitHit (Bool)
True when reverse limit is active, false otherwise

#### GeneralStatus/SoftRevLimitEnabled (Bool)
True when soft reverse limit is enabled, false when disabled

#### GeneralStatus/CurrentLimitActive (Bool)
True when the current limit is being enforced, false otherwise

#### GeneralStatus/ClosedLoopErr (Int32)
Closed loop error. Meaning depends on currently selected mode.

#### GeneralStatus/OverTempFault (Bool)
True when over temperature fault is active, false otherwise

#### GeneralStatus/UnderVoltageFault (Bool)
True when under voltage fault is active, false otherwise

#### GeneralStatus/GateDriverFault (Bool)
True when bridge driver fault is active, false otherwise

#### GeneralStatus/ModeSelect (UInt8)
| Control Mode | Mode Name | Description |
| --- | --- | --- |
| 0 | modeVoltage | Open Loop Voltage Control (Duty Cycle) |
| 1 | modeVelocity | Closed Loop Velocity Control |
| 2 | modePosition | Closed Loop Position Control |
| 3 | modeCurrent | Closed Loop Current Control |
| 4 | modeVComp | Voltage Compensation Mode |
| 5 | modeFollower | Slave Follower Mode |
| 6-14 | modeReserved | Reserved for future use |
| 15 | modeNoDrive | No Drive (output disabled) |

### EncoderStatus (UInt64)
Raw Encoder Status

#### EncoderStatus/Position (Int32)
Encoder’s current position count. Units are native to the encoder being used.

#### EncoderStatus/Velocity (Int16)
Velocity count in native units per 100 milliseconds

#### EncoderStatus/Flags (Uint8)
Raw value of encoder flags

#### EncoderStatus/PinA (Bool)
True when quadrature encoder A pin is high, false otherwise.

#### EncoderStatus/PinB (Bool)
True when quadrature encoder B pin is high, false otherwise.

#### EncoderStatus/IndexPin (Bool)
True when quadrature encoder Indexpin is high, false otherwise.

### AnalogStatus (UInt64)
Raw Analog Status

#### AnalogStatus/AIN1 (Float32)
Voltage at AIN1 in volts

#### AnalogStatus/CurrentOut (Float32)
Load current in amps

#### AnalogStatus/CaseTemp (Float32)
Internal case temperature in degrees C

#### AnalogStatus/BusVoltage (Float32)
Battery bus voltage in volts

# Services
These services correspond to vendor commands for manipulating the device.

## get_dev_list
Returns a list of the enumerated devices and session ids, in JSON format
```
bool dummy
---
bool success
string dev_list
uint8 error
```

## set_dev_number
Sets the device number of the device with the given device number or session number to the given value. If the session_id and old_dev_id do not name the same device, the session_id takes precedence. If the new device id is already in use, this service will fail. Old publications and subscriptions are deleted and re-made for the new device ID.
```
uint8 old_dev_id
uint8 session_id
uint8 new_dev_id
---
bool success
uint8 new_dev_id
uint8 error
```

## get_descriptors
Returns the device descriptors of the identified device. If the device id is non-zero, it will query the device with that id; otherwise it will use the session id.
```
uint8 dev_id
uint16 session_id
---
bool success
uint8 error
string dev_name
string mfg_name
string prod_name
string mfg_date
string hardware_version
string serial_num
```

## flash_leds
Flashes the LEDs in a rainbow pattern for five seconds.
```
uint8 dev_id
uint16 session_id
---
bool success
uint8 error
```

## reset
Perform a software reset
```
uint8 dev_id
uint16 session_id
---
bool success
uint8 error
```

# Parameters
Parameters may be set on a per-device basis or for all devices connected to the node. If the parameter name is used as shown, then it will apply to all devices not otherwise specified. Parameters may be specified on a per-device basis by appending ``_<dev_num>`` to the parameter name.

If these parameters exist on startup, this node will set them on the target hardware after startup enumeration and when the device number changes. Note that not all parameters available on the device are defined here. See DMC60C CAN Protocol Guide for more information on the format of each one. Note that conversions from ordinary floats to the fixed point versions used by the protocol are handled internally.

| Parameter Name | Function | Default Value | Type |
| --- | --- | --- |
| ADCcurrentMult | Multiplier used to convert ADC readings into current measurements | 0x0816 | float (16.16 signed) |
| ProportionalGainSlot0 | Slot 0 closed loop proportional gain | 0 | float (16.16 signed) |
| IntegralGainSlot0 | Slot 0 closed loop integral gain | 0 | float (16.16 signed) |
| DifferentialGainSlot0 | Slot 0 closed loop differential gain | 0 | float (16.16 signed) |
| IntegralLimitSlot0 | Slot 0 closed loop integral accumulator limit | 0 | unsigned integer (31 bit) |
| ForwardGainSlot0 | Slot 0 closed loop feed-forward gain | 0 | float (16.16 signed) |
| AllowableErrorSlot0 | Slot 0 closed loop allowable error | 0 | unsigned integer (31 bit) |
| RampLimitSlot0 | Slot 0 closed loop max ramp rate | 0 (disabled) | unsigned integer (31 bit) |
| MaxFwdSlot0 | Slot 0 max forward duty cycle | 32767 | integer |
| MaxRevSlot0 | Slot 0 max reverse duty cycle | -32767 | integer |
| MonimalFwdSlot0 | Slot 0 nominal forward duty cycle | 0 | integer |
| MonimalRevSlot0 | Slot 0 nominal reverse duty cycle | 0 | integer |
| ProportionalGainSlot1 | Slot 1 closed loop proportional gain | 0 | float (16.16 signed) |
| IntegralGainSlot1 | Slot 1 closed loop integral gain | 0 | float (16.16 signed) |
| DifferentialGainSlot1 | Slot 1 closed loop differential gain | 0 | float (16.16 signed) |
| IntegralLimitSlot1 | Slot 1 closed loop integral accumulator limit | 0 | unsigned integer (31 bit) |
| ForwardGainSlot1 | Slot 1 closed loop feed-forward gain | 0 | float (16.16 signed) |
| AllowableErrorSlot1 | Slot 1 closed loop allowable error | 0 | unsigned integer (31 bit) |
| RampLimitSlot1 | Slot 1 closed loop max ramp rate | 0 (disabled) | unsigned integer (31 bit) |
| MaxFwdSlot1 | Slot 1 max forward duty cycle | 32767 |  integer |
| MaxRevSlot1 | Slot 1 max reverse duty cycle | -32767 |  integer |
| NominalFwdSlot1 | Slot 1 nominal forward duty cycle | 0 | integer |
| NominalRevSlot1 | Slot 1 nominal reverse duty cycle | 0 | integer |
| ProportionalCurrentLimiterGain | Current limiter proportional gain | 0x00640000 | float (16.16 signed) |
| IntegralCurrentLimiterGain | Current limiter integral gain | 0x003C0000 | float (16.16 signed) |
| DifferentialCurrentLimiterGain | Current limiter differential gain | 0 | float (16.16 signed) |
| IntegralCurrentLimiterLimit | Current limiter integral accumulator limit | 0x014CC888 | unsigned integer (31 bit) |
| ForwardCurrentLimiterGain | Current limiter feed-forward gain | 0 | float (16.16 signed) |
| ContinuousCurrentLimit | Continuous current limit, amps | 40 | float (16.16 signed) |
| PeakCurrentLimit | Peak current limit, amps | 60 | float (16.16 signed) |
| PeakCurrentDuration | Peak current duration, ms | 500 | unsigned integer (16 bit) |
| CurrentLimitEnable | Current limit enable status | 0 (disabled) | int (bool) |
