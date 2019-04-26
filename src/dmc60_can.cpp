/*
 * Copyright (c) 2019, Pierce Nichols
 *
 * This software is licensed under the General Public License, version 3
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "can_msgs/Frame.h"
#include "dmc60_can/dmc60_can_def.h"
#include "dmc60_can/device.h"
#include <string>
#include <limits>
#include <chrono>
#include <thread>

using namespace std;

DMC60C::DMC60C (uint16_t sessionID,
                uint8_t deviceID,
                string cantx_topic,
                ros::NodeHandle *h) {
    session_id = sessionID;
    device_id = deviceID;
    can_topic = cantx_topic;
    node = h;
    makePublishers();
    subscribe();
}

bool DMC60C::setDeviceNumber(uint8_t deviceID) {
    can_msgs::Frame frame;
    if (deviceID > 63) return false;
    if (deviceID == 0) return false;
    device_id = deviceID;
    frame.is_extended = true;
    frame.id = static_cast<uint32_t>(MsgID::VendorCmd);
    frame.dlc = 0;
    uint16ToData(session_id, frame.data.data(), 0);
    frame.data[2] = static_cast<uint8_t>(VendorCommands::SetDevNumber);
    frame.data[4] = deviceID;
    return true;
}

bool DMC60C::subscribe() {
    if (device_id == 0) return false;
    modeSelectSub               = node->subscribe(to_string(device_id) +
                                                    "/ModeSelect", 1,
                                                    &DMC60C::modeSelectCallback,
                                                    this
                                                );
    brakeModeOverrideSub        = node->subscribe(to_string(device_id) +
                                                    "/BrakeModeOverride", 1,
                                                    &DMC60C::brakeModeOverrideCallback,
                                                    this
                                                );
    brakeModeOverrideSettingSub = node->subscribe(to_string(device_id) +
                                                    "/BrakeModeOverrideSetting", 1,
                                                    &DMC60C::brakeModeOverrideSettingCallback,
                                                    this
                                                );
    motorControlProfileSlotSub  = node->subscribe(to_string(device_id) +
                                                    "/MotorControlProfileSlot", 1,
                                                    &DMC60C::motorControlProfileSlotCallback,
                                                    this
                                                );
    revFdbkSnsrSub              = node->subscribe(to_string(device_id) +
                                                    "/ReverseFeedbackSensor", 1,
                                                    &DMC60C::revFdbkSnsrCallback,
                                                    this
                                                );
    revMotorSub                 = node->subscribe(to_string(device_id) +
                                                    "/ReverseMotor", 1,
                                                    &DMC60C::revMotorCallback,
                                                    this
                                                );
    disableFwdLimitSub          = node->subscribe(to_string(device_id) +
                                                    "/DisableForwardLimit", 1,
                                                    &DMC60C::disableFwdLimitCallback,
                                                    this
                                                );
    disableRevLimitSub          = node->subscribe(to_string(device_id) +
                                                    "/DisableReverseLimit", 1,
                                                    &DMC60C::disableRevLimitCallback,
                                                    this
                                                );
    enableLimitOverrideSub      = node->subscribe(to_string(device_id) +
                                                    "/EnableLimitOverride", 1,
                                                    &DMC60C::enableLimitOverrideCallback,
                                                    this
                                                );
    targetSub                   = node->subscribe(to_string(device_id) +
                                                    "/Target", 1,
                                                    &DMC60C::targetCallback,
                                                    this
                                                );
    voltageRampSub              = node->subscribe(to_string(device_id) +
                                                    "/VoltageRamp", 1,
                                                    &DMC60C::voltageRampCallback,
                                                    this
                                                );
    return true;
}

bool DMC60C::recvFrame(const can_msgs::Frame::ConstPtr& msg) {
    if (!msg->is_extended) return false;
    if (msg->is_error) return false;
    if (msg->id == static_cast<uint8_t>(MsgID::StatusGeneral)) {
        parseGeneralStatus(msg);
    } else if (msg->id == static_cast<uint8_t>(MsgID::StatusEncoder)) {
        parseEncoderStatus(msg);
    } else if(msg->id == static_cast<uint8_t>(MsgID::StatusAnalog)) {
        parseAnalogStatus(msg);
    } else if (msg->id == static_cast<uint8_t>(MsgID::VendorStatus)) {
        parseVendorStatus(msg);
    } else if (msg->id == static_cast<uint8_t>(MsgID::ParamResp)) {
        parseParamRespStatus(msg);
    } else if (msg->id == static_cast<uint8_t>(MsgID::VendorDin)) {
        parseDeviceData(msg);
    } else {
        return false;
    }
    return true;
}

bool DMC60C::setParams() {
    double tmp;
    if ((node->getParam("ADCcurrentMult_" + to_string(device_id), tmp)) ||
        (node->getParam("ADCcurrentMult", tmp))) setFloatParam(ParamID::ADCcurrentMult, tmp);
    if ((node->getParam("ProportionalGainSlot0_" + to_string(device_id), tmp)) ||
        (node->getParam("ProportionalGainSlot0", tmp))) setFloatParam(ParamID::ProportionalGainSlot0, tmp);
    if ((node->getParam("IntegralGainSlot0_" + to_string(device_id), tmp)) ||
        (node->getParam("IntegralGainSlot0", tmp))) setFloatParam(ParamID::IntegralGainSlot0, tmp);
    if ((node->getParam("DifferentialGainSlot0_" + to_string(device_id), tmp)) ||
        (node->getParam("DifferentialGainSlot0", tmp))) setFloatParam(ParamID::DifferentialGainSlot0, tmp);
    if ((node->getParam("IntegralLimitSlot0_" + to_string(device_id), tmp)) ||
        (node->getParam("IntegralLimitSlot0", tmp))) setUIntParam(ParamID::IntegralLimitSlot0, tmp);
    if ((node->getParam("ForwardGainSlot0_" + to_string(device_id), tmp)) ||
        (node->getParam("ForwardGainSlot0", tmp))) setFloatParam(ParamID::ForwardGainSlot0, tmp);
    if ((node->getParam("AllowableErrorSlot0_" + to_string(device_id), tmp)) ||
        (node->getParam("AllowableErrorSlot0", tmp))) setUIntParam(ParamID::AllowableErrorSlot0, tmp);
    if ((node->getParam("RampLimitSlot0_" + to_string(device_id), tmp)) ||
        (node->getParam("RampLimitSlot0", tmp))) setUIntParam(ParamID::RampLimitSlot0, tmp);
    if ((node->getParam("MaxFwdSlot0_" + to_string(device_id), tmp)) ||
        (node->getParam("MaxFwdSlot0", tmp))) setUIntParam(ParamID::MaxFwdSlot0, tmp);
    if ((node->getParam("MaxRevSlot0_" + to_string(device_id), tmp)) ||
        (node->getParam("MaxRevSlot0", tmp))) setUIntParam(ParamID::MaxRevSlot0, tmp);
    if ((node->getParam("NominalFwdSlot0_" + to_string(device_id), tmp)) ||
        (node->getParam("NominalFwdSlot0", tmp))) setUIntParam(ParamID::NominalFwdSlot0, tmp);
    if ((node->getParam("NominalRevSlot0_" + to_string(device_id), tmp)) ||
        (node->getParam("NominalRevSlot0", tmp))) setUIntParam(ParamID::NominalRevSlot0, tmp);
    if ((node->getParam("ProportionalGainSlot1_" + to_string(device_id), tmp)) ||
        (node->getParam("ProportionalGainSlot1", tmp))) setFloatParam(ParamID::ProportionalGainSlot1, tmp);
    if ((node->getParam("IntegralGainSlot1_" + to_string(device_id), tmp)) ||
        (node->getParam("IntegralGainSlot1", tmp))) setFloatParam(ParamID::IntegralGainSlot1, tmp);
    if ((node->getParam("DifferentialGainSlot1_" + to_string(device_id), tmp)) ||
        (node->getParam("DifferentialGainSlot1", tmp))) setFloatParam(ParamID::DifferentialGainSlot1, tmp);
    if ((node->getParam("IntegralLimitSlot1_" + to_string(device_id), tmp)) ||
        (node->getParam("IntegralLimitSlot1", tmp))) setUIntParam(ParamID::IntegralLimitSlot1, tmp);
    if ((node->getParam("ForwardGainSlot1_" + to_string(device_id), tmp)) ||
        (node->getParam("ForwardGainSlot1", tmp))) setFloatParam(ParamID::ForwardGainSlot1, tmp);
    if ((node->getParam("AllowableErrorSlot1_" + to_string(device_id), tmp)) ||
        (node->getParam("AllowableErrorSlot1", tmp))) setUIntParam(ParamID::AllowableErrorSlot1, tmp);
    if ((node->getParam("RampLimitSlot1_" + to_string(device_id), tmp)) ||
        (node->getParam("RampLimitSlot1", tmp))) setUIntParam(ParamID::RampLimitSlot1, tmp);
    if ((node->getParam("MaxFwdSlot1_" + to_string(device_id), tmp)) ||
        (node->getParam("MaxFwdSlot1", tmp))) setUIntParam(ParamID::MaxFwdSlot1, tmp);
    if ((node->getParam("MaxRevSlot1_" + to_string(device_id), tmp)) ||
        (node->getParam("MaxRevSlot1", tmp))) setUIntParam(ParamID::MaxRevSlot1, tmp);
    if ((node->getParam("NominalFwdSlot1_" + to_string(device_id), tmp)) ||
        (node->getParam("NominalFwdSlot1", tmp))) setUIntParam(ParamID::NominalFwdSlot1, tmp);
    if ((node->getParam("NominalRevSlot1_" + to_string(device_id), tmp)) ||
        (node->getParam("NominalRevSlot1", tmp))) setUIntParam(ParamID::NominalRevSlot1, tmp);
    if ((node->getParam("ProportionalCurrentLimiterGain_" + to_string(device_id), tmp)) ||
        (node->getParam("ProportionalCurrentLimiterGain", tmp))) setFloatParam(ParamID::ProportionalCurrentLimiterGain, tmp);
    if ((node->getParam("IntegralCurrentLimiterGain_" + to_string(device_id), tmp)) ||
        (node->getParam("IntegralCurrentLimiterGain", tmp))) setFloatParam(ParamID::IntegralCurrentLimiterGain, tmp);
    if ((node->getParam("DifferentialCurrentLimiterGain_" + to_string(device_id), tmp)) ||
        (node->getParam("DifferentialCurrentLimiterGain", tmp))) setFloatParam(ParamID::DifferentialCurrentLimiterGain, tmp);
    if ((node->getParam("IntegralCurrentLimiterLimit_" + to_string(device_id), tmp)) ||
        (node->getParam("IntegralCurrentLimiterLimit", tmp))) setUIntParam(ParamID::IntegralCurrentLimiterLimit, tmp);
    if ((node->getParam("ForwardCurrentLimiterGain_" + to_string(device_id), tmp)) ||
        (node->getParam("ForwardCurrentLimiterGain", tmp))) setFloatParam(ParamID::ForwardCurrentLimiterGain, tmp);
    if ((node->getParam("ContinuousCurrentLimit_" + to_string(device_id), tmp)) ||
        (node->getParam("ContinuousCurrentLimit", tmp))) setFloatParam(ParamID::ContinuousCurrentLimit, tmp);
    if ((node->getParam("PeakCurrentLimit_" + to_string(device_id), tmp)) ||
        (node->getParam("PeakCurrentLimit", tmp))) setFloatParam(ParamID::PeakCurrentLimit, tmp);
    if ((node->getParam("PeakCurrentDuration_" + to_string(device_id), tmp)) ||
        (node->getParam("PeakCurrentDuration", tmp))) setUIntParam(ParamID::PeakCurrentDuration, tmp);
    if ((node->getParam("CurrentLimitEnable_" + to_string(device_id), tmp)) ||
        (node->getParam("CurrentLimitEnable", tmp))) setUIntParam(ParamID::CurrentLimitEnable, tmp);
}

map<string, string> DMC60C::getDescriptors () {
    can_msgs::Frame frame;
    frame.dlc = 8;
    frame.id = static_cast<uint32_t>(MsgID::VendorCmd);
    frame.data[0] = session_id & 0xff;
    frame.data[1] = (session_id >> 8) & 0xff;
    frame.data[2] = static_cast<uint8_t>(VendorCommands::GetDescriptors);
    vendorRespFresh = false;
    lastVendorErr = VendorCmdErrorCodes::NoError;
    vendorDataExpected = 0;
    canFramePub.publish(frame);
    map<string, string> result;
    for (int i = 0; i < 250; i++) {
        this_thread::sleep_for(1ms);
        ros::spinOnce();
        if (lastVendorErr != VendorCmdErrorCodes::NoError) return result;
        if (vendorRespFresh && (vendorDataExpected > 0) &&
            (vendorDataExpected == vendorDataReceived.size())) {
                uint16_t byteCnt = 0;
                while (byteCnt <= vendorDataReceived.size()) {
                    if (devDescriptors.count(vendorDataReceived[byteCnt])) {
                        if ((vendorDataReceived[byteCnt + 1] + byteCnt) >= vendorDataReceived.size()) break;    // check that we're not going to overrun
                        string tmp;
                        for (int i = vendorDataReceived[byteCnt + 2]; i < (vendorDataReceived[byteCnt + 1] + byteCnt); i++) {   // iterate through the bytes of this field using the field size defined the byte after the field identifier
                            tmp.push_back(static_cast<char>(vendorDataReceived[i]));
                        }
                        result.emplace(devDescriptors.at(vendorDataReceived[byteCnt]), tmp);
                        byteCnt += (vendorDataReceived[byteCnt + 1] + 1);
                    } else break;   // If there's no valid field identifier where we expect it, break
                }
        }
    }
    return result;
}

bool DMC60C::flashLEDs() {
    can_msgs::Frame frame;
    frame.dlc = 8;
    frame.id = static_cast<uint32_t>(MsgID::VendorCmd);
    frame.data[0] = session_id & 0xff;
    frame.data[1] = (session_id >> 8) & 0xff;
    frame.data[2] = static_cast<uint8_t>(VendorCommands::FlashLEDs);
    canFramePub.publish(frame);
    vendorRespFresh = false;
}

bool DMC60C::setDevNum(uint8_t newDev) {
    if (newDev == 0) return false;
    if (newDev > 63) return false;
    can_msgs::Frame frame;
    frame.dlc = 8;
    frame.id = static_cast<uint32_t>(MsgID::VendorCmd);
    frame.data[0] = session_id & 0xff;
    frame.data[1] = (session_id >> 8) & 0xff;
    frame.data[2] = static_cast<uint8_t>(VendorCommands::SetDevNumber);
    frame.data[4] = newDev;
    vendorRespFresh = false;
    lastVendorErr = VendorCmdErrorCodes::NoError;
    canFramePub.publish(frame);
    for (int i = 0; i < 100; i++) {
        this_thread::sleep_for(1ms);
        ros::spinOnce();
        if (vendorRespFresh && (lastVendorErr == VendorCmdErrorCodes::NoError)) return true;
    }
    return false;
}

bool DMC60C::resetHardware() {
    can_msgs::Frame frame;
    frame.id = static_cast<uint32_t>(MsgID::VendorCmd);
    frame.data[2] = static_cast<uint8_t>(VendorCommands::SoftReset);
    vendorRespFresh = false;
    lastVendorErr = VendorCmdErrorCodes::NoError;
    canFramePub.publish(frame);
    for (int i = 0; i < 100; i++) {
        this_thread::sleep_for(1ms);
        ros::spinOnce();
        if (vendorRespFresh && (lastVendorErr == VendorCmdErrorCodes::NoError)) return true;
    }
    return false;
}

void DMC60C::makePublishers() {
    genStatusPub = node->advertise<std_msgs::UInt64>(to_string(device_id) +
                                                    "/GeneralStatus", 1);
    appliedDutyCyclePub = node->advertise<std_msgs::Int16>(to_string(device_id) +
                                                    "/GeneralStatus/AppliedDutyCycle", 1);
    faultStatusPub = node->advertise<std_msgs::UInt32>(to_string(device_id) +
                                                    "/GeneralStatus/FaultStatus", 1);
    fwdLimitPinPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/FwdLimitPin", 1);
    fwdLimitHitPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/FwdLimitHit", 1);
    fwdLimitDisablePub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/FwdLimitDisabled", 1);
    fwdLimitNCPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/FwdLimitNormalClosed", 1);
    revLimitPinPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/RevLimitPin", 1);
    revLimitHitPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/RevLimitHit", 1);
    revLimitDisablePub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/RevLimitDisabled", 1);
    revLimitNCPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/RevLimitNormalClosed", 1);
    fwdRevLimitOverridePub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/FwdRevLimitOverride", 1);
    fwdLimitDisableOvrdPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/FwdLimitDisableOvrd", 1);
    revLimitDisableOvrdPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/RevLimitDisableOvrd", 1);
    softFwdLimitHitPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/SoftFwdLimitHit", 1);
    softFwdLimitEnbPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/SoftFwdLimitEnabled", 1);
    softRevLimitHitPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/SoftRevLimitHit", 1);
    softRevLimitEnbPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/SoftRevLimitEnabled", 1);
    currentLimitActivePub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/CurrentLimitActive", 1);
    closedLoopErrPub = node->advertise<std_msgs::Int32>(to_string(device_id) +
                                                    "/GeneralStatus/ClosedLoopErr", 1);
    overTempFltPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/OverTempFault", 1);
    underVoltFaultPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/UnderVoltageFault", 1);
    gateDriverFaultPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/GateDriverFault", 1);
    modeSelectPub = node->advertise<std_msgs::UInt8>(to_string(device_id) +
                                                    "/GeneralStatus/ModeSelect", 1);
    divErrBy256Pub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/GeneralStatus/DivErrBy256", 1);
    encoderStatusPub = node->advertise<std_msgs::UInt64>(to_string(device_id) +
                                                    "/EncoderStatus", 1);
    positionPub = node->advertise<std_msgs::Int32>(to_string(device_id) +
                                                    "/EncoderStatus/Position", 1);
    velocityPub = node->advertise<std_msgs::Int32>(to_string(device_id) +
                                                    "/EncoderStatus/Velocity", 1);
    encoderFlagsPub = node->advertise<std_msgs::UInt8>(to_string(device_id) +
                                                    "/EncoderStatus/Flags", 1);
    encoderPinAPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/EncoderStatus/PinA", 1);
    encoderPinBPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/EncoderStatus/PinB", 1);
    encoderIndexPinPub = node->advertise<std_msgs::Bool>(to_string(device_id) +
                                                    "/EncoderStatus/IndexPin", 1);
    analogStatusPub = node->advertise<std_msgs::UInt64>(to_string(device_id) +
                                                    "/AnalogStatus", 1);
    AIN1Pub = node->advertise<std_msgs::Float32>(to_string(device_id) +
                                                    "/AnalogStatus/AIN1", 1);
    loadCurrentPub = node->advertise<std_msgs::Float32>(to_string(device_id) +
                                                    "/AnalogStatus/CurrentOut", 1);
    caseTempPub = node->advertise<std_msgs::Float32>(to_string(device_id) +
                                                    "/AnalogStatus/CaseTemp", 1);
    busVoltagePub = node->advertise<std_msgs::Float32>(to_string(device_id) +
                                                    "/AnalogStatus/BusVoltage", 1);
    canFramePub = node->advertise<can_msgs::Frame>(can_topic, 1);
}

void DMC60C::deletePublishers() {
    genStatusPub.shutdown();
    appliedDutyCyclePub.shutdown();
    faultStatusPub.shutdown();
    fwdLimitPinPub.shutdown();
    fwdLimitHitPub.shutdown();
    fwdLimitDisablePub.shutdown();
    fwdLimitNCPub.shutdown();
    revLimitPinPub.shutdown();
    revLimitHitPub.shutdown();
    revLimitDisablePub.shutdown();
    revLimitNCPub.shutdown();
    fwdRevLimitOverridePub.shutdown();
    fwdLimitDisableOvrdPub.shutdown();
    revLimitDisableOvrdPub.shutdown();
    softFwdLimitHitPub.shutdown();
    softFwdLimitEnbPub.shutdown();
    softRevLimitHitPub.shutdown();
    softRevLimitEnbPub.shutdown();
    currentLimitActivePub.shutdown();
    closedLoopErrPub.shutdown();
    overTempFltPub.shutdown();
    underVoltFaultPub.shutdown();
    gateDriverFaultPub.shutdown();
    modeSelectPub.shutdown();
    divErrBy256Pub.shutdown();
    encoderStatusPub.shutdown();
    positionPub.shutdown();
    velocityPub.shutdown();
    encoderFlagsPub.shutdown();
    encoderPinAPub.shutdown();
    encoderPinBPub.shutdown();
    encoderIndexPinPub.shutdown();
    analogStatusPub.shutdown();
    AIN1Pub.shutdown();
    loadCurrentPub.shutdown();
    caseTempPub.shutdown();
    busVoltagePub.shutdown();
    canFramePub.shutdown();
}

void DMC60C::parseGeneralStatus (const can_msgs::Frame::ConstPtr& msg) {
    if (msg->id != static_cast<uint32_t>(MsgID::StatusGeneral)) return;
    if (msg->is_error) return;
    if (msg->dlc < 8) return;
    std_msgs::Int16 dtcmsg;
    std_msgs::UInt32 fltmsg;
    std_msgs::UInt64 outmsg;
    outmsg.data = dataToUInt64(msg->data.data(), msg->dlc);
    dtcmsg.data = dataToInt16(msg->data.data(), 0);
    fltmsg.data = dataToUInt32(msg->data.data(), 2, 3);
    appliedDutyCyclePub.publish(dtcmsg);
    faultStatusPub.publish(fltmsg);
    genStatusPub.publish(outmsg);
    parseGeneralStatusFlag0(msg->data[2]);
    parseGeneralStatusFlag1(msg->data[3]);
    parseGeneralStatusFlag2(msg->data[4]);
    parseClosedLoopError(msg);
}

void DMC60C::parseEncoderStatus (const can_msgs::Frame::ConstPtr& msg) {
    if (msg->id != static_cast<uint32_t>(MsgID::StatusEncoder)) return;
    if (msg->is_error) return;
    if (msg->dlc < 8) return;
    bool posdiv8 = msg->data[7] & 0x01;
    bool veldiv4 = msg->data[7] & 0x02;
    std_msgs::Int32 vel, pos;
    std_msgs::UInt8 flags;
    std_msgs::Bool pinA, pinB, pinIdx;
    std_msgs::UInt64 stat;
    stat.data = dataToUInt64(msg->data.data(), msg->dlc);
    flags.data = msg->data[7];
    pinA.data = msg->data[7] & 0x04;
    pinB.data = msg->data[7] & 0x08;
    pinIdx.data = msg->data[7] & 0x01;
    vel.data = msg->data[4] + (static_cast<int32_t>(msg->data[3]) << 8);
    if (vel.data > numeric_limits<int16_t>::max()) {
        vel.data = vel.data - (numeric_limits<int16_t>::max() + 1);
    }
    pos.data = msg->data[2] + (static_cast<int32_t>(msg->data[1]) << 8) +
                (static_cast<int32_t>(msg->data[1]) << 16);
    if (veldiv4) vel.data *= 4;
    if (posdiv8) pos.data *= 8;
    encoderStatusPub.publish(stat);
    positionPub.publish(pos);
    velocityPub.publish(vel);
    encoderFlagsPub.publish(flags);
    encoderPinAPub.publish(pinA);
    encoderPinBPub.publish(pinB);
    encoderIndexPinPub.publish(pinIdx);
}

void DMC60C::parseAnalogStatus (const can_msgs::Frame::ConstPtr& msg) {
    if (msg->id != static_cast<uint32_t>(MsgID::StatusAnalog)) return;
    if (msg->is_error) return;
    if (msg->dlc < 8) return;
    std_msgs::Float32 ain1;
    std_msgs::Float32 current;
    std_msgs::Float32 temp;
    std_msgs::Float32 busV;
    std_msgs::UInt64 stat;
    stat.data = dataToUInt64(msg->data.data(), msg->dlc);
    ain1.data = data8_8ToFloat(msg->data.data(), 0);
    current.data = data8_8ToFloat(msg->data.data(), 2);
    temp.data = data8_8ToFloat(msg->data.data(), 4);
    busV.data = data8_8ToFloat(msg->data.data(), 6);
    analogStatusPub.publish(stat);
    AIN1Pub.publish(ain1);
    loadCurrentPub.publish(current);
    caseTempPub.publish(temp);
    busVoltagePub.publish(busV);
}

void DMC60C::parseGeneralStatusFlag0 (uint8_t val) {
    std_msgs::Bool msg;
    msg.data = (val & 0x01);
    fwdLimitPinPub.publish(msg);
    msg.data = (val & 0x02);
    fwdLimitHitPub.publish(msg);
    msg.data = (val & 0x04);
    fwdLimitDisablePub.publish(msg);
    msg.data = (val & 0x08);
    fwdLimitNCPub.publish(msg);
    msg.data = (val & 0x10);
    revLimitPinPub.publish(msg);
    msg.data = (val & 0x20);
    revLimitHitPub.publish(msg);
    msg.data = (val & 0x40);
    revLimitDisablePub.publish(msg);
    msg.data = (val & 0x80);
    revLimitNCPub.publish(msg);
}

void DMC60C::parseGeneralStatusFlag1 (uint8_t val) {
    std_msgs::Bool msg;
    msg.data = (val & 0x01);
    fwdRevLimitOverridePub.publish(msg);
    msg.data = (val & 0x02);
    fwdLimitDisableOvrdPub.publish(msg);
    msg.data = (val & 0x04);
    revLimitDisableOvrdPub.publish(msg);
    msg.data = (val & 0x08);
    softFwdLimitHitPub.publish(msg);
    msg.data = (val & 0x10);
    softFwdLimitEnbPub.publish(msg);
    msg.data = (val & 0x20);
    softRevLimitHitPub.publish(msg);
    msg.data = (val & 0x40);
    softRevLimitEnbPub.publish(msg);
    msg.data = (val & 0x80);
    currentLimitActivePub.publish(msg);
}

void DMC60C::parseGeneralStatusFlag2 (uint8_t val) {
    std_msgs::Bool msg;
    std_msgs::UInt8 modemsg;
    msg.data = (val & 0x01);
    overTempFltPub.publish(msg);
    msg.data = (val & 0x02);
    underVoltFaultPub.publish(msg);
    msg.data = (val & 0x04);
    gateDriverFaultPub.publish(msg);
    msg.data = (val & 0x80);
    divErrBy256Pub.publish(msg);
    modemsg.data = (val & 0x78) >> 3;
}

void DMC60C::parseClosedLoopError (const can_msgs::Frame::ConstPtr& msg) {
    // It would be nice to make this track the behavior correctly, but it's not documented
    std_msgs::Int32 out;
    out.data = dataToInt32(msg->data.data(), 5, 3);
    closedLoopErrPub.publish(out);
}

void DMC60C::parseVendorStatus (const can_msgs::Frame::ConstPtr& msg) {
    if (msg->id != static_cast<uint32_t>(MsgID::VendorStatus)) return;
    if (msg->is_error) return;
    if (msg->dlc < 4) return;
    vendorRespFresh = true;
    lastVendorErr = static_cast<VendorCmdErrorCodes>(msg->data[0] +
                    (static_cast<uint16_t>(msg->data[1]) << 8));
    vendorDataExpected = (msg->data[2] +
                    (static_cast<uint16_t>(msg->data[3]) << 8));
    if (vendorDataExpected) vendorDataReceived.clear();
}

void DMC60C::parseParamRespStatus (const can_msgs::Frame::ConstPtr& msg) {
    if (msg->id != static_cast<uint32_t>(MsgID::ParamResp)) return;
    if (msg->is_error) return;
    if (msg->dlc < 6) return;
    lastParamErr = static_cast<ParamErrorCodes>(msg->data[5]);
    lastParamSet = static_cast<ParamID>(msg->data[0]);
    lastParamValSet = static_cast<uint32_t>(msg->data[1]) +
                    (static_cast<uint32_t>(msg->data[2]) << 8) +
                    (static_cast<uint32_t>(msg->data[3]) << 16) +
                    (static_cast<uint32_t>(msg->data[4]) << 24);
}

void DMC60C::parseDeviceData (const can_msgs::Frame::ConstPtr& msg) {
    if (msg->id != static_cast<uint32_t>(MsgID::VendorDin)) return;
    if (msg->is_error) return;
    for (int i = 0; i < msg->dlc; i++) {
        vendorDataReceived.push_back(msg->data[i]);
    }
}

bool DMC60C::setFloatParam(ParamID param, float value) {
    return setIntParam(param, floatToFloat16_16(value));
}

bool DMC60C::setIntParam(ParamID param, int32_t value) {
    return setUIntParam(param, value);
}

bool DMC60C::setUIntParam(ParamID param, uint32_t value) {
    can_msgs::Frame frame;
    frame.dlc = 7;
    frame.id = static_cast<uint32_t>(MsgID::ParamSet);
    frame.data[2] = static_cast<uint8_t>(param);
    frame.data[0] = session_id & 0xff;
    frame.data[1] = (session_id >> 8) & 0xff;
    uint32ToData(value, frame.data.data(), 3);
    paramRespFresh = false;
    lastParamValSet = 0;
    canFramePub.publish(frame);
    for (int i = 0; i < 100; i++) {
        this_thread::sleep_for(1ms);
        ros::spinOnce();
        if (paramRespFresh) {
            if ((lastParamErr == ParamErrorCodes::NoError) &&
                (lastParamSet == param) &&
                (lastParamValSet == value)) {
                    return true;
            } else {
                return false;
            }
        }
    }
    return false;
}

void DMC60C::transmitControl() {
    can_msgs::Frame frame;
    frame.dlc = 8;
    frame.id = static_cast<uint32_t>(MsgID::Control0) + device_id;
    frame.data[0] = static_cast<uint8_t>(selectedMode) + (motorControlProfileSlot << 6);
    if (brakeModeOverride) frame.data[0] += 0x10;
    if (brakeModeOverrideSetting) frame.data[0] += 0x20;
    if (revFdbkSnsr) frame.data[0] += 0x80;
    frame.data[1] = 0;
    if (revMotor) frame.data[1] += 0x01;
    if (disableFwdLimit) frame.data[1] += 0x02;
    if (disableRevLimit) frame.data[1] += 0x04;
    if (enableLimitOverride) frame.data[1] += 0x08;
    uint32ToData(target, frame.data.data(), 3, 3, true);
    int16ToData(voltageRamp, frame.data.data(), 6);
    canFramePub.publish(frame);
}

void DMC60C::modeSelectCallback (const std_msgs::UInt8::ConstPtr& msg) {
    selectedMode = static_cast<DriveMode>(msg->data);
}

void DMC60C::brakeModeOverrideCallback (const std_msgs::Bool::ConstPtr& msg) {
    brakeModeOverride = msg->data;
}

void DMC60C::brakeModeOverrideSettingCallback (const std_msgs::Bool::ConstPtr& msg) {
    brakeModeOverrideSetting = msg->data;
}

void DMC60C::motorControlProfileSlotCallback (const std_msgs::UInt8::ConstPtr& msg) {
    if ((msg->data == 0) || (msg->data == 1)) motorControlProfileSlot = msg->data;
}

void DMC60C::revFdbkSnsrCallback (const std_msgs::Bool::ConstPtr& msg) {
    revFdbkSnsr = msg->data;
}

void DMC60C::revMotorCallback (const std_msgs::Bool::ConstPtr& msg) {
    revMotor = msg->data;
}

void DMC60C::disableFwdLimitCallback (const std_msgs::Bool::ConstPtr& msg) {
    disableFwdLimit = msg->data;
}

void DMC60C::disableRevLimitCallback (const std_msgs::Bool::ConstPtr& msg) {
    disableRevLimit = msg->data;
}

void DMC60C::enableLimitOverrideCallback (const std_msgs::Bool::ConstPtr& msg) {
    enableLimitOverride = msg->data;
}

void DMC60C::targetCallback (const std_msgs::Float32::ConstPtr& msg) {
    uint8_t tmp;
    switch(selectedMode) {
        case(DriveMode::Voltage):
            target = floatToInt16(msg->data);
            break;
        case(DriveMode::Velocity):
        case(DriveMode::Position):
            target = floatToInt24(msg->data);
            break;
        case(DriveMode::Current):
        case(DriveMode::Vcomp):
            target = floatToFloat8_16(msg->data);
            break;
        case(DriveMode::Follower):
            tmp = floor(msg->data);
            if (tmp < 64) {
                target = tmp;
            }
        case(DriveMode::NoDrive):
        default:
            target = 0;
            break;
    }
    transmitControl();
}

void DMC60C::voltageRampCallback (const std_msgs::UInt16::ConstPtr& msg) {
    voltageRamp = msg->data;
}
