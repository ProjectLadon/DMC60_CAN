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

#ifndef DMC60_CAN_DEFS_H
#define DMC60_CAN_DEFS_H

#include <map>
#include <string>

const uint32_t msgIDMask = 0x1ffffffc0;  // Drops the last six bits, which are reserved for the device ID
const uint32_t devIDMask = 0x3f; // Grabs the device ID, which is the lowest six bits

enum class MsgID : uint32_t {
    Enum            = 0x00000240,
    EnumResp        = 0x0206f000,
    VendorCmd       = 0x0206fc00,
    VendorDout      = 0x0206fc40,
    VendorDin       = 0x0206fc80,
    VendorStatus    = 0x0206fcc0,
    ParamReq        = 0x02061800,
    ParamResp       = 0x02061840,
    ParamSet        = 0x02061880,
    StatusGeneral   = 0x02061400,
    StatusEncoder   = 0x02061480,
    StatusAnalog    = 0x020614c0,
    Control0        = 0x02060000
};

enum class VendorCommands : uint8_t {
    SetDevNumber    = 0x01,
    FlashLEDs       = 0x50,
    GetDescriptors  = 0x60,
    SoftReset       = 0xf1
};

enum class VendorCmdErrorCodes : uint8_t {
    NoError         = 0,
    NotSupported    = 1,
    BadParameter    = 2,
    DataRcvMore     = 3,
    InBootloader    = 4,
    CrcMismatch     = 5,
    FlashWriteFailed = 6,
    AckReset        = 7,
    TestPassed      = 8,
    TestFailed      = 9
};

enum class TestErrorCodes : uint8_t {
    NoError         = 0,
    UserAN1         = 1,
    FwdLimit        = 2,
    RevLimit        = 3,
    QEA             = 4,
    QEB             = 5,
    QEId            = 6
};

enum class ParamErrorCodes : uint8_t {
    NoError         = 0,
    BadParameter    = 1,
    BadValue        = 2
};

enum class ParamID : uint8_t {
    None                            = 0,
    ADCcurrentMult                  = 9,
    ProportionalGainSlot0           = 10,
    IntegralGainSlot0               = 11,
    DifferentialGainSlot0           = 12,
    IntegralLimitSlot0              = 13,
    ForwardGainSlot0                = 14,
    AllowableErrorSlot0             = 15,
    RampLimitSlot0                  = 16,
    MaxFwdSlot0                     = 17,
    MaxRevSlot0                     = 18,
    NominalFwdSlot0                 = 19,
    NominalRevSlot0                 = 20,
    ProportionalGainSlot1           = 21,
    IntegralGainSlot1               = 22,
    DifferentialGainSlot1           = 23,
    IntegralLimitSlot1              = 24,
    ForwardGainSlot1                = 25,
    AllowableErrorSlot1             = 26,
    RampLimitSlot1                  = 27,
    MaxFwdSlot1                     = 28,
    MaxRevSlot1                     = 29,
    NominalFwdSlot1                 = 30,
    NominalRevSlot1                 = 31,
    ProportionalCurrentLimiterGain  = 32,
    IntegralCurrentLimiterGain      = 33,
    DifferentialCurrentLimiterGain  = 34,
    IntegralCurrentLimiterLimit     = 35,
    ForwardCurrentLimiterGain       = 36,
    ContinuousCurrentLimit          = 61,
    PeakCurrentLimit                = 62,
    PeakCurrentDuration             = 63,
    CurrentLimitEnable              = 64
};

enum class DriveMode : uint8_t {
    Voltage     = 0,
    Velocity    = 1,
    Position    = 2,
    Current     = 3,
    Vcomp       = 4,
    Follower    = 5,
    NoDrive     = 15
};

const std::map<uint8_t, std::string> devDescriptors = {{1, "DevName"},
                                                        {2, "MfgName"},
                                                        {3, "ProdName"},
                                                        {4, "MfgDate"},
                                                        {5, "HdwVersion"},
                                                        {6, "SerialNumber"},
                                                        {0, "None"}};

#endif // DMC60_CAN_DEFS_H
