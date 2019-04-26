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

#ifndef DEVICE_H
#define DEVICE_H

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "can_msgs/Frame.h"
#include "dmc60_can/dmc60_can_def.h"
#include <string>
#include <vector>
#include <map>

using namespace std;

class Device {
    public:
        Device () {};
        virtual bool setDeviceNumber(uint8_t deviceID) = 0;
        virtual uint8_t getDeviceNumber() = 0;
        virtual bool subscribe() = 0;
        virtual bool recvFrame(const can_msgs::Frame::ConstPtr& msg) = 0;
        virtual bool setParams() = 0;
        virtual map<string, string> getDescriptors () = 0;
        virtual bool flashLEDs() = 0;
        virtual bool setDevNum(uint8_t newDev) = 0;
        virtual bool resetHardware() = 0;
        static void uint16ToData (const uint16_t in, uint8_t *data, const uint8_t offset, const uint8_t len = 2, const bool reversed = false);
        static void int16ToData (const int16_t in, uint8_t *data, const uint8_t offset, const uint8_t len = 2, const bool reversed = false);
        static void uint32ToData (const uint32_t in, uint8_t *data, const uint8_t offset, const uint8_t len = 4, const bool reversed = false);
        static void int32ToData (const int32_t in, uint8_t *data, const uint8_t offset, const uint8_t len = 4, const bool reversed = false);
        static int16_t dataToInt16 (const uint8_t *data, const uint8_t offset, const uint8_t len = 2, const bool reversed = false);
        static uint32_t dataToUInt32 (const uint8_t *data, const uint8_t offset, const uint8_t len = 4, const bool reversed = false);
        static int32_t dataToInt32 (const uint8_t *data, const uint8_t offset, const uint8_t len = 4, const bool reversed = false);
        static uint64_t dataToUInt64 (const uint8_t *data, const uint8_t len = 8);
        static float data8_8ToFloat (const uint8_t *data, const uint8_t offset);
        static int32_t floatToFloat16_16(const double val);
        static uint32_t floatToFloat8_16(const double val);
        static uint32_t floatToInt24(const double val);
        static uint32_t floatToInt16(const double val);
    protected:
        uint16_t session_id;
        uint8_t device_id;
        ros::NodeHandle *node;
        string can_topic;
};

class DMC60C : public Device {
    public:
        DMC60C (uint16_t sessionID, uint8_t deviceID, string cantx_topic, ros::NodeHandle *h);
        bool setDeviceNumber(uint8_t deviceID);
        uint8_t getDeviceNumber() {return device_id;};
        bool subscribe();
        bool recvFrame(const can_msgs::Frame::ConstPtr& msg);
        bool setParams();
        map<string, string> getDescriptors ();
        bool flashLEDs();
        bool setDevNum(uint8_t newDev);
        bool resetHardware();
        ~DMC60C () {deletePublishers();};
    private:
        // private functions
        void makePublishers();
        void deletePublishers();
        void parseGeneralStatus (const can_msgs::Frame::ConstPtr& msg);
        void parseEncoderStatus (const can_msgs::Frame::ConstPtr& msg);
        void parseAnalogStatus (const can_msgs::Frame::ConstPtr& msg);
        void parseGeneralStatusFlag0 (uint8_t val);
        void parseGeneralStatusFlag1 (uint8_t val);
        void parseGeneralStatusFlag2 (uint8_t val);
        void parseClosedLoopError (const can_msgs::Frame::ConstPtr& msg);
        void parseVendorStatus (const can_msgs::Frame::ConstPtr& msg);
        void parseParamRespStatus (const can_msgs::Frame::ConstPtr& msg);
        void parseDeviceData (const can_msgs::Frame::ConstPtr& msg);
        bool setFloatParam(ParamID param, float value);
        bool setIntParam(ParamID param, int32_t value);
        bool setUIntParam(ParamID param, uint32_t value);
        void transmitControl();

        // callbacks
        void modeSelectCallback (const std_msgs::UInt8::ConstPtr& msg);
        void brakeModeOverrideCallback (const std_msgs::Bool::ConstPtr& msg);
        void brakeModeOverrideSettingCallback (const std_msgs::Bool::ConstPtr& msg);
        void motorControlProfileSlotCallback (const std_msgs::UInt8::ConstPtr& msg);
        void revFdbkSnsrCallback (const std_msgs::Bool::ConstPtr& msg);
        void revMotorCallback (const std_msgs::Bool::ConstPtr& msg);
        void disableFwdLimitCallback (const std_msgs::Bool::ConstPtr& msg);
        void disableRevLimitCallback (const std_msgs::Bool::ConstPtr& msg);
        void enableLimitOverrideCallback (const std_msgs::Bool::ConstPtr& msg);
        void targetCallback (const std_msgs::Float32::ConstPtr& msg);
        void voltageRampCallback (const std_msgs::UInt16::ConstPtr& msg);

        // subscribers
        ros::Subscriber modeSelectSub;
        ros::Subscriber brakeModeOverrideSub;
        ros::Subscriber brakeModeOverrideSettingSub;
        ros::Subscriber motorControlProfileSlotSub;
        ros::Subscriber revFdbkSnsrSub;
        ros::Subscriber revMotorSub;
        ros::Subscriber disableFwdLimitSub;
        ros::Subscriber disableRevLimitSub;
        ros::Subscriber enableLimitOverrideSub;
        ros::Subscriber targetSub;
        ros::Subscriber voltageRampSub;

        // general status publishers
        ros::Publisher genStatusPub;
        ros::Publisher appliedDutyCyclePub;
        ros::Publisher faultStatusPub;
        ros::Publisher fwdLimitPinPub;
        ros::Publisher fwdLimitHitPub;
        ros::Publisher fwdLimitDisablePub;
        ros::Publisher fwdLimitNCPub;
        ros::Publisher revLimitPinPub;
        ros::Publisher revLimitHitPub;
        ros::Publisher revLimitDisablePub;
        ros::Publisher revLimitNCPub;
        ros::Publisher fwdRevLimitOverridePub;
        ros::Publisher fwdLimitDisableOvrdPub;
        ros::Publisher revLimitDisableOvrdPub;
        ros::Publisher softFwdLimitHitPub;
        ros::Publisher softFwdLimitEnbPub;
        ros::Publisher softRevLimitHitPub;
        ros::Publisher softRevLimitEnbPub;
        ros::Publisher currentLimitActivePub;
        ros::Publisher closedLoopErrPub;
        ros::Publisher overTempFltPub;
        ros::Publisher underVoltFaultPub;
        ros::Publisher gateDriverFaultPub;
        ros::Publisher modeSelectPub;
        ros::Publisher divErrBy256Pub;

        // encoder status publishers
        ros::Publisher encoderStatusPub;
        ros::Publisher positionPub;
        ros::Publisher velocityPub;
        ros::Publisher encoderFlagsPub;
        ros::Publisher encoderPinAPub;
        ros::Publisher encoderPinBPub;
        ros::Publisher encoderIndexPinPub;

        // analog status publishers
        ros::Publisher analogStatusPub;
        ros::Publisher AIN1Pub;
        ros::Publisher loadCurrentPub;
        ros::Publisher caseTempPub;
        ros::Publisher busVoltagePub;

        // CAN publisher
        ros::Publisher canFramePub;

        // Response variables
        bool vendorRespFresh = false;
        bool paramRespFresh = false;
        VendorCmdErrorCodes lastVendorErr = VendorCmdErrorCodes::NoError;
        ParamErrorCodes lastParamErr = ParamErrorCodes::NoError;
        uint32_t lastParamValSet = 0;
        ParamID lastParamSet = ParamID::None;
        uint16_t vendorDataExpected = 0;
        vector<uint8_t> vendorDataReceived;

        // motor setting variables
        DriveMode selectedMode          = DriveMode::NoDrive;
        bool brakeModeOverride          = false;
        bool brakeModeOverrideSetting   = false;
        int8_t motorControlProfileSlot  = 0;
        bool revFdbkSnsr                = false;
        bool revMotor                   = false;
        bool disableFwdLimit            = false;
        bool disableRevLimit            = false;
        bool enableLimitOverride        = false;
        uint32_t target                  = 0;
        uint16_t voltageRamp            = 0;
};

#endif // DEVICE_H
