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
#include "dmc60_can/enumerator.h"
#include <string>
#include <limits>
#include <chrono>
#include <thread>

using namespace std;

CAN_Enumerator::CAN_Enumerator(std::string canrx_topic, std::string cantx_topic, ros::NodeHandle *h) {
    node = h;
    canrx = canrx_topic;
    cantx = cantx_topic;
    framePub = node->advertise<can_msgs::Frame>(cantx_topic, 1);
    frameSub = node->subscribe(canrx_topic, 1, &CAN_Enumerator::frameCallback, this);
    can_msgs::Frame frame;
    frame.id = static_cast<uint32_t>(MsgID::Enum);
    framePub.publish(frame);
    for (int i = 0; i < 100; i++) {
        this_thread::sleep_for(1ms);
        ros::spinOnce();
    }
    subscribe();
}

void CAN_Enumerator::subscribe() {
    for (auto &d: devs) {
        d.second->subscribe();
    }
    ledService = node->advertiseService("flashLEDs", &CAN_Enumerator::flashLEDs, this);
    getDescriptorService = node->advertiseService("getDescriptors", &CAN_Enumerator::getDescriptors, this);
    getListService = node->advertiseService("getDevList", &CAN_Enumerator::getList, this);
    setDevNumService = node->advertiseService("setDevNum", &CAN_Enumerator::setDev, this);
    resetService = node->advertiseService("reset", &CAN_Enumerator::reset, this);
}

void CAN_Enumerator::frameCallback(const can_msgs::Frame::ConstPtr& msg) {
    uint8_t devnum = 0;
    switch (msg->id & msgIDMask) {
        case (static_cast<uint32_t>(MsgID::EnumResp)):
            createNewDevice(msg);
            break;
        case (static_cast<uint32_t>(MsgID::VendorDin)):
        case (static_cast<uint32_t>(MsgID::VendorStatus)):
        case (static_cast<uint32_t>(MsgID::ParamResp)):
        case (static_cast<uint32_t>(MsgID::StatusGeneral)):
        case (static_cast<uint32_t>(MsgID::StatusEncoder)):
        case (static_cast<uint32_t>(MsgID::StatusAnalog)):
            devnum = (msg->id & devIDMask);
            if (devNumMap.count(devnum) && devs.count(devNumMap[devnum])) {
                devs[devNumMap[devnum]]->recvFrame(msg);
            }
        default:
            return;
    }
}

bool CAN_Enumerator::createNewDevice(const can_msgs::Frame::ConstPtr& msg) {
    uint16_t sessid = Device::dataToInt16(msg->data.data(), 0);
    if (devs.count(sessid) == 0) {

    }
    return true;
}

bool CAN_Enumerator::flashLEDs(dmc60_can::FlashLEDs::Request &req,
    dmc60_can::FlashLEDs::Response &res) {
        return true;
    }
bool CAN_Enumerator::getDescriptors(dmc60_can::GetDescriptors::Request &req,
    dmc60_can::GetDescriptors::Response &res) {
        return true;
    }
bool CAN_Enumerator::getList(dmc60_can::GetDevList::Request &req,
    dmc60_can::GetDevList::Response &res) {
        return true;
    }
bool CAN_Enumerator::setDev(dmc60_can::SetDevNumber::Request &req,
    dmc60_can::SetDevNumber::Response &res) {
        return true;
    }
bool CAN_Enumerator::reset(dmc60_can::SetDevNumber::Request &req,
    dmc60_can::SetDevNumber::Response &res) {
        return true;
    }
