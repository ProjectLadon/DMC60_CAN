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

#ifndef ENUMERATOR_H
#define ENUMERATOR_H

#include <map>
#include <memory>
#include <string>
#include "dmc60_can/device.h"
#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include "dmc60_can/FlashLEDs.h"
#include "dmc60_can/GetDescriptors.h"
#include "dmc60_can/GetDevList.h"
#include "dmc60_can/SetDevNumber.h"

class CAN_Enumerator {
    public:
        CAN_Enumerator(std::string canrx_topic, std::string cantx_topic, ros::NodeHandle *h);
        void subscribe();

    private:
        void frameCallback(const can_msgs::Frame::ConstPtr& msg);
        bool createNewDevice(const can_msgs::Frame::ConstPtr& msg);
        bool flashLEDs(dmc60_can::FlashLEDs::Request  &req,
                        dmc60_can::FlashLEDs::Response &res);
        bool getDescriptors(dmc60_can::GetDescriptors::Request  &req,
                            dmc60_can::GetDescriptors::Response &res);
        bool getList(dmc60_can::GetDevList::Request  &req,
                        dmc60_can::GetDevList::Response &res);
        bool setDev(dmc60_can::SetDevNumber::Request  &req,
                    dmc60_can::SetDevNumber::Response &res);
        bool reset(dmc60_can::SetDevNumber::Request  &req,
                    dmc60_can::SetDevNumber::Response &res);

        std::map<uint16_t, std::unique_ptr<Device>> devs;
        std::map<uint8_t, uint8_t> devNumMap;
        std::string canrx;
        std::string cantx;
        ros::Subscriber frameSub;
        ros::Publisher framePub;
        ros::ServiceServer ledService;
        ros::ServiceServer getDescriptorService;
        ros::ServiceServer getListService;
        ros::ServiceServer setDevNumService;
        ros::ServiceServer resetService;
        ros::NodeHandle *node;
};

#endif // ENUMERATOR_H
