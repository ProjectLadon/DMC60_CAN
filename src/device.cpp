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

void Device::uint16ToData (const uint16_t in, uint8_t *data, const uint8_t offset, const uint8_t len, const bool reversed) {
    if (len == 0) return;
    if (reversed) {
        if (len == 1) {
            data[offset] = in & 0xff;
        } else {
            data[offset] = (in >> 8) & 0xff;
            data[offset + 1] = in & 0xff;
        }
    } else {
        data[offset] = in & 0xff;
        if (len > 1) data[offset + 1] = (in >> 8) & 0xff;
    }
}

void Device::int16ToData (const int16_t in, uint8_t *data, const uint8_t offset, const uint8_t len, const bool reversed) {
    uint16ToData(in, data, offset, len, reversed);
}

void Device::uint32ToData (const uint32_t in, uint8_t *data, const uint8_t offset, const uint8_t len, const bool reversed) {
    if (reversed) {
        for (int i = offset; i < (offset + len); i++) {
            int j = len - (i - offset);
            data[i] = (in >> (j * 8)) & 0xff;
        }
    } else {
        for (int i = offset; i < (offset + len); i++) {
            int j = i - offset;
            data[i] = (in >> (j * 8)) & 0xff;
        }
    }
}

void Device::int32ToData (const int32_t in, uint8_t *data, const uint8_t offset, const uint8_t len, const bool reversed) {
    uint32ToData(in, data, offset, len, reversed);
}

int16_t Device::dataToInt16 (const uint8_t *data, const uint8_t offset, const uint8_t len, const bool reversed) {
    if (len < 1) return 0;
    int16_t result;
    if (reversed) {
        result = data[offset + (len - 1)];
        if (len > 1) result += (static_cast<int16_t>(data[offset]) << 8);
    } else {
        result = data[offset];
        if (len > 1) result += (static_cast<int16_t>(data[offset + 1]) << 8);
    }
    return result;
}

uint32_t Device::dataToUInt32 (const uint8_t *data, const uint8_t offset, const uint8_t len, const bool reversed) {
    if (len < 1) return 0;
    uint32_t result = 0;
    if (reversed) {
        for (int i = 0; i < len; i++) {
            int j = (offset + len - 1) - i;
            result += static_cast<uint32_t>(data[j]) << (i * 8);
        }
    } else {
        for (int i = 0; i < len; i++) {
            result += static_cast<uint32_t>(data[i + offset]) << (i * 8);
        }
    }
    return result;
}

int32_t Device::dataToInt32 (const uint8_t *data, const uint8_t offset, const uint8_t len, const bool reversed) {
    if (len < 1) return 0;
    int32_t result = 0;
    if (reversed) {
        for (int i = 0; i < len; i++) {
            int j = (offset + len - 1) - i;
            result += static_cast<int32_t>(data[j]) << (i * 8);
        }
    } else {
        for (int i = 0; i < len; i++) {
            result += static_cast<int32_t>(data[i + offset]) << (i * 8);
        }
    }
    return result;
}

uint64_t Device::dataToUInt64 (const uint8_t *data, uint8_t len) {
    uint64_t out = 0;
    for (int i = 0; i < len; i++) {
        out += static_cast<uint64_t>(data[i]) << (i * 8);
    }
    return out;
}

float Device::data8_8ToFloat (const uint8_t *data, const uint8_t offset) {
    int8_t tmp = data[offset + 1];
    float result = static_cast<float>(data[offset])/256;
    return (result + tmp);
}

int32_t Device::floatToFloat16_16(const double val) {
    return static_cast<int32_t>(val * 65536);
}

uint32_t Device::floatToFloat8_16(const double val) {
    // This grabs the 16 bits to the right of the decimal point.
    uint32_t result = static_cast<uint32_t>(fabs(val) * 65536) & 0xffff;
    // Now get the top eight bits in such a way that they include the sign bit.
    uint8_t tmp = static_cast<int8_t>(floor(val));
    return (result + (static_cast<uint32_t>(tmp) << 16));
}

uint32_t Device::floatToInt24(const double val) {
    int32_t result = floor(val);
    if (result < 0) {
        return 0x1000000 - (result & 0x7fffff);
    } else {
        return result & 0x7fffff;
    }
}

uint32_t Device::floatToInt16(const double val) {
    int16_t result = floor(val);
    return result;
}
