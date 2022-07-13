/*
MIT License

Copyright (c) 2022 Lou Amadio

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "Adafruit_PWMServoDriver.h"


using namespace std::chrono_literals;
using std::placeholders::_1;

const uint32_t kFrequencyOccilator = 27000000;
const float kPwmFrequency = 50.0f;

const uint16_t kMaxThrottle = 500; // +/- microseconds 
const uint16_t kPulseNeutral = 1500; // microseconds
const uint16_t kPulseMin = kPulseNeutral - kMaxThrottle; // microseconds
const uint16_t kPulseMax = kPulseNeutral + kMaxThrottle; // microseconds

const uint8_t kChannels = 16;

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


class ServoSubscriber : public rclcpp::Node
{
  public:
    ServoSubscriber()
    : Node("ros_qwiic_servo")
    , _id(0)
    , _bus("")
    {
    }

    void initialize()
    {
        get_parameter_or<uint8_t>("i2c_address", _id, 0x40);
        get_parameter_or<std::string>("bus", _bus, "/dev/i2c-1"); 

        pPwm = new Adafruit_PWMServoDriver(_id, _bus);

        pPwm->begin();
        pPwm->setOscillatorFrequency(kFrequencyOccilator);
        pPwm->setPWMFreq(kPwmFrequency);  // Analog servos run at ~50 Hz updates

        for (int channel = 0; channel < kChannels; channel++)
        {
            std::ostringstream channelTopic;
            channelTopic << "/servo/channel_" << channel;

            _subscription[channel] = create_subscription<std_msgs::msg::Float32>(
                channelTopic.str(), 1,
                [=] (const std_msgs::msg::Float32::SharedPtr msg)
                { 
                    // -1 , 0, 1
                    
                    float currentThrottle = std::abs(msg->data) * (float)kMaxThrottle;
                    if (msg->data > 0)
                    {
                        pPwm->writeMicroseconds(channel, kPulseNeutral + currentThrottle);
                    }
                    else
                    {
                        pPwm->writeMicroseconds(channel, kPulseNeutral - currentThrottle);
                    }
                });      
        }
    }

  private:
     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _subscription[kChannels];

    #ifndef _WIN32
    int _i2cFileDescriptor;
    I2CDevice _i2cDevice;  
    #endif  

    uint8_t _id;
    std::string _bus;

    Adafruit_PWMServoDriver *pPwm = NULL;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ServoSubscriber>();
    node->declare_parameter("i2c_address");
    node->declare_parameter("bus");

    node->initialize();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}