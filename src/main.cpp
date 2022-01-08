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
#include <geometry_msgs/msg/twist.hpp>
#include "i2c/i2c.h"

#ifdef _WIN32
#pragma optimize( "", off )
#endif

using namespace std::chrono_literals;
using std::placeholders::_1;

const uint32_t kFrequencyOccilator = 25000000;
const float kPwmFrequency = 1000.0f;

// TODO: Make configurable
const uint16_t kPulseMin = 1000; // microseconds
const uint16_t kPulseMax = 2000; // microseconds
const uint16_t kPulseNeutral = 1500; // microseconds
const uint16_t kMaxThrottle = 500; // +/- microseconds 


typedef enum
{
   ServoCommand_Mode1 = 0x00,
   ServoCommand_ChannelBegin_On_Low = 0x06,
   ServoCommand_ChannelBegin_On_High = 0x07,
   ServoCommand_ChannelBegin_Off_Low = 0x08,
   ServoCommand_ChannelBegin_Off_High = 0x09,

   // ... 15
   

   ServoCommand_Prescale = 0xFE
} QwiicServoCommand;

typedef enum
{
    Mode1_CalibrateAll = 0x01,
    Mode1_Sub3 = 0x02,
    Mode1_Sub2 = 0x04,
    Mode1_Sub1 = 0x08,
    Mode1_Sleep = 0x10,
    Mode1_AI = 0x20,
    Mode1_ExternalClock = 0x40,
    Mode1_Restart = 0x80
} QwiicServoMode1Value;


class CMDVelSubscriber : public rclcpp::Node
{

  public:
    CMDVelSubscriber()
    : Node("ros_qwiic_servo")
    {
    }

    void start()
    {
        if ((_i2cFileDescriptor = i2c_open("/dev/i2c-0")) == -1) 
        {
            return;
        }

        _i2cDevice.bus = _i2cFileDescriptor;
        _i2cDevice.addr = 0x40;
        _i2cDevice.tenbit = 0;
        _i2cDevice.delay = 10;
        _i2cDevice.flags = 0;
        _i2cDevice.page_bytes = 8;
        _i2cDevice.iaddr_bytes = 0;

        reset();
        setPWMFrequency(kPwmFrequency);


        get_parameter_or<float>("wheelSeparation", _wheelSeparation, 50);
        get_parameter_or<float>("wheelRadius", _wheelRadius, 10);

        _subscription = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CMDVelSubscriber::cmdVelCallback, this, _1));
    }

  private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {

      double angularComponent = _wheelSeparation / (2.0f * _wheelRadius);   // rads / second
      double linearComponent = 1.0f;// _wheelRadius; // cm / second.

      double speedRight = angularComponent * msg->angular.z + linearComponent * msg->linear.x;
      double speedLeft = angularComponent * msg->angular.z - linearComponent * msg->linear.x;

      motor(0, powerFromSpeed(speedRight));
      motor(1, powerFromSpeed(speedLeft));
    }


    uint16_t powerFromSpeed(double speed)
    {
        bool reverse = speed < 0.0f;
        double powerVal = std::abs(speed) * kMaxThrottle;
        if (powerVal > kMaxThrottle)
        {
            powerVal = kMaxThrottle;
        }

        if (reverse)
        {
            return kPulseNeutral - powerVal;
        }
        else
        {
            return kPulseNeutral + powerVal;
        }
    }

    void motor(uint8_t channel, uint16_t power)
    {
        writeMicroseconds(channel, power);
    }

    void reset()
    {
        command(ServoCommand_Mode1, Mode1_Restart);
    }

    void setPWMFrequency(float frequency)
    {
        if (frequency < 1.0f)
        {
            frequency = 1;
        }
        
        if (frequency > 3500.0f)
        {
            frequency = 3500;
        }

        float prescaleFloat = ((kFrequencyOccilator / (frequency * 4096.0f)) + 0.5f) - 1.0f;
        if (prescaleFloat < 3.0f)
            prescaleFloat = 3.0f;
        if (prescaleFloat > 255.0f)
            prescaleFloat = 255.09f;

        uint8_t prescale = (uint8_t)prescaleFloat;

        uint8_t oldMode = readByte(ServoCommand_Mode1);
        uint8_t newMode = (oldMode & ~Mode1_Restart) | Mode1_Sleep;

        command(ServoCommand_Mode1, newMode);
        command(ServoCommand_Prescale, prescale);
        command(ServoCommand_Mode1, oldMode);
        rclcpp::sleep_for(5ms);
        command(ServoCommand_Mode1, oldMode | Mode1_Restart | Mode1_AI);
    }

    void command(QwiicServoCommand command, uint8_t value)
    {
        uint8_t commandBuffer[] = {command, value};

        int ret = i2c_ioctl_write(&_i2cDevice, 0x0, commandBuffer, 2);
        if (ret == -1 || (size_t)ret != 1)
        {
            // error
        }
    }

    uint8_t readByte(QwiicServoCommand command)
    {
        uint8_t ret = 0;
        uint8_t commandBuffer[] = {command};
        i2c_ioctl_write(&_i2cDevice, 0x0, commandBuffer, 1);
        i2c_ioctl_read(&_i2cDevice, 0x0, &ret, 1);

        return ret;
    }

    uint16_t readShort(QwiicServoCommand command)
    {
        uint8_t commandBuffer[] = {command};
        i2c_ioctl_write(&_i2cDevice, 0x0, commandBuffer, 1);

        uint16_t ret = 0;
        i2c_ioctl_read(&_i2cDevice, 0x0, &ret, sizeof(ret));

        return ret;
    }

    void setPWM(uint8_t channel,  uint16_t on, uint16_t off)
    {
        uint8_t commandBuffer[] = 
        { 
            (uint8_t)(ServoCommand_ChannelBegin_On_Low + 4 * channel), 
            (uint8_t)on, (uint8_t)(on >> 8),
            (uint8_t)off, (uint8_t)(off >> 8)
        };

        i2c_ioctl_write(&_i2cDevice, 0x0, commandBuffer, sizeof(commandBuffer));
    }

    void writeMicroseconds(uint8_t channel, uint16_t microseconds)
    {
        double pulse = microseconds;

        double pulseLength = 1000000.0; // 1,000,000 us/second
        uint16_t prescale = readShort(ServoCommand_Prescale);
        prescale += 1;  //?
        pulseLength *= prescale;
        pulseLength /= kFrequencyOccilator;

        pulse /= pulseLength;

        setPWM(channel, 0, pulse);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription;

    float _wheelSeparation;
    float _wheelRadius;

    int _i2cFileDescriptor;
    I2CDevice _i2cDevice;    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CMDVelSubscriber>();

    node->declare_parameter("wheelSeparation");
    node->declare_parameter("wheelRadius");

    node->start();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}