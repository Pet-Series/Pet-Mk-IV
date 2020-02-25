#include "engines.h"

#include <Arduino.h>

#include "ros.h"
#include <ros/time.h>
#include "ros_time_operators.h"

#include "pet_mk_iv_msgs/EngineCommand.h"

constexpr unsigned int kLeftReversePin  = 7;
constexpr unsigned int kLeftForwardPin  = 8;
constexpr unsigned int kLeftSpeedPin    = 9;
constexpr unsigned int kRightSpeedPin   = 10;
constexpr unsigned int kRightForwardPin = 11;
constexpr unsigned int kRightReversePin = 12;

static const ros::Duration ENGINE_TIMEOUT(0, 0.5e9);

extern pet::ros::NodeHandle nh;
static pet_mk_iv_msgs::EngineCommand engineCommandMsg;
static ros::Subscriber<pet_mk_iv_msgs::EngineCommand> engineCommandSub("engine_command", &engineCommandCb);

void enginesSetup()
{
    pinMode(kLeftReversePin, OUTPUT);
    pinMode(kLeftForwardPin, OUTPUT);
    pinMode(kLeftSpeedPin, OUTPUT);
    pinMode(kRightSpeedPin, OUTPUT);
    pinMode(kRightForwardPin, OUTPUT);
    pinMode(kRightReversePin, OUTPUT);

    digitalWrite(kLeftReversePin, LOW);
    digitalWrite(kLeftForwardPin, LOW);
    digitalWrite(kLeftSpeedPin, LOW);
    digitalWrite(kRightSpeedPin, LOW);
    digitalWrite(kRightForwardPin, LOW);
    digitalWrite(kRightReversePin, LOW);

    nh.subscribe(engineCommandSub);
}

void enginesUpdate()
{
    if (nh.now() < engineCommandMsg.header.stamp + ENGINE_TIMEOUT)
    {
        if (engineCommandMsg.left_direction == engineCommandMsg.FORWARD)
        {
            digitalWrite(kLeftReversePin, LOW);
            digitalWrite(kLeftForwardPin, HIGH);
            analogWrite(kLeftSpeedPin, engineCommandMsg.left_pwm);
        }
        else if (engineCommandMsg.left_direction == engineCommandMsg.BACKWARD)
        {
            digitalWrite(kLeftForwardPin, LOW);
            digitalWrite(kLeftReversePin, HIGH);
            analogWrite(kLeftSpeedPin, engineCommandMsg.left_pwm);
        }
        else
        {
            digitalWrite(kLeftReversePin, LOW);
            digitalWrite(kLeftForwardPin, LOW);
            analogWrite(kLeftSpeedPin, 0);
        }

        if (engineCommandMsg.right_direction == engineCommandMsg.FORWARD)
        {
            digitalWrite(kRightReversePin, LOW);
            digitalWrite(kRightForwardPin, HIGH);
            analogWrite(kRightSpeedPin, engineCommandMsg.right_pwm);
        }
        else if (engineCommandMsg.right_direction == engineCommandMsg.BACKWARD)
        {
            digitalWrite(kRightForwardPin, LOW);
            digitalWrite(kRightReversePin, HIGH);
            analogWrite(kRightSpeedPin, engineCommandMsg.right_pwm);
        }
        else
        {
            digitalWrite(kRightReversePin, LOW);
            digitalWrite(kRightForwardPin, LOW);
            analogWrite(kRightSpeedPin, 0);
        }
    }
    else
    {
        digitalWrite(kLeftReversePin, LOW);
        digitalWrite(kLeftForwardPin, LOW);
        digitalWrite(kLeftSpeedPin, LOW);
        digitalWrite(kRightSpeedPin, LOW);
        digitalWrite(kRightForwardPin, LOW);
        digitalWrite(kRightReversePin, LOW);
    }
}

void engineCommandCb(const pet_mk_iv_msgs::EngineCommand& msg)
{
    engineCommandMsg = msg;
}
