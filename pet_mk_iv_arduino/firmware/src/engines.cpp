#include "engines.h"

#include <Arduino.h>

#include <ros/time.h>
#include <ros/duration.h>

#include <pet_mk_iv_msgs/EngineCommand.h>

#include "rosserial_node.h"
#include "timer.h"

namespace pet
{

Engines::Engines()
    : m_cmd_msg()
    , m_subscriber("engine_command", &Engines::cmd_msg_callback, this)
{
    pinMode(kLeftReversePin, OUTPUT);
    pinMode(kLeftForwardPin, OUTPUT);
    pinMode(kLeftSpeedPin, OUTPUT);
    pinMode(kRightSpeedPin, OUTPUT);
    pinMode(kRightForwardPin, OUTPUT);
    pinMode(kRightReversePin, OUTPUT);

    // Make sure engines are turned of until commanded otherwise.
    stop();

    nh.subscribe(m_subscriber);
}

ros::Time Engines::callback(const TimerEvent& event)
{
    if (event.current_time < m_cmd_msg.header.stamp + Engines::timeout()) {
        set_engine_pwm(m_cmd_msg);
    } else {
        stop();
        // nh.logwarn("Engine timeout!");
    }
    return event.desired_time + Engines::period();
}

void Engines::stop()
{
    digitalWrite(kLeftReversePin, LOW);
    digitalWrite(kLeftForwardPin, LOW);
    analogWrite(kLeftSpeedPin, 0);
    digitalWrite(kRightReversePin, LOW);
    digitalWrite(kRightForwardPin, LOW);
    analogWrite(kRightSpeedPin, 0);
}

void Engines::set_engine_pwm(const pet_mk_iv_msgs::EngineCommand& cmd)
{
    switch (cmd.left_direction)
    {
    case pet_mk_iv_msgs::EngineCommand::FORWARD:
        digitalWrite(kLeftReversePin, LOW);
        digitalWrite(kLeftForwardPin, HIGH);
        analogWrite(kLeftSpeedPin, cmd.left_pwm);
        break;
    case pet_mk_iv_msgs::EngineCommand::BACKWARD:
        digitalWrite(kLeftForwardPin, LOW);
        digitalWrite(kLeftReversePin, HIGH);
        analogWrite(kLeftSpeedPin, cmd.left_pwm);
        break;
    case pet_mk_iv_msgs::EngineCommand::STOP:
        digitalWrite(kLeftReversePin, LOW);
        digitalWrite(kLeftForwardPin, LOW);
        analogWrite(kLeftSpeedPin, 0);
        break;
    default:
        stop();
        nh.logerror("Unrecognised engine command. Stopping.");
        return;
    }

    switch (cmd.right_direction)
    {
    case pet_mk_iv_msgs::EngineCommand::FORWARD:
        digitalWrite(kRightReversePin, LOW);
        digitalWrite(kRightForwardPin, HIGH);
        analogWrite(kRightSpeedPin, cmd.right_pwm);
        break;
    case pet_mk_iv_msgs::EngineCommand::BACKWARD:
        digitalWrite(kRightForwardPin, LOW);
        digitalWrite(kRightReversePin, HIGH);
        analogWrite(kRightSpeedPin, cmd.right_pwm);
        break;
    case pet_mk_iv_msgs::EngineCommand::STOP:
        digitalWrite(kRightReversePin, LOW);
        digitalWrite(kRightForwardPin, LOW);
        analogWrite(kRightSpeedPin, 0);
        break;
    default:
        stop();
        nh.logerror("Unrecognised engine command. Stopping.");
        return;
    }
}

} // namespace pet
