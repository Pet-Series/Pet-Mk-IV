#ifndef _PET_LINEFOLLOWERS_H
#define _PET_LINEFOLLOWERS_H

#include <ros/time.h>
#include <ros/duration.h>

#include <pet_mk_iv_msgs/TripleBoolean.h>

#include "rosserial_node.h"
#include "arduino_module.h"
#include "timer.h"

namespace pet
{

// Handler class for three ground pointing light sensors arranged in a left-middle-right pattern.
// TODO: Rename to LineSensors?
class LineFollowers : public ArduinoModule
{
private:
    static constexpr double kFrequency = 100;
    static constexpr auto   kPeriod = ros::Duration{1.0/kFrequency};

    static constexpr int    kLeftPin   = 2;
    static constexpr int    kMiddlePin = 3;
    static constexpr int    kRightPin  = 4;
    static constexpr auto   kTopicName = "line_followers";

public:
    LineFollowers();

    ros::Time callback(const TimerEvent& event) override;

private:
    pet_mk_iv_msgs::TripleBoolean m_msg;
    ros::Publisher m_publisher;
};

} // namespace pet

#endif // _PET_LINEFOLLOWERS_H