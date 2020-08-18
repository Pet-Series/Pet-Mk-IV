#ifndef _PET_DISTSENSORS_H
#define _PET_DISTSENSORS_H

#include <ros/time.h>
#include <ros/duration.h>

#include "arduino_module.h"
#include "timer.h"

namespace dist_sensors
{

void setup();
void callback();

} // namespace dist_sensors

namespace pet
{

class DistSensors : public ArduinoModule
{
private:
    static constexpr double kFrequency = 30;
    static constexpr auto   kPeriod = ros::Duration{1.0/kFrequency};

public:
    DistSensors()
    {
        dist_sensors::setup();
    }

    ros::Time callback(const TimerEvent& event) override
    {
        dist_sensors::callback();
        return event.desired_time + kPeriod;
    }
};

} // namespace pet

#endif
