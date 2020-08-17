#ifndef _PET_DISTSENSORS_H
#define _PET_DISTSENSORS_H

#include "arduino_module.h"

namespace dist_sensors
{

void setup();
void callback();

} // namespace dist_sensors

namespace pet
{

class DistSensors : public ArduinoModule
{
public:
    DistSensors()
    {
        dist_sensors::setup();
    }

    void callback() override
    {
        dist_sensors::callback();
    }

    double frequency() const override
    {
        return kFrequency;
    }

public:
    static constexpr double kFrequency = 30;
};

} // namespace pet

#endif
