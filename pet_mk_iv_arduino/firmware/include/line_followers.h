#ifndef _PET_LINEFOLLOWERS_H
#define _PET_LINEFOLLOWERS_H

#include <pet_mk_iv_msgs/TripleBoolean.h>

#include "rosserial_node.h"
#include "arduino_module.h"

namespace pet
{

// Handler class for three ground pointing light sensors arranged in a left-middle-right pattern.
// TODO: Rename to LineSensors?
class LineFollowers : public ArduinoModule
{
private:
    static constexpr double kFrequency = 100;
    static constexpr int    kLeftPin   = 2;
    static constexpr int    kMiddlePin = 3;
    static constexpr int    kRightPin  = 4;
    static constexpr auto   kTopicName = "line_followers";

public:
    LineFollowers();

    void callback() override;

    double frequency() const override
    {
        return kFrequency;
    }

private:
    pet_mk_iv_msgs::TripleBoolean m_msg;
    ros::Publisher m_publisher;
};

} // namespace pet

#endif // _PET_LINEFOLLOWERS_H