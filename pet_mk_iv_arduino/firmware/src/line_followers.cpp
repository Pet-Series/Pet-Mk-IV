#include "line_followers.h"

#include <Arduino.h>

#include <ros/time.h>

#include "rosserial_node.h"
#include "timer.h"

namespace pet
{

LineFollowers::LineFollowers()
    : m_msg()
    , m_publisher(kTopicName, &m_msg)
{
    pinMode(kLeftPin, INPUT);
    pinMode(kMiddlePin, INPUT);
    pinMode(kRightPin, INPUT);

    m_msg.header.frame_id = "line_followers";
    nh.advertise(m_publisher);
}

ros::Time LineFollowers::callback(const TimerEvent& event)
{
    m_msg.left = digitalRead(kLeftPin);
    m_msg.middle = digitalRead(kMiddlePin);
    m_msg.right = digitalRead(kRightPin);
    m_msg.header.stamp = event.current_time;
    m_publisher.publish(&m_msg);
    return event.desired_time + kPeriod;
}

} // namespace pet
