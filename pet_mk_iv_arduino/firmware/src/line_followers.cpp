#include "line_followers.h"

#include <Arduino.h>

#include "rosserial_node.h"

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

void LineFollowers::callback()
{
    m_msg.header.stamp = nh.now();

    m_msg.left = digitalRead(kLeftPin);
    m_msg.middle = digitalRead(kMiddlePin);
    m_msg.right = digitalRead(kRightPin);

    m_publisher.publish(&m_msg);
}

} // namespace pet
