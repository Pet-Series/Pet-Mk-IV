#include "light_beacon.h"

#include <pet_mk_iv_msgs/LightBeacon.h>

#include "rosserial_node.h"
#include "timer.h"

namespace pet
{

LightBeacon::LightBeacon()
    : m_subscriber("beacon_mode", &LightBeacon::mode_msg_callback, this)
{
    // TODO: Rewrite (and rewire!) so that a seperate pin controls power to the light beacon. Ensure this pin is turned off in constructor.
    m_led_servo.attach(kServoPin);
    m_led_servo.write(kServoOff);
    // NOTE: We hope this short delay once during construction does not make things crash.
    const auto press_duration_millis = static_cast<int>(press_duration().toSec() * 1000);
    delay(press_duration_millis);
    nh.subscribe(m_subscriber);
}

ros::Time LightBeacon::callback(const TimerEvent& event)
{
    // TODO: Where to say that current mode is changed? On write(On) or write(Off)?
    // If servo is in 'On' state we always want to set it to 'Off', regardless of mode.
    if (m_current_position == kServoOn)
    {
        m_led_servo.write(kServoOff);
        m_current_position = kServoOff;
        return event.current_time + press_duration();
    }
    else if (m_current_mode != m_desired_mode)
    {
        m_led_servo.write(kServoOn);
        m_current_position = kServoOn;
        m_current_mode = (m_current_mode + 1) % 5;
        return event.current_time + press_duration();
    }
    else
    {
        return event.desired_time + LightBeacon::period();
    }
}

void LightBeacon::mode_msg_callback(const pet_mk_iv_msgs::LightBeacon& msg)
{
    m_desired_mode = msg.mode;
}

} // namespace pet
