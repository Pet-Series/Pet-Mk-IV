#include "dist_sensors.h"

#include <stdint.h>

#include <Arduino.h>
#include <NewPing.h>

#include <pet_mk_iv_msgs/DistanceMeasurement.h>

#include "rosserial_node.h"

namespace dist_sensors
{

constexpr unsigned int kTriggerPinRight  = 14;   // A0  "Right"
constexpr unsigned int kEchoPinRight     = 14;   // A0  "Right"
constexpr unsigned int kTriggerPinMid    = 15;   // A1  "Middle"
constexpr unsigned int kEchoPinMid       = 15;   // A1  "Middle"
constexpr unsigned int kTriggerPinLeft   = 16;   // A2  "Left"
constexpr unsigned int kEchoPinLeft      = 16;   // A2  "Left"

constexpr unsigned int kSensorCount      = 3;
constexpr unsigned int kMaxDistance      = 400;  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

static pet_mk_iv_msgs::DistanceMeasurement distSensorMsg;
static ros::Publisher distSensorPub("dist_sensors", &distSensorMsg);

static void echoCheck();

class Ultrasound
{
private:
    NewPing m_sonar;
public:
    const char* m_frame_id;

public:
    Ultrasound(int triggerPin, int echoPin, const char* frame_id)
        : m_sonar(triggerPin, echoPin, kMaxDistance)
        , m_frame_id(frame_id)
    {
    }

    void startPing()
    {
        m_sonar.ping_timer(echoCheck);
    }

    void stopPing()
    {
        m_sonar.timer_stop();
    }

    bool pingHasReturned()
    {
        return m_sonar.check_timer();
    }

    int getDistance()
    {
        return m_sonar.ping_result * 10 / US_ROUNDTRIP_CM;
    }
};

static uint8_t currentSensor = 0;
static Ultrasound sensors[kSensorCount] = {
    Ultrasound(kTriggerPinRight, kEchoPinRight, "dist_sensor_right"),
    Ultrasound(kTriggerPinMid, kEchoPinMid, "dist_sensor_mid"),
    Ultrasound(kTriggerPinLeft, kEchoPinLeft, "dist_sensor_left")
    };


/// WARNING! This function may be called inside an interrupt.
static volatile int measuredDistance = -1;
static void echoCheck()
{
    if (sensors[currentSensor].pingHasReturned())
    {
        measuredDistance = sensors[currentSensor].getDistance();
    }
}

void setup()
{
    sensors[currentSensor].startPing();

    pet::nh.advertise(distSensorPub);
}

void callback()
{
    sensors[currentSensor].stopPing();

    distSensorMsg.header.stamp = pet::nh.now();
    distSensorMsg.header.frame_id = sensors[currentSensor].m_frame_id;
    distSensorMsg.distance = measuredDistance;
    distSensorPub.publish(&distSensorMsg);

    measuredDistance = -1;
    currentSensor = (currentSensor + 1) % kSensorCount;
    sensors[currentSensor].startPing();
}

} // namespace dist_sensors
