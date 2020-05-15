#include "dist_sensors.h"

#include <stdint.h>

#include <Arduino.h>
#include <NewPing.h>

#include "ros.h"
#include "pet_mk_iv_msgs/DistanceMeasurement.h"

constexpr unsigned int kTriggerPin1     = 14;   // A0  "Left"
constexpr unsigned int kEchoPin1        = 14;   // A0  "Left"
constexpr unsigned int kTriggerPin2     = 15;   // A1  "Middle"
constexpr unsigned int kEchoPin2        = 15;   // A1  "Middle"
constexpr unsigned int kTriggerPin3     = 16;   // A2  "Right"
constexpr unsigned int kEchoPin3        = 16;   // A2  "Right"

constexpr unsigned int kSonarNum        = 3;
constexpr unsigned int kMaxDistance     = 150;  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
constexpr unsigned int kPingInterval    = 33;

extern pet::ros::NodeHandle nh;
static pet_mk_iv_msgs::DistanceMeasurement distSensorMsg;
static ros::Publisher distSensorPub("dist_sensors", &distSensorMsg);

static unsigned long pingTimer[kSonarNum];
static uint8_t currentSensor = 0;

constexpr char distSensorFrames[kSonarNum][14] = {
    "dist_sensor_0",
    "dist_sensor_1",
    "dist_sensor_2"};

static NewPing sonar[kSonarNum] = {
    NewPing(kTriggerPin1, kEchoPin1, kMaxDistance),
    NewPing(kTriggerPin2, kEchoPin2, kMaxDistance),
    NewPing(kTriggerPin3, kEchoPin3, kMaxDistance)};

void distSensorSetup()
{
    pingTimer[0] = millis() + 75;
    for (uint8_t i = 1; i < kSonarNum; ++i)
    {
        pingTimer[i] = pingTimer[i - 1] + kPingInterval;
    }

    distSensorMsg.header.frame_id = "dist_sensor_x";
    nh.advertise(distSensorPub);
}

void distSensorUpdate()
{
    for (uint8_t i = 0; i < kSonarNum; ++i)
    {
        if (millis() >= pingTimer[i])
        {
            pingTimer[i] += kPingInterval * kSonarNum;
            sonar[currentSensor].timer_stop();
            currentSensor = i;
            sonar[currentSensor].ping_timer(echoCheck);
        }
    }

    distSensorPub.publish(&distSensorMsg);
}

/// WARNING! This function is called inside an interrupt.
void echoCheck()
{
    if (sonar[currentSensor].check_timer())
        publishResult(currentSensor, sonar[currentSensor].ping_result * 10 / US_ROUNDTRIP_CM);
}

/// WARNING! This function is called inside an interrupt.
void publishResult(uint8_t sensor, int16_t dist)
{
    distSensorMsg.header.stamp = nh.now();
    distSensorMsg.header.frame_id = distSensorFrames[sensor];
    distSensorMsg.distance = dist;
}
