#include <Arduino.h>
#include <stdint.h>

#include "ros.h"
#include <ros/time.h>
#include <ros/duration.h>

#include "timer.h"
#include "engines.h"
#include "line_followers.h"
// #include "dist_sensors.h"
#include "chatter.h"


static const ros::Duration kEngineCallbackInterval(0, 20'000'000);      // 20 ms -> 50 Hz
static const ros::Duration kLfCallbackInterval(0, 10'000'000);          // 10 ms -> 100 Hz
static const ros::Duration kDistSensorCallbackInterval(0, 33'000'000);  // 33 ms -> ~30 Hz

pet::ros::NodeHandle nh;
Timer<3> timer(nh);

void setup()
{
    nh.initNode();

    while (!nh.connected())
    {
        nh.spinOnce();
    }

    nh.loginfo("Arduino starting...");

    enginesSetup();
    lineFollowerSetup();
    // distSensorSetup();
    chatterSetup();

    // We need to renegotiate topics after setup-calls since new publishers/subscribers is registered.
    nh.negotiateTopics();

    // Ensure time is synced before continuing.
    uint32_t last_sync_time = nh.get_last_sync_receive_time();
    nh.requestSyncTime();
    while (last_sync_time == nh.get_last_sync_receive_time())
    {
        nh.spinOnce();
    }

    const ros::Duration start_delay(0, 200'000'000);
    const ros::Time start_time = nh.now() + start_delay;

    timer.register_callback(enginesUpdate, kEngineCallbackInterval, start_time);
    timer.register_callback(lineFollowerUpdate, kLfCallbackInterval, start_time);
    // timer.register_callback(distSensorUpdate, kDistSensorCallbackInterval);
    timer.register_callback(chatterUpdate, ros::Duration(0, 500'000'000), start_time);

    nh.loginfo("Arduino setup done!");
}

void loop()
{
    nh.spinOnce();
    timer.spin_once();
}
