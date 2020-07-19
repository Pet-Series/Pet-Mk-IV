#include <Arduino.h>
#include <stdint.h>

#include "ros.h"
#include <ros/time.h>
#include <ros/duration.h>

#include "timer.h"
#include "engines.h"
#include "line_followers.h"
#include "dist_sensors.h"

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

    engines::setup();
    line_followers::setup();
    dist_sensors::setup();

    // We need to renegotiate topics after setup-calls since new publishers/subscribers is registered.
    nh.negotiateTopics();

    // Ensure time is synced before continuing.
    uint32_t last_sync_time = nh.get_last_sync_receive_time();
    nh.requestSyncTime();
    while (last_sync_time == nh.get_last_sync_receive_time())
    {
        nh.spinOnce();
    }

    timer.register_callback(engines::callback, engines::kPeriod);
    timer.register_callback(line_followers::callback, line_followers::kPeriod);
    timer.register_callback(dist_sensors::callback, dist_sensors::kPeriod);

    nh.loginfo("Arduino setup done!");
}

void loop()
{
    nh.spinOnce();
    timer.spin_once();
}
