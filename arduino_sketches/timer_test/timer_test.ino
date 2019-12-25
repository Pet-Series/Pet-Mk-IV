#include "ros.h"
#include <ros/time.h>

#include "timer.h"
#include "engines.h"
#include "line_followers.h"
#include "dist_sensors.h"
#include "chatter.h"

#define ENGINE_CALLBACK_INTERVALL ros::Duration(0, 100000)     // 100 ms -> 10 Hz
#define LF_CALLBACK_INTERVALL ros::Duration(0, 100000)         // 100 ms -> 10 Hz
#define DIST_SENSOR_CALLBACK_INTERVALL ros::Duration(0, 33000) //  33 ms -> ~30 Hz

pet::ros::NodeHandle nh;
Timer<4> timer(nh);

void setup()
{
    nh.initNode();
    delay(1);

    nh.loginfo("Arduino starting...");
    nh.spinOnce();

    enginesSetup();
    lineFollowerSetup();
    distSensorSetup();
    chatterSetup();

    timer.register_callback(enginesUpdate, ENGINE_CALLBACK_INTERVALL);
    timer.register_callback(lineFollowerUpdate, LF_CALLBACK_INTERVALL);
    timer.register_callback(distSensorUpdate, DIST_SENSOR_CALLBACK_INTERVALL);
    timer.register_callback(chatterUpdate, ros::Duration(1, 500000));

    nh.loginfo("Arduino setup done!");
}

void loop()
{
    nh.spinOnce();
    timer.spin_once();
}
