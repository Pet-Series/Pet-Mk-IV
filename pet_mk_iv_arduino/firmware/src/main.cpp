#include <stdint.h>

#include <ros/time.h>
#include <ros/duration.h>

#include "rosserial_node.h"
#include "arduino_module.h"
#include "timer.h"
#include "engines.h"
#include "line_followers.h"
#include "dist_sensors.h"

pet::Timer<3> timer;

// Synchronises topic information and time stamp with the rosserial server.
void synchronise_with_server()
{
    pet::nh.negotiateTopics();

    uint32_t last_sync_time = pet::nh.get_last_sync_receive_time();
    pet::nh.requestSyncTime();
    while (last_sync_time == pet::nh.get_last_sync_receive_time())
    {
        pet::nh.spinOnce();
    }
}

enum class ConfigResult
{
    Success,
    AllocationError,
    TimerRegistrationError,
};

// Configure what modules to run based on ROS parameters.
// TODO: Read ROS parameters.
ConfigResult configure_modules()
{
    constexpr bool use_engines        = true;
    constexpr bool use_line_followers = true;
    constexpr bool use_dist_sensors   = true;

    // NOTE: We only use heap allocation at configuration time.
    if (use_engines)
    {
        pet::ArduinoModule* engine_module = new pet::Engines();
        if (!engine_module) {
            return ConfigResult::AllocationError;
        }
        if (!timer.register_module(engine_module)) {
            return ConfigResult::TimerRegistrationError;
        }
    }

    if (use_line_followers)
    {
        pet::ArduinoModule* line_follower_module = new pet::LineFollowers();
        if (!line_follower_module) {
            return ConfigResult::AllocationError;
        }
        if (!timer.register_module(line_follower_module)) {
            return ConfigResult::TimerRegistrationError;
        }
    }

    if (use_dist_sensors)
    {
        pet::ArduinoModule* dist_sensor_module = new pet::DistSensors();
        if (!dist_sensor_module) {
            return ConfigResult::AllocationError;
        }
        if (!timer.register_module(dist_sensor_module)) {
            return ConfigResult::TimerRegistrationError;
        }
    }

    return ConfigResult::Success;
}

void setup()
{
    pet::nh.initNode();

    while (!pet::nh.connected())
    {
        pet::nh.spinOnce();
    }

    pet::nh.loginfo("Arduino starting...");
    
    ConfigResult result = configure_modules();
    switch (result)
    {
    case ConfigResult::Success:
        pet::nh.loginfo("Module setup done.");
        break;
    case ConfigResult::AllocationError:
        pet::nh.logerror("AllocationError during module setup!");
        break;
    case ConfigResult::TimerRegistrationError:
        pet::nh.logerror("TimerRegistrationError during module setup!");
        break;
    }

    // Ensure topic information is updated on server-side.
    synchronise_with_server();

    timer.start();

    pet::nh.loginfo("Arduino setup done!");
}

void loop()
{
    pet::nh.spinOnce();
    timer.spin_once();
}
