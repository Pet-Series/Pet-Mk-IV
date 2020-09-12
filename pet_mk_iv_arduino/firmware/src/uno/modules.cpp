#include "modules.h"

#include "timer.h"

#include "engines.h"
#include "line_sensor_module.h"
#include "dist_sensors.h"

namespace pet
{

Timer<4> g_timer{};

ConfigResult configure_modules()
{
    pet::ArduinoModule* engine_module = new pet::Engines();
    if (!engine_module) {
        return ConfigResult::AllocationError;
    }
    if (!g_timer.register_module(engine_module)) {
        return ConfigResult::TimerRegistrationError;
    }

    pet::ArduinoModule* line_follower_module = new pet::LineSensorModule();
    if (!line_follower_module) {
        return ConfigResult::AllocationError;
    }
    if (!g_timer.register_module(line_follower_module)) {
        return ConfigResult::TimerRegistrationError;
    }

    pet::ArduinoModule* dist_sensor_module = new pet::DistSensors();
    if (!dist_sensor_module) {
        return ConfigResult::AllocationError;
    }
    if (!g_timer.register_module(dist_sensor_module)) {
        return ConfigResult::TimerRegistrationError;
    }

    return ConfigResult::Success;
}

} // namespace pet
