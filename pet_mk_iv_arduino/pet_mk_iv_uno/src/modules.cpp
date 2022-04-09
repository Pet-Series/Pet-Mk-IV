#include "modules.h"

#include <timer.h>

#include "engine_module.h"
#include "line_sensor_module.h"
#include "ultrasound_module.h"

namespace pet
{

Timer<kMaxNumModules> g_timer{};

ConfigResult configure_modules()
{
    static pet::EngineModule engine_module{};
    if (!g_timer.register_module(&engine_module)) {
        return ConfigResult::TimerRegistrationError;
    }

    static pet::LineSensorModule line_sensor_left{Pin{2}, "line_sensor/left"};
    static pet::LineSensorModule line_sensor_middle{Pin{3}, "line_sensor/middle"};
    static pet::LineSensorModule line_sensor_right{Pin{4}, "line_sensor/right"};

    if (!g_timer.register_module(&line_sensor_left)) {
        return ConfigResult::TimerRegistrationError;
    }
    if (!g_timer.register_module(&line_sensor_middle)) {
        return ConfigResult::TimerRegistrationError;
    }
    if (!g_timer.register_module(&line_sensor_right)) {
        return ConfigResult::TimerRegistrationError;
    }

    constexpr int kSensorCount = 1;
    // constexpr int kSensorCount = 3;
    constexpr int ultrasound_pins[kSensorCount] = {
        // A0,
        A1,
        // A2
    };
    constexpr const char* sensor_ids[kSensorCount] = {
        // "range_sensor/right",
        "range_sensor/middle",
        // "range_sensor/left"
    };
    static pet::UltrasoundModule<kSensorCount> ultrasound_module{ultrasound_pins, sensor_ids};
    if (!g_timer.register_module(&ultrasound_module)) {
        return ConfigResult::TimerRegistrationError;
    }

    return ConfigResult::Success;
}

} // namespace pet
