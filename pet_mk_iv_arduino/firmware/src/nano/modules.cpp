#include "modules.h"

#include "timer.h"

#include "light_beacon.h"

namespace pet
{

Timer<4> g_timer{};

ConfigResult configure_modules()
{
    pet::ArduinoModule* light_beacon_module = new pet::LightBeacon();
    if (!light_beacon_module) {
        return ConfigResult::AllocationError;
    }
    if (!g_timer.register_module(light_beacon_module)) {
        return ConfigResult::TimerRegistrationError;
    }

    return ConfigResult::Success;
}

} // namespace pet
