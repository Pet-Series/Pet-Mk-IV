#include "modules.h"

#include "timer.h"

#include "ir_remote_module.h"
#include "light_beacon_module.h"

namespace pet
{

Timer<4> g_timer{};

ConfigResult configure_modules()
{
    pet::ArduinoModule* ir_remote_module = new pet::IrRemoteModule();
    if (!ir_remote_module) {
        return ConfigResult::AllocationError;
    }
    if (!g_timer.register_module(ir_remote_module)) {
        return ConfigResult::TimerRegistrationError;
    }

    pet::ArduinoModule* light_beacon_module = new pet::LightBeaconModule();
    if (!light_beacon_module) {
        return ConfigResult::AllocationError;
    }
    if (!g_timer.register_module(light_beacon_module)) {
        return ConfigResult::TimerRegistrationError;
    }

    return ConfigResult::Success;
}

} // namespace pet
