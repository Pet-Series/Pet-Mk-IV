#include "modules.h"

#include "timer.h"

#include "ir_remote_module.h"
#include "light_beacon_module.h"

namespace pet
{

Timer<4> g_timer{};

ConfigResult configure_modules()
{
    static pet::IrRemoteModule ir_remote_module{};
    if (!g_timer.register_module(&ir_remote_module)) {
        return ConfigResult::TimerRegistrationError;
    }

    static pet::LightBeaconModule light_beacon_module{};
    if (!g_timer.register_module(&light_beacon_module)) {
        return ConfigResult::TimerRegistrationError;
    }

    return ConfigResult::Success;
}

} // namespace pet
