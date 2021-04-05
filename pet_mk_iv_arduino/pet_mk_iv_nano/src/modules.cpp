#include "modules.h"

#include "timer.h"

#include "ir_remote_module.h"
#include "light_beacon_module.h"

namespace pet
{

Timer<4> g_timer{};

ConfigResult configure_modules()
{
    constexpr int kIrReceiverPin = 11;
    static pet::IrRemoteModule ir_remote_module{kIrReceiverPin};
    if (!g_timer.register_module(&ir_remote_module)) {
        return ConfigResult::TimerRegistrationError;
    }

    // Pins to attach servo to.
    constexpr int kServoSignalPin = 3;     // Yellow wire
    constexpr int kServoPowerPin = 2;      // Red wire
    static pet::LightBeaconModule light_beacon_module{kServoSignalPin, kServoPowerPin};
    if (!g_timer.register_module(&light_beacon_module)) {
        return ConfigResult::TimerRegistrationError;
    }

    return ConfigResult::Success;
}

} // namespace pet
