#ifndef _PET_ARDUINO_MODULE_H
#define _PET_ARDUINO_MODULE_H

namespace pet
{

// Abstract base class for Arduino software modules.
// TODO: Rename to ArduinoTask?
class ArduinoModule
{
public:
    virtual void callback() = 0;
    virtual double frequency() const = 0;
};

} // namespace pet

#endif // _PET_ARDUINO_MODULE_H