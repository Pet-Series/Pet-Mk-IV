#ifndef ROTATING_LED_H
#define ROTATING_LED_H

#include <Servo.h>  // Used as a "fake servo" to control the RC-gadget "rotating LED" 

// Rotating LED: Fake/Simulate a on/off switch on your RC-Controller.
constexpr int POS_OFF = 135; // Fake OFF position(unit=degrees) on RC controller Chanel on/off switch
constexpr int POS_ON  = 45; // Fake ON  position(unit=degrees) on RC controller Chanel on/off switch
constexpr int PRESS   = 50; // Fake time - For how long time(unit=milliseconds) is the button presse down on RC controller Chanel on/off switch

constexpr int kRotatingLEDpin = 3; // Define Rotating LED pin

/// Rotating LED : Mode Cycling (simulate a on/off switch on your RC-Controller)
constexpr int kRotatingFast = 0;
constexpr int kRotatingSlow = 1; 
constexpr int kFlashing     = 2;
constexpr int kStrobing     = 3;
constexpr int kLED_off      = 4;

class RotatingLED
{
public:
    Servo m_LED_servo; // Instansera "servo" object för LED  (req. #include <Servo.h>)
    int m_current_mode = kRotatingFast; // At power up

    RotatingLED();
    
    void init();

    void setMode(int new_mode);

    void cycleMode(int number_of_cycles);

    void printMode();
    
    void printAttached();

};


#endif
