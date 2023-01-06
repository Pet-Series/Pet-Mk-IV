#include "RotatingLED.h"

#include <Arduino.h>

RotatingLED::RotatingLED(){
    
}

void RotatingLED::init()
{
    this->m_LED_servo.attach(kRotatingLEDpin); // Attaches the servo on pin 3 to the servo objec
    this->m_LED_servo.write(POS_OFF); // Cycle  ...start pos of "switch"
    delay(PRESS);
    this->setMode(kLED_off);
    this->printAttached();
}

void RotatingLED::setMode(int new_mode)
{
    this->cycleMode((new_mode - this->m_current_mode + 5) % 5); // 5 = Number of modes to cycle
    this->m_current_mode = new_mode; // The King is dead. God save the new King!
}

void RotatingLED::cycleMode(int number_of_cycles)
{
    for (int i = 0; i < number_of_cycles; ++i)
    {
        this->m_LED_servo.write(POS_ON);
        delay(PRESS);
        this->m_LED_servo.write(POS_OFF);
        delay(PRESS);
    }
    
}

void RotatingLED::printMode()
{
    switch (this->m_current_mode)
    {
        case kRotatingFast:
            Serial.println("  RotatingLED.printMode='RotatingFast'");
            break;
        case kRotatingSlow:
            Serial.println("  RotatingLED.printMode='RotatingSlow'");
            break;
        case kFlashing:
            Serial.println("  RotatingLED.printMode='Flashing'");
            break;
        case kStrobing:
            Serial.println("  RotatingLED.printMode='Strobing'");
            break;
        case kLED_off:
            Serial.println("  RotatingLED.printMode='LED_off'");
            break;
        default:
            Serial.println("  RotatingLED.printMode='Fubar'");       // "Fucked Up Beyond All Recognition"
            break;         
    }
}

void RotatingLED::printAttached()
{
    Serial.print("  RotatingLED.printAttached (T/F)=");
    Serial.println(this->m_LED_servo.attached()); 
}

