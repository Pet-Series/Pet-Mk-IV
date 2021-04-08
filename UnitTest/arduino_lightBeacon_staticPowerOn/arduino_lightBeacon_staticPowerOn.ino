/*   
  <arduino_lightBeacon_staticPowerOn.ino> 
  Static Power on test for rotation light beacon. 
  Hardware/Unit test applicable for:
    - Pet Mark  V (five)
    - Pet Mark IV (four) 

  Expected behaviour: 
    When power up... 
    Then Beacon starts fast rotating/blink (power up default mode) 

  Hardware setup: 
  1) Arduino Nano 
  2) 1x Rotating beacon (from RC-truck/car accessories. ) 

      // Pin out for Arduino Nano (ATmega328)
      //     ____________________
      //    | Physic|Port | PCB  |
      //    +-------+-----+------+
      //    | Pin31 | PD1 | TX1  |
      //    | Pin30 | PD0 | RX0  |
      //    | Pin29 | PC6 | RST  |
      //    | GND   | GND | GND  | -> Servo GND
      //    | Pin32 | PD2 |  D2  | -> Servo VCC (Light beacon VCC)
      //    | Pin1  | PD3 |  D3  | -> Servo PWM Signal (Light beacon Signal)
      //    | Pin2  | PD4 |  D4  |
      //    | Pin9  | PD5 |  D5  |
      //    | Pin10 | PD6 |  D6  |
      //    | Pin11 | PD7 |  D7  | <- Line sensor FC-123
      //    | Pin12 | PB0 |  D8  | 
      //    | Pin13 | PB1 |  D9  | -> Built in LED
      //    | Pin14 | PB2 | D10  | -> Infrared receiver KY-022 for remote control via IR-Controller.
      //    | Pin15 | PB3 | D11  |     
      //    +-------+-----+------|_____
      //                         |_USB_|

  2021-04-05 / Stefan Kull (aka. "Seniorkullken") 
*/
#include "Arduino.h"  /* Not necessary when using Arduino IDE */

#define kSignal PD3 // IR-receiver Arduino Nano pin
#define kPower  PD2 // IR-receiver Arduino Nano pin


void setup()
{
//    Serial.begin(115200);     // Serial Monitor @ baud...
//    Serial.println("IR/ & Rotating LED UnitTest - Debug 2.0 <via VScode>");

  //pinMode(kSignal, OUTPUT);
  pinMode(kPower , OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  // digitalWrite(kSignal, HIGH);  
  digitalWrite(kPower , HIGH);   
}
