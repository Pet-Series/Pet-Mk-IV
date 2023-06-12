/*   
  <arduino_lineSesnor_FC-123.ino> 
  Test of InfraRed reflection sensor - Directed downward toward the floor
  Hardware/Unit test for:
    - Pet Mark V (five)

  Expected behaviour: 
    When black patch on floor is detected (no...low reflection) 
    Then:  Switch ON the built in LED. 
          Write log statement on console output. 
    When no black patch on floor is detected (reflection against the floor) 
    Then: Switch OFF the built in LED. 
          Write log statement on console output. 

  Hardware setup: 
  1) Arduino Nano 
  2) 1x FC-123 Infrared sensor for line tracing. 

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
    //    | Pin14 | PB2 | D10  | -> Infrared reciver KY-022 for remote controll via IR-Controller.
    //    | Pin15 | PB3 | D11  |     
    //    +-------+-----+------|_____
    //                         |_USB_|

  2021-04-05 / Stefan Kull (aka. "Seniorkullken") 
*/   

#include "Arduino.h"  /* Not neccesary when usinig Arduino IDE */

#define kSignalPin PD7   // Line sensor Arduino Nano pin 11
#define kLedPin    13    // Built in LED

// variables will change:
boolean lineSensor = LOW;         // variable for reading the line sensor


void setup()
{
  Serial.begin(115200);     // Serial Monitor @ baud...
  Serial.println("View digital in Pin11 / Port PD7 / PCB label D7");

  pinMode(kSignalPin, INPUT );
  pinMode(kLedPin,    OUTPUT);
}

// the loop function runs over and over again forever
void loop() 
{
  lineSensor = digitalRead(kSignalPin);
  
  if (lineSensor == 1)
  {
    Serial.println("NO line detected!");
    digitalWrite(kLedPin, LOW);
  } else {
    Serial.println("Line detected (no reflection from surface)");
    digitalWrite(kLedPin, HIGH);
  }
  
  delay(1000);
}
