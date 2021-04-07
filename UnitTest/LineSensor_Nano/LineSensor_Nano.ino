/*
  Line Follower Sensor Test.
*/



// Arduino Nano (ATmega328) Pin11 => "PD7" = "D7"



// Arduino Nano (ATmega328)
//     ____________________
//    | Physic|Port | PCB  |
//    +-------+-----+------+
//    | Pin31 | PD1 | TX1  |
//    | Pin30 | PD0 | RX0  |
//    | Pin29 | PC6 | RST  |
//    | GND   | GND | GND  | -> Servo GND
//    | Pin32 | PD2 |  D2  | -> Servo VCC
//    | Pin1  | PD3 |  D3  | -> Servo PWM Signa
//    | Pin2  | PD4 |  D4  |
//    | Pin9  | PD5 |  D5  |
//    | Pin10 | PD6 |  D6  |
//    | Pin11 | PD7 |  D7  | <- Line sensor
//    | Pin12 | PB0 |  D8  | 
//    | Pin13 | PB1 |  D9  | -> Built in LED
//    | Pin14 | PB2 | D10  | -> Light beacon Signal
//    | Pin15 | PB3 | D11  |     
//    +-------+-----+------|_____
//                         |_USB_|


#define kSignalPin PD7   // Line sensor Arduino Nano pin 11
#define kLedPin    13    // Built in LED

// variables will change:
int lineSensor = LOW;         // variable for reading the line sensor


void setup()
{
  Serial.begin(115200);     // Serial Monitor @ baud...
  Serial.println("View digital in Pin12 / Port PB0 / PCB label D8");

  pinMode(kSignalPin, INPUT );
  pinMode(kLedPin,    OUTPUT);
}

// the loop function runs over and over again forever
void loop() 
{
  lineSensor = digitalRead(kSignalPin);
  
  if (lineSensor == HIGH)
  {
    Serial.println("NO line detected!");
    digitalWrite(kLedPin, LOW);
  } else {
    Serial.println("Line detected (no reflection from surface)");
    digitalWrite(kLedPin, HIGH);
  }
  
  delay(1000);
}
