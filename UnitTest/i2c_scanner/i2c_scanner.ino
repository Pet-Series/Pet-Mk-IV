// --------------------------------------
// i2c_scanner
// https://playground.arduino.cc/Main/I2cScanner/
// Scans i2c adresses within the standard 7-bit(0x07F) address range...
// ...
// Input = n/a
// Output= Logg/progress via IDE-serial monitor.
// As a reference the table below shows where TWI pins are located on various Arduino boards.
//   Board         I2C / TWI pins
//   Uno, Ethernet A4(SDA), A5(SCL)
//   Mega2560      20(SDA), 21(SCL)
//   Leonardo       2(SDA), 3 (SCL)
//   Due           20(SDA), 21(SCL), SDA1, SCL1
//   NodeMcu       D2(SDA), D1(SLC)

// Note#1 Loop only ONCE!!!
//        To start/repeat scanning - Press Reset [RST] on your board.
// Note#2 Need to watch out for the WatchDog that will restet the chip ESP8266 => Using "yield();"
//
//
//
 
#include <Wire.h>

boolean isGoal = false;

void setup()
{
  Wire.begin();
 
  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner (setup)");
}
 
 
void loop()
{
  byte error, address;
  int nDevices;

  while ( !isGoal )
    {
    Serial.println("Start Scanning loop...");
   
    nDevices = 0;
    // Loop truth I2C adress 1..127 (0x07F)
    for(address = 1; address < 127; address++ )
    {
      // The i2c_scanner uses the return value of
      // the Write.endTransmisstion to see if
      // a device did acknowledge to the address.
      
      //***** Show next I2C adress to be tested...  
      Serial.print("I2C device testing 0x");
      if (address<16)
         Serial.print("0");
      Serial.print(address,HEX);
      Serial.println(" ?");
  
      //***** The test itself - Try to initialse I2C adress...     
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
   
      if (error == 0)
      {
          Serial.print("I2C device found at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.print(address,HEX);
          Serial.println("  !");
   
          nDevices++;
      }
      else if (error==4)
      {
        Serial.print("Unknown error at address 0x");
        if (address<16) 
        {
           Serial.print("0");
           Serial.println(address,HEX);
        }
      }
      yield();   
    }
    //***** Now the fun is over
    isGoal = true;
    if (nDevices == 0){
      Serial.println("No I2C devices found\n");
    } else {
      Serial.print("Done - ");
      Serial.print(nDevices);
      Serial.println(" I2C device found!");  
    }
  }
  // Serial.print(".");
}
