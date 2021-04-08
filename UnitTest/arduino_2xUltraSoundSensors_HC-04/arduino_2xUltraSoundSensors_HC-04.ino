/*
  <arduino_2xUltraSoundSensors(HC-04).ino>
  Hardware/Unit test for Pet Mark V (five)
  
  Hardware setup:
  1) Arduino UNO
  2) 2x HC-04 Ultra sound distance sensors.

  2021-04-05 / Stefan Kull (aka. "Seniorkullken")
*/
 
#include "Arduino.h"  /* Not neccesary when usinig Arduino IDE */
#include "NewPing.h"
 
// Define Constants
#define TRIGGER_PIN_1  17  // A3  "Front"
#define ECHO_PIN_1     17  // A3  "Front"
#define TRIGGER_PIN_2  16  // A2  "Rear"
#define ECHO_PIN_2     16  // A2  "Rear"

#define MAX_DISTANCE  400  // // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
constexpr int iterations = 5;

// Create sensors
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
 
void setup() {
  Serial.begin (115200);

}
 
void loop()
{
    
  // Stores calculated speed of sound in m/s
  // soundspeed = 331.4 + (0.606 * temp) + (0.0124 * humidity);
  const float soundspeed = 331.4;  
    

  const float soundspeedcm = soundspeed / 10000;            // Convert m/s => cm
  
  // Measure duration for first ultra sound sensor   
  const auto duration1 = sonar1.ping_median(iterations); 
  const auto distance1 = (duration1 / 2) * soundspeedcm;    // Calculate the distances 
  printDistance(distance1, " Distance1(Front): ");          // Send results to Serial Monitor
  // Add a delay between sensor readings
  delay(1000);
  
  // Measure duration for second ultra sound sensor 
  const auto duration2 = sonar2.ping_median(iterations);
  const auto distance2 = (duration2 / 2) * soundspeedcm;    // Calculate the distances 
  printDistance(distance2, " Distance2(Rear): ");           // Send results to Serial Monitor
  // Add a delay between sensor readings
  delay(1000);

  Serial.println("");
}

void printDistance(float distance, const char* identifier)
{
  Serial.print(identifier);

  if (distance >= 400 || distance <= 2)
  {
    Serial.print(distance);
    Serial.print(" cm ");
    Serial.print(" <= Out of range!!");
  }
  else
  {
    Serial.print(distance);
    Serial.print(" cm ");
  }
}
