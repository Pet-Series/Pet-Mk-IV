/*
  IR Receiver Demonstration
  Determine IR codes manufacturer type with IR Receiver
  Displays results on Serial Monitor
  Based on https://github.com/z3t0/Arduino-IRremote

  Added fetaure to set/toggle "Rotating LED" depending on which key is pressed. 
*/

#include <Servo.h>        // Used as a "fake servo" to control the RC-gadget "rotating LED" 
#include <IRremote.h>     // Include IR Remote (Library by Ken Shirriff)
#include "IRremoteLib.h"  // Mapping of key-codes for the IR remote


// Servo m_LED_servo; // Instansera "servo" object för LED  (req. #include <Servo.h>)


#include "RotatingLED.h"  //
RotatingLED rotating_LED;

// Define IR Receiver and Results Objects
#define RECV_PIN          11 // IR-reciver
IRrecv irrecv(RECV_PIN);
decode_results IR_decode_results;

const static char long_str1[] PROGMEM = "Hi, I would like to tell you a bit about myself.\n";


void setup()
{
    Serial.begin(115200);     // Serial Monitor @ baud...
    Serial.println("IR/ & Rotating LED UnitTest - Debug 2.0 <via VScode>");
    rotating_LED.init();
    //RotatingLEDinit();
    rotating_LED.printMode();
    irrecv.enableIRIn();      // Enable the IR Receiver (see RECV_PIN value for IR-reciver pin
    Serial.println(long_str1);
}

void loop(){
    if (irrecv.decode(&IR_decode_results))
    {
        Serial.print("IR-Input Code=[");
        Serial.print(IR_decode_results.value, HEX);
        Serial.println("] ");

        // Encode the raw-code vs. predifined patterns
        switch (IR_decode_results.value) 
        {
            case IR_RC6_0_t1:
            case IR_RC6_0_t2:
            case IR_NEC_0:
                Serial.println(("  IR/Key=[0]"));
                break;
            case IR_RC6_1_t1:
            case IR_RC6_1_t2:
            case IR_NEC_1:
                Serial.println(("  IR/Key=[1]"));
                //  setRotatingLEDmode(kRotatingFast);
                rotating_LED.setMode(kRotatingFast);
                break;
            case IR_RC6_2_t1:
            case IR_RC6_2_t2:
            case IR_NEC_2:
                Serial.println(("  IR/Key=[2]"));
                //  setRotatingLEDmode(kRotatingSlow);
                rotating_LED.setMode(kRotatingSlow);    
                break;
            case IR_RC6_3_t1:
            case IR_RC6_3_t2:
            case IR_NEC_3:
                Serial.println(("  IR/Key=[3]"));
                //  setRotatingLEDmode(kFlashing);
                rotating_LED.setMode(kFlashing);
                break;
            case IR_RC6_4_t1:
            case IR_RC6_4_t2:
            case IR_NEC_4:
                Serial.println(("  IR/Key=[4]"));
                //  setRotatingLEDmode(kStrobing);
                rotating_LED.setMode(kStrobing);
                break;              
            case IR_RC6_5_t1:
            case IR_RC6_5_t2:
            case IR_NEC_5:
                Serial.println(("  IR/Key=[5]"));
                //  setRotatingLEDmode(kLED_off);
                rotating_LED.setMode(kLED_off);
                break;
            case IR_RC6_6_t1:
            case IR_RC6_6_t2:
            case IR_NEC_6:
                Serial.println(("  IR/Key=[6]"));
                rotating_LED.cycleMode(1);
                break;
            case IR_RC6_7_t1:
            case IR_RC6_7_t2:
            case IR_NEC_7:
                Serial.println(("  IR/Key=[7]"));
                break;
            case IR_RC6_8_t1:
            case IR_RC6_8_t2:
            case IR_NEC_8:
                Serial.println(("  IR/Key=[8]"));
                break;              
           case IR_RC6_9_t1:
           case IR_RC6_9_t2:
           case IR_NEC_9:
               Serial.print("Key=[9]");
               break;
            default:
                Serial.println(("  IR/Key= ...no match"));
                break;
        }

        // Serial.println(" ...Å ena sidan = 'ON'");
        // rotating_LED.m_LED_servo.write(POS_ON);
        // delay(PRESS);
        // Serial.println("  - Å andra sidan = 'OFF'");
        // rotating_LED.m_LED_servo.write(POS_OFF);
        // delay(PRESS);    
        // Serial.print("....attached=");
        // Serial.println(rotating_LED.m_LED_servo.attached()); 

        irrecv.resume();
   }
}

/// --------------------------------------
/// Från .cpp filen :-)
/// --------------------------------------
void RotatingLEDinit()
{
    // create LCD object to be controled as a RC-servo :-)
    rotating_LED.m_LED_servo.attach(kRotatingLEDpin); // Attaches the servo on pin 3 to the servo objec
    rotating_LED.m_LED_servo.write(POS_OFF); // Cycle  ...start pos of "switch"
    delay(PRESS);
    rotating_LED.setMode(kLED_off);
    rotating_LED.printAttached();
}

// void setRotatingLEDmode(uint8_t newMode)
// {
//     if (newMode < rotating_LED.m_current_mode)
//     {
//         cycleRotatingLEDmode(newMode - rotating_LED.m_current_mode + 5);
//         Serial.println("\n    +5 ");
//     }
//     else
//     {
//         cycleRotatingLEDmode(newMode - rotating_LED.m_current_mode);
//     }
//     rotating_LED.m_current_mode = newMode; // Yhe King is dead. God save the new King!
// }

// void printRotatingLEDmode()
// {
//     switch (rotating_LED.m_current_mode) {
//         case kRotatingFast:
//             Serial.println("\nCurrentMode='RotatingFast'"); // 1
//             break;
//         case kRotatingSlow:
//             Serial.println("\nCurrentMode='RotatingSlow'"); // 2
//             break;
//         case kFlashing:
//             Serial.println("\nCurrentMode='Flashing'");     // 3
//             break;
//         case kStrobing:
//             Serial.println("\nCurrentMode='Strobing'");     // 4
//             break;
//         case kLED_off:
//             Serial.println("\nCurrentMode='LED_off'");      // 5
//             break;
//         default:
//             Serial.println("\nCurrentMode='FooBar'");
//             break;         
//     }
// }

// void cycleRotatingLEDmode(uint8_t NoCycles)
// {
//     printRotatingLEDmode();
//     for (uint8_t i = 0; i < NoCycles; ++i)
//     {
//         Serial.print("   :-() cycleRotatingLEDmode: Switch is pressed = 'ON'");
//         rotating_LED.m_LED_servo.write(POS_ON);
//         delay(PRESS);
//         Serial.println(" ...and then released = 'OFF'");
//         rotating_LED.m_LED_servo.write(POS_OFF);
//         delay(PRESS);
//     }
// }