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
int kRecvPin     =     11; // IR-receiver Arduino Nano pin
IRrecv m_irrecv{kRecvPin}; // IRrecv från <IRremote.h>



void setup()
{
    Serial.begin(115200);     // Serial Monitor @ baud...
    Serial.println("IR/ & Rotating LED UnitTest - Debug 2.0 <via VScode>");
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
    
    //RotatingLEDinit();
    rotating_LED.init();
    rotating_LED.printMode();
 
    m_irrecv.enableIRIn();      // Enable the IR Receiver (see kRecvPin value for IR-receiver pin)
    
}

void loop(){
    if (m_irrecv.decode())
    {
        Serial.print("IR-Input Code=[");
        Serial.print(m_irrecv.results.value, HEX);
//        Serial.print(IrReceiver.decodedIRData.command, HEX);
//        Serial.print(IrReceiver.decodedIRData.command);
        Serial.println("] ");

        // Print a short summary of received data
        // IrReceiver.printIRResultShort(&Serial);
        Serial.println();

        // Encode the raw-code vs. predifined patterns
        switch (m_irrecv.results.value) 
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

        m_irrecv.resume();
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
