#include "irremote.h"

#include "ros.h"
#include <IRremote.h>     // Include IR Remote (Library by Ken Shirriff)
#include "IRremoteLib.h"  // Mapping of key-codes for the IR remote
#include <pet_mk_iv_msgs/IrRemote.h>

extern pet::ros::NodeHandle nh;

namespace irremote
{

using pet_mk_iv_msgs::IrRemote;

// Define IR Receiver and Results Objects
#define RECV_PIN          11 // IR-receiver Arduino Nano pin 
IRrecv irrecv(RECV_PIN);
decode_results IR_decode_results;

pet_mk_iv_msgs::IrRemote irRemoteMsg;
ros::Publisher irRemotePub("irremote", &irRemoteMsg);

// Will be exposed as "irremote::setup()"
void setup()                  
{
    nh.advertise(irRemotePub);
    irrecv.enableIRIn();      // Enable the IR Receiver (see RECV_PIN value for IR-receiver pin)
    irRemoteMsg.key = IrRemote::NOTSET;
}

// Will be exposed as "irremote::callback()"
void callback()               
{
    if (irrecv.decode(&IR_decode_results))
    {
        nh.loginfo("Tjoho...");     // TODO: Te be removed
        nh.loginfo(String(IR_decode_results.value).c_str()); // TODO: Te be removed

        switch (IR_decode_results.value)
        {
            case IR_RC6_OnOff_t1:
            case IR_RC6_OnOff_t2:
                irRemoteMsg.key = IrRemote::ON;
                // TODO: irRemoteMsg.key = IrRemote::OFF;
                break;
            // ****** Color keys 
            // TODO: RED / GREEN / YELLOW / BLUE
            // ****** Arrow & Navigation keys
            case IR_RC6_Ok_t1:
            case IR_RC6_Ok_t2:
                irRemoteMsg.key = IrRemote::OK;
                break;
            case IR_RC6_Up_t1:
            case IR_RC6_Up_t2:
                irRemoteMsg.key = IrRemote::UP;
                break;
            case IR_RC6_Down_t1:
            case IR_RC6_Down_t2:
                irRemoteMsg.key = IrRemote::DOWN;
                break;
            case IR_RC6_Left_t1:
            case IR_RC6_Left_t2:
                irRemoteMsg.key = IrRemote::LEFT;
                break;
            case IR_RC6_Right_t1:
            case IR_RC6_Right_t2:
                irRemoteMsg.key = IrRemote::RIGHT;
                break;
            case IR_RC6_Menu_t1:
            case IR_RC6_Menu_t2:
                irRemoteMsg.key = IrRemote::MENU;
                break;
            case IR_RC6_Info_t1:
            case IR_RC6_Info_t2:
                irRemoteMsg.key = IrRemote::INFO;
                break;
            case IR_RC6_Back_t1:
            case IR_RC6_Back_t2:
                irRemoteMsg.key = IrRemote::BACK;
                break;
            // ****** Multimedia keys
            case IR_RC6_Play_t1:
            case IR_RC6_Play_t2:
                irRemoteMsg.key = IrRemote::PLAY;
                break;
            case IR_RC6_Stop_t1:
            case IR_RC6_Stop_t2:
                irRemoteMsg.key = IrRemote::STOP;
                break;
            case IR_RC6_Pause_t1:
            case IR_RC6_Pause_t2:
                irRemoteMsg.key = IrRemote::PAUSE;
                break;
            case IR_RC6_FFwd_t1:
            case IR_RC6_FFwd_t2:
                irRemoteMsg.key = IrRemote::FAST_FWD;
                break;
            case IR_RC6_Rewind_t1:
            case IR_RC6_Rewind_t2:
                irRemoteMsg.key = IrRemote::REWIND;
                break;
            case IR_RC6_Mute_t1:
            case IR_RC6_Mute_t2:
                irRemoteMsg.key = IrRemote::MUTE;
                break;
            case IR_RC6_Win_t1:
            case IR_RC6_Win_t2:
                irRemoteMsg.key = IrRemote::WINDOWS;
                break;
            case IR_RC6_VolPlus_t1:
            case IR_RC6_VolPlus_t2:
                irRemoteMsg.key = IrRemote::VOL_PLUS;
                break;
            case IR_RC6_VolMinus_t1:
            case IR_RC6_VolMinus_t2:
                irRemoteMsg.key = IrRemote::VOL_MINUS;
                break;
            // TODO: IR_RC6_ChPlus  CH_PLUS
            // TODO: IR_RC6_ChMinus CH_MINUS
            // TODO: IR_RC6_Eject   EJECT
            // TODO: IR_RC6_Record  RECORD
            // ****** Numeric keys
            case IR_RC6_0_t1:
            case IR_RC6_0_t2:
                irRemoteMsg.key = IrRemote::NUM_0;
                break;
            case IR_RC6_1_t1:
            case IR_RC6_1_t2:
                irRemoteMsg.key = IrRemote::NUM_1;
                break;
            case IR_RC6_2_t1:
            case IR_RC6_2_t2:
                irRemoteMsg.key = IrRemote::NUM_2;
                break;
            case IR_RC6_3_t1:
            case IR_RC6_3_t2:
                irRemoteMsg.key = IrRemote::NUM_3;
                break;
            case IR_RC6_4_t1:
            case IR_RC6_4_t2:
                irRemoteMsg.key = IrRemote::NUM_4;
                break;              
            case IR_RC6_5_t1:
            case IR_RC6_5_t2:
                irRemoteMsg.key = IrRemote::NUM_5;
                break;
            case IR_RC6_6_t1:
            case IR_RC6_6_t2:
                irRemoteMsg.key = IrRemote::NUM_6;
                break;
            case IR_RC6_7_t1:
            case IR_RC6_7_t2:
                irRemoteMsg.key = IrRemote::NUM_7;
                break;
            case IR_RC6_8_t1:
            case IR_RC6_8_t2:
                irRemoteMsg.key = IrRemote::NUM_8;
                break;             
           case IR_RC6_9_t1:
           case IR_RC6_9_t2:
                 irRemoteMsg.key = IrRemote::NUM_9;
                break;
           case IR_RC6_Enter_t1:
           case IR_RC6_Enter_t2:
                 irRemoteMsg.key = IrRemote::NUM_ENTER;
                 break;
           case IR_RC6_Clear_t1:
           case IR_RC6_Clear_t2:
                 irRemoteMsg.key = IrRemote::NUM_CLEAR;
                break;
            // TODO: IR_RC6_Star    NUM_STAR
            // TODO: IR_RC6_Hash    NUM_HASH
            // ****** Unknown key is detected
            default:
                irRemoteMsg.key = IrRemote::UNKNOWN;

        }
        
        if (irRemoteMsg.key != IrRemote::UNKNOWN)
        {
            irRemotePub.publish(&irRemoteMsg);
        }
        irrecv.resume();
    }
}

} // namespace irremote