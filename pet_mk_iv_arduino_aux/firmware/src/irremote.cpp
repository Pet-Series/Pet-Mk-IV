
#include "ros.h"
#include <IRremote.h>     // Include IR Remote (Library by Ken Shirriff)
// #include "IRremoteLib.h"  // Mapping of key-codes for the IR remote
#include <pet_mk_iv_msgs/IrRemote.h>

extern pet::ros::NodeHandle nh;

namespace irremote
{

using pet_mk_iv_msgs::IrRemote;

// Define IR Receiver and Results Objects
#define RECV_PIN          11 // IR-receiver Arduino pin 
// IRrecv irrecv(RECV_PIN);
decode_results IR_decode_results;

pet_mk_iv_msgs::IrRemote irRemoteMsg;
ros::Publisher irRemotePub("irremote", &irRemoteMsg);

// Will be exposed as "irremote::setup()"
void setup()                  
{
    nh.advertise(irRemotePub);
    // irrecv.enableIRIn();      // Enable the IR Receiver (see RECV_PIN value for IR-reciver pin
    irRemoteMsg.data = IrRemote::OK;
}

// Will be exposed as "irremote::callback()"
void callback()               
{
    irRemotePub.publish(&irRemoteMsg);
}

} // namespace irremote