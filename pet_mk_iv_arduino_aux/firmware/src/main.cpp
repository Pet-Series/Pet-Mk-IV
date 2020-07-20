#include <Arduino.h>
#include <stdint.h>

#include "ros.h"
#include "timer.h"
#include "chatter.h"
#include "irremote.h"

pet::ros::NodeHandle nh;
Timer<2> timer(nh);

void synchronise_with_server()
{
    nh.negotiateTopics();

    uint32_t last_sync_time = nh.get_last_sync_receive_time();
    nh.requestSyncTime();
    while (last_sync_time == nh.get_last_sync_receive_time())
    {
        nh.spinOnce();
    }
}

void setup() 
{
    nh.initNode();

    while (!nh.connected())
    {
        nh.spinOnce();
    }

    nh.loginfo("Arduino Aux starting...");
    
    irremote::setup();
    chatter::setup();

    // Ensure topic information is updated on server-side.
    synchronise_with_server();

    timer.register_callback(irremote::callback, irremote::kPeriod);
    timer.register_callback(chatter::callback,  chatter::kPeriod);

    nh.loginfo("Arduino Aux setup done!");
}

void loop() 
{ 
    nh.spinOnce();
    timer.spin_once();
}
