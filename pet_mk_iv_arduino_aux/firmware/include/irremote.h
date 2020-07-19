#ifndef _PET_IRREMOTE_H
#define _PET_IRREMOTE_H

#include <ros/duration.h>

namespace irremote
{

constexpr ros::Duration kPeriod(2, 0);

void setup();       // Will be exposed as "irremote::setup()"
void callback();    // Will be exposed as "irremote::callback()"
}

#endif