#ifndef _PET_IRREMOTE_H
#define _PET_IRREMOTE_H

#include <ros/duration.h>

namespace irremote
{

static const ros::Duration kPeriod;

void setup();       // Will be exposed as "irremote::setup()"
void callback();    // Will be exposed as "irremote::callback()"
}

#endif