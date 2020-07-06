#ifndef _PET_CHATTER_H
#define _PET_CHATTER_H

#include <ros/duration.h>

namespace chatter
{

static const ros::Duration kPeriod;

// Will be exposed as "chatter::setup()"
void setup();

// Will be exposed as "chatter::callback()"   
void callback();

} // namespace chatter

#endif