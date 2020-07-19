#ifndef _PET_ENGINES_H
#define _PET_ENGINES_H

#include <ros/duration.h>

#include "pet_mk_iv_msgs/EngineCommand.h"

namespace engines
{

constexpr ros::Duration kPeriod(0, 20'000'000);

void setup();
void callback();

void engineCommandCb(const pet_mk_iv_msgs::EngineCommand& msg);
void setEnginePWM(const pet_mk_iv_msgs::EngineCommand& cmd);
void emergencyStop();

} // namespace engines

#endif