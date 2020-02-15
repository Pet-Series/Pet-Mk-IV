#ifndef _PET_ENGINES_H
#define _PET_ENGINES_H

#include "pet_mk_iv_msgs/EngineCommand.h"

void enginesSetup();
void enginesUpdate();

void engineCommandCb(const pet_mk_iv_msgs::EngineCommand& msg);

#endif