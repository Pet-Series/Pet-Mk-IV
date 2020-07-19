#ifndef _PET_LINEFOLLOWERS_H
#define _PET_LINEFOLLOWERS_H

#include <ros/duration.h>

namespace line_followers
{

constexpr ros::Duration kPeriod(0, 10'000'000);

void setup();
void callback();

} // namespace line_followers

#endif