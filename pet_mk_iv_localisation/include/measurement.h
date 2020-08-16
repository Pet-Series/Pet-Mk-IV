#ifndef PET_LOCALISATION_MEASUREMENT_H
#define PET_LOCALISATION_MEASUREMENT_H

#include <ros/time.h>

namespace pet
{

class MeasurementBase
{
public:
    const ros::Time& stamp() const
    {
        return m_stamp;
    }

protected:
    MeasurementBase(const ros::Time& stamp)
        : m_stamp(stamp)
    {
    }

private:
    ros::Time m_stamp;
};

inline
bool operator<(const MeasurementBase& lhs, const MeasurementBase& rhs)
{
    // Purposfully inverted. We want old measurement to be in front in std::priority_queue.
    return lhs.stamp() > rhs.stamp();
}

} // namespace pet

#endif // PET_LOCALISATION_MEASUREMENT_H