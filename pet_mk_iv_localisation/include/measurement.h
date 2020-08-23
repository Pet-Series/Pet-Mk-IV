#ifndef PET_LOCALISATION_MEASUREMENT_H
#define PET_LOCALISATION_MEASUREMENT_H

#include <memory>

#include <ros/time.h>

namespace pet
{

class Measurement
{
public:
    virtual ~Measurement() = default;

    const ros::Time& stamp() const
    {
        return m_stamp;
    }

protected:
    Measurement(const ros::Time& stamp);

private:
    ros::Time m_stamp;
};

} // namespace pet

#endif // PET_LOCALISATION_MEASUREMENT_H