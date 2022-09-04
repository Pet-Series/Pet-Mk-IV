#include <ros/ros.h>

#include <pet_mk_iv_msgs/DistanceMeasurement.h>
#include <sensor_msgs/Range.h>

namespace pet
{

namespace
{

constexpr float deg2rad(float degrees)
{
    return degrees * M_PI / 180.0f;
}

} // namespace

class UltrasoundAdapter
{
public:
    UltrasoundAdapter();

private:
    void ultrasound_callback(const pet_mk_iv_msgs::DistanceMeasurement& msg);

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;

    ros::Subscriber m_subscriber;

    ros::Publisher m_publisher_left;
    ros::Publisher m_publisher_middle;
    ros::Publisher m_publisher_right;

    sensor_msgs::Range m_output_msg{};

    static constexpr double kMinDistance = 0.02; // [m]
    static constexpr double kMaxDistance = 4;    // [m]
};

UltrasoundAdapter::UltrasoundAdapter()
    : m_nh("")
    , m_nh_private("~")
{
    m_subscriber = m_nh.subscribe("dist_sensors", 10, &UltrasoundAdapter::ultrasound_callback, this);

    m_publisher_right  = m_nh.advertise<sensor_msgs::Range>("range_sensor/right", 10);
    m_publisher_middle = m_nh.advertise<sensor_msgs::Range>("range_sensor/middle", 10);
    m_publisher_left   = m_nh.advertise<sensor_msgs::Range>("range_sensor/left", 10);

    m_output_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    m_output_msg.field_of_view  = deg2rad(50);
    m_output_msg.min_range      = kMinDistance;
    m_output_msg.max_range      = kMaxDistance;
}

void UltrasoundAdapter::ultrasound_callback(const pet_mk_iv_msgs::DistanceMeasurement& msg)
{
    m_output_msg.header.stamp    = ros::Time::now();
    m_output_msg.header.frame_id = msg.header.frame_id;

    if (msg.distance == -1)
    {
        // distance == -1 have special meaning (no response) so we pass it forward directly.
        m_output_msg.range = -1;
    }
    else
    {
        // distance is received in mm.
        m_output_msg.range = msg.distance / 1000.0;
    }

    if (msg.header.frame_id == "range_sensor/right")
    {
        m_publisher_right.publish(m_output_msg);
    }
    else if (msg.header.frame_id == "range_sensor/middle")
    {
        m_publisher_middle.publish(m_output_msg);
    }
    else if (msg.header.frame_id == "range_sensor/left")
    {
        m_publisher_left.publish(m_output_msg);
    }
    else
    {
        ROS_WARN_THROTTLE(5, "Received unknown frame_id: [%s]", msg.header.frame_id.c_str());
    }
}

} // namespace pet

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ultrasound_adapter");

    ROS_INFO("Initialising node...");
    pet::UltrasoundAdapter node{};
    ROS_INFO("Node initialisation done.");

    ros::spin();
}