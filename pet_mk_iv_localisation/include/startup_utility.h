#ifndef PET_STARTUP_UTILITY_H
#define PET_STARTUP_UTILITY_H

#include <string>

#include <ros/ros.h>

namespace pet::utility
{

template<typename MsgType>
void wait_for_message(const ros::Subscriber& subscriber,
                      ros::Duration interval=ros::Duration(5.0),
                      ros::Duration initial_duration=ros::Duration(2.0)
                      )
{
    const std::string msg_type_name = ros::message_traits::DataType<MsgType>::value();
    const std::string topic_name = subscriber.getTopic();

    auto msg_ptr = ros::topic::waitForMessage<MsgType>(topic_name, initial_duration);
    if (msg_ptr == nullptr)
    {
        ROS_WARN("No message recieved on topic [%s] with type [%s]!", topic_name.c_str(), msg_type_name.c_str());
    }
    else
    {
        return;
    }

    while (ros::ok())
    {
        auto msg_ptr = ros::topic::waitForMessage<MsgType>(topic_name, interval);
        if (msg_ptr == nullptr)
        {
            ROS_INFO("Still waiting for message on topic [%s] with type [%s].", topic_name.c_str(), msg_type_name.c_str());
        }
        else
        {
            ROS_INFO("Finally recieved a message on topic [%s] with type [%s]!", topic_name.c_str(), msg_type_name.c_str());
            break;
        }
    }

    return;
}

}

#endif // PET_STARTUP_UTILITY_H