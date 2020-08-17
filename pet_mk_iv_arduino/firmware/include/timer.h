#ifndef _PET_TIMER_H
#define _PET_TIMER_H

#include <stdint.h>

#include <ros/time.h>
#include <ros/duration.h>

#include "rosserial_node.h"
#include "arduino_module.h"

namespace pet
{

template<uint8_t capacity>
class Timer
{
private:
    struct Callback
    {
        ArduinoModule* module_ = nullptr;
        ros::Duration period_{};
        ros::Time next_time_{};

        void call()
        {
            module_->callback();
        }
    };

public:
    bool register_module(ArduinoModule* module)
    {
        if (m_size < capacity)
        {
            m_callbacks[m_size].module_ = module;
            m_callbacks[m_size].period_ = ros::Duration{1/module->frequency()};
            ++m_size;
            return true;
        }
        else
        {
            return false;
        }
    }

    // Sets start time on registered modules.
    void start()
    {
        const ros::Time current_time = nh.now();
        for (int i = 0; i < m_size; ++i)
        {
            auto& callback = m_callbacks[i];
            callback.next_time_ = current_time + callback.period_;
        }
    }

    void spin_once()
    {
        const ros::Time current_time = nh.now();
        for (int i = 0; i < m_size; ++i)
        {
            auto& callback = m_callbacks[i];
            if (callback.next_time_ < current_time)
            {
                callback.call();
                callback.next_time_ += callback.period_;
            }
        }
    }

private:
    uint8_t m_size = 0;
    Callback m_callbacks[capacity];
};

} // namespace pet

#endif // _PET_TIMER_H
