#ifndef _TIMER_H
#define _TIMER_H

#include <stdint.h>

#include "ros.h"
#include <ros/time.h>

#include "ros_time_operators.h"


using func_ptr = void (*)();

struct Callback {
    Callback() = default;
    Callback(func_ptr func, ros::Duration interval, ros::Time start_time)
            : func(func) , interval(interval) , next_time(start_time) {}

    func_ptr func;
    ros::Duration interval;
    ros::Time next_time;
};

template<uint8_t capacity>
class Timer {
public:
    Timer(pet::ros::NodeHandle& nh) : m_nh(nh) {}

    void register_callback(func_ptr func, ros::Duration interval);
    void spin_once();    

private:
    pet::ros::NodeHandle& m_nh;
    uint8_t m_size = 0;
    Callback m_callbacks[capacity];
};


template<uint8_t capacity>
void Timer<capacity>::register_callback(func_ptr func, ros::Duration interval) 
{
    if (m_size < capacity) {
        m_callbacks[m_size] = Callback(func, interval, m_nh.now());
        ++m_size;
    }
    else {
        // TODO: Raise exception? Error handling? Ignore?
    }
}

template<uint8_t capacity>
inline void Timer<capacity>::spin_once() 
{
    ros::Time current_time = m_nh.now();
    for (uint8_t i = 0; i < m_size; ++i) {
        Callback& cb = m_callbacks[i];
        if (cb.next_time < current_time) {
            cb.func();
            cb.next_time = current_time + cb.interval;
            break;
        }
    }
}

#endif // _TIMER_H
