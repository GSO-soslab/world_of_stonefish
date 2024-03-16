#include "world_of_stonefish/actuator_driver.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include "iostream"
#include "cstdio"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

ActuatorDriver::ActuatorDriver(std::string name) : Node(name)
{
    this->declare_parameter("thruster_length", 0);
    this->get_parameter("thruster_length", m_thruster_len);

    printf("thruster number=%d\r\n", m_thruster_len);


    this->declare_parameter("thruster_topics", thruster_topics);
    this->get_parameter("thruster_topics", thruster_topics);

    for (int i =0; i< m_thruster_len; i++)
    {
        thruster_t t;
        t.index = i;
        t.topic_name = thruster_topics[i];
        t.sub_ = this->create_subscription<std_msgs::msg::Float64>(t.topic_name, 10, 
                                                                std::bind(&ActuatorDriver::f_thruster_callback, 
                                                                this, _1));
        thruster_vector.push_back(t);
    }
}

 void ActuatorDriver::f_thruster_callback(const std_msgs::msg::Float64::SharedPtr msg)
 {
    

 }