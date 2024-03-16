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

    this->declare_parameter("thruster_sub_topics", m_thruster_sub_topics);
    this->get_parameter("thruster_sub_topics", m_thruster_sub_topics);

    this->declare_parameter("thruster_pub_topic", m_thruster_pub_topic);
    this->get_parameter("thruster_pub_topic", m_thruster_pub_topic);


    // create subscribers
    for (int i =0; i< m_thruster_len; i++)
    {
        thruster_t t;
        t.index = i;
        t.topic_name = m_thruster_sub_topics[i];
        t.sub_ = this->create_subscription<std_msgs::msg::Float64>(t.topic_name, 
                                                                   10, 
                                                                   [this, i](const std_msgs::msg::Float64::SharedPtr msg){
                                                                    this->f_thruster_callback(msg, i);
                                                                    }
                                                                    );

        thruster_vector.push_back(t);
    }
    
    // create publisher
    thruster_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(m_thruster_pub_topic, 10);
    // timer_ = this->create_wall_timer(500ms, std::bind(&ActuatorDriver::f_publish_thruster, this));
    m_thruster_out.data.resize(m_thruster_len);
}

//callback function. the thruster command (array) will publish when the last thruster command was received.
// such a publishing rate is determined based on the controller.
 void ActuatorDriver::f_thruster_callback(const std_msgs::msg::Float64::SharedPtr msg, int i)
 {
    m_thruster_out.data[i] = msg->data;

    if(i == m_thruster_len-1)
    {
        thruster_pub->publish(m_thruster_out);
    }

 }


