#include "rudder_driver.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include "iostream"
#include "cstdio"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

RudderDriver::RudderDriver(std::string name) : Node(name)
{
    this->declare_parameter("rudder_length", 0);
    this->get_parameter("rudder_length", m_rudder_len);


    this->declare_parameter("rudder_sub_topics", m_rudder_sub_topics);
    this->get_parameter("rudder_sub_topics", m_rudder_sub_topics);

    this->declare_parameter("rudder_pub_topic", m_rudder_pub_topic);
    this->get_parameter("rudder_pub_topic", m_rudder_pub_topic);

    // create publisher
    rudder_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(m_rudder_pub_topic, 10);

    m_rudder_out.data.resize(m_rudder_len);
    // create subscribers
    for (int i =0; i< m_rudder_len; i++)
    {
        rudder_t t;
        t.index = i;
        t.topic_name = m_rudder_sub_topics[i];
        t.sub_ = this->create_subscription<std_msgs::msg::Float64>(t.topic_name, 
                                                                   10, 
                                                                   [this, i](const std_msgs::msg::Float64::SharedPtr msg){
                                                                    this->f_rudder_callback(msg, i);
                                                                    }
                                                                    );

        rudder_vector.push_back(t);
    }

    
}


 void RudderDriver::f_rudder_callback(const std_msgs::msg::Float64::SharedPtr msg, int i)
 {
    m_rudder_out.data[i] = msg->data;
    rudder_pub->publish(m_rudder_out);
 }
