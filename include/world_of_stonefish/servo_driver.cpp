#include "servo_driver.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include "iostream"
#include "cstdio"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

ServoDriver::ServoDriver(std::string name) : Node(name)
{
    this->declare_parameter("servo_length", 0);
    this->get_parameter("servo_length", m_servo_len);

    this->declare_parameter("servo_joint_names", m_servo_joints);
    this->get_parameter("servo_joint_names", m_servo_joints);

    this->declare_parameter("servo_sub_topics", m_servo_sub_topics);
    this->get_parameter("servo_sub_topics", m_servo_sub_topics);

    this->declare_parameter("servo_pub_topic", m_servo_pub_topic);
    this->get_parameter("servo_pub_topic", m_servo_pub_topic);

    // create publisher
    servo_pub = this->create_publisher<sensor_msgs::msg::JointState>(m_servo_pub_topic, 10);

    // create subscribers
    for (int i =0; i< m_servo_len; i++)
    {
        servo_t t;
        t.index = i;
        t.joint_name = m_servo_joints[i];
        t.topic_name = m_servo_sub_topics[i];
        t.sub_ = this->create_subscription<std_msgs::msg::Float64>(t.topic_name, 
                                                                   10, 
                                                                   [this, i](const std_msgs::msg::Float64::SharedPtr msg){
                                                                    this->f_servo_callback(msg, i);
                                                                    }
                                                                    );

        servo_vector.push_back(t);
    }

    m_servo_out.name.resize(m_servo_len);
    m_servo_out.position.resize(m_servo_len);
}


 void ServoDriver::f_servo_callback(const std_msgs::msg::Float64::SharedPtr msg, int i)
 {
    m_servo_out.name[i] = servo_vector[i].joint_name;;
    m_servo_out.position[i] = msg->data;
    servo_pub->publish(m_servo_out);
 }
