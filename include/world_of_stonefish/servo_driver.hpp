/*
This node is used to remap MVP actuator topics into stonefish compatible topics.
*/

#ifndef SERVO_DRIVER_HPP_
#define SERVO_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <functional>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

class ServoDriver : public rclcpp::Node
{
    public:
        ServoDriver(std::string name = "stonefish_servo_driver");
    
    private:
        //servo joints names
        std::vector<std::string> m_servo_joints;
        //servo sub topics
        std::vector<std::string> m_servo_sub_topics;

        // total number of thrusters
        int m_servo_len;

        //thruster structure
        struct servo_t
        {
            int index;
            std::string joint_name;
            std::string topic_name;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;

        };
        
        // servo vector
        std::vector<servo_t>  servo_vector;

        void f_servo_callback(const std_msgs::msg::Float64::SharedPtr msg, int i);    
        
        //servo command array
        sensor_msgs::msg::JointState m_servo_out;

        std::string m_servo_pub_topic;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr servo_pub;
};

#endif