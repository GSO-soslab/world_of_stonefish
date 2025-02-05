/*
This node is used to remap MVP actuator topics into stonefish compatible topics.
*/

#ifndef SERVO_DRIVER_HPP_
#define SERVO_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <functional>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>


class RudderDriver : public rclcpp::Node
{
    public:
        RudderDriver(std::string name = "stonefish_rudder_driver");
    
    private:
        //servo sub topics
        std::vector<std::string> m_rudder_sub_topics;

        // total number of thrusters
        int m_rudder_len;

        //thruster structure
        struct rudder_t
        {
            int index;
            std::string topic_name;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;

        };
        
        // servo vector
        std::vector<rudder_t>  rudder_vector;

        void f_rudder_callback(const std_msgs::msg::Float64::SharedPtr msg, int i);    
        
        //servo command array
       std_msgs::msg::Float64MultiArray m_rudder_out;

        std::string m_rudder_pub_topic;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rudder_pub;
};

#endif