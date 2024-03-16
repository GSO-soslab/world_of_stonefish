/*
This node is used to remap MVP actuator topics into stonefish compatible topics.
*/

#ifndef ACTUATOR_DRIVER_HPP_
#define ACTUATOR_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <functional>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

class ActuatorDriver : public rclcpp::Node
{
    public:
        ActuatorDriver(std::string name = "stonefish_actuator_driver");
    
    private:
        //thrusters topics
        std::vector<std::string> m_thruster_sub_topics;
        
        // total number of thrusters
        int m_thruster_len;

        //thruster structure
        struct thruster_t
        {
            int index;
            std::string topic_name;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
        };
        
        // thruster vector
        std::vector<thruster_t> thruster_vector;

        //thruster command array
        std_msgs::msg::Float64MultiArray m_thruster_out;

        void f_thruster_callback(const std_msgs::msg::Float64::SharedPtr msg, int i);    
           

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub;
        std::string m_thruster_pub_topic;
};




#endif