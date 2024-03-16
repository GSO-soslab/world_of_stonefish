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


class ActuatorDriver : public rclcpp::Node
{
    public:
        ActuatorDriver(std::string name = "stonefish_actuator_driver");
    
    private:
        //thrusters
        std::vector<std::string> thruster_topics;
        int m_thruster_len;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub;
        
        struct thruster_t
        {
            int index;
            std::string topic_name;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
            std_msgs::msg::Float64 data;
        };

        std_msgs::msg::Float64MultiArray thruster_out;

        std::vector<thruster_t> thruster_vector;

        void f_thruster_callback(const std_msgs::msg::Float64::SharedPtr msg);        

};




#endif