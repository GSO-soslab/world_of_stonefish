/*
This node is used to remap MVP actuator topics into stonefish compatible topics.
*/

#ifndef USBL_DRIVER_HPP_
#define USBL_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <functional>

#include "mvp_acomm_interfaces/msg/usbl_data.hpp"
#include "stonefish_ros2/msg/beacon_info.hpp"

class USBLDriver : public rclcpp::Node
{
    public:
        USBLDriver(std::string name = "stonefish_usbl_driver");
    
    private:
        rclcpp::Publisher<mvp_acomm_interfaces::msg::UsblData>::SharedPtr _publisher;
        rclcpp::Subscription<stonefish_ros2::msg::BeaconInfo>::SharedPtr _subscriber;
        void f_sf_usbl(const stonefish_ros2::msg::BeaconInfo::SharedPtr msg);
        double m_sound_speed;
        mvp_acomm_interfaces::msg::UsblData m_usbl_data;

};




#endif