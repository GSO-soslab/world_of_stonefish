/*
    This node converts pressure reading into pose with covariance stamped message for robot localization

*/

#ifndef PRESSURE_SENSOR_DRIVER_HPP_
#define PRESSURE_SENSOR_DRIVER_HPP_


#include "rclcpp/rclcpp.hpp"
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"


class PressureSensorDriver : public rclcpp::Node
{
    public:
        PressureSensorDriver(std::string name = "pressure_sensor_node");
    
    private:
        // rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _publisher;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _publisher;
        rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr _subscriber;
        void f_pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
        std::string m_frame_id;
        std::string m_child_frame_id;
        float m_fluid_density;
};

#endif