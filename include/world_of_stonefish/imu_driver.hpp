/*
    This node converts pressure reading into pose with covariance stamped message for robot localization

*/

#ifndef IMU_DRIVER_HPP_
#define IMU_DRIVER_HPP_


#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"


class IMUDriver : public rclcpp::Node
{
    public:
        IMUDriver(std::string name = "stonefish_imu_driver");
    
    private:
        std::string m_frame_id;

        double roll_offset;
        double pitch_offset;
        double yaw_offset;
        
        double roll_reverse;
        double pitch_reverse;
        double yaw_reverse;

        void f_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);        
        
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_out;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_in;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

#endif