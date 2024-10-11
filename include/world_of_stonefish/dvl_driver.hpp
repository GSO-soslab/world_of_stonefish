#ifndef DVL_DRIVER_HPP_
#define DVL_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "stonefish_ros2/msg/dvl.hpp"

class DVLDriver : public rclcpp::Node
{
    public:
        DVLDriver(std::string name = "stonefish_dvl_driver");
        ~DVLDriver();
        void shutdown_node(); // Shutdown function
    private:
        std::string m_dvl_in;
        std::string m_dvl_out;
        std::string m_dvl_altitude_out;

        rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr dvl_sub;
        
        rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_pub;

        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr dvl_altitude_pub;

        void f_dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg);
        
};


#endif