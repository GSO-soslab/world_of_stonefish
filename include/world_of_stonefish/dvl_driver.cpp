
#include "dvl_driver.hpp"
#include <chrono>
#include <functional>
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;


DVLDriver::DVLDriver(std::string name) : Node(name)
{
    this->declare_parameter("dvl_in", "");
    this->get_parameter("dvl_in", m_dvl_in);

    this->declare_parameter("dvl_out", "");
    this->get_parameter("dvl_out", m_dvl_out);

    //topic with namespace
    dvl_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(m_dvl_out, 10);
    //topci without namespace
    dvl_sub = this->create_subscription<stonefish_ros2::msg::DVL>(m_dvl_in, 10, 
                                                                std::bind(&DVLDriver::f_dvl_callback, 
                                                                this, _1));
}

void DVLDriver::f_dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg)
{
    geometry_msgs::msg::TwistWithCovarianceStamped pub_msg;

    pub_msg.header.stamp = msg->header.stamp;
    pub_msg.header.frame_id = msg->header.frame_id;
    pub_msg.twist.twist.linear.x = msg->velocity.x;
    pub_msg.twist.twist.linear.y = msg->velocity.y;
    pub_msg.twist.twist.linear.z = msg->velocity.z;
    pub_msg.twist.covariance[0] = msg->velocity_covariance[0];
    pub_msg.twist.covariance[4] = msg->velocity_covariance[4];
    pub_msg.twist.covariance[8] = msg->velocity_covariance[8];
    dvl_pub->publish(pub_msg);
}