#include "pressure_sensor_driver.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

PressureSensorDriver::PressureSensorDriver(std::string name) : Node(name)
{

    //parameter
    this->declare_parameter("fluid_density", 1023.0);
    this->declare_parameter("frame_id", "world");
    this->declare_parameter("child_frame_id", "pressure");
    //link parameter and variables.
    this->get_parameter("fluid_density",m_fluid_density);
    this->get_parameter("frame_id",m_frame_id);
    this->get_parameter("child_frame_id",m_child_frame_id);
    //topic with namespace
    // _publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("depth", 10);
    _publisher = this->create_publisher<nav_msgs::msg::Odometry>("depth", 10);
    //topci without namespace
    _subscriber = this->create_subscription<sensor_msgs::msg::FluidPressure>("pressure", 10, 
                                                                std::bind(&PressureSensorDriver::f_pressure_callback, 
                                                                this, _1));


}

void PressureSensorDriver::f_pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
{
    nav_msgs::msg::Odometry depth;
    depth.header = msg->header;
    depth.header.frame_id = m_frame_id;
    depth.child_frame_id = m_child_frame_id;
    depth.pose.pose.position.z = -msg->fluid_pressure / (m_fluid_density*9.81);
    
    _publisher->publish(depth);

}