#include "pressure_sensor_driver.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

PressureSensorDriver::PressureSensorDriver(std::string name) : Node(name)
{

    //parameter
    this->declare_parameter("fluid_density", 1023.0);
    this->declare_parameter("frame_id", "world");
    //link parameter and variables.
    this->get_parameter("fluid_density",m_fluid_density);
    this->get_parameter("frame_id",m_frame_id);

    //topic with namespace
    _publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("depth", 10);
    //topci without namespace
    _subscriber = this->create_subscription<sensor_msgs::msg::FluidPressure>("pressure", 10, 
                                                                std::bind(&PressureSensorDriver::f_pressure_callback, 
                                                                this, _1));


}

void PressureSensorDriver::f_pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
{
    geometry_msgs::msg::PoseWithCovarianceStamped depth;
    depth.header = msg->header;
    depth.header.frame_id = m_frame_id;
    depth.pose.pose.position.z = msg->fluid_pressure / (m_fluid_density*9.81);
    
    _publisher->publish(depth);

}