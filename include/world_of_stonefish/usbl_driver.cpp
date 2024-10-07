#include "usbl_driver.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

USBLDriver::USBLDriver(std::string name) : Node(name)
{

    //parameter
    this->declare_parameter("sound_speed", 1500.0);
    this->get_parameter("sound_speed",m_sound_speed);
    //topic with namespace
    // _publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("depth", 10);
    _publisher = this->create_publisher<mvp_acomm_interfaces::msg::UsblData>("usbl_data", 10);
    //topci without namespace
    _subscriber = this->create_subscription<stonefish_ros2::msg::BeaconInfo>("stonefish/usbl/beacon_info", 10, 
                                                                std::bind(&USBLDriver::f_sf_usbl, 
                                                                this, _1));


}

void USBLDriver::f_sf_usbl(const stonefish_ros2::msg::BeaconInfo::SharedPtr msg)
{
    printf("usbl_data got");
    m_usbl_data.header = msg->header;
    m_usbl_data.remote_address = msg->beacon_id;
    m_usbl_data.xyz.x = msg->relative_position.x;
    m_usbl_data.xyz.y = msg->relative_position.y;
    m_usbl_data.xyz.z = msg->relative_position.z;
    m_usbl_data.orientation =msg->local_orientation;
    _publisher->publish(m_usbl_data);

}