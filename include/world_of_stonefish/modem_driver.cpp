
#include "modem_driver.hpp"
#include <chrono>
#include <functional>
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;


ModemDriver::ModemDriver(std::string name) : Node(name)
{
    this->declare_parameter("modem_received_topic", "");
    this->get_parameter("modem_received_topic", m_received);

    this->declare_parameter("modem_send_topic", "");
    this->get_parameter("modem_send_topic", m_send);

    this->declare_parameter("modem_received_bytearray_topic", "");
    this->get_parameter("modem_received_bytearray_topic", m_received_bytearray);

    this->declare_parameter("modem_send_bytearray_topic", "");
    this->get_parameter("modem_send_bytearray_topic", m_send_bytearray);


    //topic with namespace
    modem_received_sub = this->create_subscription<std_msgs::msg::String>(
        m_received, 10, std::bind(&ModemDriver::f_received_callback, this, _1));

    modem_send_pub = this->create_publisher<std_msgs::msg::String>(m_send, 10);

    modem_received_bytearray_sub = this->create_subscription<std_msgs::msg::ByteMultiArray>(
        m_received_bytearray, 10, std::bind(&ModemDriver::f_send_callback, this, _1));
    
    modem_send_bytearray_pub = this->create_publisher<std_msgs::msg::ByteMultiArray>(m_send_bytearray, 10);
}

ModemDriver::~ModemDriver() 
{
    RCLCPP_INFO(this->get_logger(), "Shutting down ModemDriver node...");
}

void ModemDriver::shutdown_node() 
{
    // Add any custom cleanup logic here
    modem_received_sub.reset();
    modem_send_pub.reset();
    modem_received_bytearray_sub.reset();
    modem_send_bytearray_pub.reset();
    RCLCPP_INFO(this->get_logger(), "ModemDriver node shutdown complete.");
}


void ModemDriver::f_send_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received ByteMultiArray message with data size: %zu", msg->data.size());

    //Convert ByteMultiArray to String
    std_msgs::msg::String string_msg;
    string_msg.data = std::string(msg->data.begin(), msg->data.end());
    modem_send_pub->publish(string_msg);
}

void ModemDriver::f_received_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received String message: %s", msg->data.c_str());

    // Convert String to ByteMultiArray
    std_msgs::msg::ByteMultiArray bytearray_msg;
    bytearray_msg.data.assign(msg->data.begin(), msg->data.end());
    modem_send_bytearray_pub->publish(bytearray_msg);
}