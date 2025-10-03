
#include "modem_driver.hpp"
#include <chrono>
#include <functional>
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;


std::string uint8VectorToHexString(const std::vector<uint8_t>& data) {
    std::stringstream ss;
    for (uint8_t byte : data) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    return ss.str();
}

// Function to convert a single hex character to its integer value
uint8_t hexCharToUint(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    } else if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    } else {
        throw std::invalid_argument("Invalid hex character");
    }
}

std::vector<uint8_t> hexStringToUint8Vector(const std::string& hexString) {
    if (hexString.length() % 2 != 0) {
        throw std::invalid_argument("Hex string must have an even length.");
    }

    std::vector<uint8_t> result;
    result.reserve(hexString.length() / 2); // Pre-allocate memory

    for (size_t i = 0; i < hexString.length(); i += 2) {
        uint8_t highNibble = hexCharToUint(hexString[i]);
        uint8_t lowNibble = hexCharToUint(hexString[i+1]);
        result.push_back((highNibble << 4) | lowNibble);
    }
    return result;
}


ModemDriver::ModemDriver(std::string name) : Node(name)
{
    this->declare_parameter("modem_received_topic", "stonefish/acomms/received_data");
    this->get_parameter("modem_received_topic", m_received);

    this->declare_parameter("modem_send_topic", "stonefish/acomms/data_to_send");
    this->get_parameter("modem_send_topic", m_send);

    this->declare_parameter("modem_received_bytearray_topic", "mvp_c2/acomms/rx");
    this->get_parameter("modem_received_bytearray_topic", m_received_bytearray);

    this->declare_parameter("modem_send_bytearray_topic", "mvp_c2/acomms/tx");
    this->get_parameter("modem_send_bytearray_topic", m_send_bytearray);


    //topic with namespace
    modem_received_sub = this->create_subscription<std_msgs::msg::String>(
        m_received, 10, std::bind(&ModemDriver::f_received_callback, this, _1));

    modem_send_pub = this->create_publisher<std_msgs::msg::String>(m_send, 10);

    modem_send_bytearray_sub = this->create_subscription<std_msgs::msg::ByteMultiArray>(
        m_send_bytearray, 10, std::bind(&ModemDriver::f_send_callback, this, _1));
    
    modem_received_bytearray_pub = this->create_publisher<std_msgs::msg::ByteMultiArray>(m_received_bytearray, 10);
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
    modem_received_bytearray_pub.reset();
    modem_send_bytearray_sub.reset();
    RCLCPP_INFO(this->get_logger(), "ModemDriver node shutdown complete.");
}


void ModemDriver::f_send_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received ByteMultiArray message with data size: %zu", msg->data.size());

    //Convert ByteMultiArray to String
    std_msgs::msg::String string_msg;
    string_msg.data = uint8VectorToHexString(msg->data);

    RCLCPP_INFO(this->get_logger(), "Sending String message with data size: %zu", string_msg.data.size());

    modem_send_pub->publish(string_msg);
}

void ModemDriver::f_received_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received String message with data size: %zu:", msg->data.size());

    // Convert String to ByteMultiArray
    std_msgs::msg::ByteMultiArray bytearray_msg;
    bytearray_msg.data = hexStringToUint8Vector(msg->data);

    RCLCPP_INFO(this->get_logger(), "Sending ByteMultiArray message with data size: %zu", bytearray_msg.data.size());
    modem_received_bytearray_pub->publish(bytearray_msg);
}