#ifndef MODEM_DRIVER_HPP_
#define MODEM_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

class ModemDriver : public rclcpp::Node
{
    public:
        ModemDriver(std::string name = "stonefish_modem_driver");
        ~ModemDriver();
        void shutdown_node(); // Shutdown function
    private:
        std::string m_received;
        std::string m_send;
        std::string m_received_bytearray;
        std::string m_send_bytearray;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr modem_received_sub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr modem_send_pub;
        
        rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr modem_received_bytearray_pub;
        rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr modem_send_bytearray_sub;

        void f_send_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg);
        void f_received_callback(const std_msgs::msg::String::SharedPtr msg);
        
};


#endif