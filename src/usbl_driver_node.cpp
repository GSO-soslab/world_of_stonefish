
#include "rclcpp/rclcpp.hpp"

#include "world_of_stonefish/usbl_driver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<USBLDriver> node = std::make_shared<USBLDriver>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
