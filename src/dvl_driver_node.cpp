
#include "rclcpp/rclcpp.hpp"

#include "world_of_stonefish/dvl_driver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<DVLDriver> node = std::make_shared<DVLDriver>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
