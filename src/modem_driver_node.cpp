
#include "rclcpp/rclcpp.hpp"

#include "world_of_stonefish/modem_driver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<ModemDriver> node = std::make_shared<ModemDriver>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
