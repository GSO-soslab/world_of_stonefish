#include "rclcpp/rclcpp.hpp"
#include "world_of_stonefish/rudder_driver.hpp"



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<RudderDriver> node = std::make_shared<RudderDriver>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
