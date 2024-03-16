#include "rclcpp/rclcpp.hpp"
#include "world_of_stonefish/actuator_driver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<ActuatorDriver> node = std::make_shared<ActuatorDriver>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
