
#include "rclcpp/rclcpp.hpp"

#include "world_of_stonefish/pressure_sensor_driver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<PressureSensorDriver> node = std::make_shared<PressureSensorDriver>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
