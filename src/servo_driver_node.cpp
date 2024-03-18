#include "rclcpp/rclcpp.hpp"
#include "world_of_stonefish/servo_driver.hpp"



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<ServoDriver> node = std::make_shared<ServoDriver>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
