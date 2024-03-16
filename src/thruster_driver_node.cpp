#include "rclcpp/rclcpp.hpp"
#include "world_of_stonefish/thruster_driver.hpp"



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<ThrusterDriver> node = std::make_shared<ThrusterDriver>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
