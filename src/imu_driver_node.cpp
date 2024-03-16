
#include "rclcpp/rclcpp.hpp"

#include "world_of_stonefish/imu_driver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<IMUDriver> node = std::make_shared<IMUDriver>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
