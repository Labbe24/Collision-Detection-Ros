#include "rclcpp/rclcpp.hpp"
#include "trajectory_generator.hpp"
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGenerator>());
  rclcpp::shutdown();
  return 0;
}