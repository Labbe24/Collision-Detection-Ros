// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_state/robot_state.h>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_generator.hpp"
int main(int argc, char** argv)
{
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use

  //should be multi-threaded?
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGenerator>());
  rclcpp::shutdown();
  return 0;
}