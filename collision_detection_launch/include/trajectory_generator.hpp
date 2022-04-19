#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_srvs/srv/empty.hpp"
#include <collision_detection_msgs/srv/generate_trajectory.hpp>
#include <functional>
#include <moveit/move_group_interface/move_group_interface.h>
class TrajectoryGenerator : public rclcpp::Node{
    public:
    TrajectoryGenerator();
    void generate_trajectory(const std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Request> request,
        std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Response>      response);
    void setupMoveServiceUR(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response>      response){
      setupMoveGroup("ur_manipulator");
    }
    // trajectory_msgs::JointTrajectory generateTrajectory(const JointState& start, const JointState& end);
  private:
    moveit::planning_interface::MoveGroupInterface setupMoveGroup(std::string);
    rclcpp::Service<collision_detection_msgs::srv::GenerateTrajectory>::SharedPtr service_;
     rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service1_;


};