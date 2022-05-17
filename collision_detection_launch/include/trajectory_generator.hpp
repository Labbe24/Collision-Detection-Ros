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
    // trajectory_msgs::JointTrajectory generateTrajectory(const JointState& start, const JointState& end);
  private:
    moveit::planning_interface::MoveGroupInterfacePtr setupMoveGroup(std::string);
    trajectory_msgs::msg::JointTrajectory generateTrajectory(sensor_msgs::msg::JointState a,sensor_msgs::msg::JointState b,moveit::planning_interface::MoveGroupInterfacePtr move_group);
    rclcpp::Service<collision_detection_msgs::srv::GenerateTrajectory>::SharedPtr service_;
    rclcpp::Service<collision_detection_msgs::srv::GenerateTrajectory>::SharedPtr service1_;
    //Adds b to a.
    void addTrajectories(trajectory_msgs::msg::JointTrajectory& a,trajectory_msgs::msg::JointTrajectory& b);

};