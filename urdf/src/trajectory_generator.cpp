#include "trajectory_generator.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
TrajectoryGenerator::TrajectoryGenerator()
    : Node("trajectory_generator")
    {
        RCLCPP_INFO(rclcpp::get_logger("trajectory_generator"), "Creating trajectory generator!");
        service_ =
        this->create_service<collision_detection_msgs::srv::GenerateTrajectory>("generate_trajectory_srv", std::bind(&TrajectoryGenerator::generate_trajectory,this,std::placeholders::_1,std::placeholders::_2));
        service1_ = this->create_service<std_srvs::srv::Empty>("setup_move_group_srv", std::bind(&TrajectoryGenerator::setupMoveServiceUR,this,std::placeholders::_1,std::placeholders::_2));

    }


void TrajectoryGenerator::generate_trajectory(const std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Request> request,
    std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Response>      response)
{
    response->res.joint_names.push_back("Magnus");
    response->res.joint_names.push_back("Er");
    response->res.joint_names.push_back("En");
    response->res.joint_names.push_back("Bot");
    std::string str = "Incoming request\nfront: "+
        request->start_state.name.front()+ "\nend: "+request->end_state.name.front();
    RCLCPP_INFO(rclcpp::get_logger("trajectory_generator"), str.c_str());
}
void TrajectoryGenerator::setupMoveGroup(std::string planning_group)
{
    // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), planning_group);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(planning_group);

}


// trajectory_msgs::JointTrajectory TrajectoryGenerator::generateTrajectory(const JointState& start, const JointState& end){
//     trajectory_msgs::JointTrajectory traj();
//     return traj;
// }
