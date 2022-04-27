#include "trajectory_generator.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
TrajectoryGenerator::TrajectoryGenerator()
    : Node("trajectory_generator")
    {
        RCLCPP_INFO(rclcpp::get_logger("trajectory_generator"), "Creating trajectory generator!");
        service_ =
        this->create_service<collision_detection_msgs::srv::GenerateTrajectory>("generate_trajectory_srv", std::bind(&TrajectoryGenerator::generate_trajectory,this,std::placeholders::_1,std::placeholders::_2));
        service1_ = this->create_service<std_srvs::srv::Empty>("setup_move_group_srv", std::bind(&TrajectoryGenerator::setupMoveServiceUR,this,std::placeholders::_1,std::placeholders::_2));

    }


// void TrajectoryGenerator::generate_trajectory(const std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Request> request,
//     std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Response>      response)
// {
//     response->res.joint_names.push_back("Magnus");
//     response->res.joint_names.push_back("Er");
//     response->res.joint_names.push_back("En");
//     response->res.joint_names.push_back("Bot");
//     std::string str = "Incoming request\nfront: "+
//         request->start_state.name.front()+ "\nend: "+request->end_state.name.front();
//     RCLCPP_INFO(rclcpp::get_logger("trajectory_generator"), str.c_str());
// }
moveit::planning_interface::MoveGroupInterface TrajectoryGenerator::setupMoveGroup(std::string planning_group)
{
    RCLCPP_INFO(rclcpp::get_logger("trajectory_generator"), "trying!");
  auto ret = moveit::planning_interface::MoveGroupInterface(shared_from_this(), planning_group);
  RCLCPP_INFO(rclcpp::get_logger("trajectory_generator"), "Success!");
  return ret;
}

// void printTrajectory(const trajectory_msgs::msg::JointTrajectory& traj){
//     RCLCPP_INFO(rclcpp::get_logger("trajectory_generator"), "Printing trajectory names:");
//     auto names = std::accumulate(traj.joint_names.begin(),traj.joint_names.end(),"",[](std::string acc, std::string name ){
//         return acc+"-"+name;
//     });
//     RCLCPP_INFO(rclcpp::get_logger("trajectory_generator"), names.c_str());
// }

void TrajectoryGenerator::generate_trajectory(const std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Request> request,
        std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Response>      response){
        auto move_group = setupMoveGroup(request->move_group);
        //setStartState
        moveit_msgs::msg::RobotState start_state;
        start_state.set__joint_state(request->start_state);
        move_group.setStartState(start_state);

        //setGoalState
        move_group.setJointValueTarget(request->end_state);
        moveit::planning_interface::MoveGroupInterface::Plan robert_plant;
        //plan
        move_group.plan(robert_plant);
        // printTrajectory(robert_plant.trajectory_.joint_trajectory);
        RCLCPP_INFO_STREAM(get_logger(), "joint trajectory:" << rosidl_generator_traits::to_yaml(robert_plant.trajectory_.joint_trajectory));
        response->set__res(robert_plant.trajectory_.joint_trajectory);
}
