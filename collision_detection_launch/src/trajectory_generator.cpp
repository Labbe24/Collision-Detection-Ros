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
  return moveit::planning_interface::MoveGroupInterface(shared_from_this(), planning_group);
}

void TrajectoryGenerator::generate_trajectory(const std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Request> request,
        std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Response>      response){
        
        auto move_group = setupMoveGroup("ur_manipulator");
        //setStartState
        
        moveit_msgs::msg::RobotState start_state;
        start_state.set__joint_state(request->start_state);
        move_group.setStartState(start_state);

        //setGoalState
        move_group.setJointValueTarget(request->end_state);

            moveit::planning_interface::MoveGroupInterface::Plan robert_plant;
        //plan
        move_group.plan(robert_plant);
        response->set__res(robert_plant.trajectory_.joint_trajectory);
}
