#include "trajectory_generator.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
using JointState = sensor_msgs::msg::JointState;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
TrajectoryGenerator::TrajectoryGenerator()
    : Node("trajectory_generator")
    {
        RCLCPP_INFO(rclcpp::get_logger("trajectory_generator"), "Creating trajectory generator!");
        service_ =
        this->create_service<collision_detection_msgs::srv::GenerateTrajectory>("generate_trajectory_srv", std::bind(&TrajectoryGenerator::generate_trajectory,this,std::placeholders::_1,std::placeholders::_2));
        service1_ =
        this->create_service<collision_detection_msgs::srv::GenerateTrajectory>("generate_trajectory_srv1", std::bind(&TrajectoryGenerator::generate_trajectory,this,std::placeholders::_1,std::placeholders::_2));

    }

moveit::planning_interface::MoveGroupInterfacePtr TrajectoryGenerator::setupMoveGroup(std::string planning_group)
{
    RCLCPP_INFO(rclcpp::get_logger("trajectory_generator"), "trying!");
  auto ret = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group);
  RCLCPP_INFO(rclcpp::get_logger("trajectory_generator"), "Success!");
  return ret;
}


void TrajectoryGenerator::generate_trajectory(const std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Request> request,
        std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Response>      response){
        auto move_group = setupMoveGroup(request->move_group);
        JointTrajectory res;
        for(int i=0;i<(request->states.size()-1);i++){
            auto traj = generateTrajectory(request->states[i],request->states[i+1],move_group);
            addTrajectories(res,traj);
        }
        // printTrajectory(robert_plant.trajectory_.joint_trajectory);
        RCLCPP_INFO_STREAM(get_logger(), "joint trajectory:" << rosidl_generator_traits::to_yaml(res));
        response->set__res(res);
}

JointTrajectory TrajectoryGenerator::generateTrajectory(JointState a,JointState b,moveit::planning_interface::MoveGroupInterfacePtr move_group){
    moveit_msgs::msg::RobotState start_state;
        start_state.set__joint_state(a);
        move_group->setStartState(start_state);
        move_group->setJointValueTarget(b);
        moveit::planning_interface::MoveGroupInterface::Plan robert_plant;
        //plan
        move_group->plan(robert_plant);
        return robert_plant.trajectory_.joint_trajectory;
}

void TrajectoryGenerator::addTrajectories(JointTrajectory& a,JointTrajectory& b){
    if(a.points.size()==0){
        a = b;
        return;
    }
    //names must be the same

    //alter timestamps in b
    auto delta_t = a.points.back().time_from_start;
    for(int i=0;i<b.points.size();i++){
        builtin_interfaces::msg::Duration new_t;
        new_t.sec = b.points[i].time_from_start.sec+delta_t.sec;
        new_t.nanosec = b.points[i].time_from_start.nanosec+delta_t.nanosec;
        b.points[i].set__time_from_start(new_t);
    }
    a.points.insert( a.points.end(), b.points.begin(), b.points.end() );
}