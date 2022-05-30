#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_srvs/srv/empty.hpp"
#include <collision_detection_msgs/srv/generate_trajectory.hpp>
#include <functional>
#include <moveit/move_group_interface/move_group_interface.h>
class TrajectoryGenerator : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Trajectory Generator object.
     * Sets up the services provided by this node.
     * "generate_trajectory_srv" and "generate_trajectory_srv1"
     *
     */
    TrajectoryGenerator();
    /**
     * @brief Callback for generate_trajectory_srv.
     * Calls generate_and_add_trajectories()
     *
     * @param request contains name of move group to plan for, along with a list of joint positions the trajectory should traverse.
     * @param response contains the generated trajectory. Which is returned from the service request.
     */
    void generate_trajectory_service(const std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Request> request,
                                     std::shared_ptr<collision_detection_msgs::srv::GenerateTrajectory::Response> response);
    // trajectory_msgs::JointTrajectory generateTrajectory(const JointState& start, const JointState& end);
private:
    /**
     * @brief Wrapper for creating MoveGroupInterface
     *
     * @param group name of move-group to plan for.
     * @return moveit::planning_interface::MoveGroupInterfacePtr to the MoveGroupInterface created for the specific move group.
     */
    moveit::planning_interface::MoveGroupInterfacePtr setupMoveGroup(std::string group);
    /**
     * @brief Generate a trajectory between two joint states.
     *
     * @param a initial position
     * @param b desired position
     * @param move_group move_group to plan for.
     * @return trajectory_msgs::msg::JointTrajectory from a to b.
     */
    trajectory_msgs::msg::JointTrajectory generateTrajectory(sensor_msgs::msg::JointState a, sensor_msgs::msg::JointState b, moveit::planning_interface::MoveGroupInterfacePtr move_group);
    /**
     * @brief Append trajectory b to trajectory a.
     *
     * @param a Trajectory to append to.
     * @param b Trajectory to append.
     */
    void addTrajectories(trajectory_msgs::msg::JointTrajectory &a, trajectory_msgs::msg::JointTrajectory &b);
    /**
     * @brief Creates a trajectory traversing all the joint positions in states.
     * Uses generateTrajectory() and addTrajectories()
     * @param states points the trajectory should pass through
     * @param move_group moveit::planning_interface::MoveGroupInterfacePtr to generate trajectories with.
     * @return trajectory_msgs::msg::JointTrajectory trajectory traversing all the positions in states.
     */
    trajectory_msgs::msg::JointTrajectory generate_and_add_trajectories(std::vector<sensor_msgs::msg::JointState> states, moveit::planning_interface::MoveGroupInterfacePtr move_group);
    rclcpp::Service<collision_detection_msgs::srv::GenerateTrajectory>::SharedPtr service_;
    rclcpp::Service<collision_detection_msgs::srv::GenerateTrajectory>::SharedPtr service1_;
};