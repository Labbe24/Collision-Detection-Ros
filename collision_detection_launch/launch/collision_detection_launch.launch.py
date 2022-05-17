from email.policy import default
import imp

from click import argument, launch
import controller_manager
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution,Command,FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import launch

def generate_launch_description():



    # ur setup:
    ur_type='ur5e'
    robot_ip = '127.0.0.1'
    use_fake_hardware='true'
    arguments_for_moveit={'ur_type':ur_type, 'robot_ip':robot_ip, 'launch_rviz':'false','use_fake_hardware':use_fake_hardware}
    collision_detection_launch_dir = get_package_share_directory("collision_detection_launch")

    trajectory_generator = IncludeLaunchDescription(
                launch_description_source= PythonLaunchDescriptionSource(collision_detection_launch_dir+'/launch/trajectory_generator_ur.launch.py'),
                launch_arguments=arguments_for_moveit.items()
            )


    # shared:
    unity_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='my_tcp_endpoint',
        parameters=[{'ROS_IP':'127.0.0.1'}]
    )

    return launch.LaunchDescription([
        trajectory_generator,
        unity_tcp_endpoint
    ])