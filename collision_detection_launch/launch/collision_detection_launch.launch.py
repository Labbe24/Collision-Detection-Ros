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
    controllers_file="controllers.yml"
    arguments_for_ur_launch={'ur_type':ur_type,
                            'robot_ip':robot_ip,
                            'launch_rviz':'false',
                            'use_fake_hardware':use_fake_hardware,
                            'initial_joint_controller':'joint_trajectory_controller',
                            'description_package':'collision_detection_launch',
                            'runtime_config_package':'collision_detection_launch',
                            'controllers_file':controllers_file,
                            'description_file':"shared.urdf.xacro",
                            'multi_setup':"true"
    }
    arguments_for_ur_moveit={'ur_type':ur_type, 'robot_ip':robot_ip, 'launch_rviz':'false','use_fake_hardware':use_fake_hardware}
    ur_driver_bringup_dir = get_package_share_directory("ur_bringup")
    collision_detection_launch_dir = get_package_share_directory("collision_detection_launch")


    ur_nodes = GroupAction(
        actions=[
            # PushRosNamespace(ur_ns),
            IncludeLaunchDescription(
                launch_description_source= PythonLaunchDescriptionSource(ur_driver_bringup_dir+'/launch/ur_control.launch.py'),
                launch_arguments=arguments_for_ur_launch.items()
            )
        ]
    )
    trajectory_generator_ur = IncludeLaunchDescription(
                launch_description_source= PythonLaunchDescriptionSource(collision_detection_launch_dir+'/launch/trajectory_generator_ur.launch.py'),
                launch_arguments=arguments_for_ur_moveit.items()
            )

    # kuka setup:
    kuka_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['position_trajectory_controller', "--controller-manager", "/controller_manager"],
    )

    # shared:
    unity_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='my_tcp_endpoint',
        parameters=[{'ROS_IP':'127.0.0.1'}]
    )

    return launch.LaunchDescription([
        ur_nodes,
        kuka_joint_trajectory_controller,
        # joint_publisher_ur,
        trajectory_generator_ur,
        # joint_publisher_kuka,
        unity_tcp_endpoint
    ])