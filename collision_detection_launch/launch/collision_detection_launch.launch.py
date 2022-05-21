from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch

def generate_launch_description():



    collision_detection_launch_dir = get_package_share_directory("collision_detection_launch")

    trajectory_generator = IncludeLaunchDescription(
                launch_description_source= PythonLaunchDescriptionSource(collision_detection_launch_dir+'/launch/trajectory_generator_ur.launch.py')
            )


    # shared:
    unity_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='unity_tcp_endpoint',
        parameters=[{'ROS_IP':'127.0.0.1'}]
    )

    return launch.LaunchDescription([
        trajectory_generator,
        unity_tcp_endpoint
    ])