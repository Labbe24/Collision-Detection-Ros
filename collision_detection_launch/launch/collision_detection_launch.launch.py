from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch

##
# @file
# Launch file for spawning the ROS2 nodes used in the trajectory generation module.
#

## This function generates a launch description for the Trajectory generation module.
#It includes a tcp-endpoint node, communicating with Unity.
#A move-group node providing MoveIt2 planning services
#A trajectory-generator Node providing trajectory generation services.
def generate_launch_description():
    ## @function
    #This function generates a launch description for the Trajectory generation module.
    #It includes a tcp-endpoint node, communicating with Unity.
    #A move-group node providing MoveIt2 planning services
    #A trajectory-generator Node providing trajectory generation services.


    collision_detection_launch_dir = get_package_share_directory("collision_detection_launch")

    trajectory_generator = IncludeLaunchDescription(
                launch_description_source= PythonLaunchDescriptionSource(collision_detection_launch_dir+'/launch/trajectory_generator.launch.py')
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