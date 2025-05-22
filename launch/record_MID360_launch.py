from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Find the path to the rviz_MID360_launch.py file
    livox_launch_file = os.path.join(
        FindPackageShare('livox_ros_driver2').find('livox_ros_driver2'),
        'launch_ROS2',
        'rviz_MID360_launch.py'
    )

    return LaunchDescription([
        # Include the Livox ROS driver launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launch_file)
        ),
        # Record all Livox topics using ros2 bag
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-e', '/livox/.*'],
            output='screen'
        )
    ])