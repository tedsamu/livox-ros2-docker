from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare the 'bag_file' launch argument
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        description='Path to the ROS2 bag file to play back'
    )

    # Find the path to the fast_lio launch file
    fast_lio_launch_file = os.path.join(
        FindPackageShare('fast_lio').find('fast_lio'),
        'launch',
        'mapping.launch.py'
    )

    # ros2 bag play process
    play_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_file')],
        output='screen'
    )

    return LaunchDescription([
        bag_file_arg,
        play_process,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch_file),
            launch_arguments={'config_file': 'mid360.yaml'}.items()
        )
    ])
