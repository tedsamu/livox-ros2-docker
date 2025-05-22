from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Declare the 'record' launch argument
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Enable ros2 bag recording'
    )

    # Find the path to the livox driver launch file
    livox_launch_file = os.path.join(
        FindPackageShare('livox_ros_driver2').find('livox_ros_driver2'),
        'launch_ROS2',
        'msg_MID360_launch.py'
    )

    # Find the path to the fast_lio launch file
    fast_lio_launch_file = os.path.join(
        FindPackageShare('fast_lio').find('fast_lio'),
        'launch',
        'mapping.launch.py'
    )

    # Only add the record process if 'record' is true
    record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-e', '/livox/.*'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record'))
    )

    return LaunchDescription([
        record_arg,
        # Include the Livox ROS driver launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch_file),
            launch_arguments={'config_file': 'mid360.yaml'}.items()
        ),
        record_process
    ])