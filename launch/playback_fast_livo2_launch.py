import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    bag_file = LaunchConfiguration('bag_file')
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'fast_livo', 'mapping_avia.launch.py', 'use_rviz:=True'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-d10', bag_file],
            output='screen'
        )
    ])