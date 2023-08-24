#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                os.path.join(get_package_share_directory('humanoid_base'), 'launch'),
                '/humanoid_base.py'
            ])
        ),
        Node(
            package='humanoid_web',
            executable='humanoid_web_node',
            name='humanoid_web',
            respawn=True
        ),
        # Node(
        #     package='humanoid_chat',
        #     executable='humanoid_chat_node',
        #     name='humanoid_chat',
        #     respawn=True
        # ),
        Node(
            package='humanoid_arm',
            executable='humanoid_arm_node',
            name='humanoid_arm',
            respawn=True
        )
    ])
