#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('humanoid_arm'),
        'config',
        'motor_mapping.yaml'
    )
    return LaunchDescription([
        Node(
            package='humanoid_arm',
            namespace='/',
            executable='humanoid_arm_node',
            name='humanoid_arm',
            parameters = [config],
            respawn=False
        )
    ])
