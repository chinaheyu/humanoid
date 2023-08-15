#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('humanoid_base'),
        'config',
        'motor_mapping.yaml'
    )
    return LaunchDescription([
        Node(
            package='humanoid_base',
            namespace='/',
            executable='humanoid_base_node',
            name='humanoid_base',
            parameters = [config],
            arguments=['--ros-args', '--log-level', 'debug']
        )
    ])