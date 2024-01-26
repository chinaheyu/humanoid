#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[
                os.path.join(
                    get_package_share_directory('humanoid_chassis'),
                    'config',
                    'ps3_config.yaml'
                )
            ]
        ),
        Node(
            package='humanoid_chassis',
            executable='chassis_driver_node',
            name='chassis_driver'
        )
    ])
