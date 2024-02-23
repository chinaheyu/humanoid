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
            name='chassis_driver',
            remappings=[
                ('/cmd_vel', '/smoothed_cmd_vel')
            ]
        ),
        Node(
            package='kobuki_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            parameters=[
                {
                    "speed_lim_v": 0.1,
                    "speed_lim_w": 0.3,
                    "accel_lim_v": 0.2,
                    "accel_lim_w": 0.6,
                    "decel_factor": 1.0,
                    "frequency": 20.0,
                    "feedback": 0
                }
            ],
            remappings=[
                ('~/input', '/cmd_vel'),
                ('~/smoothed', '/smoothed_cmd_vel')
            ]
        )
    ])
