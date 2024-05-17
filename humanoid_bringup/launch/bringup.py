#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

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
            name='humanoid_web'
        ),
        Node(
            package='humanoid_chat',
            executable='humanoid_chat_node',
            name='humanoid_chat',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='humanoid_arm',
            executable='humanoid_arm_node',
            name='humanoid_arm'
        ),
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                    'robot_description': ParameterValue(
                        Command([
                                'xacro ',
                                os.path.join(
                                    get_package_share_directory('humanoid_description'),
                                    'urdf/humanoid_description.xacro'
                                )
                            ]
                        ),
                        value_type=str
                    )
                }
            ]
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                os.path.join(get_package_share_directory('rosbridge_server'), 'launch'),
                '/rosbridge_websocket_launch.xml'
            ]),
            launch_arguments={
                'port': '5001'
            }.items()
        ),
    ])
