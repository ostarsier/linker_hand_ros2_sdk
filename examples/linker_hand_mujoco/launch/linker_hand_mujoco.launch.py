#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_mujoco',
            executable='linker_hand_mujoco_node',
            name='linker_hand_mujoco_node',
            output='screen',
            parameters=[{
                'hand_type': 'right',
                'hand_joint': "L20",
                'topic_hz': 30,
                'is_touch': True,
            }],
        ),
    ])
