#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='loki_joystick_controller',
            executable='loki_joystick_reader',
            name='loki_joystick_reader'
        ),
        Node(
            package='loki_joystick_controller',
            executable='loki_joystick_controller',
            name='loki_joystick_controller'
        )
    ])

