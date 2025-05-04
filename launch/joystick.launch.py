#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kvasir_joystick_controller',
            executable='kvasir_joystick_reader',
            name='kvasir_joystick_reader'
        ),
        Node(
            package='kvasir_joystick_controller',
            executable='kvasir_joystick_controller',
            name='kvasir_joystick_controller'
        )
    ])

