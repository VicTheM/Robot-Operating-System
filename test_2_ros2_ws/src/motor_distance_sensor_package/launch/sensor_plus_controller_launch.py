"""
This file contains code to lauch two nodes at a time
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_distance_sensor_package',
            executable='distance_sensor_node',
            name='distance_sensor'
            ),
        Node(
            package='motor_distance_sensor_package',
            executable='motor_control_node',
            name='motor_control'
            )
        ])
