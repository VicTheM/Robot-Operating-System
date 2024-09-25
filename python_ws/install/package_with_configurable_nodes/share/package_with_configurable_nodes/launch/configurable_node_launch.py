"""The launch descrption function"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
# This function returns a launch description
# would have as well been done with either xml
# or CMake in the C++ counterpart
#
#	return proto:
#		return ([ ... ])
    return LaunchDescription([
        Node(
            package='package_with_configurable_nodes',
            executable='configurable_amazing_quote_publisher_node',
            name='configurable_amazing_quote_publisher_node',
            parameters=[{
                "topic_name": "Set_by_Launch_File",
                "period": 1,
                "quote": "Victory, you still have alot to learn.",
                "philosopher_name": "Victory",
            }]
        )
    ])

# Of course, many other things can still be in a launch file, but we
# don't need them here
