import os
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
  return LaunchDescription([
    Node(package='teleop_gui', executable='controller_gui', output='screen',
      emulate_tty=True)
  ])
