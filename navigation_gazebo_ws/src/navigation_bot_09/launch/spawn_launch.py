# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by robotics.snowcron.com

from launch import LaunchDescription

import launch.actions
import launch_ros.actions

import sys
sys.path.append("src/navigation_bot_09/launch") 
from globals import *

def generate_launch_description():

    return LaunchDescription([
        spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'navigation_bot_09'],
            output='screen')

#        # TODO(orduno) might not be necessary to have it's own package
#        launch_ros.actions.Node(
#            package='nav2_gazebo_spawner',
#            executable='nav2_gazebo_spawner',
#            output='screen',
#            arguments=[
#                '--robot_name', launch.substitutions.LaunchConfiguration('robot_name'),
#                '--robot_namespace', launch.substitutions.LaunchConfiguration('robot_name'),
#                '--turtlebot_type', launch.substitutions.LaunchConfiguration('turtlebot_type'),
#                '-x', launch.substitutions.LaunchConfiguration('x_pose'),
#                '-y', launch.substitutions.LaunchConfiguration('y_pose'),
#                '-z', launch.substitutions.LaunchConfiguration('z_pose')]),
    ])

