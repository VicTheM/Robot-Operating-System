# Copyright (c) 2020 Samsung Research Russia
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
# Modified: robotics.snowcron.com

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml
import launch.actions
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch_ros.actions import PushRosNamespace
from launch import LaunchContext
from launch.actions import OpaqueFunction

import sys
sys.path.append("src/nav25d_03/launch") 
from globals import *

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    
    namespaced_params = LaunchConfiguration('namespaced_params')
    params_file = namespaced_params

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Variables
    lifecycle_nodes = ['map_saver']

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time}

    configured_params = RewrittenYaml(
        source_file=namespaced_params,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=def_nav2_params_path if namespace=='' else def_nav2_params_path_multi,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    # Nodes launching commands

    # start_slam_toolbox_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(slam_launch_file),
    #     launch_arguments={'use_sim_time': use_sim_time}.items())    
    start_slam_toolbox_cmd = GroupAction(
        [
            PushRosNamespace(namespace=namespace),

            SetRemap(src='/map', dst='map'),
            SetRemap(src='/map_metadata', dst='map_metadata'),
            SetRemap(src='/slam_toolbox/scan_visualization', dst='slam_toolbox/scan_visualization'),
            SetRemap(src='/slam_toolbox/graph_visualization', dst='slam_toolbox/graph_visualization'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch_file),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': configured_params,
                }.items())
        ]
     )
    # start_slam_toolbox_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(slam_launch_file),
    #     launch_arguments={'use_sim_time': use_sim_time, 
    #         'params_file': os.path.join(def_bringup_dir, 'config', 'slam_sensors.yaml')}.items())
    
        
    # remappings=[
    # ("/map", "map"),
    # ("/map_metadata", "map_metadata"),
    # ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
    # ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
    # ]    

    start_map_saver_server_cmd = Node(
        namespace=namespace,
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[configured_params],
        #remappings=[ ('/scan', namespace + '/scan'),  ]        
    )  

    start_lifecycle_manager_cmd = Node(
        namespace=namespace,
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        #node_name='lifecycle_manager_slam',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}],
    )    

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)

    # Running SLAM Toolbox
    ld.add_action(start_slam_toolbox_cmd)

    # Running Map Saver Server
    ld.add_action(start_map_saver_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
