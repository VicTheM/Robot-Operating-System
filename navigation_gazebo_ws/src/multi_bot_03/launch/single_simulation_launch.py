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
# This is all-in-one launch script intended for use by nav2 developers.
# Attn: in ROS2 examples, this file is called "tb3_simulation_launch.py"
#
# Modified: snowcron.com

from launch import LaunchDescription
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

import xacro

import sys
sys.path.append("src/multi_bot_03/launch") 
from globals import *

def generate_launch_description():
    # Initial pose of the robot
    x_pos = 0.0
    y_pos = 0.5
    z_pos = 0.01

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    world = LaunchConfiguration('world')
    # simulator = LaunchConfiguration('simulator')
    ##use_simulator = LaunchConfiguration('use_simulator')
    ##headless = LaunchConfiguration('headless')

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    keepout_mask = LaunchConfiguration('keepout_mask')

    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')    

    # def_maps_path declared in globals.py
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=def_maps_path,
        description='Full path to map file to load')
    
    declare_keepout_mask_yaml_cmd = DeclareLaunchArgument(
        'keepout_mask',
        default_value=def_keepout_mask_path,
        description='Full path to keepout mask file to load')    

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # def_nav2_params_path declared in globals.py
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=def_nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # def_nav2_bt_navigator_path declared in globals.py
    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=def_nav2_bt_navigator_path,
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')

    # def_rviz_path defined in globals.py
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=def_rviz_path,
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=def_world_path,
        description='Full path to world file to load')

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')
    
    ## Replaced by a single call below
    ## start_gazebo_server_cmd = ExecuteProcess(
    ##     condition=IfCondition(use_simulator),
    ##     cmd=['gzserver', '-s', 'libgazebo_ros_init.so', world],
    ##     cwd=[def_launch_dir], output='screen')
    ## 
    ## start_gazebo_client_cmd = ExecuteProcess(
    ##     condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
    ##     cmd=['gzclient'],
    ##     cwd=[def_launch_dir], output='screen')

    #print(">>>", os.path.join(get_package_share_directory('multi_bot_03')))

    # gazebo = ExecuteProcess(
    #     cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_factory.so', world],
    #     output='screen')

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #     launch_arguments={
    #         'gui': 'True',
    #         'server': 'True',
    #         #'models': os.path.join(get_package_share_directory('multi_bot_03'), 'models')
    #     }.items()
    # )       
    gazebo_node = ExecuteProcess(cmd=['gazebo', '--verbose', world,'-s', 'libgazebo_ros_factory.so'], output='screen')    

    robot_description_config = xacro.process_file(def_urdf) #xacro_file)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        remappings=def_remappings,
        parameters=[params]
    )    

    # Run the spawner node from the gazebo_ros package. 
    # The entity name doesn't really matter if you only have a single robot.
    spawn_entity_cmd = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-x', str(x_pos),
            '-y', str(y_pos),
            '-z', str(z_pos),             
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    pkg_path = os.path.join(get_package_share_directory(def_package_name))
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config/ekf.yaml')]#, {'use_sim_time': use_sim_time }]
    )    

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(def_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml_filename,
            'autostart': autostart }.items())
    bringup_timer_action = launch.actions.TimerAction( period=5.0, actions=[ bringup_cmd ])

    # rviz_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(def_launch_dir, 'rviz_launch.py')),
    #     condition=IfCondition(use_rviz),
    #     launch_arguments={'namespace': namespace,
    #                       'use_namespace': use_namespace,
    #                       'rviz_config': rviz_config_file}.items())
    rviz_node = Node(package    ='rviz2',
                     executable ='rviz2',
                     name       ='rviz2',
                     output     ='log',
                     arguments  =['-d', rviz_config_file])    

    #rviz_timer_action = launch.actions.TimerAction( period=3.0, actions=[ rviz_cmd ])

    params = { 'use_sim_time': use_sim_time}
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        #condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    # slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    # slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')    
    # slam_toolbox_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(slam_launch_file),
    #     condition=IfCondition(slam),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'autostart': autostart,
    #         'params_file': params_file,
    #     }.items())

    # node_map_tf = Node( 
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map/', 'odom/'],
    #     output='screen')

    gazebo_exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_node,
            on_exit=EmitEvent(event=Shutdown(reason='gazebo exited'))))

    rviz_exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    # Create the launch description and populate
    ld = LaunchDescription()

    #ld.add_action(node_map_tf)

    ld.add_action(declare_slam_cmd)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)    
    
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_keepout_mask_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    
    ld.add_action(declare_use_simulator_cmd)

    ld.add_action(robot_localization_node)
    
    # Add any conditioned actions
    #ld.add_action(start_gazebo_server_cmd)
    #ld.add_action(start_gazebo_client_cmd)
    ld.add_action(gazebo_node)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn_entity_cmd)
        
    #ld.add_action(rviz_cmd)
    ld.add_action(rviz_node) #rviz_timer_action)
    
    #ld.add_action(bringup_cmd)
    ld.add_action(bringup_timer_action)
    #ld.add_action(slam_toolbox_cmd)

    ld.add_action(gazebo_exit_event_handler)
    ld.add_action(rviz_exit_event_handler)

    return ld
