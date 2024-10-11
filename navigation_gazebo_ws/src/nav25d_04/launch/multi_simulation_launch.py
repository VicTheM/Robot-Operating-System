# (c) robotics.snowcron.com
# Use: MIT license

from launch import LaunchDescription
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationNotEquals, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.actions import (GroupAction, SetEnvironmentVariable)
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess, EmitEvent
from launch_ros.actions import PushRosNamespace
from launch.events import Shutdown

from nav2_common.launch import ReplaceString
from launch_ros.substitutions import FindPackageShare

import xacro

from nav25d_04.Navigation import Navigation

import sys
sys.path.append("src/nav25d_04/launch") 
from globals import *

# ---

def generate_launch_description():
    robots = [
        { 'type': 'dif2wd', 'name': '', 'x_pos': 5.5, 'y_pos': 3.5, 'z_pos': 2.0, 
           "color_name" : "Blue", "color_rgb" : "0 0 1 1"},

        # {'name': 'robot1', 'x_pos': 0, 'y_pos': 2.0, 'z_pos': 2.0,
        #     "color_name" : 'Yellow', "color_rgb" : "1 1 0 1"},
        # {'name': 'robot2', 'x_pos': 0, 'y_pos': 4.0, 'z_pos': 2.0,
        #     "color_name" : 'Blue', "color_rgb" : "0 0 1 1"},
    ]

    arrNodes = []

    world = LaunchConfiguration('world')

    rviz_config_file = LaunchConfiguration('rviz_config_file')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Create the launch configuration variables
    
    # simple, slam, map
    #mode = LaunchConfiguration('mode')
    #declare_mode_cmd = DeclareLaunchArgument(
    #    'mode',
    #     default_value='simple',
    #     description='Whether run robot in a simple mode, SLAM or Map')
    # arrNodes.append(declare_mode_cmd)

    # slam = PythonExpression(["'", mode, "'=='slam'"]) 
    # declare_slam_cmd = DeclareLaunchArgument(
    #     'slam',
    #     default_value='True',
    #     description='Whether run a SLAM')
    # arrNodes.append(declare_slam_cmd)

    # map_yaml_file = LaunchConfiguration('map')
    # keepout_mask = LaunchConfiguration('keepout_mask')

    # params_file=def_nav2_params_path if robots[0]['name']=='' else def_nav2_params_path_multi
    # default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    # # def_nav2_bt_navigator_path declared in globals.py
    # declare_bt_xml_cmd = DeclareLaunchArgument(
    #     'default_bt_xml_filename',
    #     default_value=def_nav2_bt_navigator_path,
    #     description='Full path to the behavior tree xml file to use')
    # arrNodes.append(declare_bt_xml_cmd)
         
    # # def_maps_path declared in globals.py
    # declare_map_yaml_cmd = DeclareLaunchArgument(
    #     'map',
    #     default_value=def_maps_path,
    #     description='Full path to map file to load')
    # arrNodes.append(declare_map_yaml_cmd)
    
    # declare_keepout_mask_yaml_cmd = DeclareLaunchArgument(
    #     'keepout_mask',
    #     default_value=def_keepout_mask_path,
    #     description='Full path to keepout mask file to load')    
    # arrNodes.append(declare_keepout_mask_yaml_cmd)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    arrNodes.append(declare_use_sim_time_cmd)

    # def_rviz_path defined in globals.py
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=def_rviz_path,
        description='Full path to the RVIZ config file to use')
    arrNodes.append(declare_rviz_config_file_cmd)

    # use_rviz = LaunchConfiguration('use_rviz')
    # declare_use_rviz_cmd = DeclareLaunchArgument(
    #     'use_rviz',
    #     default_value='True',
    #     description='Whether to start RVIZ')
    # arrNodes.append(declare_use_rviz_cmd)

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=def_world_path,
        description='Full path to world file to load')
    arrNodes.append(declare_world_cmd)

    # gazebo_server_process = ExecuteProcess(
    #     cmd=['gzserver', 
    #         '-s', 'libgazebo_ros_init.so', 
    #         "-s", "libgazebo_ros_factory.so",
    #         world],
    #     on_exit=EmitEvent(event=Shutdown(reason='gazebo server exited')),
    #         #cwd=[def_worlds_dir], 
        
    #     output='screen')
    # arrNodes.append(gazebo_server_process)
    
    # gazebo_client_process = ExecuteProcess(
    #     cmd=['gzclient'],
    #     #cwd=[def_worlds_dir], 
    #     on_exit=EmitEvent(event=Shutdown(reason='gazebo client exited')),
    #     output='screen')  
    # arrNodes.append(gazebo_client_process)  

    gazebo_node = ExecuteProcess(
        # cmd=['gazebo', '--verbose', '--threads', '4', world, 
        cmd=['gazebo', '--verbose', world, 
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
            # '--',  # Separator between Gazebo args and ROS args
            # '--ros-args', '-p', 'publish_rate:=100.0'
        ],
        output='screen')        
    arrNodes.append(gazebo_node)    

    # arrNodes.append(start_gazebo_client_cmd)

    rviz_node = Node(package    ='rviz2',
                     executable ='rviz2',
                     name       ='rviz2',
                     output     ='log',
                     arguments  =['-d', rviz_config_file]) 
    arrNodes.append(rviz_node)    

    for robot in robots:
        namespace = "/" + robot['name'] if robot['name'] != "" else ""

        # --- nav2 params yaml

        # namespaced_params = ReplaceString(
        #     source_file=params_file, replacements = 
        #     {
        #         "namespace"  :  (namespace),                # /robot1
        #         "robot_name" :  (robot['name']),            # robot1 
        #         "robot_pos_x":  str(robot['x_pos']),
        #         "robot_pos_y":  str(robot['y_pos']),                
        #     }
        # )
       
        # --- URDF

        robot_description_config = xacro.process_file(
            def_urdf,
            mappings={
                "namespace"  : namespace,
                "robot_type": robot['type'],
                "robot_name" : robot['name'] + "/" if robot['name'] != '' else '',
                "robot_material_name": robot['color_name'],
                "robot_material_color_rgb": robot['color_rgb'],
            },
        )
            
        params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
        start_robot_state_publisher_cmd = Node(
            #condition=IfCondition(use_robot_state_pub),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            #remappings=def_remappings,
            parameters=[params]
        )    
        arrNodes.append(start_robot_state_publisher_cmd)

        # ---
             
        spawn_entity_cmd = Node(package='gazebo_ros', executable='spawn_entity.py',
            namespace=namespace,
            arguments=[
                "-topic", namespace + "/robot_description",
                '-entity', robot['name'],
                '-robot_namespace', robot['name'],
                '-x', str(robot['x_pos']),
                '-y', str(robot['y_pos']),
                '-z', str(robot['z_pos']),             
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
        arrNodes.append(spawn_entity_cmd)

        # ---

        navigation_node = Node(
            package='nav25d_04',
            executable='navigation',
            name='navigation_node',
            namespace=namespace,
            arguments=[
                'robot_name', robot['name'],     # No "-"
                '-x', str(robot['x_pos']),
                '-y', str(robot['y_pos']),
                '-z', str(robot['z_pos']),
            ], 
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
        arrNodes.append(navigation_node)

        # ---

        # Commented for now
        # robot_localization_node = Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     namespace=namespace,
        #     output='screen',
        #     parameters=[ 
        #         #os.path.join(def_bringup_dir, 'config', 'temp.yaml'),
        #         namespaced_params, 
        #         {'use_sim_time': use_sim_time }
        #     ],
        #     #remappings=[(namespace + '/odometry/filtered', namespace + '/odometry/filtered'),
        #        #('/set_pose', '/initialpose')
        #     #]
        # )    
        # arrNodes.append(robot_localization_node)

        # --- To be replaced with custom code

        # bringup_cmd = GroupAction([
        #     PushRosNamespace(robot['name']),

        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(os.path.join(def_launch_dir, 'bringup_launch.py')),
        #         condition=LaunchConfigurationNotEquals('mode', 'simple'),
        #         launch_arguments={
        #             'namespace': namespace,
        #             'use_namespace': 'True',
        #             'slam': slam,
        #             'map': map_yaml_file,
        #             'keepout_mask': keepout_mask,
        #             'use_sim_time': use_sim_time,
        #             'params_file': namespaced_params,
        #             'namespaced_params' : namespaced_params,
        #             'default_bt_xml_filename': default_bt_xml_filename,
        #             'autostart': 'True' }.items()),
        # ])

        # bringup_timer_action = launch.actions.TimerAction( period=5.0, actions=[ bringup_cmd ])
        # arrNodes.append(bringup_timer_action)

        # TBD: was temporary commented to simplify code, if uncommented, no problems
        # params = { 'use_sim_time': use_sim_time}
        # joint_state_publisher_node = Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     namespace=namespace,
        #     parameters=[params],
        #     #remappings=def_remappings,
        #     #condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
        # )
        # arrNodes.append(joint_state_publisher_node)

        node_tf_world_to_odom = launch_ros.actions.Node( 
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[str(robot['x_pos']), str(robot['y_pos']), str(robot['z_pos']), '0', '0', '0', 
                '/world', robot['name'] + '/odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
        arrNodes.append(node_tf_world_to_odom)

        # node_tf_map_to_odom = launch_ros.actions.Node( 
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=[str(robot['x_pos']), str(robot['y_pos']), str(robot['z_pos']), '0', '0', '0', 
        #         robot['name'] + '/map', robot['name'] + '/odom'],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen')
        # arrNodes.append(node_tf_map_to_odom)

        # node_tf_world_to_map = launch_ros.actions.Node( 
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=[#str(robot['x_pos']), str(robot['y_pos']), str(robot['z_pos']), '0', '0', '0', 
        #         '0', '0', '0', '0', '0', '0', 
        #         '/world', robot['name'] + '/map'],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen')
        # arrNodes.append(node_tf_world_to_map)

        # node_tf_world_to_keepout = launch_ros.actions.Node( 
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=[#str(robot['x_pos']), str(robot['y_pos']), str(robot['z_pos']), '0', '0', '0', 
        #         '0', '0', '0', '0', '0', '0', 
        #         '/world', robot['name'] + '/keepout_mask'],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen')            
        # arrNodes.append(node_tf_world_to_keepout)

    # ---

    # Instead, event is created above when Gazebo started
    # gazebo_exit_event_handler = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=gazebo_node,
    #         on_exit=EmitEvent(event=Shutdown(reason='gazebo exited'))))
    # arrNodes.append(gazebo_exit_event_handler)

    gazebo_exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_node,
            on_exit=EmitEvent(event=Shutdown(reason='gazebo exited'))))
    arrNodes.append(gazebo_exit_event_handler)    

    rviz_exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))
    arrNodes.append(rviz_exit_event_handler)

    # Create the launch description and populate
    ld = LaunchDescription()

    for node in arrNodes:
        ld.add_action(node)

    return ld


