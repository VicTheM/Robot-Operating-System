import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration #, Command 
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
#import launch
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

import xacro

import yaml 
from launch.actions import ExecuteProcess 
import launch_ros.actions

package_name = 'navigation_bot_05'
# world_file = 'room.world'

def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path,'rviz','urdf_config.rviz')
    robot_description_config = xacro.process_file(xacro_file)
    
    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
#        output='screen',
        remappings=remappings,
        parameters=[params]
    )
    
    params = { 'use_sim_time': use_sim_time}
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        #condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

#    joint_state_publisher_gui = Node(
#        package='joint_state_publisher_gui',
#        executable='joint_state_publisher_gui',
#        name='joint_state_publisher_gui',
#        parameters=[params],
#        #condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')
#    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )    

    robot_localization_node = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_path, 'config/ekf.yaml')]#, {'use_sim_time': use_sim_time }]
    )

    # Create our own temporary YAML files that include substitutions
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')  
    autostart = True
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
    }    

    # Map server
    map_yaml_path = os.path.join(pkg_path, 'maps', 'map.yaml')
    params_file = LaunchConfiguration('params')
    remappings = []  
    lifecycle_nodes = [#'controller_server',
                       #'planner_server',
                       #'recoveries_server',
                       #'bt_navigator',
                       'map_server', #---------------ADDED--------------
                       'amcl',
                       #'waypoint_follower'
                       ]
    
#    configured_params = RewrittenYaml(
#        source_file=params_file,
#        param_rewrites=param_substitutions,
#        convert_types=True)

    params = { 'source_file':params_file, 'param_rewrites':param_substitutions, 'convert_types':True }        
    configured_params = params
    
    nav2_controller = launch_ros.actions.Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings)

    nav2_planner = launch_ros.actions.Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings)

    nav2_recoveries = launch_ros.actions.Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=remappings)

    nav2_bt_navigator = launch_ros.actions.Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings)

    nav2_waypoint_follower = Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            remappings=remappings)        

    nav2_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[{'yaml_filename': map_yaml_path}])    

    nav2_amcl = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            remappings=remappings)

    nav2_lifecycle_manager = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])    

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use sim time if true'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),        
        DeclareLaunchArgument('params',
            default_value=[pkg_path, '/config/nav2_params.yaml'],
            description='Full path to the ROS2 parameters file to use'),     
        DeclareLaunchArgument('default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),  

#        DeclareLaunchArgument(name='gui', default_value='True',
#                                            description='Flag to enable joint_state_publisher_gui'),
#        DeclareLaunchArgument(name='model', default_value=default_model_path,
#                                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_path,
            description='Absolute path to rviz config file'),      

        robot_state_publisher_node,
        #joint_state_publisher_node,

        robot_localization_node,
        rviz_node,

        #nav2_controller,
        #nav2_planner,
        #nav2_recoveries,
        #nav2_bt_navigator,
        #nav2_waypoint_follower,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager
        
    ])

    