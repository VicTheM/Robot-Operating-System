# (c) robotics.snowcron.com
# Use: MIT license

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():

    arm_core_file_name = get_package_share_directory('arm_01') + '/description/arm.urdf.xacro'

    # Robot State Publisher 
    robot_state_publisher = Node(
        package ='robot_state_publisher',
        executable ='robot_state_publisher',
        name ='robot_state_publisher',
        output ='both',
        parameters =[{'robot_description': Command(['xacro', ' ', arm_core_file_name])}])


    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package ='gazebo_ros', 
        executable ='spawn_entity.py', 
        arguments = ['-entity', 'armA', '-topic', 'robot_description'],
        output ='screen')

    # Gazebo   
    world_file_name = 'empty.world'
    world = os.path.join(get_package_share_directory('arm_01'), '../worlds', world_file_name)
    gazebo_node = ExecuteProcess(cmd=['gazebo', '--verbose', world,'-s', 'libgazebo_ros_factory.so'], output='screen')

    # load and START the controllers in launch file
    
    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','joint_state_broadcaster'],
        output='screen')
    
    # joint_state_publisher_gui = Node(package  ='joint_state_publisher_gui',
    #                                 executable='joint_state_publisher_gui',
    #                                 output    ='screen',
    #                                 name      ='joint_state_publisher_gui')
    
    joint_trajectory_controller = ExecuteProcess( 
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_trajectory_controller'], 
        output='screen')

    rviz_config_file = get_package_share_directory('arm_01') + "/rviz/armA.rviz"
    rviz_node = Node(package    ='rviz2',
                     executable ='rviz2',
                     name       ='rviz2',
                     output     ='log',
                     arguments  =['-d', rviz_config_file])

    gazebo_exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_node,
            on_exit=EmitEvent(event=Shutdown(reason='gazebo exited'))))

    rviz_exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    return LaunchDescription([robot_state_publisher, spawn_entity, 
        gazebo_node, joint_state_broadcaster, joint_trajectory_controller,
        rviz_node, #joint_state_publisher_gui,
        gazebo_exit_event_handler, rviz_exit_event_handler])