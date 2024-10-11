import os

from ament_index_python.packages import get_package_share_directory

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#from time import sleep
#import filecmp

import xacro

package_name = 'diff_drive_controller_bot'
# world_file = 'room.world'

def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path,'rviz','urdf_config.rviz')
    robot_description_config = xacro.process_file(xacro_file)

    controller_config = os.path.join(pkg_path, "config", "my_controllers.yaml")
    robot_description = {"robot_description": robot_description_config.toxml()}

    #print("MODEL %s" % robot_description['robot_description'])
    #sleep(10)
    #print("Config %s" % open(controller_config, 'r').read())
    #sleep(10)

    node_controller = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="both",
    )

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
#    node_joint_state_publisher = Node(
#        package='joint_state_publisher',
#        executable='joint_state_publisher',
#        name='joint_state_publisher'
#    )

#    node_joint_state_publisher_gui = Node(
#        package='joint_state_publisher_gui',
#        executable='joint_state_publisher_gui',
#        name='joint_state_publisher_gui'
#    )

#    node_rviz = Node(
#        package='rviz2',
#        executable='rviz2',
#        name='rviz2',
#        output='screen',
#        arguments=['-d', LaunchConfiguration('rvizconfig')],
#    )    

#    diff_drive_spawner = Node(
#        package="controller_manager",
#        executable="spawner.py",
#        arguments=["diff_cont"],
#    )
#
#    joint_broad_spawner = Node(
#        package="controller_manager",
#        executable="spawner.py",
#        arguments=["joint_broad"],
#    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_path,
            description='Absolute path to rviz config file'),            

        node_controller,
        node_robot_state_publisher,
#        node_joint_state_publisher,
#        node_joint_state_publisher_gui,
#        node_rviz,

        #diff_drive_spawner,
        #joint_broad_spawner        
    ])

    