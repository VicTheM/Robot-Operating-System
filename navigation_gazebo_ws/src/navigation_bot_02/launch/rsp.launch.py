import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

package_name = 'navigation_bot_02'
# world_file = 'room.world'

def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path,'rviz','urdf_config.rviz')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
#        output='screen',
        parameters=[params]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

#    joint_state_publisher_gui = Node(
#        package='joint_state_publisher_gui',
#        executable='joint_state_publisher_gui',
#        name='joint_state_publisher_gui'
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
         parameters=[os.path.join(pkg_path, 'config/ekf.yaml'), {'use_sim_time': use_sim_time }]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use sim time if true'),
        DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_path,
            description='Absolute path to rviz config file'),            

        robot_state_publisher_node,
        joint_state_publisher_node,
        robot_localization_node,

        #joint_state_publisher_gui,
        rviz_node
    ])

    