# (c) robotics.snowcron.com
# Use: MIT license

import os
from ament_index_python.packages import get_package_share_directory

global ros_shared_path
ros_shared_path = '/opt/ros/foxy/share/'

package_name = 'navigation_bot_04'
global bringup_dir
bringup_dir = get_package_share_directory(package_name)

global launch_dir
launch_dir = os.path.join(bringup_dir, 'launch')

global maps_path
maps_path = os.path.join(bringup_dir, '../maps', 'turtlebot3_world.yaml')

global nav2_bt_navigator_path
nav2_bt_navigator_path = os.path.join(ros_shared_path, 'nav2_bt_navigator',
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

global nav2_params_path
nav2_params_path = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

# Map fully qualified names to relative ones so the node's namespace can be prepended.
# In case of the transforms (tf), currently, there doesn't seem to be a better alternative
# https://github.com/ros/geometry2/issues/32
# https://github.com/ros/robot_state_publisher/pull/30
# TODO(orduno) Substitute with `PushNodeRemapping`
#              https://github.com/ros2/launch_ros/issues/56
global remappings
remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

global rviz_path
rviz_path = os.path.join(bringup_dir, 'rviz', 'robot.rviz')

global urdf
urdf = os.path.join(bringup_dir,'description','robot.urdf.xacro')

global world_path
world_path = os.path.join(bringup_dir, '../worlds', 'maze.sdf')

