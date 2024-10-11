import os 
from glob import glob
from setuptools import setup

package_name = 'multi_bot_nav_02'

# TBD: in future, also copy resources, like images for aruco markers

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
   
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),    
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, '../worlds'), glob(os.path.join('../worlds', '*.world'))),           
        (os.path.join('share', package_name, '../worlds'), glob(os.path.join('../worlds', '*.sdf'))), 
        
        (os.path.join('share', package_name, '../maps'), glob(os.path.join('../maps', '*.pgm'))), 
        (os.path.join('share', package_name, '../maps'), glob(os.path.join('../maps', '*.yaml'))), 

        (os.path.join('share', package_name, '../models/aruco_markers/materials/scripts'), 
            glob(os.path.join('../models/aruco_markers/materials/scripts', '*.material'))), 
        (os.path.join('share', package_name, '../models/aruco_markers/textures'), 
            glob(os.path.join('../models/aruco_markers/textures', '*.png'))), 

        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*.xacro'))),
        (os.path.join('share', package_name, 'description', 'xacro'), 
            glob(os.path.join('description','xacro', '*.xacro'))),
        # (os.path.join('share', package_name, 'description', 'meshes', 'armA'), 
        #     glob(os.path.join('description','meshes','armA', '*.stl'))), 
        # (os.path.join('share', package_name, 'description', 'meshes', 'gripA'), 
        #     glob(os.path.join('description','meshes','gripA', '*.stl'))),             
                    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics.snowcron.com',
    maintainer_email='noreply@snowcron.com',
    description="Deploying multiple robots",
    license='MIT License.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_follower = multi_bot_nav_02.01_waypoint_follower:main',
            'nav_through_poses = multi_bot_nav_02.02_nav_through_poses:main',
            'charger_docking = multi_bot_nav_02.03_charger_docking:main',
            'charger_docking_with_obstacles = multi_bot_nav_02.04_charger_docking_with_obstacles:main',
            
            'coordinator =                  multi_bot_nav_02.Coordinator:main',
            'path_follower =                multi_bot_nav_02.PathFollower:main',
            'global_planner_straight_line = multi_bot_nav_02.GlobalPlannerStraightLine:main',
            'img_layer_manager =            multi_bot_nav_02.ImgLayerManager:main',
        ],
    },
)
