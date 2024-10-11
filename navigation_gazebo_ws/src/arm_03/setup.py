
import os 
from glob import glob
from setuptools import setup

package_name = 'arm_03'

# TBD: in future, also copy resources, like images for aruco markers

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),    
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, '../worlds'), glob(os.path.join('../worlds', '*.world'))),           
        (os.path.join('share', package_name, '../worlds'), glob(os.path.join('../worlds', '*.sdf'))), 
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*.xacro'))),
        (os.path.join('share', package_name, 'description', 'xacro'), 
            glob(os.path.join('description','xacro', '*.xacro'))),
        (os.path.join('share', package_name, 'description', 'meshes', 'armA'), 
            glob(os.path.join('description','meshes','armA', '*.stl'))), 
        (os.path.join('share', package_name, 'description', 'meshes', 'gripA'), 
            glob(os.path.join('description','meshes','gripA', '*.stl'))),             
                    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics.snowcron.com',
    maintainer_email='noreply@snowcron.com',
    description="Robotic Arm Package armA",
    license='MIT License.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'trajectory_points_act_server = arm_03.basic_trajectory_action_service:main',
                
        ],
    },
)
