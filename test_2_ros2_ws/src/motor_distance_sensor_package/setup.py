import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'motor_distance_sensor_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='victory',
    maintainer_email='victorychibuike121@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'distance_sensor_node = motor_distance_sensor_package.distance_sensor_node:main',
            'motor_control_node = motor_distance_sensor_package.motor_control_node:main'
        ],
    },
)
