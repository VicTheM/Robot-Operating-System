import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'package_with_configurable_nodes'

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
            'configurable_amazing_quote_publisher_node = '
            'package_with_configurable_nodes.configurable_amazing_quote_publisher_node:main'
        ],
    },
)
