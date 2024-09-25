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
            'configurable_quote_publisher = package_with_configurable_nodes.configurable_quote_publisher_node:main'
        ],
    },
)
