from setuptools import find_packages, setup

package_name = 'simple_python_package_with_node'

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
            'victory_is_talking_python_node = simple_python_package_with_node.victory_is_talking_python_node:main',
            'talk_forever_node = simple_python_package_with_node.talk_forever_node:main'
        ],
    },
)
