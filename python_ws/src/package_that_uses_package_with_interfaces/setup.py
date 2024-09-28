from setuptools import find_packages, setup

package_name = 'package_that_uses_package_with_interfaces'

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
            'amazing_quote_publisher_node = package_that_uses_package_with_interfaces.amazing_quote_publisher_node:main',
            'amazing_quote_subscription_node = package_that_uses_package_with_interfaces.amazing_quote_subscription_node:main',
            'service_server_node = package_that_uses_package_with_interfaces.service_server_node:main',
            'service_client_node = package_that_uses_package_with_interfaces.service_client_node:main',
            'fibo_action_server = package_that_uses_package_with_interfaces.fibo_action_server:main',
            'fibo_action_client = package_that_uses_package_with_interfaces.fibo_action_client:main'
            ],
    },
)
