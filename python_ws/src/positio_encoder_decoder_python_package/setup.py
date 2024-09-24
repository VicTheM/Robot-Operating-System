from setuptools import find_packages, setup

package_name = 'positio_encoder_decoder_python_package'

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
            'position_encoder_node = positio_encoder_decoder_python_package.position_encoder_node:main',
            'position_decoder_node = positio_encoder_decoder_python_package.position_decoder_node:main'
        ],
    },
)
