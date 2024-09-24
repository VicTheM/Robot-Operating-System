import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/victory/RoboNish/test_1_ros2_ws/install/simple_python_package'
