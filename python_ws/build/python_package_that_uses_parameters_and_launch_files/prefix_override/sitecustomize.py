import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/victory/Robot-Operating-System/python_ws/install/python_package_that_uses_parameters_and_launch_files'
