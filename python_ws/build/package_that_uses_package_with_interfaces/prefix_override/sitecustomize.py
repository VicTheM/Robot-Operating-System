import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/victory/Robot-Operating-System/python_ws/install/package_that_uses_package_with_interfaces'
