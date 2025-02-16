import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/uli/Desktop/tec/6to/ros2_env/install/courseworks'
