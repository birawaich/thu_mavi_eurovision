import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/li/Documents/GitHub/thu_mavi_eurovision/ros_move_test/install/turtlebot4_python_move'
