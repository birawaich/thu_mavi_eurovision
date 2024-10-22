import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bstadler/repositories/thu_mavi_eurovision/pythontest/install/turtlebot4_python_tutorials'
