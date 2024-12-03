import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/andresarias23gmailcom/ros2_ws/install/tractor_description'
