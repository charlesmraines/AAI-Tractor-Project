import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aai/AAI-Tractor-Project/tractor_ws/install/tractor_gps_driver'
