import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bgb_6342/manipulator_ws/install/manipulator_hardware'
