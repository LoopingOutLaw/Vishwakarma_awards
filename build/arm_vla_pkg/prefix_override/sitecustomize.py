import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/harshit/arm_vla_ws/install/arm_vla_pkg'
