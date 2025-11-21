import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jy/cobot1_ws/src/doosan-robot2/dsr_rokey2/install/dsr_rokey2'
