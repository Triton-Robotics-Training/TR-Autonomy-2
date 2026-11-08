import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/justin/Robotics/TR-Autonomy-2-Justin-Ackley/src/install/spinnyrobot'
