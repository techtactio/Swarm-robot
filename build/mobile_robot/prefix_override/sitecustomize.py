import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/adharsh/Desktop/Swarm-robot/install/mobile_robot'
