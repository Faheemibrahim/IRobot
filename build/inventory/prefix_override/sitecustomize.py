import sys
if sys.prefix == '/usr/local':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/faheem/IRobot/install/inventory'
