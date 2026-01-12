import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/abrham/travel_bot_ws/install/ethiopia_travel'
