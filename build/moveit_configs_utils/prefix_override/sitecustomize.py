import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/perry/Jesus/Prueba6/install/moveit_configs_utils'
