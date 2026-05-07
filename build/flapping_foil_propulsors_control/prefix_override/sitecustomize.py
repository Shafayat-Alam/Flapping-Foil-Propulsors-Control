import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shafa/flapping-propulsors-control/install/flapping_foil_propulsors_control'
