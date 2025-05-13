import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/student/AdR/mi_ws/src/dron_misil/install/dron_misil'
