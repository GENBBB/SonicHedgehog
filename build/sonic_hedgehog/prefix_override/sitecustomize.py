import sys
if sys.prefix == '/home/gennadii/miniconda3/envs/sonic':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gennadii/TrueTech/task1/SonicHedgehog/install/sonic_hedgehog'
