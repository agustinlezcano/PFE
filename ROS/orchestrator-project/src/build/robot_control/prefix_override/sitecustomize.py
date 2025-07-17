import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/agustin/pfe/PFE/ROS/orchestrator-project/src/install/robot_control'
