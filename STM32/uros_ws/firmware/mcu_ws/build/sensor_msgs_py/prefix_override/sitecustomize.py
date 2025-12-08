import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/agustin/STM32CubeIDE/workspace_1.19.0/testURosSetup/uros_ws/firmware/mcu_ws/install/sensor_msgs_py'
