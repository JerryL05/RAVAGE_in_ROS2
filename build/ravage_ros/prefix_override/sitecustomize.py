import sys
if sys.prefix == '/home/jliang/miniforge3/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jliang/ros_ws/install/ravage_ros'
