import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shaharhalevi/ros2_workspaces/merpy_ws/install/teleop_control_node'
