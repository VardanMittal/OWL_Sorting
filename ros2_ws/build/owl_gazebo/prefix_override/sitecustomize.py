import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vardan/OWL_Sorting/ros2_ws/install/owl_gazebo'
