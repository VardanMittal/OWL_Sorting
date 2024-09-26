import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aaditya/OWL_Sorting/ros2_ws/install/owl_description'
