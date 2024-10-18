import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yoheiyanagi/ros2_ws/src/realsense_yolo_detection/install/yolo'
