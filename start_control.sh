cd /agv_ws
source install/setup.bash

export ROS_DOMAIN_ID=1
export FASTRTPS_DEFAULT_PROFILES_FILE=/agv_ws/src/agv_practice/whitelist.xml 
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp 

ros2 launch agv_hw control.launch.py