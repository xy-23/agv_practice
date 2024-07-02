cd /agv_ws
source install/setup.bash

export ROS_DOMAIN_ID=1
export FASTRTPS_DEFAULT_PROFILES_FILE=/agv_ws/src/agv_practice/whitelist.xml 
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp 

ros2 launch nav2_bringup localization_launch.py \
    map:=src/agv_practice/agv_nav/map/lab102-2.yaml \
    params_file:=src/agv_practice/agv_nav/config/nav2_params.yaml
