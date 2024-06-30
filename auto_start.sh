sleep 1

docker start agv

sleep 1

screen -dmS control docker exec -it agv bash -c '/agv_ws/src/agv_practice/start_control.sh'

sleep 1

screen -dmS lidar docker exec -it agv bash -c '/agv_ws/src/agv_practice/start_lidar.sh'

sleep 1

screen -dmS amcl docker exec -it agv bash -c '/agv_ws/src/agv_practice/start_amcl.sh'

sleep 1

screen -dmS nav docker exec -it agv bash -c '/agv_ws/src/agv_practice/start_nav.sh'
