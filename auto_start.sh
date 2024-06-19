sleep 1

docker start agv

screen \
    -dmS control \
    docker exec -it agv bash -c '/agv_ws/src/agv_practice/start_control.sh'