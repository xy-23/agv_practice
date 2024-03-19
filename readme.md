AGV_PRACTICE

### Build
```bash
# create workspace folder
mkdir -p agv_ws/src
cd agv_ws/src
git clone https://github.com/xy-23/agv_practice.git

# back to workspace folder
cd ..

# check dependencies
rosdep install --from-paths src --ignore-src

# build
colcon build --symlink-install
```

### Run simulation+===
```bash
# Under workspace folder
source install/setup.bash
ros2 launch agv_sim sim.launch.py world:=src/agv_practice/agv_sim/worlds/wall.world
```