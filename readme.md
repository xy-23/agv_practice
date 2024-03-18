### Build
```bash
# create workspace
mkdir -p agv_ws/src
cd agv_ws/src
git clone https://github.com/xy-23/agv_practice.git

# back to workspace folder
cd ..

# check dependecies
rosdep install --from-paths src --ignore-src

# build
colcon build --symlink-install
```

### Run simulation
```bash
# Under workspace folder
source install/setup.bash
ros2 launch agv_simulation simulation.launch.py world:=src/agv_practice/agv_simulation/worlds/wall.world
```