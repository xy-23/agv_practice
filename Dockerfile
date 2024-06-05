FROM ros:humble-ros-base

# install ros2_control
RUN apt-get update && \
    apt-get install -y \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers && \
    rm -rf /var/lib/apt/lists/*

# install nav2
RUN apt-get update && \
    apt-get install -y \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup && \
    rm -rf /var/lib/apt/lists/*

# install others
RUN apt-get update && \
    apt-get install -y \
        ros-humble-xacro \
        ros-humble-twist-mux \
        ros-humble-joy-linux \
        ros-humble-teleop-twist-joy && \
    rm -rf /var/lib/apt/lists/*