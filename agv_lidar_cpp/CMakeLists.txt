cmake_minimum_required(VERSION 3.8)
project(agv_lidar_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sockpp REQUIRED)

include_directories(include)

add_executable(lidar_node src/lidar_node.cpp src/lidar.cpp)
target_link_libraries(lidar_node Sockpp::sockpp)
ament_target_dependencies(lidar_node rclcpp sensor_msgs)

install(TARGETS
  lidar_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
