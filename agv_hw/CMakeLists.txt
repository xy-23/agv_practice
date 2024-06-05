cmake_minimum_required(VERSION 3.8)
project(agv_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# complie
add_library(
  agv_hw
  SHARED
  src/tiger_motor.cpp
)
target_compile_features(agv_hw PUBLIC cxx_std_17)
target_include_directories(agv_hw PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/agv_hw>
)
ament_target_dependencies(
  agv_hw PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface agv_hw.xml)

# install
install(
  DIRECTORY include/
  DESTINATION include/agv_hw
)
install(TARGETS agv_hw
  EXPORT export_agv_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY config ros2_control launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

# exports
ament_export_targets(export_agv_hw HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()