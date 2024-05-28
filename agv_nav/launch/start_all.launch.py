from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
def generate_launch_description():

    #Define the dir needed
    bringup_dir = get_package_share_directory("agv_nav")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    agv_hw_dir = get_package_share_directory("agv_hw")
    teleop_twist_joy_dir=get_package_share_directory("teleop_twist_joy")
    twist_mux_yaml_dir = os.path.join(bringup_dir, "config","twist_mux.yaml")
    lab2_yaml_dir = os.path.join(bringup_dir, "map","lab2.yaml")
    nav2_params_yaml_dir = os.path.join(bringup_dir, "config","nav2_params.yaml")
    
   
    agv_lidar_node = Node(
        package = 'agv_lidar',
        executable = 'tcp_client',
    )


    agv_hw_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(agv_hw_dir, "launch","test.launch.py"))
    )

    teleop_twist_joy_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(teleop_twist_joy_dir, 'launch', 'teleop-launch.py'))
    )

    twist_mux_node = Node(
        package = 'twist_mux',
        executable = 'twist_mux',
        arguments=['--params-file', twist_mux_yaml_dir ],
        remappings=[
            ('/cmd_vel_out', '/agv_controller/cmd_vel_unstamped'),
         ]
    )

    
    amcl_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, "launch","location_launch.py")),
                    launch_arguments={"params_file": nav2_params_yaml_dir,
                                      "map":lab2_yaml_dir
                                      }.items()
    )


    nav_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, "launch","navigation_launch.py"))
    )


    
    return LaunchDescription([
        teleop_twist_joy_launch ,
        agv_lidar_node,
        twist_mux_node,
        amcl_launch,
        nav_launch,
        agv_hw_launch
    ])
