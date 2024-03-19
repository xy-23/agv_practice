import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('agv_description'), 'urdf/robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    rviz_config = os.path.join(get_package_share_directory('agv_sim'), 'rviz/sim.rviz')

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'
        ])
    )

    spawn_entity_node = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = [
            '-topic',
            'robot_description',        
            '-entity',
            'agv'
            ],
        output='screen'
    )

    gazebo_controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gazebo_controller"],
    )

    gazebo_broadcaster_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gazebo_broadcaster"],
    )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        arguments = ['-d', rviz_config],
        output = 'screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        gazebo_controller_spawner_node,
        gazebo_broadcaster_spawner_node,
        rviz_node
    ])
