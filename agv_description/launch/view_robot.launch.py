import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('agv_description'), 'urdf/robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    rviz_config = os.path.join(get_package_share_directory('agv_description'), 'rviz/view_robot.rviz')

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui",
    )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        arguments = ['-d', rviz_config],
        output = 'screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
