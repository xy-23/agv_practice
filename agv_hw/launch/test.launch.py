from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("agv_hw"),
                    "urdf",
                    "robot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("agv_hw"),
            "ros2_control",
            "agv_controller.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("agv_hw"), "rviz", "test.rviz"]
    )

    # nodes!

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    agv_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "agv_broadcaster",
        ],
    )

    agv_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "agv_controller",
        ],
    )

    delay_rviz_after_agv_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=agv_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_agv_controller_spawner_after_agv_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=agv_broadcaster_spawner,
            on_exit=[agv_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        agv_broadcaster_spawner,
        delay_rviz_after_agv_broadcaster_spawner,
        delay_agv_controller_spawner_after_agv_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
