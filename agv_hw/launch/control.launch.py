from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

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

    twist_mux_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agv_hw"),
            "config",
            "twist_mux.yaml",
        ]
    )

    joy_params = PathJoinSubstitution(
        [
            FindPackageShare("teleop_twist_joy"),
            "config",
            "ps3.config.yaml",
        ]
    )

    # nodes

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
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

    twist_mux_node = Node(
        package = 'twist_mux',
        executable = 'twist_mux',
        parameters=[twist_mux_yaml],
        remappings=[
            ('/cmd_vel_out', '/agv_controller/cmd_vel_unstamped'),
         ]
    )

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        parameters=[joy_params, {'publish_stamped_twist': False}],
        remappings=[('/cmd_vel', '/cmd_vel_joy')]
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        agv_broadcaster_spawner,
        agv_controller_spawner,
        twist_mux_node,
        joy_node,
        teleop_node,
    ]

    return LaunchDescription(nodes)
