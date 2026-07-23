from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    description_path = os.path.join(
        get_package_share_directory("amr_description"),
        "urdf",
        "amr.urdf"
    )

    controllers = os.path.join(
        get_package_share_directory("amr_description"),
        "config",
        "controllers.yaml"
    )

    with open(description_path, "r") as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description
            }
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description
            },
            controllers,
        ],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster"
        ],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "base_controller"
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        joint_state_spawner,
        diff_drive_spawner,
    ])