from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    robot_description = Command([
        "xacro",
        " ",
        PathJoinSubstitution([
            FindPackageShare("furbo_description"),
            "urdf",
            "furbo.urdf.xacro"
        ])
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",

        parameters=[
            {
                "robot_description": robot_description
            }
        ]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])