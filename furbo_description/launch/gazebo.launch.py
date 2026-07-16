from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():

    package_name = "furbo_description"

    package_path = get_package_share_directory(package_name)

    xacro_file = os.path.join(
        package_path,
        "urdf",
        "furbo.urdf.xacro"
    )

    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True
            }
        ],
        output="screen"
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "furbo",
            "-z", "0.10"
        ],
        output="screen"
    )

    ros_gz_sim_path = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(package_path, "worlds", "furbo_world.sdf")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_path,
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": ["-r ", world_path]
        }.items()
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        ],
        output="screen"
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen"
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        diff_drive_controller,
    ])