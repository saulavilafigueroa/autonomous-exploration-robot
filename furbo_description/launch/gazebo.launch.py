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
        parameters=[{"config_file": os.path.join(package_path, "config", "gz_bridge.yaml")}],
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

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(package_path, "config", "ekf.yaml")]
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[os.path.join(package_path, "config", "slam_toolbox.yaml")]
    )

    lidar_frame_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "lidar_link",
            "--child-frame-id", "furbo/base_footprint/lidar"
        ],
        output="screen"
    )

    imu_frame_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "imu_link",
            "--child-frame-id", "furbo/base_footprint/imu_sensor"
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
        robot_localization_node,
        slam_node,
        lidar_frame_fix,
        imu_frame_fix,
    ])