#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    omx_bringup_share = get_package_share_directory("omx_bringup")
    open_manipulator_description_share = get_package_share_directory("open_manipulator_description")
    open_manipulator_bringup_share = get_package_share_directory("open_manipulator_bringup")

    preview_domain_id = LaunchConfiguration("preview_domain_id")
    runtime_dir = LaunchConfiguration("runtime_dir")

    resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            os.path.join(open_manipulator_bringup_share, "worlds"),
            ":" + str(Path(open_manipulator_description_share).parent.resolve()),
        ],
    )

    set_domain = SetEnvironmentVariable(name="ROS_DOMAIN_ID", value=preview_domain_id)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": os.path.join(open_manipulator_bringup_share, "worlds", "empty_world.sdf") + " -r -v 1",
            "on_exit_shutdown": "true",
        }.items(),
    )

    xacro_file = os.path.join(omx_bringup_share, "config", "omx_f", "omx_f_with_camera.urdf.xacro")
    doc = xacro.process_file(
        xacro_file,
        mappings={
            "use_sim": "true",
            "use_mock_hardware": "false",
        },
    )
    robot_description = doc.toprettyxml(indent="  ")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description,
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
            "-name",
            "omx_f_preview",
            "-allow_renaming",
            "true",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    preview_player = Node(
        package="omx_bringup",
        executable="trajectory_preview_player",
        output="screen",
        parameters=[{"runtime_dir": runtime_dir}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("preview_domain_id", default_value="90"),
            DeclareLaunchArgument("runtime_dir", default_value="/tmp/omx_preview_runtime"),
            set_domain,
            resource_path,
            clock_bridge,
            gazebo,
            robot_state_publisher,
            spawn_entity,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[arm_controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=arm_controller_spawner,
                    on_exit=[preview_player],
                )
            ),
        ]
    )
