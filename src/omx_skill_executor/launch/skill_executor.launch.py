"""skill_executor.launch.py

Launches the omx_skill_executor node.

Optionally publishes a static TF from the camera frame to the world frame
so that block poses from omx_perception can be transformed to the planning frame.

Usage:
  ros2 launch omx_skill_executor skill_executor.launch.py

With custom camera TF:
  ros2 launch omx_skill_executor skill_executor.launch.py \\
    publish_camera_tf:=true \\
    camera_frame:=default_cam \\
    camera_x:=0.30 camera_y:=0.0 camera_z:=0.50 \\
    camera_roll:=0.0 camera_pitch:=1.5708 camera_yaw:=0.0

Prerequisite terminals:
  T1: ros2 launch omx_bringup omx_control.launch.py use_mock_hardware:=true
  T2: ros2 launch omx_bringup omx_moveit.launch.py start_rviz:=false
  T3: ros2 run omx_motion_server motion_server
  T4: ros2 launch omx_perception omx_perception.launch.py
  T5: (this launch)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("omx_skill_executor")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg, "config", "skill_executor.yaml"]),
        description="Path to skill_executor parameters YAML",
    )
    publish_camera_tf_arg = DeclareLaunchArgument(
        "publish_camera_tf",
        default_value="false",
        description="Publish a static TF from camera_frame → world",
    )
    camera_frame_arg = DeclareLaunchArgument(
        "camera_frame", default_value="default_cam",
        description="Camera frame id (must match block_detection_node camera_frame)",
    )
    # Camera position in world frame (meters)
    camera_x_arg   = DeclareLaunchArgument("camera_x",   default_value="0.30")
    camera_y_arg   = DeclareLaunchArgument("camera_y",   default_value="0.00")
    camera_z_arg   = DeclareLaunchArgument("camera_z",   default_value="0.50")
    # Camera orientation as RPY (radians)
    camera_roll_arg  = DeclareLaunchArgument("camera_roll",  default_value="0.0")
    camera_pitch_arg = DeclareLaunchArgument("camera_pitch", default_value="1.5708")  # 90°
    camera_yaw_arg   = DeclareLaunchArgument("camera_yaw",   default_value="0.0")

    skill_node = Node(
        package="omx_skill_executor",
        executable="skill_executor",
        name="pick_place_skill",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    # Optional: static TF publisher (world → camera_frame)
    # Publish this if omx_perception camera_frame != planning_frame
    # and no other TF source exists.
    camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_static_tf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
        arguments=[
            LaunchConfiguration("camera_x"),
            LaunchConfiguration("camera_y"),
            LaunchConfiguration("camera_z"),
            LaunchConfiguration("camera_yaw"),
            LaunchConfiguration("camera_pitch"),
            LaunchConfiguration("camera_roll"),
            "world",                          # parent frame
            LaunchConfiguration("camera_frame"),  # child frame
        ],
    )

    return LaunchDescription([
        params_file_arg,
        publish_camera_tf_arg,
        camera_frame_arg,
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        camera_roll_arg,
        camera_pitch_arg,
        camera_yaw_arg,
        skill_node,
        camera_tf_node,
    ])
