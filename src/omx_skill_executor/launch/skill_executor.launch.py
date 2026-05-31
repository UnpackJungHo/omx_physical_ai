"""omx_skill_executor 의 모든 스킬 액션 서버를 한 번에 띄운다.

띄우는 노드:
  - pick_place_server      → action: /omx/pick_place
  - pick_place_all_server  → action: /omx/pick_place_all

전제:
  - omx_bringup (ros2_control + MoveIt2) 가 이미 실행 중
  - omx_motion_server 가 실행 중 (MoveToJoints / MoveToPose / MoveToNamed / GripperCommand)
  - omx_perception 의 box_cup_world_pose_node 가 실행 중
    (/perception/get_box_cup_world_poses 서비스 제공, BlockPose.polygon 포함)

사용:
  ros2 launch omx_skill_executor skill_executor.launch.py
  ros2 launch omx_skill_executor skill_executor.launch.py namespace:=robot1
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import yaml


def _load_ros_parameters(config_path: str, node_name: str) -> dict:
    with open(config_path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    try:
        return data[node_name]["ros__parameters"]
    except KeyError as exc:
        raise RuntimeError(
            f"{config_path} does not contain ros__parameters for {node_name}"
        ) from exc


def generate_launch_description() -> LaunchDescription:
    config_path = os.path.join(
        get_package_share_directory("omx_skill_executor"),
        "config",
        "skill_executor.yaml",
    )

    pick_place_server = Node(
        package="omx_skill_executor",
        executable="pick_place_server",
        name="pick_place_server",
        parameters=[_load_ros_parameters(config_path, "pick_place_server")],
        output="screen",
        emulate_tty=True,
    )

    pick_place_all_server = Node(
        package="omx_skill_executor",
        executable="pick_place_all_server",
        name="pick_place_all_server",
        parameters=[_load_ros_parameters(config_path, "pick_place_all_server")],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for ROS nodes.",
        ),
        PushRosNamespace(LaunchConfiguration("namespace")),
        pick_place_server,
        pick_place_all_server,
    ])
