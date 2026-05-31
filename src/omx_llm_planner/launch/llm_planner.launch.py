"""omx_llm_planner 의 ExecuteCommand action server 를 띄운다.

전제:
  - omx_skill_executor (pick_place / pick_place_all) 실행 중
  - omx_motion_server (move_to_named) 실행 중
  - Ollama 서버가 llm_endpoint 에서 모델 서빙 중

사용:
  ros2 launch omx_llm_planner llm_planner.launch.py
  ros2 launch omx_llm_planner llm_planner.launch.py namespace:=robot1
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
        get_package_share_directory("omx_llm_planner"),
        "config",
        "llm_planner.yaml",
    )

    planner_node = Node(
        package="omx_llm_planner",
        executable="planner_node",
        name="llm_planner",
        parameters=[_load_ros_parameters(config_path, "llm_planner")],
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
        planner_node,
    ])
