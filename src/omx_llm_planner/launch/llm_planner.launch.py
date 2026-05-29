"""omx_llm_planner 의 ExecuteCommand action server 를 띄운다.

전제:
  - omx_skill_executor (pick_place / pick_place_all) 실행 중
  - omx_motion_server (move_to_named) 실행 중
  - Ollama 서버가 llm_endpoint 에서 모델 서빙 중

사용:
  ros2 launch omx_llm_planner llm_planner.launch.py
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


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
        parameters=[config_path],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([planner_node])
