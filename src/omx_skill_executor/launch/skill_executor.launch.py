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
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


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
        parameters=[config_path],
        output="screen",
        emulate_tty=True,
    )

    pick_place_all_server = Node(
        package="omx_skill_executor",
        executable="pick_place_all_server",
        name="pick_place_all_server",
        parameters=[config_path],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        pick_place_server,
        pick_place_all_server,
    ])
