"""PickDetected 스킬 서버 실행.

전제:
  - omx_bringup (ros2_control + MoveIt2) 가 이미 실행 중
  - omx_motion_server 가 실행 중 (MoveToJoints / MoveToPose / MoveToNamed / GripperCommand)
  - omx_perception 의 get_block_poses_server 가 실행 중

사용:
  ros2 launch omx_skill_executor pick_skill.launch.py
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
        "pick_skill.yaml",
    )

    pick_server = Node(
        package="omx_skill_executor",
        executable="pick_detected_server",
        name="pick_detected_server",
        parameters=[config_path],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([pick_server])
