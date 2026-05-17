"""PickPlace 스킬 서버 실행.

전제:
  - omx_bringup (ros2_control + MoveIt2) 가 이미 실행 중
  - omx_motion_server 가 실행 중 (MoveToJoints / MoveToPose / MoveToNamed / GripperCommand)
  - omx_perception 의 box_cup_world_pose_node 가 실행 중
    (/perception/get_box_cup_world_poses 서비스 제공)

사용:
  ros2 launch omx_skill_executor pick_place.launch.py
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
        "pick_place_skill.yaml",
    )

    pick_place_server = Node(
        package="omx_skill_executor",
        executable="pick_place_server",
        name="pick_place_server",
        parameters=[config_path],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([pick_place_server])
