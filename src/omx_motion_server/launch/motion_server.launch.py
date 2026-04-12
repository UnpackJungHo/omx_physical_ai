#!/usr/bin/env python3
#
# motion_server.launch.py
#
# MoveGroupInterface 는 robot_description / robot_description_semantic / kinematics 파라미터가
# 노드에 직접 전달돼야 한다. omx_moveit.launch.py 와 동일한 MoveItConfigsBuilder 를 사용한다.
#
# 사전 조건:
#   터미널 1: ros2 launch omx_bringup omx_control.launch.py use_mock_hardware:=true
#   터미널 2: ros2 launch omx_bringup omx_moveit.launch.py start_rviz:=false
#   터미널 3: ros2 launch omx_motion_server motion_server.launch.py

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # gripper 를 FollowJointTrajectory (gripper_traj_controller) 로 사용하는
    # 로컬 moveit_controllers.yaml 을 trajectory_execution 에 전달한다.
    # 업스트림 기본값은 GripperCommand 타입으로 velocity profile 을 무시한다.
    local_moveit_controllers = os.path.join(
        get_package_share_directory('omx_bringup'),
        'config', 'omx_f', 'moveit_controllers.yaml'
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='omx_f',
            package_name='open_manipulator_moveit_config',
        )
        .robot_description_semantic(
            str(Path('config') / 'omx_f' / 'omx_f.srdf')
        )
        .joint_limits(
            str(Path('config') / 'omx_f' / 'joint_limits.yaml')
        )
        .trajectory_execution(local_moveit_controllers)
        .robot_description_kinematics(
            str(Path('config') / 'omx_f' / 'kinematics.yaml')
        )
        .to_moveit_configs()
    )

    motion_server_node = Node(
        package='omx_motion_server',
        executable='motion_server',
        name='motion_server',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([motion_server_node])
