#!/usr/bin/env python3
#
# Phase D: MoveIt2 붙이기
#
# D-1) move_group 노드 실행 (MoveItConfigsBuilder 사용)
# D-2) RViz MotionPlanning 패널 활성화 (moveit.rviz 사용)
# D-3) workspace_guard 노드 실행 (Planning Scene 충돌 오브젝트 추가)
#
# 사전 조건:
#   omx_control.launch.py 가 먼저 실행되어 있어야 한다.
#   (robot_state_publisher, controller_manager, 컨트롤러 3개 active 상태)
#
# 사용법:
#   # 터미널 1
#   ros2 launch omx_bringup omx_control.launch.py start_rviz:=false
#   # 터미널 2
#   ros2 launch omx_bringup omx_moveit.launch.py
#
# 확인:
#   ros2 node list | grep move_group
#   ros2 action list | grep follow_joint_trajectory

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='RViz2 시작 여부',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='시뮬레이션 시간 사용 여부',
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── MoveIt2 설정 ──────────────────────────────────────────────────
    # gripper 를 GripperCommand → FollowJointTrajectory (gripper_traj_controller) 로
    # 교체한 로컬 moveit_controllers.yaml 을 사용한다.
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

    # ── 노드 정의 ──────────────────────────────────────────────────────

    # [D-1] move_group 노드
    #   - MoveIt2 핵심 노드: 경로 계획, 실행, Planning Scene 관리
    #   - arm_controller / gripper_controller action server에 연결
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
        ],
    )

    # [D-2] RViz — moveit.rviz (MotionPlanning 패널 포함)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_moveit_config'),
        'config', 'moveit.rviz',
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        condition=IfCondition(start_rviz),
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
    )

    # [D-3] workspace_guard 노드
    #   - Planning Scene에 충돌 오브젝트 추가 (floor, 작업 영역 경계)
    workspace_guard_node = Node(
        package='omx_bringup',
        executable='workspace_guard',
        output='screen',
    )

    return LaunchDescription(
        declared_arguments + [
            move_group_node,
            rviz_node,
            workspace_guard_node,
        ]
    )
