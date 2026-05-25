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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    local_robot_description = os.path.join(
        get_package_share_directory('omx_bringup'),
        'config', 'omx_f', 'omx_f_with_camera.urdf.xacro'
    )
    local_srdf = os.path.join(
        get_package_share_directory('omx_bringup'),
        'config', 'omx_f', 'omx_f.srdf'
    )
    local_kinematics = os.path.join(
        get_package_share_directory('omx_bringup'),
        'config', 'omx_f', 'kinematics.yaml'
    )
    # workspace 한계 단일 설정 소스 (workspace_guard 와 공유).
    workspace_yaml = os.path.join(
        get_package_share_directory('omx_bringup'),
        'config', 'omx_f', 'workspace.yaml'
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='omx_f',
            package_name='open_manipulator_moveit_config',
        )
        .robot_description(local_robot_description)
        .robot_description_semantic(local_srdf)
        .joint_limits(
            str(Path('config') / 'omx_f' / 'joint_limits.yaml')
        )
        .trajectory_execution(local_moveit_controllers)
        .robot_description_kinematics(local_kinematics)
        .to_moveit_configs()
    )

    # name= 을 지정하지 않는다 — launch_ros 가 '--ros-args -r __node:=motion_server'
    # 를 글로벌 args 로 주입하면, MoveIt2 PlanningSceneMonitor 가 내부 pnode_ 를
    # rclcpp::NodeOptions() (기본 use_global_arguments=true) 로 만들 때 이 글로벌
    # remap 이 적용돼 pnode_ 이름이 '<parent>_private_<ptr>' 대신 'motion_server'
    # 로 덮어써진다. 결과적으로 ros2 node list 에 /motion_server 가 두 개로 보이고
    # 액션 서버 discovery 가 뒤틀린다. 노드 이름은 C++ 생성자
    # Node("motion_server", ...) 에서 부여하므로 여기서는 비워둔다.
    motion_server_node = Node(
        package='omx_motion_server',
        executable='motion_server',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            workspace_yaml,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ── grasp_detector ─────────────────────────────────────────────────
    # Dynamixel current 기반 grasp 판단 노드 (force-only).
    # MoveGroupInterface 를 쓰지 않으므로 robot_description 등은 불필요.
    # 노드 이름은 executable 의 C++ 생성자에서 부여한다.
    grasp_detector_node = Node(
        package='omx_motion_server',
        executable='grasp_detector',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            # 실물 캘리브레이션 결과:
            #   빈 그리퍼 close: force_estimate ≈ -100 mA 부근
            #   박스 그립 시  : force_estimate ≈ -1150 mA
            # signed 음수 임계값으로 close 방향 부하만 잡는다.
            # 값을 바꿀 때:
            #   ros2 param set /grasp_detector grasp_force_threshold_ma <new>
            {'gripper_joint': 'gripper_joint_1'},
            {'current_unit_to_ma': 2.69},
            {'grasp_force_threshold_ma': -1000.0},
            {'stable_window_ms': 150},
            {'state_stale_ms': 200},
            {'publish_rate_hz': 20.0},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time when running with Gazebo / ros_gz clock',
        ),
        motion_server_node,
        grasp_detector_node,
    ])
