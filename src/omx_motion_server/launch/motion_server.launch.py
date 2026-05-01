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
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ── grasp_detector ─────────────────────────────────────────────────
    # Step 1: Dynamixel current 기반 grasp 판단 노드.
    # MoveGroupInterface 를 쓰지 않으므로 robot_description 등은 불필요.
    # 노드 이름은 executable 의 C++ 생성자에서 부여한다.
    grasp_detector_node = Node(
        package='omx_motion_server',
        executable='grasp_detector',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            # 실물 캘리브레이션 결과 (30mm 박스, Goal Current=400 unit):
            #   빈 그리퍼 open/close: |current| < ~100 mA
            #   박스 그립 시       : |current| ~ 1076 mA
            # 안전 마진 두고 400 mA 로 설정. 값을 바꿀 때:
            #   ros2 param set /grasp_detector current_thresh_ma <new>
            {'gripper_joint': 'gripper_joint_1'},
            {'current_unit_to_ma': 2.69},
            {'current_thresh_ma': 400.0},
            {'position_error_thresh': 0.03},
            {'velocity_thresh': 0.05},
            {'stable_window_ms': 150},
            {'state_stale_ms': 200},
            {'publish_rate_hz': 20.0},
            {'controller_state_topic': '/gripper_traj_controller/controller_state'},
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
