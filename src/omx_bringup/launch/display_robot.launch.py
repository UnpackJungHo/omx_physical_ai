#!/usr/bin/env python3
#
# Phase B: 로봇 모델 표시
#
# 목적:
#   B-1) robot_state_publisher + RViz → OMX 3D 모델 확인
#   B-2) joint_state_publisher_gui 추가 → 슬라이더로 조인트 움직이기
#
# 사용법:
#   B-1) ros2 launch omx_bringup display_robot.launch.py
#   B-2) ros2 launch omx_bringup display_robot.launch.py use_gui:=true

from launch import LaunchDescription # 반환값 타입
from launch_ros.actions import Node  # 노드 실행

from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

# 런치 인수를 받아야 할 때
from launch.actions import DeclareLaunchArgument      # 등록
from launch.substitutions import LaunchConfiguration  # 참조

# xacro 명령을 실행해야 할 때:   
# Substitution 계열 객체들은 나중에 평가되는 표현식                                                           
from launch.substitutions import Command                                                    
from launch.substitutions import FindExecutable       # xacro 실행파일 찾기                 
from launch.substitutions import PathJoinSubstitution # 경로 조합                           
from launch_ros.substitutions import FindPackageShare # 패키지 경로 찾기 

def generate_launch_description():

    # 선언: 이런 인수가 있다고 ROS2에게 알림
    declared_arguments = [
        DeclareLaunchArgument(
            'use_gui',               # 인수 이름
            default_value='false',   # 기본값 (항상 문자열)
            description='...'        # ros2 launch --show-args 에 표시됨
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='조인트/링크 이름 앞에 붙는 접두사 (기본 없음)',
        )
    ]   

    use_gui = LaunchConfiguration('use_gui')
    prefix = LaunchConfiguration('prefix')

    # ── URDF 생성 (xacro → robot_description 문자열) ──────────────────
    #
    # xacro는 여러 .xacro 파일을 하나의 URDF XML로 합쳐준다.
    # use_mock_hardware=True → 실제 Dynamixel 없이 가상 하드웨어 사용
    #
    urdf_file = Command([ # Command os.open ~~ xacro를 URDF XML 문자열 전체로 실행하기 위해
        FindExecutable(name='xacro'), # # 결과: '/opt/ros/jazzy/bin/xacro' (which xacro 와 동일)
        ' ',
        PathJoinSubstitution([
            FindPackageShare('omx_bringup'),
            'config', 'omx_f', 'omx_f_with_camera.urdf.xacro',
        ]),
        # # 결과: '/.../share/omx_bringup/config/omx_f/omx_f_with_camera.urdf.xacro'
        ' prefix:=',        prefix,
        ' use_mock_hardware:=true',
    ])

    # ── RViz 설정 파일 ────────────────────────────────────────────────
    # 레퍼런스 패키지의 기본 RViz config 재사용
    rviz_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'rviz', 'open_manipulator.rviz',
    ])

    # ── 노드 정의 ─────────────────────────────────────────────────────

    # [1] robot_state_publisher
    #   - robot_description 파라미터로 URDF를 받아서
    #   - /tf, /tf_static 토픽으로 각 링크의 TF 변환을 발행
    #   - RViz가 이 TF를 읽어서 3D 모델을 그린다

    robot_state_publisher_node = Node(                                                     
        package='robot_state_publisher',                # ros2 pkg list 에서 찾을 수 있는 패키지명         
        executable='robot_state_publisher',             # 실행 파일명     
        parameters=[{'robot_description': urdf_file}],  # 파라미터
        output='screen',                                # 로그를 터미널에 출력
    )

    # [2-a] joint_state_publisher  (B-1, use_gui:=false 일 때)
    #   - GUI 없이 모든 조인트를 0(기본값)으로 /joint_states 발행
    #   - robot_state_publisher가 이를 받아서 TF를 계산 → RViz에서 정상 표시
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(use_gui),
        output='screen',
    )

    # [2-b] joint_state_publisher_gui  (B-2, use_gui:=true 일 때만)
    #   - GUI 슬라이더로 각 조인트 각도를 /joint_states 토픽에 발행
    #   - robot_state_publisher가 이 /joint_states를 받아서 TF를 업데이트
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
        output='screen',
    )

    # [3] rviz2
    #   - TF + robot_description을 시각화
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription(
        declared_arguments +
        [
            # 실행할 노드들
            robot_state_publisher_node,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node
        ]
    )
