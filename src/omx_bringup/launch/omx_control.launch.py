#!/usr/bin/env python3
#
# 1) ros2_control_node (controller_manager) 실행
# 2) joint_state_broadcaster, arm_controller, gripper_controller spawner 추가
# 3) joint_trajectory_executor → home 포즈로 초기 이동
#
# 사용법:
#   # mock 모드 (하드웨어 없이 테스트)
#   ros2 launch omx_bringup omx_control.launch.py use_mock_hardware:=true
#
#   # 실제 하드웨어
#   ros2 launch omx_bringup omx_control.launch.py
#   ros2 launch omx_bringup omx_control.launch.py port_name:=/dev/ttyUSB0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='RViz 시작 여부',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='조인트/링크 이름 접두사',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='true: mock hardware (하드웨어 없이 테스트), false: 실제 하드웨어',
        ),
        DeclareLaunchArgument(
            'use_hardware_mock',
            default_value='false',
            description='Alias for use_mock_hardware',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyACM0',
            description='Dynamixel USB 시리얼 포트 (실제 하드웨어 전용)',
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_hardware_mock = LaunchConfiguration('use_hardware_mock')
    port_name = LaunchConfiguration('port_name')
    mock_hardware_enabled = PythonExpression([
        "'true' if '",
        use_mock_hardware,
        "'.lower() in ('true', '1', 'yes', 'on') or '",
        use_hardware_mock,
        "'.lower() in ('true', '1', 'yes', 'on') else 'false'",
    ])

    # ── URDF 생성 ──────────────────────────────────────────────────────
    urdf_file = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('omx_bringup'),
                'config', 'omx_f', 'omx_f_with_camera.urdf.xacro',
            ]),
            ' prefix:=', prefix,
            ' use_mock_hardware:=', mock_hardware_enabled,
            ' port_name:=', port_name,
        ]),
        value_type=str,
    )

    # ── 설정 파일 경로 ─────────────────────────────────────────────────

    # 하드웨어 세팅
    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('omx_bringup'),
        'config', 'omx_f', 'hardware_controller_manager.yaml',
    ])

    # 초기 자세 
    initial_positions_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config', 'omx_f', 'initial_positions.yaml',
    ])

    # rviz 
    rviz_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'rviz', 'open_manipulator.rviz',
    ])

    # ── 노드 정의 ──────────────────────────────────────────────────────

    # controller_manager
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': urdf_file},
            controller_manager_config,
        ],
        output='screen',
    )

    # robot_state_publisher
    # joint_state_broadcaster가 발행하는 /joint_states를 받아 TF 계산
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
        output='screen',
    )

    # 컨트롤러 spawner
    # gripper_controller : JointTrajectoryController (MoveIt trajectory 전체 실행)
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'arm_controller',
            'gripper_controller',
        ],
        output='screen',
    )

    # joint_trajectory_executor
    # step1 [0,0,0,0,0] → step2 [0,-1.57,1.57,1.57,0] (home 포즈)
    # arm_controller가 active 상태가 되면 자동으로 trajectory 실행
    joint_trajectory_executor = Node(
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[initial_positions_config],
        output='screen',
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    # robot_state_publisher 가 시작된 뒤에 ros2_control_node 를 띄운다.
    # ros2_control 4.x 는 /robot_description 토픽을 받아야 초기화가 시작된다.
    # 동시에 뜨면 FastDDS discovery 타이밍 문제로 토픽을 못 받는 경우가 있다.
    delayed_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[control_node],
        )
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            delayed_control_node,
            controller_spawner,
            joint_trajectory_executor,
            rviz_node,
        ]
    )
