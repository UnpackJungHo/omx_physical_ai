import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("omx_perception")
    cfg = os.path.join(pkg_share, "config", "perception.yaml")
    cam_cfg = os.path.join(pkg_share, "config", "camera_params.yaml")
    intrinsics = os.path.join(pkg_share, "config", "camera_intrinsics.yaml")

    # Step 1: fix exposure (auto → manual 120)
    set_exposure = ExecuteProcess(
        cmd=[
            "v4l2-ctl", "-d", "/dev/video0",
            "-c", "auto_exposure=1",
            "-c", "exposure_time_absolute=120",
            "-c", "exposure_dynamic_framerate=0",
        ],
        output="screen",
    )

    # Step 2: fix white balance (4500K, fluorescent)
    set_white_balance = ExecuteProcess(
        cmd=[
            "v4l2-ctl", "-d", "/dev/video0",
            "-c", "white_balance_automatic=0",
            "-c", "white_balance_temperature=4500",
        ],
        output="screen",
    )

    # Step 3: camera node — starts only after WB is set
    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        namespace="camera",
        parameters=[cam_cfg, {
            "camera_info_url": f"file://{intrinsics}",
        }],
        remappings=[
            ("image_raw", "/image/raw"),
            ("camera_info", "/camera_info"),
        ],
        output="screen",
    )

    color_ranges = os.path.join(pkg_share, "config", "color_ranges.yaml")

    detector = Node(
        package="omx_perception",
        executable="detector_node",
        name="detector_node",
        parameters=[cfg, {"color_ranges_path": color_ranges}],
        output="screen",
    )

    tracker = Node(
        package="omx_perception",
        executable="tracker_node",
        name="tracker_node",
        parameters=[cfg],
        output="screen",
    )

    server = Node(
        package="omx_perception",
        executable="get_block_poses_server",
        name="get_block_poses_server",
        parameters=[cfg],
        output="screen",
    )

    # Chain: exposure done → WB → camera + perception nodes
    start_wb_after_exposure = RegisterEventHandler(
        OnProcessExit(
            target_action=set_exposure,
            on_exit=[set_white_balance],
        )
    )

    start_nodes_after_wb = RegisterEventHandler(
        OnProcessExit(
            target_action=set_white_balance,
            on_exit=[camera_node, detector, tracker, server],
        )
    )

    return LaunchDescription([
        set_exposure,
        start_wb_after_exposure,
        start_nodes_after_wb,
    ])
