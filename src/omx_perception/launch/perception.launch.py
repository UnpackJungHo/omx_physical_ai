import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("omx_perception")
    cfg = os.path.join(pkg_share, "config", "perception.yaml")
    cam_cfg = os.path.join(pkg_share, "config", "camera_params.yaml")
    intrinsics = os.path.join(pkg_share, "config", "camera_intrinsics.yaml")

    camera_control = Node(
        package="omx_perception",
        executable="camera_control_node",
        name="camera_control",
        namespace="camera",
        parameters=[cam_cfg],
        output="screen",
    )

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

    color_prototypes = os.path.join(pkg_share, "config", "color_prototypes.yaml")

    detector = Node(
        package="omx_perception",
        executable="detector_node",
        name="detector_node",
        parameters=[cfg, {"color_prototypes_path": color_prototypes}],
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

    delayed_start = TimerAction(
        period=0.5,
        actions=[camera_node, detector, tracker, server],
    )

    return LaunchDescription([
        camera_control,
        delayed_start,
    ])
