from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_params = PathJoinSubstitution([
        FindPackageShare("omx_perception"),
        "config",
        "camera_params.yaml",
    ])
    camera_intrinsics = PathJoinSubstitution([
        FindPackageShare("omx_perception"),
        "config",
        "camera_intrinsics.yaml",
    ])

    top4_model_path_arg = DeclareLaunchArgument(
        "top4_model_path",
        default_value="/home/kjhz/omx_ws/runs/pose/cube_top_corners_yolov8n_pose/weights/best.pt",
        description="Absolute path to the trained YOLOv8-Pose best.pt file.",
    )
    top4_device_arg = DeclareLaunchArgument(
        "top4_device",
        default_value="0",
        description="Ultralytics inference device for top4_keypoints_node.",
    )
    top4_conf_arg = DeclareLaunchArgument(
        "top4_conf",
        default_value="0.85",
        description="YOLO confidence threshold for top4 keypoint service calls.",
    )
    top4_extra_pythonpath_arg = DeclareLaunchArgument(
        "top4_extra_pythonpath",
        default_value="/home/kjhz/miniconda3/envs/driving/lib/python3.12/site-packages",
        description="Optional site-packages path that contains ultralytics and torch.",
    )

    camera_control = Node(
        package="omx_perception",
        executable="camera_control_node",
        name="camera_control",
        namespace="camera",
        output="screen",
        parameters=[camera_params],
    )

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="both",
        parameters=[camera_params],
        remappings=[
            ("image_raw", "/image/raw"),
            ("image_raw/compressed", "/image/raw/compressed"),
            ("image_raw/compressedDepth", "/image/raw/compressedDepth"),
            ("image_raw/theora", "/image/raw/theora"),
            ("image_raw/zstd", "/image/raw/zstd"),
            ("camera_info", "/camera/info"),
        ],
    )

    top4_keypoints = Node(
        package="omx_perception",
        executable="top4_keypoints_node",
        name="top4_keypoints",
        output="screen",
        parameters=[
            {
                "model_path": LaunchConfiguration("top4_model_path"),
                "image_topic": "/image/raw",
                "annotated_image_topic": "/image/raw/top4_pose",
                "service_name": "/perception/get_top4_keypoints",
                "extra_pythonpath": LaunchConfiguration("top4_extra_pythonpath"),
                "device": ParameterValue(LaunchConfiguration("top4_device"), value_type=str),
                "conf": ParameterValue(LaunchConfiguration("top4_conf"), value_type=float),
                "imgsz": 640,
                "max_det": 20,
            }
        ],
    )

    top4_world_pose = Node(
        package="omx_perception",
        executable="top4_world_pose_node",
        name="top4_world_pose",
        output="screen",
        parameters=[
            {
                "camera_intrinsics_path": camera_intrinsics,
                "top4_service_name": "/perception/get_top4_keypoints",
                "world_service_name": "/perception/get_top4_world_poses",
                "target_frame": "world",
                "camera_frame": "default_cam",
                "cube_size_m": 0.030,
                "output_z_m": 0.015,
                "min_keypoint_confidence": 0.10,
                "top4_timeout_sec": 2.0,
                "keypoint_order": [0, 1, 2, 3],
            }
        ],
    )

    delayed_camera_and_perception = TimerAction(
        period=0.5,
        actions=[camera_node, top4_keypoints, top4_world_pose],
    )

    return LaunchDescription([
        top4_model_path_arg,
        top4_device_arg,
        top4_conf_arg,
        top4_extra_pythonpath_arg,
        camera_control,
        delayed_camera_and_perception,
    ])
