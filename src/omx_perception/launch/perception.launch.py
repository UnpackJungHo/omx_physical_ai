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

    box_cup_model_path_arg = DeclareLaunchArgument(
        "box_cup_model_path",
        default_value="/home/kjhz/omx_ws/runs/pose/box_cup_pose_2class_96/weights/best.pt",
        description="Absolute path to the trained YOLOv8-Pose 2-class best.pt.",
    )
    box_cup_device_arg = DeclareLaunchArgument(
        "box_cup_device",
        default_value="0",
        description="Ultralytics inference device for box_cup_pose_node.",
    )
    box_cup_conf_arg = DeclareLaunchArgument(
        "box_cup_conf",
        default_value="0.85",
        description="YOLO confidence threshold for box_cup_pose_node.",
    )
    box_cup_extra_pythonpath_arg = DeclareLaunchArgument(
        "box_cup_extra_pythonpath",
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

    box_cup_pose = Node(
        package="omx_perception",
        executable="box_cup_pose_node",
        name="box_cup_pose",
        output="screen",
        parameters=[
            {
                "model_path": LaunchConfiguration("box_cup_model_path"),
                "image_topic": "/image/raw",
                "annotated_image_topic": "/image/raw/box_cup_pose",
                "service_name": "/perception/get_box_cup_keypoints",
                "extra_pythonpath": LaunchConfiguration("box_cup_extra_pythonpath"),
                "device": ParameterValue(LaunchConfiguration("box_cup_device"), value_type=str),
                "conf": ParameterValue(LaunchConfiguration("box_cup_conf"), value_type=float),
                "imgsz": 640,
                "max_det": 20,
            }
        ],
    )

    box_cup_world_pose = Node(
        package="omx_perception",
        executable="box_cup_world_pose_node",
        name="box_cup_world_pose",
        output="screen",
        parameters=[
            {
                "camera_intrinsics_path": camera_intrinsics,
                "keypoints_service_name": "/perception/get_box_cup_keypoints",
                "world_service_name": "/perception/get_box_cup_world_poses",
                "target_frame": "world",
                "camera_frame": "default_cam",
                "cube_size_m": 0.030,
                "box_output_z_m": 0.015,
                "cup_radius_m": 0.07,
                "cup_height_m": 0.08,
                "cup_output_z_m": 0.08,
                "min_keypoint_confidence": 0.10,
                "keypoints_timeout_sec": 2.0,
                "keypoint_order": [0, 1, 2, 3],
            }
        ],
    )

    delayed_camera_and_perception = TimerAction(
        period=0.5,
        actions=[camera_node, box_cup_pose, box_cup_world_pose],
    )

    return LaunchDescription([
        box_cup_model_path_arg,
        box_cup_device_arg,
        box_cup_conf_arg,
        box_cup_extra_pythonpath_arg,
        camera_control,
        delayed_camera_and_perception,
    ])
