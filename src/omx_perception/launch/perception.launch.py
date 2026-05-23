from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from omx_perception.camera_device import (
    apply_v4l2_controls,
    load_camera_parameters,
    resolve_video_device,
)


def _launch_camera(context, *args, **kwargs):
    camera = resolve_video_device(
        LaunchConfiguration("camera_name_match").perform(context),
        explicit_device=LaunchConfiguration("video_device").perform(context),
        sysfs_dir=LaunchConfiguration("video_sysfs_dir").perform(context),
    )
    camera_params = PathJoinSubstitution([
        FindPackageShare("omx_perception"),
        "config",
        "camera_params.yaml",
    ])
    camera_params_path = camera_params.perform(context)
    applied_controls = apply_v4l2_controls(
        camera.path,
        load_camera_parameters(camera_params_path),
    )

    return [
        LogInfo(msg=f"omx_perception selected camera: {camera.name} -> {camera.path}"),
        LogInfo(msg=f"omx_perception applied camera controls: {', '.join(applied_controls)}"),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            output="screen",
            parameters=[
                camera_params_path,
                {"video_device": camera.path},
                {"image_raw.enable_pub_plugins": ["image_transport/raw"]},
            ],
            remappings=[
                ("image_raw", "/image/raw"),
                ("camera_info", "/camera/info"),
            ],
        ),
    ]


def generate_launch_description():
    yolo_keypoint = Node(
        package="omx_perception",
        executable="box_cup_keypoint_node",
        name="box_cup_keypoint",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "model_path": LaunchConfiguration("box_cup_model_path"),
                "image_topic": "/image/raw",
                "output_dir": LaunchConfiguration("box_cup_output_dir"),
                "extra_pythonpath": LaunchConfiguration("box_cup_extra_pythonpath"),
                "device": ParameterValue(LaunchConfiguration("box_cup_device"), value_type=str),
                "imgsz": 640,
                "conf": ParameterValue(LaunchConfiguration("box_cup_conf"), value_type=float),
                "max_det": 20,
                "publish_debug": ParameterValue(
                    LaunchConfiguration("box_cup_publish_debug"), value_type=bool
                ),
            }
        ],
    )

    world_pose = Node(
        package="omx_perception_world_pose",
        executable="box_cup_world_pose_node",
        name="box_cup_world_pose",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("omx_perception_world_pose"),
                "config", "box_cup_world_pose.yaml",
            ]),
            {
                "camera_intrinsics_path": PathJoinSubstitution([
                    FindPackageShare("omx_perception"),
                    "config", "camera_intrinsics.yaml",
                ]),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_name_match",
            default_value="Innomaker",
            description="Case-insensitive text used to select a connected V4L2 camera by name.",
        ),
        DeclareLaunchArgument(
            "video_device",
            default_value="",
            description="Optional explicit V4L2 capture device path. Empty means auto-detect by camera name.",
        ),
        DeclareLaunchArgument(
            "video_sysfs_dir",
            default_value="/sys/class/video4linux",
            description="Linux sysfs directory used to enumerate connected V4L2 cameras.",
        ),
        DeclareLaunchArgument(
            "box_cup_model_path",
            default_value="runs/pose/box_cup_pose_2class_201/weights/best.pt",
            description="YOLOv8-Pose best.pt for box/cup keypoint detection.",
        ),
        DeclareLaunchArgument(
            "box_cup_output_dir",
            default_value="tmp_kjh/box_cup_pose_image",
            description="Directory where p-key YOLO snapshot images are saved.",
        ),
        DeclareLaunchArgument(
            "box_cup_extra_pythonpath",
            default_value="/home/kjhz/miniconda3/envs/driving/lib/python3.12/site-packages",
            description="Optional site-packages path that contains ultralytics and torch.",
        ),
        DeclareLaunchArgument(
            "box_cup_device",
            default_value="0",
            description="Ultralytics inference device. Use 0 for GPU or cpu for CPU.",
        ),
        DeclareLaunchArgument(
            "box_cup_conf",
            default_value="0.25",
            description="YOLO confidence threshold for keypoint prediction.",
        ),
        DeclareLaunchArgument(
            "box_cup_publish_debug",
            default_value="false",
            description="Publish annotated debug image to /perception/debug_image.",
        ),
        OpaqueFunction(function=_launch_camera),
        yolo_keypoint,
        world_pose,
    ])
