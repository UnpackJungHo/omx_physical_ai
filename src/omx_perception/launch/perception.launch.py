from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # usb_cam 은 자체 reconnect 가 없어 arm 장착 카메라의 재enumeration 에 취약하다.
    # camera_supervisor 가 usb_cam 을 자식 프로세스로 띄우고 image/raw staleness 를
    # 감시하여 끊기면 안정 by-id 경로로 재시작한다. (omx_perception/camera_supervisor.py)
    camera_supervisor = Node(
        package="omx_perception",
        executable="camera_supervisor",
        name="camera_supervisor",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "camera_name_match": LaunchConfiguration("camera_name_match"),
                "video_device": LaunchConfiguration("video_device"),
                "video_sysfs_dir": LaunchConfiguration("video_sysfs_dir"),
                "camera_params_path": PathJoinSubstitution([
                    FindPackageShare("omx_perception"),
                    "config", "camera_params.yaml",
                ]),
                "image_topic": "image/raw",
                "stale_timeout_sec": ParameterValue(
                    LaunchConfiguration("camera_stale_timeout_sec"), value_type=float
                ),
                "startup_grace_sec": ParameterValue(
                    LaunchConfiguration("camera_startup_grace_sec"), value_type=float
                ),
                "restart_min_interval_sec": ParameterValue(
                    LaunchConfiguration("camera_restart_min_interval_sec"), value_type=float
                ),
            }
        ],
    )

    yolo_keypoint = Node(
        package="omx_perception",
        executable="box_cup_keypoint_node",
        name="box_cup_keypoint",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "model_path": LaunchConfiguration("box_cup_model_path"),
                "image_topic": "image/raw",
                "output_dir": LaunchConfiguration("box_cup_output_dir"),
                "extra_pythonpath": LaunchConfiguration("box_cup_extra_pythonpath"),
                "device": ParameterValue(LaunchConfiguration("box_cup_device"), value_type=str),
                "imgsz": 640,
                "conf": ParameterValue(LaunchConfiguration("box_cup_conf"), value_type=float),
                "max_det": 20,
                "publish_debug": ParameterValue(
                    LaunchConfiguration("box_cup_publish_debug"), value_type=bool
                ),
                "frame_wait_timeout_sec": ParameterValue(
                    LaunchConfiguration("box_cup_frame_wait_timeout_sec"), value_type=float
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
            "namespace",
            default_value="",
            description="Namespace for ROS nodes.",
        ),
        PushRosNamespace(LaunchConfiguration("namespace")),
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
            default_value="runs/pose/box_cup_pose_2class_v3/weights/best.pt", # Yolo 모델 경로
            description="YOLOv8-Pose best.pt for box/cup keypoint detection.",
        ),
        DeclareLaunchArgument(
            "box_cup_output_dir",
            default_value="tmp_kjh/box_cup_pose_image", # 스냅샷 저장 경로
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
        DeclareLaunchArgument(
            "box_cup_frame_wait_timeout_sec",
            default_value="2.0",
            description="Per-sample timeout while waiting for a new image frame.",
        ),
        DeclareLaunchArgument(
            "camera_stale_timeout_sec",
            default_value="3.0",
            description="Restart usb_cam if image/raw delivers no frame for this long.",
        ),
        DeclareLaunchArgument(
            "camera_startup_grace_sec",
            default_value="10.0",
            description="Grace period after (re)start before staleness is judged.",
        ),
        DeclareLaunchArgument(
            "camera_restart_min_interval_sec",
            default_value="5.0",
            description="Minimum interval between usb_cam restarts (crash-loop guard).",
        ),
        camera_supervisor,
        yolo_keypoint,
        world_pose,
    ])
