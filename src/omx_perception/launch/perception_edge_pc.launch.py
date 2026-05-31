from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


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
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
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

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for ROS nodes.",
        ),
        PushRosNamespace(LaunchConfiguration("namespace")),
        DeclareLaunchArgument(
            "image_topic",
            default_value="image/raw",
            description="Image topic bridged from the edge robot.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="camera/info",
            description="CameraInfo topic bridged from the edge robot.",
        ),
        DeclareLaunchArgument(
            "box_cup_model_path",
            default_value="runs/pose/box_cup_pose_2class_v3/weights/best.pt",
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
        DeclareLaunchArgument(
            "box_cup_frame_wait_timeout_sec",
            default_value="2.0",
            description="Per-sample timeout while waiting for a new image frame.",
        ),
        yolo_keypoint,
    ])
