from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
            description="Minimum interval between usb_cam restarts.",
        ),
        camera_supervisor,
        world_pose,
    ])
