from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
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
        ),
    ])
