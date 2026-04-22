import math
import sys
import types
import unittest


def _ensure_module(name: str) -> types.ModuleType:
    module = sys.modules.get(name)
    if module is None:
        module = types.ModuleType(name)
        sys.modules[name] = module
    return module


if "rclpy" not in sys.modules:
    sys.modules["rclpy"] = types.ModuleType("rclpy")

if "rclpy.node" not in sys.modules:
    rclpy_node_stub = types.ModuleType("rclpy.node")

    class DummyNode:
        pass

    rclpy_node_stub.Node = DummyNode
    sys.modules["rclpy.node"] = rclpy_node_stub

builtin_msg_stub = _ensure_module("builtin_interfaces.msg")


class DummyTime:
    def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
        self.sec = sec
        self.nanosec = nanosec


builtin_msg_stub.Time = DummyTime
_ensure_module("builtin_interfaces")

geometry_msg_stub = _ensure_module("geometry_msgs.msg")


class DummyPoint:
    def __init__(self, x=0.0, y=0.0, z=0.0) -> None:
        self.x = x
        self.y = y
        self.z = z


class DummyQuaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class DummyPose:
    def __init__(self, position=None, orientation=None) -> None:
        self.position = position
        self.orientation = orientation


class DummyPoseStamped:
    def __init__(self, header=None, pose=None) -> None:
        self.header = header
        self.pose = pose


geometry_msg_stub.Point = DummyPoint
geometry_msg_stub.Quaternion = DummyQuaternion
geometry_msg_stub.Pose = DummyPose
geometry_msg_stub.PoseStamped = DummyPoseStamped
_ensure_module("geometry_msgs")

std_msg_stub = _ensure_module("std_msgs.msg")


class DummyHeader:
    def __init__(self, frame_id="", stamp=None) -> None:
        self.frame_id = frame_id
        self.stamp = stamp


class DummyString:
    pass


std_msg_stub.Header = DummyHeader
std_msg_stub.String = DummyString
_ensure_module("std_msgs")

omx_msg_stub = _ensure_module("omx_interfaces.msg")
omx_srv_stub = _ensure_module("omx_interfaces.srv")


class DummyBlockPose:
    pass


class DummyGetBlockPoses:
    class Request:
        pass

    class Response:
        pass


omx_msg_stub.BlockPose = DummyBlockPose
omx_srv_stub.GetBlockPoses = DummyGetBlockPoses
_ensure_module("omx_interfaces")


from omx_perception.get_block_poses_server import _canonicalize_cube_yaw, _yaw_to_quaternion
from omx_perception.get_block_poses_server import _omx_top_down_grasp_quaternion


def _quat_to_rotation_matrix(quat) -> list[list[float]]:
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def _apply_rotation(matrix: list[list[float]], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(
        sum(matrix[row][col] * vector[col] for col in range(3))
        for row in range(3)
    )


class GetBlockPosesOrientationTest(unittest.TestCase):
    def test_canonicalize_cube_yaw_folds_into_minus45_to_45_deg(self) -> None:
        for deg in (-170.0, -95.0, -50.0, -45.0, -10.0, 0.0, 12.0, 45.0, 91.0, 179.0):
            yaw = math.radians(deg)
            folded = _canonicalize_cube_yaw(yaw)
            self.assertGreaterEqual(folded, -math.pi / 4.0 - 1e-9)
            self.assertLessEqual(folded, math.pi / 4.0 + 1e-9)

    def test_yaw_to_quaternion_matches_z_axis_rotation(self) -> None:
        yaw = math.radians(30.0)
        quat = _yaw_to_quaternion(yaw)
        self.assertAlmostEqual(quat.x, 0.0)
        self.assertAlmostEqual(quat.y, 0.0)
        self.assertAlmostEqual(quat.z, math.sin(yaw / 2.0))
        self.assertAlmostEqual(quat.w, math.cos(yaw / 2.0))

    def test_omx_top_down_grasp_quaternion_points_tool_x_down(self) -> None:
        quat = _omx_top_down_grasp_quaternion(0.0)
        rotation = _quat_to_rotation_matrix(quat)
        tool_x_world = _apply_rotation(rotation, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(tool_x_world[0], 0.0, places=6)
        self.assertAlmostEqual(tool_x_world[1], 0.0, places=6)
        self.assertAlmostEqual(tool_x_world[2], -1.0, places=6)

    def test_omx_top_down_grasp_quaternion_rotates_in_plane_about_approach_axis(self) -> None:
        yaw = math.radians(30.0)
        quat = _omx_top_down_grasp_quaternion(yaw)
        rotation = _quat_to_rotation_matrix(quat)

        tool_x_world = _apply_rotation(rotation, (1.0, 0.0, 0.0))
        tool_z_world = _apply_rotation(rotation, (0.0, 0.0, 1.0))

        self.assertAlmostEqual(tool_x_world[0], 0.0, places=6)
        self.assertAlmostEqual(tool_x_world[1], 0.0, places=6)
        self.assertAlmostEqual(tool_x_world[2], -1.0, places=6)
        self.assertAlmostEqual(tool_z_world[0], math.cos(yaw), places=6)
        self.assertAlmostEqual(tool_z_world[1], -math.sin(yaw), places=6)
        self.assertAlmostEqual(tool_z_world[2], 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
