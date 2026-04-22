import sys
import types
import unittest


if "rclpy" not in sys.modules:
    sys.modules["rclpy"] = types.ModuleType("rclpy")

if "rclpy.node" not in sys.modules:
    rclpy_node_stub = types.ModuleType("rclpy.node")

    class DummyNode:
        pass

    rclpy_node_stub.Node = DummyNode
    sys.modules["rclpy.node"] = rclpy_node_stub

if "rcl_interfaces" not in sys.modules:
    sys.modules["rcl_interfaces"] = types.ModuleType("rcl_interfaces")

if "rcl_interfaces.msg" not in sys.modules:
    rcl_interfaces_msg_stub = types.ModuleType("rcl_interfaces.msg")

    class DummyParameterDescriptor:
        def __init__(self, description: str = "") -> None:
            self.description = description

    class DummySetParametersResult:
        def __init__(self, successful: bool = False, reason: str = "") -> None:
            self.successful = successful
            self.reason = reason

    rcl_interfaces_msg_stub.ParameterDescriptor = DummyParameterDescriptor
    rcl_interfaces_msg_stub.SetParametersResult = DummySetParametersResult
    sys.modules["rcl_interfaces.msg"] = rcl_interfaces_msg_stub

from omx_perception.camera_control_node import CameraControlState, build_v4l2_command


class CameraControlCommandTest(unittest.TestCase):
    def test_build_v4l2_command_uses_manual_mode_controls(self) -> None:
        state = CameraControlState(
            video_device="/dev/video2",
            autoexposure=False,
            exposure=120,
            auto_white_balance=False,
            white_balance=4500,
            gain=4,
            brightness=128,
            contrast=32,
            saturation=32,
            sharpness=24,
        )

        command = build_v4l2_command(state)

        self.assertEqual(command[0], "v4l2-ctl")
        self.assertIn("/dev/video2", command)
        self.assertIn("auto_exposure=1", command)
        self.assertIn("exposure_time_absolute=120", command)
        self.assertIn("white_balance_automatic=0", command)
        self.assertIn("white_balance_temperature=4500", command)

    def test_build_v4l2_command_uses_auto_modes_when_enabled(self) -> None:
        state = CameraControlState(
            autoexposure=True,
            auto_white_balance=True,
        )

        command = build_v4l2_command(state)

        self.assertIn("auto_exposure=3", command)
        self.assertIn("white_balance_automatic=1", command)
        self.assertIn("exposure_dynamic_framerate=0", command)


if __name__ == "__main__":
    unittest.main()
