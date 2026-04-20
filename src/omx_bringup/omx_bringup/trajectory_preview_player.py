from __future__ import annotations

import json
import os
import signal
import time
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _float_list(values: list[float] | None) -> list[float]:
    return [float(v) for v in values] if values else []


def _duration_fields(duration_sec: float) -> tuple[int, int]:
    secs = int(duration_sec)
    nanosec = int(round((duration_sec - secs) * 1_000_000_000.0))
    if nanosec >= 1_000_000_000:
        secs += 1
        nanosec -= 1_000_000_000
    return secs, nanosec


class TrajectoryPreviewPlayer(Node):
    def __init__(self, runtime_dir: str) -> None:
        super().__init__("trajectory_preview_player")
        self._runtime_dir = Path(runtime_dir)
        self._runtime_dir.mkdir(parents=True, exist_ok=True)

        self._pid_file = self._runtime_dir / "player.pid"
        self._ready_file = self._runtime_dir / "ready.flag"
        self._request_file = self._runtime_dir / "request.json"
        self._accepted_file = self._runtime_dir / "accepted.txt"
        self._completed_file = self._runtime_dir / "completed.txt"
        self._status_file = self._runtime_dir / "status.txt"

        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self._last_completed_request_id = -1

        self._pid_file.write_text(str(os.getpid()))
        self._accepted_file.unlink(missing_ok=True)
        self._completed_file.unlink(missing_ok=True)
        self._ready_file.unlink(missing_ok=True)
        self._write_status("starting")

    def _write_status(self, status: str) -> None:
        self._status_file.write_text(status + "\n")

    def _wait_for_server(self) -> bool:
        self.get_logger().info(
            "Waiting for preview trajectory server on /arm_controller/follow_joint_trajectory"
        )
        while rclpy.ok():
            if self._client.wait_for_server(timeout_sec=1.0):
                self._ready_file.write_text("ready\n")
                self._write_status("ready")
                self.get_logger().info("Preview trajectory server ready")
                return True
            self.get_logger().info("Preview trajectory server not ready yet")
        return False

    def _load_request(self) -> dict | None:
        if not self._request_file.is_file():
            return None
        try:
            payload = json.loads(self._request_file.read_text())
        except json.JSONDecodeError:
            self.get_logger().warn("Ignoring malformed preview request file")
            return None

        request_id = int(payload.get("request_id", -1))
        trajectory_file = Path(str(payload.get("trajectory_file", "")))
        if request_id <= self._last_completed_request_id:
            return None
        if not trajectory_file.is_file():
            self.get_logger().warn(f"Preview trajectory file missing: {trajectory_file}")
            return None
        payload["request_id"] = request_id
        payload["trajectory_file"] = trajectory_file
        return payload

    def _load_trajectory(self, trajectory_file: Path) -> JointTrajectory:
        payload = json.loads(trajectory_file.read_text())
        trajectory = JointTrajectory()
        trajectory.joint_names = [str(name) for name in payload.get("joint_names", [])]

        for entry in payload.get("points", []):
            point = JointTrajectoryPoint()
            point.positions = _float_list(entry.get("positions"))
            point.velocities = _float_list(entry.get("velocities"))
            point.accelerations = _float_list(entry.get("accelerations"))

            secs, nanosec = _duration_fields(float(entry.get("time_from_start_sec", 0.0)))
            point.time_from_start.sec = secs
            point.time_from_start.nanosec = nanosec
            trajectory.points.append(point)

        return trajectory

    def _execute_request(self, request: dict) -> None:
        request_id = int(request["request_id"])
        trajectory_file = Path(request["trajectory_file"])
        trajectory = self._load_trajectory(trajectory_file)

        if not trajectory.joint_names or not trajectory.points:
            self.get_logger().error("preview trajectory payload is empty")
            self._completed_file.write_text(f"{request_id} error empty_trajectory\n")
            self._write_status("idle")
            self._last_completed_request_id = request_id
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance.sec = 2

        self._accepted_file.write_text(f"{request_id}\n")
        self._write_status(f"executing {request_id}")
        self.get_logger().info(
            f"Executing preview request {request_id} from {trajectory_file}"
        )

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("preview trajectory goal rejected")
            self._completed_file.write_text(f"{request_id} error rejected\n")
            self._write_status("idle")
            self._last_completed_request_id = request_id
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            self.get_logger().error("preview trajectory result was not returned")
            self._completed_file.write_text(f"{request_id} error no_result\n")
            self._write_status("idle")
            self._last_completed_request_id = request_id
            return

        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                "preview execution failed: code=%d error='%s'"
                % (result.result.error_code, result.result.error_string)
            )
            self._completed_file.write_text(
                f"{request_id} error {result.result.error_code} {result.result.error_string}\n"
            )
        else:
            self.get_logger().info(f"Preview trajectory {request_id} completed")
            self._completed_file.write_text(f"{request_id} ok\n")

        self._write_status("idle")
        self._last_completed_request_id = request_id

    def run(self) -> int:
        if not self._wait_for_server():
            return 1

        self.get_logger().info(f"Preview runtime directory: {self._runtime_dir}")
        while rclpy.ok():
            request = self._load_request()
            if request is not None:
                self._execute_request(request)
                continue
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.1)
        return 0

    def cleanup(self) -> None:
        self._write_status("stopped")
        self._ready_file.unlink(missing_ok=True)
        self._pid_file.unlink(missing_ok=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("trajectory_preview_player_args")
    node.declare_parameter("runtime_dir", "/tmp/omx_preview_runtime")
    runtime_dir = str(node.get_parameter("runtime_dir").value)
    node.destroy_node()

    player = TrajectoryPreviewPlayer(runtime_dir)

    def _handle_signal(signum, frame) -> None:  # noqa: ARG001
        player.get_logger().info("Preview player shutdown requested")
        player.cleanup()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        exit_code = player.run()
    finally:
        player.cleanup()
        player.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)
