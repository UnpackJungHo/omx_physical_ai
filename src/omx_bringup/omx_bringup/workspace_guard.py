#!/usr/bin/env python3
#
# workspace_guard
#
# move_group 의 Planning Scene 에 워크스페이스 z 경계용 충돌 오브젝트
# (floor / ceiling) 를 등록한다. 등록은 /apply_planning_scene 서비스로 수행한다.
# (MoveIt2 의 move_group 은 /collision_object 토픽을 더 이상 구독하지 않으므로
#  토픽 publish 는 무효이다.)
#
# 책임:
#   MoveIt2 planner 가 생성한 trajectory 의 보간 중간 지점이 z 한계 밖으로
#   빠지는 것을 충돌로 차단한다.
#   (goal pose 의 x/y/z 사전 거부는 motion_server 가 담당한다.)
#
# 파라미터 (omx_bringup/config/omx_f/workspace.yaml 단일 소스):
#   workspace.z_min                : floor 상면 높이 [m]
#   workspace.z_max                : 작업 z 상한 [m] (ceiling 기준점)
#   workspace.frame                : Planning Scene 등록 frame
#   workspace.guard_ceiling_margin : ceiling 평면을 z_max 위로 띄우는 여유 [m]
#   workspace.guard_plane_thickness: floor / ceiling 평면 두께 [m]
#   workspace.guard_plane_extent   : 평면의 가로/세로 길이 [m]

import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


_SERVICE_NAME = '/apply_planning_scene'
_SERVICE_WAIT_SEC = 30.0
_SERVICE_CALL_TIMEOUT_SEC = 5.0


class WorkspaceGuard(Node):
    def __init__(self) -> None:
        super().__init__('workspace_guard')

        # workspace 한계 (motion_server 와 공유하는 단일 설정 소스).
        # 기본값은 yaml 미주입 시의 보수적 fallback.
        self.declare_parameter('workspace.z_min', 0.0)
        self.declare_parameter('workspace.z_max', 0.45)
        self.declare_parameter('workspace.frame', 'world')
        self.declare_parameter('workspace.guard_ceiling_margin', 0.05)
        self.declare_parameter('workspace.guard_plane_thickness', 0.04)
        self.declare_parameter('workspace.guard_plane_extent', 2.0)

        z_min = self.get_parameter('workspace.z_min').value
        z_max = self.get_parameter('workspace.z_max').value
        self._frame = self.get_parameter('workspace.frame').value
        margin = self.get_parameter('workspace.guard_ceiling_margin').value
        thickness = self.get_parameter('workspace.guard_plane_thickness').value
        extent = self.get_parameter('workspace.guard_plane_extent').value

        # floor 상면이 z_min 과 일치하도록 box center 를 thickness/2 만큼 아래로 둔다.
        # ceiling 하면이 z_max + margin 에 위치하도록 thickness/2 만큼 위로 띄운다.
        floor_center_z = z_min - thickness / 2.0
        ceiling_center_z = z_max + margin + thickness / 2.0

        self._objects = [
            ('floor',   [extent, extent, thickness], [0.0, 0.0, floor_center_z]),
            ('ceiling', [extent, extent, thickness], [0.0, 0.0, ceiling_center_z]),
        ]

        self._client = self.create_client(ApplyPlanningScene, _SERVICE_NAME)
        self._timer = self.create_timer(0.5, self._try_apply)
        self._applied = False

    def _try_apply(self) -> None:
        if self._applied:
            return
        if not self._client.wait_for_service(timeout_sec=0.0):
            return

        self._timer.cancel()
        scene = PlanningScene()
        scene.is_diff = True
        scene.world = PlanningSceneWorld()
        for obj_id, dims, center in self._objects:
            scene.world.collision_objects.append(
                self._make_box(obj_id, dims, center)
            )

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._client.call_async(request)
        future.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'apply_planning_scene call failed: {exc}')
            self._timer = self.create_timer(1.0, self._try_apply)
            return

        if not response.success:
            self.get_logger().error(
                'apply_planning_scene returned success=false'
            )
            self._timer = self.create_timer(1.0, self._try_apply)
            return

        self._applied = True
        for obj_id, dims, center in self._objects:
            self.get_logger().info(
                f'CollisionObject applied: {obj_id} dims={dims} center={center}'
            )
        self.get_logger().info('Workspace guard: planning scene updated.')

    def _make_box(self, obj_id: str, dims, center) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = self._frame
        obj.id = obj_id
        obj.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(dims)

        pose = Pose()
        pose.position.x = float(center[0])
        pose.position.y = float(center[1])
        pose.position.z = float(center[2])
        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        return obj


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorkspaceGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
