#!/usr/bin/env python3
#
# workspace_guard
#
# move_group의 Planning Scene 에 워크스페이스 경계용 충돌 오브젝트를
# 등록한다. 등록은 /apply_planning_scene 서비스로 수행한다.
# (MoveIt2 의 move_group 은 /collision_object 토픽을 더 이상 구독하지
# 않으므로 토픽 publish 는 무효이다.)
#
# 충돌 오브젝트:
#   floor  : z=0 평면 (테이블 상면 슬래브, 기저부 아래 진입 차단)
#   ceiling: z=0.5 평면 (작업 높이 상한, 천장 진입 차단)

import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


_OBJECTS = [
    ('floor',   [2.0, 2.0, 0.04], [0.0, 0.0, -0.02]),
    ('ceiling', [2.0, 2.0, 0.04], [0.0, 0.0,  0.52]),
]

_SERVICE_NAME = '/apply_planning_scene'
_SERVICE_WAIT_SEC = 30.0
_SERVICE_CALL_TIMEOUT_SEC = 5.0


class WorkspaceGuard(Node):
    def __init__(self) -> None:
        super().__init__('workspace_guard')
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
        for obj_id, dims, center in _OBJECTS:
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
        for obj_id, dims, center in _OBJECTS:
            self.get_logger().info(
                f'CollisionObject applied: {obj_id} dims={dims} center={center}'
            )
        self.get_logger().info('Workspace guard: planning scene updated.')

    @staticmethod
    def _make_box(obj_id: str, dims, center) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = 'world'
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
