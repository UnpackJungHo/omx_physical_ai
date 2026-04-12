#!/usr/bin/env python3
#
# Phase D-3: workspace_guard
#
# move_group가 시작된 후 Planning Scene에 충돌 오브젝트를 추가한다.
# 하드웨어 보호를 위해 팔이 위험한 경로를 계획하지 못하게 한다.
#
# 충돌 오브젝트:
#   floor  : z=0 평면 (테이블 상면, 로봇 기저부 아래로 진입 차단)
#   ceiling: z=0.5 평면 (작업 높이 상한, 과도한 신장 차단)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


# 충돌 오브젝트 정의
# (id, box_dims, center_xyz)
_OBJECTS = [
    # 테이블 상면 슬래브: top face at z=0
    # OMX 기저부(link0)가 z=0에 있으므로 아래 방향 이동을 차단한다.
    ('floor', [2.0, 2.0, 0.04], [0.0, 0.0, -0.02]),
    # 작업 영역 상한 슬래브: bottom face at z=0.5
    # 과도한 고신장 자세와 천장 충돌을 차단한다.
    ('ceiling', [2.0, 2.0, 0.04], [0.0, 0.0, 0.52]),
]

# move_group 준비까지 대기하는 초기 지연 (초)
_STARTUP_DELAY_SEC = 3.0


class WorkspaceGuard(Node):
    """Planning Scene에 workspace 경계 충돌 오브젝트를 등록하는 노드."""

    def __init__(self) -> None:
        super().__init__('workspace_guard')
        self._pub = self.create_publisher(
            CollisionObject, '/collision_object', 10
        )
        # move_group 준비 대기 후 1회 실행
        self._timer = self.create_timer(_STARTUP_DELAY_SEC, self._add_objects)

    def _add_objects(self) -> None:
        self._timer.cancel()
        for obj_id, dims, center in _OBJECTS:
            self._pub.publish(self._make_box(obj_id, dims, center))
            self.get_logger().info(
                f'CollisionObject added: {obj_id} '
                f'dims={dims} center={center}'
            )
        self.get_logger().info('Workspace guard: all collision objects published.')

    @staticmethod
    def _make_box(
        obj_id: str,
        dims: list[float],
        center: list[float],
    ) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.id = obj_id
        obj.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dims

        pose = Pose()
        pose.position.x = center[0]
        pose.position.y = center[1]
        pose.position.z = center[2]
        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        return obj


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorkspaceGuard()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
