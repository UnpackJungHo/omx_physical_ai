"""Convert pixel coordinates to 3D world coordinates.

Strategy (flat-surface assumption):
  - No depth camera → assume all blocks lie on a known table plane (z = TABLE_Z).
  - Use camera intrinsics to back-project pixel (u, v) to a ray.
  - Intersect ray with the table plane to get (X, Y, Z) in the camera frame.
  - The caller is responsible for tf2 transform to the robot base frame.

If a real depth camera is added later, replace `pixel_to_ray_plane` with
a depth-lookup variant without changing the service API.
"""

from typing import Optional, Tuple

import numpy as np
from sensor_msgs.msg import CameraInfo


class PixelTo3D:
    """Back-projects pixel coordinates assuming a flat table plane."""

    def __init__(
        self,
        table_z_in_camera: float = 0.30,  # metres from camera to table surface
    ) -> None:
        self._table_z = table_z_in_camera
        self._K: Optional[np.ndarray] = None  # 3x3 intrinsic matrix

    def update_camera_info(self, info: CameraInfo) -> None:
        """Store camera intrinsics from a CameraInfo message."""
        self._K = np.array(info.k, dtype=float).reshape(3, 3)

    @property
    def is_ready(self) -> bool:
        return self._K is not None

    def pixel_to_camera_point(
        self, u: float, v: float
    ) -> Tuple[float, float, float]:
        """Return (x, y, z) in the camera optical frame.

        Raises RuntimeError if camera info has not been set.
        """
        if self._K is None:
            raise RuntimeError("CameraInfo not received yet.")

        fx = self._K[0, 0]
        fy = self._K[1, 1]
        cx = self._K[0, 2]
        cy = self._K[1, 2]

        # Back-project pixel to normalised ray
        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy

        # Scale ray so that the z component == table_z
        z = self._table_z
        x = x_norm * z
        y = y_norm * z
        return (x, y, z)
