from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from omx_perception.camera_geometry import (
    CameraIntrinsics,
    FrameConfig,
    Plane,
    TransformSnapshot,
    intersect_ray_with_plane,
    ray_in_reference_frame,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Project a clicked image pixel into a reference-frame ray and table intersection.",
    )
    parser.add_argument("--camera-info-yaml", required=True, help="camera_calibration YAML file")
    parser.add_argument("--config-yaml", required=True, help="omx_perception camera_extrinsics.yaml")
    parser.add_argument(
        "--translation",
        required=True,
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Current TF translation of reference_frame -> camera_frame",
    )
    parser.add_argument(
        "--quaternion",
        required=True,
        nargs=4,
        type=float,
        metavar=("X", "Y", "Z", "W"),
        help="Current TF quaternion (xyzw) of reference_frame -> camera_frame",
    )
    parser.add_argument("--pixel-u", required=True, type=float, help="Image x coordinate in pixels")
    parser.add_argument("--pixel-v", required=True, type=float, help="Image y coordinate in pixels")
    return parser


def format_vector(label: str, values) -> str:
    rounded = ", ".join(f"{value:.6f}" for value in values)
    return f"{label}: [{rounded}]"


def main() -> int:
    args = build_parser().parse_args()

    intrinsics = CameraIntrinsics.from_camera_info_yaml(Path(args.camera_info_yaml))
    frame_config = FrameConfig.from_yaml(Path(args.config_yaml))
    plane = Plane.from_yaml(Path(args.config_yaml))
    transform = TransformSnapshot(
        reference_frame=frame_config.reference_frame,
        translation_m=np.asarray(args.translation, dtype=float),
        rotation_xyzw=np.asarray(args.quaternion, dtype=float),
    )

    origin_reference, direction_reference = ray_in_reference_frame(
        intrinsics=intrinsics,
        transform=transform,
        pixel_u=args.pixel_u,
        pixel_v=args.pixel_v,
    )
    intersection = intersect_ray_with_plane(origin_reference, direction_reference, plane)

    print(f"camera_frame: {frame_config.camera_frame}")
    print(f"reference_frame: {frame_config.reference_frame}")
    print(format_vector("ray_origin_reference_m", origin_reference))
    print(format_vector("ray_direction_reference", direction_reference))
    if intersection is None:
        print("table_intersection_reference_m: no_intersection")
    else:
        print(format_vector("table_intersection_reference_m", intersection))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
