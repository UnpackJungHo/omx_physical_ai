from glob import glob
import os

from setuptools import find_packages, setup


package_name = "omx_perception"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kjhz",
    maintainer_email="kjhgfd6632@gmail.com",
    description="OMX perception node — color-based block detection via OpenCV",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "camera_control_node = omx_perception.camera_control_node:main",
            "detector_node = omx_perception.detector_node:main",
            "tracker_node = omx_perception.tracker_node:main",
            "get_block_poses_server = omx_perception.get_block_poses_server:main",
            "color_calibrator = omx_perception.color_calibrator:main",
        ],
    },
)
