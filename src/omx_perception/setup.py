from glob import glob
import os

from setuptools import find_packages, setup


package_name = "omx_perception"
source_dir = os.path.dirname(os.path.realpath(__file__))


def source_glob(pattern: str) -> list[str]:
    return [
        os.path.relpath(path, os.getcwd())
        for path in glob(os.path.join(source_dir, pattern))
    ]


def source_path(*parts: str) -> str:
    return os.path.relpath(os.path.join(source_dir, *parts), os.getcwd())


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [source_path("resource", package_name)],
        ),
        (f"share/{package_name}", [source_path("package.xml")]),
        (
            os.path.join("share", package_name, "config"),
            source_glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            source_glob("launch/*.launch.py"),
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
    entry_points={"console_scripts": [
            "box_cup_keypoint_node = omx_perception.box_cup_keypoint_node:main",
            "camera_supervisor = omx_perception.camera_supervisor:main",
        ]},
)
