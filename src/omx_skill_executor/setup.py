from glob import glob
import os
from setuptools import find_packages, setup


package_name = "omx_skill_executor"


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
    description="OMX skill executor — PickPlace first skill (Python prototype)",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "pick_place_server = omx_skill_executor.pick_place_server:main",
        ],
    },
)
