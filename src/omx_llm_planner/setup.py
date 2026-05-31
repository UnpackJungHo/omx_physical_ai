from glob import glob
import os
from setuptools import find_packages, setup


package_name = "omx_llm_planner"


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
    description="OMX LLM planner — 자연어 명령을 로봇 액션 plan 으로 변환/실행",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "planner_node = omx_llm_planner.planner_node:main",
        ],
    },
)
