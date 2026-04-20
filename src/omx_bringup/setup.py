from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'omx_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일 설치
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # config 파일 설치 (udev rules 등, 디렉토리 제외)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.rules') + glob('config/*.yaml') + glob('config/*.json')),
        # omx_f 전용 config 파일 설치
        (os.path.join('share', package_name, 'config', 'omx_f'),
            glob('config/omx_f/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='OMX Physical AI 데모 bringup 패키지',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'workspace_guard = omx_bringup.workspace_guard:main',
            'trajectory_preview_player = omx_bringup.trajectory_preview_player:main',
        ],
    },
)
