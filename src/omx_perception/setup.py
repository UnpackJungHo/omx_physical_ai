import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'omx_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kjhz',
    maintainer_email='kjhgfd6632@gmail.com',
    description='OMX perception node — color-based block detection via OpenCV',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'block_detection_node = omx_perception.block_detection_node:main',
        ],
    },
)
