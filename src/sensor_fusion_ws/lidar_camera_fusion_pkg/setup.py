import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'lidar_camera_fusion_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kil',
    maintainer_email='ros2kil105@gmail.com',
    description='Projects LiDAR scan points into the camera image and estimates distance for YOLO detections',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_fusion_node = lidar_camera_fusion_pkg.image_fusion_node:main',
            'sensor_fusion_node = lidar_camera_fusion_pkg.sensor_fusion_node:main',
        ],
    },
)
