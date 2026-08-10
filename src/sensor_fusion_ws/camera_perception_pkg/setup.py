import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'camera_perception_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kil',
    maintainer_email='ros2kil105@gmail.com',
    description='YOLOv8-based camera object detection node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_node = camera_perception_pkg.yolov8_node:main',
            'image_publisher_node = camera_perception_pkg.image_publisher_node:main',
            'bird_eye_node = camera_perception_pkg.bird_eye_node:main',
        ],
    },
)
