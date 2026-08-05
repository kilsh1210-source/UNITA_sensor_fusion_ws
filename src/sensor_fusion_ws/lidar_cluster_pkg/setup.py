import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'lidar_cluster_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kil',
    maintainer_email='ros2kil105@gmail.com',
    description='Simple sequential Euclidean clustering of LaserScan points with RViz marker visualization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_cluster_node = lidar_cluster_pkg.scan_cluster_node:main',
            'l_shape_node = lidar_cluster_pkg.l_shape_node:main',
        ],
    },
)
