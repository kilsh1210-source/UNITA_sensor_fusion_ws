import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'sensor_fusion_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kil',
    maintainer_email='ros2kil105@gmail.com',
    description="여러 서브시스템(퓨전, L-shape, 추후 bird's eye view 등)을 한 번에 묶어서 실행하는 "
                "최상위 launch 모음 패키지",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
