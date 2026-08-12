from setuptools import find_packages, setup

package_name = 'decision_making_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kil',
    maintainer_email='ros2kil105@gmail.com',
    description='Lattice path planning + pure pursuit/PD motion planning',
    license='GPL-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner_node = decision_making_pkg.path_planner_node:main',
            'motion_planner_node = decision_making_pkg.motion_planner_node:main',
        ],
    },
)
