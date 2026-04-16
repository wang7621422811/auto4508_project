from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pioneer_part2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][y]'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.sdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='parallels@todo.todo',
    description='Pioneer P3-AT Part 2 mission package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_controller = pioneer_part2.mission_controller:main',
            'waypoint_controller = pioneer_part2.waypoint_controller:main',
            'cone_weaver = pioneer_part2.cone_weaver:main',
            'vision_detector = pioneer_part2.vision_detector:main',
            'gamepad_controller = pioneer_part2.gamepad_controller:main',
            'path_recorder = pioneer_part2.path_recorder:main',
            'gps_converter = pioneer_part2.gps_converter:main',
        ],
    },
)
