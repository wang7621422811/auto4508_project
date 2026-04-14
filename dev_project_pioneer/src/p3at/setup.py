from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'p3at'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][y]'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.dae')) + glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'meshes', 'p3at_meshes'), glob(os.path.join('meshes', 'p3at_meshes', '*.dae')) + glob(os.path.join('meshes', 'p3at_meshes', '*.stl'))),
        (os.path.join('share', package_name, 'robots'), glob(os.path.join('robots', '*.urdf'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='parallels@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_controller = p3at.waypoint_controller:main',
            'path_recorder = p3at.path_recorder:main',
        ],
    },
)
