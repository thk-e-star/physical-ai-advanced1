from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pai_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='todo@todo.com',
    description='Pick and Place Package with ROS 2 and YOLO',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = pai_pick_place.yolo_detector_node:main',
            'pick_and_place = pai_pick_place.pick_and_place_controller:main',
        ],
    },
    )

