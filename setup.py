from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nour',
    maintainer_email='nour@example.com',
    description='Vision and high-level control package for librarian robot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Real camera + YOLO + browser stream node
            'vision_picam = vision_pkg.vision_picam:main',

            # GUI TCP bridge between Qt GUI laptop and Raspberry Pi ROS2
            'gui_tcp_server_node = vision_pkg.gui_tcp_server_node:main',

            # HW-139 touch button on Raspberry Pi GPIO22
            'hw139_button_node = vision_pkg.hw139_button_node:main',

            # Final GUI-driven high-level controller
            'librarian_high_level_controller = vision_pkg.librarian_high_level_controller:main',

        ],
    },
)