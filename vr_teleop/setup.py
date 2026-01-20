from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vr_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='VR Teleoperation for Franka FR3 using PyOpenVR and ALVR',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_tracker_node = vr_teleop.vr_ros2_node:main',
            'vr_converter_node = vr_teleop.franka_teleop_node:main',
            'vr_debug = vr_teleop.vr_controller_reader:main',
        ],
    },
)
