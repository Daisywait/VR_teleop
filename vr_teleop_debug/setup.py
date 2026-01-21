from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vr_teleop_debug'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Debug and monitoring tools for VR teleop',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_debug = vr_teleop_debug.vr_controller_reader:main',
            'vr_relative_debug = vr_teleop_debug.vr_relative_controller_reader:main',
            'vr_monitor_node = vr_teleop_debug.vr_monitor_node:main',
        ],
    },
)
