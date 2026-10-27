from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'uav_vio_px4_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phuc',
    maintainer_email='huynhphuc1808@gmail.com',
    description='ROS2 bridges for PX4 GPS-denied external vision simulation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'external_bridge = uav_vio_px4_bridge.external_bridge:main',
            (
                'vio_estimator_odometry_adapter = '
                'uav_vio_px4_bridge.vio_estimator_odometry_adapter:main'
            ),
            (
                'odom_to_px4_visual_odometry = '
                'uav_vio_px4_bridge.'
                'ros_odometry_to_px4_visual_odometry:main'
            ),
        ],
    },
)
