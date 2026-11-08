import os
from glob import glob

from setuptools import setup

package_name = 'd2_lidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laijinxuan',
    maintainer_email='laijinxuan@example.com',
    description='欢创 D2 雷达驱动与避障节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'd2_scan_node = d2_lidar.d2_scan_node:main',
            'd2_avoid_node = d2_lidar.d2_avoid_node:main',
            'lidar_yaw_calib = d2_lidar.lidar_yaw_calib:main',
        ],
    },
)
