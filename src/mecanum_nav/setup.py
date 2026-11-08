import os
from glob import glob

from setuptools import setup

package_name = 'mecanum_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laijinxuan',
    maintainer_email='laijinxuan@example.com',
    description='麦轮车建图与导航启动配置',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_web_view = mecanum_nav.map_web_view:main',
            'laser_yaw_check = mecanum_nav.laser_yaw_check:main',
            'car_outline = mecanum_nav.car_outline:main',
            'pose_restorer = mecanum_nav.pose_restorer:main',
            'mode_manager = mecanum_nav.mode_manager:main',
        ],
    },
)
