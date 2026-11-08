import os
from glob import glob

from setuptools import setup

package_name = 'mecanum_teleop'

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
    description='麦轮底盘手柄遥控节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_teleop = mecanum_teleop.joy_teleop:main',
            'cmd_mux = mecanum_teleop.cmd_mux:main',
        ],
    },
)
