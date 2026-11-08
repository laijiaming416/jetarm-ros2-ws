from setuptools import find_packages, setup

package_name = 'jetarm_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
         ['config/robot.json', 'config/actions.json', 'config/arm_poses.yaml']),
        ('share/' + package_name + '/launch', ['launch/arm.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laijinxuan',
    maintainer_email='laijinxuan@example.com',
    description='JetArm 机械臂驱动节点：串口独占、命名姿态、动作组与夹爪保持',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_node = jetarm_arm.arm_node:main',
        ],
    },
)