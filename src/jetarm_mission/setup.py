from setuptools import find_packages, setup

package_name = 'jetarm_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mission.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laijinxuan',
    maintainer_email='laijinxuan@example.com',
    description='取物任务编排：选点 → 导航 → 视觉抓取 → 返航 → 播报',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = jetarm_mission.mission_node:main',
        ],
    },
)