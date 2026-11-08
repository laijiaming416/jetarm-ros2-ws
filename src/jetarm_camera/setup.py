from setuptools import find_packages, setup

package_name = 'jetarm_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/handeye.json']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laijinxuan',
    maintainer_email='laijinxuan@example.com',
    description='腕部相机节点：/dev/video20 MJPG 直通到 /camera/image_raw/compressed',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = jetarm_camera.camera_node:main',
        ],
    },
)