from setuptools import find_packages, setup

package_name = 'jetarm_grasp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/grasp.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laijinxuan',
    maintainer_email='laijinxuan@example.com',
    description='视觉抓取节点：拍照位识别 → 网格下探 → 合爪 → 抬升验证 → 运输位',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_node = jetarm_grasp.grasp_node:main',
        ],
    },
)