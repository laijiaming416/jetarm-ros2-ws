from setuptools import find_packages, setup

package_name = 'jetarm_dialog'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dialog.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laijinxuan',
    maintainer_email='laijinxuan@example.com',
    description='对话节点：唤醒词门控 + 任务关键词 → /voice/intent',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dialog_node = jetarm_dialog.dialog_node:main',
        ],
    },
)