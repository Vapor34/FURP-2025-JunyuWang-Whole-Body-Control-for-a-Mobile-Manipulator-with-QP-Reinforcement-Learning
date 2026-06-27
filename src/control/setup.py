from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 注册包元数据，让 ros2 cli 工具（如 ros2 run）能找到这个包
        ('share/ament_index/resource_index/packages', ['resource/' + package_name + '/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 【关键】这一行确保 launch 文件夹中的文件能被系统找到
        # 【核心点】寻找到 launch 文件夹下的所有 .py 文件，并安装到 share/ 包名 /launch 目录下
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Junyu Wang',
    maintainer_email='scyjw16@nottingham.edu.cn',
    description='Minimal Python ROS2 Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner = control.planner:main',
        ],
    },
)
