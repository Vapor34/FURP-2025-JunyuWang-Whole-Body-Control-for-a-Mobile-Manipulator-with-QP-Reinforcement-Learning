from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'description'

def package_files(directory):
    data_files = []
    for path, _, filenames in os.walk(directory):
        files = [os.path.join(path, filename) for filename in filenames]
        if files:
            install_path = os.path.join('share', package_name, path)
            data_files.append((install_path, files))
    return data_files

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name + '/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 【关键】这一行确保 launch 文件夹中的文件能被系统找到
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ] + package_files('assets'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Junyu Wang',
    maintainer_email='scyjw16@nottingham.edu.cn',
    description='Minimal Python ROS2 Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
