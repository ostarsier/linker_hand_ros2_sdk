'''
Author: HJX
Date: 2025-04-01 13:39:24
LastEditors: Please set LastEditors
LastEditTime: 2025-04-02 15:00:24
FilePath: /linker_hand_ros2_sdk/src/linker_hand_ros2_sdk/setup.py
Description: 
symbol_custom_string_obkorol_copyright: 
'''
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'linker_hand_ros2_sdk'

custom_dir = "../src/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand"
target_dir = os.path.join("share", package_name, "LinkerHand")
setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (
            target_dir, 
            glob(os.path.join(custom_dir, "**/*"), recursive=True) 
        )
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linker-robot',
    maintainer_email='linker-robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'linker_hand_sdk = linker_hand_ros2_sdk.linker_hand_ros2_sdk:main',
        ],
    },
)
