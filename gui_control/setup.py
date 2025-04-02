'''
Author: HJX
Date: 2025-04-01 17:49:50
LastEditors: Please set LastEditors
LastEditTime: 2025-04-01 18:19:15
FilePath: /linker_hand_ros2_sdk/src/gui_control/setup.py
Description: 
symbol_custom_string_obkorol_copyright: 
'''


from setuptools import find_packages, setup

package_name = 'gui_control'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'linker_hand_ros2_sdk'],
    zip_safe=True,
    maintainer='linker-robot',
    maintainer_email='linker-robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_control = gui_control.gui_control:main'
        ],
    },
)
