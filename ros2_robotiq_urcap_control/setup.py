from setuptools import setup
import os
from glob import glob

package_name = 'ros2_robotiq_urcap_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prosjekt',
    maintainer_email='668128@stud.hvl.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotiq_urcap_ctrl_ros2 = ros2_robotiq_urcap_control.robotiq_urcap_ctrl_ros2:main',
            'open_close_node = ros2_robotiq_urcap_control.open_close_node:main',
        ],
    },
)
