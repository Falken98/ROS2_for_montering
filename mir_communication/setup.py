from setuptools import setup
import os
from glob import glob

package_name = 'mir_communication'
api_class = 'mir_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/mir_api.py']),
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
            'mir_node = mir_communication.mir_node:main',
            'send_service = mir_communication.send_service:main'
        ],
    },
)
