from setuptools import find_packages, setup

package_name = 'ur5e_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin Repvik Olsbø',
    maintainer_email='martinolsbo1@gmail.com',
    description='Minimal ur5e publisher and subscriber package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_ur5e_publisher = ur5e_comunication.simple_ur5e_publisher:main',
            'simple_ur5e_subscriber = ur5e_comunication.simple_ur5e_subscriber:main',
            'ur5e_trajectory_controller = ur5e_comunication.ur5e_trajectory_controller:main',
        ],
    },
)
