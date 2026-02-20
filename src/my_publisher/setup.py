from setuptools import setup
import os
from glob import glob

package_name = 'my_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name,
            ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),

        # Gazebo world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        # rviz =2
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Gazebo simulation package',
    license='Apache License 2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'publisher1 = my_publisher.publisher1_node:main',
            'subscriber1 = my_publisher.subscriber1_node:main',
            'publisher2 = my_publisher.publisher2_node:main',
            'subscriber2 = my_publisher.subscriber2_node:main',
            'teleop = my_publisher.teleop:main',
        ],
    },
)

