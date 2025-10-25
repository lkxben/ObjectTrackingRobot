from setuptools import setup
import os
from glob import glob

package_name = 'hand_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ben',
    author_email='lkxben@gmail.com',
    description='Hand tracking robot ROS 2 package',
    entry_points={
        'console_scripts': [
            'hand_pose_node = hand_robot.nodes.hand_pose_node:main',
            'camera_node = hand_robot.nodes.camera_node:main',
            'fake_camera_node = hand_robot.nodes.fake_camera_node:main',
        ],
    }, data_files=[
        ('share/ament_index/resource_index/packages',
            ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('hand_robot/launch/*.py')),
    ],
)